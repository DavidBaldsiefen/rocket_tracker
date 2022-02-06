#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <cuda_runtime.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <rocket_tracker/detectionMSG.h> // Autogenerated by ROS
#include <ros/package.h>
#include <ros/ros.h>

#include "NvInfer.h"

static ros::Publisher detectionPublisher;

static void **buffers;
static nvinfer1::IExecutionContext *context;
static int32_t inputIndex = 0;
static int32_t outputIndex = 4;
static int64_t output_size = 1 * 25200 * 85; // default output size for YOLOv5

static int num_classes = 80; // COCO class count
static int model_width = 640;
static int model_height = 640;

static std::string time_logging_string = "";
static bool TIME_LOGGING = false;
static bool TRACE_LOGGING = false;
static bool PERF_TEST = false;

// Define an STL compatible allocator of floats that allocates from the managed_shared_memory.
// This allocator will allow placing containers in the segment
typedef boost::interprocess::allocator<float,
                                       boost::interprocess::managed_shared_memory::segment_manager>
    ShmemAllocatorFloat;
typedef boost::interprocess::allocator<unsigned long,
                                       boost::interprocess::managed_shared_memory::segment_manager>
    ShmemAllocatorLong;

// Alias a vector that uses the previous STL-like allocator so that allocates
// its values from the segment
typedef boost::interprocess::vector<float, ShmemAllocatorFloat> FloatVector;
typedef boost::interprocess::vector<unsigned long, ShmemAllocatorLong> LongVector;

boost::interprocess::managed_shared_memory segment;

template <typename... Args> std::string string_format(const std::string &format, Args... args) {
    // from https://stackoverflow.com/a/26221725
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    if (size_s <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    auto buf = std::make_unique<char[]>(size);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

void preprocessImgTRT(FloatVector *image, void *inputBuffer) {
    float *inputArray = &(*image)[0];
    static size_t input_size = 1 * 3 * model_width * model_height * sizeof(float);
    cudaMemcpy(inputBuffer, inputArray, input_size, cudaMemcpyHostToDevice);
}

void postprocessTRTdetections(void *outputBuffer, rocket_tracker::detectionMSG *detection) {

    // inspired by https://github.com/ultralytics/yolov5/issues/708#issuecomment-674422178

    uint64_t time = ros::Time::now().toNSec();

    static size_t output_buffer_size = output_size * sizeof(float);
    std::vector<float> cpu_output(output_size);

    cudaMemcpy(cpu_output.data(), outputBuffer, output_buffer_size, cudaMemcpyDeviceToHost);

    uint64_t time2 = ros::Time::now().toNSec();

    unsigned long dimensions =
        5 + num_classes; // 0,1,2,3 ->box,4->confidence，5-85 -> coco classes confidence
    unsigned long numPredictions = cpu_output.size() / dimensions; // 25.200
    unsigned long confidenceIndex = 4;
    unsigned long labelStartIndex = 5;

    if (TRACE_LOGGING) {
        std::string outputstring = "";
        for (int i = 0; i < 6; i++) {
            outputstring += std::to_string(cpu_output[i]) + " ";
            if (i == 3 || i == 4) {
                outputstring += "| ";
            }
        }
        ROS_INFO("%s", outputstring.c_str());
    }

    int highest_conf_index = 0;
    int highest_conf_label = 0;
    float highest_conf = 0.4f;
    for (int index = 0; index < output_size; index += dimensions) {
        float confidence = cpu_output[index + confidenceIndex];

        // for multiple classes, combine the confidence with class confidences
        // for single class models, this step can be skipped
        if (num_classes > 1) {
            if (confidence <= highest_conf) {
                continue;
            }
            for (unsigned long j = labelStartIndex; j < dimensions; ++j) {
                float combined_conf = cpu_output[index + j] * confidence;
                if (combined_conf > highest_conf) {
                    highest_conf = combined_conf;
                    highest_conf_index = index;
                    highest_conf_label = j - 5;
                }
            }
        } else {
            if (confidence > highest_conf) {
                highest_conf = confidence;
                highest_conf_index = index;
                highest_conf_label = 1;
            }
        }
    }

    // Evaluate results
    detection->propability = highest_conf;
    if (highest_conf > 0.4f) {
        if (TRACE_LOGGING)
            ROS_INFO("Detected class %d with confidence %lf", highest_conf_label, highest_conf);
        detection->classID = highest_conf_label;
        detection->centerX = cpu_output[highest_conf_index];
        detection->centerY = cpu_output[highest_conf_index + 1];
        detection->width = cpu_output[highest_conf_index + 2];
        detection->height = cpu_output[highest_conf_index + 3];
    }

    uint64_t time3 = ros::Time::now().toNSec();

    if (TIME_LOGGING)
        time_logging_string +=
            string_format("[%.2lf %.2lf]", (time2 - time) / 1000000.0, (time3 - time2) / 1000000.0);
}

void processImage(FloatVector *image, double *preTime, double *fwdTime, double *pstTime,
                  rocket_tracker::detectionMSG *detection) {

    uint64_t time1 = ros::Time::now().toNSec();

    preprocessImgTRT(image, buffers[inputIndex]);

    uint64_t time2 = ros::Time::now().toNSec();

    context->executeV2(buffers); // Invoke synchronous inference

    uint64_t time3 = ros::Time::now().toNSec();

    postprocessTRTdetections(buffers[outputIndex], detection);

    uint64_t time4 = ros::Time::now().toNSec();

    *preTime = (time2 - time1) / 1000000.0;
    *fwdTime = (time3 - time2) / 1000000.0;
    *pstTime = (time4 - time3) / 1000000.0;
}

void inferRandomMats(int iterations) {
    // Infers random matrices over n iterations

    rocket_tracker::detectionMSG detection;

    uint64_t pre = 0, fwd = 0, pst = 0;

    for (int i = 0; i < iterations; i++) {
        // create random input Array

        std::srand(unsigned(std::time(nullptr)));
        std::vector<float> inputArray;
        inputArray.resize(1 * 3 * model_width * model_height);
        generate(inputArray.begin(), inputArray.end(), std::rand);

        uint64_t time1 = ros::Time::now().toNSec();
        // preprocessImgTRT(&inputArray, buffers[inputIndex]);
        uint64_t time2 = ros::Time::now().toNSec();
        context->executeV2(buffers); // Invoke synchronous inference
        uint64_t time3 = ros::Time::now().toNSec();
        postprocessTRTdetections(buffers[outputIndex], &detection);
        uint64_t time4 = ros::Time::now().toNSec();

        pre += time2 - time1;
        fwd += time3 - time2;
        pst += time4 - time3;
    }

    // Evaluate timings
    double total = (pre + fwd + pst) / 1000000.0;
    double avgtotal = (total / iterations);
    double avgpre = ((double)pre / iterations) / 1000000.0;
    double avgfwd = ((double)fwd / iterations) / 1000000.0;
    double avgpst = ((double)pst / iterations) / 1000000.0;
    ROS_INFO(
        "Performing inference for %d iterations took %.2lf ms. (Avg: %.2lf [%.2lf %.2lf %.2lf])",
        iterations, total, avgtotal, avgpre, avgfwd, avgpst);
}

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char *msg) noexcept override {

        if (severity == Severity::kINFO) {
            ROS_INFO("[TensorRT] %s", msg);
        } else if (severity == Severity::kWARNING) {
            ROS_WARN("[TensorRT] %s", msg);
        } else if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
            ROS_ERROR("[TensorRT] %s", msg);
        }
    }
} logger;

int main(int argc, char **argv) {

    ros::init(argc, argv, "IMAGEPROCESSOR");
    ros::NodeHandle nh("~");

    // Get weightfile path from arguments
    std::string weightfilepath;
    if (argc == 2) {
        weightfilepath = argv[1];
    } else {
        ROS_ERROR("Invalid number of argument passed to image processor.");
        ros::shutdown();
        return 0;
    }

    ros::param::get("/rocket_tracker/time_logging", TIME_LOGGING);
    ros::param::get("/rocket_tracker/trace_logging", TRACE_LOGGING);
    ros::param::get("/rocket_tracker/performance_test", PERF_TEST);
    if (PERF_TEST) {
        ros::param::set("rocket_tracker/fg_fps_target", 50);
    }

    // TensorRT
    ROS_INFO("Initializing TRT");
    long size;
    char *trtModelStream;
    std::ifstream file(weightfilepath, std::ios::binary);
    ROS_INFO("Loading engine from %s", weightfilepath.c_str());
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }

    // Create runtime, deserialize engine and create execution context
    nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(logger);
    assert(runtime != nullptr);
    nvinfer1::ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;

    // Allocate memory for every engine binding, and gather input & output information
    inputIndex = engine->getBindingIndex("images");
    outputIndex = engine->getBindingIndex("output");
    ROS_INFO("Reading engine bindings:");
    buffers = new void *[engine->getNbBindings()];
    for (int i = 0; i < engine->getNbBindings(); i++) {

        std::string dimension_desc = " [";
        size_t size = 1;

        // Multiply every dimension of each binding to get its total size
        nvinfer1::Dims dims = engine->getBindingDimensions(i);
        for (size_t j = 0; j < dims.nbDims; ++j) {
            size *= dims.d[j];
            dimension_desc += std::to_string(dims.d[j]) + " ";
        }

        auto binding_size = size * 1 * sizeof(float);
        if (cudaMallocHost(&buffers[i], binding_size) != cudaSuccess) {
            ROS_WARN("Failed to allocate pinned memory! Switching to pageable memory instead.");
            if (cudaMalloc(&buffers[i], binding_size) != cudaSuccess) {
                ROS_ERROR("Could not allocate cuda memory.");
                return 0;
            }
        }

        dimension_desc.pop_back();
        dimension_desc += "] (\"" + std::string(engine->getBindingName(i)) + "\")";
        dimension_desc += " Datatype: " + std::to_string((int32_t)engine->getBindingDataType(i));

        if (i == outputIndex) {
            num_classes = dims.d[dims.nbDims - 1] - 5;
            output_size = size;
        } else if (i == inputIndex) {
            model_width = dims.d[dims.nbDims - 2];
            model_height = dims.d[dims.nbDims - 1];
        }

        ROS_INFO("%s", dimension_desc.c_str());
    }

    if (engine->getBindingDataType(inputIndex) != nvinfer1::DataType::kFLOAT ||
        engine->getBindingDataType(outputIndex) != nvinfer1::DataType::kFLOAT) {
        ROS_WARN("Engine input and/or output datatype is not float. Please change to a different "
                 "engine");
    }

    ROS_INFO("Loaded model with %d classes and input size %dx%d", num_classes, model_width,
             model_height);
    ros::param::set("/rocket_tracker/model_width", model_width);
    ros::param::set("/rocket_tracker/model_height", model_height);
    ros::param::set("/rocket_tracker/trt_ready", true);

    ROS_INFO("TRT initialized");
    ROS_INFO("Warming up over 100 iterations with random mats");
    inferRandomMats(0);

    // Create ros publisher
    detectionPublisher = nh.advertise<rocket_tracker::detectionMSG>("/detection", 1);

    segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only,
                                                         "MySharedMemory");
    FloatVector *img_vector = segment.find<FloatVector>("img_vector").first;
    LongVector *notification_vector = segment.find<LongVector>("notification_vector").first;

    // All inference variables
    unsigned long lastFrameID = 0; // the first frame will be skipped, which is intentional
    unsigned long frameID, frameStamp, preTimeFG;
    double preTime, fwdTime, pstTime;
    bool droppedFrame = false;
    rocket_tracker::detectionMSG detection;

    // Main loop. There are no subscribers, so spinning is not required
    while (ros::ok()) {
        // check memory for new data
        if (notification_vector->at(0) != lastFrameID) {
            // process new image
            frameID = notification_vector->at(0);
            if (frameID - lastFrameID > 1 && frameID != 0) {
                droppedFrame = true;
                ROS_WARN("Frame dropped from FG->IP: jumped from index %lu to %lu", lastFrameID,
                         frameID);
            }
            lastFrameID = frameID;
            frameStamp = notification_vector->at(1);
            preTimeFG = notification_vector->at(2);
            processImage(img_vector, &preTime, &fwdTime, &pstTime, &detection);
            detection.timestamp = ros::Time::now();
            detection.processingTime = (detection.timestamp.toNSec() - frameStamp) / 1000000.0;
            detection.frameID = frameID;
            detectionPublisher.publish(detection);
            preTime += preTimeFG / 1000000.0;
            // Time logging statistics
            if (TIME_LOGGING)
                ROS_INFO("Total detection time: %.2f [PRE: %.2lf FWD: %.2f PST: %.2f] PRE: [%.2lf "
                         "%.2lf]",
                         detection.processingTime, preTime, fwdTime, pstTime, preTimeFG / 1000000.0,
                         preTime - (preTimeFG / 1000000.0));

            // FPS avg calculation
            if (PERF_TEST) {
                static int iterationcounter = 0;
                static int droppedFrames = 0;
                static double totalTime = 0, avg_fps = 0, avg_pre = 0, avg_fwd = 0, avg_pst = 0;
                totalTime += detection.processingTime;
                avg_pre += preTime;
                avg_fwd += fwdTime;
                avg_pst += pstTime;
                if (droppedFrame) {
                    droppedFrames++;
                }
                iterationcounter++;
                if (iterationcounter >= 1000) {
                    ROS_INFO("Totaltime: %.2f", totalTime);
                    avg_fps = 1000 / (totalTime / 1000.0);
                    avg_pre /= 1000.0;
                    avg_fwd /= 1000.0;
                    avg_pst /= 1000.0;

                    ROS_INFO(
                        "Results of performance measurement after 1000 frames:\nAVG FPS: %.1f AVG "
                        "PRE: %.2f AVG FWD: "
                        "%.2f AVG PST: %.2f Dropped Frames: %d",
                        avg_fps, avg_pre, avg_fwd, avg_pst, droppedFrames);

                    totalTime = 0;
                    iterationcounter = 0;
                    avg_pre = 0.0;
                    avg_fwd = 0.0;
                    avg_pst = 0.0;
                    droppedFrames = 0;
                }
            }
        }
    }

    // Shut everything down cleanly
    for (int i = 0; i < engine->getNbBindings(); i++) {
        cudaFreeHost(buffers[i]);
    }
    ros::shutdown();
    detectionPublisher.shutdown();
    nh.shutdown();
    return 0;
}
