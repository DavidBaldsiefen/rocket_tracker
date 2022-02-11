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
static int32_t numEngineBindings = 5;
static uint64_t input_size = 1 * 3 * 640 * 640; // default input size for YOLOv5
static uint64_t output_size = 1 * 25200 * 85;   // default output size for YOLOv5

static int num_classes = 80; // COCO class count
static int model_width = 640;
static int model_height = 640;
static int model_size = 640 * 640;

static bool TIME_LOGGING = false;
static bool TRACE_LOGGING = false;
static bool PERF_TEST = false;

void postprocessTRTdetections(float *model_output, rocket_tracker::detectionMSG *detection) {

    // inspired by https://github.com/ultralytics/yolov5/issues/708#issuecomment-674422178
    unsigned long dimensions =
        5 + num_classes; // 0,1,2,3 ->box,4->confidence，5-85 -> coco classes confidence
    const unsigned long confidenceIndex = 4;
    const unsigned long labelStartIndex = 5;

    // Trace-logging for debugging purposes
    if (TRACE_LOGGING) {
        std::string outputstring = "";
        for (int i = 0; i < 6; i++) {
            outputstring += std::to_string(model_output[i]) + " ";
            if (i == 3 || i == 4) {
                outputstring += "| ";
            }
        }
        ROS_INFO("%s", outputstring.c_str());
    }

    int highest_conf_index = 0;
    int highest_conf_label = 0;
    float highest_conf = 0.4f; // confidence threshold is 40%
    for (int index = 0; index < output_size; index += dimensions) {
        float confidence = model_output[index + confidenceIndex];

        // for multiple classes, combine the confidence with class confidences
        // for single class models, this step can be skipped
        if (num_classes > 1) {
            if (confidence <= highest_conf) {
                continue;
            }
            for (unsigned long j = labelStartIndex; j < dimensions; ++j) {
                float combined_conf = model_output[index + j] * confidence;
                if (combined_conf > highest_conf) {
                    highest_conf = combined_conf;
                    highest_conf_index = index;
                    highest_conf_label = j - labelStartIndex;
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
        detection->centerX = model_output[highest_conf_index];
        detection->centerY = model_output[highest_conf_index + 1];
        detection->width = model_output[highest_conf_index + 2];
        detection->height = model_output[highest_conf_index + 3];
    }
}

void processImage(double *cudaTime, double *pstTime, rocket_tracker::detectionMSG *detection) {

    unsigned long time0 = ros::Time::now().toNSec();

    // Invoke asynchronous inference
    context->enqueueV2(buffers, 0, nullptr);

    float *output_buffer_pointer = static_cast<float *>(buffers[outputIndex]);

    // wait for inference to finish
    cudaStreamSynchronize(0);

    unsigned long time1 = ros::Time::now().toNSec();

    // perform postprocessing & identify most confident detection
    postprocessTRTdetections(output_buffer_pointer, detection);

    *pstTime = (ros::Time::now().toNSec() - time1) / 1000000.0;
    *cudaTime = (time1 - time0) / 1000000.0;
}

void handleNewFrame(unsigned long frameID, unsigned long frameStamp, unsigned long preTimeFG,
                    int droppedFrames) {
    rocket_tracker::detectionMSG detection;
    double cudaTime, pstTime;

    // do the actual image processing
    processImage(&cudaTime, &pstTime, &detection);
    detection.timestamp = ros::Time::now();
    detection.processingTime = (detection.timestamp.toNSec() - frameStamp) / 1000000.0;
    detection.frameID = frameID;

    // publish the detection
    detectionPublisher.publish(detection);

    // Time logging statistics
    if (TIME_LOGGING)
        ROS_INFO("Total detection time: %.2f [PRE: %.2lf CUDA: %.2f PST: %.2f]",
                 detection.processingTime, preTimeFG / 1000000.0, cudaTime, pstTime);

    // Performance test statistics
    if (PERF_TEST) {
        static int iterationcounter = 0;
        static int totalDroppedFrames = 0;
        static double avgLatency = 0, avg_pre = 0, avg_cuda = 0, avg_pst = 0;
        static ros::Time throughputTimer = ros::Time::now();
        avgLatency += detection.processingTime;
        avg_pre += preTimeFG / 1000000.0;
        avg_cuda += cudaTime;
        avg_pst += pstTime;
        totalDroppedFrames += droppedFrames;
        iterationcounter++;
        if (iterationcounter >= 1000) {
            avgLatency /= 1000.0;
            avg_pre /= 1000.0;
            avg_cuda /= 1000.0;
            avg_pst /= 1000.0;
            double throughput =
                1000.0 * 1000.0 /
                ((ros::Time::now().toNSec() - throughputTimer.toNSec()) / 1000000.0);

            ROS_INFO("Results of performance measurement after 1000 frames:\nAVG Latency: "
                     "%.2fms [PRE: %.2f CUDA: %.2f PST: %.2f] Throughput: %.1fFPS Dropped "
                     "Frames: %d",
                     avgLatency, avg_pre, avg_cuda, avg_pst, throughput, totalDroppedFrames);

            avgLatency = 0;
            iterationcounter = 0;
            avg_pre = 0.0;
            avg_cuda = 0.0;
            avg_pst = 0.0;
            totalDroppedFrames = 0;
            throughputTimer = ros::Time::now();
        }
    }
}

void inferRandomMats(int iterations) {
    // Infers random matrices over n iterations
    if (iterations <= 0)
        return;

    rocket_tracker::detectionMSG detection;

    double cudaTime = 0.0, pstTime = 0.0, totalCuda = 0.0, totalPst = 0.0;

    for (int i = 0; i < iterations; i++) {
        // create random input Array

        std::srand(unsigned(std::time(nullptr)));
        std::vector<float> inputArray;
        inputArray.resize(1 * 3 * model_width * model_height);
        generate(inputArray.begin(), inputArray.end(), std::rand);
        inputArray.resize(1 * 3 * model_width * model_height);
        float *pFloat = static_cast<float *>(buffers[inputIndex]);
        std::copy(&inputArray[0], &inputArray[0] + input_size, pFloat);

        processImage(&cudaTime, &pstTime, &detection);
        totalCuda += cudaTime;
        totalPst += pstTime;
    }

    // Evaluate timings
    double total = totalCuda + totalPst;
    double avgtotal = total / iterations;
    double avgCuda = totalCuda / iterations;
    double avgPst = totalPst / iterations;
    ROS_INFO("Performing inference for %d iterations took %.2lf ms. (Avg: %.2lf [CUDA: %.2lf PST: "
             "%.2lf])",
             iterations, total, avgtotal, avgCuda, avgPst);
}

void callbackFrameGrabber(const sensor_msgs::ImageConstPtr &msg) {
    static unsigned long lastFrameID = 0;
    unsigned long frameID = msg->header.seq;
    // check for dropped frames
    int droppedFrames = 0;
    if (frameID - lastFrameID > 1) {
        droppedFrames = frameID - lastFrameID;
        if (!PERF_TEST)
            ROS_WARN("Frame dropped from FG->IP: jumped from index %lu to %lu", lastFrameID,
                     frameID);
    }
    lastFrameID = frameID;

    ros::Time time0 = ros::Time::now();

    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    // only resize down
    if (img.rows > model_height || img.cols > model_width) {
        cv::resize(img, img, cv::Size(model_width, model_height));
    }

    // prepare input buffers
    float *pFloat = static_cast<float *>(buffers[inputIndex]);
    // forEach is significantly faster than all other methods to traverse over the cv::Mat
    img.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int *position) -> void {
        // p[0-2] contains bgr data, position[0-1] the row-column location
        // Incoming data is BGR, so convert to RGB in the process
        int index = model_height * position[0] + position[1];
        pFloat[index] = p[2] / 255.0f;
        pFloat[model_size + index] = p[1] / 255.0f;
        pFloat[2 * model_size + index] = p[0] / 255.0f;
    });

    handleNewFrame(frameID, msg->header.stamp.toNSec(), ros::Time::now().toNSec() - time0.toNSec(),
                   droppedFrames);
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

bool initializeTRT(std::string enginePath) {
    ROS_INFO("Initializing TRT");
    long size;
    char *trtModelStream;
    std::ifstream file(enginePath, std::ios::binary);
    ROS_INFO("Loading engine from %s", enginePath.c_str());
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    } else {
        ROS_ERROR("Could not load engine file.");
        return false;
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
    numEngineBindings = engine->getNbBindings();
    ROS_INFO("Reading engine bindings:");
    buffers = new void *[numEngineBindings];
    for (int i = 0; i < numEngineBindings; i++) {

        std::string dimension_desc = " [";
        size_t size = 1;

        // Multiply every dimension of each binding to get its total size
        nvinfer1::Dims dims = engine->getBindingDimensions(i);
        for (size_t j = 0; j < dims.nbDims; ++j) {
            size *= dims.d[j];
            dimension_desc += std::to_string(dims.d[j]) + " ";
        }

        auto binding_size = size * sizeof(float);
        if (cudaMallocHost(&buffers[i], binding_size) != cudaSuccess) {
            ROS_WARN("Failed to allocate pinned memory! Switching to pageable memory instead.");
            if (cudaMalloc(&buffers[i], binding_size) != cudaSuccess) {
                ROS_ERROR("Could not allocate cuda memory.");
                return false;
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
            input_size = size;
        }

        ROS_INFO("%s", dimension_desc.c_str());
    }

    if (engine->getBindingDataType(inputIndex) != nvinfer1::DataType::kFLOAT ||
        engine->getBindingDataType(outputIndex) != nvinfer1::DataType::kFLOAT) {
        ROS_WARN("Engine input and/or output datatype is not float. Please change to a different "
                 "engine");
        return false;
    }

    ROS_INFO("Loaded model with %d classes and input size %dx%d", num_classes, model_width,
             model_height);
    ros::param::set("/rocket_tracker/model_width", model_width);
    ros::param::set("/rocket_tracker/model_height", model_height);
    model_size = model_width * model_height;
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "IMAGEPROCESSOR");
    ros::NodeHandle nh("~");

    // Get weightfile path from arguments
    std::string enginePath;
    if (argc == 2) {
        enginePath = argv[1];
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
        ROS_INFO("Performance test enabled. Frame-drop warnings are suppressed.");
    }

    // TensorRT
    if (!initializeTRT(enginePath)) {
        ROS_ERROR("Failed to initialize TensorRT. Shutting down image processor");
        return 0;
    }
    ROS_INFO("TRT initialized");
    ros::param::set("/rocket_tracker/trt_ready", true);
    ROS_INFO("Warming up over 100 iterations with random mats");
    inferRandomMats(100);

    // Creating image-transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subimg = it.subscribe("/image_topic", 1, &callbackFrameGrabber);

    // Create ros publisher
    detectionPublisher = nh.advertise<rocket_tracker::detectionMSG>("/detection", 1);

    // Main loop
    ros::spin();

    // Shut everything down cleanly
    for (int i = 0; i < numEngineBindings; i++) {
        cudaFreeHost(buffers[i]);
    }
    ros::shutdown();
    detectionPublisher.shutdown();
    nh.shutdown();
    return 0;
}
