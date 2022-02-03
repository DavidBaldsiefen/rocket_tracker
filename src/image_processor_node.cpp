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

static void *buffers[5];
nvinfer1::IExecutionContext *context;
static int32_t inputIndex = 0;
static int32_t outputIndex = 4;

static int num_classes = 80; // COCO class count
static int model_width = 640;
static int model_height = 640;

static std::string time_logging_string = "";
static bool TIME_LOGGING = false;

template <typename... Args> std::string string_format(const std::string &format, Args... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    if (size_s <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    auto buf = std::make_unique<char[]>(size);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

void preprocessImgTRT(cv::Mat img, void *inputBuffer) {
    // thanks to https://zhuanlan.zhihu.com/p/344810135
    if (img.empty()) {
        ROS_WARN("Empty image received!");
    }

    uint64_t time = ros::Time::now().toNSec();

    cv::resize(img, img, cv::Size(model_width, model_height));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    uint64_t time2 = ros::Time::now().toNSec();

    static int model_size = model_width * model_height;
    static float *inputArray = new float[1 * 3 * 640 * 640];

    // for each is significantly faster than all other methods to traverse over the cv::Mat (read
    // online and confirmed myself)
    img.forEach<cv::Vec3b>([](cv::Vec3b &p, const int *position) -> void {
        // p[0-2] contains rgb data, position[0-1] the xy location
        int index = model_height * position[0] + position[1];
        inputArray[index] = p[2] / 255.0f;
        inputArray[model_size + index] = p[1] / 255.0f;
        inputArray[2 * model_size + index] = p[0] / 255.0f;
    });

    uint64_t time3 = ros::Time::now().toNSec();

    static size_t input_size = 1 * 3 * model_width * model_height * sizeof(float);
    cudaMemcpy(inputBuffer, inputArray, input_size, cudaMemcpyHostToDevice);

    uint64_t time4 = ros::Time::now().toNSec();

    if (TIME_LOGGING)
        time_logging_string +=
            string_format("[PRE %.2lf %.2lf %.2lf]", (time2 - time) / 1000000.0,
                          (time3 - time2) / 1000000.0, (time4 - time3) / 1000000.0);
}

void postprocessTRTdetections(void *outputBuffer, rocket_tracker::detectionMSG *detection) {

    // inspired by https://github.com/ultralytics/yolov5/issues/708#issuecomment-674422178

    uint64_t time = ros::Time::now().toNSec();

    std::vector<float> cpu_output(1 * 25200 * (5 + num_classes));

    cudaMemcpy(cpu_output.data(), outputBuffer, cpu_output.size() * sizeof(float),
               cudaMemcpyDeviceToHost); // this can be fastened by keeping stuff on gpu

    uint64_t time2 = ros::Time::now().toNSec();

    unsigned long dimensions =
        5 + num_classes; // 0,1,2,3 ->box,4->confidence，5-85 -> coco classes confidence
    unsigned long rows = cpu_output.size() / dimensions; // 25.200
    unsigned long confidenceIndex = 4;
    unsigned long labelStartIndex = 5;

    int highest_conf_index = 0;
    int highest_conf_label = 0;
    float highest_conf = 0.0f;
    for (unsigned long i = 0; i < rows; ++i) {
        unsigned long index = i * dimensions;

        if (cpu_output[index + confidenceIndex] <= 0.4f) {
            continue;
        }

        for (unsigned long j = labelStartIndex; j < dimensions; ++j) {
            float combined_conf = cpu_output[index + j] * cpu_output[index + confidenceIndex];
            if (combined_conf > highest_conf) {
                highest_conf = combined_conf;
                highest_conf_index = index;
                highest_conf_label = j;
            }
        }
    }

    // Evaluate results
    if (highest_conf > 0.4f) {

        detection->propability = cpu_output[highest_conf_index + confidenceIndex];
        detection->classID = cpu_output[highest_conf_index + labelStartIndex + highest_conf_label];
        detection->centerX = cpu_output[highest_conf_index];
        detection->centerY = cpu_output[highest_conf_index + 1];
        detection->width = cpu_output[highest_conf_index + 2];
        detection->height = cpu_output[highest_conf_index + 3];
    }

    uint64_t time3 = ros::Time::now().toNSec();

    if (TIME_LOGGING)
        time_logging_string += string_format("[PST %.2lf %.2lf]", (time2 - time) / 1000000.0,
                                             (time3 - time2) / 1000000.0);
}

rocket_tracker::detectionMSG processImage(cv::Mat img) {

    uint64_t time0 = ros::Time::now().toNSec();

    rocket_tracker::detectionMSG result;
    result.centerX = 0.0;
    result.centerY = 0.0;
    result.width = 0.0;
    result.height = 0.0;
    result.classID = 0;
    result.propability = 0.0;
    result.frameID = 0;

    time_logging_string = "";
    uint64_t time = ros::Time::now().toNSec();

    // Prepare input tensor
    preprocessImgTRT(img, buffers[inputIndex]);

    uint64_t time2 = ros::Time::now().toNSec();

    context->executeV2(buffers); // Invoke synchronous inference

    uint64_t time3 = ros::Time::now().toNSec();

    postprocessTRTdetections(buffers[outputIndex], &result);

    uint64_t time4 = ros::Time::now().toNSec();
    if (TIME_LOGGING) {
        ROS_INFO("%s", time_logging_string.c_str());
        time_logging_string = string_format(
            "PRE: %.2lf FWD: %.2lf PST: %.2lf TOTAL: %.2lf", (time2 - time) / 1000000.0,
            (time3 - time2) / 1000000.0, (time4 - time3) / 1000000.0, (time4 - time0) / 1000000.0);

        ROS_INFO("%s", time_logging_string.c_str());
    }
    return result;
}

void callbackFrameGrabber(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // check for missed frames
    static uint last_frame_id = msg->header.seq;
    if (msg->header.seq - last_frame_id > 1) {
        ROS_WARN("Frame dropped from FG->IP: jumped from index %u to %u", last_frame_id,
                 msg->header.seq);
    }
    last_frame_id = msg->header.seq;

    if (!img->image.empty()) {
        // TODO: sync frame_ids to detected coordinates
        uint64_t time = ros::Time::now().toNSec();

        rocket_tracker::detectionMSG detection;
        detection = processImage(img->image);

        detection.processingTime = (ros::Time::now().toNSec() - time) / 1000000.0;
        detection.timestamp = time / 1000000.0;
        // total time between framecapture and detection being published:
        double detectionTime = (ros::Time::now().toNSec() - msg->header.stamp.toNSec()) / 1000000.0;
        if (TIME_LOGGING)
            ROS_INFO("Total detection time: %.2lf", detectionTime);

        detectionPublisher.publish(detection);
    } else {
        ROS_WARN("Empty Frame received in image_processor_node::callbackFrameGrabber");
    }
}

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char *msg) noexcept override {
        // suppress info-level messages
        if (severity <= Severity::kWARNING) {
            ROS_INFO("[NvInfer] %s", msg);
        }
    }
} logger;

int main(int argc, char **argv) {

    ros::init(argc, argv, "IMAGEPROCESSOR");
    ros::NodeHandle nh("~");

    // Get weightfile path from arguments
    std::string weightfilepath;
    bool usecuda = true;
    if (argc == 2) {
        weightfilepath = argv[1];
    } else if (argc == 3) {
        weightfilepath = argv[1];
        std::string arg2(argv[2]);
        usecuda = !(arg2 == "false" || arg2 == "False" || arg2 == "0");
    } else {
        ROS_ERROR("No weightfile argument passed.");
        ros::shutdown();
        return 0;
    }

    ros::param::get("/rocket_tracker/time_logging", TIME_LOGGING);

    // TensorRT
    ROS_INFO("Initializing TRT");
    long size;
    char *trtModelStream;
    std::ifstream file(weightfilepath, std::ios::binary);
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
        ROS_INFO("Loaded engine from %s", weightfilepath.c_str());
    }

    nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(logger);
    assert(runtime != nullptr);
    nvinfer1::ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);

    assert(engine != nullptr);
    context = engine->createExecutionContext();

    assert(context != nullptr);
    delete[] trtModelStream;

    // Allocate memory for every engine binding
    inputIndex = engine->getBindingIndex("images");
    outputIndex = engine->getBindingIndex("output");
    ROS_INFO("Reading engine bindings:");
    for (int i = 0; i < 5; i++) {

        std::string dimension_desc = " [";
        size_t size = 1;

        // Multiply every dimension of each binding to get its total size
        nvinfer1::Dims dims = engine->getBindingDimensions(i);
        for (size_t j = 0; j < dims.nbDims; ++j) {
            size *= dims.d[j];
            dimension_desc += std::to_string(dims.d[j]) + " ";
        }

        auto binding_size = size * 1 * sizeof(float);
        cudaMalloc(&buffers[i], binding_size);

        dimension_desc.pop_back();
        dimension_desc += "] (\"" + std::string(engine->getBindingName(i)) + "\")";
        dimension_desc += " Datatype: " + std::to_string((int32_t)engine->getBindingDataType(i));

        if (i == outputIndex) {
            num_classes = dims.d[dims.nbDims - 1] - 5;
        } else if (i == inputIndex) {
            model_width = dims.d[dims.nbDims - 2];
            model_height = dims.d[dims.nbDims - 1];
        }

        ROS_INFO("%s", dimension_desc.c_str());
    }

    ROS_INFO("Loaded model with %d classes and input size %dx%d", num_classes, model_width,
             model_height);
    ros::param::set("/rocket_tracker/model_width", model_width);
    ros::param::set("/rocket_tracker/model_height", model_height);
    ros::param::set("/rocket_tracker/trt_ready", true);

    nvinfer1::DataType modelDatatype = engine->getBindingDataType(inputIndex);
    ros::param::set("/rocket_tracker/model_datatype",
                    modelDatatype == nvinfer1::DataType::kHALF ? "FP16" : "FP32");

    ROS_INFO("TRT initialized");

    // Creating image-transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subimg = it.subscribe("/image_topic", 1, &callbackFrameGrabber);

    // Create ros publisher
    detectionPublisher = nh.advertise<rocket_tracker::detectionMSG>("/detection", 1);

    // Main loop
    ros::spin();

    // Shut everything down cleanly
    for (void *buf : buffers) {
        cudaFree(buf);
    }
    ros::shutdown();
    subimg.shutdown();
    detectionPublisher.shutdown();
    nh.shutdown();
    return 0;
}
