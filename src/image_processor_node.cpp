#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <rocket_tracker/detectionMSG.h> // Autogenerated by ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <torch/script.h>
#include <torch/torch.h>

#include "NvInfer.h"

static ros::Publisher detectionPublisher;

static torch::jit::script::Module module;
static torch::Device torchDevice = torch::Device(torch::kCPU);

static void *buffers[5];
nvinfer1::IExecutionContext *context;
static int32_t inputIndex = 0;
static int32_t outputIndex = 5;

const bool TIME_LOGGING = true;

void preprocessImgTRT(cv::Mat img, void *inputBuffer) {
    if (img.empty()) {
        ROS_WARN("Empty image received!");
    }
    cv::resize(img, img, cv::Size(640, 640));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    static float inputArray[1 * 3 * 640 * 640];
    int i = 0;
    for (int row = 0; row < 640; ++row) {
        uchar *uc_pixel = img.data + row * img.step;
        for (int col = 0; col < 640; ++col) {
            inputArray[i] = (float)uc_pixel[2] / 255.0;
            inputArray[i + 640 * 640] = (float)uc_pixel[1] / 255.0;
            inputArray[i + 2 * 640 * 640] = (float)uc_pixel[0] / 255.0;
            uc_pixel += 3;
            ++i;
        }
    }
    cudaMemcpy(inputBuffer, inputArray, 1 * 3 * 640 * 640 * sizeof(float),
               cudaMemcpyHostToDevice); // maybe I should free this every time?
}

void postprocessTRTdetections(void *outputBuffer, rocket_tracker::detectionMSG *detection) {
    std::vector<float> cpu_output(
        1 * 25200 *
        85); // =2142000. Can also be obtained by using engine->getBindingDimensions(outputIndex)
             // and then multiplying the size of each dimension
    cudaMemcpy(cpu_output.data(), outputBuffer, cpu_output.size() * sizeof(float),
               cudaMemcpyDeviceToHost); // this can be fastened by keeping stuff on gpu

    // The Problem seems to be that the output is fp16 encoded, while "float" is fp32

    unsigned long outputSize = cpu_output.size();

    unsigned long dimensions = 85; // 0,1,2,3 ->box,4->confidence，5-85 -> coco classes confidence
    unsigned long rows = outputSize / dimensions; // 25.200
    unsigned long confidenceIndex = 4;
    unsigned long labelStartIndex = 5;

    std::vector<int> labels;
    std::vector<float> confidences;
    std::vector<cv::Rect> locations;

    cv::Rect rect;
    cv::Vec4f location;
    long numPushbacks = 0;
    long long numSkips = 0;
    long long numSkips2 = 0;
    for (unsigned long i = 0; i < rows; ++i) {
        unsigned long index = i * dimensions;
        if (cpu_output[index + confidenceIndex] <= 0.4f) {
            numSkips++;
            continue;
        }

        for (unsigned long j = labelStartIndex; j < dimensions; ++j) {
            cpu_output[index + j] = cpu_output[index + j] * cpu_output[index + confidenceIndex];
        }

        for (unsigned long k = labelStartIndex; k < dimensions; ++k) {
            if (cpu_output[index + k] <= 0.5f) {
                numSkips2++;
                continue;
            }

            rect = cv::Rect(cpu_output[index] * 1, cpu_output[index + 1] * 1,
                            cpu_output[index + 2] * 1, cpu_output[index + 3] * 1);
            locations.push_back(rect);

            labels.emplace_back(k - labelStartIndex);

            confidences.emplace_back(cpu_output[index + k]);
            numPushbacks++;
        }
    }

    // Evaluate results
    if (confidences.size() > 0) {
        float highest_conf = 0.0f;
        int highest_conf_index = 0;
        for (size_t i = 0; i < confidences.size(); i++) {
            if (confidences[i] > highest_conf) {
                highest_conf = confidences[i];
                highest_conf_index = i;
            }
        }
        detection->propability = highest_conf;
        detection->classID = labels[highest_conf_index];
        int left = locations[highest_conf_index].x;        // * frame.cols / width;
        int top = locations[highest_conf_index].y;         // * frame.rows / height;
        int right = locations[highest_conf_index].width;   // * frame.cols / width;
        int bottom = locations[highest_conf_index].height; // * frame.rows / height;
        detection->centerX = left + (right - left) / 2;
        detection->centerY = top + (bottom - top) / 2;
        detection->width = (left - right);
        detection->height = (top - bottom);
    }
}

rocket_tracker::detectionMSG processImage(cv::Mat img) {

    rocket_tracker::detectionMSG result;
    result.centerX = 0.0;
    result.centerY = 0.0;
    result.width = 0.0;
    result.height = 0.0;
    result.classID = 0;
    result.propability = 0.0;
    result.frameID = 0;

    uint64_t time = ros::Time::now().toNSec();

    // Preparing input tensor
    preprocessImgTRT(img, buffers[inputIndex]);

    uint64_t time2 = ros::Time::now().toNSec();

    context->enqueueV2(buffers, 0, nullptr);

    uint64_t time3 = ros::Time::now().toNSec();

    postprocessTRTdetections(buffers[outputIndex], &result);

    uint64_t time4 = ros::Time::now().toNSec();
    if (TIME_LOGGING)
        ROS_INFO("PRE: %.2lf FWD: %.2lf PST: %.2lf", (time2 - time) / 1000000.0,
                 (time3 - time2) / 1000000.0, (time4 - time3) / 1000000.0);

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

    ros::init(argc, argv, "FRAMEGRABBER");
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
    }

    nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(logger);
    assert(runtime != nullptr);
    nvinfer1::ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;

    inputIndex = engine->getBindingIndex("images");
    outputIndex = engine->getBindingIndex("output");
    for (int i = 0; i < 5; i++) {
        size_t size = 1;
        nvinfer1::Dims dims = engine->getBindingDimensions(i);
        for (size_t i = 0; i < dims.nbDims; ++i) {
            size *= dims.d[i];
        }
        auto binding_size = size * 1 * sizeof(float);

        cudaMalloc(&buffers[i], binding_size);
    }

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
