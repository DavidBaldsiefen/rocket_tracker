#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>

static cv::VideoCapture capture;
static double aspectRatio;

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

static FloatVector *img_vector;

bool initCapture(std::string videopath) {

    // Open video file
    ROS_INFO("Loading video from %s", videopath.c_str());
    capture = cv::VideoCapture(videopath);
    if (!capture.isOpened()) {
        capture.release();
        ROS_ERROR("Failed to open video capture! Provided path: %s", videopath.c_str());
        return false;
    }
    // publish the video specs
    ros::param::set("/rocket_tracker/input_fps", capture.get(cv::CAP_PROP_FPS));
    ros::param::set("/rocket_tracker/input_width", capture.get(cv::CAP_PROP_FRAME_WIDTH));
    ros::param::set("/rocket_tracker/input_height", capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    aspectRatio = capture.get(cv::CAP_PROP_FRAME_WIDTH) / capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    // refill the img_vector with zeroes, to prevent artifacts from new images
    if (img_vector != NULL) {
        std::fill(img_vector->begin(), img_vector->end(), 0.0);
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "FRAMEGRABBER");
    ros::NodeHandle nh("~");

    // Set Log-level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Get video-path from arguments
    std::string videopath = "";
    if (argc == 2) {
        videopath = argv[1];
    }
    if (videopath == "") {
        ros::param::get("/rocket_tracker/videopath", videopath);
        if (videopath == "") {
            ROS_ERROR("No video path provided through config or launch parameter!");
            return 0;
        }

    } else {
        ros::param::set("/rocket_tracker/videopath", videopath);
    }

    // Creating image-transport publisher for rqt
    image_transport::ImageTransport it(nh);
    // queuesize = fps * 2, so there is a 2 seconds buffer to publish
    image_transport::Publisher pubimg =
        it.advertise("/image_topic", (uint32_t)capture.get(cv::CAP_PROP_FPS) * 2);

    // Open video capture
    if (!initCapture(videopath)) {
        return 0;
    }

    sensor_msgs::ImagePtr msg;
    cv::Mat videoFrame;

    // Shared memory pointers
    boost::interprocess::managed_shared_memory segment;
    LongVector *notification_vector;

    // Get & set frame grabber fps target
    int target_fps;
    ros::param::param<int>("/rocket_tracker/fg_fps_target", target_fps, cv::CAP_PROP_FPS);
    ros::Rate r(target_fps); // Set loop rate for framegrabber
    ROS_INFO("Framegrabber FPS target: %d", target_fps);

    bool TIME_LOGGING = false, PERF_TEST = false, YOLOX_MODEL = false;
    ros::param::get("/rocket_tracker/time_logging", TIME_LOGGING);
    ros::param::get("/rocket_tracker/performance_test", PERF_TEST);
    ros::param::get("rocket_tracker/yolox_model", YOLOX_MODEL);

    bool TRT_ready = false;
    int model_width = 640;
    int model_height = 640;
    int model_size = 640 * 640;
    int trt_initialized = false;
    float input_float_divisor = 255.0f;
    if (YOLOX_MODEL) {
        input_float_divisor = 1.0f;
    }

    unsigned long frame_id = 0;

    // Main loop
    while (ros::ok()) {

        if (!capture.read(videoFrame)) {
            // Skip message during performance tests
            if (!PERF_TEST) {
                ROS_INFO("End of video reached - resetting to first frame.");
            }
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }
        // wait for tensorrt to be ready before publishing frames
        if (!trt_initialized && ros::param::getCached("rocket_tracker/trt_ready", TRT_ready) &&
            TRT_ready) {
            ros::param::get("rocket_tracker/model_width", model_width);
            ros::param::get("rocket_tracker/model_height", model_height);
            model_size = model_width * model_height;

            // initialize shared memory segment and access shared vectors
            segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only,
                                                                 "rocket_tracker_shared_memory");
            img_vector = segment.find<FloatVector>("img_vector").first;
            notification_vector = segment.find<LongVector>("notification_vector").first;

            trt_initialized = true;
        }
        ros::Time timestamp = ros::Time::now();

        // perform preprocessing
        if (trt_initialized) {
            // only resize down, and maintain aspect ratio
            if (videoFrame.rows > model_height || videoFrame.cols > model_width) {
                if (videoFrame.rows > videoFrame.cols) {
                    // height > width
                    cv::resize(videoFrame, videoFrame,
                               cv::Size(model_width * aspectRatio, model_height));
                } else {
                    // width > height
                    cv::resize(videoFrame, videoFrame,
                               cv::Size(model_width, model_height / aspectRatio));
                }
            }

            // using a pointer to the vector is significantly faster than calling img_vector->at for
            // every iteration
            float *vector_pointer = &(*img_vector)[0];

            // forEach is significantly faster than all other methods to traverse over the cv::Mat
            videoFrame.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int *position) -> void {
                // p[0-2] contains bgr data, position[0-1] the row-column location
                // Incoming data is BGR, so convert to RGB in the process
                int index = model_height * position[0] + position[1];
                vector_pointer[index] = p[2] / input_float_divisor;
                vector_pointer[model_size + index] = p[1] / input_float_divisor;
                vector_pointer[2 * model_size + index] = p[0] / input_float_divisor;
            });

            // Notify the IP that a new image is in the shared memory space
            notification_vector->at(1) = timestamp.toNSec();
            notification_vector->at(2) = ros::Time::now().toNSec() - timestamp.toNSec();
            notification_vector->at(0) =
                frame_id; // the frameId increases last so the remaining memory is already prepared
        }

        // publish video frame for GUI
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", videoFrame).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.frame_id = std::to_string(frame_id);
        pubimg.publish(msg);
        frame_id++;

        // check for parameter updates
        static int new_target_fps = target_fps;
        if (ros::param::getCached("rocket_tracker/fg_fps_target", new_target_fps) &&
            (new_target_fps != target_fps)) {
            ROS_INFO("New framegrabber FPS target: %d", new_target_fps);
            target_fps = new_target_fps;
            r = ros::Rate(target_fps);
        }
        static std::string new_videopath = videopath;
        if (ros::param::getCached("rocket_tracker/videopath", new_videopath) &&
            (new_videopath != videopath)) {
            capture.release();
            if (initCapture(new_videopath)) {
                videopath = new_videopath;
            } else {
                initCapture(videopath);
                ROS_WARN("Loading video from %s failed. Reverting to old path %s",
                         new_videopath.c_str(), videopath.c_str());
                new_videopath = videopath;
                ros::param::set("/rocket_tracker/videopath", videopath);
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    // Shut everything down cleanly
    capture.release();
    ros::shutdown();
    pubimg.shutdown();
    nh.shutdown();
    return 0;
}
