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
        ros::param::param<std::string>("/rocket_tracker/videopath", videopath,
                                       "/home/david/Downloads/silent_launches.mp4");
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

    // Create a new shared memory segment with given name and size. Size is a little bit larger to
    // have some buffer
    boost::interprocess::shared_memory_object::remove("rocket_tracker_shared_memory");
    boost::interprocess::managed_shared_memory segment(
        boost::interprocess::create_only, "rocket_tracker_shared_memory",
        1 * 3 * 640 * 640 * sizeof(float) + 256 * sizeof(unsigned long));

    // Initialize shared memory STL-compatible allocator
    const ShmemAllocatorFloat alloc_inst_float(segment.get_segment_manager());
    const ShmemAllocatorLong alloc_inst_long(segment.get_segment_manager());

    // Construct vectors for the image data in shared memory
    FloatVector *imageVector = segment.construct<FloatVector>("img_vector")(alloc_inst_float);
    LongVector *notificationVector =
        segment.construct<LongVector>("notification_vector")(alloc_inst_long);

    // resize both vectors
    imageVector->resize(1 * 3 * 640 * 640);
    notificationVector->resize(3);

    // Get & set frame grabber fps target
    int target_fps;
    ros::param::param<int>("/rocket_tracker/fg_fps_target", target_fps, cv::CAP_PROP_FPS);
    ros::Rate r(target_fps); // Set loop rate for framegrabber

    bool TIME_LOGGING;
    ros::param::get("/rocket_tracker/time_logging", TIME_LOGGING);

    bool TRT_ready = false;
    int model_width = 640;
    int model_height = 640;
    int model_size = 640 * 640;
    int trt_initialized = false;

    unsigned long frame_id = 0;

    // Main loop
    while (ros::ok()) {

        if (!capture.read(videoFrame)) {
            ROS_INFO("End of video reached - resetting to first frame.");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }
        // wait for tensorrt to be ready before publishing frames
        if (!trt_initialized && ros::param::getCached("rocket_tracker/trt_ready", TRT_ready) &&
            TRT_ready) {
            ros::param::get("rocket_tracker/model_width", model_width);
            ros::param::get("rocket_tracker/model_height", model_height);
            model_size = model_width * model_height;
            imageVector->resize(1 * 3 * model_size);
            trt_initialized = true;
        }
        ros::Time timestamp = ros::Time::now();

        // perform preprocessing
        if (trt_initialized) {
            // only resize down
            if (videoFrame.rows > model_height || videoFrame.cols > model_width) {
                cv::resize(videoFrame, videoFrame, cv::Size(model_width, model_height));
            }

            // forEach is significantly faster than all other methods to traverse over the cv::Mat
            videoFrame.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int *position) -> void {
                // p[0-2] contains bgr data, position[0-1] the row-column location
                // Incoming data is BGR, so convert to RGB in the process
                int index = model_height * position[0] + position[1];
                imageVector->at(index) = p[2] / 255.0f;
                imageVector->at(model_size + index) = p[1] / 255.0f;
                imageVector->at(2 * model_size + index) = p[0] / 255.0f;
            });

            // Notify the IP that a new image is in the shared memory space
            notificationVector->at(1) = timestamp.toNSec();
            notificationVector->at(2) = ros::Time::now().toNSec() - timestamp.toNSec();
            notificationVector->at(0) =
                frame_id; // the frameId increases last so the remaining memory is already prepared
        }

        // publish video frame for GUI
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", videoFrame).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.seq = frame_id;
        pubimg.publish(msg);
        frame_id++;

        // check for parameter updates
        static int new_target_fps = target_fps;
        if (ros::param::getCached("rocket_tracker/fg_fps_target", new_target_fps) &&
            (new_target_fps != target_fps)) {
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
    segment.destroy<FloatVector>("img_vector");
    segment.destroy<LongVector>("notification_vector");
    boost::interprocess::shared_memory_object::remove("rocket_tracker_shared_memory");
    nh.shutdown();
    return 0;
}
