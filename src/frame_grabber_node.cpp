#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>

static cv::VideoCapture capture;

bool initCapture(std::string videopath) {

    // Open video file
    ROS_INFO("Loading video from %s", videopath.c_str());
    capture = cv::VideoCapture(videopath);
    if (!capture.isOpened()) {
        capture.release();
        ROS_ERROR("Failed to open capture!");
        return false;
    }
    // "publish" the video specs
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

    // Get imagesize from arguments
    int width = 480, height = 480;
    if (argc == 2) {
        width = std::stoi(argv[1]);
    } else if (argc == 3) {
        width = std::stoi(argv[1]);
        height = std::stoi(argv[2]);
    }
    ROS_INFO("Video will be scaled to %dx%d", width, height);
    ros::param::set("/rocket_tracker/rescaled_width", width);
    ros::param::set("/rocket_tracker/rescaled_height", height);

    // Creating image-transport publisher for rqt
    image_transport::ImageTransport it(nh);
    // queuesize = fps * 2, so there is a 2 seconds buffer to publish
    image_transport::Publisher pubimg =
        it.advertise("/image_topic", (uint32_t)capture.get(cv::CAP_PROP_FPS) * 2);

    // Open video capture
    std::string videopath;
    ros::param::param<std::string>("/rocket_tracker/videopath", videopath,
                                   "/home/david/Downloads/silent_launches.mp4");
    if (!initCapture(videopath)) {
        return 0;
    }

    sensor_msgs::ImagePtr msg;
    cv::Mat videoFrame;

    // Main loop
    int target_fps;
    ros::param::param<int>("/rocket_tracker/fg_fps_target", target_fps, cv::CAP_PROP_FPS);
    ros::Rate r(target_fps); // Set loop rate for framegrabber
    while (ros::ok()) {

        if (!capture.read(videoFrame)) {
            ROS_INFO("End of video reached - resetting to first frame.");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            ros::Duration(0.5).sleep();
            continue;
        }

        // resize videoframe
        cv::resize(videoFrame, videoFrame, cv::Size(width, height));

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", videoFrame).toImageMsg();
        pubimg.publish(msg);

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
    nh.shutdown();
    return 0;
}
