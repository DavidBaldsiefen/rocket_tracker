#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>

cv::VideoCapture capture;

bool init() {
    // Set Log-level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Open video file
    capture = cv::VideoCapture("/home/david/Downloads/silent_launches.mp4");
    if (!capture.isOpened()) {
        capture.release();
        ROS_ERROR("Failed to open capture!");
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    if (!init()) {
        return 0;
    }

    ros::init(argc, argv, "FRAMEGRABBER");
    ros::NodeHandle nh("~");

    // Creating image-transport publisher for rqt
    image_transport::ImageTransport it(nh);
    // queuesize = fps * 2, so there is a 2 seconds buffer to publish
    image_transport::Publisher pubimg =
        it.advertise("/image_topic", (uint32_t)capture.get(cv::CAP_PROP_FPS) * 2);

    sensor_msgs::ImagePtr msg;

    cv::Mat videoFrame;

    // Main loop
    ros::Rate r(capture.get(cv::CAP_PROP_FPS)); // Set loop rate to video fps
    while (ros::ok()) {

        if (!capture.read(videoFrame)) {
            ROS_INFO("End of video reached - resetting to first frame.");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            ros::Duration(0.5).sleep();
            continue;
        }

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", videoFrame).toImageMsg();
        pubimg.publish(msg);

        ros::spinOnce();
        r.sleep();
    }

    // Shut everything down cleanly
    ros::shutdown();
    pubimg.shutdown();
    nh.shutdown();
    return 0;
}
