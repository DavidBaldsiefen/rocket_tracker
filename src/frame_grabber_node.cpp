#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <rocket_tracker/image.h>
#include <ros/ros.h>

static cv::VideoCapture capture;

bool initCapture(std::string videopath) {

    // Open video file
    ROS_INFO("Loading video from %s", videopath.c_str());
    capture = cv::VideoCapture(videopath);
    if (!capture.isOpened()) {
        capture.release();
        ROS_ERROR("Failed to open video capture! Provided path: %s", videopath.c_str());
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
    ros::Publisher pubimg2 = nh.advertise<rocket_tracker::image>(
        "/image_topic_2", (uint32_t)capture.get(cv::CAP_PROP_FPS) * 2);

    // Open video capture
    if (!initCapture(videopath)) {
        return 0;
    }

    sensor_msgs::ImagePtr msg;
    rocket_tracker::image msg2;
    cv::Mat videoFrame;

    // Main loop
    int target_fps;
    ros::param::param<int>("/rocket_tracker/fg_fps_target", target_fps, cv::CAP_PROP_FPS);
    ros::Rate r(target_fps); // Set loop rate for framegrabber

    bool TIME_LOGGING;
    ros::param::get("/rocket_tracker/time_logging", TIME_LOGGING);

    uint frame_id = 0;
    while (ros::ok()) {

        if (!capture.read(videoFrame)) {
            ROS_INFO("End of video reached - resetting to first frame.");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            frame_id = 0;
            ros::Duration(0.5).sleep();
            continue;
        }
        ros::Time timestamp = ros::Time::now();

        // publish videoframe
        // cv::cvtColor(videoFrame, videoFrame, cv::COLOR_BGR2RGB);
        // =============================================
        // perform preprocessing
        // only resize down
        if (videoFrame.rows > 640 || videoFrame.cols > 640) {
            cv::resize(videoFrame, videoFrame, cv::Size(640, 640));
        }

        static int model_size = 640 * 640;
        std::vector<float> inputArray;
        inputArray.reserve(1 * 3 * model_size);

        // for each is significantly faster than all other methods to traverse over the cv::Mat
        // (read online and confirmed myself)
        videoFrame.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int *position) -> void {
            // p[0-2] contains bgr data, position[0-1] the row-column location
            // Incoming data is BGR, so convert to RGB in the process
            int index = 640 * position[0] + position[1];
            inputArray[index] = p[2] / 255.0f;
            inputArray[model_size + index] = p[1] / 255.0f;
            inputArray[2 * model_size + index] = p[0] / 255.0f;
        });
        if (TIME_LOGGING)
            ROS_INFO("PRE FG: %.2f", (ros::Time::now().toNSec() - timestamp.toNSec()) / 1000000.0);
        // =============================================
        msg2.image = inputArray;
        msg2.id = frame_id;
        msg2.stamp = timestamp;
        pubimg2.publish(msg2);
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
    nh.shutdown();
    return 0;
}
