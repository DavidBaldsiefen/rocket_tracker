// TODOs:
// Use Rosparam to set image size, videopath, weightfile path....
// Make FG fps configurable
// Show if CUDA is used
// Make Apply&Restart button working
// remove "core" subpackage
// Make IP publish detected object class, probability, coordinates, #of detected objects => custom
// MSG
// catch exceptions in imageprocessor
// changing capture size instead of resizing
// rethink the ordering of resizing and drawing the detection marker

#include "evaluator_gui.h"
#include <image_transport/image_transport.h>
#include <rocket_tracker/detectionMSG.h>
#include <ros/ros.h>

static cv::Mat receivedFrame;
static rocket_tracker::detectionMSG receivedDetection;

Evaluator_GUI::Evaluator_GUI(QWidget *parent) : QWidget(parent) {
    ui.setupUi(this);

    ui.imageLabel->setFixedSize(QSize(480, 480));

    QObject::connect(ui.pushButton, &QAbstractButton::pressed, this,
                     &Evaluator_GUI::on_pushButton_click);
}

void Evaluator_GUI::setImage(cv::Mat img) {
    if (!img.empty()) {

        // resize
        cv::Mat scaledImg;
        cv::resize(img, scaledImg, cv::Size(480, 480));

        // draw detection
        if (receivedDetection.propability != 0.0) {
            cv::rectangle(scaledImg, cv::Point2d(receivedDetection.right, receivedDetection.top),
                          cv::Point2d(receivedDetection.left, receivedDetection.bottom),
                          cv::Scalar(0, 255, 0));
        }

        ui.imageLabel->setPixmap(
            QPixmap::fromImage(QImage(scaledImg.data, scaledImg.cols, scaledImg.rows,
                                      scaledImg.step, QImage::Format_RGB888)));
    }
}

void Evaluator_GUI::on_pushButton_click() {
    ROS_INFO("Button pressed");
    // do something
}

void callbackFrameGrabber(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!img->image.empty()) {
        receivedFrame = img->image;
    } else {
        ROS_WARN("Empty Frame received in image_processor_node::callbackFrameGrabber");
    }
}

void callbackImageProcessor(const rocket_tracker::detectionMSG &msg) {
    receivedDetection = msg;
}

int main(int argc, char **argv) {

    // start ROS
    ros::init(argc, argv, "EVALUATOR");
    ros::NodeHandle nh("~");

    // Creating image-transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subimg = it.subscribe("/image_topic", 1, &callbackFrameGrabber);

    // Creating image_processor subscriber
    ros::Subscriber subIP = nh.subscribe("/detection", 1, &callbackImageProcessor);
    receivedDetection.propability = 0.0;

    // start GUI
    QApplication app(argc, argv);
    Evaluator_GUI gui;
    gui.show();

    // Main Loop
    ROS_INFO("Starting main loop");
    ros::Rate r(100);
    while (ros::ok()) {
        // Process GUI events
        app.processEvents();
        // Process ROS events
        ros::spinOnce();
        gui.setImage(receivedFrame);
        r.sleep();
    }

    return 0;
}
