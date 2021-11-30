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

#include "evaluator_gui.h"
#include <image_transport/image_transport.h>
#include <ros/ros.h>

static cv::Mat receivedFrame;
static bool frameReceived = true;

Evaluator_GUI::Evaluator_GUI(QWidget *parent) : QWidget(parent) {
    ui.setupUi(this);

    ui.imageLabel->setFixedSize(QSize(480, 480));

    QObject::connect(ui.pushButton, &QAbstractButton::pressed, this,
                     &Evaluator_GUI::on_pushButton_click);
}

void Evaluator_GUI::setImage(cv::Mat img) {
    ui.imageLabel->setPixmap(
        QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)));
}

void Evaluator_GUI::on_pushButton_click() {
    ROS_INFO("Button pressed");
    // do something
}

void callbackFrameGrabber(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!img->image.empty()) {
        cv::Mat frame; // TODO: Flag?
        // rescale to 480x480
        cv::resize(img->image, frame, cv::Size(480, 480));
        receivedFrame = frame;
        frameReceived = true;

    } else {
        ROS_WARN("Empty Frame received in image_processor_node::callbackFrameGrabber");
    }
}

int main(int argc, char **argv) {

    // start ROS
    ros::init(argc, argv, "EVALUATOR");
    ros::NodeHandle nh("~");

    // Creating image-transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subimg = it.subscribe("/image_topic", 1, &callbackFrameGrabber);

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
        if (frameReceived) {
            gui.setImage(receivedFrame);
        }
        r.sleep();
    }

    return 0;
}
