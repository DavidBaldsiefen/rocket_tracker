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
// evaluate subscriber/publisher buffers

#include "evaluator_gui.h"
#include <image_transport/image_transport.h>
#include <rocket_tracker/detectionMSG.h>
#include <ros/ros.h>

static cv::Mat receivedFrame;
static rocket_tracker::detectionMSG receivedDetection;

Evaluator_GUI::Evaluator_GUI(QWidget *parent) : QWidget(parent) {
    ui.setupUi(this);

    ui.imageLabel->setMaximumSize(QSize(480, 480));

    std::string videopath;
    ros::param::get("/rocket_tracker/videopath", videopath);
    ui.videopath->setText(QString::fromStdString(videopath));

    QObject::connect(ui.applyBtn, &QAbstractButton::pressed, this, &Evaluator_GUI::on_applyBtn);
}

void Evaluator_GUI::setImage(cv::Mat img) {
    if (!img.empty()) {

        // draw detection
        if (receivedDetection.propability != 0.0) {
            cv::rectangle(img, cv::Point2d(receivedDetection.right, receivedDetection.top),
                          cv::Point2d(receivedDetection.left, receivedDetection.bottom),
                          cv::Scalar(0, 255, 0));

            // note propability in GUI
            std::string str = "Probability of detected object: " +
                              std::to_string((int)(receivedDetection.propability * 100)) + "%";
            ui.propability_label->setText(QString::fromStdString(str));
        } else {
            ui.propability_label->setText(QString("Probability of detected object: ---"));
        }

        ui.imageLabel->setPixmap(QPixmap::fromImage(
            QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)));
    }
}

void Evaluator_GUI::on_applyBtn() {
    ros::param::set("rocket_tracker/videopath", ui.videopath->text().toStdString());
    ros::param::set("rocket_tracker/fg_fps_target", ui.fg_fps_target->value());
    ROS_INFO("Applying new config");
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

void pushRosParamsToGui(Ui_Form *ui) {
    double input_fps, input_width, input_height, rescaled_width, rescaled_height;
    bool usingCUDA = false;
    bool retVal = false;
    retVal |= ros::param::get("/rocket_tracker/input_fps", input_fps);
    retVal |= ros::param::get("/rocket_tracker/input_width", input_width);
    retVal |= ros::param::get("/rocket_tracker/input_height", input_height);
    retVal |= ros::param::get("/rocket_tracker/using_cuda", usingCUDA);
    retVal |= ros::param::get("/rocket_tracker/rescaled_width", rescaled_width);
    retVal |= ros::param::get("/rocket_tracker/rescaled_height", rescaled_height);
    if (retVal) {
        std::string str = "Input Video: " + std::to_string((int)input_width) + "x" +
                          std::to_string((int)input_height) + "@" + std::to_string((int)input_fps) +
                          "fps => rescaled to " + std::to_string((int)rescaled_width) + "x" +
                          std::to_string((int)rescaled_height);
        ui->video_details->setText(QString::fromStdString(str));
        str = "Using CUDA: " + std::string(usingCUDA ? "Yes" : "No");
        ui->cuda_label->setText(QString::fromStdString(str));
    }
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
        pushRosParamsToGui(gui.getUi());
        r.sleep();
    }

    return 0;
}
