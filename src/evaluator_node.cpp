// TODOs:
// catch exceptions in imageprocessor
// rethink the ordering of resizing and drawing the detection marker
// evaluate subscriber/publisher buffers

#include "evaluator_gui.h"
#include <image_transport/image_transport.h>
#include <rocket_tracker/detectionMSG.h>
#include <ros/ros.h>

static cv::Mat receivedFrame;
static rocket_tracker::detectionMSG receivedDetection;
static Ui_Form *ui;

Evaluator_GUI::Evaluator_GUI(QWidget *parent) : QWidget(parent) {
    ui.setupUi(this);

    ui.imageLabel->setMaximumSize(QSize(480, 480));

    std::string videopath;
    ros::param::get("/rocket_tracker/videopath", videopath);
    ui.videopath->setText(QString::fromStdString(videopath));
    ros::param::set("rocket_tracker/use_highest_rocket", ui.ip_height_filter_cb->isChecked());

    QObject::connect(ui.applyBtn, &QAbstractButton::pressed, this, &Evaluator_GUI::on_applyBtn);
}

void Evaluator_GUI::setImage(cv::Mat img) {
    if (!img.empty()) {

        // Confidence threshold is applied in image processor, so draw everything
        if (receivedDetection.propability != 0.0) {

            // OpenCV needs [leftX, topY, width, height] => rectangle based around top left corner
            int leftX = receivedDetection.centerX - (receivedDetection.width / 2);
            int topY = receivedDetection.centerY - (receivedDetection.height / 2);
            cv::Rect detectionRect =
                cv::Rect(leftX, topY, receivedDetection.width, receivedDetection.height);

            // draw rectangle to image
            cv::rectangle(img, detectionRect, cv::Scalar(0, 255, 0));

            // note propability in GUI
            std::string str = "Probability of detected object: " +
                              std::to_string((int)(receivedDetection.propability * 100)) + "%";
            ui.propability_label->setText(QString::fromStdString(str));
        } else {
            ui.propability_label->setText(QString("Probability of detected object: ---"));
        }

        // Push image to GUI
        cv::resize(img, img, cv::Size(ui.imageLabel->width(), ui.imageLabel->height()));
        ui.imageLabel->setPixmap(QPixmap::fromImage(
            QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)));
    }
}

void Evaluator_GUI::on_applyBtn() {
    ros::param::set("rocket_tracker/videopath", ui.videopath->text().toStdString());
    ros::param::set("rocket_tracker/fg_fps_target", ui.fg_fps_target->value());
    ros::param::set("rocket_tracker/use_highest_rocket", ui.ip_height_filter_cb->isChecked());
    ROS_INFO("Applying new config");
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

    // Calculate and display statistics
    // Processing time
    static double avgProcessingTime = 0.0;
    ui->ip_time->setText(QString::number(msg.processingTime, 'f', 2) + QString("ms"));

    // theoretically possible avg. fps
    static int avgCounter = 0;
    avgProcessingTime += msg.processingTime;
    avgCounter++;
    if (avgCounter >= 10) {
        ui->ip_fps_avg->setText(QString::number(10000.0 / avgProcessingTime, 'f', 0) +
                                QString("fps"));
        avgCounter = 0;
        avgProcessingTime = 0.0;
    }

    // actual fps based on frequenc of incoming messages in IP
    static double lastTimestamp = msg.timestamp - 0.1; // subtract 0.1 to preent div/0
    double actualFPS = (1000.0 / (msg.timestamp - lastTimestamp));
    lastTimestamp = msg.timestamp;
    ui->ip_fps_actual->setText(QString::number(actualFPS, 'f', 0) + QString("fps"));
}

void pushRosParamsToGui(Ui_Form *ui) {
    double input_fps, input_width, input_height, rescaled_width, rescaled_height;
    bool trtReady = false;
    bool retVal = false;
    retVal |= ros::param::get("/rocket_tracker/input_fps", input_fps);
    retVal |= ros::param::get("/rocket_tracker/input_width", input_width);
    retVal |= ros::param::get("/rocket_tracker/input_height", input_height);
    retVal |= ros::param::get("/rocket_tracker/trt_ready", trtReady);
    retVal |= ros::param::get("/rocket_tracker/model_width", rescaled_width);
    retVal |= ros::param::get("/rocket_tracker/model_height", rescaled_height);
    if (retVal) {
        std::string str = "Input Video: " + std::to_string((int)input_width) + "x" +
                          std::to_string((int)input_height) + "@" + std::to_string((int)input_fps) +
                          "fps => rescaled to " + std::to_string((int)rescaled_width) + "x" +
                          std::to_string((int)rescaled_height);
        ui->video_details->setText(QString::fromStdString(str));
        str = "TensorRT Engine Ready: " + std::string(trtReady ? "Yes" : "No");
        ui->trt_ready_label->setText(QString::fromStdString(str));
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
    ui = gui.getUi();
    gui.show();

    // Main Loop
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
