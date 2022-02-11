// TODOs:
// catch exceptions in imageprocessor
// rethink the ordering of resizing and drawing the detection marker
// evaluate subscriber/publisher buffers

#include "evaluator_gui.h"
#include <image_transport/image_transport.h>
#include <rocket_tracker/detectionMSG.h>
#include <ros/ros.h>

static cv::Mat receivedFrame;
static unsigned long receivedFrameID;
static std::vector<rocket_tracker::detectionMSG> receivedDetections;
static Ui_Form *ui;
static bool redrawGUI = false;

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

        cv::Mat guiIMG =
            img.clone(); // create deep copy to prevent drawing multiple times on the same frame

        if (receivedDetections.size() != 0) {
            rocket_tracker::detectionMSG receivedDetection;
            // The syncing of FrameID and detectionID can lead to nothing being drawn when FG >> IP
            if (ui.sync_dets_and_images->isChecked()) {
                // iterate backwards as the fitting detection is most likely somewhere towards the
                // end
                for (int i = (receivedDetections.size() - 1); i >= 0; i--) {
                    if (receivedDetections[i].frameID == receivedFrameID) {
                        receivedDetection = receivedDetections[i];
                        break;
                    }
                }
            } else {
                receivedDetection = receivedDetections[receivedDetections.size() - 1];
            }

            // Confidence threshold is applied in image processor
            if (receivedDetection.propability != 0.0) {

                // OpenCV needs [leftX, topY, width, height] => rectangle based around top left
                // corner
                int leftX = receivedDetection.centerX - (receivedDetection.width / 2);
                int topY = receivedDetection.centerY - (receivedDetection.height / 2);
                cv::Rect detectionRect =
                    cv::Rect(leftX, topY, receivedDetection.width, receivedDetection.height);

                // the detection coordinates relate to the models coordinate system
                // however, the iage might have been scaled down (upscaling does not happen)
                int model_width = 640, model_height = 640;
                ros::param::getCached("/rocket_tracker/model_width", model_width);
                ros::param::getCached("/rocket_tracker/model_height", model_height);
                if (guiIMG.cols > model_width || guiIMG.rows > model_height) {
                    detectionRect.x = guiIMG.cols * detectionRect.x / model_width;
                    detectionRect.y = guiIMG.rows * detectionRect.y / model_height;
                }

                // draw rectangle to image
                cv::rectangle(guiIMG, detectionRect, cv::Scalar(0, 255, 0));

                // note propability in GUI
                std::string str = "Probability of detected object: " +
                                  std::to_string((int)(receivedDetection.propability * 100)) + "%";
                ui.propability_label->setText(QString::fromStdString(str));
            } else {
                ui.propability_label->setText(QString("Probability of detected object: ---"));
            }
        }

        // Push image to GUI
        cv::resize(guiIMG, guiIMG, cv::Size(ui.imageLabel->width(), ui.imageLabel->height()));
        ui.imageLabel->setPixmap(QPixmap::fromImage(
            QImage(guiIMG.data, guiIMG.cols, guiIMG.rows, guiIMG.step, QImage::Format_RGB888)));
    }
}

void Evaluator_GUI::on_applyBtn() {
    ros::param::set("rocket_tracker/videopath", ui.videopath->text().toStdString());
    ros::param::set("rocket_tracker/fg_fps_target", ui.fg_fps_target->value());
    ROS_INFO("Applying new config");
}

void callbackFrameGrabber(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!img->image.empty()) {
        receivedFrame = img->image;
        cv::cvtColor(receivedFrame, receivedFrame, cv::COLOR_BGR2RGB);
        receivedFrameID = std::stoul(msg->header.frame_id);
        redrawGUI = true;
    } else {
        ROS_WARN("Empty Frame received in image_processor_node::callbackFrameGrabber");
    }
}

void callbackImageProcessor(const rocket_tracker::detectionMSG &msg) {

    // Buffer up to 64 detections
    receivedDetections.push_back(msg);
    if (receivedDetections.size() > 64) {
        receivedDetections.erase(receivedDetections.begin());
    }

    // Calculate and display statistics
    // Processing time
    ui->ip_time->setText(QString::number(msg.processingTime, 'f', 2) + QString("ms"));

    // avg processing time
    double avgProcessingTime = 0.0;
    for (int i = 0; i < receivedDetections.size(); i++) {
        avgProcessingTime += receivedDetections[i].processingTime;
    }
    avgProcessingTime /= receivedDetections.size();
    ui->ip_latency_avg->setText(QString::number(avgProcessingTime, 'f', 2) + QString("ms"));

    // Throughput
    unsigned long throughputNS =
        receivedDetections[receivedDetections.size() - 1].timestamp.toNSec() -
        receivedDetections[0].timestamp.toNSec();
    double throughput = throughputNS / 1000000.0; // convert to ms
    throughput /= receivedDetections.size();
    throughput = 1000.0 / throughput; // convert to fps
    ui->ip_throughput->setText(QString::number(throughput, 'f', 0) + QString("fps"));

    redrawGUI = true;
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
        str = "Engine Ready: " + std::string(trtReady ? "Yes" : "No");
        ui->trt_label->setText(QString::fromStdString(str));
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

    // start GUI
    QApplication app(argc, argv);
    Evaluator_GUI gui;
    ui = gui.getUi();
    gui.show();

    // Main Loop
    ros::Rate guiLoopRate(200);
    while (ros::ok()) {
        // Process GUI events
        app.processEvents();
        // Process ROS events
        ros::spinOnce();
        if (redrawGUI) {
            gui.setImage(receivedFrame);
            redrawGUI = false;
        }
        pushRosParamsToGui(gui.getUi());
        guiLoopRate.sleep();
    }

    return 0;
}
