#include "evaluator_gui.h"
#include <ros/ros.h>

Evaluator_GUI::Evaluator_GUI(QWidget *parent) : QWidget(parent) {
    ui.setupUi(this);

    QObject::connect(ui.pushButton, &QAbstractButton::pressed, this,
                     &Evaluator_GUI::on_pushButton_click);
}

void Evaluator_GUI::on_pushButton_click() {
    ROS_INFO("Button pressed");
    // do something
}

int main(int argc, char **argv) {

    // start ROS
    ros::init(argc, argv, "EVALUATOR");
    ros::NodeHandle nh("~");

    // start GUI
    QApplication app(argc, argv);
    Evaluator_GUI gui;
    gui.show();

    // Main Loop
    ros::Rate r(100);
    while (ros::ok()) {
        // Process GUI events
        app.processEvents();
        // Process ROS events
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
