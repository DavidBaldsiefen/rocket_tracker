#include "evaluator_node.h"
#include <ros/ros.h>

Evaluator_GUI::Evaluator_GUI(QWidget *parent) : QWidget(parent) {
    ui.setupUi(this);

    QObject::connect(ui.pushButton, &QAbstractButton::pressed, this,
                     &Evaluator_GUI::on_pushButton_click);
}

int main(int argc, char **argv) {

    ROS_INFO("Starting GUI");
    QApplication app(argc, argv);
    Evaluator_GUI gui;
    gui.show();
    return app.exec();
}

void Evaluator_GUI::on_pushButton_click() {
    ROS_INFO("Hui");
    // do something
}
