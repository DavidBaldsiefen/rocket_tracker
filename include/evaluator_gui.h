#include <QThread>
#include <QWidget>
#include <cv_bridge/cv_bridge.h>
#include <rocket_tracker/ui_evaluator_gui_form.h>
#include <ros/ros.h>

class Evaluator_GUI : public QWidget {
    Q_OBJECT

  public:
    // Evaluator_GUI(QWidget *parent = nullptr);
    Evaluator_GUI(QWidget *parent = nullptr);
    virtual ~Evaluator_GUI() {}

    void setImage(cv::Mat img);

    Ui_Form *getUi() {
        return &ui;
    }

  private slots:
    void on_pushButton_click();

  private:
    Ui_Form ui;
};
