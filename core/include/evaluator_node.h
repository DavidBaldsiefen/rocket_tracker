#include <QWidget>
#include <core/ui_gui_form.h>

class Evaluator_GUI : public QWidget {
    Q_OBJECT

  public:
    // Evaluator_GUI(QWidget *parent = nullptr);
    Evaluator_GUI(QWidget *parent = nullptr);
    virtual ~Evaluator_GUI() {}

  private slots:
    void on_pushButton_click();

  private:
    Ui_Form ui;
};
