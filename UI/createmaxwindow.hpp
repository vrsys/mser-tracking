#ifndef CREATEMAXWINDOW_H
#define CREATEMAXWINDOW_H

#include <memory>

#include <QDialog>
#include <QMessageBox>
#include <QTimer>

#include <opencv2/opencv.hpp>

#include "InputDevice/CaptureDevice/capturedevice.hpp"
#include "InputDevice/CaptureDevice/grasshoppercamera.hpp"
#include "Tools/tools.hpp"

#include "Processing/ImageProcessing/ImagePreProcessing/imagepreprocessing.hpp"

namespace Ui {
class CreateMaxWindow;
}

class CreateMaxWindow : public QDialog
{
    Q_OBJECT

public:
    explicit CreateMaxWindow(QWidget *parent = 0);
    ~CreateMaxWindow();
    void process(const cv::Mat& img);

    cv::Mat getMaxImage() const;

    bool success() const;
    void closeWindow();

    bool getCanceled() const;

    void initImage(const cv::Mat& img);

private slots:
    void on_buttonFinished_clicked();
    void on_buttonCancel_clicked();

    void keyPressEvent(QKeyEvent *event);



private:

    Ui::CreateMaxWindow *mUI;
    cv::Mat mImage;

    bool mSuccess;
    bool mCancel;
};

#endif // CREATEMAXWINDOW_H
