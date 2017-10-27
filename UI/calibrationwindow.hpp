#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <vector>
#include <memory>

#include <QDialog>
#include <QMessageBox>
#include <QTimer>

#include "InputDevice/CaptureDevice/capturedevice.hpp"
#include "InputDevice/CaptureDevice/grasshoppercamera.hpp"
#include "InputDevice/CaptureDevice/imagefolder.hpp"

#include "Processing/ImageProcessing/Calibration/calibration.hpp"

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrationWindow(QWidget *parent = 0);
    ~CalibrationWindow();

    void process(const cv::Mat& rawImg);
    void init(const cv::Mat& rawImg);

    bool cancel() const;
    void closeWindow();

private slots:
    void on_pushButtonCancel_clicked();
    void on_pushButton_skipCalib_clicked();
    void on_pushButtonApply_clicked();

private:
    void processCalibration();
    void processPerspective();

    Ui::CalibrationWindow *mUI;

    cv::Mat mRawImage;
    cv::Mat mNewUndistorted;
    cv::Mat mCurrentUndistorted;
    cv::Mat mCroped;

    Calibration mCalib;

    bool mCalibSuccess;
    bool mSkipCalibration;

    bool mPerspectiveSuccess;
    bool mCancel;

};

#endif // CALIBRATIONWINDOW_H
