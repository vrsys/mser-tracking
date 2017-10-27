#ifndef IMAGEPROCESSINGWINDOW_H
#define IMAGEPROCESSINGWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QtCore>
#include <QLabel>
#include <QKeyEvent>

#include "Processing/ImageProcessing/imageprocessing.hpp"
#include "InputDevice/CaptureDevice/capturedevice.hpp"
#include "Tools/tools.hpp"

#include "calibrationwindow.hpp"
#include "createmaxwindow.hpp"

// todo: delete
#include "global.hpp"

namespace Ui {
class ImageProcessingWindow;
}

class ImageProcessingWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ImageProcessingWindow(QWidget *parent = 0);
    ~ImageProcessingWindow();

private slots:
    void process();
    void processCreateMax();
    void processCalib();

    // top button
    void on_btnPauseResume_clicked();
    void on_btnNextFrame_clicked();
    void on_btnSaveImages_clicked();

    // CC
    void on_spinBoxCCThreshold_valueChanged(int threshold);

    // Classification
    void on_spinBoxMinParents_valueChanged(int);
    void on_spinBoxSizeDiff_valueChanged(double);
    void on_spinBoxLevel_valueChanged(int);
    void on_spinBoxHandSizeDiff_valueChanged(int);

    // general
    void on_spinBoxLoopTimer_valueChanged(int arg1);
    void on_checkBoxFastMode_toggled(bool checked);
    void on_textTUIOHost_returnPressed();

    // Min Max
    void on_buttonCaptureMin_clicked();
    void on_buttonCaptureMax_clicked();
    void on_buttonCreateMax_clicked();

    // top menu
    void on_actionCalibrate_triggered();
    void on_actionImage_Stream_triggered();
    void on_actionGrasshopper_triggered();
    void on_actionVideo_File_triggered();
    void on_actionOtherCam_triggered();


private:
    void drawImages();
    void setImage(cv::Mat const &img, QLabel* label);
    void saveImages();

    void setImageFolder(std::string s);
    void setVideoFile(const std::string& s);
    void setCameraDevice(CAM::DEVICEFLAG flag);

    void checkRootPath();

    void writeLog();

    bool loadConfigParamter();
    void saveClassificationParameter();

    void setMSERParameter();
    void setClassificationParameter();
    void keyPressEvent(QKeyEvent *event);

    Ui::ImageProcessingWindow *mUI;
    ImageProcessing       mProc;


    std::string             mRootPathDefault;
    std::string             mRootPath; // of current device (imagefolder, videofolder)

    std::vector<QLabel*>   mQImagesLabels; // Gui Image labels

    // Window timer
    QTimer*               mQTimer;
    QTimer*               mQTimerCreateMax;
    QTimer*               mQTimerCalib;

    std::vector<cv::Mat>  mMatImages;
    cv::Mat               mNewMaxImage;

    std::vector<std::shared_ptr<Hand> > mHands;

    // update timer
    int             mLoopTimer;

    // save images
    std::string     mSaveImagesPath;
    int             mSaveIndex;

    // FPS
    QElapsedTimer   mElapsedTimer;
    int             mEllapsedPerSecond;
    int             mFrame;
    int             mLastFrame;
    int             mFPS;

    // create new max
    CreateMaxWindow mCreateMaxWindow;

    // calibrate window
    CalibrationWindow mCalibWindow;

    // tuio host
    std::string mTUIOHost;

    // device modes
    bool mPause;
    bool mCamSuccess;
    bool mSaveImages;
    bool mSpeedMode;
    bool mVideoMode;
    bool mImageMode;

};

#endif // IMAGEPROCESSINGWINDOW_H
