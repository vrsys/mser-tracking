#include "calibrationwindow.hpp"
#include "ui_calibrationwindow.h"

CalibrationWindow::CalibrationWindow(QWidget *parent) :
    QDialog(parent),
    mUI(new Ui::CalibrationWindow),
    mCalib(),
    mCalibSuccess(false),
    mPerspectiveSuccess(false),
    mCancel(false)
{
    mUI->setupUi(this);
}

CalibrationWindow::~CalibrationWindow()
{
    delete mUI;
}

void CalibrationWindow::process(const cv::Mat& rawImg)
{
    mRawImage = rawImg;
    if(!mSkipCalibration && !mCalibSuccess){
        processCalibration();
    }
    else if (!mPerspectiveSuccess){
        processPerspective();
    }
}

void CalibrationWindow::init(const cv::Mat& rawImg)
{
    mUI->labelImageLeft->setPixmap(QPixmap::fromImage(matToQImage(rawImg)));
    mUI->labelImageRight->setPixmap(QPixmap::fromImage(matToQImage(rawImg)));

    mPerspectiveSuccess = false;
    mCancel = false;
    mCalibSuccess = false;
    mSkipCalibration = false;
}

void CalibrationWindow::processCalibration()
{
    mUI->pushButton_skipCalib->setEnabled(true);

    mCalibSuccess = mCalib.findCalibrationMatrix(mRawImage);

    if(!mCalib.getTrackedImage().empty()){
        mUI->labelImageLeft->setPixmap(QPixmap::fromImage(matToQImage(mCalib.getTrackedImage())));
    }
    mUI->labelImageRight->setPixmap(QPixmap::fromImage(matToQImage(mRawImage)));

    if(!mCalibSuccess) return;

    mUI->labelImageLeft->setPixmap(QPixmap::fromImage(matToQImage(mCalib.getNewUndistort())));
    mUI->labelImageRight->setPixmap(QPixmap::fromImage(matToQImage(mRawImage)));
    mUI->pushButton_skipCalib->setEnabled(false);

    QMessageBox::StandardButton happyCalib;
    happyCalib = QMessageBox::question(this, "happy?", "are you happy with result?", QMessageBox::Yes|QMessageBox::No);


    if (happyCalib == QMessageBox::No) {
        closeWindow();
        return;
    }
    else std::cout << happyCalib << std::endl;

    mCalib.saveNewCalibrationMatrix();
    mUI->instructionBox->append(QString::fromStdString("2. Congratulation. Calibration matrix and undistort image was saved to "+mCalib.getPath()+"."));
}

void CalibrationWindow::processPerspective()
{
    mUI->instructionBox->append("3. To correct perspective please open undistort.jpg (in calibration folder) with gimp / photoshop and mask the table surface white and the rest black.");

    QString strFileName = QFileDialog::getOpenFileName();

    if(strFileName == "") {
        std::cout << "no file chosen." << std::endl;
        closeWindow();
        return;
    }

    QDir dir(QCoreApplication::applicationDirPath());
    //QString relativePath = dir.relativeFilePath(strFileName);
    QString absolutePath = dir.absoluteFilePath(strFileName);

    std::string hardCoded("Processing/ImageProcessing/ImagePreProcessing/Calibration/undistort.jpg");
    std::cout << absolutePath.toStdString() << std::endl;

    cv::Mat maskedImage = cv::imread(absolutePath.toStdString());//relativePath.toStdString());
    if (maskedImage.empty()) {
        std::cout << "error: image not read from file" << absolutePath.toStdString() << std::endl;
        closeWindow();
        return;
    }

    mUI->labelImageLeft->setPixmap(QPixmap::fromImage(matToQImage(maskedImage)));
    mUI->labelImageRight->setPixmap(QPixmap::fromImage(matToQImage(maskedImage)));

    mCalib.setNewPerspectiveCorrection(maskedImage);
    std::cout << "setNewPerspectiveCorrection" << std::endl;

    mUI->labelImageLeft->setPixmap(QPixmap::fromImage(matToQImage(mCalib.getTransformedImage())));

    mUI->instructionBox->append("Congratulation. Click apply to save calibration data. Calibration matrices will be loaded automatically next start.");
    //delay();

    mUI->pushButtonApply->setEnabled(true);
    mPerspectiveSuccess = true;
}

void CalibrationWindow::closeWindow()
{
    mCalib.mIndex = 0;
    mCancel = true;
    this->close();
}

bool CalibrationWindow::cancel() const
{
    return mCancel;
}

void CalibrationWindow::on_pushButtonCancel_clicked()
{
    mCancel = true;
}

void CalibrationWindow::on_pushButton_skipCalib_clicked()
{
    mCalib.saveCurrentUndistort();
    mSkipCalibration = true;
}

void CalibrationWindow::on_pushButtonApply_clicked()
{
    mCalib.saveNewPerspectiveMatrix();
    mCalib.saveNewCropRect();

    closeWindow();
}
