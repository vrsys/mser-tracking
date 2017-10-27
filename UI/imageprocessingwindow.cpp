#include "imageprocessingwindow.hpp"
#include "ui_imageprocessingwindow.h"

ImageProcessingWindow::ImageProcessingWindow(QWidget *parent) :
    QMainWindow(parent),
    mUI(new Ui::ImageProcessingWindow),
    mRootPathDefault(""),
    mRootPath(mRootPathDefault),
    mMatImages(6),
    mProc(),
    mLoopTimer(0),
    mSaveIndex(0),
    mEllapsedPerSecond(0),
    mFrame(0),
    mLastFrame(0),
    mFPS(0),
    mPause(false),
    mCamSuccess(false),
    mSaveImages(false),
    mSpeedMode(false),
    mVideoMode(false),
    mImageMode(false)
{
    setFocusPolicy(Qt::StrongFocus);

    mUI->setupUi(this);
    mQImagesLabels.push_back(mUI->labelImage1);
    mQImagesLabels.push_back(mUI->labelImage2);
    mQImagesLabels.push_back(mUI->labelImage3);
    mQImagesLabels.push_back(mUI->labelImage4);
    mQImagesLabels.push_back(mUI->labelImage5);


    //***************************************************************//
    // Instantiate timer for GUI windows and associate to each process
    //***************************************************************//
    mQTimer = new QTimer(this);
    connect(mQTimer, SIGNAL(timeout()), this, SLOT(process()));
    mQTimer->start(mLoopTimer);

    mQTimerCreateMax = new QTimer(this);
    connect(mQTimerCreateMax, SIGNAL(timeout()), this, SLOT(processCreateMax()));

    mQTimerCalib = new QTimer(this);
    connect(mQTimerCalib, SIGNAL(timeout()), this, SLOT(processCalib()));


    //***************************************************************//
    // prepare GUI visualisation
    //***************************************************************//
    setImage(mProc.getBackgroundImage(), mUI->labelImage4);
    setImage(mProc.getMaxImage(), mUI->labelImage5);
    mUI->menuBar->setNativeMenuBar(false);


    //***************************************************************//
    // prepare GUI settings
    //***************************************************************//
    mTUIOHost = mUI->textTUIOHost->text().toStdString();
    mProc.setCCParameter(mUI->spinBoxCCThreshold->value());
    loadConfigParamter();
    HandModel::setCurrentFPS(30);

    //***************************************************************//
    // choose start device
    //***************************************************************//
#if 0
    //setImageFolder("Data/Images/Andre3");
    //setImageFolder("Data/Images/handsTogether");
    //setImageFolder("Data/Images/specialCase4");
    //setImageFolder("Data/Images/herausziehen");
    setImageFolder("Data/Images/lastImagestream2");
#else
    //setVideoFile("Data/Videos/SUR40/sur40.webm");
    setCameraDevice(CAM::GRASSHOPPER);
#endif

    //***************************************************************//
    // start process
    //***************************************************************//
    mElapsedTimer.start();
}

ImageProcessingWindow::~ImageProcessingWindow()
{
    delete mUI;
}

void ImageProcessingWindow::process()
{
    ++mFrame;
    if(!mCamSuccess) return;

    //***************************************************************//
    // process
    //***************************************************************//
    mProc.process();


    //***************************************************************//
    // update each second FPS and update background image visualisation
    //***************************************************************//
    int ellapsedMs =  mElapsedTimer.restart();
    mEllapsedPerSecond += ellapsedMs;
    if (mEllapsedPerSecond > 1000.0){
        mEllapsedPerSecond = 0;
        mFPS = mFrame-mLastFrame;
        mLastFrame = mFrame;
        if (!mPause && !mVideoMode && !mImageMode){
            HandModel::setCurrentFPS(mFPS);
        }
        if(!mSpeedMode && mHands.size() == 0) {
            setImage(mProc.getBackgroundImage(), mUI->labelImage4);
        }
    }


    //***************************************************************//
    // update visualisation
    //***************************************************************//
    if(!mSpeedMode){
        // draw images
        mMatImages = mProc.getImages();
        drawImages();
    }

    // write log
    mHands = mProc.getHands();
    writeLog();

    // update Filename
    if (mImageMode) {
        mUI->labelImageName->setText(QString::fromStdString(Global::FILENAME));
    } else {
        mUI->labelImageName->setText("");
    }
    mUI->labelFPS->setText(QString::fromStdString(std::to_string(ellapsedMs) + "ms  - " + std::to_string(mFPS)+" FPS"));


    // save images in save mode
    if (mSaveImages && !mPause){
        saveImages();
    }

}

void ImageProcessingWindow::processCreateMax()
{
    mCreateMaxWindow.process(mProc.captureNextImage());
    mCreateMaxWindow.show();

    if (mCreateMaxWindow.getCanceled()){
        // cancel
        mCreateMaxWindow.closeWindow();
        mQTimerCreateMax->stop();
        mQTimer->start();
    } else {
        if (mCreateMaxWindow.success()){
            mCreateMaxWindow.closeWindow();

            mProc.setMaxImage(mCreateMaxWindow.getMaxImage());
            setImage(mProc.getMaxImage(), mUI->labelImage5);

            mQTimerCreateMax->stop();
            mQTimer->start();
        }
    }
}

void ImageProcessingWindow::processCalib()
{
    mCalibWindow.process(mProc.captureNextRawImage());
    mCalibWindow.show();

    if (mCalibWindow.cancel()){
        mCalibWindow.closeWindow();
        mQTimerCalib->stop();
        mQTimer->start();
    }
}

void ImageProcessingWindow::on_btnPauseResume_clicked()
{
    if(mPause){
        mQTimer->start();
        mPause = false;
//        mProc.resume();
        Global::PAUSE = false;
        mUI->btnPauseResume->setText("&pause");
    } else{
        mQTimer->stop();
        mPause = true;
//        mProc.pause();
        Global::PAUSE = true;
        mUI->btnPauseResume->setText("&resume");
    }
}


void ImageProcessingWindow::on_buttonCaptureMin_clicked()
{
    mProc.setBackgroundImage();
    setImage(mProc.getBackgroundImage(), mUI->labelImage4);
}

void ImageProcessingWindow::on_buttonCaptureMax_clicked()
{
    mProc.setMaxImage();
    setImage(mProc.getMaxImage(), mUI->labelImage5);
}


void ImageProcessingWindow::on_buttonCreateMax_clicked()
{
    //stop process timer
    mQTimer->stop();

    mCreateMaxWindow.initImage(mProc.captureNextImage());

    mQTimerCreateMax->start();
}




void ImageProcessingWindow::setImage(const cv::Mat &img, QLabel* label)
{
    // draw
    label->setPixmap(QPixmap::fromImage(matToQImage(img)));
}

void ImageProcessingWindow::saveImages()
{
    std::string number;
    if (10000 > mSaveIndex){
        number = std::to_string(mSaveIndex);
    }
    if (1000 > mSaveIndex ){
        number = "0" + std::to_string(mSaveIndex);
    }
    if (100 > mSaveIndex ){
        number = "00" + std::to_string(mSaveIndex);
    }
    if (10 > mSaveIndex ){
        number = "000" + std::to_string(mSaveIndex);
    }

    std::string fileName = "frame"+number+".jpg";

    cv::imwrite(mSaveImagesPath + fileName, mProc.getImage());
    ++mSaveIndex;
}

void ImageProcessingWindow::setImageFolder(std::string s)
{
    mProc.disconnectCamDevice();
    mCamSuccess = mProc.setImageFolder(s);

    if (!mCamSuccess) {									// if unable to open image
        mUI->textField->setText("error: images could not read from folder");      // update lable with error message
        // resume application
        mQTimer->start(mLoopTimer);
        return;                                                             // and exit function
    }

    mRootPath = s;
    HandModel::setCurrentFPS(60);

    if(!loadConfigParamter()) {
        std::cout << "could not load config " << s << std::endl;
        mRootPath = mRootPathDefault;
        loadConfigParamter();
    }

    mUI->actionGrasshopper->setChecked(false);
    mUI->actionOtherCam->setChecked(false);
    mUI->actionVideo_File->setChecked(false);
    mUI->actionImage_Stream->setChecked(true);

    // if we get to this point image was opened successfully
    mUI->textField->setText(QString::fromStdString("Image path set to: " + s));                // update label with file name
    mUI->btnSaveImages->setEnabled(false);
    mUI->buttonCaptureMax->setEnabled(false);
    mUI->buttonCreateMax->setEnabled(false);

    setImage(mProc.getBackgroundImage(), mUI->labelImage4);
    setImage(mProc.getMaxImage(), mUI->labelImage5);

    mVideoMode = false;
    mImageMode = true;
}

void ImageProcessingWindow::setVideoFile(std::string const& s)
{
    mProc.disconnectCamDevice();
    mCamSuccess = mProc.setVideoFile(s);

    if (!mCamSuccess) {									// if unable to open image
        mUI->textField->setText("error: images could not read from folder");      // update lable with error message
        // resume application
        mQTimer->start(mLoopTimer);
        return;                                                             // and exit function
    }

    std::string videoPath = s;
    while(videoPath.back() != '/'){
        videoPath.pop_back();
    }
    mRootPath = videoPath;
    HandModel::setCurrentFPS(24);

    if(!loadConfigParamter()) {
        mRootPath = mRootPathDefault;
        loadConfigParamter();
    }

    mUI->actionImage_Stream->setChecked(false);
    mUI->actionGrasshopper->setChecked(false);
    mUI->actionOtherCam->setChecked(false);
    mUI->actionVideo_File->setChecked(true);

    // if we get to this point image was opened successfully
    mUI->textField->setText(QString::fromStdString("Video file: " + s));                // update label with file name
    mUI->btnSaveImages->setEnabled(false);
    mUI->buttonCaptureMax->setEnabled(false);
    mUI->buttonCreateMax->setEnabled(false);

    setImage(mProc.getBackgroundImage(), mUI->labelImage4);
    setImage(mProc.getMaxImage(), mUI->labelImage5);

    mVideoMode = true;
    mImageMode = false;
}

void ImageProcessingWindow::setCameraDevice(CAM::DEVICEFLAG flag)
{
    //stop process timer
    mQTimer->stop();
    mProc.disconnectCamDevice();
    mCamSuccess = mProc.setCamDevice(flag);

    mUI->labelImageName->setText("");

    mUI->actionImage_Stream->setChecked(false);
    mUI->actionGrasshopper->setChecked(false);
    mUI->actionVideo_File->setChecked(false);
    mUI->actionOtherCam->setChecked(false);

    if (!mCamSuccess) {									// if unable to open image
        mUI->textField->setText("error: couldn't access cam.");      // update lable with error message
        // resume application
        mQTimer->start(mLoopTimer);
        return;                                                             // and exit function
    }

    switch (flag) {
        case CAM::GRASSHOPPER: mUI->actionGrasshopper->setChecked(true);
        case CAM::OTHER: mUI->actionOtherCam->setChecked(true);
    }

    mRootPath = mRootPathDefault;
    loadConfigParamter();

    // camera was opened successfully
    mUI->textField->setText(QString::fromStdString("Successfully init camera device: "));

    mUI->btnSaveImages->setEnabled(true);
    mUI->buttonCaptureMax->setEnabled(true);
    mUI->buttonCreateMax->setEnabled(true);

    mVideoMode = false;
    mImageMode = false;

    setImage(mProc.getBackgroundImage(), mUI->labelImage4);
    setImage(mProc.getMaxImage(), mUI->labelImage5);

    // resume application
    mQTimer->start(mLoopTimer);
}

void ImageProcessingWindow::checkRootPath()
{
    if(mRootPath.back() != '/') mRootPath += "/";
    if(mRootPath.size() == 1) mRootPath = "";
}

bool ImageProcessingWindow::loadConfigParamter()
{
    checkRootPath();

    Config config(mRootPath+"config");
    if (config.successLoad()) {
        bool success = true;
        int minParents = atoi(config.getValue("F_MinParents", success).c_str());
        if(success) mUI->spinBoxMinParents->setValue(minParents);

        success = true;
        int maxSizeDiff = atoi(config.getValue("F_MaxSizeDiff", success).c_str());
        if(success)mUI->spinBoxSizeDiff->setValue(maxSizeDiff);

        success = true;
        int minLevel = atoi(config.getValue("F_MinLevel", success).c_str());
        if(success)mUI->spinBoxLevel->setValue(minLevel);

        success = true;
        int handMaxSizeDiff = atoi(config.getValue("H_MaxSizeDiff", success).c_str());
        if(success)mUI->spinBoxHandSizeDiff->setValue(handMaxSizeDiff);

        success = true;
        int FPS = atoi(config.getValue("FPS", success).c_str());
        if(success){
            HandModel::setCurrentFPS(FPS);
        } else {
            //default
            HandModel::setCurrentFPS(30);
        }
    } else {
        return false;
    }

    setClassificationParameter();
    return true;
}

void ImageProcessingWindow::saveClassificationParameter()
{
    checkRootPath();

    Config config(mRootPath+"config");
    if (config.successLoad()) {
        config.setValue("H_MaxSizeDiff", std::to_string(mUI->spinBoxHandSizeDiff->value()));
        config.setValue("F_MinLevel", std::to_string(mUI->spinBoxLevel->value()));
        config.setValue("F_MaxSizeDiff", std::to_string((int)mUI->spinBoxSizeDiff->value()));
        config.setValue("F_MinParents", std::to_string(mUI->spinBoxMinParents->value()));
    }
}

void ImageProcessingWindow::writeLog()
{
    QString s = "current hands: ";
    s.append(QString::number(mHands.size()));
    mUI->textField->append(s);

    for(const std::shared_ptr<Hand> h: mHands){
        QString t = "ID: ";
        t.append(QString::number(h->mID));
        t.append("  with ");
        t.append(QString::number(h->mBlobs.size()));
        t.append("  fingers.");

        mUI->textField->append(t);
    }
}

void ImageProcessingWindow::setClassificationParameter()
{
    mProc.setClassificationParameter(mUI->spinBoxMinParents->value(), mUI->spinBoxSizeDiff->value(),
                                     mUI->spinBoxLevel->value(), mUI->spinBoxHandSizeDiff->value());
}

void ImageProcessingWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Space) {
        //std::cout << "space" << std::endl;
        on_btnPauseResume_clicked();
        event->accept();
    }
    if(event->key() == Qt::Key_P){
        //std::cout << "p" << std::endl;
        on_btnPauseResume_clicked();
        event->accept();
    }
    if(event->key() == Qt::Key_N) {
        //std::cout << "n" << std::endl;
        on_btnNextFrame_clicked();
        event->accept();
    }
    if(event->key() == Qt::Key_Escape) {
        //std::cout << "esc" << std::endl;
        saveClassificationParameter();
        this->close();
        event->accept();
    }
}

void ImageProcessingWindow::drawImages()
{
    for(unsigned int i = 0; i < mMatImages.size(); ++i){
        if(i >= mQImagesLabels.size()){ break; }

        // draw
        mQImagesLabels[i]->setPixmap(QPixmap::fromImage(matToQImage(mMatImages[i])));
    }
}

void ImageProcessingWindow::on_btnNextFrame_clicked()
{
    //Global::NEXT = true;
    //mProc.next();
    process();
}

void ImageProcessingWindow::on_spinBoxLoopTimer_valueChanged(int arg1)
{
    mLoopTimer = arg1;
    mQTimer->start(mLoopTimer);
}


void ImageProcessingWindow::on_btnSaveImages_clicked()
{
    //stop process timer
    mQTimer->stop();

    if(mSaveImages){
        mSaveImages = false;
        mProc.setSaveMode(mSaveImages);
        mUI->btnSaveImages->setIcon(QIcon());
        mSaveImagesPath = "";
    } else{
        QString strFolderName = QFileDialog::getExistingDirectory();
        // if file was not chosen
        if(strFolderName == "") {
            mQTimer->start(mLoopTimer);
            return;
        }

        mSaveImages = true;
        mProc.setSaveMode(mSaveImages);
        mSaveIndex = 0;
        mUI->btnSaveImages->setIcon(QIcon("UI/Ressources/Icon/saveImages.png"));

        QDir dir(QDir::currentPath());
        QString relativePath = dir.relativeFilePath(strFolderName);
        mSaveImagesPath = relativePath.toStdString();

        if(mSaveImagesPath.back() != '/'){
            mSaveImagesPath = mSaveImagesPath + "/";
        }

        // create Calibration folder and add min max image
        std::cout << relativePath.toStdString() << "  " << mSaveImagesPath<< std::endl;
        relativePath.append("/Calibration");
        std::cout << relativePath.toStdString() << std::endl;

        QDir().mkdir(relativePath);


        cv::imwrite(mSaveImagesPath+"Calibration/min.jpg", mProc.getBackgroundImage());
        cv::imwrite(mSaveImagesPath+"Calibration/max.jpg", mProc.getMaxImage());
    }

    // resume application
    mQTimer->start(mLoopTimer);
}

void ImageProcessingWindow::on_spinBoxCCThreshold_valueChanged(int threshold)
{
    mProc.setCCParameter(threshold);
}


void ImageProcessingWindow::on_spinBoxMinParents_valueChanged(int)
{
    setClassificationParameter();
}

void ImageProcessingWindow::on_spinBoxSizeDiff_valueChanged(double)
{
    setClassificationParameter();
}

void ImageProcessingWindow::on_spinBoxLevel_valueChanged(int)
{
    setClassificationParameter();
}

void ImageProcessingWindow::on_spinBoxHandSizeDiff_valueChanged(int)
{
    setClassificationParameter();
}


void ImageProcessingWindow::on_checkBoxFastMode_toggled(bool checked)
{
    mSpeedMode = checked;
    mProc.setSpeedMode(checked);

    if(checked) {
        mUI->labelImage1->clear();
        mUI->labelImage2->clear();
        mUI->labelImage3->clear();
        mUI->labelImage4->clear();
        mUI->labelImage5->clear();
        mMatImages.clear();
    } else {
        setImage(mProc.getBackgroundImage(), mUI->labelImage4);
        setImage(mProc.getMaxImage(), mUI->labelImage5);
    }
}

void ImageProcessingWindow::on_actionCalibrate_triggered()
{
    //stop process timer
    mQTimer->stop();
    mCalibWindow.init(mProc.captureNextRawImage());
    mQTimerCalib->start();
}

void ImageProcessingWindow::on_actionImage_Stream_triggered()
{
    //stop process timer
    mQTimer->stop();
    mUI->actionImage_Stream->setChecked(false);


    QString strFolderName = QFileDialog::getExistingDirectory();  // bring up open file dialog

    if(strFolderName == "") {                                // if file was not chosen
        mUI->textField->setText("file not chosen");     // update label
        // resume application
        mQTimer->start(mLoopTimer);
        return;                                            // and exit function
    }
    QDir dir(QDir::currentPath());
    QString relativePath = dir.relativeFilePath(strFolderName);

    setImageFolder(relativePath.toStdString());

    // resume application
    mQTimer->start(mLoopTimer);
}



void ImageProcessingWindow::on_actionVideo_File_triggered()
{
    //stop process timer
    mQTimer->stop();

    QString strFileName = QFileDialog::getOpenFileName();

    if(strFileName == "") {
        mUI->textField->setText("file not chosen");     // update label
        // resume application
        mQTimer->start(mLoopTimer);
        return;                                            // and exit function
    }


    QDir dir(QDir::currentPath());
    QString relativePath = dir.relativeFilePath(strFileName);

    setVideoFile(relativePath.toStdString());

    // resume application
    mQTimer->start(mLoopTimer);
}


void ImageProcessingWindow::on_actionGrasshopper_triggered()
{
    setCameraDevice(CAM::GRASSHOPPER);
}

void ImageProcessingWindow::on_actionOtherCam_triggered()
{
    setCameraDevice(CAM::OTHER);
}


void ImageProcessingWindow::on_textTUIOHost_returnPressed()
{
    mTUIOHost = mUI->textTUIOHost->text().toStdString();
    mProc.setHost(mTUIOHost);
}
