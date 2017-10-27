#include "imageprocessing.hpp"


ImageProcessing::ImageProcessing()
    : mValidCaptureDevice(false),
      mPreProcesser(),
      mBS(),
      mCC(),
      mMSER(),
      mMSERCluster(),
      mHandTracker(),
      mClassificator(),
      mTUIOSender("localhost", 3333, cv::Size(0,0)),
      mIndex(0),
      mParallelismEnabled(0),
      mSpeedMode(false),
      mSaveMode(false)
{
    HandModel::initHandModel("handmodel");
}

void ImageProcessing::process()
{

    mImages.clear();
    if(!mValidCaptureDevice){
        return;
    }

    //***************************************************************//
    // Capture Image
    //***************************************************************//
    mRawImage = mCam->capture(); // 0ms


    //***************************************************************//
    // Calibrate Camera Image
    //***************************************************************//
    if (typeid(*mCam) == typeid(GrasshopperCamera)){
        // pre process raw input data
        mImage = mPreProcesser.process(mRawImage); // 3ms
        //mask part of image
        cv::circle(mImage, cv::Point (400,100), 220, cv::Scalar(0,0,0),-1);    
        //---
    } else {
        mImage = mRawImage;
    }


    //***************************************************************//
    // Save images in save mode and return
    //***************************************************************//
    if (mSaveMode){
        mImages.push_back(mImage);
        return;
    }


    //***************************************************************//
    // Background substraction (3ms - 10ms)
    //***************************************************************//
    mBSImage = mBS.process(mImage);


    //***************************************************************//
    // connected components (3ms - 4ms)
    //***************************************************************//
    std::vector<CC::Component> components = mCC.process(mBSImage, mCCImage);


    //***************************************************************//
    // reset every 100 frames background if no object is around
    //***************************************************************//
    
    if(mIndex % 100 ==  0 && typeid(*mCam) != typeid(VideoFile) && cv::countNonZero(mCCImage) == 0){
        mBS.setBackgroundImage(mImage);
    }
    mMSERImage = mBSImage;
    

    //***************************************************************//
    // Init Color Images
    //***************************************************************//
    if(!mSpeedMode){
        mImageVis = mImage;
        cv::cvtColor(mImageVis, mImageVis, CV_GRAY2RGB);
        cv::cvtColor(mMSERImage, mMSERImage, CV_GRAY2RGB);
        cv::cvtColor(mBSImage, mBSImage, CV_GRAY2RGB);
        cv::cvtColor(mCCImage, mCCImage, CV_GRAY2RGB);
    }


    //***************************************************************//
    // Do MSER on each CC and find cluster (ms)
    //***************************************************************//
    std::vector<std::shared_ptr<MSERCluster> > clusters;
    #pragma omp parallel for if(mParallelismEnabled)
    for (unsigned int i = 0; i < components.size(); ++i){
        // Inverted Image
        std::shared_ptr<MSER::Region> root = nullptr;
        if(mParallelismEnabled){
            cv::Mat inverted;
            cv::subtract(cv::Scalar::all(255), components[i].img_, inverted);
            MSER mser;
            root = mser.getUnstableRoot(inverted);
        } else {
            cv::subtract(cv::Scalar::all(255), components[i].img_, mInvertedImage);
            root = mMSER.getUnstableRoot(mInvertedImage);
        }

        if (root){
            std::vector<std::shared_ptr<MSERCluster>> clustered = mMSERCluster.process(root);
            if(!mSpeedMode){
                drawComponentTree(mCCImage, root, components[i].pos_, cv::Scalar(255,0,0));
            }

            for (auto& c : clustered) {
                c = c + components[i].pos_;
            }

            if (mParallelismEnabled){
                for(const std::shared_ptr<MSERCluster> c : clustered){
                    clusters.push_back(c);
                }
            } else {
                clusters.insert(clusters.end(), clustered.begin(), clustered.end());
            }
        }
    }


    //***************************************************************//
    // Track current cluster
    //***************************************************************//
    mHands = mHandTracker.process(clusters);
    mHands.push_back(mHandTracker.undefinedHand()); // undefined Finger (without a current hand)


    //***************************************************************//
    // classificate hands
    //***************************************************************//
    mClassificator.process(mHands); // 0ms


    //***************************************************************//
    // send hand and finger informations over TUIO
    //***************************************************************//
    mTUIOSender.send(mHands);


    //***************************************************************//
    // Draw cluster and hands and return for GUI visualisation
    //***************************************************************//
    if(!mSpeedMode){
        for(auto c : clusters){
            drawCluster(mMSERImage, c, cv::Scalar(255, 0, 255));
        }

        drawHands(mImageVis, mHands, cv::Scalar(0,255,0));
        mImages.push_back(mImageVis);
        mImages.push_back(mMSERImage);
        mImages.push_back(mCCImage);
    }


    Global::NEXT = false;

    ++mIndex;
}

void ImageProcessing::showRawCam()
{
    if (!mCam->getStatus()) return;
    while(true){
        // capture image
        mRawImage = mCam->capture();
        cv::imshow("rawImage", mRawImage);
        cv::waitKey(0);
    }
}

void ImageProcessing::showCam()
{
    while(true){
        // capture image
        mRawImage = mCam->capture();

        if (typeid(*mCam) != typeid(ImageFolder)){
            // pre process raw input data
            mImage = mPreProcesser.process(mRawImage);
        } else {
            mImage = mRawImage;
        }

        cv::imshow("rawImage", mImage);
        cv::waitKey(1);
    }
}

void ImageProcessing::setBackgroundImage()
{
    mBS.setBackgroundImage(mImage);
    process();
}

void ImageProcessing::setMaxImage()
{
    mBS.setMaxImage(mImage);
    process();
}

void ImageProcessing::setMaxImage(const cv::Mat& img)
{
    mBS.setMaxImage(img);
    process();
}

bool ImageProcessing::setCamDevice(CAM::DEVICEFLAG flag = CAM::GRASSHOPPER)
{
    mValidCaptureDevice = false;
    if (flag == CAM::GRASSHOPPER){
        mCam = std::shared_ptr<CaptureDevice>(std::make_shared<GrasshopperCamera> ());
        mBS.loadMinMax();
    } else if (flag == CAM::OTHER){
        mCam = std::shared_ptr<CaptureDevice>(std::make_shared<CameraDevice> ());
        mBS.loadMinMax();
    } else {
        return false;
    }

    if (!mCam->getStatus()){
        return false;
    }

    // capture first image
    mRawImage = mCam->capture();

    if (mRawImage.empty()){
        return false;
    }

    if (flag == CAM::GRASSHOPPER){
        // pre process raw input data
        mPreProcesser.setResolution(cv::Size(mRawImage.cols, mRawImage.rows));
        mImage = mPreProcesser.process(mRawImage);
    } else {
        mImage = mRawImage;
    }

    // get resolution and define stats..
    mResolution = cv::Size(mImage.cols, mImage.rows);

    Config config("config");

    bool success = true;
    int touchSizeX = atoi(config.getValue("TouchSizeX", success).c_str());
    int touchSizeY = atoi(config.getValue("TouchSizeY", success).c_str());

    if (!config.successLoad() || !success){
        throw std::runtime_error("Please specify config file with TouchSizeX and TouchSizeY parameter and store it to Data.");
    }
    cv::Size touchSize(touchSizeX, touchSizeY);
    mTUIOSender.setResolution(mResolution);
    HandModel::defineResolution(mResolution, touchSize);
    //HandModel::print();

    mValidCaptureDevice = true;
    return true;
}

bool ImageProcessing::setImageFolder(const std::string& path)
{
    mValidCaptureDevice = false;
    mCam = std::shared_ptr<CaptureDevice>(std::make_shared<ImageFolder>(path));
    mBS.loadMinMax(path+"/Calibration/");

    // capture first image
    mRawImage = mCam->capture();
    mImage = mRawImage;

    if (mRawImage.empty()){
        return false;
    }

    // get resolution and define stats..
    mResolution = cv::Size(mImage.cols, mImage.rows);

    Config config("config");
    bool success = true;
    int touchSizeX = atoi(config.getValue("TouchSizeX", success).c_str());
    int touchSizeY = atoi(config.getValue("TouchSizeY", success).c_str());

    if (!config.successLoad() || !success){
        throw std::runtime_error("Please specify config file with TouchSizeX and TouchSizeY parameter and store it to Data.");
    }

    cv::Size touchSize(touchSizeX, touchSizeY);

    HandModel::defineResolution(mResolution, touchSize);
    mPreProcesser.setResolution(mResolution);
    mTUIOSender.setResolution(mResolution);

    //HandModel::print();


    mValidCaptureDevice = true;
    return true;
}

bool ImageProcessing::setVideoFile(std::string path)
{
    mValidCaptureDevice = false;
    mCam = std::shared_ptr<CaptureDevice>(std::make_shared<VideoFile>(path));

    while(path.back() != '/'){
        path.pop_back();
    }

    mBS.loadMinMax(path+"/Calibration/");

    // capture first image
    mRawImage = mCam->capture();
    mImage = mRawImage;

    if (mRawImage.empty()){
        return false;
    }

    // get resolution and define stats..
    mResolution = cv::Size(mImage.cols, mImage.rows);

    Config config(path + "config");
    bool success = true;
    int touchSizeX = atoi(config.getValue("TouchSizeX", success).c_str());
    int touchSizeY = atoi(config.getValue("TouchSizeY", success).c_str());

    if (!config.successLoad() || !success){
        throw std::runtime_error("Please specify config file with TouchSizeX and TouchSizeY parameter and store it to Data.");
    }

    cv::Size touchSize(touchSizeX, touchSizeY);

    HandModel::defineResolution(mResolution, touchSize);
    mPreProcesser.setResolution(mResolution);
    mTUIOSender.setResolution(mResolution);

    mValidCaptureDevice = true;
    return true;
}

void ImageProcessing::disconnectCamDevice()
{
    if(!mValidCaptureDevice) return;
    mCam->disconnect();
    mCam = nullptr;
}

void ImageProcessing::setClassificationParameter(int minParents, int maxSizeDiff, int minLevel, int maxHandSizeDiff)
{
    mMSERCluster.setParameter(minParents, maxSizeDiff, minLevel, maxHandSizeDiff);
}

void ImageProcessing::setCCParameter(int threshold)
{
    mCC.setThreshold(threshold);
}

void ImageProcessing::pause()
{
    mCam->pause();
}

void ImageProcessing::resume()
{
    mCam->resume();
}

void ImageProcessing::next()
{
    mCam->next();
}

const cv::Mat& ImageProcessing::captureNextImage()
{
    // capture image
    mRawImage = mCam->capture();

    if (typeid(*mCam) != typeid(ImageFolder)){
        // pre process raw input data
        mImage = mPreProcesser.process(mRawImage);
    } else {
        mImage = mRawImage;
    }

    return mImage;
}

const cv::Mat&ImageProcessing:: captureNextRawImage()
{
    // capture image
    mRawImage = mCam->capture();

    return mRawImage;
}

void ImageProcessing::setSpeedMode(bool checked)
{
    mSpeedMode = checked;
}

const cv::Mat &ImageProcessing::getBackgroundImage() const
{
    return mBS.getBackgroundImage();
}

const cv::Mat &ImageProcessing::getMaxImage() const
{
    return mBS.getMaxImage();
}

const std::vector<cv::Mat>& ImageProcessing::getImages() const
{
    return mImages;
}
const std::vector<std::shared_ptr<Hand> >& ImageProcessing::getHands() const
{
    return mHands;
}

void ImageProcessing::setSaveMode(bool saveMode)
{
    mSaveMode = saveMode;
}

cv::Mat ImageProcessing::getImage() const
{
    return mImage;
}

void ImageProcessing::setHost(const std::string host)
{
    mTUIOSender.setHost(host);
}

void ImageProcessing::setParallelismEnabled(int i)
{
    mParallelismEnabled = i;
}



