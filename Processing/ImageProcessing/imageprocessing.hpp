#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <omp.h>

#include "Processing/ImageProcessing/MSER/mser.hpp"
#include "Clustering/mserclustering.hpp"
#include "Processing/ImageProcessing/ImagePreProcessing/imagepreprocessing.hpp"
#include "InputDevice/CaptureDevice/capturedevice.hpp"
#include "InputDevice/CaptureDevice/grasshoppercamera.hpp"
#include "InputDevice/CaptureDevice/imagefolder.hpp"
#include "InputDevice/CaptureDevice/videofile.h"
#include "InputDevice/CaptureDevice/cameradevice.h"

#include "Tools/visualisation.hpp"
#include "Tools/config.hpp"

#include "Processing/ImageProcessing/Filter/oneeurofilter.hpp"
#include "Processing/ImageProcessing/Filter/doubleexponentialsmoothing.hpp"
#include "Classification/classification.hpp"

#include "Tracking/handtracker.hpp"
#include "Network/tuiosender.hpp"

#include "blob.hpp"
#include "hand.hpp"
#include "handmodel.hpp"


// todo: delete
#include "global.hpp"

namespace CAM {
    enum DEVICEFLAG {
        GRASSHOPPER = 0,
        OTHER = 1
    };
}

class ImageProcessing
{
public:
    ImageProcessing();
    void process();

    void showRawCam();
    void showCam();
    void setBackgroundImage();
    void setMaxImage();
    void setMaxImage(const cv::Mat& img);


    bool setCamDevice(CAM::DEVICEFLAG flag);
    bool setImageFolder(std::string const& path);
    bool setVideoFile(std::string path);

    void disconnectCamDevice();

    void setClassificationParameter(int minParents, int maxSizeDiff, int minLevel, int maxHandSizeDiff);
    void setCCParameter(int threshold);

    void pause();
    void resume();
    void next();

    const cv::Mat& captureNextImage();
    const cv::Mat& captureNextRawImage();

    void setSpeedMode(bool checked);

    const cv::Mat& getBackgroundImage() const;
    const cv::Mat& getMaxImage() const;

    const std::vector<cv::Mat> &getImages() const;
    const std::vector<std::shared_ptr<Hand> > &getHands() const;

    void setSaveMode(bool saveMode);

    cv::Mat getImage() const;

    void setHost(const std::string host);

    void setParallelismEnabled(int i);

private:
    cv::Mat mRawImage;
    cv::Mat mImage;
    cv::Mat mImageVis;
    cv::Mat mBSImage;
    cv::Mat mCCImage;
    cv::Mat mInvertedImage;
    cv::Mat mMSERImage;

    std::vector<cv::Mat> mImages;

    std::shared_ptr<CaptureDevice> mCam;
    bool           mValidCaptureDevice;

    ImagePreProcessing     mPreProcesser;
    BackgroundSubstraction mBS;
    ConnectedComponents    mCC;
    MSER                   mMSER;
    MSERClustering         mMSERCluster;
    HandTracker            mHandTracker;
    Classification         mClassificator;
    TUIOSender             mTUIOSender;

    unsigned int           mIndex;
    int                    mParallelismEnabled;

    bool                   mSpeedMode;
    bool                   mSaveMode;

    cv::Size               mResolution;

    // hands
    std::vector<std::shared_ptr<Hand> > mHands;
};

#endif // IMAGEPROCESSING_H
