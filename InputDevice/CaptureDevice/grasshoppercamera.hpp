#ifndef GRASSHOPPERCAMERA_H
#define GRASSHOPPERCAMERA_H

#include <iostream>

#include <FlyCapture2.h>
#include <Camera.h>

#include <opencv2/opencv.hpp>

#include "capturedevice.hpp"

class GrasshopperCamera : public CaptureDevice
{
public:
    GrasshopperCamera();
    ~GrasshopperCamera();

    /*virtual*/ const cv::Mat& capture();
    /*virtual*/ void pause();
    /*virtual*/ void resume();
    /*virtual*/ void next();
    /*virtual*/ void disconnect();


    bool getStatus() const;

private:
    void printBuildInfo();
    void printCameraInfo(FlyCapture2::CameraInfo* camInfo);
    void printFormat7Capabilities(FlyCapture2::Format7Info info);
    bool pollForTriggerReady(FlyCapture2::Camera* cam);
    bool checkError(const std::string &prefix, FlyCapture2::Error &error) const;

    void configureCamera();
    void setCameraSettings();
    void setTriggerMode();
    void setFormat7Settings();

    bool mStatusSuccessful;
    bool mPause;

    int                               mCamWidth;
    int                               mCamHeight;

    cv::Mat                           mImage;

    cv::Size                          mImageSize;

    FlyCapture2::Error                mError;
    FlyCapture2::Camera               mCam;
    FlyCapture2::Image                mRawImage;
    FlyCapture2::Image                mMonoImage;
    FlyCapture2::TriggerMode          mTriggerMode;

    FlyCapture2::Mode                 mFmt7Mode;
    FlyCapture2::PixelFormat          mFmt7PixFmt;
    FlyCapture2::Format7Info          mFmt7Info;
    FlyCapture2::Format7ImageSettings mFmt7ImageSettings;
    FlyCapture2::Format7PacketInfo    mFmt7PacketInfo;
};

#endif // GRASSHOPPERCAMERA_H
