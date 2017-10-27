#ifndef CAMERADEVICE_H
#define CAMERADEVICE_H

#include <iostream>

#include <opencv2/opencv.hpp>

#include "capturedevice.hpp"

class CameraDevice : public CaptureDevice {

    public:
        CameraDevice();

        /*virtual*/ const cv::Mat &capture();
        /*virtual*/ void pause();
        /*virtual*/ void resume();
        /*virtual*/ void next();
        /*virtual*/ bool getStatus() const;
        /*virtual*/ void disconnect();

        void setVideoPath(const std::string &path);

    private:
        cv::Size                 mImageSize;
        cv::Mat                  mImage;

        unsigned int             mIndex;
        bool                     mPause;

        cv::VideoCapture         mCam;
};

#endif // CAMERADEVICE_H
