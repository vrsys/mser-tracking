#ifndef VIDEOFILE_H
#define VIDEOFILE_H

#include <iostream>

#include <opencv2/opencv.hpp>

#include "capturedevice.hpp"

//ToDo delete:
#include "global.hpp"

class VideoFile : public CaptureDevice {

    public:
        VideoFile(std::string path);

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
        std::string              mVideoPath;
        bool                     mPause;

        cv::VideoCapture         mVideo;
};

#endif // VIDEOFILE_H
