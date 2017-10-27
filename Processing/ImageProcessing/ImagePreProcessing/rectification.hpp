#ifndef RECTIFICATION_H
#define RECTIFICATION_H

#include <iostream>

#include <opencv2/opencv.hpp>

class Rectification
{
public:
    Rectification(const std::string& path);
    Rectification(cv::Mat const& image, const std::string& path);
    cv::Mat process(cv::Mat const& image);

    void setResolution(const cv::Size& resolution);

private:
    void loadCalibrationData();


    cv::Size   mResolution;

    cv::Mat    mImage;

    cv::Mat    mMapX;
    cv::Mat    mMapY;

    cv::Mat    mIntrinsic;
    cv::Mat    mDistortion;
    cv::Mat    mPerspectiveCorrection;

    std::string mPath;
    bool mLoadedSuccess;
};

#endif // RECTIFICATION_H
