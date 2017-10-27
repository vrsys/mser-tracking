#ifndef BACKGROUNDSUBSTRACTION_H
#define BACKGROUNDSUBSTRACTION_H

#include <iostream>

#include <opencv2/opencv.hpp>

#include "Tools/tools.hpp"

class BackgroundSubstraction
{
public:
    BackgroundSubstraction();
    const cv::Mat& process(cv::Mat const& img);

    void setBackgroundImage(const cv::Mat& img);
    void setMaxImage(const cv::Mat& img);
    void setMaxImage(const std::string& imagePath);

    const cv::Mat& getBackgroundImage() const;
    const cv::Mat& getMaxImage() const;

    void loadMinMax(std::string const& path = "");
private:

    void loadMin(const std::string& path);
    void loadMax(const std::string& path);
    void initMin();
    void initMax();

    std::string mPath;

    cv::Mat mBackgroundImage;
    cv::Mat mMaxImage;
    cv::Mat mMaxInitImage;

    cv::Mat mImage;

    bool mSuccessMin;
    bool mSuccessMax;

    std::vector<uchar*> mImagePtr;
    std::vector<uchar*> mBackgroundPtr;
    std::vector<uchar*> mMaxPtr;
    std::vector<uchar*> mMaxInitPtr;

    //cv::Ptr<cv::BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
};

#endif // BACKGROUNDSUBSTRACTION_H
