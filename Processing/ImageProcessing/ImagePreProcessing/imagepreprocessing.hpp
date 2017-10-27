#ifndef IMAGEPREPROCESSING_H
#define IMAGEPREPROCESSING_H

#include <opencv2/opencv.hpp>

#include "Processing/ImageProcessing/ImagePreProcessing/backgroundsubstraction.hpp"
#include "Processing/ImageProcessing/ConnectedComponents/connectedcomponents.hpp"
#include "Processing/ImageProcessing/ImagePreProcessing/rectification.hpp"

class ImagePreProcessing
{
public:
    ImagePreProcessing();
    const cv::Mat &process(const cv::Mat& image);

    void setResolution(const cv::Size& resolution);

private:
    cv::Mat mImage;
    cv::Size mOldResolution;
    cv::Size mNewResolution;
    cv::Size mResolution;

    cv::Rect mCropRect;

    std::string mPath;
    Rectification mRectification;
};

#endif // IMAGEPREPROCESSING_H
