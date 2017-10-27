#include "imagepreprocessing.hpp"

ImagePreProcessing::ImagePreProcessing()
    : mPath("Processing/ImageProcessing/ImagePreProcessing/Calibration/"),
      mRectification(mPath)
{
    cv::FileStorage fs(mPath + "cropedRect.yml", cv::FileStorage::READ);
    fs["x"] >> mCropRect.x;
    fs["y"] >> mCropRect.y;
    fs["width"] >> mCropRect.width;
    fs["height"] >> mCropRect.height;
    fs.release();

    // todo: get resolution and init matrixes with right size --> performance?!
}

const cv::Mat& ImagePreProcessing::process(cv::Mat const& image)
{
    // RECTIFICATION
    mImage = mRectification.process(image);

    // CROP
    mImage = mImage(mCropRect);

    // FLIPPING
    cv::flip(mImage, mImage, -1);

    return mImage;
}

void ImagePreProcessing::setResolution(const cv::Size& resolution)
{
    mResolution = resolution;
    mRectification.setResolution(mResolution);
}
