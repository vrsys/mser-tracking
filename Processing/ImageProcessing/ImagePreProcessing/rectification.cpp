#include "rectification.hpp"

Rectification::Rectification(const std::string &path)
    : mPath(path),
      mLoadedSuccess(false)
{
    //loadCalibrationData();
}

Rectification::Rectification(cv::Mat const& image, const std::string& path)
    : mPath(path),
      mLoadedSuccess(false)
{
    setResolution(cv::Size(image.cols, image.rows));
}

cv::Mat Rectification::process(const cv::Mat &image)
{
    if(!mLoadedSuccess) return image;

    // DISTORTION
    cv::remap(image, mImage, mMapX, mMapY, cv::INTER_LINEAR);

    // PERSPECTIVE CORRECTION
    cv::warpPerspective(mImage, mImage, mPerspectiveCorrection, mImage.size());

    return mImage;
}

void Rectification::loadCalibrationData()
{
    // LOAD DATA FOR CALIBRATION
    //Load specific intrinsic and distortion matrices
    cv::FileStorage storage1(mPath + "Intrinsics.xml", cv::FileStorage::READ);
    cv::FileNode fn1 = storage1.getFirstTopLevelNode();
    fn1 >> mIntrinsic;
    storage1.release();
    cv::FileStorage storage2(mPath + "Distortion.xml", cv::FileStorage::READ);
    cv::FileNode fn2 = storage2.getFirstTopLevelNode();
    fn2 >> mDistortion;
    storage2.release();

    if (mIntrinsic.empty() || mDistortion.empty()) {
        throw std::runtime_error("couldn't load calibration matrices. Please provide Intrinsic.xml and Distortion.xml file in " + mPath);
    }

    mMapX = cv::Mat::zeros(mResolution.width, mResolution.height, CV_32F);
    mMapY = cv::Mat::zeros(mResolution.width, mResolution.height, CV_32F);

    cv::Mat R ;
    cv::initUndistortRectifyMap(mIntrinsic, mDistortion, R, mIntrinsic, mResolution, CV_32F, mMapX, mMapY);
    R.release();

    // LOAD DATA FOR PERSPECTIVE CORRECTION
    cv::FileStorage storage3(mPath + "perspectiveCorrection.yml", cv::FileStorage::READ);
    storage3["transmtx"] >> mPerspectiveCorrection;
    storage3.release();
}

void Rectification::setResolution(const cv::Size& resolution)
{
    mResolution = resolution;
    mImage = cv::Mat::zeros(mResolution, CV_8U);

    loadCalibrationData();
    mLoadedSuccess = true;
}
