#include "imagefolder.hpp"

ImageFolder::ImageFolder(std::string imagePath)
    : mIndex(0),
      mImagePath(imagePath),
      mPause(false)
{
    if(mImagePath.back() != '/'){
        mImagePath = mImagePath + "/";
    }

    //load file names
    getFileNames(mImagePath, mImageNames);
}

const cv::Mat& ImageFolder::capture()
{
    if (mImagePath == "" || mImageNames.size() == 0) return mImage;

    if (mIndex >= mImageNames.size()){
        mIndex = 0;
    }

    std::string fileName = mImagePath + mImageNames[mIndex];

    Global::FILENAME = mImageNames[mIndex];

    mImage = cv::imread(fileName, 0);
    if (!mPause){
        ++mIndex;
    }

    return mImage;
}

void ImageFolder::pause()
{
    mPause = true;
}

void ImageFolder::resume()
{
    mPause = false;
}

void ImageFolder::next()
{
    ++mIndex;
}

bool ImageFolder::getStatus() const
{
    if (mImage.empty()) return false;
    return true;
}

void ImageFolder::disconnect()
{
    mImageNames.clear();
    mImage.release();
    mIndex = 0;
    mImagePath = "";
}

std::string ImageFolder::getImagePath() const
{
    return mImagePath;
}

void ImageFolder::setImagePath(const std::string &value)
{
    mImagePath = value;

    if(mImagePath.back() != '/'){
        mImagePath = mImagePath + "/";
    }

    mImageNames.clear();
    //load file names
    getFileNames(mImagePath, mImageNames);
    mIndex = 0;
}

