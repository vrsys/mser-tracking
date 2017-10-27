#include "cameradevice.h"

CameraDevice::CameraDevice()
    : mIndex(0),
      mPause(false)
{
    // open cam
    mCam.open(0);
    //mCam.read(mImage);
}

const cv::Mat& CameraDevice::capture()
{
    if (!getStatus() ) return mImage;

    if (!mPause){
        bool success = mCam.read(mImage);

        if(!success) {
            return mImage;
        }
        cv::cvtColor(mImage, mImage, CV_RGB2GRAY);

        ++mIndex;
    }
    return mImage;
}

void CameraDevice::pause()
{
    mPause = true;
}

void CameraDevice::resume()
{
    mPause = false;
}

void CameraDevice::next()
{
    mCam.read(mImage);
    ++mIndex;
}

bool CameraDevice::getStatus() const
{
    return mCam.isOpened();
}

void CameraDevice::disconnect()
{
    mCam.release();
    mImage.release();
}

