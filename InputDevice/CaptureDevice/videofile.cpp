#include "videofile.h"

VideoFile::VideoFile(std::string path)
    : mIndex(0),
      mVideoPath(path),
      mPause(false)
{
    // load video
    mVideo.open(path);
    mVideo.read(mImage);
}


const cv::Mat& VideoFile::capture()
{
    if (!getStatus() ) return mImage;

    if (!mPause){
        bool success = mVideo.read(mImage);

        if (!success){
            setVideoPath(mVideoPath);
            mVideo.read(mImage);
        }

        cv::cvtColor(mImage, mImage, CV_RGB2GRAY);

        ++mIndex;
    }

    return mImage;
}

void VideoFile::pause()
{
    mPause = true;
}

void VideoFile::resume()
{
    mPause = false;
}

void VideoFile::next()
{
    ++mIndex;
}

bool VideoFile::getStatus() const
{
    return mVideo.isOpened();
}

void VideoFile::disconnect()
{
    mVideo.release();
    mIndex = 0;
    mVideoPath = "";
    mImage.release();
}

void VideoFile::setVideoPath(const std::string& path)
{
    mVideoPath = path;
    mVideo.release();
    mVideo.open(path);
    mIndex = 0;
}
