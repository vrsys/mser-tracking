#ifndef CAPTUREDEVICE_H
#define CAPTUREDEVICE_H

#include <opencv2/opencv.hpp>

class CaptureDevice
{
public:
    virtual const cv::Mat& capture() = 0;
    virtual void pause() = 0;
    virtual void resume() = 0;
    virtual void next() = 0;
    virtual bool getStatus() const = 0;
    virtual void disconnect() = 0;
};

#endif // CAPTUREDEVICE_H
