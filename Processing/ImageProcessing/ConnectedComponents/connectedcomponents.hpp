#ifndef CONNECTEDCOMPONENTS_H
#define CONNECTEDCOMPONENTS_H

#include <iostream>

#include <opencv2/opencv.hpp>

#include "handmodel.hpp"

namespace CC {
    struct Component {
        Component(cv::Mat const& img, cv::Rect const& rect)
            :img_(img),
             pos_(rect.x, rect.y),
             size_(rect.size()),
             area_(rect.area()),
             center_(rect.x + rect.width/2, rect.y + rect.height/2)
        {
        }
        Component(cv::Mat const& img, cv::Mat const& stats, cv::Mat const& center)
            :img_(img),
             pos_(stats.at<int>(0, cv::CC_STAT_LEFT), stats.at<int>(0, cv::CC_STAT_TOP)),
             size_(stats.at<int>(0, cv::CC_STAT_HEIGHT), stats.at<int>(0, cv::CC_STAT_WIDTH)),
             area_(stats.at<int>(0, cv::CC_STAT_AREA)),
             center_(center)
        {
            //cv::copyMakeBorder(img_, img_, 50,50,50,50,cv::BORDER_CONSTANT,cv::Scalar(0));
        }
        cv::Mat img_;
        cv::Point2i pos_;
        cv::Size size_;
        float area_;
        cv::Point2i center_;
    };
}

class ConnectedComponents
{
public:
    ConnectedComponents();
    const std::vector<CC::Component> process(cv::Mat const& img, cv::Mat& binaryImg);

    void setThreshold(float threshold);

private:
    cv::Mat mCC;
    float mThreshold;
};

#endif // CONNECTEDCOMPONENTS_H
