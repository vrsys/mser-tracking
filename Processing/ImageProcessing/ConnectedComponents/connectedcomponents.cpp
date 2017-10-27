#include "connectedcomponents.hpp"

ConnectedComponents::ConnectedComponents()
    :mThreshold(1)
{
}

const std::vector<CC::Component> ConnectedComponents::process(const cv::Mat& img, cv::Mat& binaryImg)
{
    cv::threshold(img, binaryImg, mThreshold, 255, cv::THRESH_BINARY);

    //cv::Mat connectedComponents(img.size(), CV_8U);
    //imgOut = cv::Mat::zeros(img.size(), img.type());
    cv::Mat stats;
    cv::Mat centroids;
    cv::Mat labels;
    int nLabels = cv::connectedComponentsWithStats(binaryImg, labels, stats, centroids);
    std::vector<CC::Component> components;
    for(int i = 0; i < nLabels; ++i){
        if (i != 0 &&  //ignore root component (background)
            stats.at<int>(i, cv::CC_STAT_AREA) > (2 * HandModel::HandAreaMin) )//&& // ignore too small Components
           //(std::fabs((stats.at<int>(i, cv::CC_STAT_WIDTH) / stats.at<int>(i, cv::CC_STAT_HEIGHT)) -1 ) > 0.2 )) //ignore bad aspect ratio
        {
            cv::Rect rect(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP),stats.at<int>(i,cv::CC_STAT_WIDTH),stats.at<int>(i,cv::CC_STAT_HEIGHT));

            // get current mask
            cv::Mat mask;
            cv::compare(labels, i, mask, cv::CMP_EQ);

            // get current image region
            cv::Mat image = img(rect);

            // crop mask to rect and copy only mask region to image.. (no fingertips from other component..)
            mask = mask(rect);

            cv::Mat maskedImage;
            image.copyTo(maskedImage, mask);

            components.push_back(CC::Component(maskedImage, rect));
        }
    }

    return components;
}

void ConnectedComponents::setThreshold(float threshold)
{
    mThreshold = threshold;
}

