#ifndef MSERCLUSTERING_H
#define MSERCLUSTERING_H

#include <iostream>
#include <vector>
#include <memory>
#include <utility>

#include <opencv2/opencv.hpp>

#include "Processing/ImageProcessing/MSER/mser.hpp"
#include "blob.hpp"

#include "handmodel.hpp"

struct MSERCluster {
    MSERCluster()
        :mTopLeft(0,0), mPosition(0,0), mArm(), mHand(), mHandSimilarity(1000), mArmSimilarity(1000), mBlobs(), mConfident(true), mLastConfident(true)
    {
    }
    cv::Point2i mTopLeft;
    cv::Point2i mPosition;
    Ellipse mArm;
    Ellipse mHand;
    int mHandSimilarity;
    int mArmSimilarity;
    std::vector<std::shared_ptr<Blob> > mBlobs;

    bool mConfident;
    bool mLastConfident;
    //std::shared_ptr<MSER::Region> mRoot;

    friend std::shared_ptr<MSERCluster> operator+(const std::shared_ptr<MSERCluster> cluster, const cv::Point2i& pos);
};

class MSERClustering
{
public:
    MSERClustering();
    const std::vector<std::shared_ptr<MSERCluster> > process(std::shared_ptr<MSER::Region> root);
    void setParameter(int minParents, int maxSizeDiff, int minLevel, int maxHandSizeDiff);


private:
    void classificateHands(std::shared_ptr<MSER::Region> node, std::vector<std::shared_ptr<MSER::Region> >& handsOut, int counter);
    void classificateArm(const std::shared_ptr<MSERCluster>& cluster, std::shared_ptr<MSER::Region> node, std::shared_ptr<MSER::Region>& arm);
    void classificateFinger(std::shared_ptr<MSER::Region> prev, std::shared_ptr<MSER::Region> node, int similarNodes, std::vector<std::pair<bool, std::shared_ptr<MSER::Region>> >& regions);

    void addBlobs(std::vector<std::pair<bool, std::shared_ptr<MSER::Region>> >& regions, std::shared_ptr<MSERCluster> cluster, std::shared_ptr<MSER::Region> node);

    void initModell();
    int countChilds(const std::shared_ptr<MSER::Region>& node);
    int checkSimilarity(int x, int y);

    int getDistance(const std::shared_ptr<MSER::Region>& reg1, const std::shared_ptr<MSER::Region>& reg2);
    int getDistance(const cv::Point2i& p, const std::shared_ptr<MSER::Region> &reg2);
    int getDistance(const cv::Point2i &p1, const cv::Point2i &p2);

    int computeAngle(const cv::Vec2f& v1, const cv::Vec2f& v2) const;

    bool checkTrustability(std::shared_ptr<MSER::Region> node);

    void checkOutlier(std::vector<std::shared_ptr<MSERCluster> > clusters);
    void checkOutlier2(std::shared_ptr<MSERCluster> cluster);
    void defineCenterPoint(std::shared_ptr<MSERCluster> cluster);

    void checkOutlierHands(std::vector<std::shared_ptr<MSER::Region> >& hands);
    void checkClusterConfidence(std::vector<std::shared_ptr<MSERCluster> > clusters);

    // parameters
    int mMinFingerParents;
    int mMinLevel;
    int mMaxSizeDiff;
    int mMaxHandSizeDiff;
};

#endif // MSERCLUSTERING_H
