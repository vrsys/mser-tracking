#ifndef HAND_H
#define HAND_H

#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "blob.hpp"
#include "Clustering/mserclustering.hpp"

typedef enum {
    HAND_UNKNOWN = 0,
    HAND_LEFT = 1,
    HAND_RIGHT = 2
} HandClass;

struct Hand
{
    Hand()
        :mID(-1), mPosition(-100,-100), mFingerCenter(mPosition), mFilteredPosition(mPosition),
          mPalmPos(mPosition), mArmEllipse(), mArmDirection(0,0), mUpdateCount(0),
          mClass(HAND_UNKNOWN), mClassLeftCounter(0), mClassRightCounter(0), mBlobs(0),
          mMissCounter(0), mTracked(false), mConfident(true), mLastClusterConfident(true)
    {
        mBoundingBox = cv::Rect(cv::Point2i(-20,-20), cv::Point2i(-20,-20));

        // 1 € filter
        mXFilter.initFilterParameter(31, 1, 0.007, 1);
        mYFilter.initFilterParameter(31, 1, 0.007, 1);
    }

    Hand(const std::shared_ptr<MSERCluster> cluster)
        :mID(-1), mPosition(cluster->mPosition), mFingerCenter(mPosition), mFilteredPosition(mPosition),
          mPalmPos(cluster->mHand.mPosition), mArmEllipse(cluster->mArm), mArmDirection(0,0), mUpdateCount(0),
          mClass(HAND_UNKNOWN), mClassLeftCounter(0), mClassRightCounter(0), mBlobs(0),
          mMissCounter(0), mTracked(true), mConfident(true), mLastClusterConfident(true)
    {
        mBoundingBox = cv::Rect(cv::Point2i(-20,-20), cv::Point2i(-20,-20));

        // 1 € filter
        mXFilter.initFilterParameter(31, 1, 0.007, 1);
        mYFilter.initFilterParameter(31, 1, 0.007, 1);
    }

    int                     mID;
    cv::Point2i             mPosition;
    cv::Point2i             mFingerCenter;
    cv::Point2i             mFilteredPosition;
    cv::Point2i             mPalmPos;
    cv::Rect                mBoundingBox;
    Ellipse                 mArmEllipse;
    cv::Vec2f               mArmDirection;
    int                     mUpdateCount;
    HandClass               mClass;
    int                     mClassLeftCounter;
    int                     mClassRightCounter;
    std::vector<std::shared_ptr<Blob> >       mBlobs;
    int                     mMissCounter;

    OneEuroFilter           mXFilter;
    OneEuroFilter           mYFilter;

    bool                    mTracked;
    bool                    mConfident;
    bool                    mLastClusterConfident;

    friend bool operator == (const std::shared_ptr<Hand>& h1, const std::shared_ptr<Hand>& h2);
};

inline bool operator==(const std::shared_ptr<Hand>& h1, const std::shared_ptr<Hand>& h2) {
    if (h1->mPosition == h2->mPosition) return true;
    else return false;
}

#endif // HAND_H
