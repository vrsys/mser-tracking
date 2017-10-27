#ifndef BLOB_H
#define BLOB_H

#include <memory>

#include <opencv2/opencv.hpp>

#include "Processing/ImageProcessing/MSER/ellipse.hpp"
#include "Processing/ImageProcessing/MSER/mser.hpp"

#include "Processing/ImageProcessing/Filter/oneeurofilter.hpp"
#include "Processing/ImageProcessing/Filter/doubleexponentialsmoothing.hpp"

#include "handmodel.hpp"
struct Hand;

typedef enum {
    FINGER_UNKNOWN = 0,
    FINGER_THUMB,
    FINGER_INDEX,
    FINGER_MIDDLE,
    FINGER_RING,
    FINGER_LITTLE
} FingerClass;


struct Blob
{
    Blob()
        : mEllipse(), mIntensity(0),
          mID(-1), mPosition(0,0), mOrientation(0,0), mPrevPosition(0,0), mFilteredPosition(0,0), mPredictedPosition(0,0),
          mUpdateCount(0), mClass(FINGER_UNKNOWN), mHand(nullptr), mHandIDLast(-1), mMissCounter(0), mTracked(false), mTrackedTemp(false), mConfident(true)
    {
    }

    Blob(cv::Point2i pos)
        : mEllipse(), mIntensity(0),
          mID(-1), mPosition(pos), mOrientation(0,0), mPrevPosition(0,0), mFilteredPosition(0,0), mPredictedPosition(0,0),
          mUpdateCount(0), mClass(FINGER_UNKNOWN), mHand(nullptr), mHandIDLast(-1), mMissCounter(0), mTracked(false), mTrackedTemp(false), mConfident(true)
    {
    }


    Blob(const std::shared_ptr<MSER::Region> reg)
        : mEllipse(reg), mIntensity(reg->level_),
          mID(-1), mPosition(mEllipse.mPosition), mPrevPosition(mPosition), mFilteredPosition(mPosition), mPredictedPosition(mPosition),
          mUpdateCount(0), mClass(FINGER_UNKNOWN), mHand(nullptr), mHandIDLast(-1), mMissCounter(0), mTracked(false), mTrackedTemp(false), mConfident(true)
    {
        // copare to parents...
        // get pos of last (finger) parent
        std::shared_ptr<MSER::Region> current = reg;
        cv::Point2i currentPos(0,0);
        while (current->parent_.lock()) {
            std::shared_ptr<MSER::Region> currentParent = current;

            //while(currentParent->next_){
                currentPos.x = currentParent->moments_[0]  / currentParent->area_ ;
                currentPos.y = currentParent->moments_[1]  / currentParent->area_;
                if(cv::norm(currentPos - mPosition) > 5 && currentParent->area_ > HandModel::FingerAreaMax * 2)  break;
                currentParent = currentParent->next_;
            //}

            current = current->parent_.lock();
        }

        mOrientation = cv::Vec2f(cv::Point2f(currentPos - mPosition));
        //mOrientation = cv::normalize(mOrientation);
    }

    Blob(const std::shared_ptr<MSER::Region> reg, const cv::Point2i handCenter)
        : mEllipse(reg), mIntensity(reg->level_),
          mID(-1), mPosition(mEllipse.mPosition), mPrevPosition(mPosition), mFilteredPosition(mPosition), mPredictedPosition(mPosition),
          mUpdateCount(0), mClass(FINGER_UNKNOWN), mHand(nullptr), mHandIDLast(-1), mMissCounter(0), mTracked(false), mTrackedTemp(false), mConfident(true)
    {
        // to hand center
        mOrientation = cv::Vec2f(cv::Point2f(handCenter - mPosition));
    }

    // parameter
    Ellipse                 mEllipse;
    int                     mIntensity;

    // tracking parameter
    int                     mID;
    cv::Point2i             mPosition;
    cv::Vec2f               mOrientation;
    cv::Vec2f               mLastDirection;
    cv::Point2i             mPrevPosition;
    cv::Point2i             mFilteredPosition;
    cv::Point2i             mPredictedPosition;
    int                     mUpdateCount;
    FingerClass             mClass;
    std::shared_ptr<Hand>   mHand;

    int                     mHandIDLast; // use for finding backup hand

    int                     mMissCounter;

    bool                    mTracked;
    bool                    mTrackedTemp;

    bool                    mConfident;

    // 1 Euro
    OneEuroFilter           mXFilter;
    OneEuroFilter           mYFilter;

    // Double Exponential Smoothin
    std::shared_ptr<SmoothedPoint2D> mPredictedPositionStatistics;

    inline void initFilter(){
        mPrevPosition = mPosition;

        // 1 € filter
        mFilteredPosition = mPosition;
        mXFilter.initFilterParameter(31, 2, 0.007, 3);
        mYFilter.initFilterParameter(31, 2, 0.007, 3);

        // Double Exponential Smoothing
        mPredictedPosition = mPosition;
        mPredictedPositionStatistics = std::make_shared<SmoothedPoint2D>();
        mPredictedPositionStatistics->x = std::make_shared<SmoothingStatistics>();
        mPredictedPositionStatistics->y = std::make_shared<SmoothingStatistics>();
        initSmoothedPoint2D(mPredictedPositionStatistics);
        setSmoothedPoint2DAlpha(mPredictedPositionStatistics, 0.44f);
        updateSmoothedPoint2D(mPredictedPositionStatistics, mPosition);
    }
};

#endif // BLOB_H
