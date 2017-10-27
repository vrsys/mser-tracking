#ifndef ELLIPSE_H
#define ELLIPSE_H

#include <opencv2/opencv.hpp>

#include "Processing/ImageProcessing/MSER/mser.hpp"

struct Ellipse{
    Ellipse()
        :mPosition(0,0), mInclination(0), mMinor_axis(0), mMajor_axis(0)
    {
    }

    Ellipse(const std::shared_ptr<MSER::Region>& reg)
        :mPosition(0,0), mInclination(0), mMinor_axis(0), mMajor_axis(0)
    {
        // avoid wrong region pointer..
        //std::cout << typeid(reg->level_).name() << "   leevel: " << reg->level_ << std::endl;
        if (reg->level_ < 0 || reg->level_ > 255)return;

        double a,b,c,d, xc, yc;

        xc =     reg->moments_[0]  / reg->area_;
        yc =     reg->moments_[1]  / reg->area_;
        a = 	 reg->moments_[2] / reg->area_ - xc * xc;
        b = 2 * (reg->moments_[3] / reg->area_ - xc * yc);
        c = 	 reg->moments_[4] / reg->area_ - yc * yc;

        d = sqrt(b * b + (a - c) * (a - c));

        mPosition = cv::Point2i(xc, yc);

        if (a + c >= d) {
            mMinor_axis = sqrt( 3. * (a + c - d));
        } else {
            mMinor_axis = 0; //sqrt(-3. * (a + c - d));
        }

        mMajor_axis = sqrt(3 * (a + c + d));

        mInclination = ((0.5 * atan2(b, a - c)) / M_PI) * 180;
    }

    inline
    bool contains(cv::Point2i pos) const{

        cv::Point2i vec (pos - mPosition);

        float cont = ((float)((vec.x * vec.x)/(mMajor_axis * mMajor_axis)) + (float)((vec.y * vec.y) / (mMinor_axis * mMinor_axis)));
        return (cont <= 1);
    }

    cv::Point2i mPosition;
    int mInclination;
    double mMinor_axis;
    double mMajor_axis;
};

#endif // ELLIPSE_H
