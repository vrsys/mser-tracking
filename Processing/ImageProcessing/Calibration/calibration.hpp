#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <iostream>

#include <opencv2/opencv.hpp>

class Calibration
{
public:
    Calibration();
    bool findCalibrationMatrix(const cv::Mat &img);
    void setNewPerspectiveCorrection(const cv::Mat &img);


    cv::Mat getTrackedImage() const;

    cv::Mat getNewUndistort() const;
    cv::Mat getTransformedImage() const;

    void saveCurrentUndistort() const;
    void saveNewCalibrationMatrix() const;
    void saveNewPerspectiveMatrix() const;
    void saveNewCropRect() const;

    cv::Mat transform(const cv::Mat img) const;

    cv::Rect getBoundRect() const;

    std::string getPath() const;

    int mIndex;

private:
    void initCalib(const cv::Mat rawImg);

    int mBoard_w;  // Board width in squares
    int mBoard_h;  // Board height

    std::string mPath;

    CvMat* mIntrinsic;
    CvMat* mDistortion;

    cv::Mat mTrackedImage;
    cv::Mat mCurrentUndistort;
    cv::Mat mNewUndistort;

    int processFrame;
    int n_boards; // Number of boards
    int board_n;

    CvSize board_sz;

    // bound rect (for croping)
    cv::Rect mBoundRect;

    // Allocate Sotrage
    CvMat* image_points;
    CvMat* object_points;
    CvMat* point_counts;

    CvPoint2D32f* corners;
    int corner_count;
    int successes;
    int step;
    int frame;

    // perspective
    cv::Mat mTransformedImage;
    cv::Mat mPerspectiveTransform;

};

#endif // CALIBRATION_H
