#include "calibration.hpp"

Calibration::Calibration()
{
    processFrame = 100;
    mBoard_w = 7;
    mBoard_h = 10;
    n_boards = 10; // Number of boards
    board_n = mBoard_w * mBoard_h;
    board_sz = cvSize( mBoard_w, mBoard_h );


    // Allocate Sotrage
    image_points		= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
    object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
    point_counts		= cvCreateMat( n_boards, 1, CV_32SC1 );
    mIntrinsic          = cvCreateMat( 3, 3, CV_32FC1 );
    mDistortion     	= cvCreateMat( 5, 1, CV_32FC1 );


    corners = new CvPoint2D32f[ board_n ];


    corner_count = 0;
    successes = 0;
    step = 0;
    frame = 0;

    mIndex = 0;

    // todo avoid hard coded path!
    mPath = "Processing/ImageProcessing/ImagePreProcessing/Calibration/";
}


bool Calibration::findCalibrationMatrix(const cv::Mat& img)
{
    if (mIndex == 0){
        initCalib(img);
    }

    ++mIndex;


    /* Do calibration infront of the mirror !!!!!!!!!!!*/

    // convert back to iplimage
    IplImage* image = cvCreateImage(cvSize(img.cols,img.rows), IPL_DEPTH_8U, 1);
    image->imageData = (char *) img.data;

    cv::Mat imgColor = img.clone();
    cv::cvtColor(imgColor, imgColor, CV_GRAY2RGB);

    mTrackedImage = cv::Mat();
    corner_count = 0;

    if (successes < n_boards){
        // Skp every board_dt frames to allow user to move chessboard
        if( ++frame % processFrame == 0 ){
            // Find chessboard corners:
            int found = cvFindChessboardCorners( image, board_sz, corners,
                &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

            // Get subpixel accuracy on those corners
            //cvCvtColor( image, gray_image, CV_BGR2GRAY );
            cvFindCornerSubPix( image, corners, corner_count, cvSize( 11, 11 ),
                cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            cv::Mat corn;
            bool suc = cv::findChessboardCorners(imgColor, board_sz, corn);
            cv::drawChessboardCorners(imgColor, board_sz, corn, suc);

            // Draw it
            //cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );

            // If we got a good board, add it to our data
            if( found && corner_count == board_n ){
                step = successes*board_n;
                for( int i=step, j=0; j < board_n; ++i, ++j ){
                    CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
                    CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
                    CV_MAT_ELEM( *object_points, float, i, 0 ) = j/mBoard_w;
                    CV_MAT_ELEM( *object_points, float, i, 1 ) = j%mBoard_w;
                    CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
                }
                CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
                mTrackedImage = imgColor;
                successes++;
            }
        }
    } else {
        // Allocate matrices according to how many chessboards found
        CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
        CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
        CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );

        // Transfer the points into the correct size matrices
        for( int i = 0; i < successes*board_n; ++i ){
            CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
            CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
            CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
            CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
            CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
        }

        for( int i=0; i < successes; ++i ){
            CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
        }

        // At this point we have all the chessboard corners we need
        // Initiliazie the intrinsic matrix such that the two focal lengths
        // have a ratio of 1.0

        CV_MAT_ELEM( *mIntrinsic, float, 0, 0 ) = 1.0;
        CV_MAT_ELEM( *mIntrinsic, float, 1, 1 ) = 1.0;

        // Calibrate the camera
        cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ),
            mIntrinsic, mDistortion, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );

        // Build the undistort map that we will use for all subsequent frames
        IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
        IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );

        cvInitUndistortMap( mIntrinsic, mDistortion, mapx, mapy );

        // remap...
        IplImage *t1 = cvCloneImage( image );
        cvRemap( t1, image, mapx , mapy ); // undistort image
        cvReleaseImage( &t1 );

        mNewUndistort = cv::cvarrToMat(image);

        return true;
    }

    return false;
}

void Calibration::saveNewCalibrationMatrix() const
{
    // Save the intrinsics and distortions
    cvSave( (mPath + "Intrinsics.xml").c_str(), mIntrinsic );
    cvSave( (mPath + "Distortion.xml").c_str(), mDistortion );
    cv::imwrite(mPath + "undistort.jpg", mNewUndistort);
}
cv::Mat Calibration::getTrackedImage() const
{
    return mTrackedImage;
}
cv::Mat Calibration::getNewUndistort() const
{
    return mNewUndistort;
}

void Calibration::saveNewPerspectiveMatrix() const
{
    cv::FileStorage storage(mPath + "perspectiveCorrection.yml", cv::FileStorage::WRITE);
    storage << "transmtx" << mPerspectiveTransform;
    storage.release();

    cv::imwrite(mPath + "current.jpg", mTransformedImage);
}

void Calibration::saveNewCropRect() const
{
    cv::FileStorage fs(mPath + "cropedRect.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() ){
        fs << "x" << mBoundRect.x << "y" << mBoundRect.y;
        fs << "width" << mBoundRect.width << "height" << mBoundRect.height;
        fs.release();
    }
}

cv::Mat Calibration::transform(const cv::Mat img) const
{
    cv::Mat transformed;
    cv::warpPerspective(img, transformed, mPerspectiveTransform, img.size());
    return transformed;
}
cv::Rect Calibration::getBoundRect() const
{
    return mBoundRect;
}

std::string Calibration::getPath() const
{
    return mPath;
}

void Calibration::initCalib(const cv::Mat rawImg)
{
    cv::Mat intrinsic, distortion;
    // LOAD DATA FOR CALIBRATION
    //Load specific intrinsic and distortion matrices
    cv::FileStorage storage1(mPath + "Intrinsics.xml", cv::FileStorage::READ);
    cv::FileNode fn1 = storage1.getFirstTopLevelNode();
    fn1 >> intrinsic;
    storage1.release();
    cv::FileStorage storage2(mPath + "Distortion.xml", cv::FileStorage::READ);
    cv::FileNode fn2 = storage2.getFirstTopLevelNode();
    fn2 >> distortion;
    storage2.release();

    if (intrinsic.empty() || distortion.empty()) return;

    cv::Size res(rawImg.cols, rawImg.rows);

    // Build the undistort map that we will use for all subsequent frames
    cv::Mat mapX = cv::Mat::zeros(res.width, res.height, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(res.width, res.height, CV_32F);

    cv::Mat R ;
    cv::initUndistortRectifyMap(intrinsic, distortion, R, intrinsic, res, CV_32F, mapX, mapY);
    R.release();

    cv::remap(rawImg, mCurrentUndistort, mapX, mapY, cv::INTER_LINEAR);
}

cv::Mat Calibration::getTransformedImage() const
{
    return mTransformedImage;
}

void Calibration::saveCurrentUndistort() const
{
    cv::imwrite(mPath + "undistort.jpg", mCurrentUndistort);
}

void sortCorners(std::vector<cv::Point>& corners)
{
    // Get mass center
    cv::Point center(0,0);
    for (unsigned int i = 0; i < corners.size(); i++)
        center += corners[i];

    center *= (1. / corners.size());


    std::vector<cv::Point> top, bot;

    for (unsigned int i = 0; i < corners.size(); i++)
    {
        if (corners[i].y < center.y)
            top.push_back(corners[i]);
        else
            bot.push_back(corners[i]);
    }

    cv::Point tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point br = bot[0].x > bot[1].x ? bot[0] : bot[1];

    corners.clear();
    corners.push_back(tl);
    corners.push_back(bl);
    corners.push_back(br);
    corners.push_back(tr);
}




void Calibration::setNewPerspectiveCorrection(const cv::Mat &img)
{
    cv::Mat src = img;
    cv::Mat thr;
    cv::cvtColor(src,thr,CV_BGR2GRAY);
    cv::threshold( thr, thr, 70, 255, CV_THRESH_BINARY );

    std::vector< std::vector <cv::Point> > contours; // Vector for storing contour
    std::vector< cv::Vec4i > hierarchy;
    int largest_contour_index=0;
    int largest_area=0;

    cv::Mat dst(src.rows,src.cols,CV_8UC1,cv::Scalar::all(0)); //create destination image
    cv::findContours( thr.clone(), contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
    for( unsigned int i = 0; i< contours.size(); i++ ){
        double a=cv::contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
        }
    }

    cv::drawContours( dst, contours, largest_contour_index, cv::Scalar(255,255,255),CV_FILLED, 8, hierarchy );
    std::vector<std::vector<cv::Point> > contours_poly(1);
    cv::approxPolyDP( cv::Mat(contours[largest_contour_index]), contours_poly[0],100, true );
    mBoundRect = cv::boundingRect(contours[largest_contour_index]);

    std::vector<cv::Point> sortedCorners = contours_poly[0];
    sortCorners(sortedCorners);

    if(sortedCorners.size()==4){
        std::vector<cv::Point2f> quad_pts;
        std::vector<cv::Point2f> squre_pts;
        quad_pts.push_back(cv::Point2f(sortedCorners[0].x,sortedCorners[0].y));
        quad_pts.push_back(cv::Point2f(sortedCorners[1].x,sortedCorners[1].y));
        quad_pts.push_back(cv::Point2f(sortedCorners[3].x,sortedCorners[3].y));
        quad_pts.push_back(cv::Point2f(sortedCorners[2].x,sortedCorners[2].y));
        squre_pts.push_back(cv::Point2f(mBoundRect.x,mBoundRect.y));
        squre_pts.push_back(cv::Point2f(mBoundRect.x,mBoundRect.y+mBoundRect.height));
        squre_pts.push_back(cv::Point2f(mBoundRect.x+mBoundRect.width,mBoundRect.y));
        squre_pts.push_back(cv::Point2f(mBoundRect.x+mBoundRect.width,mBoundRect.y+mBoundRect.height));

        mPerspectiveTransform = cv::getPerspectiveTransform(quad_pts,squre_pts);
        mTransformedImage = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

        cv::warpPerspective(src, mTransformedImage, mPerspectiveTransform, src.size());
        cv::Point P1=sortedCorners[0];
        cv::Point P2=sortedCorners[1];
        cv::Point P3=sortedCorners[2];
        cv::Point P4=sortedCorners[3];
        cv::line(src,P1,P2, cv::Scalar(0,0,255),1,CV_AA,0);
        cv::line(src,P2,P3, cv::Scalar(0,0,255),1,CV_AA,0);
        cv::line(src,P3,P4, cv::Scalar(0,0,255),1,CV_AA,0);
        cv::line(src,P4,P1, cv::Scalar(0,0,255),1,CV_AA,0);
        cv::rectangle(src,mBoundRect,cv::Scalar(0,255,0),1,8,0);
        cv::rectangle(mTransformedImage,mBoundRect,cv::Scalar(0,255,0),1,8,0);
//        cv::imshow("quadrilateral", mTransformedImage);
//        cv::imshow("thr",thr);
//        cv::imshow("dst",dst);
//        cv::imshow("src",src);
    } else
    {
        std::cout << "Make sure that your are getting 4 corner using approxPolyDP..." << std::endl;
    }
    //std::cout << "New perspective correction saved to data/perspectiveCorrection/1200x880/perspectiveCorrection.yml" << std::endl;
}

