#include "backgroundsubstraction.hpp"

BackgroundSubstraction::BackgroundSubstraction()
    : mPath("Processing/ImageProcessing/ImagePreProcessing/Calibration/"),
      mSuccessMin(false),
      mSuccessMax(false)
{
}

const cv::Mat& BackgroundSubstraction::process(const cv::Mat& img)
{
    // if no background image could find or the resolution differ to current one -> set new background.
    if (!mSuccessMin || mBackgroundImage.cols != img.cols || mBackgroundImage.rows != img.rows) {
        setBackgroundImage(img);
    }

    if (mSuccessMax){
        // 3 - 11ms

        //normalize every pixel value with that from max image
        #pragma omp parallel for
        for(int i=0;i<img.rows;++i)
        {
            const uchar* imgPtr = img.ptr<uchar>(i);
            for (int j=0;j<img.cols;++j)
            {
                if(mBackgroundPtr[i][j] < imgPtr[j]){
                    //mImagePtr[i][j] = (std::pow(clamp((imgPtr[j] - mBackgroundPtr[i][j]), 0.f, mMaxInitPtr[i][j]) / mMaxInitPtr[i][j], 2) * 255.f) ;
                    mImagePtr[i][j] = (std::pow(clamp((imgPtr[j] - mBackgroundPtr[i][j]), 0.f, mMaxPtr[i][j]) / mMaxPtr[i][j], 2) * 255.f) ;
                } else {
                    mImagePtr[i][j] = 0;
                }
            }
        }

        // blur
        cv::GaussianBlur(mImage, mImage, cv::Size(7, 7), 2, 2);
        return mImage;
    }

    // 0 - 3ms (~5-8 fps faster)
    cv::subtract(img, mBackgroundImage, mImage);
    //cv::absdiff(mBackgroundImage, img, mImage); // same cv::subtract(img, mBackgroundImage, diff);
    cv::GaussianBlur(mImage, mImage, cv::Size(7, 7), 2, 2);

    return mImage;
}

void BackgroundSubstraction::setBackgroundImage(const cv::Mat& img)
{
    mBackgroundImage = img.clone();

    if(mBackgroundImage.empty()){
        mSuccessMin = false;
        return;
    } else {
        mSuccessMin = true;
    }

    // add tiny brightness -> less noise
    mBackgroundImage += cv::Scalar(1);

    cv::imwrite(mPath+"min.jpg", mBackgroundImage);


    initMin();
}

void BackgroundSubstraction::setMaxImage(const cv::Mat& img)
{
    mMaxImage = img.clone();

    if(mMaxImage.empty()){
        mSuccessMax = false;
        return;
    } else {
        mSuccessMax = true;
    }

    // blur image
    cv::GaussianBlur(mMaxImage, mMaxImage, cv::Size(9, 9), 3, 3);

    cv::imwrite(mPath+"max.jpg", mMaxImage);

    initMax();
}

void BackgroundSubstraction::setMaxImage(std::string const& imageName)
{
    cv::Mat img = cv::imread(imageName, 0);
    setMaxImage(img);
}

void BackgroundSubstraction::loadMinMax(const std::string& path)
{
    mPath = path;

    if(mPath != ""){
        if(mPath.back() != '/'){
            mPath = mPath + "/";
        }
    } else {
        mPath = "Processing/ImageProcessing/ImagePreProcessing/Calibration/";
    }

    loadMin(mPath);
    loadMax(mPath);
}

void BackgroundSubstraction::loadMin(const std::string& path)
{
    mBackgroundImage.release();
    mBackgroundImage = cv::imread(path+"min.jpg", 0);
    if(mBackgroundImage.empty()) mSuccessMin = false;
    else mSuccessMin = true;

    initMin();
}

void BackgroundSubstraction::loadMax(const std::string& path)
{
    mMaxImage.release();
    mMaxImage = cv::imread(path+"max.jpg", 0);
    if(mMaxImage.empty() || mBackgroundImage.cols != mMaxImage.cols || mBackgroundImage.rows != mMaxImage.rows) mSuccessMax = false;
    else mSuccessMax = true;

    initMax();
}

void BackgroundSubstraction::initMin()
{
    mImage = cv::Mat::zeros(mBackgroundImage.rows, mBackgroundImage.cols, mBackgroundImage.type());

    mBackgroundPtr.clear();
    mImagePtr.clear();

    if(!mSuccessMin) return;

    for(int i = 0; i < mBackgroundImage.rows; ++i){
        mBackgroundPtr.push_back(mBackgroundImage.ptr<uchar>(i));
        mImagePtr.push_back(mImage.ptr<uchar>(i));
    }
}

void BackgroundSubstraction::initMax()
{
    mMaxInitImage = mMaxImage.clone();

    mMaxPtr.clear();
    mMaxInitPtr.clear();

    if(!mSuccessMax) return;

    for(int i = 0; i < mMaxImage.rows; ++i){
        mMaxPtr.push_back(mMaxImage.ptr<uchar>(i));
        mMaxInitPtr.push_back(mMaxInitImage.ptr<uchar>(i));
    }

    for (int i = 0; i < mImage.rows; ++i){
        for (int j = 0;j < mImage.cols;++j)
        {
            mMaxInitPtr[i][j] = clamp(mMaxInitPtr[i][j] - mBackgroundPtr[i][j], 0, 255);
        }
    }
}


const cv::Mat &BackgroundSubstraction::getBackgroundImage() const
{
    return mBackgroundImage;
}

const cv::Mat &BackgroundSubstraction::getMaxImage() const
{
    return mMaxImage;
}
