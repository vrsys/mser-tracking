#include "createmaxwindow.hpp"
#include "ui_createmaxwindow.h"

CreateMaxWindow::CreateMaxWindow(QWidget *parent) :
    QDialog(parent),
    mUI(new Ui::CreateMaxWindow),
    mSuccess(false),
    mCancel(false)
{
    mUI->setupUi(this);
}

CreateMaxWindow::~CreateMaxWindow()
{
    delete mUI;
}

void CreateMaxWindow::process(cv::Mat const& img)
{
    if (img.empty()) return;

    for(int i=0;i<mImage.rows;++i)
    {
        for (int j=0;j<mImage.cols;++j)
        {
            if (mImage.at<uchar>(i,j) < img.at<uchar>(i,j)){
                mImage.at<uchar>(i,j) = img.at<uchar>(i,j);
            }
        }
    }

    mUI->labelImage->setPixmap(QPixmap::fromImage(matToQImage(mImage)));
}

void CreateMaxWindow::closeWindow()
{
    mSuccess = false;
    mCancel = false;
    this->close();
}

cv::Mat CreateMaxWindow::getMaxImage() const
{
    return mImage;
}

bool CreateMaxWindow::success() const
{
    return mSuccess;
}

void CreateMaxWindow::on_buttonFinished_clicked()
{
    mSuccess = true;
}

void CreateMaxWindow::on_buttonCancel_clicked()
{
    mCancel = true;
}

void CreateMaxWindow::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Escape) {
        mCancel = true;
        event->accept();
    }
}

bool CreateMaxWindow::getCanceled() const
{
    return mCancel;
}

void CreateMaxWindow::initImage(const cv::Mat& img)
{
    mImage = img.clone();
}
