#include "visualisation.hpp"


void drawCCComponent(cv::Mat &img, const CC::Component& comp, const cv::Point2i pos, const cv::Scalar& color){
    cv::Rect rect(comp.pos_, comp.size_);

    cv::rectangle(img, rect, color, 2);
    cv::Mat temp = comp.img_;
    cv::cvtColor(temp, temp, CV_GRAY2RGB);

    temp.copyTo(img.rowRange(pos.y, pos.y + rect.height).colRange(pos.x, pos.x + rect.width));
}

void drawComponent(cv::Mat &img, const std::shared_ptr<MSER::Region>& node, cv::Point2i const& pos, const cv::Scalar& color)
{
    Ellipse ellipse(node);
    cv::ellipse(img, ellipse.mPosition + pos, cv::Size(ellipse.mMajor_axis, ellipse.mMinor_axis),
                ellipse.mInclination, 0, 360, color);
}


void drawComponentTree(cv::Mat& img, const std::shared_ptr<MSER::Region>& root, cv::Point2i const& pos, cv::Scalar const& color){
    if (root == nullptr) return;

    drawComponent(img, root, pos, color);
    drawComponentTree(img, root->child_, pos, color);
    drawComponentTree(img, root->next_, pos, color);
}


void drawHands(cv::Mat& img, const std::vector<std::shared_ptr<Hand> >& hands, cv::Scalar color){
    for (const std::shared_ptr<Hand> hand : hands){
        if (hand->mID == -1) color = cv::Scalar(255,255,255);

        //if(hand->mBlobs.size() == 0) continue;
        // center
        // position of hand
        cv::line(img, hand->mFilteredPosition + cv::Point2i(-5,-5), hand->mFilteredPosition + cv::Point2i(5,5),  color, 2);
        cv::line(img, hand->mFilteredPosition + cv::Point2i(-5,5),  hand->mFilteredPosition + cv::Point2i(5,-5), color, 2);

        // vis arm
        cv::ellipse(img, hand->mArmEllipse.mPosition, cv::Size(hand->mArmEllipse.mMajor_axis, hand->mArmEllipse.mMinor_axis),
                         hand->mArmEllipse.mInclination, 0, 360, color);

        // arm direction
        //cv::arrowedLine(img, hand->mPosition, hand->mPosition + cv::Point2i(30 * hand->mArmDirection), cv::Scalar(0,0,255), 2, 8, 0, 0.3);

        cv::Rect rect(hand->mBoundingBox.tl() + cv::Point2i(-10,-10), hand->mBoundingBox.br() + cv::Point2i(10,10));
        cv::rectangle(img, rect, cv::Scalar(255,255,255));
        // vis finger
        for (const std::shared_ptr<Blob> blob : hand->mBlobs){
            if(blob->mClass == FINGER_THUMB){
                //cv::circle(img, blob->mFilteredPosition, 10, color, 2);
            }else if(blob->mClass == FINGER_LITTLE) {
                //cv::circle(img, blob->mFilteredPosition, 5, cv::Scalar(255,255,0),1);
            } else {
                //cv::circle(img, blob->mFilteredPosition, 5, color);
            }
            if (hand->mID == -1) cv::circle(img, blob->mPosition, 7, color);
            else cv::circle(img, blob->mPosition, 5, color);
            //cv::circle(img, blob->mPosition, HandModel::BlobSpeedMax, color);

            // direction vec:
            //cv::arrowedLine(img, blob->mFilteredPosition, blob->mFilteredPosition + cv::Point2i(20 * blob->mLastDirection), cv::Scalar(0,0,255), 1, 8, 0, 0.1);

            // line to arm center
            //cv::line(img, hand->mArmEllipse.mCenter, blob->mPosition, cv::Scalar(255,255,255));
            //cv::arrowedLine(img, blob->mFilteredPosition, blob->mFilteredPosition + cv::Point2i(1 * blob->mOrientation), color, 1, 8, 0, 0.1);

            cv::putText(img, std::to_string(blob->mID), blob->mPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255) );
        }

        if(hand->mClass == HAND_LEFT){
            cv::putText(img, std::to_string(hand->mID)+"_left", hand->mBoundingBox.br()+cv::Point2i(-10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255) );
        }else if (hand->mClass == HAND_RIGHT) {
            cv::putText(img, std::to_string(hand->mID)+"_right", hand->mBoundingBox.br()+cv::Point2i(-10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255) );
        } else if (hand->mBlobs.size() > 0){
            cv::putText(img, std::to_string(hand->mID), hand->mBoundingBox.br()+cv::Point2i(-10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255) );
        } else {
            cv::putText(img, std::to_string(hand->mID), hand->mPalmPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255) );
        }
    }
}



void drawCluster(cv::Mat& img, const std::shared_ptr<MSERCluster> cluster, const cv::Scalar& color)
{
    //if (cluster->mBlobs.size()== 0) return;

    if (cluster->mConfident){
        // position of cluster
        cv::line(img, cluster->mPosition + cv::Point2i(-5, -5), cluster->mPosition + cv::Point2i(5, 5),  cv::Scalar(0, 255, 0), 2);
        cv::line(img, cluster->mPosition + cv::Point2i(-5, 5),  cluster->mPosition + cv::Point2i(5, -5), cv::Scalar(0, 255, 0), 2);
    } else {
        cv::line(img, cluster->mPosition + cv::Point2i(-5, -5), cluster->mPosition + cv::Point2i(5, 5),  cv::Scalar(0, 0, 255), 2);
        cv::line(img, cluster->mPosition + cv::Point2i(-5, 5),  cluster->mPosition + cv::Point2i(5, -5), cv::Scalar(0, 0, 255), 2);
    }

    // vis hand
    cv::ellipse(img, cluster->mHand.mPosition, cv::Size(cluster->mHand.mMajor_axis, cluster->mHand.mMinor_axis),
                cluster->mHand.mInclination, 0, 360, cv::Scalar(0,255,255));

    // vis arm
    cv::ellipse(img, cluster->mArm.mPosition, cv::Size(cluster->mArm.mMajor_axis+5, cluster->mArm.mMinor_axis+5),
                     cluster->mArm.mInclination, 0, 360, color);


    // vis finger
    for (auto finger : cluster->mBlobs){
        cv::circle(img, finger->mPosition, 3, cv::Scalar(255,0,255));

        // line to arm center
        cv::line(img, cluster->mHand.mPosition, finger->mPosition, cv::Scalar(255,255,255));

        //int distance = cv::norm(cluster->arm_.mCenter - finger->mPosition);
        //cv::putText(img, std::to_string(distance), finger->mPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255) );
    }
}
