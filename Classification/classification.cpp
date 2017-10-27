#include "classification.hpp"

Classification::Classification()
{
}

void Classification::process(std::vector<std::shared_ptr<Hand> >& hands)
{
    for (std::shared_ptr<Hand> hand: hands){

        if (hand->mID == -1) continue; // skip  undefined hand with undefined finger

        // compute arm direction
        if(hand->mArmEllipse.mInclination < 0){
            float x = -90-hand->mArmEllipse.mInclination;
            float y = 90-x;
            cv::Vec2f temp (x, y);
            temp = cv::normalize(temp);
            hand->mArmDirection = temp;
        } else {
            float x = 90-hand->mArmEllipse.mInclination;
            float y = 90-x;
            cv::Vec2f temp (x, y);
            temp = cv::normalize(temp);
            hand->mArmDirection = temp;
        }


        // classificate only 10 times and choose best solution
        if(hand->mBlobs.size() == 5 && hand->mClass == HAND_UNKNOWN && (hand->mClassLeftCounter + hand->mClassRightCounter) < 10){
            int maxDist = 0;
            int indexThumb = 0;

            // find thumb (finger with highest min distance to other finger)
            for (unsigned int i = 0; i < hand->mBlobs.size(); ++i) {
                int minDist = -1;
                for (unsigned int j = 0; j < hand->mBlobs.size(); ++j) {
                    if(i==j) continue;

                    int distFinger = cv::norm(hand->mBlobs[i]->mPosition - hand->mBlobs[j]->mPosition);
                    if(minDist == -1 || distFinger < minDist){
                        minDist = distFinger;
                    }
                }

                if(minDist > maxDist){
                    indexThumb = i;
                    maxDist = minDist;
                }
            }
            hand->mBlobs[indexThumb]->mClass = FINGER_THUMB;
            cv::Point2i thumbPos = hand->mBlobs[indexThumb]->mPosition;

            // sort for sequence thumb -> index --> middle --> ring --> little (by distance to thumb)
            // todo: not allways correct.. improve.
            std::sort(hand->mBlobs.begin(), hand->mBlobs.end(), [thumbPos](const std::shared_ptr<Blob> b1, const std::shared_ptr<Blob> b2) -> bool
            {
                return cv::norm(thumbPos - b1->mPosition) < cv::norm(thumbPos - b2->mPosition);
            });


            // classificate fingers (by distance to thumb)
            // start with thumb
            hand->mBlobs[0]->mClass = FINGER_THUMB;
            hand->mBlobs[1]->mClass = FINGER_INDEX;
            hand->mBlobs[2]->mClass = FINGER_MIDDLE;
            hand->mBlobs[3]->mClass = FINGER_RING;
            hand->mBlobs[4]->mClass = FINGER_LITTLE;


            // determine left / right (angle between thumb->index and thumb->little)
            cv::Vec2i thumbIndex = hand->mBlobs[1]->mPosition - thumbPos;
            cv::Vec2i thumbLittle = hand->mBlobs[4]->mPosition - thumbPos;

            // thumbIndex = cv::normalize(thumbIndex);
            thumbLittle = cv::normalize(thumbLittle);
            int rotY = -thumbLittle[0] * thumbIndex[1] + thumbLittle[1] * thumbIndex[0];
            if(rotY < 0){
                ++hand->mClassLeftCounter;// = HAND_LEFT;
            } else {
                ++hand->mClassRightCounter;// = HAND_RIGHT;
            }
        } else if(hand->mClass == HAND_UNKNOWN && (hand->mClassLeftCounter + hand->mClassRightCounter) >= 10) {
            if (hand->mClassRightCounter > hand->mClassLeftCounter*2){
                hand->mClass = HAND_RIGHT;
            } else if (hand->mClassLeftCounter > hand->mClassRightCounter*2){
                hand->mClass = HAND_LEFT;
            } else {
                hand->mClass = HAND_UNKNOWN;
                hand->mClassLeftCounter = 0;
                hand->mClassRightCounter = 0;
            }
        }
    }
}
