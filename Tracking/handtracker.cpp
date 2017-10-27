#include "handtracker.hpp"

#define BACKUPHANDS 1
#define MINUPDATECOUNT 5

HandTracker::HandTracker()
    : mHandIDs(0),
      mBlobIDs(0),
      mUndefinedHand(std::make_shared<Hand>())
{
    mUndefinedHand->mID = -1;
}

std::vector<std::shared_ptr<Hand> > HandTracker::process(const std::vector<std::shared_ptr<MSERCluster> >& clusters)
{
#if BACKUPHANDS
    // init backup vector
    mBackupHands.clear();
    for (const std::shared_ptr<Hand>& h : mHands){
        std::shared_ptr<Hand> backupH = std::make_shared<Hand>(*h);
        backupH->mBlobs.clear();
        for (const std::shared_ptr<Blob>& b : h->mBlobs){
            std::shared_ptr<Blob> backupB = std::make_shared<Blob>(*b);
            backupB->mHand = backupH;
            backupH->mBlobs.push_back(backupB);
        }
        mBackupHands.push_back(backupH);
    }
#endif

    // untrack hands and blobs. all untracked objects will be deletet at the end
    for (std::shared_ptr<Hand> hand : mHands){
        hand->mTracked = false;
        hand->mConfident = true;
        hand->mLastClusterConfident = true;
        for (std::shared_ptr<Blob> b : hand->mBlobs){
            b->mTracked = false;
            if (!b->mConfident) {
                hand->mLastClusterConfident = false;
            }
        }
    }

    for (const std::shared_ptr<Blob> b : mUndefinedHand->mBlobs){
        b->mTracked = false;
    }

    int handSize = mHands.size();

    std::vector<std::array<int, 4>> corresPair; // pair <idcluster, idhand, distance, updateCount>

    // finde corresponding (closest) hands
    for (unsigned i = 0; i < clusters.size(); ++i){
        // supposed clustering works fine
        if (clusters[i]->mConfident){
            // find closest hand
            int index(-1);
            int distance     = findClosest(clusters[i], mHands, index);
            if(index != -1){
                corresPair.push_back({ {(int)i, index, distance, mHands[index]->mUpdateCount} });
            } else {
                corresPair.push_back({ {(int)i, index, distance, 0} });
            }
        }
    }

    // first update the nearest cluster <-> hand connections. possible also to sort against updateCounter
    std::sort(corresPair.begin(), corresPair.end(), [](std::array<int, 4> p1, std::array<int, 4> p2){ return (p1[2] <  p2[2]);});

    for (const std::array<int, 4>& pair: corresPair){
        if (pair[1] != -1 && handSize != 0 && pair[2] < HandModel::HandSpeedMax  && !mHands[pair[1]]->mTracked &&
            (mHands[pair[1]]->mUpdateCount >= MINUPDATECOUNT || mHands[pair[1]]->mLastClusterConfident)) {
            updateHand(mHands[pair[1]], clusters[pair[0]]);
        } else {
            addHand(mHands, clusters[pair[0]]);
        }
    }

    // Process unconfident Cluster and undefined (no handassignement) finger
    processUnconfidentCluster(mHands, clusters);
    trackUndefinedFinger(clusters);

    processIDKeeper();

#if BACKUPHANDS
    processUnconfidentHands();
#endif

    deleteUntracked();

    // reset IDs
#if 1
    if (mHands.size() == 0 && mBackupHands.size() == 0 && mUndefinedHand->mBlobs.size() == 0) {
        mHandIDs = 0;
        mBlobIDs = 0;
    } else {
        int biggestCurrentID(0);
        for (std::shared_ptr<Hand> h: mHands) {
            for (std::shared_ptr<Blob> b: h->mBlobs) {
                if (b->mID > biggestCurrentID) {
                    biggestCurrentID = b->mID;
                }
            }
        }

        for (std::shared_ptr<Blob> b: mUndefinedHand->mBlobs) {
            if (b->mID > biggestCurrentID) {
                biggestCurrentID = b->mID;
            }
        }

        mBlobIDs = biggestCurrentID +1;
    }
#endif

    std::vector<std::shared_ptr<Hand> > goodHands;
    for (std::shared_ptr<Hand>h : mHands){
        int currentBlobs = 0;
        for (std::shared_ptr<Blob> b : h->mBlobs){
            if (b->mMissCounter == 0) ++currentBlobs;
        }

        if(currentBlobs > 5){
            //std::cout << "!whaat should never happen!:  hand " << h->mID << "   has more than 5 blobs ( " << currentBlobs << " )"<< std::endl;
        } else if (h->mUpdateCount > 3) {
            goodHands.push_back(h);
        }
    }

    return goodHands;
}

void HandTracker::processUnconfidentCluster(const std::vector<std::shared_ptr<Hand> >& hands, const std::vector<std::shared_ptr<MSERCluster> >& clusters)
{
    if (hands.size()==0 || clusters.size() == 0)return;

    std::vector<std::shared_ptr<Blob>> candidatePoints;
    for (const std::shared_ptr<MSERCluster> & c : clusters) {
        if (!c->mConfident) {
            for (const std::shared_ptr<Blob> & b : c->mBlobs){
                b->mConfident = false;
                if(!b->mTracked) {
                    candidatePoints.push_back(b);
                }
            }
        }
    }

    if (candidatePoints.size() == 0) return;


    for(const std::shared_ptr<Hand> hand : hands){
        if(!hand->mTracked && hand->mUpdateCount >= MINUPDATECOUNT ) {

            std::vector<std::shared_ptr<Blob>> current;
            for(const std::shared_ptr<Blob> b : hand->mBlobs){
                if(!b->mTracked){
                    current.push_back(b);
                }
            }

            std::vector<CorresPoints> cPts;
            globalBlobTracker(current, candidatePoints, cPts, 2);
            //ICPTrackerCombinedUnconfident(current, candidatePoints, cPts, 10);

            for (const CorresPoints& cPt : cPts ){
                if(cPt.mIDCurrent != -1  && !current[cPt.mIDCurrent]->mTracked &&
                   !candidatePoints[cPt.mIDCandidate]->mTracked && cPt.mDist < HandModel::BlobSpeedMax){
                    updateBlob(current[cPt.mIDCurrent], candidatePoints[cPt.mIDCandidate]);
                }
            }

            if (hand->mBlobs.size() > 0) {
                // bounding box
                // search for top left and button right point.
                cv::Point2i tl(hand->mBlobs[0]->mPosition);
                cv::Point2i br(tl);

                cv::Point2i center(0,0);
                for (const std::shared_ptr<Blob> b: hand->mBlobs){
                    center += b->mPosition;
                    if (b->mPosition.x < tl.x) tl.x = b->mPosition.x;
                    if (b->mPosition.y < tl.y) tl.y = b->mPosition.y;

                    if (b->mPosition.x > br.x) br.x = b->mPosition.x;
                    if (b->mPosition.y > br.y) br.y = b->mPosition.y;
                }
                hand->mPosition = center / (int)hand->mBlobs.size();
                hand->mPalmPos = hand->mPosition;
                hand->mFingerCenter = hand->mPosition;
                hand->mBoundingBox = cv::Rect(tl, br);

                // 1 € filter
                hand->mFilteredPosition.x = hand->mXFilter.filter(hand->mPosition.x);
                hand->mFilteredPosition.y = hand->mYFilter.filter(hand->mPosition.y);

            } else {
                hand->mBoundingBox = cv::Rect(cv::Point2i(-20,-20), cv::Point2i(-20,-20));
                hand->mFingerCenter = hand->mPalmPos;
                hand->mPosition = hand->mPalmPos;

                // 1 € filter
                hand->mFilteredPosition.x = hand->mXFilter.filter(hand->mPosition.x);
                hand->mFilteredPosition.y = hand->mYFilter.filter(hand->mPosition.y);
            }
        }
    }
}

void HandTracker::updateHand(std::shared_ptr<Hand> hand, const std::shared_ptr<MSERCluster> cluster){
    // move all points into direction of cluster
    cv::Point2i moveVec = cluster->mPosition - hand->mPosition ;
    cv::Point2i moveVecPalm = cluster->mHand.mPosition - hand->mPalmPos;

    if(cv::norm(moveVecPalm) < cv::norm(moveVec)){
        moveVec = moveVecPalm;
    }

    for (std::shared_ptr<Blob> b : hand->mBlobs) {
        b->mPosition += moveVec;
    }

    if(hand->mBlobs.size() > 0){
        ++hand->mUpdateCount;
    }

    hand->mPalmPos = cluster->mHand.mPosition;
    hand->mPosition = cluster->mPosition;
    hand->mArmEllipse = cluster->mArm;

    // 1 € filter
    hand->mFilteredPosition.x = hand->mXFilter.filter(hand->mPosition.x);
    hand->mFilteredPosition.y = hand->mYFilter.filter(hand->mPosition.y);

    hand->mTracked = true;
    hand->mLastClusterConfident = true;
    hand->mMissCounter = 0;

    // update finger:
    std::vector<CorresPoints> cBlobs;
    ICPTrackerCombined(hand->mBlobs, cluster->mBlobs, cBlobs);
    //globalBlobTracker(hand->mBlobs, cluster->mBlobs, cBlobs, 3);

    for (const CorresPoints& corresBlobs : cBlobs){
        if (corresBlobs.mIDCurrent != -1 && corresBlobs.mDist < HandModel::HandSpeedMax &&
            !hand->mBlobs[corresBlobs.mIDCurrent]->mTracked &&
            !cluster->mBlobs[corresBlobs.mIDCandidate]->mTracked ) {
            updateBlob(hand->mBlobs[corresBlobs.mIDCurrent], cluster->mBlobs[corresBlobs.mIDCandidate] );
        } else {
            addBlob(hand, cluster->mBlobs[corresBlobs.mIDCandidate]);
        }
    }


    if (hand->mBlobs.size() > 0) {
        // bounding box
        // search for top left and button right point.
        cv::Point2i tl(hand->mBlobs[0]->mPosition);
        cv::Point2i br(tl);

        cv::Point2i fingerCenterPos(0,0);
        for (const std::shared_ptr<Blob> b: hand->mBlobs){
            fingerCenterPos += b->mPosition;
            if (b->mPosition.x < tl.x) tl.x = b->mPosition.x;
            if (b->mPosition.y < tl.y) tl.y = b->mPosition.y;

            if (b->mPosition.x > br.x) br.x = b->mPosition.x;
            if (b->mPosition.y > br.y) br.y = b->mPosition.y;
        }
        hand->mFingerCenter = (fingerCenterPos / (int)hand->mBlobs.size());
        hand->mBoundingBox = cv::Rect(tl, br);
    } else {
        hand->mFingerCenter = hand->mPalmPos;
        hand->mBoundingBox = cv::Rect(cv::Point2i(-20,-20), cv::Point2i(-20,-20));
    }
}

void HandTracker::updateBlob(std::shared_ptr<Blob> blob, const std::shared_ptr<Blob> newBlob){
    //std::cout << "update Blob" << std::endl;
    newBlob->mTracked = true;

    blob->mLastDirection = cv::Vec2f(cv::Point2f(newBlob->mPosition - blob->mPrevPosition));
    cv::normalize(blob->mLastDirection, blob->mLastDirection);

    blob->mPrevPosition = blob->mPosition;
    blob->mEllipse = newBlob->mEllipse;
    blob->mPosition = newBlob->mPosition;
    blob->mOrientation = newBlob->mOrientation;

    ++blob->mUpdateCount;
    blob->mTracked = true;
    blob->mMissCounter = 0;

    blob->mConfident = newBlob->mConfident;
    if(!blob->mConfident) {
        blob->mHand->mLastClusterConfident = false;
    }
    blob->mHand->mTracked = true;

    // 1 € filter
    blob->mFilteredPosition.x = blob->mXFilter.filter(blob->mPosition.x);
    blob->mFilteredPosition.y = blob->mYFilter.filter(blob->mPosition.y);

    // double exponential smoothing
    updateSmoothedPoint2D(blob->mPredictedPositionStatistics, blob->mPosition);
    predictSmoothedPoint2D(blob->mPredictedPositionStatistics, blob->mPredictedPosition, 1);
}

void HandTracker::addBlob(std::shared_ptr<Hand> hand, std::shared_ptr<Blob> newBlob){
    //std::cout << "add Blob" << std::endl;
    newBlob->mMissCounter = 0;
    newBlob->mUpdateCount = 0;
    newBlob->mTracked = true;
    newBlob->mHand = hand;
    newBlob->mHand->mTracked = true;
    newBlob->mHandIDLast = hand->mID;

    newBlob->mLastDirection = cv::Vec2i(0,0);

    newBlob->mID = mBlobIDs;
    ++mBlobIDs;

    newBlob->initFilter();

    hand->mBlobs.push_back(newBlob);
}

void HandTracker::addHand(std::vector<std::shared_ptr<Hand> >& hands, const std::shared_ptr<MSERCluster> cluster){
    std::shared_ptr<Hand> hand = std::make_shared<Hand>(cluster);

    ++hand->mUpdateCount;
    hand->mMissCounter = 0;
    hand->mTracked = true;

    hand->mID = mHandIDs;
    ++mHandIDs;

    for (std::shared_ptr<Blob> blob : cluster->mBlobs) {
        addBlob(hand, blob);
    }

    std::vector<std::shared_ptr<Blob> > candidates;
    for (const std::shared_ptr<Hand> h: mHands){
        for (const std::shared_ptr<Blob> b: h->mBlobs){
            if(!b->mTracked) {
                candidates.push_back(b);
            }
        }
    }

    if (candidates.size() != 0){
        std::vector<CorresPoints> cPts;
        ICPTrackerRidgid(cluster->mBlobs, candidates, cPts, 3, false);
        for (const CorresPoints& corresBlobs : cPts){
            if (corresBlobs.mIDCurrent != -1 && corresBlobs.mDist < HandModel::HandSpeedMax) {
                // keep id
                //std::cout << "keep ID" << std::endl;
                hand->mBlobs[corresBlobs.mIDCurrent]->mID = candidates[corresBlobs.mIDCandidate]->mID;
            }
        }
    }

    if (hand->mBlobs.size() > 0) {
        // bounding box
        // search for top left and button right point.
        cv::Point2i tl(hand->mBlobs[0]->mPosition);
        cv::Point2i br(tl);


        cv::Point2i fingerCenterPos(0,0);

        for (const std::shared_ptr<Blob> b: hand->mBlobs){
            fingerCenterPos += b->mPosition;

            if (b->mPosition.x < tl.x) tl.x = b->mPosition.x;
            if (b->mPosition.y < tl.y) tl.y = b->mPosition.y;

            if (b->mPosition.x > br.x) br.x = b->mPosition.x;
            if (b->mPosition.y > br.y) br.y = b->mPosition.y;
        }
        hand->mFingerCenter = (fingerCenterPos / (int)hand->mBlobs.size());

        hand->mBoundingBox = cv::Rect(tl, br);
    } else {
        hand->mBoundingBox = cv::Rect(cv::Point2i(-20,-20), cv::Point2i(-20,-20));
    }

    hands.push_back(hand);
}

void HandTracker::deleteHand(const std::shared_ptr<Hand> hand) {
    //std::cout << "delete hand" << std::endl;
    std::vector<std::shared_ptr<Hand> >::iterator position = std::find_if(mHands.begin(), mHands.end(),
                [hand](const std::shared_ptr<Hand> h) -> bool {return ((h->mPosition == hand->mPosition) ? true : false);});

    if (position != mHands.end()) // == myVector.end() means the element was not found
        mHands.erase(position);
}

void HandTracker::deleteBlob(std::shared_ptr<Hand> hand, std::shared_ptr<Blob> blob){
    //std::cout << "delete blob" << std::endl;
    std::vector<std::shared_ptr<Blob> >::iterator position = std::find_if(hand->mBlobs.begin(), hand->mBlobs.end(),
                [blob](const std::shared_ptr<Blob> b) -> bool {return ((b->mPosition == blob->mPosition) ? true : false);});
    if (position != hand->mBlobs.end())
        hand->mBlobs.erase(position);
}

void HandTracker::deleteUntracked()
{
#if 0
    // process addBlob vector
    for (std::shared_ptr<Blob> b : mCandidateBlobs){
        ++b->mMissCounter;
    }
    std::remove_if(mCandidateBlobs.begin(), mCandidateBlobs.end(), [](const std::shared_ptr<Blob> b) -> bool
    {
        if (b->mMissCounter > 0) return true;
        else return false;
    });
#endif

    // delete not tracked hands and blobs
    std::vector<unsigned> delIndexeHand;
    for (unsigned int i= 0; i < mHands.size() ; ++i){

        if(!mHands[i]->mTracked){
            if(mHands[i]->mMissCounter > -1){
                //std::cout << "delete hand " << mHands[i]->mID << std::endl;
                delIndexeHand.push_back(i);
            }
            ++mHands[i]->mMissCounter;
        }

        std::vector<unsigned> delIndexeBlob;
        for (unsigned j = 0; j < mHands[i]->mBlobs.size(); ++j){
            if(!mHands[i]->mBlobs[j]->mTracked){
                //std::cout << "del blob " << mHands[i]->mBlobs[j]->mID << std::endl;
                if(mHands[i]->mBlobs[j]->mMissCounter > -1){
                    // deleteBlob
                    //std::cout << "delete blob " << mHands[i]->mBlobs[j]->mID << std::endl;
                    delIndexeBlob.push_back(j);
                }
                ++mHands[i]->mBlobs[j]->mMissCounter;
            }
        }

        // sort to garanty right delete order (no index change) todo: find other solution
        std::sort(delIndexeBlob.begin(), delIndexeBlob.end(), [](unsigned i, unsigned j) {return i>j;});
        for (unsigned del : delIndexeBlob){
            mHands[i]->mBlobs[del] = nullptr;
            mHands[i]->mBlobs.erase(mHands[i]->mBlobs.begin() + del);
        }
    }

    std::vector<unsigned> delIndexeUnconfidentBlob;
    for (unsigned j = 0; j < mUndefinedHand->mBlobs.size(); ++j){
        if(!mUndefinedHand->mBlobs[j]->mTracked){
            //std::cout << "del blob " << mHands[i]->mBlobs[j]->mID << std::endl;
            if(mUndefinedHand->mBlobs[j]->mMissCounter > -1){
                // deleteBlob
                //std::cout << "delete blob " << mHands[i]->mBlobs[j]->mID << std::endl;
                delIndexeUnconfidentBlob.push_back(j);
            }
            ++mUndefinedHand->mBlobs[j]->mMissCounter;
        }
    }

    // sort to garanty right delete order (no index change) todo: find other solution
    std::sort(delIndexeUnconfidentBlob.begin(), delIndexeUnconfidentBlob.end(), [](unsigned i, unsigned j) {return i>j;});
    for (unsigned del : delIndexeUnconfidentBlob){
        mUndefinedHand->mBlobs[del] = nullptr;
        mUndefinedHand->mBlobs.erase(mUndefinedHand->mBlobs.begin() + del);
    }

    // sort to garanty right delete order (no index change) todo: find other solution
    std::sort(delIndexeHand.begin(), delIndexeHand.end(), [](unsigned i, unsigned j) {return i>j;});
    for (unsigned int del : delIndexeHand){
        mHands[del] = nullptr;
        mHands.erase(mHands.begin() + del);
    }
}


void HandTracker::processIDKeeper()
{
    std::vector<std::shared_ptr<Blob> > untrackedFinger;
    for (const std::shared_ptr<Hand> h: mHands){
        for (const std::shared_ptr<Blob> b: h->mBlobs){
            if(!b->mTracked) {
                untrackedFinger.push_back(b);
            }
        }
    }

    if (untrackedFinger.size() != 0){
        for (const std::shared_ptr<Hand> hand: mHands){
            std::vector<std::shared_ptr<Blob> > current;
            for (const std::shared_ptr<Blob> b: hand->mBlobs){
                if(b->mUpdateCount==0){
                    current.push_back(b);
                }
            }

            std::vector<CorresPoints> cPts;
            globalBlobTracker(current, untrackedFinger, cPts, 2);
            //ICPTrackerRidgid(hand->mBlobs, untrackedFinger, cPts, 10, 1);
            for (const CorresPoints& corresBlobs : cPts){
                if (corresBlobs.mIDCurrent != -1 && corresBlobs.mDist < HandModel::HandSpeedMax &&
                    !untrackedFinger[corresBlobs.mIDCandidate]->mTrackedTemp) {

                    int dist = computeDistance(hand->mFingerCenter, untrackedFinger[corresBlobs.mIDCandidate]->mHand->mFingerCenter);
                    if (dist > HandModel::F2H_DistanceMin*2) continue;

                    // keep id
                    current[corresBlobs.mIDCurrent]->mID = untrackedFinger[corresBlobs.mIDCandidate]->mID;
                    untrackedFinger[corresBlobs.mIDCandidate]->mTrackedTemp = true;

                    current[corresBlobs.mIDCurrent]->mHand->mConfident = false;
                    untrackedFinger[corresBlobs.mIDCandidate]->mHand->mConfident = false;
                    current[corresBlobs.mIDCurrent]->mHandIDLast = untrackedFinger[corresBlobs.mIDCandidate]->mHand->mID;
                }
            }
        }
    }
}


void HandTracker::trackUndefinedFinger(const std::vector<std::shared_ptr<MSERCluster> >& clusters)
{
    std::vector<std::shared_ptr<Blob> > undefinedFinger;
    for (const std::shared_ptr<Blob> b: mUndefinedHand->mBlobs){
        b->mConfident = false;
        undefinedFinger.push_back(b);
    }

    std::vector<std::shared_ptr<Blob> > untrackedCandidates;
    for (const std::shared_ptr<MSERCluster> c: clusters){
        for (const std::shared_ptr<Blob> b: c->mBlobs){
            if(!b->mTracked){
                untrackedCandidates.push_back(b);
            }
        }
    }

    std::vector<CorresPoints> cPts;
    globalBlobTracker(undefinedFinger, untrackedCandidates, cPts, 2);

    for (const CorresPoints& cPt : cPts ){
        if(cPt.mIDCurrent != -1  && !undefinedFinger[cPt.mIDCurrent]->mTracked &&
           !untrackedCandidates[cPt.mIDCandidate]->mTracked && cPt.mDist < HandModel::BlobSpeedMax){
            updateBlob(undefinedFinger[cPt.mIDCurrent], untrackedCandidates[cPt.mIDCandidate]);
        } else {
            addBlob(mUndefinedHand, untrackedCandidates[cPt.mIDCandidate]);
        }
    }

    // adopt IDs
    std::vector<std::shared_ptr<Blob> > currentNewFinger;
    for (const std::shared_ptr<Hand> hand: mHands){
        for (const std::shared_ptr<Blob> b: hand->mBlobs){
            if(b->mTracked && b->mUpdateCount == 0) {
                currentNewFinger.push_back(b);
            }
        }
    }
    //std::cout << undefinedFinger.size() << "  current  " << currentNewFinger.size() << std::endl;

    std::vector<CorresPoints> cPtsNew;
    globalBlobTracker(currentNewFinger, mUndefinedHand->mBlobs, cPtsNew, 2);

    for (const CorresPoints& corresBlobs : cPtsNew){
        if (corresBlobs.mIDCurrent != -1 && corresBlobs.mDist < (HandModel::HandSpeedMax) &&
            !currentNewFinger[corresBlobs.mIDCurrent]->mTrackedTemp)
        {
            // update position and id.
            currentNewFinger[corresBlobs.mIDCurrent]->mID = mUndefinedHand->mBlobs[corresBlobs.mIDCandidate]->mID;
            currentNewFinger[corresBlobs.mIDCurrent]->mTrackedTemp = true;
            currentNewFinger[corresBlobs.mIDCurrent]->mHand->mTracked = true;
            mUndefinedHand->mBlobs[corresBlobs.mIDCandidate]->mTracked = false;
        }
    }
}


void HandTracker::processUnconfidentHands()
{
#if 0
    std::vector<std::shared_ptr<Hand> > unconfidentHands;
    std::vector<std::shared_ptr<Blob> > unconfidentBlobs;
    for (const std::shared_ptr<Hand> hand : mHands) {
        if (!hand->mConfident) {
            unconfidentHands.push_back(hand);
            for (const std::shared_ptr<Blob> b : hand->mBlobs) {
                if(b->mTracked){
                    unconfidentBlobs.push_back(b);
                }
            }
            hand->mBlobs.clear();
        }
    }

    for (const std::shared_ptr<Hand> hand : unconfidentHands) {
        std::cout << "unconfident hand " << hand->mID << std::endl;
        for (const std::shared_ptr<Blob> b : unconfidentBlobs) {
            if(b->mHandIDLast == hand->mID) {
                b->mTracked = true;
                b->mHand = hand;
                hand->mTracked = true;
                std::cout << "add Blob " << b->mID << std::endl;
                hand->mBlobs.push_back(b);
            }
        }
    }

#else
    std::vector<std::shared_ptr<Blob> > unconfidentBlobs;
    std::vector<std::shared_ptr<Hand> > unconfidentBackupHands;

    std::vector<unsigned> delHands;

    for (unsigned i = 0; i < mHands.size(); ++i){
        if (!mHands[i]->mConfident) {
            delHands.push_back(i);

            for (const std::shared_ptr<Blob> b : mHands[i]->mBlobs) {
                if(b->mTracked){
                    unconfidentBlobs.push_back(b);
                }
            }
        }
    }

    std::vector<std::shared_ptr<Blob> > current;
    for (const std::shared_ptr<Hand> h : mBackupHands) {
        for (unsigned i : delHands) {
            if (h->mID == mHands[i]->mID) {
                unconfidentBackupHands.push_back(h);
                for (const std::shared_ptr<Blob> b : h->mBlobs) {
                    b->mTracked = false;
                    current.push_back(b);
                }
            }
        }
    }


    for (const std::shared_ptr<Blob> b : unconfidentBlobs) {
        b->mTracked = false;
    }

    std::vector<CorresPoints> cPts;
    globalBlobTracker(current, unconfidentBlobs, cPts, 2);

    for (const CorresPoints& cPt : cPts ){
        if(cPt.mIDCurrent != -1  && !current[cPt.mIDCurrent]->mTracked &&
           !unconfidentBlobs[cPt.mIDCandidate]->mTracked && cPt.mDist < HandModel::BlobSpeedMax){
            updateBlob(current[cPt.mIDCurrent], unconfidentBlobs[cPt.mIDCandidate]);
        }
    }


    std::sort(delHands.begin(), delHands.end(), [](unsigned i, unsigned j) {return i>j;});
    for (unsigned int del : delHands){
        mHands.erase(mHands.begin() + del);
    }

    mHands.insert(mHands.end(), unconfidentBackupHands.begin(), unconfidentBackupHands.end());

#endif
}


std::shared_ptr<Hand> HandTracker::undefinedHand() const
{
    return mUndefinedHand;
}
