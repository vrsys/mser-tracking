#include "trackingalgorithm.hpp"

#define VIS 0

int computeDistance(const cv::Point2i& p1, const cv::Point2i& p2) {
    return cv::norm(p2 - p1);
}

int findClosest(const std::shared_ptr<Blob> b, const std::vector<std::shared_ptr<Blob> >& blobs, int& index)
{
    int distance = -1;
    for (unsigned int i = 0; i < blobs.size(); ++i){
        if (b->mPosition == blobs[i]->mPosition) continue;

        int currentDist = computeDistance(b->mPosition, blobs[i]->mPosition);

        if (distance == -1 || currentDist < distance ) {
            distance = currentDist;
            index = i;
        }
    }
    return distance;
}

int findClosest(const cv::Point2i &p, const std::vector<cv::Point2i >& points, int& index){
    int distance = -1;
    for (unsigned int i = 0; i < points.size(); ++i){
        int currentDist = computeDistance(points[i], p);
        if (distance == -1 || currentDist < distance ) {
            distance = currentDist;
            index = i;
        }
    }

    return distance;
}

int findClosest(const cv::Point2i& p, const std::vector<std::shared_ptr<Blob> >& blobs, int& index){
    std::vector<cv::Point2i> points;
    for (std::shared_ptr<Blob> blob : blobs){
        points.push_back(blob->mPosition);
    }

    return findClosest(p, points, index);
}

int findClosest(const cv::Point2i &p, const std::vector<std::shared_ptr<MSERCluster> >& clusters, int& index, bool palm, bool fingers){

    int indexToBlob(-1);
    int distanceToBlob(-1);
    if (fingers){
        for (unsigned int i = 0; i < clusters.size(); ++i){
            int tempIndex = 0;
            int currentDist = findClosest(p, clusters[i]->mBlobs, tempIndex);
            if ((distanceToBlob == -1 || currentDist < distanceToBlob) && currentDist != -1) {
                distanceToBlob = currentDist;
                indexToBlob = i;
            }
        }
    }

    std::vector<cv::Point2i> points;
    for (std::shared_ptr<MSERCluster> cluster : clusters){
        points.push_back((palm) ? cluster->mHand.mPosition: cluster->mPosition);
    }
    int indexToCluster(-1);
    int distanceToCluster = findClosest(p, points, indexToCluster);

    if (distanceToBlob != -1 && distanceToBlob < distanceToCluster){
        index = indexToBlob;
        return distanceToBlob;
    }
    index = indexToCluster;
    return distanceToCluster;
}

int findClosest(const std::shared_ptr<Hand>& hand, const std::vector<std::shared_ptr<MSERCluster> >& clusters, int& index)
{
    return findClosest(hand->mPosition, clusters, index);
}

int findClosest(const std::shared_ptr<MSERCluster> &cluster, const std::vector<std::shared_ptr<Hand> >& hands, int& index) {

    // if distance allready to big do not evaluate other position
    int dist = findClosest(cluster->mHand.mPosition, hands, index, 0);
    if (dist > (2*HandModel::HandSpeedMax)) return dist;

    // distance fingercenter to fingercenter
    if (cluster->mBlobs.size() > 0){
        cv::Point2i centerPos(0,0);
        for (std::shared_ptr<Blob> b : cluster->mBlobs){
            centerPos += b->mPosition;
        }
        centerPos /= (int)cluster->mBlobs.size();
        int indexFingerCenter(-1);
        int distanceFingerCenter = findClosest(centerPos, hands, indexFingerCenter, 3);

        if (index == -1 || (distanceFingerCenter < dist && indexFingerCenter != -1)){
            dist = distanceFingerCenter;
            index = indexFingerCenter;
        }
    }

    // check to palm
    int indexToPalm(-1);
    int distanceToPalm = findClosest(cluster->mPosition, hands, indexToPalm, 1);

    if (index == -1 || (distanceToPalm < dist && indexToPalm != -1)){
        dist = distanceToPalm;
        index = indexToPalm;
    }

    // distance palm to palm
    int indexPalmToPalm(-1);
    int distancePalmToPalm = findClosest(cluster->mHand.mPosition, hands, indexPalmToPalm, 1);

    if (distancePalmToPalm < dist && indexPalmToPalm != -1){
        dist = distancePalmToPalm;
        index = indexPalmToPalm;
    }

#if 0

    // distance arm to arm
    int indexArmToArm(-1);
    int distanceArmToArm = findClosest(cluster->mArm.mPosition, hands, indexArmToArm, 2);

    if (distanceArmToArm < dist && indexArmToArm != -1){
        dist = distanceArmToArm;
        index = indexArmToArm;
    }

    // distance to each finger
    int indexToFinger(-1);
    int distanceToFinger(-1);

    for (std::shared_ptr<Blob> b : cluster->mBlobs){
        for (unsigned i = 0; i < hands.size(); ++i){
            int tempIndex = 0;
            int currentDist = findClosest(b->mPosition, hands[i]->mBlobs, tempIndex);
            if ((distanceToFinger == -1 || currentDist < distanceToFinger) && currentDist != -1) {
                distanceToFinger = currentDist;
                indexToFinger = i;
            }
        }
    }
    if (distanceToFinger < dist && indexToFinger != -1){
        dist = distanceToFinger;
        index = indexToFinger;
    }

//    std::cout << "dist: " << dist;
//    std::cout << "   ToPalm: " << distanceToPalm;
//    std::cout << "   PalmToPalm: " << distancePalmToPalm;
//    std::cout << "   PalmToPos: " << distancePalmToPos;
//    std::cout << "   ToFinger: " << distanceToFinger << std::endl;




#endif
    return dist;
}


int findClosest(const cv::Point2i &p, const std::vector<std::shared_ptr<Hand> >& hands, int& index, int mode)
{
    std::vector<cv::Point2i> points;

    for (std::shared_ptr<Hand> hand : hands){
        switch (mode) {
            case 0:
                points.push_back(hand->mPosition);
                break;
            case 1:
                points.push_back(hand->mPalmPos);
                break;
            case 2:
                points.push_back(hand->mArmEllipse.mPosition);
                break;
            case 3:
                points.push_back(hand->mFingerCenter);
                break;
            default:
                break;
        }
    }

    return findClosest(p, points, index);
}



void minimalDistanceTracker(const std::vector<std::shared_ptr<Blob> > &currentBlobs, const std::vector<std::shared_ptr<Blob> > &candidateBlobs, std::vector<CorresPoints> &corresPts, bool predictedPos)
{
    for(unsigned i = 0; i < currentBlobs.size(); ++i){
        std::vector<cv::Point2i> candidatePoints;

        for(unsigned j = 0; j < candidateBlobs.size(); ++j) {
            bool found = false;
            for (CorresPoints cPt : corresPts) {
                if (cPt.mIDCandidate == (int)j) {
                    found = true;
                }
            }

            if(!found) {
                candidatePoints.push_back(candidateBlobs[j]->mPosition);
            } else {
                candidatePoints.push_back(cv::Point2i(-100,-100));
            }
        }

        int index(-1), dist(0);

        if (predictedPos) {
            dist = findClosest(currentBlobs[i]->mPredictedPosition, candidatePoints, index);
        }else {
            dist = findClosest(currentBlobs[i]->mPosition, candidatePoints, index);
        }

        if (index != -1){
            corresPts.push_back(CorresPoints(index, i, dist));
        }
    }

    // fill up with not set candidates..
    for(unsigned i = 0; i < candidateBlobs.size(); ++i) {
        bool found = false;
        for (CorresPoints cPt : corresPts) {
            if (cPt.mIDCandidate == (int)i) {
                found = true;
            }
        }

        if(!found) {
            corresPts.push_back(CorresPoints(i, -1, -1));
        }
    }
}

void knFingerTracker(const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs, std::vector<CorresPoints>& corresPts , bool predictedPos){
    std::vector<cv::Point2i> currentPoints(currentBlobs.size()), candidatePoints(candidateBlobs.size());

#if VIS
        cv::Mat img = cv::Mat::zeros(447, 655, CV_8U);
        cv::cvtColor(img, img, CV_GRAY2RGB);
#endif

    for(unsigned i = 0; i < currentBlobs.size(); ++i){
        if (predictedPos){
            currentPoints[i] = currentBlobs[i]->mPredictedPosition;
        } else {
            currentPoints[i] = currentBlobs[i]->mPosition;
        }
#if VIS
        cv::circle(img, currentPoints[i], 3, cv::Scalar(0,255,255));
#endif
    }

    for(unsigned i = 0; i < candidateBlobs.size(); ++i) {
        candidatePoints[i] = candidateBlobs[i]->mPosition;
#if VIS
        cv::circle(img, candidatePoints[i], 3, cv::Scalar(255,0,0));
#endif
    }

    knFingerTracker(currentPoints, candidatePoints, corresPts);

#if VIS
    for (unsigned int i = 0 ; i < corresPts.size(); ++i){
        if(corresPts[i].mIDCurrent == -1) continue;

        cv::Point2i current = currentPoints[corresPts[i].mIDCurrent];
        cv::Point2i cand = candidatePoints[corresPts[i].mIDCandidate];
        cv::line(img, current, cand, cv::Scalar(0,0,255), 1);
    }
    cv::imshow("knnFlannTracker", img);
    cv::waitKey();
#endif
}

void knFingerTracker(const std::vector<cv::Point2i>& currentPoints, const std::vector<cv::Point2i>& candidatePoints, std::vector<CorresPoints>& corresPts)
{
    // CONVERT TO MAT
    cv::Mat currentFingerPos(currentPoints);
    cv::Mat candidates(candidatePoints);

    if(currentFingerPos.channels() == 2) { currentFingerPos = currentFingerPos.reshape(1); }
    if(candidates.channels() == 2) { candidates = candidates.reshape(1); }

    // find nearest neighbors using FLANN
    cv::Mat indexMat(candidates.rows, 1, CV_32S);
    cv::Mat distMat(candidates.rows, 1, CV_32F);

    cv::Mat cand_32f; candidates.convertTo(cand_32f, CV_32FC2);
    cv::Mat finger_32f; currentFingerPos.convertTo(finger_32f, CV_32FC2);

    cv::flann::Index flann_index(finger_32f, cv::flann::LinearIndexParams());  // using 4 randomized kdtrees
    flann_index.knnSearch(cand_32f, indexMat, distMat, 1, cv::flann::SearchParams(64) ); // maximum number of leafs checked

    // index pointer (faster)
    int* indices_ptr = indexMat.ptr<int>(0);

    // for each candidate push (if existing) a probably match
    for (unsigned i = 0; i < candidatePoints.size(); ++i) {
        if(i < (unsigned) indexMat.rows && (unsigned) indices_ptr[i] < currentPoints.size() && i < candidatePoints.size()){
            corresPts.push_back(CorresPoints(i, indices_ptr[i], cv::norm(currentPoints[indices_ptr[i]] - candidatePoints[i])));
        } else {
            corresPts.push_back(CorresPoints(i, -1, -1));
        }
    }

    // sort by distance (probability)...
    std::sort(corresPts.begin(), corresPts.end(), [](const CorresPoints& c1, const CorresPoints& c2){
        return c1.mDist < c2.mDist;
    });
}


void knCandidateTracker(const std::vector<cv::Point2i>& candidatePoints, const std::vector<cv::Point2i>& currentPoints, std::vector<CorresPoints>& corresPts)
{
    // CONVERT TO MAT
    cv::Mat candidateFingerPos(candidatePoints);
    cv::Mat currents(currentPoints);

    if(candidateFingerPos.channels() == 2) { candidateFingerPos = candidateFingerPos.reshape(1); }
    if(currents.channels() == 2) { currents = currents.reshape(1); }

    // find nearest neighbors using FLANN
    cv::Mat indexMat(currents.rows, 1, CV_32S);
    cv::Mat distMat(currents.rows, 1, CV_32F);

    cv::Mat current_32f; currents.convertTo(current_32f, CV_32FC2);
    cv::Mat cand_32f; candidateFingerPos.convertTo(cand_32f, CV_32FC2);

    cv::flann::Index flann_index(cand_32f, cv::flann::LinearIndexParams());  // using 4 randomized kdtrees
    flann_index.knnSearch(current_32f, indexMat, distMat, 1, cv::flann::SearchParams(64) ); // maximum number of leafs checked

    // index pointer (faster)
    int* indices_ptr = indexMat.ptr<int>(0);

    // for each candidate push (if existing) a probably match
    for (unsigned i = 0; i < currentPoints.size(); ++i) {
        if(i < (unsigned) indexMat.rows && (unsigned) indices_ptr[i] < candidatePoints.size() && i < currentPoints.size()){
            corresPts.push_back(CorresPoints(indices_ptr[i], i, cv::norm(candidatePoints[indices_ptr[i]] - currentPoints[i])));
        }
    }

    // sort by distance (probability)...
    std::sort(corresPts.begin(), corresPts.end(), [](const CorresPoints& c1, const CorresPoints& c2){
        return c1.mDist < c2.mDist;
    });
}




void ICPTrackerTrans(const std::vector<std::shared_ptr<Blob> > &currentBlobs, const std::vector<std::shared_ptr<Blob> > &candidateBlobs, std::vector<CorresPoints> &corresPoints, int maxIterations)
{
    int iterations = 0;

    std::vector<cv::Point2i> transformedCurrent;
    for (unsigned int i = 0; i < currentBlobs.size(); ++i){
        transformedCurrent.push_back(currentBlobs[i]->mPosition);
    }

    std::vector<cv::Point2i> candidates;
    //std::cout << "candidate vector" << std::endl;
    for (unsigned int i = 0; i < candidateBlobs.size(); ++i){
        candidates.push_back(candidateBlobs[i]->mPosition);
    }

    std::vector<CorresPoints> oldCPts;

    while (iterations <= maxIterations){
#if VIS
        cv::Mat img = cv::Mat::zeros(447, 655, CV_8U);
        cv::cvtColor(img, img, CV_GRAY2RGB);

        for (unsigned int i = 0 ; i < transformedCurrent.size(); ++i){
            cv::circle(img, transformedCurrent[i], 3, cv::Scalar(255,0,0));
        }
        for (unsigned int i = 0 ; i < candidates.size(); ++i){
            cv::circle(img, candidates[i], 3, cv::Scalar(0,0,255));
        }
#endif


        /************** find closest points ******************/
        std::vector<CorresPoints> cPts;
        knFingerTracker(transformedCurrent, candidates, cPts);

        /************** compute translation of each point ******************/
        std::vector<cv::Point2i> corresPtsCurrent;
        std::vector<cv::Point2i> corresPtsCandidates;

        int count = 0;
        cv::Vec2i trans(0,0);
        for (unsigned int i = 0 ; i < cPts.size(); ++i){
            if(cPts[i].mIDCurrent == -1) continue;
            // don't allow double correspondances.

            if(std::find(corresPtsCurrent.begin(), corresPtsCurrent.end(), transformedCurrent[cPts[i].mIDCurrent]) == corresPtsCurrent.end()){
                cv::Point2i current = transformedCurrent[cPts[i].mIDCurrent];
                cv::Point2i cand = candidates[cPts[i].mIDCandidate];
                corresPtsCurrent.push_back(current);
                corresPtsCandidates.push_back(cand);

#if VIS
                cv::line(img, current, cand, cv::Scalar(0,0,255), 1);
#endif
                cv::Vec2i t = cand - current;
                trans += t;
                ++count;
            }
        }

        trans /= count;

        // get mean transcv::Mat estimateTranslation(const std::vector<cv::Point2i>& currentPoints, const std::vector<cv::Point2i>& candidates){

        // transform each point
        for (unsigned int i = 0 ; i < transformedCurrent.size(); ++i){
            transformedCurrent[i] += cv::Point2i(trans);
        }


        // break if success
        if (oldCPts == cPts /*cv::norm(trans) < 0*/){
            break;
        }
        oldCPts = cPts;

#if VIS
        cv::imshow("ICPTracker", img);
        cv::waitKey();
#endif
        ++iterations;
    }
    // return indices...
    corresPoints = oldCPts;
}

// see http://nghiaho.com/?page_id=671
cv::Mat estimateSVDTransform(const std::vector<cv::Point2i>& currentPoints, const std::vector<cv::Point2i>& candidates)
{
    if(currentPoints.size() == 0 || candidates.size() == 0) {
        cv::Mat R = cv::Mat::eye(2, 2, CV_32F);
        cv::Mat T = cv::Mat(cv::Point2i(0,0));
        T.convertTo(T, R.type());
        cv::Mat transform;
        cv::hconcat(R, T, transform);
        return transform;
    }

    cv::Mat currentsMat = cv::Mat(currentPoints).reshape(1);
    cv::Mat candidatesMat = cv::Mat(candidates).reshape(1);

    currentsMat.convertTo(currentsMat, CV_32F);
    candidatesMat.convertTo(candidatesMat, CV_32F);

    /************** find centroid of both datasets ******************/
    cv::Mat centroidCurrent = cv::Mat::zeros(1, 2, CV_32F);
    cv::Mat centroidCandidate = cv::Mat::zeros(1, 2, CV_32F);


    for (int i = 0 ; i < currentsMat.rows; ++i){
        centroidCurrent += currentsMat.row(i);
        centroidCandidate += candidatesMat.row(i);
    }

    centroidCurrent /= currentsMat.rows;
    centroidCandidate /= currentsMat.rows;

    /************** transform both datasets to origin and find rotation ******************/
    for (int i = 0 ; i < currentsMat.rows; ++i){
        currentsMat.row(i) -= centroidCurrent;
        candidatesMat.row(i) -= centroidCandidate;
    }

    //covariance matrix
    cv::Mat H = candidatesMat.t() * currentsMat;
    cv::SVD svd(H);
    cv::Mat R = svd.u * svd.vt;

    // get angle
    float angle = acos(R.at<float> (0,0))*(180/M_PI);

#if 0
    // theoretically check for reflection ... pratical not ToDo.. correct?
    if(determinant(R) < 0){
        R.row(1) *= -1;
    }
#endif

    /************** find translation ******************/
    // rot to big -> ignore rotation
    // Todo: adjust max rot from fps.
    cv::Mat transformation;
    cv::Mat trans;

    if (angle > HandModel::HandRotationMax) {
        //std::cout << "estimated rot angle too big. use only translation: " << angle << "   (" << HandModel::HandRotationMax << ")" << std::endl;
        cv::Point2i t(0,0);

        for(unsigned i = 0; i < currentPoints.size(); ++i){
            t += (candidates[i] - currentPoints[i]);
        }
        t /= (int) currentPoints.size();
        trans = cv::Mat(t);
        trans.convertTo(trans, R.type());
        R = cv::Mat::eye(2, 2, R.type());
        cv::hconcat(R, trans, transformation);
    }else {
        trans = centroidCandidate.t() - (R * centroidCurrent.t());
        cv::hconcat(R, trans, transformation);
    }

    return transformation;
}


void ICPTrackerRidgid(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts, int maxIterations, int maxConnections, bool predicted, bool candidateTracker)
{
    int iteration = 0;

    std::vector<cv::Point2i> transformedCurrent;
    for (unsigned int i = 0; i < currentBlobs.size(); ++i){
        if(!currentBlobs[i]->mTracked){
            if(predicted){
                transformedCurrent.push_back(currentBlobs[i]->mPredictedPosition);
            } else {
                transformedCurrent.push_back(currentBlobs[i]->mPosition);
            }
        } else {
            transformedCurrent.push_back(cv::Point2i(-1000,-1000));
        }
    }

    std::vector<cv::Point2i> candidates;
    //std::cout << "candidate vector" << std::endl;
    for (unsigned int i = 0; i < candidateBlobs.size(); ++i){
        if(!candidateBlobs[i]->mTracked){
            candidates.push_back(candidateBlobs[i]->mPosition);
        } else {
            candidates.push_back(cv::Point2i(-1000,-1000));
        }
    }

    std::vector<CorresPoints> oldCPts;

    while (iteration <= maxIterations) {
#if VIS
        cv::Mat img = cv::Mat::zeros(447, 655, CV_8U);
        cv::cvtColor(img, img, CV_GRAY2RGB);

        for (unsigned int i = 0 ; i < transformedCurrent.size(); ++i){
            cv::circle(img, transformedCurrent[i], 5, cv::Scalar(255,0,0));
        }
        for (unsigned int i = 0 ; i < candidates.size(); ++i){
            cv::circle(img, candidates[i], 3, cv::Scalar(0,255,255));
        }
#endif
        /************** find closest points ******************/
        std::vector<CorresPoints> cPts;
        if(candidateTracker) {
            knCandidateTracker(candidates, transformedCurrent, cPts);
        } else {
            knFingerTracker(transformedCurrent, candidates, cPts);
        }

        if(transformedCurrent.size() == 0 || candidateBlobs.size() == 0){
            corresPts = cPts;
            return;
        }

        /************** compare closest points (point clouds need same size) ***************/
        std::vector<cv::Point2i> corresPtsCurrent;
        std::vector<cv::Point2i> corresPtsCandidates;

        for (unsigned int i = 0 ; i < cPts.size(); ++i){
            if(cPts[i].mIDCurrent == -1) continue;

            // diff in behavior for number of current fingers.
            if (maxConnections != 1) {
                if(maxConnections <= 0 || maxConnections > 2) {
                    /*** USE ALL CONNECTIONS ***/
                    cv::Point2i current = transformedCurrent[cPts[i].mIDCurrent];
                    cv::Point2i cand = candidates[cPts[i].mIDCandidate];

                    if (cPts[i].mDist < HandModel::BlobSpeedMax){
                        corresPtsCurrent.push_back(current);
                        corresPtsCandidates.push_back(cand);
            #if VIS
                        cv::line(img, current, cand, cv::Scalar(0,0,255), 1);
            #endif
                    }
                } else {
                    // --> use maximum two candidates that reference the same finger (1 to 2 relation)
                    auto it = std::find(corresPtsCurrent.begin(), corresPtsCurrent.end(), transformedCurrent[cPts[i].mIDCurrent]);
                    if(it == corresPtsCurrent.end() || std::find(it+1, corresPtsCurrent.end(), transformedCurrent[cPts[i].mIDCurrent]) == corresPtsCurrent.end()){
                        cv::Point2i current = transformedCurrent[cPts[i].mIDCurrent];
                        cv::Point2i cand = candidates[cPts[i].mIDCandidate];

                        if (cPts[i].mDist < HandModel::BlobSpeedMax){
                            corresPtsCurrent.push_back(current);
                            corresPtsCandidates.push_back(cand);
                #if VIS
                            cv::line(img, current, cand, cv::Scalar(0,0,255), 1);
                #endif
                        }
                    }
                }
            } else {
                if(std::find(corresPtsCurrent.begin(), corresPtsCurrent.end(), transformedCurrent[cPts[i].mIDCurrent]) == corresPtsCurrent.end()){
                    cv::Point2i current = transformedCurrent[cPts[i].mIDCurrent];
                    cv::Point2i cand = candidates[cPts[i].mIDCandidate];

                    if (cPts[i].mDist < HandModel::BlobSpeedMax){
                        corresPtsCurrent.push_back(current);
                        corresPtsCandidates.push_back(cand);
            #if VIS
                        cv::line(img, current, cand, cv::Scalar(0,0,255), 1);
            #endif
                    }
                }
            }
        }

        //cv::Mat transform = cv::estimateRigidTransform(corresPtsCurrent, corresPtsCandidates, false);
        cv::Mat transform = estimateSVDTransform(corresPtsCurrent, corresPtsCandidates);
//        bool success = computeAffine(corresPtsCurrent, corresPtsCandidates, transform);

        // couldn't find any transformation.
        if (transform.empty()) break;

        cv::transform(transformedCurrent, transformedCurrent, transform);

        if (oldCPts == cPts /*cv::norm(trans) < 0*/){
            break;
        }
        oldCPts = cPts;
#if VIS
        cv::imshow("ridgidTrans", img);
        cv::waitKey();
#endif

        ++iteration;
    }

    // reupdate distances
    for (unsigned i = 0; i < oldCPts.size(); ++i) {
        oldCPts[i].mDist = computeDistance(candidateBlobs[oldCPts[i].mIDCandidate]->mPosition, currentBlobs[oldCPts[i].mIDCurrent]->mPosition);
    }

    // fill up with not set candidates..
    for(unsigned i = 0; i < candidateBlobs.size(); ++i) {
        bool found = false;
        for (CorresPoints cPt : oldCPts) {
            if (cPt.mIDCandidate == (int)i) {
                found = true;
            }
        }

        if(!found) {
            oldCPts.push_back(CorresPoints(i, -1, -1));
        }
    }

    // return indices...
    corresPts = oldCPts;
}

void permute(std::vector<std::vector<CorresPoints> >& allCombinations ,
             const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs,
             std::vector<CorresPoints> currentSolution, int indexFinger, unsigned currentSize, unsigned candidateSize, unsigned maxConnections)
{

    if (indexFinger == (int)currentSize ) {
        allCombinations.push_back(currentSolution);
    } else {
        std::vector <std::pair<unsigned, int>> nearestCandidates;
        for (unsigned c = 0; c < candidateSize; ++c) {
            bool found = false;
            for (CorresPoints cPt : currentSolution) {
                if(cPt.mIDCandidate == (int) c ) {
                    found = true;
                }
            }
            if(!found) {
                // find x nearest points...
                int dist = computeDistance(currentBlobs[indexFinger]->mPosition, candidateBlobs[c]->mPosition);
                if(dist < HandModel::BlobSpeedMax) {
                    nearestCandidates.push_back(std::make_pair(c, dist));
                }
            }
        }

        // sort for nearest candidates
        std::sort(nearestCandidates.begin(), nearestCandidates.end(), [](std::pair<unsigned, int> p1, std::pair<unsigned, int>p2) {return p1.second < p2.second;});

        // for maxConnections nearest candidates find permutation again
        for (unsigned c = 0; c < nearestCandidates.size() && c < maxConnections; ++c) {
            currentSolution.push_back(CorresPoints(nearestCandidates[c].first, indexFinger, nearestCandidates[c].second));
            permute(allCombinations, currentBlobs, candidateBlobs, currentSolution, indexFinger+1, currentSize, candidateSize, maxConnections);
            currentSolution.pop_back();
        }
    }
}

void globalBlobTracker(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts, int maxConnections)
{
    if (candidateBlobs.size() == 0) return;

    std::vector<CorresPoints> temp;
    std::vector<std::vector<CorresPoints> > allCombinations;
    permute(allCombinations, currentBlobs, candidateBlobs, temp, 0, currentBlobs.size(), candidateBlobs.size(), maxConnections);
    //std::cout << "size of permutation: " <<allCombinations.size() << std::endl;

    std::sort (allCombinations.begin(), allCombinations.end(), [](const std::vector<CorresPoints>& v1, const std::vector<CorresPoints>& v2){
        return v1.size() > v2.size();
    });

    if(allCombinations.size() != 0){

        int index(0);
        const unsigned updateSize = allCombinations[0].size();
        int minDist(-1);
        for (unsigned i = 0; i < allCombinations.size() ; ++i){
            if (allCombinations[i].size() == updateSize){

                int distSize = 0;

                for (CorresPoints corres : allCombinations[i]){
                    distSize += corres.mDist;
                }

                if(minDist == -1 || distSize < minDist){
                    index = i;
                    minDist = distSize;
                }

            }
        }
        for(unsigned j = 0 ; j < candidateBlobs.size(); ++j){
            bool found = false;
            for (CorresPoints cPt: allCombinations[index]){
                if (cPt.mIDCandidate == (int)j){
                    corresPts.push_back(cPt);
                    found = true;
                    break;
                }
            }

            if(!found){
                corresPts.push_back(CorresPoints(j, -1, 0));
            }
        }
    } else {
        for(unsigned j = 0 ; j < candidateBlobs.size(); ++j){
            corresPts.push_back(CorresPoints(j, -1, 0));
        }
    }

#if VIS
    cv::Mat img = cv::Mat::zeros(447, 655, CV_8U);
    cv::cvtColor(img, img, CV_GRAY2RGB);

    for (CorresPoints cPt: corresPts){
        if(cPt.mIDCurrent != -1) {
            cv::circle(img, candidateBlobs[cPt.mIDCandidate]->mPosition, 3, cv::Scalar(0,255,0));
            cv::circle(img, currentBlobs[cPt.mIDCurrent]->mPosition, 5, cv::Scalar(255,0,0));

            cv::line(img, candidateBlobs[cPt.mIDCandidate]->mPosition, currentBlobs[cPt.mIDCurrent]->mPosition, cv::Scalar(0,0,255), 1);
        }
    }
    cv::imshow("global Tracker", img);
    cv::waitKey();
#endif
}

int computeAngle(const cv::Vec2f& v1, const cv::Vec2f& v2)
{
    // need to be allready normalized!!
    cv::Vec2f a, b;
    cv::normalize(v1, a);
    cv::normalize(v2, b);
    return acos(a.dot(b))*180/M_PI;
}

void knnTrackerCombined(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts)
{
    std::vector<CorresPoints> KNN;
    std::vector<CorresPoints> KNNpredicted;
    knFingerTracker(currentBlobs, candidateBlobs, KNN, false);
    knFingerTracker(currentBlobs, candidateBlobs, KNNpredicted, true);

    std::vector <std::vector<CorresPoints>> solutions;
    solutions.push_back(KNN);
    solutions.push_back(KNNpredicted);

    int index = 0;
    checkBestTracking(solutions, index);

    corresPts = solutions[index];
}

void ICPTrackerCombined(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts, int maxIterations)
{
    std::vector<CorresPoints> ICP1;
    //std::vector<CorresPoints> ICP1Candidate;
    std::vector<CorresPoints> ICP2;
    std::vector<CorresPoints> ICPAll;

    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP1,            maxIterations, 1);
    //ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP1Candidate,   maxIterations, 1, false, true);

    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP2,            maxIterations, 2);
    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICPAll,          maxIterations, 5);

    std::vector <std::vector<CorresPoints>> solutions;
    solutions.push_back(ICP1);
    //solutions.push_back(ICP1Candidate);
    solutions.push_back(ICP2);
    solutions.push_back(ICPAll);

    int index = 0;
    checkBestTracking(solutions, index);

    corresPts = solutions[index];
}

void checkBestTracking(const std::vector<std::vector<CorresPoints> >& possibleCorresPts, int& index)
{
    int maxUpdates(0);

    std::vector<std::pair<int, int> > solutions;

    for (unsigned i = 0; i < possibleCorresPts.size(); ++i) {
        int updateCount = 0;
        std::vector<int> updateIDs;

        for (const CorresPoints& cPt : possibleCorresPts[i]) {
            if(cPt.mIDCurrent != -1 && cPt.mDist < HandModel::HandSpeedMax &&
               std::find(updateIDs.begin(), updateIDs.end(), cPt.mIDCurrent) == updateIDs.end()) {
                ++updateCount;
                updateIDs.push_back(cPt.mIDCurrent);
            }
        }

        solutions.push_back(std::make_pair(i, updateCount));

        if (updateCount >= maxUpdates) {
            maxUpdates = updateCount;
        }
    }


    int minDist(-1);

    for (unsigned i = 0; i < solutions.size(); ++i) {
        if(solutions[i].second == maxUpdates){
            int distSize = 0;
            std::vector<int> updateIDs;

            std::vector<CorresPoints> solution = possibleCorresPts[solutions[i].first];

            std::sort(solution.begin(), solution.end(), [](CorresPoints c1, CorresPoints c2) {return c1.mDist < c2.mDist;});

            for (CorresPoints corres : solution){
                if(corres.mIDCurrent != -1 && corres.mDist < HandModel::HandSpeedMax &&
                   std::find(updateIDs.begin(), updateIDs.end(), corres.mIDCurrent) == updateIDs.end()) {
                    updateIDs.push_back(corres.mIDCurrent);
                    distSize += corres.mDist;
                }
            }
            if(minDist == -1 || distSize < minDist){
                index = i;
                minDist = distSize;
            }
        }
    }
}

void ICPTrackerCombinedUnconfident(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts, int maxIterations)
{
    std::vector<CorresPoints> ICP1;
    std::vector<CorresPoints> ICP1Predicted;
    std::vector<CorresPoints> ICP2;
    std::vector<CorresPoints> ICP1Candidate;
    std::vector<CorresPoints> ICP1PredictedCandidate;
    std::vector<CorresPoints> ICP2Candidate;

    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP1,          maxIterations, 1, false, false);
    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP1Predicted, maxIterations, 1, true,  false);
    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP2,          maxIterations, 2, false, false);

    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP1Candidate,          maxIterations, 1, false, true);
    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP1PredictedCandidate, maxIterations, 1, true,  true);
    ICPTrackerRidgid(currentBlobs, candidateBlobs, ICP2Candidate,          maxIterations, 2, false, true);

    std::vector <std::vector<CorresPoints>> solutions;
    solutions.push_back(ICP1);
    solutions.push_back(ICP1Predicted);
    solutions.push_back(ICP2);

    solutions.push_back(ICP1Candidate);
    solutions.push_back(ICP1PredictedCandidate);
    solutions.push_back(ICP2Candidate);

    int index = 0;
    checkBestTracking(solutions, index);

    corresPts = solutions[index];
}

void minimalDistanceCombined(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts)
{
    std::vector<CorresPoints> MDT;
    std::vector<CorresPoints> MDTpredicted;
    minimalDistanceTracker(currentBlobs, candidateBlobs, MDT, false);
    minimalDistanceTracker(currentBlobs, candidateBlobs, MDTpredicted, true);

    std::vector <std::vector<CorresPoints>> solutions;
    solutions.push_back(MDT);
    solutions.push_back(MDTpredicted);

    int index = 0;
    checkBestTracking(solutions, index);

    corresPts = solutions[index];
}
