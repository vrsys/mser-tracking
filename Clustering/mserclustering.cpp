#include "mserclustering.hpp"

#include "Tracking/trackingalgorithm.hpp"

MSERClustering::MSERClustering():
    mMinFingerParents(5),
    mMinLevel(120),
    mMaxSizeDiff(10),
    mMaxHandSizeDiff(300)
{
}

const std::vector<std::shared_ptr<MSERCluster> > MSERClustering::process(const std::shared_ptr<MSER::Region> root)
{
    std::vector<std::shared_ptr<MSERCluster> > clusters;

    std::vector<std::shared_ptr<MSER::Region> > hands;

    // find hand regions
    classificateHands(root, hands, 0);

    // check that hand regions do not overlap
    checkOutlierHands(hands);

    // find finger regions
    std::vector<std::pair<bool, std::shared_ptr<MSER::Region>> > blobRegions; // <qualified , MSER::Region>
    classificateFinger(root, root, 0, blobRegions);

    // sort hand by size (start with bigger one)
    std::sort(hands.begin(), hands.end(), [](std::shared_ptr<MSER::Region> h1, std::shared_ptr<MSER::Region> h2){
        return(h1->area_ > h2->area_);
    });

    // for each hand find arm and assign blobs.
    for (std::shared_ptr<MSER::Region> hand : hands){
        std::shared_ptr<MSERCluster> cluster = std::make_shared<MSERCluster>();
        cluster->mHand = Ellipse(hand);
        //cluster->mPosition = cluster->mHand.mPosition;

        std::shared_ptr<MSER::Region> arm (hand);
        classificateArm(cluster, hand->parent_.lock(), arm);
        if (arm == nullptr) arm = hand;

        cluster->mArm = Ellipse(arm);
        addBlobs(blobRegions, cluster, arm);
        clusters.push_back(cluster);
    }


#if 1
    // add unqualified blob to nearest cluster
    // todo: use finger direction to improve result
    for (std::pair<bool, std::shared_ptr<MSER::Region>>& b : blobRegions){
        if (b.first) continue;
        // find nearest cluster and add blob to hands
        int index(-1);
        cv::Point2i pos (b.second->moments_[0]  / b.second->area_, b.second->moments_[1]  / b.second->area_);
        int currentDist = findClosest(pos, clusters, index, true, false);

        // todo: find cluster that is in range and has allready blobs.
        if(index != -1 && currentDist < HandModel::F2H_DistanceMax*1.5) {
            clusters[index]->mBlobs.push_back(std::make_shared<Blob>(b.second));
            b.first = true;
        }
    }

#if 1
    // check for nearer cluster (hands)...
    std::vector<unsigned> delIndexe;
    for (unsigned i = 0; i < clusters.size(); ++i){
        if(clusters[i]->mBlobs.size() == 0) continue;
        int temp1;
        int closestI = findClosest(clusters[i]->mHand.mPosition, clusters[i]->mBlobs, temp1);
        for (unsigned j = 0; j < clusters.size(); ++j){
            if (i==j || clusters[j]->mBlobs.size() != 0) continue;

            int farthestJ = 0;
            for (std::shared_ptr<Blob> b : clusters[i]->mBlobs){
                int dist = getDistance(clusters[j]->mHand.mPosition, b->mPosition);
                if (dist > farthestJ){
                    farthestJ = dist;
                }
            }

            if(farthestJ != 0 && farthestJ < closestI){
                clusters[j]->mBlobs.insert(clusters[j]->mBlobs.end(), clusters[i]->mBlobs.begin(), clusters[i]->mBlobs.end());
                delIndexe.push_back(i);
            }
        }
    }
    std::sort(delIndexe.begin(), delIndexe.end(), [](unsigned i, unsigned j) {return i>j;});
    for (unsigned int del : delIndexe){
        clusters.erase(clusters.begin() + del);
    }
#endif

    checkOutlier(clusters);
    for (unsigned i = 0; i< clusters.size(); ++i){
        defineCenterPoint(clusters[i]);
    }
    checkClusterConfidence(clusters);

    // post process all cluster
    // for all unconfident cluster search for cluster in range and define as unconfident too
    for (std::shared_ptr<MSERCluster> cluster : clusters){
        if (cluster->mBlobs.size() == 0) continue;
        if(!cluster->mConfident){
            for (std::shared_ptr<MSERCluster> cluster2 : clusters){
                if(cluster->mBlobs.size() > 0 && getDistance(cluster->mHand.mPosition, cluster2->mHand.mPosition) <= HandModel::H2H_DistanceMax){
                    cluster2->mConfident = false;
                    cluster2->mPosition = cluster2->mHand.mPosition;
                }
            }
            cluster->mPosition = cluster->mHand.mPosition;
        } else {
            // delete finger that are to near to handpalm
            checkOutlier2(cluster);
            defineCenterPoint(cluster);
        }
    }
#endif

    return clusters;
}

void MSERClustering::classificateHands(std::shared_ptr<MSER::Region> node, std::vector<std::shared_ptr<MSER::Region> > &handsOut, int counter){
    if(!node) return;

    // classificate hand (do not use the first root node)
    if(counter != 0 && node->area_ > HandModel::HandAreaMin && node->area_ < HandModel::HandAreaMax &&  checkSimilarity(node->parent_.lock()->area_, node->area_) < mMaxHandSizeDiff){
        // define first node as arm
        if (handsOut.size() == 0){
            handsOut.push_back(node);
        } else {
            // check distance to other allready classified hands
            bool success = true;
            for (std::shared_ptr<MSER::Region> hand : handsOut){
                int distance = getDistance(hand, node);
                if (distance < HandModel::H2H_DistanceMin){
                    if (checkSimilarity(node->parent_.lock()->area_, node->area_) < checkSimilarity(hand->parent_.lock()->area_, hand->area_)){
                        hand = node;
                    }
                    success = false;
                }
            }
            if (success){
                handsOut.push_back(node);
            }
        }
    }
    classificateHands(node->child_, handsOut, ++counter);
    classificateHands(node->next_, handsOut, ++counter);
}

void MSERClustering::classificateArm(const std::shared_ptr<MSERCluster>& cluster, std::shared_ptr<MSER::Region> node, std::shared_ptr<MSER::Region>& arm)
{
    if(!node || !node->parent_.lock()) return;

    if (node->area_ < HandModel::ArmAreaMax && node->area_ > HandModel::ArmAreaMin){
        int distance = getDistance(cluster->mHand.mPosition, node);
        if (distance < HandModel::H2A_DistanceMax && Ellipse(arm).contains(Ellipse(node).mPosition)){
            arm = node;
        }
    }

    classificateArm(cluster, node->parent_.lock(), arm);
    classificateArm(cluster, node->next_,arm);
}


void MSERClustering::classificateFinger(std::shared_ptr<MSER::Region> prev, std::shared_ptr<MSER::Region> node, int similarNodes, std::vector<std::pair<bool, std::shared_ptr<MSER::Region>> >& regions)
{
    if(!prev || !node) return;

    // classificate finger
    if (node->area_ < HandModel::FingerAreaMax && node->area_ > HandModel::FingerAreaMin && !node->child_ &&
             (255-node->level_) > mMinLevel && checkSimilarity(prev->area_, node->area_) < mMaxSizeDiff &&
             checkTrustability(node)){
        regions.push_back(std::make_pair(false, node));
    }

    classificateFinger(node, node->child_, similarNodes, regions);
    classificateFinger(node, node->next_, similarNodes, regions);
}


void MSERClustering::addBlobs(std::vector<std::pair<bool, std::shared_ptr<MSER::Region> > >& regions, std::shared_ptr<MSERCluster> cluster, std::shared_ptr<MSER::Region> node)
{
    if(!node) return;

    if (node->area_ < HandModel::FingerAreaMax){
        for (std::pair<bool, std::shared_ptr<MSER::Region>>& reg : regions){
            if(reg.first == false && reg.second->moments_ == node->moments_ &&
               getDistance(cluster->mHand.mPosition, reg.second) < HandModel::F2H_DistanceMax*1.5){
                cluster->mBlobs.push_back(std::make_shared<Blob>(node));
                reg.first = true;
            }
        }
    }

    addBlobs (regions, cluster, node->child_);

    if (node->area_ < HandModel::HandAreaMax){
        addBlobs (regions, cluster, node->next_);
    }
}


int MSERClustering::countChilds(const std::shared_ptr<MSER::Region> &node) {
    if(!node->child_) return 0;

    std::shared_ptr<MSER::Region> currentNode = node->child_;
    int sum = 1;

    while(currentNode->next_){
        ++sum;
        currentNode=  currentNode->next_;
    }

    return sum;
}

int MSERClustering::checkSimilarity(int x, int y)
{
    return fabs(x-y);
}

int MSERClustering::getDistance(const cv::Point2i& p, const std::shared_ptr<MSER::Region> &reg2){
    if (!reg2){
        return -1;
    }

    return getDistance(p, cv::Point2i(reg2->moments_[0]  / reg2->area_, reg2->moments_[1]  / reg2->area_));
}

int MSERClustering::getDistance(const cv::Point2i& p1, const cv::Point2i& p2){
    return cv::norm(p1 - p2);
}

int MSERClustering::getDistance(const std::shared_ptr<MSER::Region> &reg1, const std::shared_ptr<MSER::Region> &reg2){
    if (!reg1 || !reg2){
        return -1;
    }
    return getDistance(cv::Point2i(reg1->moments_[0]  / reg1->area_, reg1->moments_[1]  / reg1->area_), cv::Point2i(reg2->moments_[0]  / reg2->area_, reg2->moments_[1]  / reg2->area_));
}

int MSERClustering::computeAngle(const cv::Vec2f& v1, const cv::Vec2f& v2) const
{
    // need to be allready normalized!!
    cv::Vec2f a = cv::normalize(v1);
    cv::Vec2f b = cv::normalize(v2);
    return acos(a.dot(b))*180/M_PI;
}

bool MSERClustering::checkTrustability(std::shared_ptr<MSER::Region> node)
{
    int i = 0;
    std::shared_ptr<MSER::Region> currenNode = node;
    while (currenNode->parent_.lock()){
        if (checkSimilarity(currenNode->area_, currenNode->parent_.lock()->area_) < mMaxSizeDiff){
            currenNode = currenNode->parent_.lock();
            ++i;
        } else {
            break;
        }
    }
    if(i >= mMinFingerParents ) {
        return true;
    }
    return false;
}

void MSERClustering::checkOutlier2(std::shared_ptr<MSERCluster> cluster) {
    // delete
    std::vector<unsigned> delIndexe;
    for (unsigned int i= 0; i < cluster->mBlobs.size() ; ++i){
        if (std::find(delIndexe.begin(), delIndexe.end(), i) != delIndexe.end()) continue;

        int distToCenter = getDistance(cluster->mBlobs[i]->mPosition, cluster->mHand.mPosition);
        if(distToCenter < HandModel::F2H_DistanceMin){
            //std::cout << "delete: blob do not fit to handModel (F2H_DistanceMin = " << distToCenter << ") "<< cluster->mBlobs[i]->mPosition << ", " << cluster->mHand.mPosition << std::endl;
            delIndexe.push_back(i);
            continue;
        }
    }

    // sort to garanty right delete order (no index change) todo: find other solution
    std::sort(delIndexe.begin(), delIndexe.end(), [](unsigned i, unsigned j) {return i>j;});
    for (unsigned int del : delIndexe){
        cluster->mBlobs.erase(cluster->mBlobs.begin() + del);
    }
}

void MSERClustering::checkOutlier(std::vector<std::shared_ptr<MSERCluster>> clusters)
{
    for (std::shared_ptr<MSERCluster> cluster : clusters){
        // delete
        std::vector<unsigned> delIndexe;
        for (unsigned int i= 0; i < cluster->mBlobs.size() ; ++i){
            if (std::find(delIndexe.begin(), delIndexe.end(), i) != delIndexe.end()) continue;

            // check for double classificated blobs (~same angle)
            for (unsigned int j= 0; j < cluster->mBlobs.size() ; ++j){
                if(i==j || std::find(delIndexe.begin(), delIndexe.end(), i) != delIndexe.end()
                        || std::find(delIndexe.begin(), delIndexe.end(), j) != delIndexe.end()) {
                    continue;
                }
                // compute distance.. delete if smaller than hand model min finger to finger distance
                int dist = getDistance(cluster->mBlobs[i]->mPosition, cluster->mBlobs[j]->mPosition);
                //int angle = computeAngle(cluster->mBlobs[i]->mOrientation, cluster->mBlobs[j]->mOrientation);
                if( dist < HandModel::F2F_DistanceMin ) {
                    //std::cout << "delete cluster blob, because it doesn't fit to hand model (F2F_DistanceMin = " << dist << ")" << std::endl;
                    // delete the shorter one
                    if(cv::norm(cluster->mHand.mPosition - cluster->mBlobs[i]->mPosition) < cv::norm(cluster->mHand.mPosition - cluster->mBlobs[j]->mPosition)){
                        delIndexe.push_back(i);
                    }else {
                        delIndexe.push_back(j);
                    }
                }
            }
        }

        // sort to garanty right delete order (no index change) todo: find other solution
        std::sort(delIndexe.begin(), delIndexe.end(), [](unsigned i, unsigned j) {return i>j;});
        for (unsigned int del : delIndexe){
            cluster->mBlobs.erase(cluster->mBlobs.begin() + del);
        }
    }
}

void MSERClustering::defineCenterPoint(std::shared_ptr<MSERCluster> cluster)
{
    if (cluster->mBlobs.size() == 0) {
        cluster->mPosition = cluster->mHand.mPosition;
        return;
    }

    cv::Point2i center(0,0);
    for (const std::shared_ptr<Blob> b : cluster->mBlobs){
        center += b->mPosition;
    }
    center += cluster->mHand.mPosition;

    cluster->mPosition = (cluster->mBlobs.size() < 3) ? cluster->mHand.mPosition : center/((int)cluster->mBlobs.size()+1);
}

void MSERClustering::checkOutlierHands(std::vector<std::shared_ptr<MSER::Region> >& hands)
{
    std::vector<unsigned> delIndex;
    for (unsigned i = 0; i < hands.size(); ++i){
        for (unsigned j = 0; j < hands.size(); ++j){
            if (i==j || std::find(delIndex.begin(), delIndex.end(), j) != delIndex.end()) continue;

            if (hands[i]->area_ > hands[j]->area_ && Ellipse(hands[i]).contains(Ellipse(hands[j]).mPosition)){
                //std::cout << "delete cluster " << j << "  size: " << hands[i]->area_ <<"  " << hands[j]->area_<< std::endl;
                delIndex.push_back(j);
                continue;
            }
        }
    }

    std::sort(delIndex.begin(), delIndex.end(), [](unsigned i, unsigned j) {return i>j;});
    for (unsigned int del : delIndex){
        hands.erase(hands.begin() + del);
    }
}

void MSERClustering:: checkClusterConfidence(std::vector<std::shared_ptr<MSERCluster> > clusters)
{
    for (unsigned i = 0; i < clusters.size(); ++i){
        if (clusters[i]->mBlobs.size() == 0) continue;

        bool gotomainloop = false;

        for (unsigned j = 0; j < clusters.size() && !gotomainloop; ++j){
            if (i == j || clusters[j]->mBlobs.size() == 0) continue;
            if (clusters[i]->mArm.mPosition == clusters[j]->mArm.mPosition){
                clusters[i]->mConfident = false;
                clusters[j]->mConfident = false;
                gotomainloop = true;
                //std::cout << "cluster do not fit to handModel (same arm)." << std::endl;
                break;
            }
        }

        if (gotomainloop) continue;

        if(clusters[i]->mBlobs.size() > 5) {
            //std::cout << "blob do not fit to handModel (number of fingers > 5)." << std::endl;
            clusters[i]->mConfident = false;
            break;
        }
        // check distances to hand center and other fingers.
        int meanDistCenter(0);
        for(unsigned j = 0; j < clusters[i]->mBlobs.size() && !gotomainloop; ++j) {

            int distF2H = getDistance(clusters[i]->mHand.mPosition, clusters[i]->mBlobs[j]->mPosition);
            meanDistCenter += distF2H;
            if (distF2H < HandModel::F2H_DistanceMin || distF2H > (HandModel::F2H_DistanceMax)) {
                //std::cout << "blob do not fit to handModel (F2H distance = " << distF2H << "("<< HandModel::F2H_DistanceMax<<")." << std::endl;
                clusters[i]->mConfident = false;
                gotomainloop = true;
                break;
            }

            for(unsigned j = 0; j < clusters[i]->mBlobs.size(); ++j) {
                if (j==j) continue;

                int distF2F = getDistance(clusters[i]->mBlobs[i]->mPosition, clusters[i]->mBlobs[j]->mPosition);
                if (distF2F < HandModel::F2F_DistanceMin || distF2F > HandModel::F2F_DistanceMax) {
                    //std::cout << "blob do not fit to handModel (F2F distance = " << distF2F << ")." << std::endl;
                    clusters[i]->mConfident = false;
                    gotomainloop = true;
                    break;
                }


                int angle = computeAngle(clusters[i]->mBlobs[j]->mOrientation, clusters[i]->mBlobs[j]->mOrientation);
                if( angle < HandModel::F2F_AngleMin ) {
                    clusters[i]->mConfident = false;
                    //std::cout << "blob do not fit to handModel (F2F angle = " << angle << ")." << std::endl;
                    gotomainloop = true;
                    break;
                }
            }
        }

        if (gotomainloop) continue;

        meanDistCenter /= clusters[i]->mBlobs.size();
        if(clusters[i]->mBlobs.size() > 3) {
            for(unsigned j = 0; j < clusters[i]->mBlobs.size() && !gotomainloop; ++j) {
                int distF2H = getDistance(clusters[i]->mHand.mPosition, clusters[i]->mBlobs[j]->mPosition);
                // all blobs need nearly same distance to palm

                if(distF2H < (meanDistCenter*0.5)){
                    clusters[i]->mConfident = false;
                    //std::cout << "blob do not fit to handModel (mean dist to handCenter = " << meanDistCenter << "("<< distF2H << "))." << std::endl;
                    gotomainloop = true;
                    break;
                }
            }
        }
    }
}

void MSERClustering::setParameter(int minParents, int maxSizeDiff, int minLevel, int maxHandSizeDiff)
{
    mMinFingerParents = minParents;
    mMaxSizeDiff = maxSizeDiff;
    mMinLevel = minLevel;
    mMaxHandSizeDiff = maxHandSizeDiff;
}

std::shared_ptr<MSERCluster> operator+(const std::shared_ptr<MSERCluster> cluster, const cv::Point2i &pos)
{
    std::shared_ptr<MSERCluster> c(cluster);

    c->mTopLeft += pos;
    c->mPosition += pos;
    c->mArm.mPosition += pos;
    c->mHand.mPosition += pos;
    for (std::shared_ptr<Blob> blob : c->mBlobs){
        blob->mPosition += pos;
        blob->mPredictedPosition += pos;
        blob->mFilteredPosition += pos;
    }

    return c;
}
