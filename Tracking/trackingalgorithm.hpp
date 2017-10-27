#ifndef TRACKINGALGORITHM
#define TRACKINGALGORITHM

#include <opencv2/opencv.hpp>

#include "blob.hpp"
#include "hand.hpp"
#include "handmodel.hpp"

struct CorresPoints {
    CorresPoints(int candidateID, int currentID, int dist)
        : mIDCandidate(candidateID), mIDCurrent(currentID), mDist(dist)
    {}

    int mIDCandidate;
    int mIDCurrent;
    int mDist;

    friend bool operator == (const CorresPoints& cP1, const CorresPoints& cP2);
};

inline bool operator==(const CorresPoints& cP1, const CorresPoints& cP2) {
    if (cP1.mIDCurrent == cP2.mIDCurrent && cP1.mIDCandidate == cP2.mIDCandidate && cP1.mDist == cP2.mDist) return true;
    else return false;
}

int computeDistance(const cv::Point2i &p1, const cv::Point2i &p2);
int computeAngle(const cv::Vec2f& v1, const cv::Vec2f& v2);

// tracking algo
int findClosest(const cv::Point2i &p, const std::vector<cv::Point2i> &points, int &index);
int findClosest(const cv::Point2i &p, const std::vector<std::shared_ptr<Blob> > &blobs, int &index);
int findClosest(const cv::Point2i &p, const std::vector<std::shared_ptr<MSERCluster> >& clusters, int& index, bool palm = false, bool fingers = false);
int findClosest(const cv::Point2i &p, const std::vector<std::shared_ptr<Hand> >& hands, int &index, int mode=false);
int findClosest(const std::shared_ptr<MSERCluster> &cluster, const std::vector<std::shared_ptr<Hand> >& hands, int& index);
int findClosest(const std::shared_ptr<Hand> & hand, const std::vector<std::shared_ptr<MSERCluster> > &clusters, int& index);

// find best tracking. choose solution with most updates and shortest total distance
void checkBestTracking(const std::vector<std::vector<CorresPoints> >& possibleCorresPts, int& index);


/*** GLOBAL - TRACKER ***/
void permute(std::vector<std::vector<CorresPoints> >& allCombinations ,
             const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs,
             std::vector<CorresPoints> currentSolution, int indexFinger, unsigned currentSize, unsigned candidateSize, unsigned maxConnections = 3);
void globalBlobTracker(const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs, std::vector<CorresPoints>& corresPts, int maxConnections = 3);


/*** Minimal Distance Tracker ***/
void minimalDistanceTracker(const std::vector<std::shared_ptr<Blob> > & currentBlobs, const std::vector<std::shared_ptr<Blob> > & candidateBlobs, std::vector<CorresPoints>& corresPts, bool predictedPos);
void minimalDistanceCombined(const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs, std::vector<CorresPoints>& corresPts);


/*** K - nearest - Neigbor ***/
void knFingerTracker(const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs, std::vector<CorresPoints>& corresPts, bool predictedPos = true);
void knFingerTracker(const std::vector<cv::Point2i>& currentPoints, const std::vector<cv::Point2i>& candidatePoints, std::vector<CorresPoints>& corresPts);
void knCandidateTracker(const std::vector<cv::Point2i>& candidatePoints, const std::vector<cv::Point2i>& currentPoints, std::vector<CorresPoints>& corresPts);
void knnTrackerCombined(const std::vector<std::shared_ptr<Blob>>& currentBlobs, const std::vector<std::shared_ptr<Blob>>& candidateBlobs, std::vector<CorresPoints>& corresPts);


/*** ICP ***/
cv::Mat estimateSVDTransform(const std::vector<cv::Point2i>& currentPoints, const std::vector<cv::Point2i>& candidates);
void ICPTrackerTrans(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> > & candidateBlobs, std::vector<CorresPoints>& CorresPoints, int maxIterations = 10);
void ICPTrackerRidgid(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs,
                      std::vector<CorresPoints>& corresPts, int maxIterations = 5,
                      int maxConnections = 1, bool predicted = false, bool candidateTracker = false);
void ICPTrackerCombined(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> > & candidateBlobs, std::vector<CorresPoints>& corresPts, int maxIterations = 10);
void ICPTrackerCombinedUnconfident(const std::vector<std::shared_ptr<Blob> >& currentBlobs, const std::vector<std::shared_ptr<Blob> >& candidateBlobs, std::vector<CorresPoints>& corresPts, int maxIterations = 10);
#endif // TRACKINGALGORITHM

