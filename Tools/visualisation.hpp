#ifndef VISUALISATION_H
#define VISUALISATION_H

#include <opencv2/opencv.hpp>

#include "Processing/ImageProcessing/MSER/mser.hpp"
#include "Clustering/mserclustering.hpp"
#include "Processing/ImageProcessing/ConnectedComponents/connectedcomponents.hpp"

#include "hand.hpp"


void drawComponentTree(cv::Mat& img, const std::shared_ptr<MSER::Region> &root, const cv::Point2i& pos = cv::Point2i(0,0), cv::Scalar const& color = cv::Scalar(255,255,0));

void drawCCComponent(cv::Mat &img, const CC::Component& comp, const cv::Point2i pos  = cv::Point2i(0,0), const cv::Scalar& color = cv::Scalar(255,255,255));
void drawComponent(cv::Mat &img, const std::shared_ptr<MSER::Region> &node, const cv::Point2i& pos = cv::Point2i(0,0), cv::Scalar const& color = cv::Scalar(255,255,0));

void drawCluster(cv::Mat &img, const std::shared_ptr<MSERCluster> cluster, cv::Scalar const& color = cv::Scalar(255,255,0));
void drawHands(cv::Mat &img, const std::vector<std::shared_ptr<Hand> >& hands, cv::Scalar color = cv::Scalar(255,255,0));

#endif // VISUALISATION_H
