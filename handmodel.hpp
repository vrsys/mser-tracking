#ifndef HANDMODELL
#define HANDMODELL

#include <iostream>

#include <opencv2/opencv.hpp>

#include "Tools/config.hpp"

struct HandModel {
    static void initHandModel(std::string configFile);
    static void defineResolution(cv::Size resolution, cv::Size tableSize);
    static void setCurrentFPS(int FPS);
    static void print();

    // pixel - cm ratio
    static float PixelsIn1CM;

    // classification
    static int ArmAreaMin;
    static int ArmAreaMax;
    static int HandAreaMin;
    static int HandAreaMax;
    static int FingerAreaMax;
    static int FingerAreaMin;

    static int H2A_DistanceMax;
    static int H2H_DistanceMin;
    static int F2H_DistanceMin;
    static int F2H_DistanceMax;

    static float F2F_DistanceMin;
    static int F2F_DistanceMax;
    static int F2F_AngleMin;

    // tracking
    static int HandSpeedMaxPerS;
    static int HandSpeedMax;
    static int HandRotationMaxPerS;
    static int HandRotationMax;
    static int BlobSpeedMaxPerS;
    static int BlobSpeedMax;
    static int H2H_DistanceMax;

    static std::string mConfigPath;
};

#endif // HANDMODELL

