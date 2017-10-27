#include "handmodel.hpp"


// all values has to be in pixel dimensions.
// current are good for 600x400 images.

std::string HandModel::mConfigPath("");

float HandModel::PixelsIn1CM(5.767);

// all values specified in cm.
int       HandModel::ArmAreaMin(7 * 12);
int       HandModel::ArmAreaMax(16 * 10);
int       HandModel::HandAreaMin(5 * 5);
int       HandModel::HandAreaMax(7 * 12);

int       HandModel::FingerAreaMax(1 * 1);
int       HandModel::FingerAreaMin(0.5 * 0.5);

int       HandModel::H2A_DistanceMax(20);
int       HandModel::H2H_DistanceMin(10);
int       HandModel::F2H_DistanceMin(0);
int       HandModel::F2H_DistanceMax(10);

float       HandModel::F2F_DistanceMin(1);
int       HandModel::F2F_DistanceMax(23);
int       HandModel::F2F_AngleMin(1); //degree

// cm per second
int       HandModel::HandSpeedMaxPerS(300);
int       HandModel::HandSpeedMax(20);
int       HandModel::HandRotationMaxPerS(90);
int       HandModel::HandRotationMax(10);


int       HandModel::BlobSpeedMaxPerS(250);
int       HandModel::BlobSpeedMax(20);

/**** independent from resolution *****/

// while wrong clustering...
int       HandModel::H2H_DistanceMax(F2F_DistanceMax/3);


void HandModel::initHandModel(std::string configFile)
{
    mConfigPath = configFile;

    Config config(configFile);
    if(!config.successLoad()){
        std::cout << "couldn't load Data/handmodel config. Use default values." << std::endl;
        return;
    }
    bool success = true;

    ArmAreaMin          = atoi(config.getValue("ArmAreaMin",          success).c_str());
    ArmAreaMax          = atoi(config.getValue("ArmAreaMax",          success).c_str());
    HandAreaMin         = atoi(config.getValue("HandAreaMin",         success).c_str());
    HandAreaMax         = atoi(config.getValue("HandAreaMax",         success).c_str());
    FingerAreaMax       = atoi(config.getValue("FingerAreaMax",       success).c_str());
    FingerAreaMin       = atoi(config.getValue("FingerAreaMin",       success).c_str());
    H2A_DistanceMax     = atoi(config.getValue("H2A_DistanceMax",     success).c_str());
    H2H_DistanceMin     = atoi(config.getValue("H2H_DistanceMin",     success).c_str());
    F2H_DistanceMin     = atoi(config.getValue("F2H_DistanceMin",     success).c_str());
    F2H_DistanceMax     = atoi(config.getValue("F2H_DistanceMax",     success).c_str());
    F2F_DistanceMin     = atof(config.getValue("F2F_DistanceMin",     success).c_str());
    F2F_DistanceMax     = atoi(config.getValue("F2F_DistanceMax",     success).c_str());
    F2F_AngleMin        = atoi(config.getValue("F2F_AngleMin",        success).c_str());
    HandSpeedMaxPerS    = atoi(config.getValue("HandSpeedMaxPerS",    success).c_str());
    HandRotationMaxPerS = atoi(config.getValue("HandRotationMaxPerS", success).c_str());
    BlobSpeedMaxPerS    = atoi(config.getValue("BlobSpeedMaxPerS",    success).c_str());

    HandSpeedMax    = HandSpeedMaxPerS    / 30; // supposed 30 FPS (updated each frame)
    HandRotationMax = HandRotationMaxPerS / 30; // supposed 30 FPS (updated each frame)
    BlobSpeedMax    = BlobSpeedMaxPerS    / 30; // supposed 30 FPS (updated each frame)

    H2H_DistanceMax = F2F_DistanceMax * .75;

    if (success){
        //std::cout << "successfully loaded handmodel." << std::endl;
    } else {
        std::cout << "failed to load all handmodel data.  (" << mConfigPath << ")" << std::endl;
    }
}

void HandModel::defineResolution(cv::Size resolution, cv::Size tableSize)
{
    initHandModel(mConfigPath);
    PixelsIn1CM = ((float)resolution.width / (float)tableSize.width + (float)resolution.height / (float)tableSize.height) / 2.0;

    // area
    ArmAreaMin       *= (PixelsIn1CM*PixelsIn1CM);
    ArmAreaMax       *= (PixelsIn1CM*PixelsIn1CM);
    HandAreaMin      *= (PixelsIn1CM*PixelsIn1CM);
    HandAreaMax      *= (PixelsIn1CM*PixelsIn1CM);
    FingerAreaMax    *= (PixelsIn1CM*PixelsIn1CM);
    FingerAreaMin    *= (PixelsIn1CM*PixelsIn1CM);

    H2A_DistanceMax  *= PixelsIn1CM;
    H2H_DistanceMin  *= PixelsIn1CM;
    F2H_DistanceMin  *= PixelsIn1CM;
    F2H_DistanceMax  *= PixelsIn1CM;
    F2F_DistanceMin  *= PixelsIn1CM;
    F2F_DistanceMax  *= PixelsIn1CM;

    HandSpeedMaxPerS *= PixelsIn1CM;
    BlobSpeedMaxPerS *= PixelsIn1CM;

    HandSpeedMax = HandSpeedMaxPerS       / 30; // supposed 30 FPS
    HandRotationMax = HandRotationMaxPerS / 30; // supposed 30 FPS (updated each frame)
    BlobSpeedMax = BlobSpeedMaxPerS       / 30; // supposed 30 FPS

    H2H_DistanceMax = F2F_DistanceMax * .75;
}

void HandModel::setCurrentFPS(int FPS)
{
    HandSpeedMax    = HandSpeedMaxPerS    / FPS;
    HandRotationMax = HandRotationMaxPerS / FPS;
    BlobSpeedMax    = BlobSpeedMaxPerS    / FPS;
}

void HandModel::print()
{
    std::cout << "ArmAreaMin " << "  " << HandModel::ArmAreaMin << std::endl;
    std::cout << "ArmAreaMax " << "  " << HandModel::ArmAreaMax << std::endl;
    std::cout << "HandAreaMin " << "  " << HandModel::HandAreaMin << std::endl;
    std::cout << "HandAreaMax " << "  " << HandModel::HandAreaMax << std::endl;
    std::cout << "FingerAreaMax " << "  " << HandModel::FingerAreaMax << std::endl;
    std::cout << "FingerAreaMin " << "  " << HandModel::FingerAreaMin << std::endl;
    std::cout << "H2A_DistanceMax " << "  " << HandModel::H2A_DistanceMax << std::endl;
    std::cout << "H2H_DistanceMin " << "  " << HandModel::H2H_DistanceMin << std::endl;
    std::cout << "F2H_DistanceMin " << "  " << HandModel::F2H_DistanceMin << std::endl;
    std::cout << "F2H_DistanceMax " << "  " << HandModel::F2H_DistanceMax << std::endl;
    std::cout << "F2F_DistanceMin " << "  " << HandModel::F2F_DistanceMin << std::endl;
    std::cout << "F2F_DistanceMax " << "  " << HandModel::F2F_DistanceMax << std::endl;
    std::cout << "F2F_AngleMin " << "  " << HandModel::F2F_AngleMin << std::endl;
    std::cout << "HandSpeedMaxPerSeconde " << "  " << HandModel::HandSpeedMaxPerS << std::endl;
    std::cout << "HandSpeedMax (approx) " << "  " << HandModel::HandSpeedMax << "  (updated secondly)." << std::endl;
    std::cout << "BlobSpeedMaxPerSeconde " << "  " << HandModel::BlobSpeedMaxPerS << std::endl;
    std::cout << "BlobSpeedMax (approx) " << "  " << HandModel::BlobSpeedMax << "  (updated secondly)." << std::endl;
    std::cout << "H2H_DistanceMax " << "  " << HandModel::H2H_DistanceMax << std::endl;
}

