#include "grasshoppercamera.hpp"

GrasshopperCamera::GrasshopperCamera()
    : mStatusSuccessful(false),
      mPause(false),
      mError(),
      mCam(),
      mRawImage(),
      mMonoImage(),
      mTriggerMode(),
      mFmt7Mode(),
      mFmt7PixFmt(),
      mFmt7Info(),
      mFmt7ImageSettings(),
      mFmt7PacketInfo()
{
    std::cout << "GrasshopperCamera: create instance" << std::endl;

    // Camera setup
    printBuildInfo();

    FlyCapture2::BusManager busMgr;
    unsigned int numCameras;
    mError = busMgr.GetNumOfCameras(&numCameras);
    mStatusSuccessful = checkError("GetNumOfCameras", mError);

    std::cout << "MESSAGE: Number of cameras detected: " << numCameras << std::endl;

    if (numCameras < 1)
    {
        //throw std::runtime_error("ERROR: Insufficient number of cameras.");
        std::cout << "ERROR: Insufficient number of cameras." << std::endl;
        return;
    }

    // Select camera
    FlyCapture2::PGRGuid guid;
    mError = busMgr.GetCameraFromIndex(0, &guid); // take first camera
    mStatusSuccessful = checkError("getCameraFromIndex", mError);


    // Connect to a camera
    mError = mCam.Connect(&guid);
    mStatusSuccessful = checkError("Connect", mError);

    configureCamera();

    mError = mCam.StartCapture();
    mStatusSuccessful = checkError("StartCapture", mError);

    mError = mCam.RetrieveBuffer(&mRawImage);
    mStatusSuccessful = checkError("RetrieveBuffer", mError);

    mCamWidth  = mRawImage.GetCols();
    mCamHeight = mRawImage.GetRows();
}

GrasshopperCamera::~GrasshopperCamera()
{
    disconnect();
}

const cv::Mat &GrasshopperCamera::capture()
{
    if(mPause) return mImage;

    // Retrieve an image
    mError = mCam.RetrieveBuffer(&mMonoImage);
    checkError("RetrieveBuffer", mError);

    // convert to mono --> necessary?
    //mRawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &mMonoImage );

    // convert to OpenCV Mat
    unsigned int rowBytes = (double)mMonoImage.GetReceivedDataSize()/(double)mMonoImage.GetRows();
    mImage = cv::Mat(mMonoImage.GetRows(), mMonoImage.GetCols(), CV_8U, mMonoImage.GetData(), rowBytes);

    return mImage;
}

void GrasshopperCamera::pause()
{
    mPause = true;
}

void GrasshopperCamera::resume()
{
    mPause = false;
}

void GrasshopperCamera::next()
{
    mPause = false;
    capture();
    mPause = true;
}



void GrasshopperCamera::configureCamera()
{
    setCameraSettings();

    setFormat7Settings();

    setTriggerMode();
}

void GrasshopperCamera::setCameraSettings()
{
    FlyCapture2::Property prop;

    // ___BRIGHTNESS
    prop.type = FlyCapture2::BRIGHTNESS;
    mError = mCam.GetProperty( &prop );
    checkError("BRIGHTNESS", mError);
    prop.autoManualMode = false;
    //prop.valueA = 0;  // 0 - 511
    prop.absControl = true;
    prop.absValue =0.0;
    mError = mCam.SetProperty( &prop );
    checkError("BRIGHTNESS", mError);

    // ___AUTO_EXPOSURE
    prop.type = FlyCapture2::AUTO_EXPOSURE;
    mError = mCam.GetProperty( &prop );
    checkError("AUTO_EXPOSURE", mError);
    prop.autoManualMode = false;
    prop.valueA = 50;  // 0 - 1023
    mError = mCam.SetProperty( &prop );
    checkError("AUTO_EXPOSURE", mError);

    // ___GAIN
    prop.type = FlyCapture2::GAIN;
    mError = mCam.GetProperty( &prop );
    checkError("GAIN", mError);
    prop.autoManualMode = false;
    prop.absControl = true;
    prop.absValue = 9.0;
    mError = mCam.SetProperty( &prop );
    checkError("GAIN", mError);

    // ___SHUTTER
    prop.type = FlyCapture2::SHUTTER;
    mError = mCam.GetProperty( &prop );
    checkError("SHUTTER", mError);
    prop.autoManualMode = false;
    prop.absControl = true;
    prop.absValue = 2.5; // 0 - 4095
    mError = mCam.SetProperty( &prop );
    checkError("SHUTTER", mError);
}

void GrasshopperCamera::setFormat7Settings()
{
    // Format 7 configurations
    bool supported;
    mFmt7Mode = FlyCapture2::MODE_1; // 1x1 binning
    mFmt7Info.mode = mFmt7Mode;
    mError = mCam.GetFormat7Info( &mFmt7Info, &supported);

    checkError("GetFormat7Info", mError);
    if (!supported){
        throw std::runtime_error("PointGreyCamera::setFormat7 Format 7 mode not supported on this camera.");
    }

    // Make Format7 Configuration
    mFmt7ImageSettings.mode = mFmt7Mode;
    mFmt7ImageSettings.offsetX = 0;
    mFmt7ImageSettings.offsetY = 0;
    mFmt7ImageSettings.width = mFmt7Info.maxWidth;
    mFmt7ImageSettings.height = mFmt7Info.maxHeight;
    mFmt7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;

    // Validate the settings to make sure that they are valid
    bool valid;
    mError = mCam.ValidateFormat7Settings(&mFmt7ImageSettings, &valid, &mFmt7PacketInfo );
    checkError("ValidateFormat7Settings", mError);
    if (!valid)
    {
        throw std::runtime_error("PointGreyCamera::setFormat7 Format 7 Settings Not Valid.");
    }

    // Stop the camera to allow settings to change.
    mError = mCam.SetFormat7Configuration(&mFmt7ImageSettings, mFmt7PacketInfo.recommendedBytesPerPacket);
    checkError("SetFormat7Configuration", mError);

    printFormat7Capabilities(mFmt7Info);
}

bool GrasshopperCamera::getStatus() const
{
    return mStatusSuccessful;
}


void GrasshopperCamera::setTriggerMode()
{
    //-----------------------------------------------------------------------
    // Check for external trigger support
    //-----------------------------------------------------------------------
    FlyCapture2::TriggerModeInfo triggerModeInfo;
    mError = mCam.GetTriggerModeInfo(&triggerModeInfo);
    checkError("GetTriggerModeInfo", mError);

    if (triggerModeInfo.present != true)
    {
        std::cout << std::endl << "Camera does not support external trigger! Exiting...";
    }

    // Get current trigger settings
    mError = mCam.GetTriggerMode(&mTriggerMode);
    checkError("GetTriggerMode", mError);

    // Set camera to trigger mode 0
    mTriggerMode.onOff = true;
    mTriggerMode.mode = 0;
    mTriggerMode.parameter = 0;
    mTriggerMode.polarity = 1;

    // Triggering the camera externally using source 0.
    mTriggerMode.source = 0;

    mError = mCam.SetTriggerMode(&mTriggerMode);
    checkError("SetTriggerMode", mError);

    std::cout << "Trigger the camera by sending a trigger pulse to GPIO_" << mTriggerMode.source << std::endl;
}

void GrasshopperCamera::disconnect()
{
    mError = mCam.StopCapture();
    mCam.Disconnect();
}

bool GrasshopperCamera::checkError(const std::string &prefix, FlyCapture2::Error &error) const{
    if (error == FlyCapture2::PGRERROR_TIMEOUT){
        //throw std::runtime_error("PointGreyCamera: Failed to retrieve buffer within timeout.");
        std::cout << "PointGreyCamera: Failed to retrieve buffer within timeout." << std::endl;
        return false;
    } else if (error != FlyCapture2::PGRERROR_OK){ // If there is actually an error (PGRERROR_OK means the function worked as intended...)
        std::string start(" | FlyCapture2::ErrorType ");
        std::stringstream out;
        out << error.GetType();
        std::string desc(error.GetDescription());
        //throw std::runtime_error(prefix + start + out.str() + " " + desc);
        std::cout << prefix + start + out.str() + " " + desc << std::endl;
        return false;
    }
    return true;
}

void GrasshopperCamera::printBuildInfo()
{
    FlyCapture2::FC2Version fc2Version;
    FlyCapture2::Utilities::GetLibraryVersion(&fc2Version);

    std::cout << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build << std::endl;
    std::cout << "Application build date: " << __DATE__ << " " << __TIME__ << std::endl << std::endl;
}

void GrasshopperCamera::printCameraInfo(FlyCapture2::CameraInfo* camInfo)
{
    std::cout << "*** CAMERA INFORMATION ***" << std::endl;
    std::cout << "Serial number -" << camInfo->serialNumber << std::endl;
    std::cout << "Camera model - " << camInfo->modelName << std::endl;
    std::cout << "Camera vendor - " << camInfo->vendorName << std::endl;
    std::cout << "Sensor - " << camInfo->sensorInfo << std::endl;
    std::cout << "Resolution - " << camInfo->sensorResolution << std::endl;
    std::cout << "Firmware version - " << camInfo->firmwareVersion << std::endl;
    std::cout << "Firmware build time - " << camInfo->firmwareBuildTime << std::endl << std::endl;
    //std::cerr << "PixelMode - " << camInfo->      << std::endl << std::endl;
}

void GrasshopperCamera::printFormat7Capabilities(FlyCapture2::Format7Info info){
    std::cout << "*** Format 7 information ***" << std::endl;
    std::cout << "max packet size: " << info.maxPacketSize << std::endl;
    std::cout << "mode: " << info.mode << std::endl;
    std::cout << "packet size: " << info.packetSize << std::endl;
    std::cout << "format bit field: " << info.pixelFormatBitField << std::endl << std::endl;
}

bool GrasshopperCamera::pollForTriggerReady(FlyCapture2::Camera* cam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    FlyCapture2::Error error;
    unsigned int regVal = 0;
    do
    {
        error = cam->ReadRegister(k_softwareTrigger, &regVal);
        checkError("ReadRegister", error);
    } while ( (regVal >> 31) != 0 );
    return true;
}
