QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

#TARGET = mser-tracking

TEMPLATE = app
CONFIG += console

#QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -std=c++11 -fopenmp -fPIC -g -fexpensive-optimizations -D_GNULINUX -O3


INCLUDEPATH += /opt/flycapture-sdk/include /opt/OpenCV/opencv-3.0.0/install/include /opt/OpenCV/opencv-3.0.0/install/include/opencv2/

unix:!macx: LIBS += -L/opt/OpenCV/opencv-3.0.0/install/lib \
                    -L/opt/flycapture-sdk/lib \
                    -l:libopencv_objdetect.so.3.0 \
                    -l:libopencv_videoio.so.3.0 \
                    -l:libopencv_calib3d.so.3.0 \
                    -l:libopencv_features2d.so.3.0 \
                    -l:libopencv_highgui.so.3.0 \
                    -l:libopencv_flann.so.3.0 \
                    -l:libopencv_imgcodecs.so.3.0 \
                    -l:libopencv_imgproc.so.3.0 \
                    -l:libopencv_core.so.3.0 \
                    -lboost_system \
                    -l:libflycapture.so \
                    -fopenmp


SOURCES += main.cpp \
    Network/tuiosender.cpp \
    Processing/ImageProcessing/ImagePreProcessing/rectification.cpp \
    Processing/ImageProcessing/ImagePreProcessing/backgroundsubstraction.cpp \
    Classification/classification.cpp \
    InputDevice/CaptureDevice/grasshoppercamera.cpp \
    Processing/ImageProcessing/imageprocessing.cpp \
    Processing/ImageProcessing/ImagePreProcessing/imagepreprocessing.cpp \
    Tools/tools.cpp \
    Tracking/handtracker.cpp \
    InputDevice/CaptureDevice/imagefolder.cpp \
    UI/imageprocessingwindow.cpp \
    Processing/ImageProcessing/MSER/mser.cpp \
    Processing/ImageProcessing/ConnectedComponents/connectedcomponents.cpp \
    Tools/visualisation.cpp \
    UI/calibrationwindow.cpp \
    Processing/ImageProcessing/Calibration/calibration.cpp \
    Network/Osc/OscOutboundPacketStream.cpp \
    Network/Osc/OscReceivedElements.cpp \
    Network/Osc/OscTypes.cpp \
    UI/createmaxwindow.cpp \
    Tracking/trackingalgorithm.cpp \
    handmodel.cpp \
    Tools/config.cpp \
    global.cpp \
    InputDevice/CaptureDevice/videofile.cpp \
    InputDevice/CaptureDevice/cameradevice.cpp \
    Clustering/mserclustering.cpp

HEADERS += \
    Network/Osc/MessageMappingOscPacketListener.h \
    Network/Osc/OscException.h \
    Network/Osc/OscHostEndianness.h \
    Network/Osc/OscOutboundPacketStream.h \
    Network/Osc/OscPacketListener.h \
    Network/Osc/OscReceivedElements.h \
    Network/Osc/OscTypes.h \
    Classification/classification.hpp \
    InputDevice/CaptureDevice/capturedevice.hpp \
    InputDevice/CaptureDevice/imagefolder.hpp \
    InputDevice/CaptureDevice/grasshoppercamera.hpp \
    Network/tuiosender.hpp \
    Processing/ImageProcessing/Calibration/calibration.hpp \
    Processing/ImageProcessing/ConnectedComponents/connectedcomponents.hpp \
    Processing/ImageProcessing/Filter/doubleexponentialsmoothing.hpp \
    Processing/ImageProcessing/Filter/oneeurofilter.hpp \
    Processing/ImageProcessing/ImagePreProcessing/backgroundsubstraction.hpp \
    Processing/ImageProcessing/ImagePreProcessing/imagepreprocessing.hpp \
    Processing/ImageProcessing/ImagePreProcessing/rectification.hpp \
    Processing/ImageProcessing/MSER/ellipse.hpp \
    Processing/ImageProcessing/MSER/mser.hpp \
    Processing/ImageProcessing/imageprocessing.hpp \
    Tools/config.hpp \
    Tools/tools.hpp \
    Tools/visualisation.hpp \
    Tracking/handtracker.hpp \
    Tracking/trackingalgorithm.hpp \
    UI/calibrationwindow.hpp \
    UI/createmaxwindow.hpp \
    UI/imageprocessingwindow.hpp \
    blob.hpp \
    global.hpp \
    hand.hpp \
    handmodel.hpp \
    InputDevice/CaptureDevice/videofile.h \
    InputDevice/CaptureDevice/cameradevice.h \
    Clustering/mserclustering.hpp

FORMS += \
    UI/imageprocessingwindow.ui \
    UI/calibrationwindow.ui \
    UI/createmaxwindow.ui

DISTFILES += \
    Data/handmodel \
    config \
    handmodel
