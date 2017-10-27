# mser-tracking

Optical hand and finger tracking (multitouch with diffuse illumination) using the MSER algorithm (maximally extremal regions) for hand and finger recognition and the ISP algorithm (iterative closest point) for finger tracking.

# QT project
# currently configured for a monochrome PTGrey Grasshopper 3 camera with external sync
# part of the camera image is currently masked to cope with specular reflections of a projector:
#  cv::circle(mImage, cv::Point (400,100), 220, cv::Scalar(0,0,0),-1); (see imageprocessing.cpp)

