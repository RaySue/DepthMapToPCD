#ifndef PIXEL2POINT_PIXEL_TOOL_H
#define PIXEL2POINT_PIXEL_TOOL_H

#include <math.h>
#include "camera_parameter.h"
#include <iostream>

class PixelTool {
public:
    PixelTool(const CameraIntrinsics& intrinsics, const CameraExtrinsics& extrinsics);

    /* 反投影像素坐标 --> 世界坐标 */
    void pixel2point(float point[3], const int pixel[2], float depth, int mode = DISTORTION_NONE);

public:
    enum{
        DISTORTION_NONE,
        DISTORTION_BROWN_CONRADY
    };
private:
    CameraIntrinsics cameraIntrinsics;
    CameraExtrinsics cameraExtrinsics;
};


#endif //PIXEL2POINT_PIXEL_TOOL_H
