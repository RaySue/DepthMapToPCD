
#include "pixel_tool.h"

PixelTool::PixelTool(const CameraIntrinsics &intrinsics, const CameraExtrinsics &cameraExtrinsicssics) : cameraIntrinsics(intrinsics), cameraExtrinsics(cameraExtrinsicssics)
{

}

void PixelTool::pixel2point(float *point, const int *pixel, float depth, int mode)
{

    float x = (pixel[0] - cameraIntrinsics.cx) / cameraIntrinsics.fx;
    float y = (pixel[1] - cameraIntrinsics.cy) / cameraIntrinsics.fy;

    if(mode == DISTORTION_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + cameraIntrinsics.coeffs[0]*r2 + cameraIntrinsics.coeffs[1]*r2*r2 + cameraIntrinsics.coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*cameraIntrinsics.coeffs[2]*x*y + cameraIntrinsics.coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*cameraIntrinsics.coeffs[3]*x*y + cameraIntrinsics.coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }

    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}