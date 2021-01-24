//
// Created by surui on 2020/09/06.
//

#ifndef PIXEL2POINT_CAMERA_PARAMETER_H
#define PIXEL2POINT_CAMERA_PARAMETER_H

/* 内参 */
struct CameraIntrinsics
{
    int           width;     /* 图像宽 */
    int           height;    /* 图像高*/
    int           factor;    /* 缩放因子 (对深度图像，变化多大等于实际1米)*/
    float         cx;       /* 相机中心横坐标 */
    float         cy;       /* 相机中心纵坐标 */
    float         fx;        /* fx = f / dx (dx为像素x轴方向物理尺寸)*/
    float         fy;        /* fy = f / dy (dy为像素y轴方向物理尺寸)*/
    float         coeffs[5]; /* 畸变系数 (k1, k2, p1, p2, k3) */
};

/* 外参 */
struct CameraExtrinsics
{
    float rotation[9];    /**< 3x3 rotation matrix */
    float translation[3]; /**< translation vector, in meters */
};

#endif //PIXEL2POINT_CAMERA_PARAMETER_H