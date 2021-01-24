#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "pixel_tool.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

using namespace std;
using namespace cv;

void getRealSenseD415Int_720p(CameraIntrinsics& param, CameraExtrinsics& param2)
{
    param.width = 1280;
    param.height = 720;
    param.factor = 1000;
    param.fx = 940.173;  //焦距：内参矩阵中的　f/dx
    param.fy = 940.173;  //焦距：内参矩阵中的　f/dy
    param.cx = 635.389;  //内参矩阵中的图像中心的横坐标u0
    param.cy = 364.28;

    param2.rotation[0] = 0;
}

// 距离地面再高一点
const int &floorHeight = 4000;

pcl::PointCloud<pcl::PointXYZ>::Ptr depth2pc(const cv::Mat& depthImg, const std::string& path, PixelTool& ptool)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //点云ptr;

    for(int i = 0; i < depthImg.rows; ++i)
    {
        const ushort* pDepth = depthImg.ptr<ushort>(i);

        for(int j = 0; j < depthImg.cols; ++j)
        {
            if(pDepth[j] == 0) continue;
//            if(pDepth[j] > floorHeight) continue;
            if(pDepth[j] > 2900) continue;

            pcl::PointXYZ p;

            float point3d[3] = {};
            int pixel2d[2] = {j, i};
            ptool.pixel2point(point3d, pixel2d, pDepth[j], ptool.DISTORTION_NONE);

            p.x = point3d[0];
            p.y = point3d[1];
            p.z = point3d[2];

            cloud->points.push_back(p);
        }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();

    std::cout << "point cloud size = " << cloud->points.size() << std::endl;

    cloud->is_dense = false;
    pcl::io::savePCDFile(path, *cloud);

    std::cout << "Point cloud saved." << std::endl;
    return cloud;
}


int main(int argc, char** argv)
{
    // parameters initialization
    CameraIntrinsics intrinsics;
    CameraExtrinsics extrinsics;
    getRealSenseD415Int_720p(intrinsics, extrinsics);

    PixelTool pt(intrinsics, extrinsics);

    string depthPicName(argv[1]);
//    string depthPicName = "/Users/surui/DeepLearning/MOT/jqs_weights/test_mask/depth/7-69_2018-12-01-10-19-13.png";
//    string depthPicName = "/Users/surui/CLionProjects/pcl_restore_show/img/201811211353886346659.png";
//    string depthPicName = "/Users/surui/DeepLearning/MOT/jqs_weights/test_mask/depth/10-93.2_2018-12-01-10-53-57.png";
//    string depthPicName = "/Users/surui/DeepLearning/MOT/jqs_weights/depth.png";
//    string depthPicName = "/Users/surui/DeepLearning/MOT/jqs_weights/test_mask/depth/16-111$20181207160915#1258.png";

    Mat depthImg = imread(depthPicName, IMREAD_ANYDEPTH);

//    string savePcdName = depthPicName.substr(0, depthPicName.length()-4) + ".pcd";
    string savePcdName(argv[2]);
//    string savePcdName = "./one_pig.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc = depth2pc(depthImg, savePcdName, pt);

    return 0;
}