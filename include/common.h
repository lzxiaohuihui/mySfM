/*
 * @Date: 2022-05-26 10:39:09
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-27 13:39:58
 * @FilePath: /mySfmUsingCV/include/common.h
 * @Description: 练练手
 */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <map>

const float POSE_INLIERS_MINIMAL_RATIO = 0.5;

struct Intrinsics{
    cv::Mat K;
    cv::Mat Kinv;
    cv::Mat distortion;
};

struct ImagePair{
    size_t left, right;
};

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Point2f> Points2f;
typedef std::vector<cv::Point3f> Points3f;

struct Features{
    Points2f points;
    std::vector<float> descriptors;
};

struct Point3DInMap{
    cv::Point3f p;
    std::map<int,int> originatingViews;
};

struct Point3DInMapRGB{
    Point3DInMap p;
    cv::Scalar rgb;
};

typedef std::vector<cv::DMatch> Matching;
typedef std::vector<Point3DInMap> PointCloud;
typedef std::vector<Point3DInMapRGB> PointCloudRGB;

typedef cv::Matx34f Pose;

struct Image2D3DMatch{
    Points2f points2D;
    Points3f points3D;
};



#endif