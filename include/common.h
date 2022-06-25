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

using namespace std;

/// Rotational element in a 3x4 matrix
const cv::Rect ROT(0, 0, 3, 3);

/// Translational element in a 3x4 matrix
const cv::Rect TRA(3, 0, 1, 3);

const float POSE_INLIERS_MINIMAL_RATIO = 0.5;

struct Intrinsics
{
    cv::Mat K;
    cv::Mat Kinv;
    cv::Mat distortion;
};

struct ImagePair
{
    size_t left, right;
};
std::ostream &operator<<(std::ostream &stream, const ImagePair &pair);

// typedef std::vector<cv::KeyPoint> Keypoints;
// typedef std::vector<cv::Point2f> Points2f;
// typedef std::vector<cv::Point3f> Points3f;

struct Features
{
    vector<cv::KeyPoint> keypoints;
    vector<cv::Point2f> points;
    vector<float> descriptors;
};

struct Point3DInMap
{
    cv::Point3f p;
    // 被那副图像哪个特征点观测到
    map<int, int> originatingViews;
};

struct Point3DInMapRGB
{
    Point3DInMap p;
    cv::Scalar rgb;
};

typedef std::vector<cv::DMatch> Matching;
typedef std::vector<Point3DInMap> PointCloud;
typedef std::vector<Point3DInMapRGB> PointCloudRGB;

typedef cv::Matx34f Pose;

struct Image2D3DMatch
{
    vector<cv::Point2f> points2D;
    vector<cv::Point3f> points3D;
};

void getAlignedPointsFromMatch(const Features &leftFeatures,
                               const Features &rightFeatures,
                               const vector<cv::DMatch> &matches,
                               Features &alignedLeft,
                               Features &alignedRight);
// 获取2d匹配点对
void getAlignedPointsFromMatch(const Features &leftFeatures,
                               const Features &rightFeatures,
                               const vector<cv::DMatch> &matches,
                               Features &alignedLeft,
                               Features &alignedRight,
                               std::vector<int> &leftBackReference,
                               std::vector<int> &rightBackReference);
void keypointsToPoints(const vector<cv::KeyPoint> &kps, vector<cv::Point2f> &ps);
void pointsToKeypoints(const vector<cv::Point2f> &ps, vector<cv::KeyPoint> &kps);

vector<cv::DMatch> getAlignedMatching(size_t size);

#endif