/*
 * @Date: 2022-05-26 10:57:58
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-30 10:07:16
 * @FilePath: /mySfmUsingCV/include/myStereo.h
 * @Description: 练练手
 */
#ifndef _MYSTEREO_H_
#define _MYSTEREO_H_

#include "common.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

using namespace std;

class MyStereo
{
private:
    /* data */

public:
    MyStereo(/* args */);
    ~MyStereo();

    /**
     * @description:
     * @param {Features} &left
     * @param {Features} &right
     * @param {Matching} &matches
     * @return {*}
     */
    static int findHomographyInliers(
        const Features &left,
        const Features &right,
        const vector<cv::DMatch> &matches);

    static int findEssenialMatrix(
        const Features &left,
        const Features &right,
        const vector<cv::DMatch> &matches,
        const Intrinsics &intrinsics);

    static bool findCameraPoseFromMatch(
        const Intrinsics &intrinsics,
        const vector<cv::DMatch> &featureMatching,
        const Features &featuresLeft,
        const Features &featuresRight,
        vector<cv::DMatch> &prueMatches,
        cv::Matx34f &PoseLeft,
        cv::Matx34f &PoseRight);

    static bool triangulateViews(
        const Intrinsics &intrinsics,
        const ImagePair imagePair,
        const vector<cv::DMatch> &matches,
        const Features &leftFeatures,
        const Features &rightFeatures,
        const cv::Matx34f &PoseLeft,
        const cv::Matx34f &PoseRight,
        PointCloud &pointcloud);

    static bool findCameraPoseFromPnP(
        const Intrinsics &intrinsics,
        const Image2D3DMatch &match,
        cv::Matx34f &cameraPose);

    static bool decomposeEssenialMatrix(
        const Eigen::Matrix3d &E,
        Eigen::Matrix3d &R1,
        Eigen::Matrix3d &R2,
        Eigen::Vector3d &t);

    static bool checkE(
        Eigen::Matrix<double, 3, 4> T1,
        Eigen::Matrix3d R,
        Eigen::Vector3d t,
        Eigen::Vector2d point1,
        Eigen::Vector2d point2);

    static bool findCameraPoseFrom2D3DMatch(
        const Intrinsics &intrinsics,
        const Image2D3DMatch &match,
        cv::Matx34f &cameraPose);
};

#endif