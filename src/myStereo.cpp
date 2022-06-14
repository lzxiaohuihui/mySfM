/*
 * @Date: 2022-05-29 15:41:23
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-30 10:23:53
 * @FilePath: /mySfmUsingCV/src/myStereo.cpp
 * @Description: 练练手
 */

#include "../include/myStereo.h"
#include <iostream>

MyStereo::MyStereo()
{
}

MyStereo::~MyStereo()
{
}

int MyStereo::findHomographyInliers(
    const Features &left,
    const Features &right,
    const Matching &matches)
{
    int returnValue;
    return returnValue;
}

bool MyStereo::findCameraPoseFromMatch(
    const Intrinsics &intrinsics,
    const Matching &featureMatching,
    const Features &featuresLeft,
    const Features &featuresRight,
    Matching &prueMatches,
    cv::Matx34f &PoseLeft,
    cv::Matx34f &PoseRight)
{
    bool returnValue;
    return returnValue;
}

bool MyStereo::triangulateViews(
    const Intrinsics &intrinsics,
    const ImagePair imagePair,
    const Matching &matches,
    const Features &leftFeatures,
    const Features &rightFeatures,
    const cv::Matx34f &PoseLeft,
    const cv::Matx34f &PoseRight,
    PointCloud &pointcloud)
{
    bool returnValue;
    return returnValue;
}

bool MyStereo::findCameraPoseFromPnP(
    const Intrinsics &intrinsics,
    const Image2D3DMatch &match,
    cv::Matx34f &cameraPose)
{
    bool returnValue;
    return returnValue;
}

bool MyStereo::decomposeEssenialMatrix(
    const Eigen::Matrix3d &E,
    Eigen::Matrix3d &R1,
    Eigen::Matrix3d &R2,
    Eigen::Vector3d &t)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() < 0)
        U *= -1;
    if (V.determinant() < 0)
        V *= -1;

    Eigen::Matrix3d W;
    W << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    R1 = U * W * V;
    R2 = U * W.transpose() * V;
    t = U.col(2).normalized();

    return true;
}
bool MyStereo::checkE(
    Eigen::Matrix<double, 3, 4> T1,
    Eigen::Matrix3d R,
    Eigen::Vector3d t,
    Eigen::Vector2d point1,
    Eigen::Vector2d point2)
{
    Eigen::Matrix<double, 3, 4> T2;
    T2.leftCols<3>() = R;
    T2.rightCols<1>() = t;

    Eigen::Matrix4d A;
    A.row(0) = point1(0) * T1.row(2) - T1.row(0);
    A.row(1) = point1(1) * T1.row(2) - T1.row(1);
    A.row(2) = point2(0) * T2.row(2) - T2.row(0);
    A.row(3) = point2(1) * T2.row(2) - T2.row(1);
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    Eigen::Vector3d point = svd.matrixV().col(3).hnormalized();

    bool flag1 = T1.row(2).dot(point.homogeneous()) >= std::numeric_limits<double>::epsilon();
    bool flag2 = T2.row(2).dot(point.homogeneous()) >= std::numeric_limits<double>::epsilon();
    return flag1 && flag2;
}

int MyStereo::findEssenialMatrix(
    const Features &left,
    const Features &right,
    const Matching &matches,
    const Intrinsics &intrinsics)
{
    int returnValue = 0;
    Eigen::Matrix3d K, F, E;
    cv::cv2eigen(intrinsics.K, K);

    std::vector<cv::Point2f> points1, points2;
    for (int i = 0; i < matches.size(); ++i)
    {
        cv::Point2f p1, p2;
        p1 = cv::Point2f(left.points[matches[i].queryIdx].x, left.points[matches[i].queryIdx].y);
        points1.push_back(p1);
        p2 = cv::Point2f(right.points[matches[i].trainIdx].x, right.points[matches[i].trainIdx].y);
        points1.push_back(p2);
    }

    std::cout << "points1 size is: " << points1.size() << std::endl;
    cv::Mat f_matrix = cv::findFundamentalMat(points1, points2);

    cv::cv2eigen(f_matrix, F);
    std::cout << "the F matrix is: \n"
              << F << std::endl;
    E = K.transpose() * F * K;
    Eigen::Matrix3d R1, R2;
    Eigen::Vector3d t;
    std::cout << "decompose...." << std::endl;
    decomposeEssenialMatrix(E, R1, R2, t);
    bool flag = true;
    Eigen::Matrix<double, 3, 4> T1;
    T1 << 1, 0, 0, 0,
     0, 1, 0, 0, 
     0, 0, 1, 0;
    Eigen::Matrix<double, 3, 4> T2;

    Eigen::Vector2d point1 = Eigen::Vector2d(
        (points1[0].x - K(0, 2)) / K(0, 0),
        (points1[0].y - K(1, 2)) / K(1, 1));
    Eigen::Vector2d point2 = Eigen::Vector2d(
        (points2[0].x - K(0, 2)) / K(0, 0),
        (points2[0].y - K(1, 2)) / K(1, 1));

    if (checkE(T1, R1, t, point1, point2))
    {
        T2.leftCols<3>() = R1;
        T2.rightCols<1>() = t;
    }
    else if (checkE(T1, R1, -t, point1, point2))
    {
        T2.leftCols<3>() = R1;
        T2.rightCols<1>() = -t;
    }
    else if (checkE(T1, R2, t, point1, point2))
    {
        T2.leftCols<3>() = R2;
        T2.rightCols<1>() = t;
    }
    else if (checkE(T1, R2, -t, point1, point2))
    {
        T2.leftCols<3>() = R2;
        T2.rightCols<1>() = -t;
    }
    else
    {
        flag = false;
        std::cerr << "error at two view stereo" << std::endl;
    }

    return returnValue;
}