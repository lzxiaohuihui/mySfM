/*
 * @Date: 2022-05-29 15:41:23
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-30 10:23:53
 * @FilePath: /mySfmUsingCV/src/myStereo.cpp
 * @Description: 练练手
 */

#include "../include/myStereo.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;

const double RANSAC_THRESHOLD = 10.0f;     // RANSAC inlier threshold
const float MIN_REPROJECTION_ERROR = 10.0; // Maximum 10-pixel allowed re-projection error

MyStereo::MyStereo()
{
}

MyStereo::~MyStereo()
{
}

int MyStereo::findHomographyInliers(
    const Features &left,
    const Features &right,
    const vector<cv::DMatch> &matches)
{
    Features alignedLeft, alignedRight;

    cout << "get aligned points" << endl;
    getAlignedPointsFromMatch(left, right, matches, alignedLeft, alignedRight);

    cv::Mat inlierMask;
    cv::Mat homography;
    cout << "find homography... " << endl;
    if (matches.size() >= 4)
    {
        homography = cv::findHomography(alignedLeft.points,
                                        alignedRight.points,
                                        cv::RANSAC,
                                        RANSAC_THRESHOLD,
                                        inlierMask);
    }
    if (matches.size() < 4 || homography.empty())
        return 0;

    return cv::countNonZero(inlierMask);
}

bool MyStereo::findCameraPoseFromMatch(
    const Intrinsics &intrinsics,
    const vector<cv::DMatch> &featureMatching,
    const Features &featuresLeft,
    const Features &featuresRight,
    vector<cv::DMatch> &prueMatches,
    cv::Matx34f &PoseLeft,
    cv::Matx34f &PoseRight)
{
    if (intrinsics.K.empty())
    {
        cerr << "Intrinsics matrix (K) must be initialized. " << endl;
        return false;
    }
    double focal = intrinsics.K.at<float>(0, 0);
    cv::Point2d pp(intrinsics.K.at<float>(0, 2), intrinsics.K.at<float>(1, 2));

    Features alignedLeft;
    Features alignedRight;

    getAlignedPointsFromMatch(featuresLeft, featuresRight, featureMatching, alignedLeft, alignedRight);

    cv::Mat E, R, t;
    cv::Mat mask;
    // TODO: rewrite the Essenial Matrix
    // E = findEssenialMatrix(alignedLeft, alignedRight, featureMatching, intrinsics);
    E = cv::findEssentialMat(alignedLeft.points, alignedRight.points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    // TODO: write the recover pose
    cv::recoverPose(E, alignedLeft.points, alignedRight.points, R, t, focal, pp, mask);

    PoseLeft = cv::Matx34f::eye();
    PoseRight = cv::Matx34f(
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(0, 3),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(1, 3),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), R.at<double>(2, 3));

    prueMatches.clear();
    for (size_t i = 0; i < mask.rows; ++i)
    {
        if (mask.at<uchar>(i))
        {
            prueMatches.push_back(featureMatching[i]);
        }
    }

    return true;
}

bool MyStereo::triangulateViews(
    const Intrinsics &intrinsics,
    const ImagePair imagePair,
    const vector<cv::DMatch> &matches,
    const Features &leftFeatures,
    const Features &rightFeatures,
    const cv::Matx34f &PoseLeft,
    const cv::Matx34f &PoseRight,
    PointCloud &pointcloud)
{
    vector<int> leftBackReference;
    vector<int> rightBackReference;
    Features alignedLeft;
    Features alignedRight;

    getAlignedPointsFromMatch(
        leftFeatures,
        rightFeatures,
        matches,
        alignedLeft,
        alignedRight,
        leftBackReference,
        rightBackReference);

    cv::Mat normalizedLeftPts, normalizedRightPts;
    cv::undistortPoints(alignedLeft.points, normalizedLeftPts, intrinsics.K, cv::Mat());
    cv::undistortPoints(alignedRight.points, normalizedRightPts, intrinsics.K, cv::Mat());

    cv::Mat points3dHomogeneous;
    cv::triangulatePoints(PoseLeft, PoseRight, normalizedLeftPts, normalizedRightPts, points3dHomogeneous);

    cv::Mat points3d;
    cv::convertPointsFromHomogeneous(points3dHomogeneous.t(), points3d);

    cv::Mat rvecLeft;
    cv::Rodrigues(PoseLeft.get_minor<3, 3>(0, 0), rvecLeft);
    cv::Mat tvecLeft(PoseLeft.get_minor<3, 1>(0, 3).t());

    vector<cv::Point2f> projectedOnLeft(alignedLeft.points.size());
    cv::projectPoints(points3d, rvecLeft, tvecLeft, intrinsics.K, cv::Mat(), projectedOnLeft);

    cv::Mat rvecRight;
    cv::Rodrigues(PoseRight.get_minor<3, 3>(0, 0), rvecRight);
    cv::Mat tvecRight(PoseRight.get_minor<3, 1>(0, 3).t());

    vector<cv::Point2f> projectedOnRight(alignedRight.points.size());
    projectPoints(points3d, rvecRight, tvecRight, intrinsics.K, cv::Mat(), projectedOnRight);

    for (size_t i = 0; i < points3d.rows; i++)
    {
        // check if point reprojection error is small enough
        if (norm(projectedOnLeft[i] - alignedLeft.points[i]) > MIN_REPROJECTION_ERROR or
            norm(projectedOnRight[i] - alignedRight.points[i]) > MIN_REPROJECTION_ERROR)
        {
            continue;
        }

        Point3DInMap p;
        p.p = cv::Point3f(points3d.at<float>(i, 0),
                          points3d.at<float>(i, 1),
                          points3d.at<float>(i, 2));

        // use back reference to point to original features in images
        p.originatingViews[imagePair.left] = leftBackReference[i];
        p.originatingViews[imagePair.right] = rightBackReference[i];

        pointcloud.push_back(p);
    }

    return true;
}

bool MyStereo::findCameraPoseFromPnP(
    const Intrinsics &intrinsics,
    const Image2D3DMatch &match,
    cv::Matx34f &cameraPose)
{
    cv::Mat rvec, tvec;
    cv::Mat inliers;

    cv::solvePnPRansac(match.points3D,
                       match.points2D, intrinsics.K,
                       intrinsics.distortion,
                       rvec,
                       tvec,
                       false,
                       100,
                       RANSAC_THRESHOLD,
                       0.99,
                       inliers);

    if (((float)cv::countNonZero(inliers) / (float)match.points2D.size()) < POSE_INLIERS_MINIMAL_RATIO)
    {
        cerr << "Inliers ratio is too small: " << countNonZero(inliers) << " / " << match.points2D.size() << endl;
        return false;
    }

    cv::Mat rotMat;
    cv::Rodrigues(rvec, rotMat);

    rotMat.copyTo(cv::Mat(3, 4, CV_32FC1, cameraPose.val)(ROT));
    tvec.copyTo(cv::Mat(3, 4, CV_32FC1, cameraPose.val)(TRA));
    return true;
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
    const vector<cv::DMatch> &matches,
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