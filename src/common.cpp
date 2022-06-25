#include "../include/common.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

std::ostream &operator<<(std::ostream &stream, const ImagePair &pair)
{
    return stream << "[" << pair.left << ", " << pair.right << "]";
}

void getAlignedPointsFromMatch(const Features &leftFeatures,
                               const Features &rightFeatures,
                               const vector<cv::DMatch> &matches,
                               Features &alignedLeft,
                               Features &alignedRight)
{
    vector<int> leftBackReference, rightBackReference;
    getAlignedPointsFromMatch(leftFeatures,
                              rightFeatures,
                              matches,
                              alignedLeft,
                              alignedRight,
                              leftBackReference,
                              rightBackReference);
}
// 获取2d匹配点对
void getAlignedPointsFromMatch(const Features &leftFeatures,
                               const Features &rightFeatures,
                               const vector<cv::DMatch> &matches,
                               Features &alignedLeft,
                               Features &alignedRight,
                               vector<int> &leftBackReference,
                               vector<int> &rightBackReference)
{
    // cout << "00000" << endl;
    alignedLeft.points.clear();
    alignedRight.points.clear();
    // alignedLeft.descriptors.clear();
    // alignedRight.descriptors.clear();
    // cout << "1111" << endl;
    for (int i = 0; i < matches.size(); ++i)
    {
        alignedLeft.points.push_back(leftFeatures.points[matches[i].queryIdx]);
        // alignedLeft.descriptors.push_back(leftFeatures.descriptors[matches[i].queryIdx]);
        alignedRight.points.push_back(rightFeatures.points[matches[i].trainIdx]);
        // alignedRight.descriptors.push_back(rightFeatures.descriptors[matches[i].trainIdx]);
        leftBackReference.push_back(matches[i].queryIdx);
        rightBackReference.push_back(matches[i].trainIdx);
    }
    cout << "get keypoints  done.." << endl;
    pointsToKeypoints(alignedLeft.points, alignedLeft.keypoints);
    pointsToKeypoints(alignedRight.points, alignedRight.keypoints);
}

void keypointsToPoints(const vector<cv::KeyPoint> &kps, vector<cv::Point2f> &ps)
{
    ps.clear();
    for (const auto &kp : kps)
    {
        ps.push_back(kp.pt);
    }
}

void pointsToKeypoints(const vector<cv::Point2f> &ps, vector<cv::KeyPoint> &kps)
{
    kps.clear();
    for (const auto &p : ps)
    {
        kps.push_back(cv::KeyPoint(p, 1.0f));
    }
}

vector<cv::DMatch> getAlignedMatching(size_t size)
{
    vector<cv::DMatch> match;
    for (size_t i = 0; i < size; ++i)
    {
        match.push_back(cv::DMatch(i, i, 0));
    }
    return match;
}
