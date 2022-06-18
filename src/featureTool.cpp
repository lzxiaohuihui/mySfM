/*
 * @Date: 2022-05-27 13:22:06
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-06-14 20:25:12
 * @FilePath: /mySfmUsingCV/src/featureTool.cpp
 * @Description: 练练手
 */

#include "featureTool.h"
#include <iostream>

using namespace std;

MyFeature2D::MyFeature2D()
{
    char *myargv[4] = {(char *)"-fo", (char *)"-1", (char *)"-v", (char *)"1"};
    sift.ParseParam(4, myargv);
    int support = sift.CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        cerr << "SiftGPU is not supported!" << endl;
    }

    matcher.VerifyContextGL();
    matcher.SetMaxSift(maxSiftMatchPair);
}

MyFeature2D::~MyFeature2D()
{
}

Features MyFeature2D::extractFeatures(const cv::Mat &image)
{
    Features result_features;
    std::vector<SiftGPU::SiftKeypoint> siftKps;

    sift.RunSIFT(image.cols, image.rows, image.data, GL_RGB, GL_UNSIGNED_BYTE);

    int num = sift.GetFeatureNum();
    siftKps.resize(num);
    result_features.points.resize(num);
    result_features.descriptors.resize(128 * num);

    sift.GetFeatureVector(&siftKps[0], &result_features.descriptors[0]);
    // frame.cvKeys.resize(num);
    for (int i = 0; i < num; ++i)
    {
        result_features.points[i].x = siftKps[i].x;
        result_features.points[i].y = siftKps[i].y;
    }
    return result_features;
}

vector<cv::DMatch> MyFeature2D::matchFeatures(const Features &featuresLeft, const Features &featuresRight)
{
    vector<cv::DMatch> initMatches;

    matcher.SetDescriptors(0, featuresLeft.points.size(), &(featuresLeft.descriptors[0]));
    matcher.SetDescriptors(1, featuresRight.points.size(), &(featuresRight.descriptors[0]));
    int(*match_buf)[2] = new int[featuresLeft.points.size()][2];
    int num_match = matcher.GetSiftMatch(featuresLeft.points.size(), match_buf);
    // std::cout << "match number : " << num_match << endl;
    // frame1.link.push_back(frame2.frameID);
    // frame2.link.push_back(frame1.frameID);
    for (int i = 0; i < num_match; ++i)
    {
        cv::DMatch match;
        match.queryIdx = match_buf[i][0];
        match.trainIdx = match_buf[i][1];
        cv::Point2f p1, p2;
        p1.x = featuresLeft.points[match.queryIdx].x;
        p1.y = featuresLeft.points[match.queryIdx].y;
        p2.x = featuresRight.points[match.trainIdx].x;
        p2.y = featuresRight.points[match.trainIdx].y;

        float dist = (float)sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        match.distance = dist;
        initMatches.push_back(match);
    }
    delete[] match_buf;

    return initMatches;
}
