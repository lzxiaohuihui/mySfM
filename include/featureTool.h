/*
 * @Date: 2022-05-26 10:47:24
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-27 14:59:24
 * @FilePath: /mySfmUsingCV/include/featureTool.h
 * @Description: 练练手
 */
#ifndef _MYFEATURE2D_H_
#define _MYFEATURE2D_H_

#include "common.h"
#include "SiftGPU.h"
#include <GL/gl.h>

class MyFeature2D
{
private:
    /* data */
    SiftGPU sift;
    SiftMatchGPU matcher;
    const int maxSiftMatchPair = 20000;

public:
    MyFeature2D(/* args */);
    ~MyFeature2D();
    Features extractFeatures(const cv::Mat &image);

    vector<cv::DMatch> matchFeatures(const Features &featuresLeft, const Features &featuresRight);
};

#endif
