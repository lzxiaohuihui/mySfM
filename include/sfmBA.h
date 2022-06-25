#ifndef __SFMBA_H__
#define __SFMBA_H__

#include "common.h"

class SfmBA
{
public:
    static void bundleAdjust(
        PointCloud &pointCloud,
        std::vector<cv::Matx34f> &cameraPoses,
        Intrinsics &intrinsics,
        const std::vector<Features> &image2dFeatures);
};

#endif // __SFMBA_H__