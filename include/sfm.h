/*
 * @Date: 2022-05-26 09:36:50
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-27 10:46:07
 * @FilePath: /mySfmUsingCV/include/sfm.h
 * @Description: 练练手
 */

#ifndef _SFM_H_
#define _SFM_H_

#include "common.h"
#include "featureTool.h"

#include <string>
#include <vector>
#include <map>
#include <set>

enum RunResult
{
    OKAY = 0,
    ERROR
};

typedef std::vector<std::vector<Matching>> MatchMatrix;
typedef std::map<int, Image2D3DMatch> Image2D3DMatches;

class Sfm
{

private:
    /* data */
    std::vector<std::string> mvImageFilenames;
    std::vector<cv::Mat> mvImages;
    std::vector<Features> mvImageFeatures;
    std::vector<cv::Matx34f> mvCameraPoses;
    std::set<int> msDoneViews;
    std::set<int> msGoodViews;
    MatchMatrix mFeatureMatchMatrix;
    Intrinsics mintrinsics;
    float mDownscaleFactor;

    MyFeature2D myFeature2d;

    /**
     *  Extract feature for all image
     */
    void extractFeatures();

    void createFeatureMatchMatrix();

    void findBaselineTriangulation();

    void adjustCurrentBundle();

    std::map<float, ImagePair> sortViewsForBaseline();

    void addMoreViewsToReconstruction();

    Image2D3DMatches find2D3DMatches();

    /**
     * @param {PointCloud&} cloud
     * @return {*}
     */
    void mergeNewPointCloud(const PointCloud &cloud);

public:
    Sfm(const float downscale = 1.0);
    virtual ~Sfm();

    /**
     * @description:
     * @param {string} &dirPath
     * @return {*}
     */
    bool setImagesDir(const std::string &dirDirectory);

    /**
     * @description: main function
     * @return {Result enum}
     */
    RunResult runSfm();

    void saveCloudAndCamerasToPLY(const std::string &filename);
};

#endif