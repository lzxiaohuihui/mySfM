/*
 * @Date: 2022-05-26 09:36:51
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-05-29 15:29:05
 * @FilePath: /mySfmUsingCV/src/sfm.cpp
 * @Description: ¡∑¡∑ ÷
 */
#include "sfm.h"

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

using namespace std;

void Sfm::extractFeatures()
{
    cout << "------------------Extract Features---------------------" << endl;

    mvImageFeatures.resize(mvImages.size());
    for (size_t i = 0; i < mvImages.size(); i++)
    {
        mvImageFeatures[i] = myFeature2d.extractFeatures(mvImages[i]);
        cout << "Image " << i << ": " << mvImageFeatures[i].points.size() << " keypoints" << endl;
    }
}
void Sfm::createFeatureMatchMatrix()
{
    
}
void Sfm::findBaselineTriangulation()
{
}
void Sfm::adjustCurrentBundle()
{
}
std::map<float, ImagePair> Sfm::sortViewsForBaseline()
{
    std::map<float, ImagePair> returnValue;
    return returnValue;
}
void Sfm::addMoreViewsToReconstruction()
{
}

void Sfm::mergeNewPointCloud(const PointCloud &cloud)
{
}
Sfm::Sfm(const float downscale)
    : mDownscaleFactor(downscale)
{
}
Sfm::~Sfm()
{
}
bool Sfm::setImagesDir(const std::string &dirDirectory)
{
    using namespace boost::filesystem;
    path dirPath(dirDirectory);
    if (not exists(dirDirectory) or not is_directory(dirDirectory))
    {
        cerr << "Cannot open directory: " << dirDirectory;
        return false;
    }

    for (directory_entry &x : directory_iterator(dirPath))
    {
        string extension = x.path().extension().string();
        boost::algorithm::to_lower(extension);
        if (extension == ".jpg" or extension == ".png")
        {
            mvImageFilenames.push_back(x.path().string());
        }
    }

    if (mvImageFilenames.size() <= 0)
    {
        cerr << "cannot find the valid image." << endl;
        return false;
    }

    for (auto &imageFilename : mvImageFilenames)
    {
        mvImages.push_back(cv::imread(imageFilename));

        if (mDownscaleFactor != 1.0)
        {
            cv::resize(mvImages.back(), mvImages.back(), cv::Size(), mDownscaleFactor, mDownscaleFactor);
        }
    }
    return true;
}
RunResult Sfm::runSfm()
{
    if (mvImages.size() <= 0)
    {
        // cerr << "No images to work." <<endl;
        return RunResult::ERROR;
    }
    mintrinsics.K = (cv::Mat_<float>(3, 3) << 525, 0, mvImages[0].rows / 2,
                     0, 525, mvImages[0].cols / 2,
                     0, 0, 1);
    mintrinsics.Kinv = mintrinsics.K.inv();
    mintrinsics.distortion = cv::Mat_<float>::zeros(1, 4);

    mvCameraPoses.resize(mvImages.size());

    extractFeatures();

    return RunResult::OKAY;
}
void Sfm::saveCloudAndCamerasToPLY(const std::string &filename)
{
}
Image2D3DMatches Sfm::find2D3DMatches()
{
    Image2D3DMatches returnValue;
    return returnValue;
}