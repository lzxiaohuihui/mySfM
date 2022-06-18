/*
 * @Date: 2022-05-26 09:36:51
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-06-14 21:42:34
 * @FilePath: /mySfmUsingCV/src/sfm.cpp
 * @Description: ������
 */
#include "../include/sfm.h"

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include <thread>
#include <mutex>

using namespace std;

const int MIN_POINT_COUNT_FOR_HOMOGRAPHY = 100;
const int MIN_POINT_COUNT_FOR_ESSENSIAL = 100;

void Sfm::extractFeatures()
{
    cout << "------------------Extract Features---------------------" << endl;

    mvImageFeatures.resize(mvImages.size());
    for (size_t i = 0; i < mvImages.size(); i++)
    {
        mvImageFeatures[i] = myFeature2d.extractFeatures(mvImages[i]);
        // cout << "Image " << i << ": " << mvImageFeatures[i].points.size() << " keypoints" << endl;
    }
}
void Sfm::createFeatureMatchMatrix()
{
    cout << "------------------Create Feature Match Matrix---------------------" << endl;
    const size_t numImages = mvImages.size();
    mFeatureMatchMatrix.resize(numImages, vector<vector<cv::DMatch>>(numImages));

    vector<ImagePair> pairs;
    for (size_t i = 0; i < numImages; ++i)
    {
        for (size_t j = i + 1; j < numImages; ++j)
        {
            pairs.push_back({i, j});
        }
    }

    vector<thread> threads;
    const int numThreads = std::thread::hardware_concurrency() - 1;

    for (int i = 0; i < pairs.size(); ++i)
    {
        const ImagePair &pair = pairs[i];
        mFeatureMatchMatrix[pair.left][pair.right] = myFeature2d.matchFeatures(mvImageFeatures[pair.left], mvImageFeatures[pair.right]);
        cout << "Match (pair " << i << ") " << pair.left << ", " << pair.right << ": " << mFeatureMatchMatrix[pair.left][pair.right].size() << " matched features" << endl;
    }

    // const int numPairsForThread = (numThreads > pairs.size()) ? 1:(int)ceilf((float)(pairs.size()) / numThreads);
    // mutex writeMutex;
    // cout << "Launch " << numThreads << "threads with " << numPairsForThread << "paris per thread " << endl;

    // for (size_t threadId = 0; threadId < MIN(numThreads, pairs.size()); ++threadId)
    // {
    //     threads.push_back(thread([&, threadId]
    //                              {
    //                                  const int startPair = numPairsForThread * threadId;

    //                                  for (int j = 0; j < numPairsForThread; ++j)
    //                                  {
    //                                      const int pairId = startPair + j;
    //                                      if (pairId >= pairs.size())
    //                                          break;
    //                                      const ImagePair &pair = pairs[pairId];
    //                                      mFeatureMatchMatrix[pair.left][pair.right] = myFeature2d.matchFeatures(mvImageFeatures[pair.left], mvImageFeatures[pair.right]);

    //                                      writeMutex.lock();
    //                                      cout << "Thread " << threadId << ": Match (pair " << pairId << ") " << pair.left << ", " << pair.right << ": " << mFeatureMatchMatrix[pair.left][pair.right].size() << " matched features" << endl;
    //                                      writeMutex.unlock();
    //                                  }
    //                              }));
    // }
    // for (auto &t : threads)
    // {
    //     t.join();
    // }
}
void Sfm::findBaselineTriangulation()
{
    cout << "--- Sort views by homography inliers" << endl;

    map<float, ImagePair> pairHomographyInliers = sortViewsForBaseline();

    cv::Matx34f pLeft = cv::Matx34f::eye();
    cv::Matx34f pRight = cv::Matx34f::eye();
    vector<Point3DInMap> pointCloud;
    cout << "--- Try views in triangulation" << endl;

    // try to find the best pair,
    for (auto &imagePair : pairHomographyInliers)
    {
        cout << "Trying " << imagePair.second << " ratio: " << imagePair.first << endl
             << flush;
        size_t i = imagePair.second.left;
        size_t j = imagePair.second.right;

        cout << "--- Find camera matrices" << endl;

        vector<cv::DMatch> prunedMatching;

        bool success = MyStereo::findCameraPoseFromMatch(
            mintrinsics,
            mFeatureMatchMatrix[i][j],
            mvImageFeatures[i],
            mvImageFeatures[j],
            prunedMatching,
            pLeft, pRight);
        if (not success)
        {
            cerr << "stereo view could not be obtained " << imagePair.second << ", go to next pair " << endl;
            continue;
        }
        float poseInliersRatio = (float)prunedMatching.size() / (float)mFeatureMatchMatrix[i][j].size();

        mFeatureMatchMatrix[i][j] = prunedMatching;
        cout << "---- Triangulate from stereo views: " << imagePair.second << endl;

        success = MyStereo::triangulateViews(mintrinsics,
                                             imagePair.second,
                                             mFeatureMatchMatrix[i][j],
                                             mvImageFeatures[i],
                                             mvImageFeatures[j],
                                             pLeft,
                                             pRight,
                                             pointCloud);

        mReconstructionCloud = pointCloud;
        mvCameraPoses[i] = pLeft;
        mvCameraPoses[j] = pRight;
        msDoneViews.insert(i);
        msGoodViews.insert(j);
        msDoneViews.insert(i);
        msGoodViews.insert(j);

        break;
    }
}
void Sfm::adjustCurrentBundle()
{
}
std::map<float, ImagePair> Sfm::sortViewsForBaseline()
{
    std::map<float, ImagePair> matchesSizes;
    const size_t numImages = mvImages.size();
    for (size_t i = 0; i < numImages - 1; ++i)
    {
        for (size_t j = i + 1; j < numImages; ++j)
        {
            if (mFeatureMatchMatrix[i][j].size() < MIN_POINT_COUNT_FOR_HOMOGRAPHY)
            {
                matchesSizes[1.0] = {i, j};
                continue;
            }
            // have enough matches
            const int numInliers = MyStereo::findHomographyInliers(
                mvImageFeatures[i],
                mvImageFeatures[j],
                mFeatureMatchMatrix[i][j]);
            const float inliersRatio = (float)numInliers / (float)(mFeatureMatchMatrix[i][j].size());
            matchesSizes[inliersRatio] = {i, j};

            cout << "Homography inliers ratio : " << i << "," << j << " " << inliersRatio << endl;
        }
    }

    return matchesSizes;
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

    createFeatureMatchMatrix();

    findBaselineTriangulation();
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