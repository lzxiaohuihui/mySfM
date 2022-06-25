/*
 * @Date: 2022-05-26 09:36:51
 * @LastEditors: lzxiaohuihui 895827938@qq.com
 * @LastEditTime: 2022-06-14 21:42:34
 * @FilePath: /mySfmUsingCV/src/sfm.cpp
 * @Description: ������
 */
#include "../include/sfm.h"
#include "../include/sfmBA.h"

#include <iostream>
#include <algorithm>

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
const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE = 0.01;
const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

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
        cout << "Trying " << imagePair.second << " ratio: " << imagePair.first << endl;

        size_t i = imagePair.second.left;
        size_t j = imagePair.second.right;

        cout << "--- Find camera matrices" << endl;

        vector<cv::DMatch> prunedMatching;

        // 本质矩阵恢复相机位姿
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
        cout << "---- Triangulate from stereo views: " << imagePair.second.left << endl;

        // 三角化初始两帧的匹配点
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

        adjustCurrentBundle();
        break;
    }
}
void Sfm::adjustCurrentBundle()
{
    SfmBA::bundleAdjust(
        mReconstructionCloud,
        mvCameraPoses,
        mintrinsics,
        mvImageFeatures);
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
    cout << "----------------Add More Views---------------" << endl;
    while (msDoneViews.size() != mvImages.size())
    {
        Image2D3DMatches matches2D3D = find2D3DMatches();

        size_t bestView;
        size_t bestNumMatches = 0;
        for (const auto &match2D3D : matches2D3D)
        {
            const size_t numMatches = match2D3D.second.points2D.size();
            if (numMatches > bestNumMatches)
            {
                bestView = match2D3D.first;
                bestNumMatches = numMatches;
            }
        }
        cout << "Best view " << bestView << " has " << bestNumMatches << " matches" << endl;
        cout << "Adding " << bestView << " to existing " << cv::Mat(vector<int>(msGoodViews.begin(), msGoodViews.end())).t() << endl;
        msDoneViews.insert(bestView);

        cv::Matx34f newCameraPose;
        bool success = MyStereo::findCameraPoseFrom2D3DMatch(
            mintrinsics,
            matches2D3D[bestView],
            newCameraPose);

        if (not success)
        {
            cerr << "Cannot recover camera pose for view " << bestView << endl;
            continue;
        }
        mvCameraPoses[bestView] = newCameraPose;

        cout << "New view " << bestView << " Pose " << endl
             << newCameraPose << endl;

        bool anyViewSuccess = false;
        for (const int goodView : msGoodViews)
        {
            size_t leftViewIdx = (goodView < bestView) ? goodView : bestView;
            size_t rightViewIdx = (goodView < bestView) ? bestView : goodView;

            Matching prunedMatching;
            cv::Matx34f Pleft = cv::Matx34f::eye();
            cv::Matx34f Pright = cv::Matx34f::eye();

            bool success = MyStereo::findCameraPoseFromMatch(
                mintrinsics,
                mFeatureMatchMatrix[leftViewIdx][rightViewIdx],
                mvImageFeatures[leftViewIdx],
                mvImageFeatures[rightViewIdx],
                prunedMatching,
                Pleft, Pright);

            mFeatureMatchMatrix[leftViewIdx][rightViewIdx] = prunedMatching;

            PointCloud pointCloud;
            success = MyStereo::triangulateViews(
                mintrinsics,
                {leftViewIdx, rightViewIdx},
                mFeatureMatchMatrix[leftViewIdx][rightViewIdx],
                mvImageFeatures[leftViewIdx],
                mvImageFeatures[rightViewIdx],
                mvCameraPoses[leftViewIdx],
                mvCameraPoses[rightViewIdx],
                pointCloud);
            if (success)
            {
                cout << "Merge triangulation between " << leftViewIdx << " and " << rightViewIdx << " (# matching pts = " << (mFeatureMatchMatrix[leftViewIdx][rightViewIdx].size()) << ") ";

                mergeNewPointCloud(pointCloud);

                anyViewSuccess = true;
            }
            else
            {
                cerr << "Failed to triangulation " << leftViewIdx << " and " << rightViewIdx << endl;
            }
        }
        if (anyViewSuccess)
        {
            adjustCurrentBundle();
        }
        msGoodViews.insert(bestView);
    }
}

void Sfm::mergeNewPointCloud(const PointCloud &cloud)
{
    const size_t numImages = mvImages.size();
    MatchMatrix mergeMatchMatrix;
    mergeMatchMatrix.resize(numImages, vector<Matching>(numImages));

    size_t newPoints = 0;
    size_t mergedPoints = 0;

    for (const Point3DInMap &p : cloud)
    {
        const cv::Point3f newPoint = p.p; // new 3D point

        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;
        for (Point3DInMap &existingPoint : mReconstructionCloud)
        {
            if (cv::norm(existingPoint.p - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE)
            {
                // This point is very close to an existing 3D cloud point
                foundMatching3DPoint = true;

                // Look for common 2D features to confirm match
                for (const auto &newKv : p.originatingViews)
                {
                    // kv.first = new point's originating view
                    // kv.second = new point's view 2D feature index

                    for (const auto &existingKv : existingPoint.originatingViews)
                    {
                        // existingKv.first = existing point's originating view
                        // existingKv.second = existing point's view 2D feature index

                        bool foundMatchingFeature = false;

                        const bool newIsLeft = newKv.first < existingKv.first;
                        const int leftViewIdx = (newIsLeft) ? newKv.first : existingKv.first;
                        const int leftViewFeatureIdx = (newIsLeft) ? newKv.second : existingKv.second;
                        const int rightViewIdx = (newIsLeft) ? existingKv.first : newKv.first;
                        const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                        const Matching &matching = mFeatureMatchMatrix[leftViewIdx][rightViewIdx];
                        for (const cv::DMatch &match : matching)
                        {
                            if (match.queryIdx == leftViewFeatureIdx and match.trainIdx == rightViewFeatureIdx and match.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE)
                            {

                                mergeMatchMatrix[leftViewIdx][rightViewIdx].push_back(match);

                                // Found a 2D feature match for the two 3D points - merge
                                foundMatchingFeature = true;
                                break;
                            }
                        }

                        if (foundMatchingFeature)
                        {
                            // Add the new originating view, and feature index
                            existingPoint.originatingViews[newKv.first] = newKv.second;

                            foundAnyMatchingExistingViews = true;
                        }
                    }
                }
            }
            if (foundAnyMatchingExistingViews)
            {
                mergedPoints++;
                break; // Stop looking for more matching cloud points
            }
        }

        if (not foundAnyMatchingExistingViews and not foundMatching3DPoint)
        {
            // This point did not match any existing cloud points - add it as new.
            mReconstructionCloud.push_back(p);
            newPoints++;
        }
    }

    // debug: show new matching points in the cloud
    for (size_t i = 0; i < numImages - 1; i++)
    {
        for (size_t j = i; j < numImages; j++)
        {
            const Matching &matching = mergeMatchMatrix[i][j];
            if (matching.empty())
            {
                continue;
            }

            cv::Mat outImage;
            drawMatches(mvImages[i], mvImageFeatures[i].keypoints,
                        mvImages[j], mvImageFeatures[j].keypoints,
                        matching, outImage);
            // write the images index...
            cv::putText(outImage, "Image " + to_string(i), cv::Point(10, 50), CV_FONT_NORMAL, 3.0, cv::Scalar(0, 255, 0), 3);
            cv::putText(outImage, "Image " + to_string(j), cv::Point(10 + outImage.cols / 2, 50), CV_FONT_NORMAL, 3.0, cv::Scalar(0, 255, 0), 3);
            cv::resize(outImage, outImage, cv::Size(), 0.25, 0.25);
            cv::imshow("Merge Match", outImage);
            cv::waitKey(0);
        }
    }
    cv::destroyWindow("Merge Match");

    cout << " adding: " << cloud.size() << " (new: " << newPoints << ", merged: " << mergedPoints << ")" << endl;
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
void Sfm::saveCloudAndCamerasToPLY(const std::string &prefix)
{
    cout << "Saving result reconstruction with prefix " << prefix << endl;

    ofstream ofs(prefix + "_points.ply");

    // write PLY header
    ofs << "ply                 " << endl
        << "format ascii 1.0    " << endl
        << "element vertex " << mReconstructionCloud.size() << endl
        << "property float x    " << endl
        << "property float y    " << endl
        << "property float z    " << endl
        << "property uchar red  " << endl
        << "property uchar green" << endl
        << "property uchar blue " << endl
        << "end_header          " << endl;

    for (const Point3DInMap &p : mReconstructionCloud)
    {
        // get color from first originating view
        auto originatingView = p.originatingViews.begin();
        const int viewIdx = originatingView->first;
        cv::Point2f p2d = mvImageFeatures[viewIdx].points[originatingView->second];
        cv::Vec3b pointColor = mvImages[viewIdx].at<cv::Vec3b>(p2d);

        // write vertex
        ofs << p.p.x << " " << p.p.y << " " << p.p.z << " " << (int)pointColor(2) << " " << (int)pointColor(1) << " " << (int)pointColor(0) << " " << endl;
    }

    ofs.close();

    ofstream ofsc(prefix + "_cameras.ply");

    // write PLY header
    ofsc << "ply                 " << endl
         << "format ascii 1.0    " << endl
         << "element vertex " << (mvCameraPoses.size() * 4) << endl
         << "property float x    " << endl
         << "property float y    " << endl
         << "property float z    " << endl
         << "element edge " << (mvCameraPoses.size() * 3) << endl
         << "property int vertex1" << endl
         << "property int vertex2" << endl
         << "property uchar red  " << endl
         << "property uchar green" << endl
         << "property uchar blue " << endl
         << "end_header          " << endl;

    // save cameras polygons..
    for (const auto &pose : mvCameraPoses)
    {
        cv::Point3d c(pose(0, 3), pose(1, 3), pose(2, 3));
        cv::Point3d cx = c + cv::Point3d(pose(0, 0), pose(1, 0), pose(2, 0)) * 0.2;
        cv::Point3d cy = c + cv::Point3d(pose(0, 1), pose(1, 1), pose(2, 1)) * 0.2;
        cv::Point3d cz = c + cv::Point3d(pose(0, 2), pose(1, 2), pose(2, 2)) * 0.2;

        ofsc << c.x << " " << c.y << " " << c.z << endl;
        ofsc << cx.x << " " << cx.y << " " << cx.z << endl;
        ofsc << cy.x << " " << cy.y << " " << cy.z << endl;
        ofsc << cz.x << " " << cz.y << " " << cz.z << endl;
    }

    const int camVertexStartIndex = mReconstructionCloud.size();

    for (size_t i = 0; i < mvCameraPoses.size(); i++)
    {
        ofsc << (i * 4 + 0) << " " << (i * 4 + 1) << " "
             << "255 0 0" << endl;
        ofsc << (i * 4 + 0) << " " << (i * 4 + 2) << " "
             << "0 255 0" << endl;
        ofsc << (i * 4 + 0) << " " << (i * 4 + 3) << " "
             << "0 0 255" << endl;
    }
}
Image2D3DMatches Sfm::find2D3DMatches()
{
    Image2D3DMatches matches;

    // 遍历所有未注册的图像
    for (size_t viewIdx = 0; viewIdx < mvImages.size(); ++viewIdx)
    {
        if (msDoneViews.find(viewIdx) != msDoneViews.end())
        {
            continue;
        }

        Image2D3DMatch match2D3D;

        // 遍历所有3D点
        for (const Point3DInMap &cloudPoint : mReconstructionCloud)
        {
            bool found2DPoint = false;
            // 遍历观测到这个3D点的所有特征点
            for (const auto &origViewAndPoint : cloudPoint.originatingViews)
            {
                // 观测到这个3D点的图像索引
                const int originatingViewIndex = origViewAndPoint.first;
                // 观测这到个3D点的特征点索引
                const int originatingViewFeatureIndex = origViewAndPoint.second;

                // 以观测到这个3D点的图像作为左视图，当前未注册的图像作为右视图
                const int leftViewIdx = (originatingViewIndex < viewIdx) ? originatingViewIndex : viewIdx;
                const int rightViewIdx = (originatingViewIndex < viewIdx) ? viewIdx : originatingViewIndex;

                // 查看左右视图之间的匹配关系
                for (const cv::DMatch &m : mFeatureMatchMatrix[leftViewIdx][rightViewIdx])
                {
                    // 要区分左右视图，才能正确找到DMatch中的索引，queryIdx是左视图中特征点的索引，trainIdx是右视图的。
                    // 找到这个2D点索引
                    int matched2DPointInNewView = -1;
                    if (originatingViewIndex < viewIdx)
                    { // originating view is 'left'
                        if (m.queryIdx == originatingViewFeatureIndex)
                        {
                            matched2DPointInNewView = m.trainIdx;
                        }
                    }
                    else
                    { // originating view is 'right'
                        if (m.trainIdx == originatingViewFeatureIndex)
                        {
                            matched2DPointInNewView = m.queryIdx;
                        }
                    }
                    // 那么就建立了一对2D-3D点关联
                    if (matched2DPointInNewView >= 0)
                    {
                        // This point is matched in the new view
                        const Features &newViewFeatures = mvImageFeatures[viewIdx];
                        match2D3D.points2D.push_back(newViewFeatures.points[matched2DPointInNewView]);
                        match2D3D.points3D.push_back(cloudPoint.p);
                        found2DPoint = true;
                        break;
                    }
                }
                // 当前这个3D点已经建立了关联
                if (found2DPoint)
                {
                    break;
                }
            }
        }
        matches[viewIdx] = match2D3D;
    }
    return matches;
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

    saveCloudAndCamerasToPLY("test");
    // addMoreViewsToReconstruction();

    cout << "------------Done--------------" << endl;
    return RunResult::OKAY;
}