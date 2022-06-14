#include "SiftGPU.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <GL/gl.h>

using namespace std;

int maxSiftMatchPair = 20000;

int main()
{
    vector<vector<SiftGPU::SiftKeypoint>> keys;
    vector<vector<float>> descriptors;
    map<pair<int, int>, vector<cv::DMatch>> matches;
    SiftGPU sift;
    char *myargv[4] = {(char *)"-fo", (char *)"-1", (char *)"-v", (char *)"1"};
    sift.ParseParam(4, myargv);

    int support = sift.CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        cerr << "SiftGPU is not supported!" << endl;
    }

    string fileDir = "/home/lzh/Documents/my_git/mySFM/res";
    vector<cv::string> imgList;
    cv::glob(fileDir, imgList);
    int imgNumbers = imgList.size();

    keys.resize(imgNumbers);
    descriptors.resize(imgNumbers);

    for (int i = 0; i < imgList.size(); ++i)
    {
        // cv::Mat img = cv::imread(fileName);
        sift.RunSIFT(imgList[i].data());
        int num = sift.GetFeatureNum();

        keys[i].resize(num);
        descriptors[i].resize(128 * num);
        sift.GetFeatureVector(&keys[i][0], &descriptors[i][0]);
    }

    SiftMatchGPU matcher;
    matcher.VerifyContextGL();
    matcher.SetMaxSift(maxSiftMatchPair);

    for (int i = 0; i < imgNumbers - 1; ++i)
    {
        for (int j = i + 1; j < imgNumbers; ++j)
        {
            matcher.SetDescriptors(0, keys[i].size(), &descriptors[i][0]);
            matcher.SetDescriptors(1, keys[j].size(), &descriptors[j][0]);
            int(*match_buf)[2] = new int[keys[i].size()][2];
            int num_match = matcher.GetSiftMatch(keys[i].size(), match_buf);
            cout << "Matches between Image " << i << " and image " << j << " are " << num_match << endl;
            vector<cv::DMatch> vMatches;

            // store match point into dmatch vector.
            for (size_t mk = 0; mk < num_match; mk++)
            {
                cv::DMatch match;
                match.queryIdx = match_buf[mk][0];
                match.trainIdx = match_buf[mk][1];

                // cal match points distance
                cv::Point2f p1, p2;
                p1.x = keys[i][match.queryIdx].x;
                p1.y = keys[i][match.queryIdx].y;
                p2.x = keys[i][match.trainIdx].x;
                p2.y = keys[i][match.trainIdx].y;

                float dist = (float)sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
                match.distance = dist;
                vMatches.push_back(match);
            }
            matches[make_pair(i, j)] = vMatches;
            delete[] match_buf;
        }
    }

    // visualization.....
    // auto img = cv::imread(fileName);
    // for (int i = 0; i < num; ++i)
    // {
    //     cv::circle(img, cv::Point2f(keys[i].x, keys[i].y), keys[i].s, (0, 255, 0), 2);
    // }
    // cv::namedWindow("testSift", 0);
    // cv::resizeWindow("testSift", 2000, 1200);
    // cv::imshow("testSift", img);
    // cv::waitKey(0);
    return 0;
}