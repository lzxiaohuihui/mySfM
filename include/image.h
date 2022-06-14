/**
 * @ Author: LZH
 * @ Create Time: 2022-05-20 09:39:12
 * @ Modified by: LZH
 * @ Modified time: 2022-05-23 09:49:20
 * @ Description:
 */
#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>
#include "point2d.h"

class Image
{
public:
    Image();

private:
    uint32_t image_id_;
    // path
    std::string name_;
    uint32_t camera_id_;
    bool registerd_;
    // 观察到的3D点个数
    uint32_t num_points3D_;
    // 有关联的2D点个数
    uint32_t num_observations_;
    // 总的关联数
    uint32_t num_correspondences_;

    // The pose of the image, defined as the transformation from world to image.
    Eigen::Vector4d qvec_;
    Eigen::Vector3d tvec_;

    // 特征点
    std::vector<class Point2D> points2D_;

    // 具有3D点关联的图像点
    std::vector<uint32_t> num_correspondences_have_point3D_;
};
