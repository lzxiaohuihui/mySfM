/**
 * @ Author: LZH
 * @ Create Time: 2022-05-20 10:37:19
 * @ Modified by: LZH
 * @ Modified time: 2022-05-20 10:52:09
 * @ Description:
 */

#include <eigen3/Eigen/Core>

class Point2D
{
public:
    Point2D();

    inline const Eigen::Vector2d& XY() const;
    inline Eigen::Vector2d& XY();
    inline double X() const;
    inline double Y() const;
    inline void SetXY(const Eigen::Vector2d &xy);

    inline uint32_t Point3DId() const;
    inline bool HasPoint3D() const;
    inline void SetPoint3DId(const uint32_t point3D_id);

private:
    Eigen::Vector2d xy_;
    uint32_t point3D_id_;
};