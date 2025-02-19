#ifndef PNPSLOVER_H
#define PNPSLOVER_H

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <eigen3/Eigen/Core>

#include <armor_detect.h>

namespace armor_auto_aim {
class ArmorPnpSlover {
public:
    ArmorPnpSlover() = default;
    ~ArmorPnpSlover() = default;
    ArmorPnpSlover(const std::array<double, 9>& intrinsic_matrix,
                   const std::vector<double>& distortion_vector);
    bool pnpSlove(const Armor &armor, cv::Mat& rvec, cv::Mat& tvec);
    cv::Mat getCameraMatrix() {return m_intrinsic_matrix;}
    cv::Mat getDistortionVector() {return m_distortion_vector;}

    geometry_msgs::msg::Quaternion orientationFromRvec(const cv::Mat& r_vec);

    float computeArmorToCenter(const cv::Point2f &center);
private:
// mm
    static constexpr float LARGE_ARMOR_WIDTH = 230;
    static constexpr float LARGE_ARMOR_HEIGHT = 54;
    static constexpr float SMALL_ARMOR_WIDTH = 130.4;
    static constexpr float SMALL_ARMOR_HEIGHT = 55;
    // static constexpr float SMALL_ARMOR_WIDTH = 0;
    // static constexpr float SMALL_ARMOR_HEIGHT = 0;
    cv::Mat m_rvec;
    cv::Mat m_tvec;
    cv::Mat m_intrinsic_matrix;
    cv::Mat m_distortion_vector;
    std::vector<cv::Point3f> m_large_armor_point3d;
    std::vector<cv::Point3f> m_small_armor_point3d;
    tf2::Quaternion last_q;
    bool is_first_frame  = true;
    double last_roll = 0.0;
    double last_pitch = 0.0;
    double last_yaw = 0.0;
};
} // namespace armor_auto_aim
#endif