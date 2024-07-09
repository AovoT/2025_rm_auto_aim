#include <pnpslover.h>


namespace armor_auto_aim {

ArmorPnpSlover::ArmorPnpSlover(const std::array<double, 9>& intrinsic_matrix,
                               const std::vector<double>& distortion_vector) 
                               : m_intrinsic_matrix(cv::Mat(3, 3, CV_64F, const_cast<double*>(intrinsic_matrix.data())).clone()),
                               m_distortion_vector(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_vector.data())).clone()) {
    constexpr double small_armro_half_x = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double small_armro_half_y = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    constexpr double large_armro_half_x = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double large_armro_half_y = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    // 相机坐标系和相机坐标系都设为   z向前   x向右  y向下 
    m_small_armor_point3d.push_back(cv::Point3f(-small_armro_half_x, -small_armro_half_y, 0)); 
    m_small_armor_point3d.push_back(cv::Point3f(-small_armro_half_x, small_armro_half_y, 0)); 
    m_small_armor_point3d.push_back(cv::Point3f(small_armro_half_x, small_armro_half_y, 0)); 
    m_small_armor_point3d.push_back(cv::Point3f(small_armro_half_x, -small_armro_half_y, 0)); 

    m_large_armor_point3d.push_back(cv::Point3f(large_armro_half_x, -large_armro_half_y, 0)); 
    m_large_armor_point3d.push_back(cv::Point3f(large_armro_half_x, large_armro_half_y, 0)); 
    m_large_armor_point3d.push_back(cv::Point3f(large_armro_half_x, large_armro_half_y, 0)); 
    m_large_armor_point3d.push_back(cv::Point3f(large_armro_half_x, large_armro_half_y, 0)); 

}

bool ArmorPnpSlover::pnpSlove(const Armor &armor, cv::Mat& rvec, cv::Mat& tvec) {
    try {
        if (armor.ifsmall) {
            return cv::solvePnP(m_small_armor_point3d, armor.points, m_intrinsic_matrix, m_distortion_vector,
                                rvec, tvec, false, cv::SOLVEPNP_IPPE);
        } else if(!armor.ifsmall) {
            return cv::solvePnP(m_large_armor_point3d, armor.points, m_intrinsic_matrix, m_distortion_vector,
                                rvec, tvec, false, cv::SOLVEPNP_IPPE);
        } else {
            return false;
        }

    } catch (const cv::Exception& e) {
        std::cout << "Intrinsic Matrix:" << std::endl << m_intrinsic_matrix << std::endl;
        std::cout << "Distortion Coefficients:" << std::endl << m_distortion_vector << std::endl;

        for (int i = 0; i < 4; ++i) {
            auto item = armor.points[i];
            RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "%d: %f, %f;", i, item.x, item.y);
        }
        RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "-----");
        return false;
    }
}

float ArmorPnpSlover::computeArmorToCenter(const cv::Point2f &center) {
    float cx = m_intrinsic_matrix.at<double>(0, 2);
    float cy = m_intrinsic_matrix.at<double>(1, 2);
    return cv::norm(center - cv::Point2f(cx, cy));

}

geometry_msgs::msg::Quaternion ArmorPnpSlover::orientationFromRvec(const cv::Mat &r_vec)  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(r_vec, rotation_matrix);
    tf2::Matrix3x3 tf2_rotation_matrix(
    rotation_matrix.at<double>(0, 0),rotation_matrix.at<double>(0, 1),rotation_matrix.at<double>(0, 2),
    rotation_matrix.at<double>(1, 0),rotation_matrix.at<double>(1, 1),rotation_matrix.at<double>(1, 2),
    rotation_matrix.at<double>(2, 0),rotation_matrix.at<double>(2, 1),rotation_matrix.at<double>(2, 2)
    );
    tf2::Quaternion tf2_q;
    tf2_rotation_matrix.getRotation(tf2_q);
    return tf2::toMsg(tf2_q);
}


} // namespace armor_auto_aim


































































































































































































