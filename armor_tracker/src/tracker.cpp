#include <cmath>
#include <cstddef>
#include <float.h>

// ROS2
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <armor_interfaces/msg/armors.hpp>
#include "tracker.h"

namespace armor_auto_aim {

void TrackerStateMachine::update(bool detector_result) {
    if (m_state == State::Lost) {
        if (!detector_result) {
            m_detect_count = 0;
            m_lost_count = 0;
        }
    } else if (m_state == State::Detecting) {
        if (detector_result) {
            m_detect_count++;
            if (m_detect_count > m_tracking_threshold) {
                m_state = State::Tracking;
                m_detect_count = 0;
            }
        } else {
            m_detect_count = 0;
            m_state = State::Lost;
        }
    } else if (m_state == State::Tracking) {
        if (!detector_result) {
            m_state = State::TempLost;
        }
    } else if (m_state == State::TempLost) {
        if (detector_result) {
            m_lost_count = 0;
            m_state = State::Tracking;
        } else {
            m_lost_count++;
            if (m_lost_count > m_lost_threshold) {
                m_lost_count = 0;
                m_state = State::Lost;
                std::cout << "[Tracker] state changed to Lost." << std::endl;
            }
        }
    }
}

void Tracker::initTracker(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    if (armors_msg->armors.empty()) {
        std::cout << "No armors to init tracker." << std::endl;
        return;
    }
    if (!ekf) {
        std::cout << "EKF not created yet." << std::endl;
        return;
    }

    // 优先选择距离最近的装甲板
    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];
    for (const auto& armor: armors_msg->armors) {
        if (armor.distance_to_center < min_distance) {
            min_distance = armor.distance_to_center;
            tracked_armor = armor;
        }
    }
    initEkf(tracked_armor);
    m_tracker_state_machine.initState();
    m_tracked_id = tracked_armor.number;
    std::cout << "[Tracker] Start Detecting" << std::endl;
    updateArmorNum(tracked_armor);
}
// 计算旋转中心
Eigen::Vector3d Tracker::computeRotationCenter(const Armor& armor, double radius) {
    double x_c = armor.pose.x - radius * cos(armor.yaw);
    double y_c = armor.pose.y - radius * sin(armor.yaw);
    return {x_c, y_c, armor.pose.z};
}

// 计算其他装甲板的位置
Armor Tracker::computeOtherArmor(const Eigen::Vector3d& center, double radius, double base_yaw, double angle_offset_rad) {
    double new_yaw = base_yaw + angle_offset_rad;
    // 保持 yaw 在 [-π, π] 范围内
    new_yaw = fmod(new_yaw + KDL::PI, 2 * KDL::PI) - KDL::PI;

    double x = center.x() + radius * cos(new_yaw);
    double y = center.y() + radius * sin(new_yaw);
    Armor armor;
    armor.yaw = new_yaw;
    armor.pose.x = x;
    armor.pose.y = y;
    armor.pose.z = center[2];

    return armor;

}

void Tracker::updateTracker(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    bool has_measurement = false;
    m_target_predict_state = ekf->predict(); // 先预测
    if (!armors_msg->armors.empty()) {
        for (const auto &armor : armors_msg->armors) {
            const auto &p = armor.world_pose.position;
            // 观测量 z = [x_armor, y_armor, z_armor, yaw_armor]
            // 注意这里的 yaw 可以通过四元数转换得到
            double measured_yaw = orientationToYaw(armor.world_pose.orientation);
            Armor armorValue;
            armorValue.yaw = measured_yaw;
            armorValue.pose.x = p.x;
            armorValue.pose.y = p.y;
            armorValue.pose.z = p.z;
            auto center = computeRotationCenter(armorValue, 0.31);
            m_left_armor = computeOtherArmor(center, 0.31, measured_yaw, -2 * KDL::PI / 3);
            m_right_armor = computeOtherArmor(center, 0.31, measured_yaw, 2 * KDL::PI / 3);
            Eigen::Vector4d measurement(p.x, p.y, p.z, measured_yaw);
            ekf->update(measurement);
            has_measurement = true;
            tracked_armor = armor; // 记录当前用来更新的装甲板
        }
    }
    m_tracker_state_machine.update(has_measurement);
}

void Tracker::initEkf(const armor_interfaces::msg::Armor &armor) {
    // x(0)= x_center, x(1)= vx_center, x(2)= y_center, x(3)= vy_center
    // x(4)= z_center, x(5)= vz_center, x(6)= yaw, x(7)= yaw_v
    double xa = armor.world_pose.position.x;
    double ya = armor.world_pose.position.y;
    double za = armor.world_pose.position.z;

    double yaw = orientationToYaw(armor.world_pose.orientation);

    dz = 0.0;
    m_last_yaw = yaw;
    m_last_pitch = 0.0;
    m_last_roll  = 0.0;

    // 初始时假设中心速度未知，就先设为0；yaw_v 也先设为0
    m_target_predict_state = Eigen::VectorXd::Zero(8);
    m_target_predict_state << xa, 0.0, ya, 0.0, za, 0.0, yaw, 1.2;

    ekf->setState(m_target_predict_state);
    std::cout << "[Tracker] EKF state init: x=" << xa 
              << ", y=" << ya << ", z=" << za << ", yaw=" << yaw << std::endl;

    m_tracker_state_machine.initState();
}

void Tracker::updateArmorNum(const armor_interfaces::msg::Armor &armor) {
    // 示例，根据装甲板编号来判定类型
    if (armor.number == 10)
        m_armor_num = 3;
    else if (armor.type == "LARGE" && 
            (armor.number == 3 || armor.number == 4 || armor.number == 5)) 
    {
        m_armor_num = 2;
    } else {
        m_armor_num = 4;
    }
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) {
    // 简化版本：直接用 tf2::Matrix3x3
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double r, p, y;
    tf2::Matrix3x3(tf_q).getRPY(r, p, y);
    // 如果想做连续解包，可使用 shortest_angular_distance 累加
    // 这里简单返回 y
    m_last_yaw = m_last_yaw + angles::shortest_angular_distance(m_last_yaw, y);
    return m_last_yaw;
}

AngleInfo Tracker::orientationToAngle(const geometry_msgs::msg::Quaternion &q) {
    AngleInfo info;
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double r, p, y;
    tf2::Matrix3x3(tf_q).getRPY(r, p, y);
    m_last_yaw = m_last_yaw + angles::shortest_angular_distance(m_last_yaw, y);
    m_last_pitch = m_last_pitch + angles::shortest_angular_distance(m_last_pitch, p);
    m_last_roll = m_last_roll + angles::shortest_angular_distance(m_last_roll, r);
    info.yaw_angle = angles::to_degrees(m_last_yaw);
    info.pitch_angle = angles::to_degrees(m_last_pitch);
    info.roll_angle = angles::to_degrees(m_last_roll);
    info.roll  = m_last_roll;
    info.pitch = m_last_pitch;
    info.yaw   = m_last_yaw;
    return info;
}


} // namespace armor_auto_aim