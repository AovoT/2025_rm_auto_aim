/**
 * @projectName armor_auto_aim
 * @file track.cpp
 * @brief 
 * 
 * @author yx 
 * @date 2023-11-03 19:29
 */


#include <float.h>

// ROS2
#include <rclcpp/logging.hpp>
#include <tf2/convert.h>
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
            if (++m_detect_count > m_tracking_threshold) m_state = State::Tracking;
        } else {
            m_detect_count = 0;
            m_state = State::Lost;
        }
    } else if (m_state == State::Tracking) {
        if (!detector_result) m_state = State::TempLost;

    } else if (m_state == State::TempLost) {
        if (detector_result) {
            m_lost_count = 0;
            m_state = State::Tracking;
        } else {
            if (++m_lost_count > m_lost_threshold) {
                m_lost_count = 0;
                m_state = State::Lost;
            }
        }
    }
}

void Tracker::initTracker(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    if (armors_msg->armors.empty()) {
        std::cout << "info is empty" << std::endl;
        return;
    }
    if (ekf == nullptr) {
        std::cout << "ekf == nullptr" << std::endl;
        return;
    }

    // Select tracked armor(优先选择距离相机光心最近的)
    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];
    for (const auto& armor: armors_msg->armors) {
        if (armor.distance_to_center < min_distance) {
            min_distance = armor.distance_to_center;
            tracked_armor = armor;
        }
    }
    std::cout << "min distance is :" << min_distance << std::endl;
    // initialization
    initEkf(tracked_armor);
    m_tracker_state_machine.initState();
    m_tracked_id = tracked_armor.number;
    updateArmorNum(tracked_armor);
}

void Tracker::updateTracker(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    // prior
    bool is_matched = false;
    const armor_interfaces::msg::Armor* same_id_armor;
    std::vector<armor_interfaces::msg::Armor> same_id_armors;
    int same_id_armor_count = 0;
    m_target_predict_state = ekf->update();
    // xxx
    if (!isTracking())
        m_is_complex_pattern = false;
    // 寻找tracked装甲板
    if (!armors_msg->armors.empty()) {
        double min_position_difference = DBL_MAX;
        double yaw_difference = DBL_MAX;
        Eigen::Vector3d measurement_position_vec;
        Eigen::Vector3d predicted_position_vec(getArmorPositionFromState(m_target_predict_state));
        for (const auto& armor: armors_msg->armors) {
            if (armor.number != m_tracked_id) continue;
            same_id_armor = &armor;
            same_id_armor_count++;
            same_id_armors.push_back(armor);
            auto const& p = armor.world_pose.position;
            measurement_position_vec = Eigen::Vector3d(p.x, p.y, p.z);
            double position_difference = (predicted_position_vec - measurement_position_vec).norm();
            if (position_difference < min_position_difference) {
                min_position_difference = position_difference;
                yaw_difference = std::abs(m_target_predict_state[6] - orientationToYaw(armor.world_pose.orientation));
                tracked_armor = armor;
            }
        }
        // 后验及装甲板跳变处理
        if (same_id_armor_count >= 1) {

            for (const auto& armor: same_id_armors) {
                double yd = std::abs(m_target_predict_state[6] - orientationToYaw(armor.world_pose.orientation));
                if (yd < yaw_difference) {
                    yaw_difference = yd;
                    same_id_armor = &armor;
                }
            }

        }
        if (min_position_difference < m_max_match_distance &&
            yaw_difference < m_max_match_yaw) {
            is_matched = true;
            auto const& p = tracked_armor.world_pose.position;
            measurement = Eigen::Vector4d(p.x, p.y, p.z, orientationToYaw(tracked_armor.world_pose.orientation));
            m_target_predict_state = ekf->predict(measurement);

        } else if (same_id_armor_count >= 1 && yaw_difference > m_max_match_yaw) {
            // handleArmorJump(*same_id_armor);
        } else {
            RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("armor_tracker"), 
                min_position_difference > m_max_match_distance && same_id_armor_count != 0,
                "No matched armor(because Position)! %lf > %lf(%lf)",
                min_position_difference, m_max_match_distance, min_position_difference - m_max_match_distance);
            RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("armor_tracker"), 
                yaw_difference > m_max_match_yaw && same_id_armor_count != 0,
                "No matched armor(because Yaw)! %lf > %lf(%lf)",
                yaw_difference, m_max_match_yaw, yaw_difference - m_max_match_yaw);
            RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("armor_tracker"), 
                same_id_armor_count != 1,
                "No matched armor(because same count)! %d", same_id_armor_count);
        }
    }
    if (m_target_predict_state(8) < 0.12) {
        m_target_predict_state(8) = 0.12;
        ekf->setState(m_target_predict_state);
    } else if (m_target_predict_state(8) > 0.4) {
        m_target_predict_state(8) = 0.4;
        ekf->setState(m_target_predict_state);
    }

    m_tracker_state_machine.update(is_matched);
}

void Tracker::initEkf(const armor_interfaces::msg::Armor& armor) {
    double xa = armor.world_pose.position.x;
    double ya = armor.world_pose.position.y;
    double za = armor.world_pose.position.z;
    dz = m_last_yaw = 0.0;
    double yaw = orientationToYaw(armor.world_pose.orientation);
    double r = 0.28;
    m_target_predict_state = Eigen::VectorXd::Zero(9);
    m_target_predict_state << xa + r*cos(yaw), 0, ya + r*sin(yaw), 0, za, 0, yaw, 0, r;
    another_r = r;
    ekf->setState(m_target_predict_state);
}


void Tracker::updateArmorNum(const armor_interfaces::msg::Armor& armor) {
    if (armor.number == 10)
        m_armor_num = 3;
    else if (armor.type == "LARGE" && (armor.number == 3 || armor.number == 4 || armor.number == 5))
        m_armor_num = 2;
    else
        m_armor_num = 4;
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double r, p, y;
    tf2::Matrix3x3(tf_q).getRPY(r, p, y);

    m_last_yaw = m_last_yaw + angles::shortest_angular_distance(m_last_yaw, y);
    return m_last_yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd& x) {
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r*cos(yaw);
    double ya = yc - r*sin(yaw);
    return Eigen::Vector3d(xa, ya, za);
}
} // armor_auto_aim



