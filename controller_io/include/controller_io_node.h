#ifndef ARMOR_AUTO_AIM_CONTROLLER_IO_H
#define ARMOR_AUTO_AIM_CONTROLLER_IO_H

#include <cmath>
#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>


#include <custom_serial_interfaces/msg/receive.hpp>
#include <custom_serial_interfaces/srv/send_package.hpp>
#include <armor_interfaces/msg/target.hpp>
#include <custom_serial_interfaces/msg/receive.hpp>
#include "auto_aim_package.h"

namespace armor_auto_aim {
class ControllerIONode : public rclcpp::Node {
enum FunId {
    PCId = 0,
    TimestampPackte = 0,
    AimPacket= 1
};
enum DecisonId {
    ControllerId = 10,
    SlaveControllerId = 11,
    NeedUpdateTimestamp = 0,
    GimbalPose = 1,
    SetTargetColor = 2
};
public:
    ControllerIONode(const rclcpp::NodeOptions &options);
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_gimbal_tf_broad;
    std::array<bool, 2> m_trakcer_state;
    std::thread m_send_thread;
    // Subscription
    rclcpp::Subscription<custom_serial_interfaces::msg::Receive>::SharedPtr m_serial_sub;
    rclcpp::Subscription<armor_interfaces::msg::Target>::SharedPtr m_tracker_sub;
    // Client
    rclcpp::Client<custom_serial_interfaces::srv::SendPackage>::SharedPtr m_serial_cli;
    void setDetectorColor(const rclcpp::Parameter& param);
    void setParam( int index, const rclcpp::Parameter& param, const std::function<void(void)> callback);
    void trackerCallback(const armor_interfaces::msg::Target::SharedPtr target_msg);
    // Callback
    void serialHandle(const custom_serial_interfaces::msg::Receive::SharedPtr serial_msg);
};
}; // namespace armor_auto_aim

#endif
