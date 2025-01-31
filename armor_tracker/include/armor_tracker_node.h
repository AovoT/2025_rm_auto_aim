#ifndef ARMOR_TRACKER_NODE_H
#define ARMOR_TRACKER_NODE_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <eigen3/Eigen/Core>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <armor_interfaces/msg/armor.hpp>
#include <armor_interfaces/msg/armors.hpp>

#include "kalman_filter.h"

namespace armor_auto_aim {
class ArmorTrackerNode : public rclcpp::Node {
public:
    ArmorTrackerNode(const rclcpp::NodeOptions &options);
    void declareParameters();

    void subarmorCallback(const armor_interfaces::msg::Armors &armors);

    void initExtentedKalman();

    void initTracker();
    
private:
    rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr m_armors_sub;

    float m_dt;

    std::shared_ptr<ExtenedKalmanFilter> m_ekf = nullptr;



};
} // namespace armor_auto_aim
#endif // ARMOR_TRACKER_NODE_