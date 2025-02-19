#ifndef ARMOR_TRACKER_NODE_H
#define ARMOR_TRACKER_NODE_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>

#include <armor_interfaces/msg/armor.hpp>
#include <armor_interfaces/msg/armors.hpp>
#include <armor_interfaces/msg/target.hpp>

#include "kalman_filter.h"
#include "tracker.h"
#include "HKCameraNode.h"

namespace armor_auto_aim {
class ArmorTrackerNode : public rclcpp::Node {
public:
    ArmorTrackerNode(const rclcpp::NodeOptions &options);
    void declareParameters();
    void subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg);
    void initExtentedKalman();
    void initTracker();
    cv::Point2d project3dTo2d(const cv::Point3d& pt_3d);
private:
    std::string m_odom_frame;
    rclcpp::Time m_last_stamp;
    // rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr m_armors_sub;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    rclcpp::Publisher<armor_interfaces::msg::Target>::SharedPtr m_target_pub;
    message_filters::Subscriber<armor_interfaces::msg::Armors> m_armors_sub;
    std::shared_ptr<tf2_ros::MessageFilter<armor_interfaces::msg::Armors>> m_tf_filter;
    float m_dt;
    std::shared_ptr<ExtendedKalmanFilter> m_ekf = nullptr;
    Tracker m_tracker;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    tf2_ros::StaticTransformBroadcaster m_static_broadcaster;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;
    HKCameraNode::CameraInfo m_camera_info;
    std::ofstream m_output_file;
    int all_time = 1;
    int rel_time = 1;
    double m_yaw_v;
    double m_yaw_th_l;
    double m_yaw_th_r;

};
} // namespace armor_auto_aim
#endif // ARMOR_TRACKER_NODE_