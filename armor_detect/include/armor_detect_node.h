#ifndef ARMOR_DETECT_NODE_H
#define ARMOR_DETECT_NODE_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <armor_interfaces/msg/armor.hpp>
#include <armor_interfaces/msg/armors.hpp>

#include <opencv2/opencv.hpp>

#include "armor_detect.h"
#include "armor_classify.h"
#include "pnpslover.h"

namespace armor_auto_aim {
class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode(const rclcpp::NodeOptions &options);

    void declareParams();

    void subCamInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info);

    void subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    void updateThresold();

    rcl_interfaces::msg::SetParametersResult paramChangedCallback(const std::vector<rclcpp::Parameter> &parameters);
public:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle_; // 处理参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameter_event_handler;
    //功能
    ArmorDetect m_armor_detector; // 侦查器
    ArmorNumberClassify m_armor_classify; // 数字分类器
    ArmorPnpSlover m_armor_pnp_slover; // pnp解算
    //数据
    AllThresold m_declare_all_thresold;
    ClassifyInfo m_classify_info;
    armor_interfaces::msg::Armors m_armors;
    // 订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_sub;
    // 推送
    image_transport::Publisher m_result_img_pub;
    rclcpp::Publisher<armor_interfaces::msg::Armors>::SharedPtr m_armors_publish;
};
} // armor_auto_aim
#endif