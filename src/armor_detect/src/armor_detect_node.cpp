#include <armor_detect_node.h>
// std
#include <sstream>
#include <algorithm>
#include <functional>
#include <filesystem>

// 3rdlibs
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <fmt/format.h>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace armor_auto_aim {
    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("armor_detector", options) {
        RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode Constructor");
        declareParams();
        m_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ArmorDetectorNode::subImageCallback, this, std::placeholders::_1)
        );

        m_cam_info_sub = this->create_subscription<
            sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(),
            std::bind(&ArmorDetectorNode::subCamInfoCallback, this, std::placeholders::_1));

        m_armors_publish = this->create_publisher<armor_interfaces::msg::Armors>("armor_detector/armors", rclcpp::SensorDataQoS());

        // 绑定参数更改回调函数
        m_parameter_event_handler = this->add_on_set_parameters_callback(std::bind(&ArmorDetectorNode::paramChangedCallback, this, std::placeholders::_1));

        m_armor_detector = ArmorDetect(m_declare_all_thresold);
        m_armor_classify = ArmorNumberClassify(m_classify_info);
        RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode Initialized");
    }

    void ArmorDetectorNode::declareParams() {
        
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "Parameter range";
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = 0;
        range.to_value = 255;
        range.step = 1;
        desc.integer_range.push_back(range);

        RCLCPP_INFO(this->get_logger(), "Declaring Parameters");
        //声明数字分类中的各种信息
        m_classify_info.model_path = this->declare_parameter("model_path", "/home/mayuqi/Desktop/2025_rm_auto_aim/src/armor_detect/model/mlp.onnx");
        m_classify_info.infer_place = this->declare_parameter("infer_place", "CPU");
        m_classify_info.confidence = this->declare_parameter<int>("confidence", 0.3);

        // 声明在传统视觉识别装甲板中的各种阈值
        m_declare_all_thresold.process_way = this->declare_parameter("process_way", "rgb");
        // 图像处理阈值
        m_declare_all_thresold.h_up = this->declare_parameter<int>("h_up", 0, desc);
        m_declare_all_thresold.s_up = this->declare_parameter<int>("s_up", 0, desc);
        m_declare_all_thresold.v_up = this->declare_parameter<int>("v_up", 0, desc);
        m_declare_all_thresold.h_lower = this->declare_parameter<int>("h_lower", 0, desc);
        m_declare_all_thresold.s_lower = this->declare_parameter<int>("s_lower", 0, desc);
        m_declare_all_thresold.v_lower = this->declare_parameter<int>("v_lower", 0, desc);
        m_declare_all_thresold.gray_thresold = this->declare_parameter<int>("gray_thresold", 163, desc);
        m_declare_all_thresold.rgb_thresold = this->declare_parameter<int>("rgb_thresold", 45, desc);
        m_declare_all_thresold.detect_color = this->declare_parameter("detect_color", "BLUE");
        m_declare_all_thresold.morphologyex_times = this->declare_parameter("morphologyex_times", 3);
        
        // 筛选灯条阈值
        m_declare_all_thresold.max_light_angle = this->declare_parameter("max_light_angle", 30);
        m_declare_all_thresold.max_light_lenght_width_ratio = this->declare_parameter<double>("max_light_lenght_width_ratio", 0.5);
        m_declare_all_thresold.min_light_lenght_width_ratio = this->declare_parameter<double>("min_light_lenght_width_ratio", 0.01);
        m_declare_all_thresold.min_light_area = this->declare_parameter("min_light_area", 200);
        m_declare_all_thresold.max_light_area = this->declare_parameter("max_light_area", 50000);
        // 筛选装甲板的阈值
        m_declare_all_thresold.min_armor_light_area_ratio = this->declare_parameter<double>("min_armor_light_area_ratio", 0.6);
        m_declare_all_thresold.max_armor_light_area_ratio = this->declare_parameter<double>("max_armor_light_area_ratio", 2);
        m_declare_all_thresold.min_armor_area = this->declare_parameter("min_armor_area", 100);
        m_declare_all_thresold.max_armor_area = this->declare_parameter("max_armor_area", 10000);
        m_declare_all_thresold.max_armor_lenght_width_ratio = this->declare_parameter<double>("max_armor_lenght_width_ratio", 2.5);
        m_declare_all_thresold.min_armor_lenght_width_ratio = this->declare_parameter<double>("min_armor_lenght_width_ratio", 0.4);

        RCLCPP_INFO(this->get_logger(), "Parameters Declared Successfully");

        // 添加日志输出以确认参数声明
        RCLCPP_INFO(this->get_logger(), "process_way: %s", m_declare_all_thresold.process_way.c_str());
        RCLCPP_INFO(this->get_logger(), "h_up: %d", m_declare_all_thresold.h_up);
        RCLCPP_INFO(this->get_logger(), "s_up: %d", m_declare_all_thresold.s_up);
        RCLCPP_INFO(this->get_logger(), "v_up: %d", m_declare_all_thresold.v_up);
        RCLCPP_INFO(this->get_logger(), "h_lower: %d", m_declare_all_thresold.h_lower);
        RCLCPP_INFO(this->get_logger(), "s_lower: %d", m_declare_all_thresold.s_lower);
        RCLCPP_INFO(this->get_logger(), "v_lower: %d", m_declare_all_thresold.v_lower);
        RCLCPP_INFO(this->get_logger(), "gray_thresold: %d", m_declare_all_thresold.gray_thresold);
        RCLCPP_INFO(this->get_logger(), "rgb_thresold: %d", m_declare_all_thresold.rgb_thresold);
        RCLCPP_INFO(this->get_logger(), "detect_color: %s", m_declare_all_thresold.detect_color.c_str());
        RCLCPP_INFO(this->get_logger(), "max_light_angle: %d", m_declare_all_thresold.max_light_angle);
        RCLCPP_INFO(this->get_logger(), "max_light_lenght_width_ratio: %f", m_declare_all_thresold.max_light_lenght_width_ratio);
        RCLCPP_INFO(this->get_logger(), "min_light_lenght_width_ratio: %f", m_declare_all_thresold.min_light_lenght_width_ratio);
        RCLCPP_INFO(this->get_logger(), "min_light_area: %d", m_declare_all_thresold.min_light_area);
        RCLCPP_INFO(this->get_logger(), "max_light_area: %d", m_declare_all_thresold.max_light_area);
        RCLCPP_INFO(this->get_logger(), "max_armor_area: %d", m_declare_all_thresold.max_armor_area);
        RCLCPP_INFO(this->get_logger(), "min_armor_area: %d", m_declare_all_thresold.min_armor_area);
        RCLCPP_INFO(this->get_logger(), "min_armor_lenght_width_ratio: %f", m_declare_all_thresold.min_armor_lenght_width_ratio);
        RCLCPP_INFO(this->get_logger(), "max_armor_lenght_width_ratio: %f", m_declare_all_thresold.max_armor_lenght_width_ratio);
        RCLCPP_INFO(this->get_logger(), "morphologyex_times: %d", m_declare_all_thresold.morphologyex_times);

    }

    void ArmorDetectorNode::subCamInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
        m_armor_pnp_slover = ArmorPnpSlover(cam_info->k, cam_info->d);
        m_cam_info_sub.reset();
    }

    void ArmorDetectorNode::subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
        updateThresold();
        // 图像处理代码
        cv::Mat img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        if (img.empty())
            return;
        auto s = this->now();
        m_armor_detector.startDetect(img);

        std::vector<Armor> armors = m_armor_detector.getArmors();
        if (!armors.empty()) {
            m_armor_classify.startClassify(armors);

        }
        m_armor_detector.updateArmors(armors);
        m_armor_detector.drawArmor(img);
        armor_interfaces::msg::Armor armor_msg;
        for (const auto &armor : armors) {
            cv::Mat r_vec, t_vec;
            if (m_armor_pnp_slover.pnpSlove(armor, r_vec, t_vec)) {
                armor_msg.color = this->get_parameter("detect_color").as_string();
                armor_msg.number = armor.number_class;
                armor_msg.distance_to_center = m_armor_pnp_slover.computeArmorToCenter(armor.center);
                armor_msg.pose.position.x = t_vec.at<double>(0); 
                armor_msg.pose.position.y = t_vec.at<double>(1); 
                armor_msg.pose.position.z = t_vec.at<double>(2); 
                armor_msg.pose.orientation = m_armor_pnp_slover.orientationFromRvec(r_vec);
                m_armors.armors.push_back(armor_msg);
            }
        }
        m_armors_publish->publish(m_armors); 
    }

    rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::paramChangedCallback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(this->get_logger(), "参数更改回调函数被调用");

        for (const auto &parameter : parameters) {
            const std::string &param_name = parameter.get_name();
            if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                int param_value = parameter.as_int();
                RCLCPP_INFO(this->get_logger(), "参数名称: %s, 值: %d", param_name.c_str(), param_value);
                if ((param_name == "h_up" || param_name == "s_up" || param_name == "v_up") && param_value <= 255) {
                    std::string lower_param_name = param_name.substr(0, 1) + "_lower";
                    int lower_value = this->get_parameter(lower_param_name).as_int();
                    RCLCPP_INFO(this->get_logger(), "lower_param_name: %s, lower_value: %d", lower_param_name.c_str(), lower_value);

                    if (param_value > lower_value) {
                    } else {
                        result.successful = false;
                    }       
                } else if ((param_name == "h_lower" || param_name == "s_lower" || param_name == "v_lower") && param_value <= 255) {
                    std::string upper_param_name = param_name.substr(0, 1) + "_up";
                    int upper_value = this->get_parameter(upper_param_name).as_int();
                    RCLCPP_INFO(this->get_logger(), "upper_param_name: %s, upper_value: %d", upper_param_name.c_str(), upper_value);

                    if (param_value < upper_value) {
                    } else {
                        result.successful = false;
                    }
                } else {
                }
            } else if(param_name == "model_path") {
                std::filesystem::path model_path = parameter.as_string();
                if (std::filesystem::exists(model_path) && model_path.extension() == ".onnx") {
                    m_classify_info.model_path = model_path;
                    m_armor_classify = ArmorNumberClassify(m_classify_info);
                    result.successful = true;
                } else {
                    result.successful = false;
                }
            } else if (param_name == "confidence") {
                if (parameter.as_double() <= 1 && parameter.as_double() > 0) {
                    m_classify_info.confidence = parameter.as_double();
                    m_armor_classify = ArmorNumberClassify(m_classify_info);
                    result.successful = true;
                } else {
                    result.successful = false;
                }

            } else if (param_name == "infer_place"){
                std::string infer_place = parameter.as_string();
                if (infer_place == "GPU" && infer_place == "CPU" && infer_place == "AUTO") {
                    m_classify_info.infer_place = infer_place;
                    m_armor_classify = ArmorNumberClassify(m_classify_info);
                    result.successful = true;
                } else {
                    result.successful = false;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "不支持的参数类型: %s", param_name.c_str());
            }
        } 

        return result;
    }

    void ArmorDetectorNode::updateThresold() {
        // RCLCPP_INFO(this->get_logger(), "Updating Thresholds");

        m_declare_all_thresold.h_up = this->get_parameter("h_up").as_int();
        m_declare_all_thresold.s_up = this->get_parameter("s_up").as_int();
        m_declare_all_thresold.v_up = this->get_parameter("v_up").as_int();
        m_declare_all_thresold.h_lower = this->get_parameter("h_lower").as_int();
        m_declare_all_thresold.s_lower = this->get_parameter("s_lower").as_int();
        m_declare_all_thresold.v_lower = this->get_parameter("v_lower").as_int();
        m_declare_all_thresold.gray_thresold = this->get_parameter("gray_thresold").as_int();
        m_declare_all_thresold.rgb_thresold = this->get_parameter("rgb_thresold").as_int();
        m_declare_all_thresold.detect_color = this->get_parameter("detect_color").as_string();
        m_declare_all_thresold.max_light_angle = this->get_parameter("max_light_angle").as_int();
        m_declare_all_thresold.max_light_lenght_width_ratio = this->get_parameter("max_light_lenght_width_ratio").as_double();
        m_declare_all_thresold.min_light_lenght_width_ratio = this->get_parameter("min_light_lenght_width_ratio").as_double();
        m_declare_all_thresold.min_light_area = this->get_parameter("min_light_area").as_int();
        m_declare_all_thresold.max_light_area = this->get_parameter("max_light_area").as_int();
        m_declare_all_thresold.max_armor_area = this->get_parameter("max_armor_area").as_int();
        m_declare_all_thresold.min_armor_area = this->get_parameter("min_armor_area").as_int();
        m_declare_all_thresold.min_armor_lenght_width_ratio = this->get_parameter("min_armor_lenght_width_ratio").as_double();
        m_declare_all_thresold.max_armor_lenght_width_ratio = this->get_parameter("max_armor_lenght_width_ratio").as_double();
        m_declare_all_thresold.morphologyex_times = this->get_parameter("morphologyex_times").as_int();
        m_declare_all_thresold.min_armor_light_area_ratio = this->get_parameter("min_armor_light_area_ratio").as_double();
        m_declare_all_thresold.max_armor_light_area_ratio = this->get_parameter("max_armor_light_area_ratio").as_double();

        // RCLCPP_INFO(this->get_logger(), "阈值更新成功");
        m_armor_detector.updateAllThresold(m_declare_all_thresold);
}


} // namespace armor_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorDetectorNode)
