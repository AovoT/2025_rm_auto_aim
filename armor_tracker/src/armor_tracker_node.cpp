#include <algorithm>
#include <armor_tracker_node.h>
#include <cstdio>
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <fstream>
#include <memory>
#include <ratio>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace armor_auto_aim {

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options)
    : Node("armor_tracker_node", options), m_static_broadcaster(this) 
{
    declareParameters();

    std::filesystem::path filePath = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path autoAimConfigPath = (filePath / "../../config/demo.yaml").lexically_normal();
    YAML::Node config = YAML::LoadFile(autoAimConfigPath.string());
    std::string type = config["type"].as<std::string>();
    std::string lastPath = "../../config/" + type + ".yaml";
    std::filesystem::path cameraConfigPath = (filePath / lastPath).lexically_normal();
    m_camera_info = HKCameraNode::getInstance().getCameraInfo(cameraConfigPath.string());

    m_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("target_marker", 10);
    m_target_pub = this->create_publisher<armor_interfaces::msg::Target>(
        "/armor_tracker/target", rclcpp::SensorDataQoS());
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    m_tf_buffer->setCreateTimerInterface(timer_interface);

    m_armors_sub.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
    m_tf_filter = std::make_shared<tf2_ros::MessageFilter<armor_interfaces::msg::Armors>>(
        m_armors_sub, *m_tf_buffer, m_odom_frame, 100, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(20));
    m_tf_filter->registerCallback(&ArmorTrackerNode::subArmorsCallback, this);

    // 读取外参 transform
    std::vector<double> translation = config["transform"]["translation"].as<std::vector<double>>();
    std::vector<double> rotation_rpy = config["transform"]["rotation"].as<std::vector<double>>();
    m_yaw_v    = config["kalman"]["yaw_v"].as<double>();
    m_yaw_th_l = config["kalman"]["yaw_th_l"].as<double>();
    m_yaw_th_r = config["kalman"]["yaw_th_l"].as<double>();
    this->declare_parameter("v_yaw", m_yaw_v);

    // 提取 x, y, z
    double x = translation[0];
    double y = translation[1];
    double z = translation[2];

    // 提取 roll, pitch, yaw(面向轴正方向, 顺时针为+)
    double roll  = rotation_rpy[0] * M_PI / 180.0;
    double pitch = rotation_rpy[1] * M_PI / 180.0;
    double yaw   = rotation_rpy[2] * M_PI / 180.0;

    // 转换为四元数
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::TransformStamped static_transform;
    while (rclcpp::ok() && !(this->get_clock()->now().seconds() > 0)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for clock synchronization...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    static_transform.header.stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Current time: %f", this->get_clock()->now().seconds());
    static_transform.header.frame_id = "imu_frame";       
    static_transform.child_frame_id  = "camera_frame";    

    static_transform.transform.translation.x = x;
    static_transform.transform.translation.y = y;
    static_transform.transform.translation.z = z;
    static_transform.transform.rotation.x = q.x();
    static_transform.transform.rotation.y = q.y();
    static_transform.transform.rotation.z = q.z();
    static_transform.transform.rotation.w = q.w();

    m_static_broadcaster.sendTransform(static_transform);
    initExtentedKalman();

    while (rclcpp::ok() && !m_tf_buffer->canTransform(
        "odom_frame", "camera_frame", tf2::TimePointZero)) 
    {
        RCLCPP_INFO(this->get_logger(),
            "Waiting for transform from 'camera_frame' to 'odom_frame' to become available...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void ArmorTrackerNode::declareParameters() {
    m_odom_frame = this->declare_parameter("odom_frame", "odom_frame");

    // 位置、速度、偏航角速度等噪声
    this->declare_parameter("acceleration_variance", 0.08);
    this->declare_parameter("yaw_acceleration_variance", 5.0);
    this->declare_parameter("radius_variance", 80.0);

    // 测量的噪声因子
    this->declare_parameter("location_factor", 0.05);
    this->declare_parameter("yaw_factor", 0.01);

    // 已知半径
    this->declare_parameter("r", 0.31);

    int tt = this->declare_parameter("tracker.tracking_threshold", 1);
    int lt = this->declare_parameter("tracker.lost_threshold", 10);
    double mmd = this->declare_parameter("tracker.max_match_distance", 0.9);
    double mmy = this->declare_parameter("tracker.max_match_yaw", 1.0);

    m_tracker.setMatchDistance(mmd);
    m_tracker.setMatchYaw(mmy);
    m_tracker.getStateMachine()->setTrackingCount(tt);
    m_tracker.getStateMachine()->setlostcount(lt);
}

// 将世界坐标系下的 3D 点投影到 2D 像素坐标
cv::Point2d ArmorTrackerNode::project3dTo2d(const cv::Point3d& pt_3d) {
    std::vector<cv::Point3d> object_points;
    object_points.emplace_back(pt_3d);

    std::vector<cv::Point2d> image_points;

    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
        m_camera_info.intrinsic_matrix[0], m_camera_info.intrinsic_matrix[1], m_camera_info.intrinsic_matrix[2],
        m_camera_info.intrinsic_matrix[3], m_camera_info.intrinsic_matrix[4], m_camera_info.intrinsic_matrix[5],
        m_camera_info.intrinsic_matrix[6], m_camera_info.intrinsic_matrix[7], m_camera_info.intrinsic_matrix[8]);

    cv::Mat dist_coeffs = cv::Mat(m_camera_info.distortion_vector).clone();

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    if (image_points.empty()) {
        return cv::Point2d(-1, -1);
    }
    return image_points[0];
}

void ArmorTrackerNode::subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    if (!armors_msg->armors.empty()) {
        for (auto& armor : armors_msg->armors) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = armors_msg->header;
            ps.pose   = armor.pose;
            rclcpp::Time pose_time = ps.header.stamp;

            // 先将装甲板坐标从 camera_frame 转到 imu_frame
            geometry_msgs::msg::Pose imu_pose;
            try {
                geometry_msgs::msg::TransformStamped static_transform = 
                    m_tf_buffer->lookupTransform("imu_frame", "camera_frame", tf2::TimePointZero);
                tf2::doTransform(ps.pose, imu_pose, static_transform);
            } catch (const tf2::ExtrapolationException& e) {
                RCLCPP_ERROR(this->get_logger(), "armor pose to odom");
                return;
            }
            // 再将 imu_frame 转到 odom_frame
            try {
                geometry_msgs::msg::TransformStamped dynamic_transform = 
                    m_tf_buffer->lookupTransform("odom_frame", "imu_frame", tf2::TimePointZero);
                tf2::doTransform(imu_pose, armor.world_pose, dynamic_transform);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to transform from imu_frame to odom_frame: %s", ex.what());
                return;
            }
        }
    }

    rclcpp::Time timestamp = armors_msg->header.stamp;
    armor_interfaces::msg::Target target_msg;
    target_msg.header.stamp = timestamp; 
    target_msg.header.frame_id = m_odom_frame;
    // 是否是主相机
    target_msg.is_master = (armors_msg->header.frame_id == "camera_optical_frame");

    // 如果当前是 Lost 状态，则初始化
    if (m_tracker.state() == TrackerStateMachine::State::Lost) {
        m_last_stamp = this->now();
        if (armors_msg->armors.empty()) {
            return;
        }
        initExtentedKalman();
        m_tracker.initTracker(armors_msg);
        target_msg.tracking = false;
        return;
    } else {
        m_dt = (timestamp - m_last_stamp).seconds();
        m_tracker.updateTracker(armors_msg);
        target_msg.tracking = m_tracker.isTracking();
        if (m_tracker.isTracking()) {
            const auto& state = m_tracker.getTargetPredictSate();
            // state = [x, vx, y, vy, z, vz, yaw, yaw_v]
            target_msg.position.x = state(0);
            target_msg.velocity.x = state(1);
            target_msg.position.y = state(2);
            target_msg.velocity.y = state(3);
            target_msg.position.z = state(4);
            target_msg.velocity.z = state(5);
            target_msg.yaw   = state(6);
            target_msg.v_yaw = state(7);

            // 半径、上下装甲板间距之类可根据需求发布
            target_msg.r1 = 0.28;
            target_msg.r2 = 0.28;
            target_msg.dz = m_tracker.dz;
            target_msg.num = m_tracker.getNum();
            target_msg.delay = m_dt * 1000;
            target_msg.id = m_tracker.tracked_armor.number;

            m_last_stamp = timestamp;

            // 发布目标
            m_target_pub->publish(target_msg);

            // （以下是投影到图像中，可根据需要保留或删除）
            try {
                geometry_msgs::msg::PointStamped odom_point;
                odom_point.header.frame_id = m_odom_frame;
                odom_point.header.stamp    = armors_msg->header.stamp;
                odom_point.point = target_msg.position;

                geometry_msgs::msg::PointStamped camera_point;
                auto transform = m_tf_buffer->lookupTransform(
                    "camera_frame", "odom_frame", tf2::TimePointZero);
                tf2::doTransform(odom_point, camera_point, transform);

                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(armors_msg->image, sensor_msgs::image_encodings::BGR8);

                cv::Point3d pt_3d(camera_point.point.x, camera_point.point.y, camera_point.point.z);
                cv::Point2d uv = project3dTo2d(pt_3d);

                cv::Mat image = cv_ptr->image.clone();
                if (uv.x >= 0 && uv.x < image.cols && uv.y >= 0 && uv.y < image.rows) {
                    cv::circle(image, uv, 8, cv::Scalar(255, 0, 0), -1);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Projected point is out of image bounds");
                }
                cv::imshow("Predicted Image", image);
                cv::waitKey(1);

            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to transform to camera_frame: %s", ex.what());
                return;
            }
        }
    }
}

// 构建 8 维状态量 [x, vx, y, vy, z, vz, yaw, yaw_v] 的 EKF
void ArmorTrackerNode::initExtentedKalman() {
    // 预测模型
    auto f = [this](const Eigen::VectorXd &x)->Eigen::VectorXd {
        // x = [x_pos, vx, y_pos, vy, z_pos, vz, yaw, yaw_v]
        Eigen::VectorXd x_new(8);
        double x_pos = x(0);
        double vx    = x(1);
        double y_pos = x(2);
        double vy    = x(3);
        double z_pos = x(4);
        double vz    = x(5);
        double yaw   = x(6);
        double yaw_v = x(7);

        // 匀速模型 (中心 + 偏航角匀速)
        x_new(0) = x_pos + vx  * m_dt;    // x
        x_new(1) = vx;                    // vx
        x_new(2) = y_pos + vy  * m_dt;    // y
        x_new(3) = vy;                    // vy
        x_new(4) = z_pos + vz  * m_dt;    // z
        x_new(5) = vz;                    // vz
        x_new(6) = yaw   + yaw_v * m_dt;  // yaw
        x_new(7) = yaw_v;                 // yaw_v (保持不变)

        return x_new;
    };

    // 观测模型
    // 已知装甲板测到的位置是 [x_armor, y_armor, z_armor, yaw_armor]
    // 其中 x_armor = x - r*cos(yaw), y_armor = y + r*sin(yaw), z_armor = z, yaw_armor = yaw
    auto h = [this](const Eigen::VectorXd &x)->Eigen::VectorXd {
        Eigen::VectorXd z(4);
        double r     = this->get_parameter("r").as_double();
        double x_pos = x(0);
        double y_pos = x(2);
        double z_pos = x(4);
        double yaw   = x(6);

        double x_armor = x_pos + r * std::cos(yaw);
        double y_armor = y_pos + r * std::sin(yaw);

        z(0) = x_armor;
        z(1) = y_armor;
        z(2) = z_pos;
        z(3) = yaw;
        return z;
    };

    // F(x) 的雅可比矩阵 8×8
    auto jacob_f = [this](const Eigen::VectorXd &x)->Eigen::MatrixXd {
        // f(·) 中只有加上 vx * dt 之类，故雅可比固定
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(8, 8);

        F(0,1) = m_dt; // x 对 vx 的偏导
        F(2,3) = m_dt; // y 对 vy
        F(4,5) = m_dt; // z 对 vz
        F(6,7) = m_dt; // yaw 对 yaw_v

        return F;
    };

    // H(x) 的雅可比矩阵 4×8
    // z(0)= x + r*cos(yaw), z(1)= y + r*sin(yaw), z(2)= z, z(3)= yaw
    auto jacob_h = [this](const Eigen::VectorXd &x)->Eigen::MatrixXd {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 8);
        double r   = this->get_parameter("r").as_double();
        double yaw = x(6);

        // 对 x_armor = x(0) - r*cos(yaw)
        // ∂x_armor/∂x(0)=1, ∂x_armor/∂yaw= -r*sin(yaw)
        H(0,0) = 1.0;
        H(0,6) = -r * std::sin(yaw);

        // 对 y_armor = x(2) + r*sin(yaw)
        // ∂y_armor/∂x(2)=1, ∂y_armor/∂yaw= r*cos(yaw)
        H(1,2) = 1.0;
        H(1,6) = r * std::cos(yaw);

        // 对 z_armor = x(4)
        H(2,4) = 1.0;

        // 对 yaw_armor = x(6)
        H(3,6) = 1.0;

        return H;
    };

    // Q(8×8) 过程噪声协方差
    auto update_Q = [this]()->Eigen::MatrixXd {
        double dt = m_dt;
        double acceleration_variance      = this->get_parameter("acceleration_variance").as_double();
        double yaw_acceleration_variance  = this->get_parameter("yaw_acceleration_variance").as_double();

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(8, 8);

        // x, vx
        double q_xx = std::pow(dt, 4)/4 * acceleration_variance;
        double q_xv = std::pow(dt, 3)/2 * acceleration_variance;
        double q_vv = std::pow(dt, 2)    * acceleration_variance;

        // x/ y / z 三组类似
        Q(0,0) = q_xx;   Q(0,1) = q_xv;
        Q(1,0) = q_xv;   Q(1,1) = q_vv;

        Q(2,2) = q_xx;   Q(2,3) = q_xv;
        Q(3,2) = q_xv;   Q(3,3) = q_vv;

        Q(4,4) = q_xx;   Q(4,5) = q_xv;
        Q(5,4) = q_xv;   Q(5,5) = q_vv;

        // yaw, yaw_v
        double q_yawyaw = std::pow(dt, 4)/4 * yaw_acceleration_variance;
        double q_yawvel = std::pow(dt, 3)/2 * yaw_acceleration_variance;
        double q_velvel = std::pow(dt, 2)    * yaw_acceleration_variance;

        Q(6,6) = q_yawyaw;   Q(6,7) = q_yawvel;
        Q(7,6) = q_yawvel;   Q(7,7) = q_velvel;

        return Q;
    };

    // R(4×4) 测量噪声协方差
    auto update_R = [this](const Eigen::VectorXd &z)->Eigen::MatrixXd {
        float location_factor = this->get_parameter("location_factor").as_double();
        float yaw_factor      = this->get_parameter("yaw_factor").as_double();

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
        // 位置和yaw对应的噪声
        R(0,0) = location_factor;
        R(1,1) = location_factor;
        R(2,2) = location_factor;
        R(3,3) = yaw_factor;
        return R;
    };

    // 初始协方差阵 P0(8×8)
    Eigen::MatrixXd p0 = Eigen::MatrixXd::Identity(8, 8) * 1.0;

    m_tracker.ekf = std::make_shared<ExtendedKalmanFilter>(
        p0, f, h, jacob_f, jacob_h, update_Q, update_R, m_dt);

    std::cout << "EKF init successfully." << std::endl;
}

} // namespace armor_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorTrackerNode)