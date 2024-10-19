#include <algorithm>
#include <armor_tracker_node.h>
#include <fstream>
#include <ratio>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <yaml-cpp/yaml.h>
namespace armor_auto_aim {

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options) : Node("armor_tracker_node", options), m_static_broadcaster(this) {
    declareParameters();
    m_target_pub = this->create_publisher<armor_interfaces::msg::Target>("armor_tracker/target", rclcpp::SensorDataQoS());
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
    
    YAML::Node config = YAML::LoadFile("/home/myq/Desktop/my_code/auto_aim/2025_rm_auto_aim/config/demo.yaml");
    std::vector<double> translation = config["transform"]["translation"].as<std::vector<double> >();
    std::vector<double> rotation_rpy = config["transform"]["rotation"].as<std::vector<double> >();
    // 提取 x, y, z
    double x = translation[0];
    double y = translation[1];
    double z = translation[2];

    // 提取 roll, pitch, yaw
    double roll = rotation_rpy[0];
    double pitch = rotation_rpy[1];
    double yaw = rotation_rpy[2];

    // 将 RPY 转换为四元数
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    geometry_msgs::msg::TransformStamped static_transform;
    while (rclcpp::ok() && !this->get_clock()->now().seconds() > 0) {
    RCLCPP_INFO(this->get_logger(), "Waiting for clock synchronization...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    static_transform.header.stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Current time: %f", this->get_clock()->now().seconds());
    static_transform.header.frame_id = "imu_frame";       // imu坐标系
    static_transform.child_frame_id = "camera_frame";      // 相机坐标系

    static_transform.transform.translation.x = x;
    static_transform.transform.translation.y = y;
    static_transform.transform.translation.z = z;

    static_transform.transform.rotation.x = q.x();
    static_transform.transform.rotation.y = q.y();
    static_transform.transform.rotation.z = q.z();
    static_transform.transform.rotation.w = q.w();
    m_static_broadcaster.sendTransform(static_transform);
    initExtentedKalman();
    while (rclcpp::ok() && !m_tf_buffer->canTransform("odom_frame", "camera_frame", tf2::TimePointZero)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform from 'camera_frame' to 'odom_frame' to become available...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}


void ArmorTrackerNode::declareParameters() {
    m_odom_frame = this->declare_parameter("odom_frame", "odom_frame");
    // 位置  速度  偏航角速度  r  的方差
    std::cout << "declareParameters" << std::endl;
    this->declare_parameter("acceleration_variance", 0.08);
    this->declare_parameter("yaw_acceleration_variance", 5.0);
    this->declare_parameter("radius_variance", 80.0);

    // 测量的位置和偏航角的噪声因子
    this->declare_parameter("location_factor", 0.05);
    this->declare_parameter("yaw_factor", 0.01);
    int tt = this->declare_parameter("tracker.tracking_threshold", 5);
    int lt = this->declare_parameter("tracker.lost_threshold", 30);
    double mmd = this->declare_parameter("tracker.max_match_distance", 0.5);
    double mmy = this->declare_parameter("tracker.max_match_yaw", 1.0);
    m_tracker.setMatchDistance(mmd);
    m_tracker.setMatchYaw(mmy);
    m_tracker.getStateMachine()->setTrackingCount(tt);
    m_tracker.getStateMachine()->setlostcount(lt);
}

void ArmorTrackerNode::subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    if (armors_msg->armors.empty()) {
        return;
    }
    for (auto& armor: armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose = armor.pose;
        // 获取目标姿态的时间戳
        rclcpp::Time pose_time = ps.header.stamp;
        RCLCPP_INFO(this->get_logger(), "Pose time in camera frame: %f", pose_time.seconds());
        geometry_msgs::msg::Pose imu_pose;
        std::cout << "camera_frame pose : "
                << "x: " << ps.pose.position.x << ", "
                << "y: " << ps.pose.position.y << ", "
                << "z: " << ps.pose.position.z << std::endl;
        try {
        // 获取从 camera_frame 到 imu_frame 的静态变换
        geometry_msgs::msg::TransformStamped static_transform = 
            m_tf_buffer->lookupTransform("imu_frame", "camera_frame", tf2::TimePointZero);
            rclcpp::Time static_transform_time(static_transform.header.stamp);
            RCLCPP_INFO(this->get_logger(), "Static transform time: %f", static_transform_time.seconds());

        // 将位姿从 camera_frame 转换到 imu_frame
        tf2::doTransform(ps.pose, imu_pose, static_transform);
            std::cout << "imu pose : "
                        << "x: " << imu_pose.position.x << ", "
                        << "y: " << imu_pose.position.y << ", "
                        << "z: " << imu_pose.position.z << std::endl;
            // 得到最新变化
            // geometry_msgs::msg::TransformStamped transform_stamped = 
            // m_tf_buffer->lookupTransform(m_odom_frame, ps.header.frame_id, tf2::TimePointZero);
            // 从相机坐标系转换到惯性系
            // tf2::doTransform(ps.pose, armor.world_pose, transform_stamped);
            // armor.world_pose = m_tf_buffer->transform(ps, m_odom_frame, tf2::Duration(0)).pose;
            // 在转换前后打印位姿
        } catch (const tf2::ExtrapolationException& e) {
            RCLCPP_ERROR(this->get_logger(), "armor pose to odom");
            return;
        }
        try {
            geometry_msgs::msg::TransformStamped dynamic_transform = 
                m_tf_buffer->lookupTransform("odom_frame", "imu_frame", tf2::TimePointZero);
            rclcpp::Time dynamic_transform_time(dynamic_transform.header.stamp);
            RCLCPP_INFO(this->get_logger(), "Dynamic transform time: %f", dynamic_transform_time.seconds());


            tf2::doTransform(imu_pose, armor.world_pose, dynamic_transform);
            std::cout << "odom pose : "
                        << "x: " << armor.world_pose.position.x << ", "
                        << "y: " << armor.world_pose.position.y << ", "
                        << "z: " << armor.world_pose.position.z << std::endl;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform from imu_frame to odom_frame: %s", ex.what());
            return;
        }

    }
    // rclcpp::Time timestamp = armors_msg->header.stamp;
    // armor_interfaces::msg::Target target_msg;
    // target_msg.header.stamp = timestamp;
    // target_msg.header.frame_id = m_odom_frame;
    // target_msg.is_master = armors_msg->header.frame_id == "camera_optical_frame";
    // if (m_tracker.state() == TrackerStateMachine::State::Lost) {
    //     std::cout << "init tracker" << std::endl;
    //     m_last_stamp = this->now();
    //     if (armors_msg->armors.empty()) {
    //         std::cout << "armor_msg is empty" << std::endl;
    //         return;
    //     }
    //     m_tracker.initTracker(armors_msg);
    //     target_msg.tracking = false;
    // } else {
    //     std::cout << "start tracking" << std::endl;
    //     m_dt = (timestamp - m_last_stamp).seconds();
    //     m_tracker.updateTracker(armors_msg);
    //     target_msg.tracking = m_tracker.isTracking();
    //     if (m_tracker.isTracking()) {
    //         const auto& state = m_tracker.getTargetPredictSate();
    //         std::cout << "target message" << " position x :" << state(0)
    //         << " position y :" << state(2) << " position z :" << state(4) 
    //         << " yaw :" << state(6) << std::endl;
    //         target_msg.position.x = state(0);
    //         target_msg.velocity.x = state(1);
    //         target_msg.position.y = state(2);
    //         target_msg.velocity.y = state(3);
    //         target_msg.position.z = state(4);
    //         target_msg.velocity.z = state(5);
    //         target_msg.yaw = state(6);
    //         target_msg.v_yaw = state(7);
    //         target_msg.r1 = state(8);
    //         target_msg.r2 = m_tracker.another_r;
    //         target_msg.dz = m_tracker.dz;
    //         target_msg.num = m_tracker.getNum();
    //         target_msg.delay = m_dt * 1000;
    //         target_msg.id = m_tracker.tracked_armor.number;
    //         // message yaw
    //         auto q = m_tracker.tracked_armor.world_pose.orientation;
    //         tf2::Quaternion tf_q;
    //         tf2::fromMsg(q, tf_q);
    //         double r, p, y;
    //         tf2::Matrix3x3(tf_q).getRPY(r, p, y);
    //         std_msgs::msg::Float64 yaw_msg;
    //         yaw_msg.data = y;
    //     }
    // }

    // m_last_stamp = timestamp;
    // m_target_pub->publish(target_msg);
}

void ArmorTrackerNode::initExtentedKalman() {
        auto f = [this](const Eigen::VectorXd &x)->Eigen::VectorXd {
        Eigen::VectorXd x_new(9);
        double x_pos = x(0);
        double vx = x(1);
        double y_pos = x(2);
        double vy = x(3);
        double z_pos = x(4);
        double vz = x(5);
        double yaw = x(6);
        double v_yaw = x(7);
        double r = x(8);
        
        // 更新位置
        x_new(0) = x_pos + vx * m_dt;
        x_new(2) = y_pos + vy * m_dt;
        x_new(4) = z_pos + vz * m_dt;
        
        // 速度由非线性关系确定
        x_new(1) = -r * v_yaw * sin(yaw);
        x_new(3) = r * v_yaw * cos(yaw);
        x_new(5) = vz; 
        
        // 更新偏航角和偏航角速度
        x_new(6) = yaw + v_yaw * m_dt;
        x_new(7) = v_yaw;  
        x_new(8) = r; 
        
        return x_new;
    };
    auto h = [this](const Eigen::VectorXd &x)->Eigen::VectorXd {
        Eigen::VectorXd z(4);
        double x_pos = x(0);
        double y_pos = x(2);
        double z_pos = x(4);
        double yaw = x(6);
        double r = x(8);
        z(0) = x_pos + r * cos(yaw);
        z(1) = y_pos + r * sin(yaw);
        z(2) = z_pos;
        z(3) = yaw;
        return z;
    };
    auto jacob_f = [this](const Eigen::VectorXd& x)->Eigen::MatrixXd {
        double yaw = x(6);
        double v_yaw = x(7);
        double r = x(8);

        double sin_yaw = sin(yaw);
        double cos_yaw = cos(yaw);

        Eigen::MatrixXd F(9, 9);

        F << 
        //  x      vx      y      vy      z      vz     yaw    v_yaw     r
        1,    m_dt,     0,      0,     0,      0,      0,      0,      0,    // x_pos 更新
        0,      1,      0,      0,     0,      0, -r * v_yaw * cos_yaw, -r * sin_yaw, -v_yaw * sin_yaw, // vx 更新
        0,      0,      1,    m_dt,    0,      0, -r * v_yaw * sin_yaw,  r * cos_yaw,  v_yaw * cos_yaw, // y_pos 更新
        0,      0,      0,      1,     0,      0,      0,      0,      0,    // vy 更新
        0,      0,      0,      0,     1,    m_dt,     0,      0,      0,    // z_pos 更新
        0,      0,      0,      0,     0,      1,      0,      0,      0,    // vz 更新
        0,      0,      0,      0,     0,      0,      1,    m_dt,     0,    // yaw 更新
        0,      0,      0,      0,     0,      0,      0,      1,      0,    // v_yaw 更新
        0,      0,      0,      0,     0,      0,      0,      0,      1;    // r 更新
        return F;
    };
    auto jacob_h = [this](const Eigen::VectorXd &x)->Eigen::MatrixXd {
        Eigen::MatrixXd h(4, 9);
        double yaw = x[6], r = x[8];
        //   x  vx  y  vy   z yz     yaw       v_yaw     r
        h << 1, 0,  0,  0,  0, 0,  -r*sin(yaw),   0,    cos(yaw), // x
             0, 0,  1,  0,  0, 0,   r*cos(yaw),   0,    sin(yaw), // y
             0, 0,  0,  0,  1, 0,     0      ,   0,      0,      // z
             0, 0,  0,  0,  0, 0,     1      ,   0,      0;      // yaw
        return h;
    };
    auto update_Q = [this]()->Eigen::MatrixXd {
        double acceleration_variance = this->get_parameter("acceleration_variance").as_double();
        double yaw_acceleration_variance =  this->get_parameter("yaw_acceleration_variance").as_double();
        double radius_variance = this->get_parameter("radius_variance").as_double();

        double q_xx = pow(m_dt, 4) / 4 * acceleration_variance;
        double q_xv = pow(m_dt, 3) / 2 * acceleration_variance;
        double q_vv = pow(m_dt, 2) * acceleration_variance;

        double q_yawyaw = pow(m_dt, 4) / 4 * yaw_acceleration_variance;
        double q_yawv = pow(m_dt, 3) / 2 * yaw_acceleration_variance;
        double q_vyawvyaw = pow(m_dt, 2) * yaw_acceleration_variance;

        Eigen::MatrixXd Q(9, 9);

        Q << 
        //  x      vx      y      vy      z      vz     yaw    v_yaw     r
        q_xx,  q_xv,     0,      0,     0,      0,     0,      0,      0,    // x
        q_xv,  q_vv,     0,      0,     0,      0,     0,      0,      0,    // vx
            0,     0,    q_xx,   q_xv,    0,      0,     0,      0,      0,    // y
            0,     0,    q_xv,   q_vv,    0,      0,     0,      0,      0,    // vy
            0,     0,      0,     0,    q_xx,   q_xv,    0,      0,      0,    // z
            0,     0,      0,     0,    q_xv,   q_vv,    0,      0,      0,    // vz
            0,     0,      0,     0,     0,      0,   q_yawyaw, q_yawv,   0,    // yaw
            0,     0,      0,     0,     0,      0,    q_yawv, q_vyawvyaw,0,    // v_yaw
            0,     0,      0,     0,     0,      0,      0,      0,  pow(m_dt, 2) * radius_variance; // r

        return Q;

    };
    auto update_R = [this](const Eigen::VectorXd &z)->Eigen::MatrixXd {
        float location_factor = this->get_parameter("location_factor").as_double();
        float yaw_factor      = this->get_parameter("yaw_factor").as_double();
        Eigen::MatrixXd R(4, 4);
        R << location_factor,  0,  0,  0,
             0,  location_factor,  0,  0,
             0,  0,  location_factor,  0,
             0,  0,  0,  yaw_factor;          
        return R;
    };
    int p = 0.1;
    // Eigen::Matrix<double, 9, 9> p0;
    Eigen::MatrixXd p0 = Eigen::MatrixXd::Identity(9, 9) * 0.1;
    //    x   vx   y   vy  z   vz  yaw v_yaw  r
    // p0 << p,  0,   0,  0,  0,   0,   0,  0,   0, // x
    //       0,  p,   0,  0,  0,   0,   0,  0,   0, // vx
    //       0,  0,   p,  0,  0,   0,   0,  0,   0, // y
    //       0,  0,   0,  p,  0,   0,   0,  0,   0, // vy
    //       0,  0,   0,  0,  p,   0,   0,  0,   0, // z
    //       0,  0,   0,  0,  0,   p,   0,  0,   0, // vz
    //       0,  0,   0,  0,  0,   0,   p,  0,   0, // yaw
    //       0,  0,   0,  0,  0,   0,   0,  p,   0, // v_yaw
    //       0,  0,   0,  0,  0,   0,   0,  0,   p; // r

    m_tracker.ekf = std::make_shared<ExtendedKalmanFilter>(p0, f, h, jacob_f, jacob_h, update_Q, update_R, m_dt);
    std::cout << "ekf init successfully " << std::endl;
}

} // namespace armor_auto_aim
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorTrackerNode)