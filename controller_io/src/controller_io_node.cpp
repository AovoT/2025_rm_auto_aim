#include <armor_interfaces/msg/detail/target__struct.hpp>
#include <controller_io_node.h>


namespace armor_auto_aim {
using Receive = custom_serial_interfaces::msg::Receive;
using SendPackage = custom_serial_interfaces::srv::SendPackage;
using AimTarget = armor_interfaces::msg::Target;
ControllerIONode::ControllerIONode(const rclcpp::NodeOptions &options)
    : Node("controller_io", options) {
  // Subsciption
    m_serial_sub = this->create_subscription<Receive>(
      "/custom_serial/receive", rclcpp::SensorDataQoS(),
      std::bind(&ControllerIONode::serialHandle, this, std::placeholders::_1));
    m_tracker_sub = this->create_subscription<AimTarget>(
        "/armor_tracker/target", rclcpp::SensorDataQoS(),
        std::bind(&ControllerIONode::trackerCallback, this, std::placeholders::_1)
    );
    m_gimbal_tf_broad = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // Client
    m_serial_cli = this->create_client<SendPackage>("/custom_serial/send");
    m_send_thread = std::thread([this]() -> void {
    using namespace std::chrono_literals;
    std::string msg = "Controller test";
    int len = msg.size() + 1;
    while (true) {
      std::this_thread::sleep_for(500ms);
      auto request = std::make_shared<SendPackage::Request>();
      request->id = PCId;
      request->func_code = 1;
      request->len = len;
      request->data.resize(request->len); // NOTE: must to be resize
      std::memcpy(request->data.data(), msg.c_str(), len);

      while (!m_serial_cli->wait_for_service(500ms))
        RCLCPP_WARN(this->get_logger(), "wait service timeout!");
      if (rclcpp::ok()) {
        m_serial_cli->async_send_request(request);
      }
    }
  });
}

void ControllerIONode::serialHandle(const Receive::SharedPtr serial_msg) {  
    RCLCPP_DEBUG(this->get_logger(), "fun: %d; id: %d; len: %d; data: %s;",
        serial_msg->func_code, serial_msg->id, serial_msg->len, serial_msg->data.data());
    if (serial_msg->id == ControllerId || serial_msg->id == SlaveControllerId) {
        switch (serial_msg->func_code) {
            case NeedUpdateTimestamp: {
                std::cout << "start update timestamp" << std::endl;
                uint64_t timestamp = static_cast<uint64_t>(this->now().nanoseconds());
                auto request = std::make_shared<SendPackage::Request>();
                request->func_code = TimestampPackte;
                request->id = serial_msg->id - 10;
                request->len = sizeof(uint64_t);
                request->data.resize(request->len);
                std::memcpy(request->data.data(), &timestamp, sizeof(timestamp));

                using namespace std::chrono_literals;
                while (!m_serial_cli->wait_for_service(500ms)) {
                    if (!rclcpp::ok())
                        RCLCPP_ERROR(this->get_logger(), "Failed wait service;");
                    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
                }
                m_serial_cli->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Sync timerstmap (controller id: %d)", serial_msg->id);
                break;
            }
            case GimbalPose: {
                std::cout << "start_receive GimbalPosePacket" << std::endl;
                GimbalPosePacket gimbal_pose_pkt;
                std::memcpy(&gimbal_pose_pkt, serial_msg->data.data(), sizeof(GimbalPosePacket));
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = serial_msg->id == ControllerId? "odom_frame": "slave_odom_frame"; // 设置父坐标系
                t.child_frame_id = serial_msg->id == ControllerId? "imu_frame": "slave_imu_frame"; // 设置子坐标系
                // 设置平移
                t.transform.translation.x = 0;
                t.transform.translation.y = 0;
                t.transform.translation.z = 0;
                tf2::Quaternion q(gimbal_pose_pkt.x, gimbal_pose_pkt.y, gimbal_pose_pkt.z, gimbal_pose_pkt.w); // 
                std::cout <<  "q :" << q << std::endl;
                std::cout << " x :" <<  gimbal_pose_pkt.x <<
                 " y : " <<  gimbal_pose_pkt.y <<
                  " z : " <<  gimbal_pose_pkt.z <<
                   " w :" <<  gimbal_pose_pkt.w << std::endl;
                t.transform.rotation = tf2::toMsg(q); //转换为ros消息格式
                m_gimbal_tf_broad->sendTransform(t);
                break;
            }
            case SetTargetColor: {
            }
            default: {
                RCLCPP_WARN(this->get_logger(), "Unknow func_code: %d", serial_msg->func_code);
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknow id: %d", serial_msg->id);
    }
}
void ControllerIONode::trackerCallback(const armor_interfaces::msg::Target::SharedPtr target_msg) {
    using namespace std::chrono_literals;

    // 更新状态信息
    m_trakcer_state[static_cast<int>(target_msg->is_master)] = target_msg->tracking;

    // 打印 Target 消息中的信息
    std::cout << "====================== Target Message ======================" << std::endl;
    std::cout << "Header:" << std::endl;
    std::cout << "  stamp: " << target_msg->header.stamp.sec << "." << target_msg->header.stamp.nanosec << std::endl;
    std::cout << "  frame_id: " << target_msg->header.frame_id << std::endl;
    std::cout << "Tracking: " << target_msg->tracking << std::endl;
    std::cout << "ID      : " << static_cast<int>(target_msg->id) << std::endl;
    std::cout << "Position: (" << target_msg->position.x << ", " << target_msg->position.y << ", " << target_msg->position.z << ")" << std::endl;
    std::cout << "Velocity: (" << target_msg->velocity.x << ", " << target_msg->velocity.y << ", " << target_msg->velocity.z << ")" << std::endl;
    std::cout << "Yaw     : " << target_msg->yaw << std::endl;
    std::cout << "Yaw_v   : " << target_msg->v_yaw << std::endl;
    std::cout << "r1      : " << target_msg->r1 << std::endl;
    std::cout << "r2      : " << target_msg->r2 << std::endl;
    std::cout << "dz      : " << target_msg->dz << std::endl;
    std::cout << "Delay   : " << static_cast<int>(target_msg->delay) << std::endl;
    std::cout << "Num     : " << static_cast<int>(target_msg->num) << std::endl;
    std::cout << "Is Master: " << target_msg->is_master << std::endl;

    // 填充 AutoAimPacket 数据
    AutoAimPacket packet;
    packet.x = target_msg->position.x;
    packet.y = target_msg->position.y;
    packet.z = target_msg->position.z;
    packet.v_x = target_msg->velocity.x;
    packet.v_y = target_msg->velocity.y;
    packet.v_z = target_msg->velocity.z;
    packet.theta = target_msg->yaw;
    packet.omega = target_msg->v_yaw;
    packet.r1 = target_msg->r1;
    packet.r2 = target_msg->r2;
    packet.dz = target_msg->dz;
    packet.delay = target_msg->delay;
    packet.is_tracking = target_msg->tracking;
    packet.num = target_msg->num;
    packet.id = target_msg->id;

    // 创建请求
    auto request = std::make_shared<SendPackage::Request>();
    request->func_code = AimPacket;
    request->id = !static_cast<uint8_t>(target_msg->is_master);
    // request->id = 0;
    request->len = sizeof(AutoAimPacket);
    request->data.resize(request->len);
    std::memcpy(request->data.data(), &packet, request->len);

    // 打印生成的 AutoAimPacket 信息
    std::cout << "====================== AutoAimPacket ======================" << std::endl;
    std::cout << "x          : " << packet.x << std::endl;
    std::cout << "y          : " << packet.y << std::endl;
    std::cout << "z          : " << packet.z << std::endl;
    std::cout << "v_x        : " << packet.v_x << std::endl;
    std::cout << "v_y        : " << packet.v_y << std::endl;
    std::cout << "v_z        : " << packet.v_z << std::endl;
    std::cout << "theta      : " << packet.theta << std::endl;
    std::cout << "omega      : " << packet.omega << std::endl;
    std::cout << "r1         : " << packet.r1 << std::endl;
    std::cout << "r2         : " << packet.r2 << std::endl;
    std::cout << "dz         : " << packet.dz << std::endl;
    std::cout << "delay      : " << static_cast<int>(packet.delay) << std::endl;
    std::cout << "is_tracking: " << static_cast<int>(packet.is_tracking) << std::endl;
    std::cout << "num        : " << static_cast<int>(packet.num) << std::endl;
    std::cout << "id         : " << static_cast<int>(packet.id) << std::endl;

    // 模拟反序列化打印数据
    AutoAimPacket parsed_packet;
    std::memcpy(&parsed_packet, request->data.data(), sizeof(AutoAimPacket));
    std::cout << "====================== Parsed Packet ======================" << std::endl;
    std::cout << "x          : " << parsed_packet.x << std::endl;
    std::cout << "y          : " << parsed_packet.y << std::endl;
    std::cout << "z          : " << parsed_packet.z << std::endl;
    std::cout << "v_x        : " << parsed_packet.v_x << std::endl;
    std::cout << "v_y        : " << parsed_packet.v_y << std::endl;
    std::cout << "v_z        : " << parsed_packet.v_z << std::endl;
    std::cout << "theta      : " << parsed_packet.theta << std::endl;
    std::cout << "omega      : " << parsed_packet.omega << std::endl;
    std::cout << "r1         : " << parsed_packet.r1 << std::endl;
    std::cout << "r2         : " << parsed_packet.r2 << std::endl;
    std::cout << "dz         : " << parsed_packet.dz << std::endl;
    std::cout << "delay      : " << static_cast<int>(parsed_packet.delay) << std::endl;
    std::cout << "is_tracking: " << static_cast<int>(parsed_packet.is_tracking) << std::endl;
    std::cout << "num        : " << static_cast<int>(parsed_packet.num) << std::endl;
    std::cout << "id         : " << static_cast<int>(parsed_packet.id) << std::endl;

    // 检查服务是否存在并发送请求
    while (!m_serial_cli->wait_for_service(500ms)) {
        RCLCPP_WARN(this->get_logger(), "wait service timeout!");
        return;
    }

    if (rclcpp::ok()) {
        m_serial_cli->async_send_request(request);
    } else {
        RCLCPP_ERROR(this->get_logger(), "rclcpp is not ok!");
    }
}


// void ControllerIONode::trackerCallback(const armor_interfaces::msg::Target::SharedPtr target_msg) {
//     using namespace std::chrono_literals;

//     m_trakcer_state[static_cast<int>(target_msg->is_master)] = target_msg->tracking;

//     std::cout << "======================target msg======================" << std::endl;
//     std::cout << " x     : " << target_msg->position.x;
//     std::cout << " x_v   : " << target_msg->velocity.x;
//     std::cout << " y     : " << target_msg->position.y;
//     std::cout << " y_v   : " << target_msg->velocity.y;
//     std::cout << " z     : " << target_msg->position.z;
//     std::cout << " z_v   : " << target_msg->velocity.z;
//     std::cout << " yaw   : " << target_msg->yaw;
//     std::cout << " yaw_v : " << target_msg->v_yaw;
//     std::cout << " r     : " << target_msg->r1 << std::endl;

//     AutoAimPacket packet;
//     packet.x = target_msg->position.x;
//     packet.y = target_msg->position.y;
//     packet.z = target_msg->position.z;
//     packet.v_x = target_msg->velocity.x;
//     packet.v_y = target_msg->velocity.y;
//     packet.v_z = target_msg->velocity.z;
//     packet.theta = target_msg->yaw;
//     packet.omega = target_msg->v_yaw;
//     packet.r1 = target_msg->r1;
//     packet.r2 = target_msg->r2;
//     packet.dz = target_msg->dz;
//     packet.delay = target_msg->delay;
//     packet.is_tracking = target_msg->tracking;
//     packet.num = target_msg->num;
//     packet.id = target_msg->id;

//     auto request = std::make_shared<SendPackage::Request>();
//     request->func_code = AimPacket;
//     request->id = !static_cast<uint8_t>(target_msg->is_master);
//     request->len = sizeof(AutoAimPacket);
//     request->data.resize(request->len);
//     std::memcpy(request->data.data(), &packet, request->len);

//     while (!m_serial_cli->wait_for_service(500ms)) {
//         RCLCPP_WARN(this->get_logger(), "wait service timeout!");
//         return;
//     }
//     if (rclcpp::ok())
//         m_serial_cli->async_send_request(request);
//     else
//         RCLCPP_ERROR(this->get_logger(), "rclcpp is not ok!");
// }
} // namespace armor_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ControllerIONode)
