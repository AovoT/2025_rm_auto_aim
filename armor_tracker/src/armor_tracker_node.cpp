#include <armor_tracker_node.h>

namespace armor_auto_aim {

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options) : Node("armor_tracker_node", options) {
    declareParameters();
    m_armors_sub = this->create_subscription<armor_interfaces::msg::Armors>("armor_detect", 20, std::bind(&ArmorTrackerNode::subarmorCallback, this, std::placeholders::_1));
}

void ArmorTrackerNode::declareParameters() {
    // 位置  速度  偏航角速度  r  的方差
    this->declare_parameter("location_variance", 0.08);
    this->declare_parameter("speed_variance", 5.0);
    this->declare_parameter("yaw_variance", 80.0);
    this->declare_parameter("r_variance", 80.0);
    // 测量的位置和偏航角的噪声因子
    this->declare_parameter("location_factor", 4e-4);
    this->declare_parameter("yaw_factor", 5e-5);
    
}

void ArmorTrackerNode::subarmorCallback(const armor_interfaces::msg::Armors &armors) {
    
    

}

void ArmorTrackerNode::initExtentedKalman() {
    auto f = [this](const Eigen::VectorXd &x)->Eigen::VectorXd {
        Eigen::VectorXd next_x = x;
        next_x(0) = x(1) * m_dt;
        next_x(2) = x(3) * m_dt;
        next_x(4) = x(5) * m_dt;
        next_x(6) = x(7) * m_dt;
        return next_x;
    };

    auto h = [this](const Eigen::VectorXd &x)->Eigen::VectorXd {
        Eigen::VectorXd z(4);
        double xc = x[0], yc = x[2], yaw = x[6], r = x[8];
        z[0] = xc - r*cos(yaw);
        z[1] = yc - r*sin(yaw);
        z[2] = x[4];
        z[3] = x[6];
        return z;
    };

    auto jacob_f = [this]()->Eigen::MatrixXd {
        Eigen::Matrix<double, 9, 9> F;
        //   x  vx  y  vy   z   yz  yaw v_yaw  r
        F << 1, m_dt, 0,  0,  0,  0,   0,  0,    0, // x
             0, 1,  0,  0,  0,  0,   0,  0,    0, // vx
             0, 0,  1,  m_dt, 0,  0,   0,  0,    0, // y
             0, 0,  0,  1,  0,  0,   0,  0,    0, // vy
             0, 0,  0,  0,  1,  m_dt,  0,  0,    0, // z
             0, 0,  0,  0,  0,  1,   0,  0,    0, // vz
             0, 0,  0,  0,  0,  0,   1,  m_dt,   0, // yaw
             0, 0,  0,  0,  0,  0,   0,  1,    0, // v_yaw
             0, 0,  0,  0,  0,  0,   0,  0,    1; // r
        return F;
    };

    auto jacob_h = [this](const Eigen::VectorXd &x)->Eigen::MatrixXd {
        Eigen::MatrixXd h(4, 9);
        double yaw = x[6], r = x[8];
        //   x  vx  y  vy   z yz     yaw       v_yaw     r
        h << 1, 0,  0,  0,  0, 0,  r*sin(yaw),   0,   -cos(yaw), // x
             0, 0,  1,  0,  0, 0, -r*cos(yaw),   0,   -sin(yaw), // y
             0, 0,  0,  0,  1, 0,     0      ,   0,      0,      // z
             0, 0,  0,  0,  0, 0,     1      ,   0,      0;      // yaw
        return h;
    };

    auto update_Q = [this]()->Eigen::MatrixXd {
        float location_variance = this->get_parameter("location_variance").as_double();
        float speed_variance    = this->get_parameter("speed_variance").as_double();
        float yawv_variance     = this->get_parameter("yawv_variance").as_double();
        float yaw_variance      =  this->get_parameter("yaw_variance").as_double();
        float r_variance        = this->get_parameter("r_variance_variance").as_double();
        
        double q_x_x = pow(m_dt, 4) / 4 * location_variance,// 表示位置与位置的噪声方差
             q_x_v = pow(m_dt, 3) / 2 * speed_variance, // 表示位置与速度的噪声方差
             q_vx_vx = pow(m_dt, 2) * speed_variance, // 表示各方向速度的噪声方差
             q_vy_vy = pow(m_dt, 2) * yawv_variance, // 表示偏航角速度的噪声方差
             q_yaw_yaw = pow(m_dt, 4) / 4 * yaw_variance,  // 表示偏航角的噪声方差
             q_yaw_vy = pow(m_dt, 3) / 2 * yawv_variance, // 表示偏航角和偏航角速度的噪声方差
             q_r_r = pow(m_dt, 4) / 4 * r_variance; // 表示旋转半径的过程噪声方差。
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        Eigen::MatrixXd q;
        q << q_x_x,  q_x_v, 0,      0,      0,      0,      0,      0,      0,
             q_x_v, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
             0,      0,      q_x_x,  q_x_v, 0,      0,      0,      0,      0,
             0,      0,      q_x_v, q_vx_vx,0,      0,      0,      0,      0,
             0,      0,      0,      0,      q_x_x,  q_x_v, 0,      0,      0,
             0,      0,      0,      0,      q_x_v, q_vx_vx,0,      0,      0,
             0,      0,      0,      0,      0,      0,      q_yaw_yaw,  q_yaw_vy, 0,
             0,      0,      0,      0,      0,      0,      q_yaw_vy, q_vy_vy,0,
             0,      0,      0,      0,      0,      0,      0,      0,      q_r_r;
        return q;
    };

    auto update_R = [this](const Eigen::VectorXd &z)->Eigen::MatrixXd {
        float location_factor = this->get_parameter("location_factor").as_double();
        float yaw_factor      = this->get_parameter("yaw_factor").as_double();

        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        Eigen::MatrixXd R;
        R << z(0) * location_factor,  0,  0,  0,
             0,  z(1) * location_factor,  0,  0,
             0,  0,  z(2) * location_factor,  0,
             0,  0,  0,  z(3) * yaw_factor;          
        return R;
    };
    int p = 1;
    Eigen::Matrix<double, 9, 9> p0;
    //  xa  vxa  ya  vya  za  vza  yaw v_yaw  r
    p0 << p,  0,   0,  0,  0,   0,   0,  0,   0, // xa
          0,  p,   0,  0,  0,   0,   0,  0,   0, // vxa
          0,  0,   p,  0,  0,   0,   0,  0,   0, // ya
          0,  0,   0,  p,  0,   0,   0,  0,   0, // vya
          0,  0,   0,  0,  p,   0,   0,  0,   0, // za
          0,  0,   0,  0,  0,   p,   0,  0,   0, // vza
          0,  0,   0,  0,  0,   0,   p,  0,   0, // yaw
          0,  0,   0,  0,  0,   0,   0,  p,   0, // v_yaw
          0,  0,   0,  0,  0,   0,   0,  0,   p; // r

    m_ekf = std::make_shared<ExtenedKalmanFilter>(f, h, jacob_f, jacob_h, update_Q, update_R, m_dt);

}

void ArmorTrackerNode::initTracker() {

}

} // namespace armor_auto_aim