#include "kalman_filter.h"
#include <Eigen/Cholesky>  // 包含 LLT 分解相关的头文件


namespace armor_auto_aim {

ExtendedKalmanFilter::ExtendedKalmanFilter(Eigen::MatrixXd& P0, const StateTransformFunction f, const StateTransformFunction h,
                                         const GetMatrixFunction jacobian_f, const GetMatrixFunction jacobian_h,
                                         const GetMatrixVoidInputFunction update_Q, const GetMatrixFunction update_R, double dt )
                                         : m_P_pos(P0), m_f(f), m_h(h), m_jacobian_f(jacobian_f), m_jacobian_h(jacobian_h), m_update_Q(update_Q), m_update_R(update_R) {}
Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd& measurement_vector) {
    m_H = m_jacobian_h(m_state_pri);
    m_R = m_update_R(measurement_vector);
    m_K = m_P_pri * m_H.transpose() * (m_H * m_P_pri * m_H.transpose() + m_R).inverse(); // Kalman gain
    m_state_pos = m_state_pri + m_K * (measurement_vector - m_H * m_state_pri);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_P_pri.rows(), m_P_pri.cols());
    m_P_pos = (I - m_K * m_H) * m_P_pri;
    return m_state_pos;
}

Eigen::VectorXd ExtendedKalmanFilter::predict() {
    m_F = m_jacobian_f(m_state_pos);
    m_Q = m_update_Q();
    m_state_pri = m_f(m_state_pos);
    m_P_pri = m_F * m_P_pos * m_F.transpose() + m_Q;
    return m_state_pri;
}
}; // namespace armor_auto_aim