#include "kalman_filter.h"

namespace armor_auto_aim {

ExtenedKalmanFilter::ExtenedKalmanFilter(const StateTransformFunction f, const StateTransformFunction h,
                                         const GetMatrixVoidInputFunction jacobian_f, const GetMatrixFunction jacobian_h,
                                         const GetMatrixVoidInputFunction update_Q, const GetMatrixFunction update_R, double dt )
                                         : m_f(f), m_h(h), m_jacobian_f(jacobian_f), m_jacobian_h(jacobian_h), m_update_Q(update_Q), m_update_R(update_R) {}
Eigen::VectorXd ExtenedKalmanFilter::update(const Eigen::VectorXd& contorl_vector) {
    m_H = m_jacobian_h(m_state_pri);
    m_R = m_update_R(contorl_vector);
    m_K = m_P_pri * m_H.transpose() * (m_H * m_P_pri * m_H.transpose() + m_R).inverse();
    m_state_pos = m_state_pri + m_K * (contorl_vector - m_h(m_state_pri));
    Eigen::MatrixXd I;
    m_P_pos = (I - m_K * m_H) * m_P_pri;
    return m_state_pos;
}

Eigen::VectorXd ExtenedKalmanFilter::predict() {   
    m_F = m_jacobian_f(), m_Q = m_update_Q();
    m_state_pri = m_f(m_state_pos);
    m_P_pri = m_F * m_P_pos * m_F.transpose() + m_Q;
    return m_state_pri;
}


} // namespace armor_auto_aim

