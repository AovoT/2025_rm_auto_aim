#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <eigen3/Eigen/Core>

namespace armor_auto_aim {
class ExtenedKalmanFilter {
using StateTransformFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>; // 输入一个向量返回一个向量
using GetMatrixFunction = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>; // 输入一个向量返回一个矩阵
using GetMatrixVoidInputFunction = std::function<Eigen::MatrixXd()>; // 不用输入向量返回一个矩阵

public:
    ExtenedKalmanFilter(const StateTransformFunction f, const StateTransformFunction h,
                        const GetMatrixFunction jacobian_f, const GetMatrixFunction jacobian_h,
                        const GetMatrixVoidInputFunction update_Q, const GetMatrixFunction update_R, double dt);
    void updateDt(double dt) {m_dt = dt;}

    Eigen::VectorXd update(const Eigen::VectorXd &measurement_vector);

    Eigen::VectorXd predict();
private:
    StateTransformFunction m_f;
    StateTransformFunction m_h;
    GetMatrixFunction m_jacobian_f;
    GetMatrixFunction m_jacobian_h;
    GetMatrixVoidInputFunction m_update_Q;
    GetMatrixFunction m_update_R;
    Eigen::MatrixXd m_F;
    Eigen::MatrixXd m_H;
    Eigen::MatrixXd m_Q;
    Eigen::MatrixXd m_R;
    Eigen::MatrixXd m_P_pri;
    Eigen::MatrixXd m_P_pos;
    Eigen::MatrixXd m_K;
    Eigen::VectorXd m_state_pri;
    Eigen::VectorXd m_state_pos;
    Eigen::VectorXd m_measurement;
    double m_dt;
};
} // namespace armor_auto_aim
#endif // KALMAN_FILTER_H