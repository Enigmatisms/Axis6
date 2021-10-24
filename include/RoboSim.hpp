#pragma once
/**
 * @brief 6轴机器人仿真主要计算模块 
 * @author @copyright hqy (Enigmatisms)
 * @date 2021.10.24
 */
#include <Eigen/Dense>
#include <iostream>
#include <vector>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
constexpr double M_2PI = 2 * M_PI;

class RoboSim {
public:
    RoboSim();
    RoboSim(const Vector6d& pose);
    ~RoboSim();
public:
    // 根据目标位置以及目标欧拉角（姿态），求解球腕的中心点位置
    Eigen::Vector3d getWristPosition(const Eigen::Vector3d& tar_pos, const Eigen::Vector3d& tar_ang) const;

    // 根据球腕中心位置求底部三个关节的角度
    void solveBaseAngles(const Eigen::Vector3d& wrist, Eigen::Vector3d& angles);

    // 根据R03以及R，t计算球腕三个关节的角度
    void solveWristAngles(const Eigen::Vector3d& base_ang, const Eigen::Matrix3d& tar_rot, const Eigen::Vector3d& tar_pos);
private:
    // 根据欧拉角计算旋转矩阵
    static Eigen::Matrix3d getRotMatrix(const Eigen::Vector3d eulers);

    static double goodAngle(double angle) {
        if (angle > M_PI)
            return angle - M_2PI;
        else if (angle < -M_PI)
            return angle + M_2PI;
    }

    // 由于运动模型解耦 底部三个角度与 顶部三个角度的存储分开 所有角度的角度范围都是-pi~pi
    Eigen::Vector3d base_angles;
    Eigen::Vector3d top_angles;
    Vector6d offsets;
};
