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

struct LinkInfo {
    const double offset;
    const double length;
    const double twist;
    double angle;
    LinkInfo(): offset(0.0), length(0.5), twist(0.0), angle(0.0) {}
    LinkInfo(double of, double le, double tw, double an):
        offset(of), length(le), twist(tw), angle(an) {}
};

class RoboSim {
public:
    RoboSim(const std::vector<LinkInfo>& links): links(links) {}
    ~RoboSim();
public:
    // 根据目标位置以及目标欧拉角（姿态），求解球腕的中心点位置
    Eigen::Vector3d getWristPosition(const Eigen::Vector3d& tar_pos, const Eigen::Vector3d& tar_ang) const;

    // 根据球腕中心位置求底部三个关节的角度
    void solveBaseAngles(const Eigen::Vector3d& wrist);

    // 根据R03以及R，t计算球腕三个关节的角度
    void solveWristAngles(const Eigen::Matrix3d& tar_rot, const Eigen::Vector3d& tar_pos);
private:
    static void forwardTransform(const LinkInfo& link, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
        const double cos_a = cos(link.angle), sin_a = sin(link.angle), cos_t = cos(link.twist), sin_t = sin(link.twist);
        R << cos_a, -sin_a * cos_t, sin_a * sin_t, 
             sin_a, cos_a * cos_t, -cos_a * sin_t, 
             0,     sin_t,      cos_t;
        t << link.length * cos_a, link.length * sin_a, link.offset;
    }

    Eigen::Matrix3d getWristCenterTransform() const;

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
    std::vector<LinkInfo> links;
};
