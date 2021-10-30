#pragma once
/**
 * @brief 6轴机器人仿真主要计算模块 
 * @author @copyright hqy (Enigmatisms)
 * @date 2021.10.24
 */
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <array>
#include <visualization_msgs/Marker.h>

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

extern const std::vector<LinkInfo> init_links;

class RoboSim {
public:
    RoboSim(const std::vector<LinkInfo>& links): links(links) {}
    ~RoboSim();
public:
    // 根据目标位置以及目标欧拉角（姿态），求解球腕的中心点位置
    Eigen::Vector3d getWristPosition(const Eigen::Vector3d& tar_pos, const Eigen::Matrix3d& tar_rot) const;

    Eigen::Matrix3d getTransform(int id_from = 0, int id_to = 3) const;

    Eigen::Matrix4d getFullTranfrom(int id_from = 0, int id_to = 6) const;

    // 根据球腕中心位置求底部三个关节的角度
    bool solveBaseAngles(const Eigen::Vector3d& wrist);

    // 根据R03以及R，t计算球腕三个关节的角度
    bool solveWristAngles(const Eigen::Matrix3d& tar_rot, const Eigen::Vector3d& tar_pos);

    // 主要是RVIZ操作
    void visualize(ros::Publisher& bar_pub) const;

    static Eigen::Matrix3d getRotMatrix(const Eigen::Vector3d eulers) {
        Eigen::AngleAxisd roll(eulers.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(eulers.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(eulers.z(), Eigen::Vector3d::UnitZ());
        return roll.toRotationMatrix() * pitch.toRotationMatrix() * yaw.toRotationMatrix();
    }

    void getAngles(std::vector<double>& vec) const {
        for (int i = 0; i < 6; i++) {
            vec[i] = links[i].angle;
        } 
    }
private:
    static void forwardTransform(const LinkInfo& link, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
        const double cos_a = cos(link.angle), sin_a = sin(link.angle), cos_t = cos(link.twist), sin_t = sin(link.twist);
        R << cos_a, -sin_a * cos_t, sin_a * sin_t, 
             sin_a, cos_a * cos_t, -cos_a * sin_t, 
             0,     sin_t,      cos_t;
        t << link.length * cos_a, link.length * sin_a, link.offset;
    }
    // 根据欧拉角计算旋转矩阵

    static double goodAngle(double angle) {
        if (angle > M_PI)
            return angle - M_2PI;
        else if (angle < -M_PI)
            return angle + M_2PI;
        return angle;
    }
    // 由于运动模型解耦 底部三个角度与 顶部三个角度的存储分开 所有角度的角度范围都是-pi~pi
    std::vector<LinkInfo> links;
};
