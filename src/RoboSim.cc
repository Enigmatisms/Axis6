#include "RoboSim.hpp"

RoboSim::RoboSim() {

}

// 还需要输入机器人的形状
RoboSim::RoboSim(const Vector6d& pose) {

}
RoboSim::~RoboSim() {

}

// 根据目标位置以及目标欧拉角（姿态），求解球腕的中心点位置
Eigen::Vector3d RoboSim::getWristPosition(const Eigen::Vector3d& tar_pos, const Eigen::Vector3d& tar_ang) const {
    Eigen::Matrix3d rot = getRotMatrix(tar_ang);
    Eigen::Vector3d slice_rot = rot.block<3, 1>(0, 2);
    return tar_pos - offsets(5) * slice_rot;
}

// 根据球腕中心位置求底部三个关节的角度
void RoboSim::solveBaseAngles(const Eigen::Vector3d& wrist, Eigen::Vector3d& angles) {
    const double r2 = std::pow(wrist.x(), 2) + std::pow(wrist.y(), 2), zd = wrist.z() - offsets(0), r = std::sqrt(r2);
    const double pos_ang = atan2(r, zd), edge = std::sqrt(std::pow(zd, 2) + r2) / 2.0, edge_ang = acos(edge / wrist(1));
    const double theta1 = goodAngle(pos_ang - edge_ang), theta2 = goodAngle(pos_ang + edge_ang);
    const double angle_diff1 = std::abs(base_angles(1) - theta1) + std::abs(base_angles(2) - theta2),
        angle_diff2 = std::abs(base_angles(1) - theta2) + std::abs(base_angles(2) - theta1);
    if (angle_diff1 < angle_diff2) {
        angles(1) = theta1;
        angles(2) = theta2;
    } else {
        angles(1) = theta2;
        angles(2) = theta1;
    }
    const double d2d3 = offsets(1) + offsets(2), perp_edge = std::sqrt(r2 - std::pow(d2d3, 2));
    const double angle_1 = goodAngle(atan2(wrist.y(), wrist.x()) - acos(d2d3 / perp_edge)), angle_2 = goodAngle(M_PI - angle_1);
    if (std::abs(base_angles(0) - angle_1) < std::abs(base_angles(0) - angle_2)) {
        angles(0) = angle_1;
    } else {
        angles(0) = angle_2;
    }
}

// 根据R03以及R，t计算球腕三个关节的角度
void RoboSim::solveWristAngles(const Eigen::Vector3d& base_ang, const Eigen::Matrix3d& tar_rot, const Eigen::Vector3d& tar_pos) {

}

// 根据欧拉角计算旋转矩阵
Eigen::Matrix3d RoboSim::getRotMatrix(const Eigen::Vector3d eulers) {

}