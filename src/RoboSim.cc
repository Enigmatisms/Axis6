#include "RoboSim.hpp"

RoboSim::~RoboSim() {
    ;
}

// 根据目标位置以及目标欧拉角（姿态），求解球腕的中心点位置
Eigen::Vector3d RoboSim::getWristPosition(const Eigen::Vector3d& tar_pos, const Eigen::Vector3d& tar_ang) const {
    Eigen::Matrix3d rot = getRotMatrix(tar_ang);
    Eigen::Vector3d slice_rot = rot.block<3, 1>(0, 2);
    return tar_pos - links.back().offset * slice_rot;
}

// 根据球腕中心位置求底部三个关节的角度
void RoboSim::solveBaseAngles(const Eigen::Vector3d& wrist) {
    const double r2 = std::pow(wrist.x(), 2) + std::pow(wrist.y(), 2), zd = wrist.z() - links.front().offset, r = std::sqrt(r2);
    const double pos_ang = atan2(r, zd), edge = std::sqrt(std::pow(zd, 2) + r2) / 2.0, edge_ang = acos(edge / wrist(1));
    const double theta1 = goodAngle(pos_ang - edge_ang), theta2 = goodAngle(pos_ang + edge_ang);
    const double angle_diff1 = std::abs(base_angles(1) - theta1) + std::abs(base_angles(2) - theta2),
        angle_diff2 = std::abs(base_angles(1) - theta2) + std::abs(base_angles(2) - theta1);
    if (angle_diff1 < angle_diff2) {
        links[1].angle = theta1;
        links[2].angle = theta2;
    } else {
        links[1].angle = theta2;
        links[2].angle = theta1;
    }
    const double d2d3 = links[1].offset + links[2].offset, perp_edge = std::sqrt(r2 - std::pow(d2d3, 2));
    const double angle_1 = goodAngle(atan2(wrist.y(), wrist.x()) - acos(d2d3 / perp_edge)), angle_2 = goodAngle(M_PI - angle_1);
    if (std::abs(base_angles(0) - angle_1) < std::abs(base_angles(0) - angle_2)) {
        links[0].angle = angle_1;
    } else {
        links[0].angle = angle_2;
    }
}

// 根据R03以及R，t计算球腕三个关节的角度
void RoboSim::solveWristAngles(const Eigen::Matrix3d& tar_rot, const Eigen::Vector3d& tar_pos) {
    Eigen::Matrix3d R03 = getWristCenterTransform();
    Eigen::Matrix3d R36 = R03.inverse() * tar_rot;
    Eigen::Vector3d p3 = R03.inverse() * tar_pos;       // 在第三个坐标系下的坐标
    // 可以继续使用投影法求解各个角度
    links[3].angle = atan2(p3.y(), p3.x());
    links[4].angle = atan2(p3.z() - links[3].offset, std::sqrt(p3(0) * p3(0) + p3(1) * p3(1)));   
    Eigen::Matrix3d R34, R45, R56;
    Eigen::Vector3d tmp;
    forwardTransform(links[3], R34, tmp);
    forwardTransform(links[4], R45, tmp);
    R56 = R45.inverse() * R34.inverse() * R36;
    links[5].angle = asin((R56(1, 0) - R56(0, 1)) * 0.5);
}

// 根据欧拉角计算旋转矩阵
Eigen::Matrix3d RoboSim::getRotMatrix(const Eigen::Vector3d eulers) {
    Eigen::AngleAxisd axis(eulers);
    return axis.toRotationMatrix();
}

Eigen::Matrix3d RoboSim::getWristCenterTransform() const {
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; i++) {
        Eigen::Matrix3d R;
        Eigen::Vector3d tmp;
        forwardTransform(links[i], R, tmp);
        result = (result * R).eval();
    }
    return result;
}
