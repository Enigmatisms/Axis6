#include <Eigen/Dense>
#include <iostream>

Eigen::Matrix3d getRotMatrix(const Eigen::Vector3d eulers) {
    Eigen::AngleAxisd Rx(eulers(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(eulers(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(eulers(2), Eigen::Vector3d::UnitZ());
    return Rx.toRotationMatrix() * Ry.toRotationMatrix() * Rz.toRotationMatrix();
}

int main() {
    std::cout << getRotMatrix(Eigen::Vector3d(M_PI / 4, M_PI / 2, 0)) << std::endl;
    printf("%lf\n", atan2(0.0, 0.0));
}