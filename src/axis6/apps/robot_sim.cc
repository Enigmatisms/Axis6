#include <thread>
#include "RoboSim.hpp"
#include "keyCtrl.hpp"

std::atomic_char status = 0x00;
std::array<bool, 8> states;
const std::string dev_name = "/dev/input/by-id/usb-Keychron_Keychron_K2-event-kbd";

void controlFlow(char stat) {
    for (int i = 0; i < 8; i++) {
        if (stat & (0x01 << i))
            states[i] = true;
        else    
            states[i] = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_sim");
    ros::NodeHandle nh;
    RoboSim sim(init_links);
    const Eigen::Matrix4d full_trans = sim.getFullTranfrom(0, 6);
    const double delta = nh.param<double>("/robot_sim/delta", 0.05);
    ros::Publisher bar_pub = nh.advertise<visualization_msgs::Marker>("arms", 24);
    ros::Rate rate(50.0);
    KeyCtrl kc(dev_name, status);
    Eigen::Matrix3d init_rot = full_trans.block<3, 3>(0, 0);
    Eigen::Vector3d tar_pos = full_trans.block<3, 1>(0, 3);
    states.fill(false);
    printf("tar pos: %lf, %lf, %lf\n", tar_pos.x(), tar_pos.y(), tar_pos.z());
    while (ros::ok()) {
        sim.visualize(bar_pub);
        char c = getchar();
        if (c == 'n') 
            break;
    }
    std::thread worker(&KeyCtrl::onKeyThread, &kc);
    worker.detach();
    while (ros::ok()) {
        controlFlow(status);
        if (states[0] == true) {
            tar_pos(0) += delta;
            printf("W\n");
        }
        if (states[1] == true) {
            tar_pos(0) -= delta;
            printf("A\n");
        }
        if (states[2] == true) {
            tar_pos(1) -= delta;
            printf("S\n");
        }
        if (states[3] == true) {
            tar_pos(1) += delta;
            printf("D\n");
        }
        if (states[4] == true) {
            tar_pos(2) -= delta;
            printf("O\n");
        }
        if (states[5] == true) {
            tar_pos(2) += delta;
            printf("P\n");
        }
        Eigen::Matrix4d full_tmp = sim.getFullTranfrom(0, 4);
        Eigen::Vector3d wrist_pos = sim.getWristPosition(tar_pos, init_rot);
        printf("Full tmp is: %lf, %lf, %lf, while wrist is %lf, %lf, %lf\n", full_tmp(0, 3), full_tmp(1, 3), full_tmp(2, 3),
                wrist_pos.x(), wrist_pos.y(), wrist_pos.z()
        );
        // getchar();
        sim.solveBaseAngles(wrist_pos);
        sim.solveWristAngles(init_rot, tar_pos);
        sim.visualize(bar_pub);
        rate.sleep();
    }
    return 0;
}