#include <thread>
#include <std_msgs/Float64.h>
#include "RoboSim.hpp"
#include "keyCtrl.hpp"

std::atomic_short status = 0x0000;
std::array<bool, 16> states;
const std::string dev_name = "/dev/input/by-id/usb-Keychron_Keychron_K2-event-kbd";

void controlFlow(short stat) {
    for (int i = 0; i < 16; i++) {
        if (stat & (0x0001 << i))
            states[i] = true;
        else    
            states[i] = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_sim");
    ros::NodeHandle nh;
    RoboSim sim(init_links);
    const Eigen::Matrix4d full_trans = sim.getFullTranfrom(0, 6);
    const double delta_t = nh.param<double>("/gazebo_sim/delta_t", 0.05);
    const double delta_a = nh.param<double>("/gazebo_sim/delta_a", 0.005);
    ros::Publisher pubs[6];
    for (int i = 0; i < 6; i++) {
        std::string topic_name = "/axis6/joint" + std::to_string(i) + "_position_controller/command";
        pubs[i] = nh.advertise<std_msgs::Float64>(topic_name, 4, false);
    }
    ros::Rate rate(60.0);
    KeyCtrl kc(dev_name, status);
    Eigen::Matrix3d tar_rot = full_trans.block<3, 3>(0, 0);
    Eigen::Vector3d tar_pos = full_trans.block<3, 1>(0, 3);
    states.fill(false);
    std::thread worker(&KeyCtrl::onKeyThread, &kc);
    worker.detach();
    Eigen::Vector3d delta_pos(0.0, 0.0, 0.0);
    Eigen::Vector3d delta_rot(0.0, 0.0, 0.0);       // 旋转
    while (ros::ok()) {
        short key = status;
        controlFlow(key);
        if (states[0] == true) {        // W
            delta_pos(1) = delta_t;
        }
        if (states[1] == true) {        // A
            delta_pos(0) = -delta_t;
        }
        if (states[2] == true) {        // S
            delta_pos(1) = -delta_t;
        }
        if (states[3] == true) {        // D
            delta_pos(0) = delta_t;
        }
        if (states[5] == true) {        // O
            delta_pos(2) = -delta_t;
        }
        if (states[6] == true) {        // P
            delta_pos(2) = delta_t;
        }
        if (states[7] == true) {
            break;
        }

        if (states[8] == true) {        // B
            delta_rot(0) = delta_a;
        }
        if (states[9] == true) {        // H
            delta_rot(0) = -delta_a;
        }
        if (states[10] == true) {       // N
            delta_rot(1) = delta_a;
        }
        if (states[11] == true) {       // J
            delta_rot(1) = -delta_a;
        }
        if (states[12] == true) {       // M
            delta_rot(2) = delta_a;
        }
        if (states[13] == true) {       // K
            delta_rot(2) = -delta_a;
        }
        Eigen::Matrix3d dr = RoboSim::getRotMatrix(delta_rot);
        Eigen::Vector3d tmp_tar_pos = tar_pos + delta_pos;
        Eigen::Matrix3d tmp_tar_rot = tar_rot * dr;
        Eigen::Vector3d wrist_pos = sim.getWristPosition(tmp_tar_pos, tmp_tar_rot);
        // Eigen::Matrix4d full_tmp = sim.getFullTranfrom(0, 4);
        // printf("Full tmp is: %lf, %lf, %lf, while wrist is %lf, %lf, %lf\n", full_tmp(0, 3), full_tmp(1, 3), full_tmp(2, 3),
        //         wrist_pos.x(), wrist_pos.y(), wrist_pos.z()
        // );
        bool success = sim.solveBaseAngles(wrist_pos);
        if (success == true) {
            success = sim.solveWristAngles(tar_rot, tmp_tar_pos);
            if (success == true) {
                tar_rot = tmp_tar_rot;
                tar_pos = tmp_tar_pos;
            }
        } 
        delta_pos.setZero();
        delta_rot.setZero();
        std::vector<double> angles(6, 0.0);
        sim.getAngles(angles);
        for (int i = 0; i < 6; i++) {
            std_msgs::Float64 msg;
            msg.data = angles[i];
            pubs[i].publish(msg);
        }
        rate.sleep();
    }
    return 0;
}