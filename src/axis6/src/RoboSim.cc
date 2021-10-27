#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "RoboSim.hpp"

typedef Eigen::Vector3d Point;
typedef visualization_msgs::Marker Bar;

const Eigen::Vector4d RED(1.0, 0, 0, 0.8);

// d, a, alpha(twist), theta(init_angles)
const std::vector<LinkInfo> init_links = {
    LinkInfo(0.5, 0, M_PI_2, M_PI / 4), 
    LinkInfo(0.1, 0.8, 0, M_PI / 4), 
    LinkInfo(0.1, 0, M_PI_2, M_PI), 
    LinkInfo(0.8, 0, M_PI_2, M_PI / 6.0), 
    LinkInfo(0, 0, M_PI_2, -M_PI / 4.0),
    LinkInfo(0.4, 0, 0, 0)  
};

// const std::vector<LinkInfo> init_links = {
//     LinkInfo(0.5, 0, M_PI_2, 0), 
//     LinkInfo(0.1, 0.8, 0, 0), 
//     LinkInfo(0.1, 0, M_PI_2, 0), 
//     LinkInfo(0.8, 0, M_PI_2, 0), 
//     LinkInfo(0, 0, -M_PI_2, 0),
//     LinkInfo(0.4, 0, 0, 0)  
// };

RoboSim::~RoboSim() {
    ;
}

Eigen::Vector3d RoboSim::getWristPosition(const Eigen::Vector3d& tar_pos, const Eigen::Matrix3d& rot) const {
    Eigen::Vector3d slice_rot = rot.block<3, 1>(0, 2);
    return tar_pos - links.back().offset * slice_rot;
}

// 根据球腕中心位置求底部三个关节的角度
void RoboSim::solveBaseAngles(const Eigen::Vector3d& wrist) {
    const double r2 = std::pow(wrist.x(), 2) + std::pow(wrist.y(), 2), zd = wrist.z() - links.front().offset;
    const double d2d3 = links[1].offset + links[2].offset, perp_edge = std::sqrt(r2 - std::pow(d2d3, 2)), ofp = std::pow(d2d3, 2);
    const double angle_1 = goodAngle(atan2(wrist.y(), wrist.x()) - atan2(d2d3, perp_edge)), angle_2 = goodAngle(M_PI + angle_1);
    if (std::abs(links[0].angle - angle_1) < std::abs(links[0].angle - angle_2)) {
        links[0].angle = angle_1;
    } else {
        links[0].angle = angle_2;
    }
    const double r_proj = std::sqrt(r2 - ofp);
    const double pos_ang = atan2(zd, r_proj), edge = std::sqrt(std::pow(zd, 2) + r2 - ofp) / 2.0, edge_ang = acos(edge / links[1].length);
    const double theta1 = goodAngle(pos_ang - edge_ang), theta2 = goodAngle(pos_ang + edge_ang);
    const double angle_diff1 = std::abs(links[1].angle - theta1), angle_diff2 = std::abs(links[1].angle - theta2);
    if (angle_diff1 < angle_diff2) {
        links[1].angle = theta1;
        links[2].angle = goodAngle(theta2 - theta1 + M_PI_2);
    } else {
        links[1].angle = theta2;
        links[2].angle = goodAngle(theta1 - theta2 + M_PI_2);
    }
    
    // ========== debug ===========
    printf("Base angles:\n");
    for (int i = 0; i < 3; i++) {
        printf("%lf, ", links[i].angle * 180.0 / M_PI);
    }
    printf("%lf, %lf, %lf, %lf, %lf, %lf\n", angle_1 * 180.0 / M_PI, angle_2 * 180.0 / M_PI, 
        theta1 * 180.0 / M_PI, theta2 * 180.0 / M_PI, edge_ang * 180.0 / M_PI, pos_ang * 180.0 / M_PI);
}

// 根据R03以及R，t计算球腕三个关节的角度
// 球腕问题需要重新推导（个人感觉你坐标系变换理解的不行）
void RoboSim::solveWristAngles(const Eigen::Matrix3d& tar_rot, const Eigen::Vector3d& tar_pos) {
    Eigen::Matrix3d R03 = getTransform();
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
    printf("Wrist angles:\n");
    for (int i = 3; i < 6; i++) {
        printf("%lf, ", links[i].angle * 180.0 / M_PI);
    }
    printf("\n");
}

Eigen::Matrix3d RoboSim::getTransform(int id_from, int id_to) const {
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    for (int i = id_from; i < id_to; i++) {
        Eigen::Matrix3d R;
        Eigen::Vector3d tmp;
        forwardTransform(links[i], R, tmp);
        result = (result * R).eval();
    }
    return result;
}

Eigen::Matrix4d RoboSim::getFullTranfrom(int id_from, int id_to) const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    for (int i = id_from; i < id_to; i++) {
        Eigen::Matrix3d R;
        Eigen::Vector3d tmp;
        forwardTransform(links[i], R, tmp);
        Eigen::Matrix4d transform;
        transform << R, tmp, Eigen::RowVector4d(0, 0, 0, 1);
        result = (result * transform).eval();
    }
    return result;
}


bool sendTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, std::string frame_id, std::string child_frame_id) {
    static tf::TransformBroadcaster tfbr;
    tf::Matrix3x3 tf_R(R(0, 0), R(0, 1), R(0, 2),
                       R(1, 0), R(1, 1), R(1, 2),
                       R(2, 0), R(2, 1), R(2, 2));
    tf::Vector3 tf_t(t.x(), t.y(), t.z());
    tf::Transform transform(tf_R, tf_t);
    tfbr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
    return true;
}

bool makeVisualizationMarker(const std::vector<Point>& pts, Bar& line_list, std::string ns, std::string frame_id, Eigen::Vector4d color) {
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.action = visualization_msgs::Marker::MODIFY;
    line_list.pose.orientation.w = 1.0;
    line_list.scale.x = 0.02;
    line_list.color.r = color(0);
    line_list.color.g = color(1);
    line_list.color.b = color(2);
    line_list.color.a = color(3);
    line_list.lifetime = ros::Duration(0);
    for (const Point& p: pts) {
        geometry_msgs::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        line_list.points.push_back(pt);
    }
}

// 需要有7个坐标系
void RoboSim::visualize(ros::Publisher& bar_pub) const {
    //求出每个坐标系与下一个坐标系之间的位姿变换
    double last_link_length = 0.0;
    for (int i = 0; i < 6; i++) {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        forwardTransform(links[i], R, t);
        sendTransform(R, t, "base" + std::to_string(i), "base" + std::to_string(i + 1));
        const double offset = links[i].offset;
        std::vector<Point> arm_bar;
        if (last_link_length > 1e-4)
            arm_bar.emplace_back(- last_link_length, 0, 0);
        arm_bar.emplace_back(0, 0, 0);
        arm_bar.emplace_back(0, 0, offset);
        last_link_length = links[i].length;
        Bar bar_offset;
        makeVisualizationMarker(arm_bar, bar_offset, "link" + std::to_string(i), "base" + std::to_string(i), RED);
        bar_pub.publish(bar_offset);
    }
}