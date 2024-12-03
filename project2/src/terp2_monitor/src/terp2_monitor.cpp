
// Project: ENPM662-Project1-Group1
// License: MIT
// The code in this file represents the collective work of Group 1.
// At times, code has been liberally borrowed from files provided
// with the project's instructions or from OSRF training materials.
// Please see the project report for a list of references, or contact
// the team with specific questions.

#include "terp2_monitor.h"
#include <gazebo_msgs/msg/detail/model_states__struct.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

using namespace std::chrono_literals;

terp2_monitor::terp2_monitor() : Node("terp2_monitor") {

    m_sub_l = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states", 10,
                                                                      [this](const gazebo_msgs::msg::LinkStates::SharedPtr msg) {
                                                                          this->link_state_callback(msg);
                                                                      });
    m_sub_g = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 10,
                                                                       [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
                                                                           this->model_state_callback(msg);
                                                                       });

    m_slow_timer = this->create_wall_timer(2000ms, [this]() { slow_update(); });
}

void terp2_monitor::link_state_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg) {
    double units = 1000;
    m_link_names.clear();
    m_link_coords.clear();
    m_link_orients.clear();
    std::vector<std::string> names = msg->name;
    for (size_t i = 0; i < names.size(); ++i) {
        auto name = names[i];
        auto pose = msg->pose[i];
        // auto twist = msg->twist[i];
        std::vector<double> coord = {pose.position.x * units, pose.position.y * units, pose.position.z * units};
        Quaternion quat = {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};

        m_link_names.push_back(name);
        m_link_coords.push_back(coord);
        m_link_orients.push_back(quat);
    }
}

void terp2_monitor::log_link_positions() const {
    for (size_t i = 0; i < m_link_names.size(); ++i) {
        auto name = m_link_names[i];
        if (name.starts_with("terp2")) {
            auto c_name = name.c_str();
            RCLCPP_INFO(this->get_logger(), "%s Position: x = %.2f, y = %.2f, z = %.2f", c_name, m_link_coords[i][0], m_link_coords[i][1], m_link_coords[i][2]);
            RCLCPP_INFO(this->get_logger(), "%s Orientation: qw = %.2f, qx = %.2f, qy = %.2f, qz = %.2f", c_name, m_link_orients[i].w, m_link_orients[i].x, m_link_orients[i].y, m_link_orients[i].z);
            std::cout << name << " = [";
            for (const auto &row : m_link_orients[i].asTransformationMatrix(m_link_coords[i])) {
                for (double num : row) {
                    std::cout << num << ",";
                }
                std::cout << ";";
            }
            std::cout << "];" << std::endl << std::endl;
        }
    }
    std::cout << std::endl << "----------------------------------------------------------------------" << std::endl;
}

void terp2_monitor::model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    std::vector<std::string> names = msg->name;
    int robot_index = -1;
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i].starts_with("terp2")) {
            robot_index = i;
        }
    }
    if (robot_index > -1) {
        auto pose = msg->pose[robot_index];
        m_position[0] = pose.position.x;
        m_position[1] = pose.position.y;
        m_position[2] = pose.position.z;
        m_orientation.w = pose.orientation.w;
        m_orientation.x = pose.orientation.x;
        m_orientation.y = pose.orientation.y;
        m_orientation.z = pose.orientation.z;

        auto twist = msg->twist[robot_index];
        m_vel_linear[0] = twist.linear.x;
        m_vel_linear[1] = twist.linear.y;
        m_vel_linear[2] = twist.linear.z;
        m_vel_angular[0] = twist.angular.x;
        m_vel_angular[1] = twist.angular.y;
        m_vel_angular[2] = twist.angular.z;
    }
}

void terp2_monitor::slow_update() {
    log_link_positions();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<terp2_monitor>());
    rclcpp::shutdown();
    return 0;
}
