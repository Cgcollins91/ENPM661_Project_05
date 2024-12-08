// Project: ENPM662-Project1-Group1
// License: MIT
// The code in this file represents the collective work of Group 1.
// At times, code has been liberally borrowed from files provided
// with the project's instructions or from OSRF training materials.
// Please see the project report for a list of references, or contact
// the team with specific questions.

#include "pid.hpp"
#include "quaternion.h"
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class terp2_monitor : public rclcpp::Node {
  public:
    terp2_monitor();
    void update();
    void slow_update();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    void link_state_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg);

  private:
    std::vector<double> m_vel_linear{0, 0, 0};
    std::vector<double> m_vel_angular{0, 0, 0};
    std::vector<double> m_position{0, 0, 0};
    Quaternion m_orientation{0, 0, 0, 0};

    std::vector<std::string> m_link_names;
    std::vector<std::vector<double>> m_link_coords;
    std::vector<Quaternion> m_link_orients;

    OnSetParametersCallbackHandle::SharedPtr m_ptrParameterSet;
    rclcpp::TimerBase::SharedPtr m_slow_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_sub_j;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr m_sub_g;
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr m_sub_l;

    double m_velocity = 0;
    double m_steer = 0;

    void log_link_positions() const;

};
