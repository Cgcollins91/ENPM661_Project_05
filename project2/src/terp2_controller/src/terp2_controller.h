// Project: ENPM662-Project1-Group1
// License: MIT
// The code in this file represents the collective work of Group 1.
// At times, code has been liberally borrowed from files provided
// with the project's instructions or from OSRF training materials.
// Please see the project report for a list of references, or contact
// the team with specific questions.

#include "quaternion.h"
#include "pid.hpp"
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class terp2_controller : public rclcpp::Node {
  public:
    terp2_controller();
    void update();
    void parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  private:
    const std::string m_robot_id = "terp2";
    const double m_PI = std::acos(-1);

    std::vector<double> m_goal_xy{0, 0};
    double m_goal_radius = 0;
    double m_goal_theta = 0;

    std::vector<double> m_vel_linear{0, 0, 0};
    std::vector<double> m_vel_angular{0, 0, 0};
    std::vector<double> m_position{0, 0, 0};
    Quaternion m_orientation{0, 0, 0, 0};

    OnSetParametersCallbackHandle::SharedPtr m_ptrParameterSet;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_v;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_p;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_sub_j;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr m_sub_g;

    double m_velocity = 0;
    double m_steer = 0;
    double m_velocity_max = 40;
    double m_steer_max = m_PI / 2; //90degrees
    double m_turn_radius = 2;
    double m_target_radius = 0.5;  //how close is good enough.
    int m_hold_turn = 0;
    int m_hold_turn_max = 4;
    
    Pid m_pid_steer;
    Pid m_pid_velocity;
    double m_dt = 0.5;
  

    // do stuff
    void set_goals();
    void set_rotational_goal();
    void set_distance_goal();
    void pid_update();
    void robot_go();
  

    // move robot
    void set_robot_drive_wheels(double velocity);
    void set_robot_steering(double value);

    // helpers
    double distance_formula(const double x1, const double y1, const double x2, const double y2);

    // log fuctions
    void log(const std::string &msg) const;
    void log_position() const;
    void log_velocity() const;
    void log_double(const std::string &msg, const double value) const;
};
