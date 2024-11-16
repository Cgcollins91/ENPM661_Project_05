
// Project: ENPM662-Project1-Group1
// License: MIT
// The code in this file represents the collective work of Group 1.
// At times, code has been liberally borrowed from files provided
// with the project's instructions or from OSRF training materials.
// Please see the project report for a list of references, or contact
// the team with specific questions.

#include "terp1_controller.h"
#include <gazebo_msgs/msg/detail/model_states__struct.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

using namespace std::chrono_literals;

terp1_controller::terp1_controller() : Node("terp1_controller") {

    // parameters
    this->declare_parameter("goal", std::vector<double>{0, 0});

    // publishers
    m_pub_p = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    m_pub_v = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

    // subscribers
    m_sub_j = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                                      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                                                                          this->joint_state_callback(msg);
                                                                      });
    m_sub_g = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 10,
                                                                       [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
                                                                           this->model_state_callback(msg);
                                                                       });

    // parameters callback
    m_ptrParameterSet = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
            this->parameter_callback(parameters);
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    // init pid controllers
    m_pid_velocity.set_k_values(2, 0.01, 0.6);
    m_pid_steer.set_k_values(0.3, 0.001, 0.1);

    // spin the update
    m_timer = this->create_wall_timer(500ms, [this]() { update(); });
}

void terp1_controller::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg; // supression of warning
}

void terp1_controller::model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    std::vector<std::string> names = msg->name;
    int robot_index = -1;
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i].starts_with(m_robot_id)) {
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

    } else {
        log("ERROR: terp1 robot not found!");
    }
}

void terp1_controller::update() {
    set_goals(); // reset goals based on current odometry and current xy target.
    pid_update();
    robot_go();
}

void terp1_controller::pid_update() {

    m_velocity = std::min(m_pid_velocity.calculate(m_goal_radius, m_dt), m_velocity_max);

    m_steer = std::min(std::max(m_pid_steer.calculate(m_goal_theta, m_dt), -m_steer_max), m_steer_max);

    //deadband near target
    if (m_goal_radius < m_target_radius) {
        m_steer = 0;
        m_velocity = 0;
    } else if (std::abs(m_goal_theta) > m_steer_max && m_goal_radius < m_turn_radius) {
    // goal is inside the robot's turning radius
        m_steer = 0;
        m_velocity = m_velocity_max;
        log("Going straight until viable turning radius...");
    }
}

void terp1_controller::robot_go() {
    set_robot_steering(m_steer);
    set_robot_drive_wheels(m_velocity);
}

void terp1_controller::parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &param : parameters) {
        if (param.get_name() == "goal") {
            log("GOAL RECEIVED!");
            m_goal_xy = param.as_double_array();

            // reset pid controllers
            m_pid_steer.reset();
            m_pid_velocity.reset();

            set_goals();
        } else {
            log("UNKNOWN PARAMETER");
        }
    }
}

void terp1_controller::set_robot_drive_wheels(double velocity) {
    std_msgs::msg::Float64MultiArray message = std_msgs::msg::Float64MultiArray();
    message.data = {-1 * velocity, -1 * velocity};
    m_pub_v->publish(message);
}

void terp1_controller::set_robot_steering(double steer_angle) {
    double steer_ratio = steer_angle / m_steer_max;
    std_msgs::msg::Float64MultiArray message = std_msgs::msg::Float64MultiArray();
    message.data = {steer_ratio, -1 * steer_ratio};
    m_pub_p->publish(message);
}

void terp1_controller::set_goals() {
    set_rotational_goal();
    set_distance_goal();
}

void terp1_controller::set_rotational_goal() {

    // yaw angle
    double yawgoal = std::atan2(m_goal_xy[1] - m_position[1], m_goal_xy[0] - m_position[0]);
    yawgoal = yawgoal * 180.0 / m_PI;
    if (yawgoal < 0.0)
        yawgoal += 360.0;
    //log_double("YAW (Z-Axis) GOAL", yawgoal);

    // orientation angle
    double oangle = m_orientation.yawAngleDeg();
    oangle += 90.0;
    if (oangle < 0.0)
        oangle += 360.0;
    //log_double("ORIENTATION ANGLE", oangle);

    // delta angle
    double dangle = yawgoal - oangle;
    if (dangle < 0.0)
        dangle += 360.0;
    //log_double("DELTA ANGLE", dangle);

    // fix angle to goal so that robot makes the smaller of the two turns
    if (dangle > 180.0) {
        dangle = 360.0 - dangle;
        dangle *= -1;
    }
    log_double("ANGLE TO GOAL", dangle);

    m_goal_theta = dangle * m_PI / 180.0;
}

void terp1_controller::set_distance_goal() {
    m_goal_radius = distance_formula(m_position[0], m_position[1], m_goal_xy[0], m_goal_xy[1]);
    log_double("DISTANCE TO GOAL", m_goal_radius);
}

double terp1_controller::distance_formula(const double x1, const double y1, const double x2, const double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double m = (dx * dx) + (dy * dy);
    return std::sqrt(m);
}

void terp1_controller::log(const std::string &msg) const {
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
}

void terp1_controller::log_position() const {
    RCLCPP_INFO(this->get_logger(), "Position: x = %.2f, y = %.2f, z = %.2f", m_position[0], m_position[1], m_position[2]);
    RCLCPP_INFO(this->get_logger(), "Orientation: qw = %.2f, qx = %.2f, qy = %.2f, qz = %.2f", m_orientation.w, m_orientation.x, m_orientation.y, m_orientation.z);
}

void terp1_controller::log_velocity() const {
    RCLCPP_INFO(this->get_logger(), "Linear Velocity: x = %.2f, y = %.2f, z = %.2f", m_vel_linear[0], m_vel_linear[1], m_vel_linear[2]);
    RCLCPP_INFO(this->get_logger(), "Angular Velocity: x = %.2f, y = %.2f, z = %.2f", m_vel_linear[0], m_vel_linear[1], m_vel_linear[2]);
}

void terp1_controller::log_double(const std::string &msg, double value) const {
    RCLCPP_INFO(this->get_logger(), "%s: %.2f", msg.c_str(), value);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<terp1_controller>());
    rclcpp::shutdown();
    return 0;
}
