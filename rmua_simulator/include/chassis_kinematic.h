#pragma once
#include <memory>
#include <thread>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "ros_headers.h"

struct CarState {
    double x; // x position
    double y; // y position
    double theta; // orientation
    double velocity;
    double velocity_x;
    double velocity_y;
    double acceleration;
    double angular_velocity;
};
class ChassisKinematic {
public:
    ChassisKinematic(){};
    ChassisKinematic(ros::NodeHandle& nh, int id) : nh_(nh), robot_id(id){}
    ~ChassisKinematic() = default;

    void init(bool control_mode);
    void set_control_input(const geometry_msgs::Twist& input);
    void set_init_state(double x, double y, double a);
private:
    void update_pose(const ros::TimerEvent& e);
    void update_pose();
    void pub_odom(ros::Time timestamp);
    void pub_pose_gt(ros::Time timestamp);
    CarState update_nonholonomic(const CarState& start,
                                 double velocity,
                                 double yaw_rate,
                                 double dt);
    CarState update_holonomic(const CarState& start,
                              double velocity_x,
                              double velocity_y,
                              double yaw_rate,
                              double dt);
    ros::NodeHandle nh_;
    ros::Timer update_pose_timer;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;

    int robot_id;
    double update_rate;
    double last_period_time;
    std::string map_frame;
    std::string base_frame;
    std::string topic;
    int buffer_length;

    bool control;
    CarState init_state;
    CarState current_state;
    std::vector<double> control_buffer;
    double control_acc;
    double control_vel;
    double control_vel_x;
    double control_vel_y;
    double control_yaw_rate;
    //parmas
    double length;
    double width;
    double max_speed;
    double max_accel;
    double max_decel;
    double max_yaw_vel;
};