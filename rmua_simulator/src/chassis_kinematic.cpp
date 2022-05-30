#include "chassis_kinematic.h"

using namespace std;
void ChassisKinematic::init(bool control_mode) {
    nh_.getParam("/update_pose_rate", update_rate);
    nh_.getParam("/width", width);
    nh_.getParam("/length", length);
    nh_.getParam("/max_speed", max_speed);
    nh_.getParam("/max_accel", max_accel);
    nh_.getParam("/max_decel", max_decel);
    nh_.getParam("/max_omega", max_yaw_vel);
    nh_.getParam("/buffer_length", buffer_length);
    control = control_mode;

    topic = "/robot_" + to_string(robot_id);
    string pose_topic = topic + "/base_pose_ground_truth";
    string odom_topic = topic + "/odom";

    map_frame = "map";
    base_frame = topic + "/base_link";
    
    update_pose_timer = nh_.createTimer(ros::Duration(1.0/update_rate), boost::bind(&ChassisKinematic::update_pose, this));
    pose_pub_ = nh_.advertise<nav_msgs::Odometry>(pose_topic, 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
    control_buffer = vector<double>(buffer_length);

    last_period_time = ros::Time::now().toSec();
    
}
void ChassisKinematic::set_init_state(double x, double y, double a) {
    init_state.x = x;
    init_state.y = y;
    init_state.theta = a;
    init_state.angular_velocity = 0.0;
    init_state.velocity = 0.0;
    init_state.velocity_x = 0.0;
    init_state.velocity_y = 0.0;
    init_state.acceleration = 0.0;
    current_state = init_state;
}

void ChassisKinematic::set_control_input(const geometry_msgs::Twist& input) {
    if (control) {
        control_vel = input.linear.x;
        control_vel = max(min(max_speed, control_vel), -max_speed);
        control_yaw_rate = input.angular.z;
        control_yaw_rate = max(min(max_yaw_vel, control_yaw_rate), -max_yaw_vel);
    } else {
        control_vel_x = input.linear.x;
        control_vel_y = input.linear.y;
        control_vel_x = max(min(max_speed, control_vel_x), -max_speed);
        control_vel_y = max(min(max_speed, control_vel_y), -max_speed);
        control_yaw_rate = input.angular.z;
        control_yaw_rate = max(min(max_yaw_vel, control_yaw_rate), -max_yaw_vel);
    }

}
void ChassisKinematic::update_pose() {
    double current_period_time = ros::Time::now().toSec();
    if (control) {
        current_state = update_nonholonomic(current_state,
                                            control_vel,
                                            control_yaw_rate,
                                            current_period_time - last_period_time); 
    } else {
        current_state = update_holonomic(current_state,
                                         control_vel_x,
                                         control_vel_y,
                                         control_yaw_rate,
                                         current_period_time - last_period_time);
    }
    current_state.velocity = min(max(current_state.velocity, -max_speed), max_speed);
    last_period_time = current_period_time;
    pub_odom(ros::Time::now());
    pub_pose_gt(ros::Time::now());
}
void ChassisKinematic::update_pose(const ros::TimerEvent& e) {
    update_pose();
}
void ChassisKinematic::pub_odom(ros::Time timestamp) {
    // Make an odom message and publish it
    nav_msgs::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = map_frame;
    odom.child_frame_id = base_frame;
    if (control) {
        odom.twist.twist.linear.x = current_state.velocity;
        odom.twist.twist.linear.y = 0;
    } else {
        odom.twist.twist.linear.x = current_state.velocity_x;
        odom.twist.twist.linear.y = current_state.velocity_y;
    }

    odom.twist.twist.angular.z = current_state.angular_velocity;
    odom_pub_.publish(odom);
}
void ChassisKinematic::pub_pose_gt(ros::Time timestamp) {
    nav_msgs::Odometry pose;
    pose.header.stamp = timestamp;
    pose.header.frame_id = map_frame;
    pose.child_frame_id = base_frame;
    
    pose.pose.pose.position.x = current_state.x;
    pose.pose.pose.position.y = current_state.y;
    pose.pose.pose.position.z = 0.0;
    
    pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_state.theta);
    pose_pub_.publish(pose);

    tf::TransformBroadcaster tf_;
    geometry_msgs::TransformStamped tf_map2odom;
    tf_map2odom.header.frame_id = "/map";
    tf_map2odom.child_frame_id = topic + "/odom";
    tf_map2odom.header.stamp = ros::Time::now();
    tf_map2odom.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
    tf_.sendTransform(tf_map2odom);

    geometry_msgs::TransformStamped tf_odomo2baselink;
    tf_odomo2baselink.header.frame_id = topic + "/odom";
    tf_odomo2baselink.child_frame_id = topic + "/base_link";  
    tf_odomo2baselink.transform.translation.x = current_state.x;
    tf_odomo2baselink.transform.translation.y = current_state.y;
    tf_odomo2baselink.transform.rotation = tf::createQuaternionMsgFromYaw(current_state.theta);
    tf_odomo2baselink.header.stamp = ros::Time::now();
    tf_.sendTransform(tf_odomo2baselink);    
}
CarState ChassisKinematic::update_holonomic(const CarState& start,
                                            double velocity_x,
                                            double velocity_y,
                                            double yaw_rate,
                                            double dt) {
    CarState end;
    double x_dot = velocity_x*std::cos(start.theta) + velocity_y*std::cos(start.theta+M_PI_2);
    double y_dot = velocity_x*std::sin(start.theta) + velocity_y*std::sin(start.theta+M_PI_2);
    double theta_dot = yaw_rate;

    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity_x = velocity_x;
    end.velocity_y = velocity_y;
    end.angular_velocity = yaw_rate;
    return end;     
}
CarState ChassisKinematic::update_nonholonomic(const CarState& start,
                                               double velocity,
                                               double yaw_rate,
                                               double dt) {
    CarState end;
    
    double x_dot = velocity * cos(start.theta);
    double y_dot = velocity * sin(start.theta);
    double theta_dot = yaw_rate;

    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = velocity;
    end.angular_velocity = yaw_rate;
    return end;     
}