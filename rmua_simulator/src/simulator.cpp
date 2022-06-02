#include <iostream>
#include <vector>
#include <math.h>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <Eigen/Core>

#include "ros_headers.h"
#include <yaml-cpp/yaml.h>

#include "chassis_kinematic.h"
#include "gimbal_kinematic.h"
#include "visualization/visualization.h"
#include "math/polygon2d.h"
using namespace std;

ros::Subscriber pos_sub_, odom_sub_, joy_sub_, map_sub_, ctrl_sub_, init_pos_sub_;
ros::Publisher pose_pub_, history_pub_, footprint_pub_;

nav_msgs::OccupancyGrid static_map;
geometry_msgs::Twist ctrl_command;
double current_vel, current_yawrate;
geometry_msgs::PoseStamped current_pose, init_pose;

int agent_id = 0;
bool human_control = false;
int initialize = 0;
double joy_vel_coeff = 2.0;
double joy_omega_coeff = 2.0;

double length = 0.6;
double width = 0.5;
double height = 0.5;
vector<geometry_msgs::PoseStamped> history_trajectory;
int history_buff_size = 20;
int history_buff_ptr = 0;
std::unique_ptr<ChassisKinematic> car_phy_sim;
std::unique_ptr<GimbalKinematic> shoot_phy_sim;
double init_x, init_y, init_a;
int control_id = -1;
int show_mesh = 0;
std::vector<math::Polygon2d> static_obs;
int enable_collision_check;
void visualization_sim() {
    std::vector<std::vector<double>> points_list_triangle = {{0, width/2}, {0, -width/2}, {length/2, 0}};                                 
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_arry;
    geometry_msgs::Point point;
    // car footprint  
    int color_id = (agent_id == 0 || agent_id == 1) ? 0 : 2;
    if (!show_mesh) {
        marker.header.frame_id = "map";
        marker.ns = "TRIANGLE_LIST";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose = current_pose.pose;
        for(size_t i = 0; i < 3; ++i) {
            point.x = points_list_triangle[i][0];
            point.y = points_list_triangle[i][1];
            point.z = height + 0.001;
            marker.points.emplace_back(point);
        }
        marker.color = Visualization::getColor(color::YELLOW, 0.5);
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker_arry.markers.emplace_back(marker);

        marker.ns = "CUBE";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.position.z += height/2;
        marker.points.clear();
        marker.scale.x = length;
        marker.scale.y = width;
        marker.scale.z = height;
        marker.color = Visualization::getColor(color_id, 1.0);    
        marker_arry.markers.emplace_back(marker);    
    } else {
        marker.header.frame_id = "map";
        marker.ns = "MESH";
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = Visualization::getColor(color_id, 1.0); 

        marker.id++;
        marker.pose = current_pose.pose;
        marker.mesh_resource = "package://rmua_simulator/mesh/zero2hero_rmua2022_simulation/base_link.dae";
        marker_arry.markers.emplace_back(marker);
        
        marker.id++;
        marker.pose = current_pose.pose;
        marker.pose.position.z += 0.26;
        marker.mesh_resource = "package://rmua_simulator/mesh/zero2hero_rmua2022_simulation/arm.dae";
        marker_arry.markers.emplace_back(marker);
        
        marker.id++;
        marker.pose = current_pose.pose;
        marker.pose.position.z += 0.4;
        marker.mesh_resource = "package://rmua_simulator/mesh/zero2hero_rmua2022_simulation/gun.dae";
        marker_arry.markers.emplace_back(marker);   
    }
    footprint_pub_.publish(marker_arry);
}
void poseCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.seq = agent_id;
    current_pose.pose = (*msg).pose.pose;
    pose_pub_.publish(current_pose);
    if(!initialize) {
        initialize = 1;
        init_pose = current_pose;
    }
    history_trajectory[(history_buff_ptr++)%history_buff_size] = current_pose;
    if(history_trajectory.back().header.stamp >= ros::Time(0.1)) {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        path.poses = history_trajectory;
        history_pub_.publish(path);
    }
    visualization_sim();

    if (enable_collision_check) {
        math::Vec2d p(current_pose.pose.position.x, current_pose.pose.position.y);
        double a = tf::getYaw(current_pose.pose.orientation);
        std::vector<math::Vec2d> points{{length/2, width/2}, {-length/2, width/2}, {-length/2, -width/2}, {length/2, -width/2}};
        for (int j = 0; j < 4; ++j) {
            points[j] = points[j].rotate(a) + p;
        }
        math::Polygon2d car(points);
        for (int i = 0; i < static_obs.size(); ++i) {
            if (static_obs[i].DistanceTo(car) < math::kMathEpsilon) {
                ctrl_command.linear.x = 0;
                ctrl_command.linear.y = 0;
                ctrl_command.angular.z = 0;
                car_phy_sim->set_control_input(ctrl_command);
                break;
            }
        }
    }
}
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    control_id = (*msg).buttons[0];
    if(control_id == (agent_id+1)) {
        if (agent_id == 0) {
            ctrl_command.linear.x = (*msg).axes[1]*joy_vel_coeff;
            ctrl_command.linear.y = (*msg).axes[2]*joy_vel_coeff; 
        } else {
            ctrl_command.linear.x = (*msg).axes[1]*joy_vel_coeff;
            ctrl_command.angular.z = (*msg).axes[2]*joy_omega_coeff;            
        }
        car_phy_sim->set_control_input(ctrl_command);        
    }
}
void ctrlCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
    ctrl_command.linear.x = (*msg).linear.x;
    ctrl_command.linear.y = (*msg).linear.y;
    ctrl_command.angular.z = (*msg).angular.z;
    car_phy_sim->set_control_input(ctrl_command);
}
void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    current_vel = (*msg).twist.twist.linear.x;
    current_yawrate = (*msg).twist.twist.angular.z;
}
void initCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (car_phy_sim == nullptr) return;
    if (control_id == agent_id + 1) {
        init_x = (*msg).pose.pose.position.x;
        init_y = (*msg).pose.pose.position.y;
        init_a = tf::getYaw((*msg).pose.pose.orientation);
        car_phy_sim->set_init_state(init_x, init_y, init_a);        
    }
}
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    static_map = (*msg);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh("~");
    
    if(!nh.getParam("agent_id", agent_id)) {
        ROS_ERROR("Failed to get param %d", agent_id);
        assert(false);        
    }
    string topic = "/robot_" + to_string(agent_id);
    pos_sub_ = nh.subscribe<nav_msgs::Odometry>(topic + "/base_pose_ground_truth", 1, &poseCallBack);
    ctrl_sub_ = nh.subscribe<geometry_msgs::Twist>(topic + "/cmd_vel", 1, &ctrlCallBack);
    init_pos_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &initCallBack);
    nh.param("human_control", human_control, true);
    if(human_control) {
        joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &joyCallback);
    }
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic + "/position", 1);
    history_pub_ = nh.advertise<nav_msgs::Path>(topic + "/history_traj", 1);
    footprint_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic + "/vis_footprint", 1);
    history_trajectory.resize(history_buff_size);

    std::string path = ros::package::getPath("rmua_simulator")+"/map/map.yaml";
    YAML::Node data = YAML::LoadFile(path);
    int n = data["Rectangles"].size();
    for (int i = 0; i < n; ++i) {
        double x = data["Rectangles"][i]["rectangle"]["x"].as<double>();
        double y = data["Rectangles"][i]["rectangle"]["y"].as<double>();
        double a = data["Rectangles"][i]["rectangle"]["a"].as<double>()/180*M_PI;
        double l = data["Rectangles"][i]["rectangle"]["l"].as<double>();
        double w = data["Rectangles"][i]["rectangle"]["w"].as<double>();
        double h = data["Rectangles"][i]["rectangle"]["h"].as<double>();
        math::Vec2d center(x, y);
        std::vector<math::Vec2d> points{{l/2, w/2}, {-l/2, w/2}, {-l/2, -w/2}, {l/2, -w/2}};
        for (int j = 0; j < 4; ++j) {
            points[j] = points[j].rotate(a) + center;
        }
        static_obs.emplace_back(points);
    }

    nh.getParam("/length", length);
    nh.getParam("/width", width);
    nh.getParam("/height", height);
    nh.getParam("/show_mesh", show_mesh);
    nh.getParam("/check_collision", enable_collision_check);

    nh.param("max_vel", joy_vel_coeff, 2.0);
    nh.param("max_yaw", joy_omega_coeff, 2.0);
    nh.param("x", init_x, 2.0);
    nh.param("y", init_y, 4.0);
    nh.param("a", init_a, 90.0);
    init_a = init_a/180.0*M_PI;

    shoot_phy_sim = std::unique_ptr<GimbalKinematic>(new GimbalKinematic());
    car_phy_sim = std::unique_ptr<ChassisKinematic>(new ChassisKinematic(nh, agent_id));
    car_phy_sim->init(agent_id);
    car_phy_sim->set_init_state(init_x, init_y, init_a);
    ros::spin();
    car_phy_sim.reset();
    return 0;
}
