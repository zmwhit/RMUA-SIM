#pragma once
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>


#include <Eigen/Eigen>
#include <Eigen/Core>

#include "helper.h"
#include "ros_headers.h"
#include "teb_local_planner/teb_planner.h"

#include "dbg.h"
class NlpPlanner {
public:
    NlpPlanner() = default;
    ~NlpPlanner() = default;
    NlpPlanner(ros::NodeHandle& n, const nav_msgs::OccupancyGrid& map_, double dt, double max_v_, double max_a_);
    bool SetInitTrajectory(const std::vector<geometry_msgs::PoseStamped>& path, const nav_msgs::Odometry& odom,
                           const geometry_msgs::PoseStamped& current_point, int& progess);
    void SetObstacles();
    bool OptimizeNLP();
    void getControl(double& vx, double& vy, double& ax, double& ay);
    std::vector<geometry_msgs::PoseStamped> getOptimalPath();
private:
    ros::NodeHandle nh;
    
    double length = 0.6;
    double width = 0.5;
    double R = 0.4;
    double dt;
    int n;
    double max_v;
    double max_a;
    double init_vx;
    double init_vy;
    double control_vx, control_vy;
    double control_ax, control_ay;
    std::vector<geometry_msgs::PoseStamped> opti_traj;
    std::vector<geometry_msgs::PoseStamped> init_traj;
    std::vector<double> time_list;

    teb_local_planner::ObstContainer static_obs;
    teb_local_planner::DynObstContainer dynamic_obs;
    teb_local_planner::RobotFootprintModelPtr robot_footprint;
    teb_local_planner::ViaPoseContainer via_poses;
    
    teb_local_planner::TebOptimalPlanner teb_planner;
    teb_local_planner::TebConfig teb_config;

    nav_msgs::OccupancyGrid map;
};