#include "nlp_planner.h"
using namespace teb_local_planner;
NlpPlanner::NlpPlanner(ros::NodeHandle& n, const nav_msgs::OccupancyGrid& map_, double dt, double max_v_, double max_a_) {
    map = map_;
    nh = n;

    max_v = max_v_;
    max_a = max_a_;

    teb_config.trajectory.dt_ref = dt;
    teb_config.trajectory.dt_hysteresis = 0.01;
    teb_config.trajectory.exact_arc_length = true;
    teb_config.trajectory.global_plan_overwrite_orientation = true;
    teb_config.trajectory.allow_init_with_backwards_motion = true;
    teb_config.trajectory.max_global_plan_lookahead_dist = 2.0;
    teb_config.trajectory.force_reinit_new_goal_dist = 0.1;
    teb_config.trajectory.feasibility_check_no_poses = 10;
    teb_config.trajectory.min_samples = 3;
    teb_config.trajectory.max_samples = 100;

    teb_config.optim.optimization_verbose = true;
    teb_config.robot.max_vel_y = max_v;
    teb_config.robot.acc_lim_y = max_a;
    teb_config.robot.max_vel_x = max_v;
    teb_config.robot.max_vel_x_backwards = -max_v;
    teb_config.robot.acc_lim_x = max_a;
    teb_config.robot.max_vel_theta = 1e3;
    teb_config.robot.acc_lim_theta = 1e3;
    teb_config.robot.min_turning_radius = 0.0;
        
    teb_config.obstacles.min_obstacle_dist = 0.02;
    teb_config.obstacles.obstacle_association_cutoff_factor = 0.1;
    teb_config.obstacles.obstacle_association_force_inclusion_factor = 0.1;

    teb_config.optim.optimization_verbose = false;
    teb_config.optim.no_outer_iterations = 10;
    teb_config.optim.no_inner_iterations = 10;
    teb_config.optim.penalty_epsilon = 0.05;
    teb_config.optim.weight_max_vel_x = 1.0;
    teb_config.optim.weight_acc_lim_x = 5.0;
    teb_config.optim.weight_max_vel_y = 1.0;
    teb_config.optim.weight_acc_lim_y = 5.0;
    teb_config.optim.weight_max_vel_theta = 0.0;
    teb_config.optim.weight_acc_lim_theta = 0.0;
    teb_config.optim.weight_optimaltime = 1.0;
    teb_config.optim.weight_kinematics_nh = 0.0;
    teb_config.optim.weight_kinematics_turning_radius = 0.0;
    teb_config.optim.weight_viapoint = 0.1;
    teb_config.optim.weight_obstacle = 1.0;
    teb_config.optim.obstacle_cost_exponent = 1.0;
    teb_config.optim.weight_adapt_factor = 1.1;

    teb_config.recovery.divergence_detection_enable = true;

    robot_footprint = boost::make_shared<PointRobotFootprint>();//质点模型
}
bool NlpPlanner::SetInitTrajectory(const std::vector<geometry_msgs::PoseStamped>& path, const nav_msgs::Odometry& odom, 
                                   const geometry_msgs::PoseStamped& current_point, int& progess) {
    if (path.size() <= 2)
        return false;
    double dis = DBL_MAX;
    int project = progess;
    for (int i = progess; i < path.size(); ++i) {
        double d = Helper::getDistance(path[i], current_point);
        if (d < dis) {
            dis = d;
            project = i;
        }
    }
    if (project >= path.size() - 2)
        return false;
    double s = 0;
    init_traj = {current_point};
    for (int i = project + 1; i < path.size(); ++i) {
        s += Helper::getDistance(path[i], init_traj.back());
        if (s >= teb_config.trajectory.max_global_plan_lookahead_dist)
            break;
        init_traj.emplace_back(path[i]);
    }
    progess = project;

    double vx = odom.twist.twist.linear.x;
    double vy = odom.twist.twist.linear.y;
    init_vx = vx;
    init_vy = vy;
}
void NlpPlanner::SetObstacles() {
    if (init_traj.empty())  return;
    const auto& path = init_traj;
    double range = teb_config.obstacles.obstacle_association_cutoff_factor;

    static_obs.clear();
    dynamic_obs.time_list.clear();
    dynamic_obs.ObstContainer.clear();
    
    for (int i = 0; i < map.info.height; ++i) {
        for (int j = 0; j < map.info.width; ++j) {
            Eigen::Vector2d pt((j + 0.5)*map.info.resolution, (i + 0.5)*map.info.resolution);
            for (int k = 0; k < path.size(); ++k) {
                Eigen::Vector2d p(path[k].pose.position.x, path[k].pose.position.y);
                if ((p - pt).norm() <= range) {
                    PointObstacle* obs = new PointObstacle(pt);
                    static_obs.emplace_back(obs);
                    break;                    
                }
            }
        }
    }
    double teb_handle_obs_num = static_obs.size();
    dbg(teb_handle_obs_num);
}
bool NlpPlanner::OptimizeNLP() { 
    double t1 = std::clock();
    teb_planner = TebOptimalPlanner(teb_config, robot_footprint);
    teb_planner.setDynamicObs(&dynamic_obs);
    teb_planner.setStaticObs(&static_obs);
    teb_planner.setViaPoses(&via_poses);
    geometry_msgs::Twist init_vel;
    init_vel.linear.x = init_vx;
    init_vel.linear.y = init_vy;
    init_vel.angular.z = 0.0;
    bool result = teb_planner.plan(init_traj, &init_vel, false);
    double t2 = std::clock();
    double time_teb_optimize = (t2 - t1)/CLOCKS_PER_SEC*1000;
    dbg(time_teb_optimize);
    if (result) {
        std::vector<double> x, y, t;
        teb_planner.getFullTrajectory(x, y, t);
        n = x.size();
        
        double v1x = (x[1] - x[0])/t[0];
        double v1y = (y[1] - y[0])/t[0];
        double v2x = (x[2] - x[1])/t[0];
        double v2y = (y[2] - y[1])/t[0];

        control_vx = v1x;
        control_vy = v1y;
        control_ax = (v2x - v1x)/t[0];
        control_ay = (v2y - v1y)/t[0];

        opti_traj.resize(n);
        geometry_msgs::PoseStamped pose;
        for (int i = 0; i < n; ++i) {
            pose.pose.position.x = x[i];
            pose.pose.position.y = y[i];
            pose.pose.position.z = 0.0;
            opti_traj[i] = pose;
        }
        return true;
    }
    return false;
}
void NlpPlanner::getControl(double& vx, double& vy, double& ax, double& ay) {
    vx = control_vx;
    vy = control_vy;
    ax = control_ax;
    ay = control_ay;
}
std::vector<geometry_msgs::PoseStamped> NlpPlanner::getOptimalPath() {
    return opti_traj;
}