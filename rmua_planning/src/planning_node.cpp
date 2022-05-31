#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <random>
#include <map>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/Core>

#include "ros_headers.h"

#include "dbg.h"
#include "visualization/visualization.h"
#include "a_star_planner.h"
#include "path_smoother.h"
#include "speed_optimizer.h"
#include "nlp_planner.h"

#include "glog/logging.h"
#include "gflags/gflags.h"
ros::Publisher cmd_vel_pub_ ;

enum FSM_STATE { INIT, WAIT_TARGET, SEARCH_GLOBAL, REPLAN_TRAJECTORY, STOP };
FSM_STATE task_state = FSM_STATE::INIT;

Visualization visualize;
nav_msgs::OccupancyGrid grid_map, static_map;
double res;
geometry_msgs::PoseStamped current_pose, goal_pose;
nav_msgs::Odometry current_odom;
double target_angle;
double pos_kp, pos_kd, ang_kp, ang_kd;
geometry_msgs::Twist cmd_vel_, cmd_;
ros::Time speed_time;

std::mutex plan_mutex_;

void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {  
    grid_map = (*msg);
    res = grid_map.info.resolution;
    // dbg("recieve static grid map");
}
void StaticMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    static_map = (*msg);
}
void GoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pose = *msg;
    target_angle = tf::getYaw(goal_pose.pose.orientation);
    dbg("recieve a new goal");
    task_state = FSM_STATE::SEARCH_GLOBAL;
}
void ClickPointCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {
    goal_pose.pose.position.x = (*msg).point.x;
    goal_pose.pose.position.y = (*msg).point.y;
    goal_pose.pose.orientation.w = 1.0;
    target_angle = tf::getYaw(goal_pose.pose.orientation);
    dbg("recieve a new goal");
    task_state = FSM_STATE::SEARCH_GLOBAL;
}
void PoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}
void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
}
void SpeedTrackTimer() {
    plan_mutex_.lock();
    ros::Time time = ros::Time::now();
    double dt = (time - speed_time).toSec();
    cmd_vel_.linear.x = cmd_vel_.linear.x + dt*cmd_vel_.angular.x;
    cmd_vel_.linear.y = cmd_vel_.linear.y + dt*cmd_vel_.angular.y;
    speed_time = time;
    plan_mutex_.unlock();
}
void ControlPubTimer() {
    cmd_.linear.x = cmd_vel_.linear.x;
    cmd_.linear.y = cmd_vel_.linear.y;
    cmd_.angular.z = cmd_vel_.angular.z;
    cmd_vel_pub_.publish(cmd_);
}
void ResetControl() {
    plan_mutex_.lock();
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0; 
    plan_mutex_.unlock();
}
void SetControlSpeed(double vx, double vy, double ax, double ay, double theta) {
    plan_mutex_.lock();
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    cmd_vel_.linear.x = vx*cos_theta + vy*sin_theta;
    cmd_vel_.linear.y = -vx*sin_theta + vy*cos_theta;
    cmd_vel_.angular.x = ax*cos_theta + ay*sin_theta;
    cmd_vel_.angular.y = -ax*sin_theta + ay*cos_theta;
    plan_mutex_.unlock();
}
void SetControlYawRate(double w) {
    plan_mutex_.lock();
    cmd_vel_.angular.z = w; 
    plan_mutex_.unlock();
}
void PositionTrack(double goal_x, double goal_y, double x, double y) {
    double control_x = pos_kp*(goal_x - x);
    double control_y = pos_kp*(goal_y - y);
    double theta = tf::getYaw(current_pose.pose.orientation);
    SetControlSpeed(control_x, control_y, 0.0, 0.0, theta);
}
void AngleTrack(double goal_a, double a) {
    double angle_diff = Helper::NormalizeAngle(goal_a - a);
    double w = ang_kp*angle_diff;
    SetControlYawRate(w);
}
void wait_for_messages(double time) {
    double t1 = ros::Time::now().toSec();
    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        double t2 = ros::Time::now().toSec();
        if (t2 - t1 > time)
            break;
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh("~");

    std::string path = ros::package::getPath("rmua_planning");
    Helper::default_pic_path = path +"/scripts/pic";
    Helper::default_csv_path = path +"/scripts/data";
    std::string pos_topic = "/robot_0/position";
    std::string odom_topic = "/robot_0/odom";
    std::string control_topic = "/robot_0/cmd_vel";


    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_log_dir = path + "/log";
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;

    double speed_rate = 200;
    double control_pub_rate = 1000;
    double goal_dist_tol = 0.2;
    double goal_vel_tol = 0.2;
    int progress = 0;
    double plan_rate = 40;

    double path_opti_w_smooth = 5.0;
    double path_opti_w_ref = 0.5;
    double path_opti_w_length = 5.0;
    double path_opti_xy_bound = 0.1;
    Eigen::Vector3d path_w(path_opti_w_smooth, path_opti_w_ref, path_opti_w_length);
    double plan_target_vel = 2.5;
    double plan_max_accel = 6.0;
    double plan_max_vel = 3.0;
    double plan_horizon = plan_max_vel/plan_max_accel*2;

    double speed_acc_opti_w_s = 0.1;
    double speed_acc_opti_w_v = 5.0;
    double speed_acc_opti_w_a = 0.01;

    double speed_dec_opti_w_s = 10.0;
    double speed_dec_opti_w_v = 1.0;
    double speed_dec_opti_w_a = 0.01;
    Eigen::Vector3d speed_w1(speed_acc_opti_w_s, speed_acc_opti_w_v, speed_acc_opti_w_a);
    Eigen::Vector3d speed_w2(speed_dec_opti_w_s, speed_dec_opti_w_v, speed_dec_opti_w_a);

    pos_kp = 2.0;
    ang_kp = 2.0;

    bool random_walk = true;
    bool use_teb = false;
    ros::Subscriber map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/dynamic_map", 1, &MapCallBack);
    ros::Subscriber static_map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/static_map", 1, &StaticMapCallBack);
    ros::Subscriber goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &GoalCallBack);
    ros::Subscriber click_point_sub_ = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &ClickPointCallBack);
    ros::Subscriber pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pos_topic, 1, &PoseCallBack);
    ros::Subscriber odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, &OdomCallBack);
    ros::Timer speed_timer_ = nh.createTimer(ros::Duration(1.0/speed_rate), boost::bind(&SpeedTrackTimer));
    ros::Timer control_pub_timer_ = nh.createTimer(ros::Duration(1.0/control_pub_rate), boost::bind(&ControlPubTimer));
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(control_topic, 1);
    wait_for_messages(0.5);

    visualize.Init(nh, "vis");

    std::vector<geometry_msgs::PoseStamped> global_path, stitch_path, smooth_path, fitting_path, teb_path;
    AStarPlanner path_search(grid_map);
    PathSmoother path_smoother(path_w, path_opti_xy_bound);
    SpeedOptimizer speed_optimizer(speed_w1, speed_w2, plan_max_vel, plan_max_accel);
    
    LOG(INFO) << "begin planning";
    ros::Rate rate(plan_rate);
    while (ros::ok()) { 
        if (task_state == FSM_STATE::SEARCH_GLOBAL) {
            path_search.UpdateMap(grid_map);
            path_search.Plan(current_pose, goal_pose, global_path);   
            progress = 0;
            if (global_path.size() > 2) {
                LOG(INFO) << "find global path";
                task_state = FSM_STATE::REPLAN_TRAJECTORY;
            } else {
                LOG(WARNING) << "goal is too short";
                task_state = FSM_STATE::STOP;
            }
        }
        if (task_state == FSM_STATE::REPLAN_TRAJECTORY) {
            double curr_speed = std::hypot(current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y);
            if (Helper::getDistance(current_pose, goal_pose) < goal_dist_tol &&
                curr_speed < goal_vel_tol) {
                LOG(INFO) << "reach goal tol";
                task_state = FSM_STATE::STOP;
            } else {
                if (use_teb) {
                    //TODO: add teb
                    NlpPlanner teb_planner(nh, grid_map, 1.0/plan_rate, plan_max_vel, plan_max_accel);
                    if (teb_planner.SetInitTrajectory(global_path, current_odom, current_pose, progress)) {
                        teb_planner.SetObstacles();
                        if (teb_planner.OptimizeNLP()) {
                            double vx, vy, ax, ay;
                            teb_planner.getControl(vx, vy, ax, ay);
                            teb_path = teb_planner.getOptimalPath();
                            double theta = tf::getYaw(current_pose.pose.orientation);
                            SetControlSpeed(vx, vy, ax, ay, theta);                        
                        } else {
                            LOG(ERROR) << "teb optimize fail";
                            task_state = FSM_STATE::STOP;
                        }                        
                    } else {
                        LOG(ERROR) << "teb init fail";
                        task_state = FSM_STATE::STOP;
                    }
                } else {
                    stitch_path = path_smoother.Stitcher(global_path, current_pose, progress);
                    if (path_smoother.Optimize(stitch_path)) {
                        smooth_path = path_smoother.getPath();
                        fitting_path = path_smoother.BsplineFitting(smooth_path, res);
                        bool init = speed_optimizer.Init(current_pose, fitting_path, current_odom, plan_horizon, 1.0/plan_rate, plan_target_vel);
                        if (init) {
                            if (speed_optimizer.Optimize()) {
                                auto control = speed_optimizer.getControl();
                                double theta = tf::getYaw(current_pose.pose.orientation);
                                SetControlSpeed(control(0), control(1), control(2), control(3), theta);
                            } else {
                                LOG(ERROR) << "speed optimize fail";
                                task_state = FSM_STATE::STOP;         
                            }           
                        } else {
                            LOG(ERROR) << "speed init fail";
                            task_state = FSM_STATE::STOP;
                        }
                    } else {
                        LOG(ERROR) << "path optimize fail";
                        task_state = FSM_STATE::STOP;
                    }                    
                }
            }
        }
        if (task_state == FSM_STATE::STOP) {
            PositionTrack(goal_pose.pose.position.x, goal_pose.pose.position.y, 
                          current_pose.pose.position.x, current_pose.pose.position.y);
            if (Helper::getDistance(current_pose, goal_pose) < res && 
                std::hypot(current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y) < 5e-2) {
                task_state = FSM_STATE::WAIT_TARGET;
            }
        }
        if (task_state == FSM_STATE::WAIT_TARGET) {
            ResetControl();
            LOG(INFO) << "wait for target";
            if (random_walk) {
                double x, y;
                int map_x, map_y;
                int get_goal = 0;
                std::random_device rd;
                std::uniform_real_distribution<double> rand_pos = std::uniform_real_distribution<double>(0.0, 1.0);;
                std::default_random_engine eng(rd());
                while (!get_goal) {
                    x = rand_pos(eng) * grid_map.info.width*res;
                    y = rand_pos(eng) * grid_map.info.height*res;
                    if (path_search.World2Map(x, y, map_x, map_y)) {
                        if (path_search.GetCost(map_x, map_y) == 0) 
                            get_goal = 1;
                    }
                }
                goal_pose.pose.position.x = x;
                goal_pose.pose.position.y = y;
                target_angle = M_PI*2*std::sin(std::clock());
                task_state = FSM_STATE::SEARCH_GLOBAL;                
            }
        }
        double t = ros::Time::now().toSec();
        double swing_angle = M_PI/3*std::sin(2*M_PI*t);
        // swing_angle = 0;
        AngleTrack(target_angle + swing_angle, tf::getYaw(current_pose.pose.orientation));


        visualize.drawLine(0, global_path, 0, 0.075, color::ORANGE, 1.0);
        visualize.drawLine(1, fitting_path, 0.05, 0.05, color::PURPLE, 1.0);
        visualize.drawLine(2, teb_path, 0.05, 0.05, color::LIME_GREEN, 1.0);
        visualize.Trigger();
        ros::spinOnce();
        rate.sleep(); 
    }

    google::ShutdownGoogleLogging();
    return 0;
}