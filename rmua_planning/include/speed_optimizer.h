#pragma once
#include <vector>
#include <math.h>
#include <thread>
#include <mutex>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include "OsqpEigen/Solver.hpp"
#include <tinyspline_ros/tinysplinecpp.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "dbg.h"
#include "helper.h"
#include "matplotlibcpp.h"
class SpeedOptimizer {
public:
    SpeedOptimizer(const Eigen::Vector3d& acc_opti_w, const Eigen::Vector3d& dec_opti_w, 
                   double max_v_, double max_a_);
    SpeedOptimizer() = default;
    ~SpeedOptimizer() = default;
    bool Init(const geometry_msgs::PoseStamped& current_point, const std::vector<geometry_msgs::PoseStamped>& path, const nav_msgs::Odometry& odom, 
              const double plan_t, const double plan_dt, const double plan_v);
    bool Optimize();
    Eigen::Vector4d getControl();
    int getProject() {return project;}
    void visualzie();
private:
    void SetHession(const int n, Eigen::SparseMatrix<double>& hession, Eigen::VectorXd& gradient);
    void SetConstrains(const int n, Eigen::SparseMatrix<double>& liner_constrains, Eigen::VectorXd& lower_bound, Eigen::VectorXd& upper_bound);
    std::unique_ptr<tinyspline::BSpline> b_spline;

    bool stop;
    int project;
    int n;
    int n_send;
    double max_s;
    double s0;
    std::vector<double> s_list;
    double init_v;
    double theta0;
    double t;
    double dt;
    double target_v, target_a;
    std::vector<Eigen::Vector3d> init_traj;
    std::vector<Eigen::Vector3d> opti_traj;
    double acc_opti_w_v;
    double acc_opti_w_a;
    double acc_opti_w_s;
    double dec_opti_w_v;
    double dec_opti_w_a;
    double dec_opti_w_s;

    double max_v;
    double max_a;
};