#pragma once
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include "OsqpEigen/Solver.hpp"
#include <tinyspline_ros/tinysplinecpp.h>
#include <geometry_msgs/PoseStamped.h>
#include "dbg.h"
#include "helper.h"
class PathSmoother {
public:
    PathSmoother() = default;
    ~PathSmoother() = default;
    PathSmoother(const Eigen::Vector3d& w, double bound_x);
    bool Optimize(const std::vector<geometry_msgs::PoseStamped> &path);
    std::vector<geometry_msgs::PoseStamped> BsplineFitting(const std::vector<geometry_msgs::PoseStamped>& path, const double sample_s);
    std::vector<geometry_msgs::PoseStamped> getPath();
    std::vector<geometry_msgs::PoseStamped> Stitcher(const std::vector<geometry_msgs::PoseStamped> &path, 
                                                     const geometry_msgs::PoseStamped& current_point, int& progess);

private:
    void SetHession(int n, int m, Eigen::MatrixXd& hession, const double opti_w1, const double opti_w2, const double opti_w3);
    void SetHession(int n, Eigen::SparseMatrix<double>& hession, const double opti_w1, const double opti_w2, const double opti_w3);
    void SetGradient(int n, Eigen::VectorXd& gradient, const std::vector<std::vector<double>>* ref_xy, const double opti_w_ref);
    void SetConstrains(int n, Eigen::SparseMatrix<double>& liner_constrains, Eigen::VectorXd& lowerbound, Eigen::VectorXd& upperbound,
                       const std::vector<std::vector<double>>& ref_xy, const double delta_x, const double delta_y);
    int n;
    double bound_x;
    double bound_y;
    double opti_w_smooth;
    double opti_w_ref;
    double opti_w_length;
    std::vector<Eigen::Vector2d> opti_path;
    std::vector<geometry_msgs::PoseStamped> stitch_path;
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> theta_list;
};