#include "path_smoother.h"

void PathSmoother::SetHession(int n, int m, Eigen::MatrixXd& hession, const double opti_w1, const double opti_w2, const double opti_w3) {
    for (int i = 1; i < n - 1; ++i) {
        hession.coeffRef(i + m, i + m) += 4*opti_w3*2;
        hession.coeffRef(i + 1 + m, i + 1 + m) += 1*opti_w3*2;
        hession.coeffRef(i - 1 + m, i - 1 + m) += 1*opti_w3*2;

        hession.coeffRef(i + 1 + m, i - 1 + m) += 1*opti_w3*2;
        hession.coeffRef(i - 1 + m, i + 1 + m) += 1*opti_w3*2;
        hession.coeffRef(i + m, i + 1 + m) += -2*opti_w3*2;
        hession.coeffRef(i + 1 + m, i + m) += -2*opti_w3*2;
        hession.coeffRef(i + m, i - 1 + m) += -2*opti_w3*2;
        hession.coeffRef(i - 1 + m, i + m) += -2*opti_w3*2;
    }   
    for (int i = 0; i < n; ++i) {
        hession.coeffRef(i + m, i + m) += 1*opti_w1*2;
    }
    for (int i = 0; i < n - 1; ++i) {
        hession.coeffRef(i + m, i + m) += 1*opti_w2*2;
        hession.coeffRef(i + 1 + m, i + 1 + m) += 1*opti_w2*2;
        hession.coeffRef(i + m, i + 1 + m) += -1*opti_w2*2;
        hession.coeffRef(i + 1 + m, i + m) += -1*opti_w2*2;
    }
}
void PathSmoother::SetHession(int n, Eigen::SparseMatrix<double>& hession, const double opti_w1, const double opti_w2, const double opti_w3) {
    Eigen::MatrixXd Q(2*n, 2*n);
    Q.setZero();
    SetHession(n, 0, Q, opti_w1, opti_w2, opti_w3);
    SetHession(n, n, Q, opti_w1, opti_w2, opti_w3);
    hession = Q.sparseView();
}

void PathSmoother::SetGradient(int n, Eigen::VectorXd& gradient, const std::vector<std::vector<double>>* ref_xy, const double opti_w_ref) {
    gradient = Eigen::VectorXd::Zero(2*n);
    for (int i = 0; i < n; ++i) {
        gradient(i) = -2*opti_w_ref*ref_xy->at(i)[0];
        gradient(i + n) = -2*opti_w_ref*ref_xy->at(i)[1];
    }
}

void PathSmoother::SetConstrains(int n, Eigen::SparseMatrix<double>& liner_constrains, Eigen::VectorXd& lowerbound, Eigen::VectorXd& upperbound,
                   const std::vector<std::vector<double>>& ref_xy, const double delta_x, const double delta_y) {
    int num_diff = 2*(n - 1);
    int num_init_vel = num_diff + 4;
    lowerbound = Eigen::VectorXd::Zero(2*(n - 1) + 4 + 2);
    upperbound = Eigen::VectorXd::Zero(2*(n - 1) + 4 + 2);
    for (int i = 0; i < n - 1; ++i) {
        lowerbound(i) = -delta_x;
        upperbound(i) = delta_x;
        lowerbound(i + n - 1) = -delta_y;
        upperbound(i + n - 1) = delta_y;
    }
    
    lowerbound(num_diff) = upperbound(num_diff) = ref_xy.back()[0];//xn
    lowerbound(num_diff + 1) = upperbound(num_diff + 1) = ref_xy.back()[1];//yn
    lowerbound(num_diff + 2) = upperbound(num_diff + 2) = ref_xy[0][0];//x0
    lowerbound(num_diff + 3) = upperbound(num_diff + 3) = ref_xy[0][1];//y0

    
    Eigen::MatrixXd A(2*(n - 1) + 4 + 2, 2*n);
    A.setZero();
    for (int i = 0; i < n - 1; ++i) {
        A.coeffRef(i, i) += -1;
        A.coeffRef(i, i + 1) += 1;
        A.coeffRef(i + n - 1, i + n) += -1;
        A.coeffRef(i + n - 1, i + 1 + n) += 1;
    }
    A.coeffRef(num_diff, n - 1) = A.coeffRef(num_diff + 1, 2*n - 1) = 1;
    A.coeffRef(num_diff + 2, 0) = A.coeffRef(num_diff + 3, n) = 1;

    liner_constrains = A.sparseView();
}
PathSmoother::PathSmoother(const Eigen::Vector3d& w, double bound_xy) {
    opti_w_smooth = w(0);
    opti_w_ref = w(1);
    opti_w_length = w(2);
    bound_x = bound_xy;
    bound_y = bound_xy;
}

std::vector<geometry_msgs::PoseStamped> PathSmoother::Stitcher(const std::vector<geometry_msgs::PoseStamped> &path, 
                                                               const geometry_msgs::PoseStamped& current_point, int& progess) {
    double dis = DBL_MAX;
    int project = progess;
    for (int i = progess; i < path.size(); ++i) {
        double d = Helper::getDistance(path[i], current_point);
        if (d < dis) {
            dis = d;
            project = i;
        }
    }
    stitch_path = {current_point};
    for (int i = project + 1; i < path.size(); ++i) {
        stitch_path.emplace_back(path[i]);
    }
    progess = project;
    return stitch_path;
}
bool PathSmoother::Optimize(const std::vector<geometry_msgs::PoseStamped> &path) {
    n = path.size();
    if (n <= 2) {
        return false;
    } else if (n > 2 && n <= 4) {
        opti_path.resize(n);
        for (int i = 0; i < n; ++i) 
            opti_path[i] = Eigen::Vector2d(path[i].pose.position.x, 
                                           path[i].pose.position.y);
        return true;
    }
    std::vector<std::vector<double>> ref_xy;
    for (int i = 0; i < n; ++i) {
        std::vector<double> temp = {path[i].pose.position.x, path[i].pose.position.y};
        ref_xy.emplace_back(temp);
    }
    double t1 = std::clock();
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(2*n);
    solver.data()->setNumberOfConstraints(2*(n - 1) + 4 + 2);
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd primal_variables, dual_variables;
    dual_variables.resize(2*(n - 1) + 4 + 2);
    dual_variables.setZero();
    primal_variables.resize(2*n);
    for (int i = 0; i < n; ++i) {
        primal_variables(i) = ref_xy[i][0];
        primal_variables(i+n) = ref_xy[i][1];
    } 
    SetHession(n, hessian, opti_w_ref, opti_w_length, opti_w_smooth);
    SetGradient(n, gradient, &ref_xy, opti_w_ref);
    SetConstrains(n, linearMatrix, lowerBound, upperBound, ref_xy, bound_x, bound_y);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setBounds(lowerBound, upperBound);
    if (!solver.initSolver()) {
        return false;
    }
    solver.setWarmStart(primal_variables, dual_variables);
    auto result = solver.solveProblem();
    double t2 = std::clock();
    const Eigen::VectorXd& solution = solver.getSolution();
    solver.clearSolver();
    double time_smooth = (t2 - t1)/CLOCKS_PER_SEC*1000;
    dbg(time_smooth);
    if (result == OsqpEigen::ErrorExitFlag::NoError) {
        opti_path.resize(n);
        for (int i = 0; i < n; ++i) {
            double x = solution(i);
            double y = solution(i+n);
            opti_path[i] = Eigen::Vector2d(x, y);
        }
        return true;
    }
    return false;
}
std::vector<geometry_msgs::PoseStamped> PathSmoother::getPath() {
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped point;
    point.pose.orientation.w = 1.0;
    for (int i = 0; i < n; ++i) {
        point.pose.position.x = opti_path[i](0);
        point.pose.position.y = opti_path[i](1);
        path.emplace_back(point);
    }
    return path;
}
std::vector<geometry_msgs::PoseStamped> PathSmoother::BsplineFitting(const std::vector<geometry_msgs::PoseStamped>& path, const double sample_s) {
    double length = 0;
    for (int i = 0; i < path.size() - 1; ++i) 
        length += Helper::getDistance(path[i+1], path[i]);

    int degree = (path.size() - 1 <= 5) ? path.size() - 1 : 5;
    tinyspline::BSpline b_spline(path.size(), 2, degree);
    b_spline = b_spline.fillKnots(tsBSplineType::TS_CLAMPED, 0, length);
    std::vector<tinyspline::real> ctrlp_raw = b_spline.controlPoints();
    for (int i = 0; i < path.size(); ++i) {
        ctrlp_raw[2 * i + 0] = path[i].pose.position.x;
        ctrlp_raw[2 * i + 1] = path[i].pose.position.y;
    }
    b_spline.setControlPoints(ctrlp_raw);
    auto b_spline_1d = b_spline.derive();
    x_list.clear();
    y_list.clear();
    theta_list.clear();
    std::vector<double> s_list;
    for (double s = 0; s < length; s+=sample_s) {
        s_list.emplace_back(s);
    }
    s_list.emplace_back(length);
    for (int i = 0; i < s_list.size(); ++i) {
        auto result = b_spline.eval(s_list[i]).result();
        auto result1d = b_spline_1d.eval(s_list[i]).result();
        x_list.emplace_back(result[0]);
        y_list.emplace_back(result[1]);
        theta_list.emplace_back(result1d[1]/result1d[0]);
    }
    std::vector<geometry_msgs::PoseStamped> path_;
    geometry_msgs::PoseStamped point;
    point.pose.orientation.w = 1.0;
    for (int i = 0; i < x_list.size(); ++i) {
        point.pose.position.x = x_list[i];
        point.pose.position.y = y_list[i];
        path_.emplace_back(point);
    }
    return path_;
}