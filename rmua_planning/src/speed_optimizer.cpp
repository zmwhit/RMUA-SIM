#include "speed_optimizer.h"
namespace plt = matplotlibcpp;

SpeedOptimizer::SpeedOptimizer(const Eigen::Vector3d& acc_opti_w, const Eigen::Vector3d& dec_opti_w, 
                               double max_v_, double max_a_) {
    acc_opti_w_s = acc_opti_w(0);
    acc_opti_w_v = acc_opti_w(1);
    acc_opti_w_a = acc_opti_w(2);
    dec_opti_w_s = dec_opti_w(0);
    dec_opti_w_v = dec_opti_w(1);
    dec_opti_w_a = dec_opti_w(2);

    max_v = max_v_;
    max_a = max_a_;

    std::thread(&SpeedOptimizer::visualzie, this).detach();
}
bool SpeedOptimizer::Init(const geometry_msgs::PoseStamped& current_point, const std::vector<geometry_msgs::PoseStamped>& path, const nav_msgs::Odometry& odom, 
                          const double plan_t, const double plan_dt, const double plan_v) {
    int n_path = path.size();
    if (n_path <= 1)
        return false;
    //以最近点为投影点开始纵向规划
    double dis = DBL_MAX;
    for (int i = 0; i < n_path; ++i) {
        double d = Helper::getDistance(path[i], current_point);
        if (d < dis) {
            dis = d;
            project = i;
        }
    }   
    //滚动时域长度
    t = plan_t;
    //时间步长
    dt = plan_dt;
    //优化点数量
    n = t/dt + 1;
    //第一点的纵坐标
    s0 = 0;
    //最后一个点的纵坐标
    max_s = 0;

    std::vector<double> x_list(n), y_list(n), theta_list(n), s_list(n);
    n_left = n_path - project;
    if (n_left <= 1) {
        return false;
    } else if (n_left <= 2) {
        double x0 = current_point.pose.position.x;
        double y0 = current_point.pose.position.y;
        double x1 = path[n_path-1].pose.position.x;
        double y1 = path[n_path-1].pose.position.y;     
        theta0 = std::atan2(y1 - y0, x1 - x0);  
        max_s = std::hypot(y1 - y0, x1 - x0);
        // for (int i = 0; i < n; ++i) {

        // }
    } else {
        int degree = (n_left - 1 <= 5) ? n_left - 1 : 5;
        //B样条拟合路径
        b_spline = std::unique_ptr<tinyspline::BSpline>(new tinyspline::BSpline(n_left, 2, degree));
        std::vector<tinyspline::real> ctrlp_raw = b_spline->controlPoints();
        for (int i = project; i < path.size(); ++i) {
            int j = i - project;
            ctrlp_raw[2*j + 0] = path[i].pose.position.x;
            ctrlp_raw[2*j + 1] = path[i].pose.position.y;
        }
        b_spline->setControlPoints(ctrlp_raw);
        auto b_spline_1d = b_spline->derive();
        //B样条采样
        double delta_s = 1.0/(n - 1);
        for (int i = 0; i < n; ++i) {
            auto result = b_spline->eval(i*delta_s).result();
            auto result1d = b_spline_1d.eval(i*delta_s).result();
            x_list[i] = result[0];
            y_list[i] = result[1];
            theta_list[i] = std::atan2(result1d[1], result1d[0]);
        }
        //路径的切线方向
        theta0 = theta_list[0];
        //纵坐标序列
        s_list.emplace_back(s0);
        for (int i = 0; i < n - 1; ++i) {
            max_s += std::hypot(x_list[i+1] - x_list[i], y_list[i+1] - y_list[i]);
            s_list.emplace_back(max_s);
        }
    }
    
    //地图坐标系下的vx和vy，local2global
    double theta = tf::getYaw(current_point.pose.orientation);
    double vx0 = odom.twist.twist.linear.x*std::cos(theta) - odom.twist.twist.linear.y*std::sin(theta);
    double vy0 = odom.twist.twist.linear.x*std::sin(theta) + odom.twist.twist.linear.y*std::cos(theta);
    //地图坐标系下的速度在路径切线上的投影速度vx, global2local
    init_v = std::min(max_v, std::max(vx0*std::cos(theta0) + vy0*std::sin(theta0), 0.0));

    double stop_s;
    if (init_v < plan_v) {
        stop_s = (plan_v*plan_v)/(2*max_a) + (plan_v*plan_v - init_v*init_v)/(2*max_a);
    } else {
        stop_s = (init_v*init_v)/(2*max_a);
    }
    if (max_s <= stop_s || n >= n_left) {
        n_left = (n >= n_left) ? n_left : n;
        stop = true;
        target_v = 0;
        target_a = -max_a;
    } else {
        stop = false;
        target_v = plan_v;
        target_a = max_a;
    }
    // for (int i = 0; i < n; ++i) {
    //     init_traj.emplace_back(s_list[i], target_v, 0.0);
    // }
    return true;
}
void SpeedOptimizer::visualzie() {
    plt::figure_size(700, 700);
    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!opti_traj.empty()) {
            std::vector<double> t, s, v, a;
            for (int i = 0; i < opti_traj.size(); ++i) {
                t.emplace_back(i*dt);
                s.emplace_back(opti_traj[i](0));
                v.emplace_back(opti_traj[i](1));
                a.emplace_back(opti_traj[i](2));
            }
            
            plt::suptitle("optimize result");
            
            plt::subplot(2, 2, 1);
            plt::plot(t, s);
            plt::grid(true);
            plt::title("s-t");
            
            plt::subplot(2, 2, 2);
            plt::plot(t, v);
            plt::grid(true);
            plt::title("v-t");
            
            plt::subplot(2, 2, 3);
            plt::plot(t, a);
            plt::grid(true);
            plt::title("a-t");

            plt::show(false);
            plt::save(Helper::default_pic_path + "/" + "speed_optimize.png");
            plt::clf();
        }
    }
}
Eigen::Vector4d SpeedOptimizer::getControl() {
    if (opti_traj.size() <= 1)
        return Eigen::Vector4d::Zero();
    int control_index = 0;
    Eigen::Vector4d control;
    control(0) = opti_traj[control_index](1)*std::cos(theta0);
    control(1) = opti_traj[control_index](1)*std::sin(theta0);
    control(2) = opti_traj[control_index](2)*std::cos(theta0);
    control(3) = opti_traj[control_index](2)*std::sin(theta0);
    return control;
}
void SpeedOptimizer::SetHession(const int n, Eigen::SparseMatrix<double>& hession, Eigen::VectorXd& gradient) {
    const int m = 3;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m*n, m*n);
    Eigen::MatrixXd weight(m, m);
    if (stop) {
        weight << 0, 0, 0, 
                  0, dec_opti_w_v*2, 0,
                  0, 0, dec_opti_w_a*2;
        for (int i = 0; i < n; ++i) {
            if (i >= n_left - 1)
                weight(0, 0) = dec_opti_w_s*2; 
            else
                weight(0, 0) = 0; 
            H.block(m*i, m*i, m, m) += weight;
        }
        hession = H.sparseView();
        gradient = Eigen::VectorXd::Zero(m*n);
        for (int i = 0; i < n; ++i) {
            if (i >= n_left - 1)
                gradient(m*i) = -2*dec_opti_w_s*max_s;
            gradient(m*i + 1) = -2*dec_opti_w_v*target_v;
        } 
    } else {
        weight << 0, 0, 0, 
                  0, acc_opti_w_v*2, 0,
                  0, 0, acc_opti_w_a*2;
        for (int i = 0; i < n; ++i) {
            if (i == n - 1)
                weight(0, 0) = acc_opti_w_s*2; 
            H.block(m*i, m*i, m, m) += weight;
        }
        hession = H.sparseView();
        gradient = Eigen::VectorXd::Zero(m*n);
        for (int i = 0; i < n; ++i) {
            if (i == n - 1)
                gradient(m*i) = -2*acc_opti_w_s*max_s;
            gradient(m*i + 1) = -2*acc_opti_w_v*target_v;
        } 
    }
}
void SpeedOptimizer::SetConstrains(const int n, Eigen::SparseMatrix<double>& liner_constrains, Eigen::VectorXd& lower_bound, Eigen::VectorXd& upper_bound) {
    const int m = 3;
    int num_kinematic = m*(n - 1);
    int num_bound = m*n;
    int num_terminal = 2*(m - 1);
    int offset = num_terminal + num_bound + num_kinematic;
    Eigen::MatrixXd A(num_kinematic + num_bound + num_terminal, m*n);
    A.setZero();
    lower_bound = Eigen::VectorXd::Zero(num_kinematic + num_bound + num_terminal);
    upper_bound = Eigen::VectorXd::Zero(num_kinematic + num_bound + num_terminal);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d I2 = Eigen::Matrix3d::Identity();
    I2(2, 2) = 0;
    Eigen::Matrix3d C;
    C << 1, dt, 0.5*dt*dt,
         0, 1, dt, 
         0, 0, 0;
    for (int i = 0; i < n - 1; ++i) {
        A.block(m*i, m*i, m ,m) += C;
        A.block(m*i, m*(i+1), m, m) += -I2;
    }
    std::vector<std::vector<double>> bound{{0, max_s}, {-max_v, max_v}, {-max_a, max_a}};
    for (int i = 0; i < n; ++i) {
        A.block(num_kinematic + m*i, m*i, m, m) = I;
        for (int j = 0; j < m; ++j) {
            lower_bound(num_kinematic + m*i + j) = bound[j][0];
            upper_bound(num_kinematic + m*i + j) = bound[j][1];         
        }
    }
    A.coeffRef(num_kinematic + num_bound, 0) = A.coeffRef(num_kinematic + num_bound + 1, 1) = 1;
    lower_bound(num_kinematic + num_bound) = upper_bound(num_kinematic + num_bound) = 0;
    lower_bound(num_kinematic + num_bound + 1) = upper_bound(num_kinematic + num_bound + 1) = init_v;
    if (stop) {
        // A.coeffRef(num_kinematic + num_bound + 2, m*(n-1)) = 1;
        A.coeffRef(num_kinematic + num_bound + 3,  m*(n-1)+1) = 1;
        // lower_bound(num_kinematic + num_bound + 2) = upper_bound(num_kinematic + num_bound + 2) = max_s;
        lower_bound(num_kinematic + num_bound + 3) = upper_bound(num_kinematic + num_bound + 3) = 0;
    } 
    // std::cout << A << std::endl;
    liner_constrains = A.sparseView();
}
bool SpeedOptimizer::Optimize() {
    double t1 = std::clock();
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false); 
    solver.data()->setNumberOfVariables(3*n);
    solver.data()->setNumberOfConstraints(3*(n - 1) + 3*n + 4);
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd primal_variables, dual_variables;
    dual_variables.resize(3*(n - 1) + 3*n + 4);
    dual_variables.setZero();
    primal_variables.resize(3*n);
    primal_variables.setZero();
    // for (int i = 0; i < n; ++i) {
    //     primal_variables(3*i) = init_traj[i](0);
    //     primal_variables(3*i + 1) = init_traj[i](1);
    //     primal_variables(3*i + 2) = init_traj[i](2);
    // } 
    SetHession(n, hessian, gradient);
    SetConstrains(n, linearMatrix, lowerBound, upperBound);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setBounds(lowerBound, upperBound);
    if (!solver.initSolver()) {
        return false;
    }
    // solver.setWarmStart(primal_variables, dual_variables);
    auto result = solver.solveProblem();
    double t2 = std::clock();
    const Eigen::VectorXd& solution = solver.getSolution();
    solver.clearSolver();
    double time_vel_opti = (t2 - t1)/CLOCKS_PER_SEC*1000;
    dbg(time_vel_opti);
    if (result == OsqpEigen::ErrorExitFlag::NoError) {
        opti_traj.resize(n);
        for (int i = 0; i < n; ++i) { 
            double s = solution(3*i);
            double v = solution(3*i + 1);
            double a = solution(3*i + 2);
            // std::cout << "t: " << i*dt << "\ts: " << s << "\tv: " << v << "\ta: " << a << std::endl;
            opti_traj[i] = Eigen::Vector3d(s, v, a);
        }
        opti_traj[n-1](2) = opti_traj[n-2](2);
        return true;        
    }
    return false;
}