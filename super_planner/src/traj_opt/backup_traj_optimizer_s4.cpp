/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <traj_opt/backup_traj_optimizer_s4.h>
#include <utils/header/color_msg_utils.hpp>

using namespace traj_opt;
using namespace color_text;
using namespace super_utils;

bool BackupTrajOpt::processCorridor() {
    PolyhedronV curIV, curIOB; // 走廊的顶点
    if (!geometry_utils::enumerateVs(opt_vars.hPolytope, curIV)) {
        std::cout << YELLOW << " -- [SUPER] enumerateVs failed." << RESET << std::endl;
        return false;
    }
    long nv = curIV.cols();
    curIOB.resize(3, nv);
    // 第一个点存储第一个顶点
    curIOB.col(0) = curIV.col(0);
    // 后面的点都归一化到第一个点坐标系下
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytope = curIOB;
    return true;
}

bool BackupTrajOpt::setupProblemAndCheck() {
    // 1. Check if the corridor is feasible
    const Eigen::ArrayXd norms = opt_vars.hPolytope.leftCols<3>().rowwise().norm();
    opt_vars.hPolytope.array().colwise() /= norms;

    if (!processCorridor()) {
        std::cout << YELLOW << " -- [processCorridor] Failed to get Overlap enumerateVs ." << RESET << std::endl;
        return false;
    }

    // 2. Reset the problem dimension
    if (opt_vars.uniform_time_en) {
        opt_vars.temporalDim = 1;
    } else {
        opt_vars.temporalDim = opt_vars.piece_num;
    }
    switch (opt_vars.pos_constraint_type) {
        case 1: {
            opt_vars.spatialDim = 3 * opt_vars.piece_num;
            break;
        }
        default: {
            opt_vars.spatialDim = opt_vars.vPolytope.cols() * opt_vars.piece_num;
        }
    }

    opt_vars.points.resize(3, opt_vars.piece_num);
    return true;
}

bool BackupTrajOpt::configureSplineProblem() {
    time_cost_.weight = opt_vars.rho;
    spatial_map_.reset(&opt_vars.vPolytope,
                       opt_vars.piece_num,
                       opt_vars.pos_constraint_type == 1);
    integral_cost_.reset(&opt_vars.hPolytope,
                         opt_vars.smooth_eps,
                         opt_vars.magnitudeBounds,
                         opt_vars.penaltyWeights,
                         &opt_vars.quadrotor_flatness);
    auxiliary_state_map_.reset(&opt_vars.exp_traj,
                               opt_vars.min_ts,
                               opt_vars.max_ts,
                               opt_vars.weight_ts,
                               opt_vars.uniform_time_en,
                               opt_vars.piece_num,
                               opt_vars.ts);

    optimizer_.setSpatialMap(&spatial_map_);
    optimizer_.setAuxiliaryStateMap(&auxiliary_state_map_);
    optimizer_.setEnergyWeights(opt_vars.block_energy_cost ? 0.0 : 1.0);
    optimizer_.setIntegralNumSteps(opt_vars.integral_res);

    SplineTrajectory::OptimizationFlags flags;
    flags.start_p = false;
    flags.end_p = true;
    flags.start_v = false;
    flags.end_v = false;
    flags.start_a = false;
    flags.end_a = false;
    flags.start_j = false;
    flags.end_j = false;
    optimizer_.setOptimizationFlags(flags);

    StatePVAJ head_state;
    opt_vars.exp_traj.getState(opt_vars.ts, head_state);
    opt_vars.headPVAJ = head_state;
    opt_vars.tailPVAJ.setZero();
    opt_vars.tailPVAJ.col(0) = opt_vars.points.rightCols(1);

    spline_opt::WaypointsType waypoints(opt_vars.piece_num + 1, 3);
    waypoints.row(0) = head_state.col(0).transpose();
    for (int i = 0; i < opt_vars.piece_num - 1; ++i) {
        waypoints.row(i + 1) = opt_vars.points.col(i).transpose();
    }
    waypoints.row(opt_vars.piece_num) = opt_vars.points.col(opt_vars.piece_num - 1).transpose();

    SplineTrajectory::BoundaryConditions<3> bc;
    bc.start_velocity = head_state.col(1);
    bc.start_acceleration = head_state.col(2);
    bc.start_jerk = head_state.col(3);
    bc.end_velocity.setZero();
    bc.end_acceleration.setZero();
    bc.end_jerk.setZero();

    std::vector<double> time_segments(opt_vars.times.data(), opt_vars.times.data() + opt_vars.times.size());
    opt_vars.temporalDim = opt_vars.piece_num;
    return optimizer_.setInitState(time_segments, waypoints, 0.0, bc);
}

double BackupTrajOpt::evaluateCurrentSplineCost(const Eigen::VectorXd &vars, Eigen::VectorXd &grad) {
    ++opt_vars.iter_num;
    integral_cost_.beginEvaluation();
    double cost = optimizer_.evaluate(vars, grad, time_cost_, integral_cost_);
    spatial_map_.addNormPenalty(vars, opt_vars.temporalDim, opt_vars.spatialDim, grad, cost);
    opt_vars.penalty_log = integral_cost_.getPenaltyLog();
    return cost;
}

double BackupTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    Vec3f step = (opt_vars.tailPVAJ.col(0) - opt_vars.headPVAJ.col(0)) / opt_vars.piece_num;
    for (int i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.points.col(i) = step * (i + 1) + opt_vars.headPVAJ.col(0);
    }
    opt_vars.points.rightCols(1) = opt_vars.tailPVAJ.col(0);

    if(opt_vars.given_init_ts_and_ps){
        opt_vars.times = opt_vars.given_init_t_vec;
        for (int i = 0; i < opt_vars.given_init_ps.size(); i++) {
            opt_vars.points.col(i) = opt_vars.given_init_ps[i];
        }
        opt_vars.ts = opt_vars.given_init_ts;
    }

    if (!configureSplineProblem()) {
        return INFINITY;
    }

    Eigen::VectorXd x = optimizer_.generateInitialGuess();
    double minCostFunctional;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = relCostTol;
    int ret;
    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();
    opt_vars.iter_num = 0;

    opt_vars.init_ts = opt_vars.ts;
    opt_vars.init_t_vec = opt_vars.times;
    opt_vars.init_ps.clear();
    for (int col = 0; col < opt_vars.points.cols(); col++) {
        opt_vars.init_ps.emplace_back(opt_vars.points.col(col));
    }

    TimeConsuming ttt(" -- [BackupTrajOpt]", false);
    if (opt_vars.debug_en) {
        throw std::runtime_error(" -- [BackupTrajOpt] Debug mode is not supported yet.");
    } else {
        ret = lbfgs::lbfgs_optimize(x,
                                    minCostFunctional,
                                    [](void *ptr, const Eigen::VectorXd &vars, Eigen::VectorXd &grad) -> double {
                                        return static_cast<BackupTrajOpt *>(ptr)->evaluateCurrentSplineCost(vars, grad);
                                    },
                                    nullptr,
                                    nullptr,
                                    this,
                                    lbfgs_params);

    }

    const int optimizer_iters = opt_vars.iter_num;
    Eigen::VectorXd grad(x.size());
    minCostFunctional = evaluateCurrentSplineCost(x, grad);
    opt_vars.iter_num = optimizer_iters;
    const SplineType *optimal_spline = optimizer_.getOptimalSpline();
    opt_vars.penalty_log(0) = (!opt_vars.block_energy_cost && optimal_spline != nullptr) ? optimal_spline->getEnergy() : 0.0;

    const int aux_offset = opt_vars.temporalDim + opt_vars.spatialDim;
    if (x.size() >= aux_offset + auxiliary_state_map_.getDimension()) {
        const int tau_ts_idx = aux_offset + (opt_vars.uniform_time_en ? 1 : 0);
        opt_vars.ts = auxiliary_state_map_.decodeStartTime(x(tau_ts_idx));
    }

    using namespace std;
    if (cfg_.print_optimizer_log) {
        cout << " -- [BaclOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tTs: " << opt_vars.ts << endl;
    }
    if ((cfg_.penna_pos > 0 && opt_vars.penalty_log(1) > 0.2) ||
        (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin) ||
        (cfg_.penna_acc > 0 && opt_vars.penalty_log(3) > cfg_.max_acc * cfg_.penna_margin) ||
        (cfg_.penna_omg > 0 && opt_vars.penalty_log(6) > cfg_.max_omg * cfg_.penna_margin) ||
        (cfg_.penna_thr > 0 && opt_vars.penalty_log(7) > cfg_.max_acc * cfg_.penna_margin)) {
        ret = -1;
        if (cfg_.print_optimizer_log) {
            cout << " -- [BaclOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
            cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
            cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
            cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
            cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
            cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
            cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
            cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
            cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
            cout << "\tTs: " << opt_vars.ts << endl;
        }
        ros_ptr_->warn(" -- [BackOpt] Opt failed, Omg or thr or Pos violation.");
    }

    if (ret >= 0) {
        if (optimal_spline != nullptr) {
            traj = spline_opt::splineToSuperTrajectory(*optimal_spline);
            opt_vars.times.resize(traj.getPieceNum());
            for (int i = 0; i < opt_vars.times.size(); ++i) {
                opt_vars.times(i) = traj[i].getDuration();
            }
            opt_vars.points.resize(3, opt_vars.piece_num);
            for (int i = 0; i < opt_vars.points.cols(); ++i) {
                opt_vars.points.col(i) = traj.getJuncPos(i + 1);
            }
            opt_vars.headPVAJ = opt_vars.exp_traj.getState(opt_vars.ts);
            opt_vars.tailPVAJ.setZero();
            opt_vars.tailPVAJ.col(0) = opt_vars.points.rightCols(1);
        } else {
            traj.clear();
            minCostFunctional = INFINITY;
        }
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        std::cout << YELLOW << " -- [SUPER] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    }
    return minCostFunctional;
}

BackupTrajOpt::BackupTrajOpt(const traj_opt::Config &cfg, const ros_interface::RosInterface::Ptr &ros_ptr)
        : cfg_(cfg), ros_ptr_(ros_ptr) {
    using namespace std;

    cfg_ = cfg;
    std::string filename = "back_opt_log.csv";
    if(cfg_.save_log_en){
        failed_traj_log.open(DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
        penalty_log.open(DEBUG_FILE_DIR("back_opt_penna.csv"), std::ios::out | std::ios::trunc);
    }
    opt_vars.magnitudeBounds.resize(6);
    opt_vars.penaltyWeights.resize(7);
    opt_vars.magnitudeBounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
            cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
    opt_vars.penaltyWeights << cfg_.penna_pos, cfg_.penna_vel,
            cfg_.penna_acc, cfg_.penna_jerk,
            cfg_.penna_attract, cfg_.penna_omg,
            cfg_.penna_thr;
    opt_vars.rho = cfg_.penna_t;
    opt_vars.pos_constraint_type = cfg_.pos_constraint_type;
    opt_vars.block_energy_cost = cfg_.block_energy_cost;
    opt_vars.smooth_eps = cfg_.smooth_eps;
    opt_vars.integral_res = cfg_.integral_reso;
    opt_vars.quadrotor_flatness = cfg_.quadrotot_flatness;
    opt_vars.weight_ts = cfg_.penna_ts;
    opt_vars.uniform_time_en = cfg_.uniform_time_en;
    opt_vars.piece_num = cfg_.piece_num;
}

bool BackupTrajOpt::checkTrajMagnitudeBound(Trajectory &out_traj) {
    if (cfg_.penna_vel > 0 && out_traj.getMaxVelRate() > 1.2 * cfg_.max_vel) {
        std::cout << YELLOW << " -- [TrajOpt] Backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max vel:\t" << out_traj.getMaxVelRate() << " m/s" << RESET
                  << std::endl;
        return false;
    }
    if (cfg_.penna_acc > 0 && out_traj.getMaxAccRate() > 1.2 * cfg_.max_acc) {
        std::cout << YELLOW << " -- [TrajOpt] Backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max Acc:\t" << out_traj.getMaxAccRate() << " m/s" << RESET
                  << std::endl;
        return false;
    }
    return true;
}

bool
BackupTrajOpt::optimize(const Trajectory &exp_traj,
                        const double &t_0,
                        const double &t_e,
                        const double &heu_ts,
                        const VecDf &heu_end_pt,
                        double &heu_dur,
                        const Polytope &sfc,
                        Trajectory &out_traj,
                        double &out_ts,
                        const bool &debug) {
    opt_vars.hPolytope = sfc.GetPlanes();
    if (std::isnan(opt_vars.hPolytope.sum())) {
        std::cout << YELLOW << " -- [BackTrajOpt] Polytope is nan." << RESET << std::endl;
        return false;
    }

    opt_vars.debug_en = debug;
    /// Setup optimization problems
    opt_vars.default_init = true;
    opt_vars.given_init_ts_and_ps = false;
    opt_vars.headPVAJ = exp_traj.getState(heu_ts);
    opt_vars.tailPVAJ.setZero();
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.exp_traj = exp_traj;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.max_ts = t_e;
    opt_vars.min_ts = t_0;
    opt_vars.tailPVAJ.col(0) = heu_end_pt;
    opt_vars.times.resize(opt_vars.piece_num);
    opt_vars.times.setConstant(heu_dur / opt_vars.piece_num);
    opt_vars.ts = heu_ts;

    out_traj.clear();
    PolyhedronH planes = sfc.GetPlanes();
    bool success{true};

    if (!setupProblemAndCheck()) {
        std::cout << YELLOW << " -- [TrajOpt] Backup corridor preprocess error." << RESET << std::endl;
        success = false;
    }

    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
        std::cout << YELLOW << " -- [SUPER] Backup trajectory optimization failed." << RESET << std::endl;
        success = false;
    }

    if (opt_vars.penalty_log(1) > cfg_.penna_pos * 0.05) {
        std::cout << YELLOW << " -- [SUPER] Backup trajectory leaves corridor." << RESET << std::endl;
        success = false;
    }
    out_ts = opt_vars.ts;

    if (!checkTrajMagnitudeBound(out_traj)) {
        success = false;
    }

    if (!success && cfg_.save_log_en) {
        // log the optimization problem
        failed_traj_log << 123321 << std::endl;
        failed_traj_log << t_0 << std::endl;
        failed_traj_log << t_e << std::endl;
        failed_traj_log << heu_ts << std::endl;
        failed_traj_log << heu_end_pt.transpose() << std::endl;
        failed_traj_log << heu_dur << std::endl;
        failed_traj_log << 0 << std::endl;
        failed_traj_log << sfc.GetPlanes() << std::endl;
        failed_traj_log << exp_traj.getPieceNum() << std::endl;
        failed_traj_log << exp_traj.getDurations().transpose() << std::endl;
        for (int i = 0; i < exp_traj.getPieceNum(); i++) {
            failed_traj_log << exp_traj[i].getCoeffMat() << std::endl;
        }
        out_ts = heu_ts;
    }

    return success;
}

bool
BackupTrajOpt::optimize(const Trajectory &exp_traj,
                        const double &t_0,
                        const double &t_e,
                        const double &heu_ts,
                        const Polytope &sfc,
                        const VecDf & init_t_vec,
                        const vec_Vec3f &init_ps,
                        Trajectory &out_traj,
                        double & out_ts) {
    opt_vars.hPolytope = sfc.GetPlanes();
    if (std::isnan(opt_vars.hPolytope.sum())) {
        std::cout << YELLOW << " -- [BackTrajOpt] Polytope is nan." << RESET << std::endl;
        return false;
    }

    opt_vars.debug_en = false;
    /// Setup optimization problems
    opt_vars.default_init = true;

    opt_vars.headPVAJ = exp_traj.getState(heu_ts);
    opt_vars.tailPVAJ.setZero();
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.exp_traj = exp_traj;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.max_ts = t_e;
    opt_vars.min_ts = t_0;
    opt_vars.tailPVAJ.col(0) = init_ps.back();
    opt_vars.times.resize(opt_vars.piece_num);
    const double heu_dur = init_t_vec.sum();
    opt_vars.times.setConstant(heu_dur / opt_vars.piece_num);
    opt_vars.ts = heu_ts;

    opt_vars.given_init_ts_and_ps = true;
    opt_vars.given_init_t_vec = init_t_vec;
    opt_vars.given_init_ps = init_ps;
    opt_vars.given_init_ts = heu_ts;

    out_traj.clear();
    PolyhedronH planes = sfc.GetPlanes();
    bool success{true};

    if (!setupProblemAndCheck()) {
        std::cout << YELLOW << " -- [TrajOpt] Backup corridor preprocess error." << RESET << std::endl;
        success = false;
    }

    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
        std::cout << YELLOW << " -- [SUPER] Backup trajectory optimization failed." << RESET << std::endl;
        success = false;
    }

    if (opt_vars.penalty_log(1) > cfg_.penna_pos * 0.05) {
        std::cout << YELLOW << " -- [SUPER] Backup trajectory leaves corridor." << RESET << std::endl;
        success = false;
    }
    out_ts = opt_vars.ts;

    if (!checkTrajMagnitudeBound(out_traj)) {
        success = false;
    }


    return success;
}
