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

#include <traj_opt/exp_traj_optimizer_s4.h>
#include <utils/optimization/lbfgs.h>
#include <ros_interface/ros_interface.hpp>

using namespace traj_opt;
using namespace color_text;
using namespace super_utils;
using namespace math_utils;
using namespace optimization_utils;
static void truncateToSixDecimals(double &num) {
    num = std::trunc(num * 1e6) / 1e6; // 直接截断，无四舍五入
}

/*
 * @ brief: This function pre-process the corridor
 *
 */
bool ExpTrajOpt::processCorridor() {
    const long sizeCorridor = static_cast<long>(opt_vars.hPolytopes.size() - 1);

    opt_vars.vPolytopes.clear();
    opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

    long nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    opt_vars.waypoint_attractor.resize(3, sizeCorridor);
    opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
    opt_vars.hOverlapPolytopes.resize(sizeCorridor);

    for (long i = 0; i < sizeCorridor; i++) {
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
            cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs." << RESET
                 << endl;
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
        curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
        curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
        curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
        opt_vars.hOverlapPolytopes[i] = curIH;
        Vec3f interior;
        const double &dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
        opt_vars.waypoint_attractor.col(i) = curIV.colwise().mean();
        opt_vars.waypoint_attractor_dead_d(i) = dis;
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs." <<
             RESET << endl;
        return false;
    }

    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);
    return true;
}

bool ExpTrajOpt::processCorridorWithGuideTraj() {
    // * 1) allocate memory for vertex
    const int sizeCorridor = static_cast<int>(opt_vars.hPolytopes.size() - 1);

    opt_vars.vPolytopes.clear();
    opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

    long nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    opt_vars.waypoint_attractor.resize(3, sizeCorridor);
    opt_vars.hOverlapPolytopes.resize(sizeCorridor);
    opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
    // * 2) Process the corridor
    for (int i = 0; i < sizeCorridor; i++) {
        // * 2.1) Get current vertex
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
            cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs."
                 << RESET << endl;

            return false;
        }
        // * 2.3) Conver the vertex to the frame of the first point
        nv = curIV.cols();
        curIOB.resize(3, nv);
        // *    Save the position of the first point
        curIOB.col(0) = curIV.col(0);
        // *    Use the relative position of the rest vertex.
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        // *    save the i-th corridor's vertex
        opt_vars.vPolytopes.push_back(curIOB);

        // * 2.4) Find the overlap corridor
        curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
        curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
        curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
        opt_vars.hOverlapPolytopes[i] = curIH;
        Vec3f interior;

        const double dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
        if (dis < 0.0 || std::isinf(dis)) {

            cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed findInteriorDist Vs." <<
                 RESET << endl;
            return false;
        }
        geometry_utils::enumerateVs(curIH, interior, curIV);
        const double test_sum = curIV.sum();
        if (std::isnan(test_sum) || std::isinf(test_sum)) {
            return false;
        }
        opt_vars.waypoint_attractor.col(i) = interior;
        opt_vars.waypoint_attractor_dead_d(i) = dis;
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    // * 3) Time and waypoint allocation for hot initialization
    VecDf min_dis(opt_vars.waypoint_attractor.cols());
    VecDi min_id(opt_vars.waypoint_attractor.cols());
    VecDf time_stamps(opt_vars.waypoint_attractor.cols() + 2);
    time_stamps(0) = 0.0;
    time_stamps(opt_vars.waypoint_attractor.cols() + 1) = opt_vars.guide_t.back();
    min_id.setConstant(0);
    min_dis.setConstant(std::numeric_limits<double>::max());
    for (int i = 0; i < opt_vars.guide_path.size(); i++) {
        for (int j = 0; j < opt_vars.waypoint_attractor.cols(); j++) {
            const double dis = (opt_vars.guide_path[i] - opt_vars.waypoint_attractor.col(j)).norm();
            if (dis < min_dis[j]) {
                min_dis[j] = dis;
                min_id[j] = i;
                opt_vars.points.col(j) = opt_vars.waypoint_attractor.col(j);//opt_vars.guide_path[i];
                time_stamps(j + 1) = opt_vars.guide_t[i];
            }
        }
    }

    for (int i = 1; i < time_stamps.size(); i++) {
        opt_vars.times(i - 1) = time_stamps(i) - time_stamps(i - 1);
        opt_vars.times(i - 1) = std::max(0.01, opt_vars.times(i - 1));
    }

    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);
    return true;
}

void ExpTrajOpt::defaultInitialization() {
    const VecDf dis = (opt_vars.init_path.leftCols(opt_vars.piece_num) -
                       opt_vars.init_path.rightCols(opt_vars.piece_num)).colwise().norm();
    const double speed = cfg_.max_vel;
    opt_vars.times = dis / speed;
    opt_vars.points = opt_vars.waypoint_attractor;
}

bool ExpTrajOpt::setupProblemAndCheck() {
    // init internal variables size;
    opt_vars.piece_num = static_cast<int>(opt_vars.hPolytopes.size());
    opt_vars.times.resize(opt_vars.piece_num);
    opt_vars.points.resize(3, opt_vars.piece_num - 1);


    // Check corridor and init points
    if (opt_vars.default_init) {
        throw std::runtime_error("Not support default init in this version.");
        if (!processCorridor()) {
            return false;
        }
    } else {
        if (!processCorridorWithGuideTraj()) {
            return false;
        }
    }
    opt_vars.init_path.resize(3, opt_vars.piece_num + 1);
    for (long i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.init_path.col(i + 1) = opt_vars.waypoint_attractor.col(i);
    }
    opt_vars.init_path.col(0) = opt_vars.headPVAJ.col(0);
    opt_vars.init_path.rightCols(1) = opt_vars.tailPVAJ.col(0);
    if (opt_vars.default_init) {
        defaultInitialization();
    } else {
        opt_vars.times *= 0.8;
    }

    if (std::isnan(opt_vars.times.sum())) {
        cout << YELLOW << " -- [ExpOpt] Init times and point failed." << RESET << endl;
        return false;
    }

    const Mat3Df deltas = opt_vars.init_path.rightCols(opt_vars.piece_num)
                          - opt_vars.init_path.leftCols(opt_vars.piece_num);
    opt_vars.pieceIdx = (deltas.colwise().norm() / INFINITY).cast<int>().transpose();
    opt_vars.pieceIdx.array() += 1;

    opt_vars.temporalDim = opt_vars.piece_num;
    opt_vars.spatialDim = 0;
    opt_vars.vPolyIdx.resize(opt_vars.piece_num - 1);
    opt_vars.hPolyIdx.resize(opt_vars.piece_num);

    switch (cfg_.pos_constraint_type) {
        case 1: {
            for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                k = opt_vars.pieceIdx(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i;
                    } else if (i < opt_vars.piece_num - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i + 1;
                    }
                    opt_vars.hPolyIdx(j) = i;
                }
            }
            opt_vars.spatialDim = 3 * (opt_vars.piece_num - 1);
            break;
        }
        default: {
            for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                k = opt_vars.pieceIdx(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i;
                        opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i].cols());
                    } else if (i < opt_vars.piece_num - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i + 1;
                        opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i + 1].cols());
                    }
                    opt_vars.hPolyIdx(j) = i;
                }
            }
        }
    }

    return true;
}

bool ExpTrajOpt::setInitPsAndTs(const vec_Vec3f &init_ps, const vector<double> &init_ts) {
    opt_vars.default_init = false;
    if (opt_vars.times.size() != init_ts.size()) {
        return false;
    }
    if (opt_vars.points.cols() != init_ps.size()) {
        return false;
    }

    for (long i = 0; i < opt_vars.points.cols(); i++) {
        opt_vars.times[i] = init_ts[i];
        opt_vars.points.col(i) = init_ps[i];
    }
    opt_vars.times[opt_vars.times.size() - 1] = init_ts.back();
    return true;
}

bool ExpTrajOpt::configureSplineProblem() {
    time_cost_.weight = opt_vars.rho;
    spatial_map_.reset(&opt_vars.vPolytopes,
                       &opt_vars.vPolyIdx,
                       opt_vars.piece_num,
                       opt_vars.pos_constraint_type == 1);
    integral_cost_.reset(&opt_vars.hPolytopes,
                         &opt_vars.hPolyIdx,
                         &opt_vars.waypoint_attractor,
                         &opt_vars.waypoint_attractor_dead_d,
                         opt_vars.smooth_eps,
                         opt_vars.magnitudeBounds,
                         opt_vars.penaltyWeights,
                         &opt_vars.quadrotor_flatness);

    optimizer_.setSpatialMap(&spatial_map_);
    optimizer_.setAuxiliaryStateMap(nullptr);
    optimizer_.setEnergyWeights(opt_vars.block_energy_cost ? 0.0 : 1.0);
    const auto integral_steps_status = optimizer_.setIntegralNumSteps(opt_vars.integral_res);
    if (!integral_steps_status) {
        return false;
    }

    spline_opt::WaypointsType waypoints(opt_vars.piece_num + 1, 3);
    waypoints.row(0) = opt_vars.headPVAJ.col(0).transpose();
    for (int i = 0; i < opt_vars.piece_num - 1; ++i) {
        waypoints.row(i + 1) = opt_vars.points.col(i).transpose();
    }
    waypoints.row(opt_vars.piece_num) = opt_vars.tailPVAJ.col(0).transpose();

    SplineTrajectory::BoundaryConditions<3> bc;
    bc.start_velocity = opt_vars.headPVAJ.col(1);
    bc.start_acceleration = opt_vars.headPVAJ.col(2);
    bc.start_jerk = opt_vars.headPVAJ.col(3);
    bc.end_velocity = opt_vars.tailPVAJ.col(1);
    bc.end_acceleration = opt_vars.tailPVAJ.col(2);
    bc.end_jerk = opt_vars.tailPVAJ.col(3);

    std::vector<double> time_segments(opt_vars.times.data(), opt_vars.times.data() + opt_vars.times.size());
    const auto init_status = optimizer_.setInitState(time_segments, waypoints, 0.0, bc);
    if (!init_status.ok) {
        return false;
    }
    return true;
}

double ExpTrajOpt::evaluateCurrentSplineCost(const VecDf &vars, VecDf &grad) {
    ++opt_vars.iter_num;
    std::vector<double> eval_times(opt_vars.temporalDim);
    SplineTrajectory::QuadInvTimeMap time_map;
    for (int i = 0; i < opt_vars.temporalDim; ++i) {
        eval_times[i] = time_map.toTime(vars(i));
    }
    integral_cost_.beginEvaluation(&eval_times);
    const auto eval_spec = Optimizer::makeEvaluateSpec(time_cost_, integral_cost_, spline_workspace_);
    const auto eval_result = optimizer_.evaluate(vars, grad, eval_spec);
    if (!eval_result) {
        return INFINITY;
    }
    double cost = eval_result.cost;
    spatial_map_.addNormPenalty(vars, opt_vars.temporalDim, opt_vars.spatialDim, grad, cost);
    opt_vars.penalty_log = integral_cost_.getPenaltyLog();
    return cost;
}

double ExpTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();

    if (opt_vars.times.minCoeff() < 1e-3) {
        cout << YELLOW << " -- [TrajOpt] Error, the init times have zero, force return." << RESET << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.headPVAJ << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.tailPVAJ << endl;
        cout << " -- Times: " << endl;
        cout << opt_vars.times.transpose() << endl;
        return INFINITY;
    }

    if (opt_vars.given_init_ts_and_ps) {
        opt_vars.times = opt_vars.init_ts;
        for (int i = 0; i < opt_vars.init_ps.size(); i++) {
            opt_vars.points.col(i) = opt_vars.init_ps[i];
        }
    }

    if (!configureSplineProblem()) {
        return INFINITY;
    }

    VecDf x = optimizer_.generateInitialGuess();

    opt_vars.init_ts = opt_vars.times;
    opt_vars.init_ps.clear();
    for (int col = 0; col < opt_vars.points.cols(); col++) {
        opt_vars.init_ps.emplace_back(opt_vars.points.col(col));
    }

    opt_vars.iter_num = 0;
    double minCostFunctional{0};
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = relCostTol;

    for (int i = 0; i < opt_vars.waypoint_attractor_dead_d.size(); i++) {
        truncateToSixDecimals(opt_vars.waypoint_attractor_dead_d(i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(0, i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(1, i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(2, i));
    }

    cout << std::fixed << std::setprecision(15);
    opt_vars.iter_num = 0;
    const auto evaluator = [](void *ptr, const VecDf &vars, VecDf &grad) -> double {
        return static_cast<ExpTrajOpt *>(ptr)->evaluateCurrentSplineCost(vars, grad);
    };

    int ret = lbfgs::lbfgs_optimize(x,
                                    minCostFunctional,
                                    evaluator,
                                    nullptr,
                                    nullptr,
                                    this,
                                    lbfgs_params);

    const int optimizer_iters = opt_vars.iter_num;
    VecDf grad(x.size());
    minCostFunctional = evaluateCurrentSplineCost(x, grad);
    opt_vars.iter_num = optimizer_iters;
    const SplineType *optimal_spline = &optimizer_.getWorkingSpline(spline_workspace_);
    if (optimal_spline != nullptr) {
        opt_vars.times.resize(optimal_spline->getTrajectory().getNumSegments());
        for (int i = 0; i < opt_vars.times.size(); ++i) {
            opt_vars.times(i) = optimal_spline->getTrajectory()[i].duration();
        }
    }
    opt_vars.penalty_log(0) = (!opt_vars.block_energy_cost && optimal_spline != nullptr) ? optimal_spline->getEnergy() : 0.0;

    if (cfg_.print_optimizer_log) {
        cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
    }

    if ((cfg_.penna_pos > 0 && opt_vars.penalty_log(1) > 0.2) ||
        // (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin) ||
        (cfg_.penna_acc > 0 && opt_vars.penalty_log(3) > cfg_.max_acc * cfg_.penna_margin) ||
        (cfg_.penna_omg > 0 && opt_vars.penalty_log(6) > cfg_.max_omg * cfg_.penna_margin) ||
        (cfg_.penna_thr > 0 && opt_vars.penalty_log(7) > cfg_.max_acc * cfg_.penna_margin)) {
        if (cfg_.print_optimizer_log) {
            cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
            cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
            cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
            cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
            cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
            cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
            cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
            cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
            cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
            cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
        }
        ros_ptr_->warn(" -- [ExpOpt] Opt failed, Omg or thr or Pos violation.");
        ret = -1;
    }

    if (ret >= 0) {
        if (optimal_spline != nullptr) {
            traj = spline_opt::splineToSuperTrajectory(*optimal_spline);
            opt_vars.points.resize(3, std::max(0, opt_vars.piece_num - 1));
            for (int i = 0; i < opt_vars.points.cols(); ++i) {
                opt_vars.points.col(i) = traj.getJuncPos(i + 1);
            }
        } else {
            traj.clear();
            minCostFunctional = INFINITY;
        }
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        cout << YELLOW << " -- [SUPER] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << endl;
    }
    return minCostFunctional + ret;
}

ExpTrajOpt::ExpTrajOpt(const traj_opt::Config &cfg, const ros_interface::RosInterface::Ptr &ros_ptr) :
        cfg_(cfg),
        ros_ptr_(ros_ptr) {
    /// Use time as log file name
    //    auto now = std::chrono::system_clock::now();
    //    std::time_t t = std::chrono::system_clock::to_time_t(now);
    //    std::tm tm = *std::localtime(&t);
    //    std::stringstream ss;
    //    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    //    std::string filename = ss.str() + "_exp_opt_log.csv";
    if(cfg_.save_log_en){
        std::string filename = "exp_opt_log.csv";
        failed_traj_log.open(DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
        penalty_log.open(DEBUG_FILE_DIR("exp_opt_penna.csv"), std::ios::out | std::ios::trunc);
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
}

ExpTrajOpt::~ExpTrajOpt() {
    failed_traj_log.close();
    penalty_log.close();
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj) {
    vec_E<Vec3f> guide_path;
    guide_path.emplace_back(headPVAJ.col(0));
    guide_path.emplace_back(tailPVAJ.col(0));

    const double nominal_speed = std::max(cfg_.max_vel, 0.1);
    const double nominal_time = std::max(0.01, (tailPVAJ.col(0) - headPVAJ.col(0)).norm() / nominal_speed);
    vector<double> guide_t{0.0, nominal_time};
    return optimize(headPVAJ, tailPVAJ, guide_path, guide_t, sfcs, out_traj);
}


//bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
//                          PolytopeVec &sfcs,
//                          Trajectory &out_traj) {
//    /// Check if SFC is valid
//    if (sfcs.empty()) {
//        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
//        return false;
//    }
//
//    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
//        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
//        //        VisualUtils::VisualizePoint(mkr_pub_, headPVAJ.col(0),Color::Pink(),"ill_start",0.5,1);
//        //        VisualUtils::VisualizePoint(mkr_pub_, tailPVAJ.col(0),Color::Pink(),"ill_end",0.5,2);
//        //        cout << "headPVAJ: " << headPVAJ.col(0).transpose() << endl;
//        //        cout << "tailPVAJ: " << tailPVAJ.col(0).transpose() << endl;
//        //        cout << YELLOW << "Killing the node." << RESET << endl;
//        //        exit(-1);
//        return false;
//    }
//
//    for (const auto &poly: sfcs) {
//        if (std::isnan(poly.GetPlanes().sum())) {
//            cout << YELLOW << " -- [TrajOpt] Error, the SFC containes NaN." << RESET << endl;
//            return false;
//        }
//    }
//
//    bool success{true};
//
//    /// Setup optimization problems
//    opt_vars.default_init = true;
//    opt_vars.given_init_ts_and_ps = false;
//    opt_vars.headPVAJ = headPVAJ;
//    opt_vars.tailPVAJ = tailPVAJ;
//    opt_vars.guide_path.clear();
//    opt_vars.guide_t.clear();
//    opt_vars.hPolytopes.resize(sfcs.size());
//    for (long i = 0; i < sfcs.size(); i++) {
//        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
//    }
//
//    if (!setupProblemAndCheck()) {
//        cout << YELLOW << " -- [SUPER] Minco corridor preprocess error." << RESET << endl;
//        success = false;
//    }
//
//    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
//        std::cout << YELLOW << " -- [SUPER] in [ExpTrajOpt::optimize]: Optimization failed." << RESET << std::endl;
//        success = false;
//    }
//
//    if(success){
//        out_traj.start_WT = ros_ptr_->getSimTime();
//    }
//
//    if (!success && cfg_.save_log_en) {
//        failed_traj_log << 990419 << endl;
//        failed_traj_log << headPVAJ << endl;
//        failed_traj_log << tailPVAJ << endl;
//        for (long i = 0; i < sfcs.size(); i++) {
//            failed_traj_log << i << endl;
//            failed_traj_log << sfcs[i].GetPlanes() << endl;
//        }
//    }
//
//    return success;
//}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj) {
    /// Check if hot init is valid
    if (guide_path.size() != guide_t.size()) {
        cout << YELLOW << " -- [TrajOpt] Error, the guide trajectory has wrong path and time stamp." << RESET
             << endl;
        return false;
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
        return false;
    }

    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = false;
    opt_vars.given_init_ts_and_ps = false;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.hPolytopes.resize(sfcs.size());

    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        const Eigen::ArrayXd norms = opt_vars.hPolytopes[i].leftCols<3>().rowwise().norm();
        opt_vars.hPolytopes[i].array().colwise() /= norms;
    }

    if (!setupProblemAndCheck()) {
        cout << YELLOW << " -- [SUPER] Exp trajectory corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();


    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
        cout << YELLOW << " -- [SUPER] Exp trajectory optimization failed." << RESET << endl;
        success = false;
    }

    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        out_traj.start_WT = ros_ptr_->getSimTime();
    }

    if (!success && cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (double i: guide_t) {
            failed_traj_log << i << " ";
        }
        failed_traj_log << endl;
        for (const auto &i: guide_path) {
            failed_traj_log << i.transpose() << " ";
        }
        failed_traj_log << endl;
        for (long i = 0; i < sfcs.size(); i++) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          const vec_Vec3f &init_ps,
                          const VecDf &init_ts,
                          Trajectory &out_traj) {
    vec_Vec3f guide_path;
    guide_path.emplace_back(headPVAJ.col(0));
    for (const auto &i: init_ps) {
        guide_path.emplace_back(i);
    }
    guide_path.emplace_back(tailPVAJ.col(0));
    vector<double> guide_t;
    guide_t.emplace_back(0);
    double accumulate_t = 0;
    for (int i = 0; i < init_ts.size(); i++) {
        accumulate_t += init_ts[i];
        guide_t.emplace_back(accumulate_t);
    }
    /// Check if hot init is valid
    if (guide_path.size() != guide_t.size()) {
        cout << YELLOW << " -- [TrajOpt] Error, the guide trajectory has wrong path and time stamp." << RESET
             << endl;
        return false;
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
        return false;
    }

    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = false;
    opt_vars.given_init_ts_and_ps = true;
    opt_vars.init_ts = init_ts;
    opt_vars.init_ps = init_ps;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.hPolytopes.resize(sfcs.size());

    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        const Eigen::ArrayXd norms = opt_vars.hPolytopes[i].leftCols<3>().rowwise().norm();
        opt_vars.hPolytopes[i].array().colwise() /= norms;
    }

    if (!setupProblemAndCheck()) {
        cout << YELLOW << " -- [SUPER] Exp trajectory corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();

    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
        cout << YELLOW << " -- [SUPER] Exp trajectory optimization failed." << RESET << endl;
        success = false;
    }
    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        out_traj.start_WT = ros_ptr_->getSimTime();
    }


    if (!success && cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (double i: guide_t) {
            failed_traj_log << i << " ";
        }
        failed_traj_log << endl;
        for (const auto &i: guide_path) {
            failed_traj_log << i.transpose() << " ";
        }
        failed_traj_log << endl;
        for (long i = 0; i < sfcs.size(); i++) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}
