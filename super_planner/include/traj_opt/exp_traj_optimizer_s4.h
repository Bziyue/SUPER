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

#ifndef SUPER_EXP_TRAJ_OPT_H
#define SUPER_EXP_TRAJ_OPT_H

#include <iostream>
#include <vector>

#include <traj_opt/config.hpp>
#include <traj_opt/spline_components.hpp>


#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>

#include <utils/header/scope_timer.hpp>
#include <utils/header/type_utils.hpp>
#include <utils/optimization/optimization_utils.h>
#include <utils/geometry/geometry_utils.h>

#include <ros_interface/ros_interface.hpp>

namespace traj_opt {

    using geometry_utils::PolytopeVec;
    using geometry_utils::Trajectory;
    using super_utils::MatD3f;
    using super_utils::Mat3Df;
    using super_utils::StatePVAJ;
    using super_utils::VecDi;
    using super_utils::Vec3f;
    using super_utils::VecDf;
    using super_utils::PolyhedraH;
    using super_utils::PolyhedraV;
    using super_utils::vec_E;
    using super_utils::vec_Vec3f;


    class ExpTrajOpt {
        using SplineType = spline_opt::SepticSpline;
        using SpatialMap = spline_opt::PolytopeSpatialMap;
        struct Parameterization
        {
            SplineTrajectory::QuadInvTimeMap time;
            const SpatialMap &space;
            SplineTrajectory::NoAuxiliaryMap auxiliary;
        };
        using Optimizer = SplineTrajectory::SplineOptimizer<SplineType, Parameterization>;
        struct Objective
        {
            const spline_opt::LinearTimeCost &duration;
            const spline_opt::ExpPenaltyIntegralCost &integral;
            spline_opt::DomainCost decision;
        };

        traj_opt::Config cfg_;
        std::ofstream failed_traj_log;
        std::ofstream penalty_log;
        ros_interface::RosInterface::Ptr ros_ptr_;
        SpatialMap spatial_map_;
        Optimizer optimizer_{Parameterization{{}, spatial_map_, {}}};
        spline_opt::LinearTimeCost time_cost_;
        spline_opt::ExpPenaltyIntegralCost integral_cost_;

        struct OptimizationVariables {
            double rho;
            int iter_num{0};
            int pos_constraint_type;
            bool block_energy_cost;
            double smooth_eps;
            int integral_res;
            flatness::FlatnessMap quadrotor_flatness;

            bool default_init{true};
            bool given_init_ts_and_ps{false};
            int piece_num;
            Mat3Df points;
            VecDf times;
            VecDf magnitudeBounds, penaltyWeights;

            PolyhedraV vPolytopes; // the original sfc and intersecting sfc
            PolyhedraH hPolytopes; // the original sfc
            PolyhedraH hOverlapPolytopes;
            Mat3Df init_path;
            VecDf init_ts;
            vec_Vec3f init_ps;
            Mat3Df waypoint_attractor;
            VecDf waypoint_attractor_dead_d;

            VecDi vPolyIdx;
            VecDi hPolyIdx;

            StatePVAJ headPVAJ;
            StatePVAJ tailPVAJ;
            vec_E<Vec3f> guide_path;
            vector<double> guide_t;


            VecDf penalty_log;
        } opt_vars;

        bool processCorridor();

        bool processCorridorWithGuideTraj();

        bool configureSplineProblem();

        double evaluateCurrentSplineCost(const VecDf &vars, VecDf &grad);

        void defaultInitialization();

        bool setupProblemAndCheck();





        bool setInitPsAndTs(const vec_Vec3f &init_ps, const vector<double> &init_ts);

        double optimize(Trajectory &traj, const double &relCostTol);

    public:
        typedef std::shared_ptr<ExpTrajOpt> Ptr;

        ExpTrajOpt(const traj_opt::Config &cfg, const ros_interface::RosInterface::Ptr & ros_ptr);

        ~ExpTrajOpt();

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        void getInitValue(VecDf &ts, vec_Vec3f &ps) const {
            ts = opt_vars.init_ts;
            ps = opt_vars.init_ps;
        }

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      const vec_Vec3f & init_ps,
                      const VecDf & init_ts,
                      Trajectory &out_traj);

    };
}

#endif
