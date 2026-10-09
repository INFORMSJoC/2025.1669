// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include<boost/date_time/posix_time/posix_time.hpp>
#include <Flight.hpp>
#include <Aircraft.hpp>
#include <gurobi_c++.h>

class SolRegistry
{
private:
   static SolRegistry* sol_instance;
   SolRegistry();

   ~SolRegistry() = default;

public:
   static SolRegistry* instance() {
      if (!sol_instance) {
         sol_instance = new SolRegistry();
      }
      return sol_instance;
   }

   // store solutions
   std::map<size_t, std::pair<Aircraft*, boost::posix_time::time_duration> > map_leg_id_to_assignment;        // solution w.r.t. each leg (id: aircraft, delay)
   std::vector<Leg* > unassigned_legs;
   std::vector<Leg* > unassigned_mtcs;
   std::map<size_t, std::vector<std::pair<Leg*, boost::posix_time::time_duration>> > map_aircraft_id_to_route;   // route for each aircraft
   std::map<size_t, boost::posix_time::time_duration> accelerated_legs; // legs with cruise speed control (id: time reduced)
   std::vector<int> position_violations; // list of position violations 

   GRBEnv* env;
   double objective, lp_objective;
   double gap;
   boost::posix_time::time_duration solve_time;
   // Seconds. Label/network times sum work across threads and overlap generation times.
   double copyGenTime, colGenTime, solveMpTime, solveSppTime, updateNetworkTime;
   double solveIpTime = 0;
   size_t labels_considered = 0, propagation_leaves = 0, network_nodes = 0;
   size_t minlp_solves = 0;

   void clear()
   {
      map_leg_id_to_assignment.clear();
      map_aircraft_id_to_route.clear();
      objective = 0.0;
      solve_time = boost::posix_time::time_duration(0, 0, 0);
   }
};


