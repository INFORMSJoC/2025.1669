// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include "gurobi_c++.h"
#include "datamngr.hpp"
#include <map>
#include <tuple>
#include <unordered_map>

class Leg;
class Aircraft;
class Slot;

namespace ColGen {
   class Solver;
}
namespace Continuous {
   class Solver;
}

namespace CGCG {
   class Solver;
}

namespace CCG {
   class Solver;
}

namespace Timespace {
   class SubNetwork;
   class FlightArc;
};

class PathModel {
protected:
   GRBModel* master_mod;
   GRBLinExpr master_obj;

   std::vector<std::shared_ptr<GRBVar>> var_assign;		// whether assign an aircraft to a flight
   GRBVar* var_cancel;
   GRBVar* var_miss_position;        // whether the aircraft family is missing at the end of recovery period in each airport
   
   GRBConstr* path_select;
   GRBConstr* flight_cover;
   GRBConstr* flow_control;
   GRBConstr* position_req;          // aircraft position requirements

   std::vector<double> dual_path_select;
   std::vector<double> dual_flight_cover;
   std::vector<double> dual_flow_control;
   std::vector<double> dual_position_req;

   int num_paths;
   std::unordered_map<const Aircraft*, int> aircraft_indices;
   std::unordered_map<const Leg*, int> leg_indices;
   std::unordered_map<const Slot*, int> slot_indices;
   using PositionKey = std::tuple<const Station*, PositionType, const AircraftFamily*, const AircraftModel*>;
   std::map<PositionKey, int> position_indices;
   std::vector<std::vector<int>> leg_slots;
   std::vector<std::vector<ptime>> propagation_times;

public:
   PathModel() : num_paths(0), master_obj(0), master_mod(nullptr),
      var_cancel{ nullptr }, var_miss_position{ nullptr },
      path_select{ nullptr }, flight_cover{ nullptr }, flow_control{ nullptr },
      position_req{ nullptr }
   {
      dual_path_select.clear();
      dual_flight_cover.clear();
      dual_flow_control.clear();
      dual_position_req.clear();
   };

   PathModel(GRBEnv* env) : num_paths(0), master_obj(0), master_mod(nullptr),
      var_cancel{ nullptr }, var_miss_position{ nullptr },
      path_select{ nullptr }, flight_cover{ nullptr }, flow_control{ nullptr },
      position_req{ nullptr }
   {
      master_mod = new GRBModel(*env);
      dual_path_select.clear();
      dual_flight_cover.clear();
      dual_flow_control.clear();
      dual_position_req.clear();
   };

   virtual void create_master_problem();
   virtual ~PathModel();
   PathModel(const PathModel&) = delete;
   PathModel& operator=(const PathModel&) = delete;

   // Input-dependent caches are built once, before any parallel pricing work.
   void initialize_indices();

   virtual void solve(bool as_ilp = false);

   double get_new_copy_rd_cost(const Aircraft* aircraft, const LegCopy* copy) const;
   const std::vector<int>& get_relevant_slots(const Leg* leg) const { return leg_slots.at(get_index(leg)); }
   const std::vector<ptime>& get_propagation_times(const Leg* leg) const { return propagation_times.at(get_index(leg)); }
   void initialize_arc_cost(const Aircraft* aircraft, Timespace::FlightArc* arc) const;
   void update_arc_cost(Timespace::FlightArc* arc) const;
   double adjust_slot_score(double score, const LegCopy* original, const LegCopy* adjusted) const;

   int get_index(const Aircraft* a) const { return aircraft_indices.at(a); }
   int get_index(const Slot* s) const { return slot_indices.at(s); }
   int get_index(const Leg* l) const { return leg_indices.at(l); }
   int get_index(const Station* s, PositionType t, const AircraftFamily* f=nullptr,
      const AircraftModel* m=nullptr) const {
      const auto it = position_indices.find({s, t, f, m});
      return it == position_indices.end() ? -1 : it->second;
   }

   friend class Continuous::Solver;
   friend class ColGen::Solver;
   friend class CGCG::Solver;
   friend class CCG::Solver;
   friend class Timespace::SubNetwork;
};
