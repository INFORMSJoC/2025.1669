// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "PathModel.h"
#include "solmngr.h"
#include "timespace.hpp"

PathModel::~PathModel()
{
   delete[] var_cancel;
   delete[] var_miss_position;
   delete[] path_select;
   delete[] flight_cover;
   delete[] flow_control;
   delete[] position_req;
   delete master_mod;
}

void PathModel::initialize_indices()
{
   const auto data = DataRegistry::instance();
   aircraft_indices.clear();
   leg_indices.clear();
   slot_indices.clear();
   position_indices.clear();
   for (int i = 0; i < data->aircrafts.size(); ++i) aircraft_indices.emplace(data->aircrafts[i].get(), i);
   for (int i = 0; i < data->legs.size(); ++i) leg_indices.emplace(data->legs[i], i);
   for (int i = 0; i < data->slots.size(); ++i) slot_indices.emplace(data->slots[i].get(), i);
   for (int i = 0; i < data->positions.size(); ++i) {
      const auto& p = data->positions[i];
      // emplace preserves the first matching constraint for wildcard queries.
      for (const auto family : {static_cast<AircraftFamily*>(nullptr), p->get_family()}) {
         for (const auto model : {static_cast<AircraftModel*>(nullptr), p->get_model()}) {
            position_indices.emplace(PositionKey{p->get_station(), p->get_type(), family, model}, i);
         }
      }
   }
   leg_slots.assign(data->legs.size(), {});
   propagation_times.assign(data->legs.size(), {});
   for (int i = 0; i < data->legs.size(); ++i) {
      const auto leg = data->legs[i];
      for (int j = 0; j < data->slots.size(); ++j) {
         const auto& slot = data->slots[j];
         const auto type = slot->get_slot_type();
         const bool departure = type != SlotType::Arrival && slot->get_station() == leg->get_origin();
         const bool arrival = type != SlotType::Departure && slot->get_station() == leg->get_destination();
         if (!departure && !arrival) continue;
         leg_slots[i].push_back(j); // Preserve global slot order and count mixture slots once.
         propagation_times[i].push_back(departure ? slot->get_slot_interval().second :
            slot->get_slot_interval().second - leg->get_duration());
      }
      auto& times = propagation_times[i];
      std::sort(times.begin(), times.end());
      times.erase(std::unique(times.begin(), times.end()), times.end());
   }
}

void PathModel::initialize_arc_cost(const Aircraft* aircraft, Timespace::FlightArc* arc) const
{
   arc->assignment_cost = aircraft->get_assignment_cost(arc->get_leg_copy());
   arc->leg_index = get_index(arc->get_leg());
   arc->slot_indices.clear();
   const auto data = DataRegistry::instance();
   for (int i : get_relevant_slots(arc->get_leg())) {
      if (arc->is_in_slot(data->slots[i].get())) arc->slot_indices.push_back(i);
   }
   update_arc_cost(arc);
}

void PathModel::update_arc_cost(Timespace::FlightArc* arc) const
{
   double rc = arc->assignment_cost - dual_flight_cover[arc->leg_index];
   arc->set_cost_exclude_slots(rc);
   for (int i : arc->slot_indices) rc -= dual_flow_control[i];
   arc->set_cost(rc);
}

double PathModel::adjust_slot_score(double score, const LegCopy* original, const LegCopy* adjusted) const
{
   const auto data = DataRegistry::instance();
   for (int i : get_relevant_slots(original->get_leg())) {
      if (original->is_in_slot(data->slots[i].get())) score += dual_flow_control[i];
      if (adjusted->is_in_slot(data->slots[i].get())) score -= dual_flow_control[i];
   }
   return score;
}

void PathModel::create_master_problem()
{
   initialize_indices();
   const auto data_reg = DataRegistry::instance();
   const auto config = Config::instance();
   const auto& sch_legs = DataRegistry::instance()->legs;
   const auto& slots = DataRegistry::instance()->slots;
   const int num_legs = static_cast<int>(sch_legs.size());
   const int num_slots = static_cast<int>(slots.size());
   const int num_aircraft = static_cast<int>(DataRegistry::instance()->aircrafts.size());
   const auto num_positions = static_cast<int>(data_reg->positions.size());

   try
   {
      master_mod = new GRBModel(SolRegistry::instance()->env);
      // Variables
      var_cancel = master_mod->addVars(num_legs, GRB_CONTINUOUS);
      for (int i = 0; i < num_legs; i++) {
         var_cancel[i].set(GRB_StringAttr_VarName, "Cancel(" + data_reg->legs[i]->to_simple_string() + ")");
      }

      var_miss_position = master_mod->addVars(num_positions, GRB_CONTINUOUS);
      for (int i = 0; i < data_reg->positions.size(); i++) {
         const auto& pos = data_reg->positions[i];
         var_miss_position[i].set(GRB_StringAttr_VarName, "MissPos[" + std::to_string(i) + "]");
      }

      // Objective
      master_obj = 0;
      for (int i = 0; i < num_legs; i++)
      {
         double cost_cancel = (sch_legs[i]->is_maintenance() ? Config::instance()->rule_config.cost_mtc_cancel :
            Config::instance()->rule_config.cost_flt_cancel);
         if (sch_legs[i]->is_maintenance() && sch_legs[i]->is_aog_maintenance())
         {
            cost_cancel = Config::instance()->alg_config.big_m;
         }
         master_obj += var_cancel[i] * cost_cancel;
      }

      // cost of end airport violation
      for (int i = 0; i < data_reg->positions.size(); i++) {
         const auto &pos = data_reg->positions[i];
         switch (pos->get_type())
         {
         case PositionType::Family:
            master_obj += var_miss_position[i] * config->rule_config.cost_pos_family;
            break;
         case PositionType::Model:
            master_obj += var_miss_position[i] * config->rule_config.cost_pos_model;
            break;
         }
      }

      master_mod->setObjective(master_obj, GRB_MINIMIZE);

      // Constraints
      path_select = new GRBConstr[num_aircraft];
      for (int i = 0; i < num_aircraft; i++)
      {
         path_select[i] = master_mod->addConstr(GRBLinExpr() == 1, "path_select(" + std::to_string(data_reg->aircrafts[i]->get_id()) + ")");
      }

      flight_cover = new GRBConstr[num_legs];
      for (int i = 0; i < num_legs; i++)
      {
         flight_cover[i] = master_mod->addConstr(var_cancel[i] == 1, "FltCover(" + sch_legs[i]->to_simple_string() + ")");
      }

      if (num_slots)
      {
         flow_control = new GRBConstr[num_slots];
         for (int i = 0; i < slots.size(); i++)
         {
            double rhs = slots[i]->get_limit();
            for (const auto& leg : data_reg->all_legs) {
               if (leg_indices.contains(leg.get())) {
                  continue;
               }
               if ((rhs > 0) && leg->is_in_slot(slots[i].get())) {
                  rhs -= 1;
               }
            }
            flow_control[i] = master_mod->addConstr(GRBLinExpr() <= rhs, "Slot(" + std::to_string(slots[i]->get_id()) + ")");
         }
      }

      if (num_positions) {
         position_req = new GRBConstr[num_positions];
         for (int i = 0; i < num_positions; i++) {
            const auto& pos = data_reg->positions[i];
            position_req[i] = master_mod->addConstr(var_miss_position[i] >= pos->get_desired_count(), pos->to_string());
         }
      }

      // Arrays to store dual solutions
      dual_path_select = std::vector<double>(num_aircraft, 0);
      dual_flight_cover = std::vector<double>(sch_legs.size(), 0);
      dual_flow_control = std::vector<double>(slots.size(), 0);
      dual_position_req = std::vector<double>(num_positions, 0);

      master_mod->update();

      // parameter settings
      master_mod->set(GRB_IntParam_OutputFlag, 0);
      master_mod->set(GRB_IntParam_Method, config->alg_config.lp_method);
      master_mod->set(GRB_IntParam_Crossover, config->alg_config.lp_method == GRB_METHOD_BARRIER ? 0 : -1);
   }
   catch (const GRBException& e)
   {
      std::cerr << "Exception caught: " << e.getMessage() << std::endl;
   }
   catch (...)
   {
      std::cerr << "Unknown exception caught!" << std::endl;
   }
}

void PathModel::solve(bool as_ilp)
{
   if (as_ilp) {
      for (int i = 0; i < num_paths; i++) {
         var_assign[i]->set(GRB_CharAttr_VType, GRB_BINARY);
      }
      master_mod->set(GRB_IntParam_Method, GRB_METHOD_AUTO);
      master_mod->set(GRB_IntParam_Crossover, -1);
      master_mod->set(GRB_IntParam_OutputFlag, Config::instance()->alg_config.print_solver_output);
      master_mod->set(GRB_DoubleParam_MIPGap, Config::instance()->alg_config.gap_tolerance);
      master_mod->set(GRB_DoubleParam_TimeLimit, Config::instance()->alg_config.max_IP_run_time * 60);
   }
   else {
      const int method = Config::instance()->alg_config.lp_method;
      master_mod->set(GRB_IntParam_Method, method);
      master_mod->set(GRB_IntParam_Crossover, method == GRB_METHOD_BARRIER ? 0 : -1);
      master_mod->set(GRB_IntParam_OutputFlag, 0);
   }
   master_mod->update();
   if (as_ilp && Config::instance()->alg_config.write_lp_to_file)
   {
      master_mod->write(Config::instance()->main_config.path_to_output + "MP-final.lp");
   }
   master_mod->optimize();

   if (!as_ilp && master_mod->get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
      master_mod->set(GRB_IntParam_Crossover, -1);
      master_mod->optimize();
   }
}

double PathModel::get_new_copy_rd_cost(const Aircraft* aircraft, const LegCopy* copy) const
{
   double rc = aircraft->get_assignment_cost(copy);

   if (this->dual_flight_cover.size()) {
      rc -= this->dual_flight_cover[this->get_index(copy->get_leg())];
   }

   if (this->dual_flow_control.size()) {
      for (int i : get_relevant_slots(copy->get_leg())) {
         if (copy->is_in_slot(DataRegistry::instance()->slots[i].get())) {
            rc -= this->dual_flow_control[i];
         }
      }
   }

   return rc;
}
