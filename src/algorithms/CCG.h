// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include<stdio.h>
#include<string>
#include<ctime>
#include<vector>
#include<map>
#include <unordered_set>

#include "datamngr.hpp"
#include "Flight.hpp"
#include "Slot.hpp"
#include "timespace.hpp"
#include "PathModel.h"

class DataManager;

namespace CCG
{
   class Path
   {
   private:
      Aircraft* aircraft;
      double cost;
      double reduced_cost;
      int path_id;

   public:
      std::vector<Timespace::FlightArc*> arc_list;

      Path() : aircraft(nullptr), cost(0), reduced_cost(0), path_id(-1)
      {
         arc_list.clear();
      };

      Path(Aircraft* _a, double _c, double _d) : aircraft(_a), cost(_c), reduced_cost(_d), path_id(-1)
      {
         arc_list.clear();
      };

      void set_id(int _id) { path_id = _id; }
      int get_id() const { return path_id; }

      void add_arc(Timespace::FlightArc* _arc, bool prep = false) {
         if (prep) {
            arc_list.insert(arc_list.begin(), _arc);
         }
         else {
            arc_list.push_back(_arc);
         }
      };

      std::vector<Leg* > get_legs() const {
         std::vector<Leg* > legs;
         for (auto arc : arc_list) {
            legs.push_back(arc->get_leg());
         }
         return legs;
      }

      int get_num_flights() const {
         return static_cast<int>(arc_list.size());
      }

      void set_aircraft(Aircraft* a) { aircraft = a; }
      Aircraft* get_aircraft() const { return aircraft; }

      void compute_lof_cost() {
         cost = 0;
         for (const auto arc : arc_list) {
            cost += aircraft->get_assignment_cost(arc->get_leg_copy());
         }

         if (Config::instance()->rule_config.position_rule == PositionRule::Hard) {
            if (aircraft->get_end_station() != this->get_destination()) {
               cost += Config::instance()->rule_config.cost_miss_endpos;
            }
         }
      }

      void set_cost(double c) { cost = c; }
      double get_cost() const { return cost; }

      void set_rd_cost(double c) { reduced_cost = c; }
      double get_rd_cost() const { return reduced_cost; }

      Station* get_origin() { return arc_list.empty() ? aircraft->get_start_station() : arc_list.front()->get_origin(); }
      Station* get_destination() { return arc_list.empty() ? aircraft->get_start_station() : arc_list.back()->get_destination(); }

      ptime get_start_time() { return arc_list.front()->get_start_time(); }
      ptime get_end_time() { return arc_list.back()->get_end_time(); }

      int num_legs_in_slot(const Slot* const slot) const
      {
         int result = 0;
         for (const auto arc : arc_list)
         {
            if (arc->is_in_slot(slot)) result++;
         }
         return result;
      }

      bool is_in_slot(const Slot* const slot)
      {
         for (auto arc : this->arc_list) {
            if (arc->is_in_slot(slot)) return true;
         }
         return false;
      }
   };


   class Solver
   {
   private:
      std::vector<Path* > new_paths;
      std::vector<std::shared_ptr<Path> > all_paths;
      std::map<int, GRBVar*> map_path_vars;
      int iteration;
      double mp_ub, mp_lb, mp_new_lb;
      double gap;
      boost::posix_time::time_duration solve_time;
      std::vector<double> min_rd_cost;
      std::vector<std::vector<Leg*>> eligible_legs;
      std::vector<std::unordered_set<LegCopy, LegCopyHash>> existing_copies;
      std::vector<std::unique_ptr<GRBEnv>> pricing_environments;
      std::map<size_t, std::vector<std::shared_ptr<LegCopy>>> aircraft_to_reserve_copies;
      std::map<size_t, std::vector<std::shared_ptr<LegCopy>>> aircraft_to_new_copies;
      std::map<size_t, std::vector<std::shared_ptr<LegCopy>>> aircraft_to_all_copies;

      Timespace::Network network;

   public:
      PathModel path_model;

      explicit Solver() 
      {
         mp_ub = Config::instance()->alg_config.big_m;
         mp_lb = 0;
         mp_new_lb = 0;
         iteration = 0;
         gap = Config::instance()->alg_config.big_m;
         solve_time = boost::posix_time::seconds(0);

         aircraft_to_all_copies.clear();
         aircraft_to_new_copies.clear();
         aircraft_to_reserve_copies.clear();
      }
      ~Solver() {};

      void run();
      void copy_generation();
      void column_generation();

      void set_network_arc_costs();
      void multi_label_algorithm(const Aircraft* aircraft, const bool forward, const bool backward, const bool sequential);
      void label_setting(const Timespace::Arc* arc, const Timespace::Label* const predecessor, const Aircraft* aircraft,
         const bool forward, const bool sequential);
      void copy_evaluation_individual(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies);
      void copy_evaluation_multiple(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies);

      void copy_evaluation_individual_csc(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies);
      void copy_evaluation_multiple_csc_propagation(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies);
      void copy_evaluation_multiple_csc_solve_minlp(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies);

      void initialize();
      void get_mp_duals();

      double scoring_copy(LegCopy& delay_copy, const Aircraft* aircraft, Timespace::Node* left, Timespace::Node* right);
      double scoring_propagation_copies(std::vector<LegCopy>& candidate_copies, const Aircraft* aircraft, LegCopy& delay_copy,
         const Timespace::Label* fwd_label, const Timespace::Label* bwd_label);
      void depth_first_search_copies(const Aircraft* aircraft, std::vector<LegCopy>& best_copies,
         std::vector<LegCopy>& set_copies, const std::vector<LegCopy*>& copies_to_adjust,
         size_t i, boost::posix_time::ptime new_dep_time, double score, double& best_score, size_t& leaves);

      bool has_copy(const Aircraft* aircraft, const LegCopy& copy) const {
         return existing_copies.at(path_model.get_index(aircraft)).contains(copy);
      }

      double scoring_csc_copies(std::vector<LegCopy>& candidate_copies, const Aircraft* aircraft, LegCopy& delay_copy,
         const Timespace::Label* fwd_label, const Timespace::Label* bwd_label);
      void solve_time_decisions_with_minlp(const std::vector<LegCopy*>& copies_to_adjust, std::vector<LegCopy>& copies_adjusted, 
         const Aircraft* aircraft);

      void manage_copies();

      void add_columns();																					/* ******** Add columns to master problem ******** */
      void update_bounds();

      void get_solution();

      void write_paths(const std::vector<Path*>& paths) const;
   };
}
