// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once
#include <string>
#include <vector>
#include<boost/date_time/posix_time/posix_time.hpp>
#include "util.h"


enum class PositionRule {
   Hard,           // each aircraft must end at its destination
   Relaxed,        // each aircraft can end at any station
   Balanced,      // the number of aircrafts of each family/model at each station should be balanced, penalty applied otherwise
   Specified      // the number of aircrafts of each family/model at each station should meet specified requirements, penalty applied otherwise
};

enum class Algorithm { CCG };

struct MainConfig {
   std::string path_to_instance_folder = "";
   std::string path_to_input = "";
   std::string path_to_output = "";
};

struct RuleConfig {
   PositionRule position_rule{ PositionRule::Hard };

   boost::posix_time::time_duration max_flt_delay{ Util::get_duration_from_minutes(180) };
   boost::posix_time::time_duration max_mtc_delay{ boost::posix_time::seconds(0) };
   boost::posix_time::time_duration max_mtc_prepone{ boost::posix_time::seconds(0) };

   double cost_flt_delay{ 100 };
   double cost_mtc_delay{ 0 };
   double cost_swap{ 1000 };
   double cost_flt_cancel{ 1e6 };
   double cost_mtc_cancel{ 1e8 };
   double cost_miss_endpos{ 1e8 };
   double cost_pos_family{ 2e4 };
   double cost_pos_model{ 5000 };
   double cost_pos_config{ 1000 };
   double cost_fuel{ 1.0 };         // fuel cost due to cruise speed change
   double cost_compression_a{ 10.0 }; // cost coefficient a for compression cost due to cruise speed change
   double cost_compression_b{ 1.0 };  // cost coefficient b for compression cost due to cruise speed change

   bool use_cruise_control{ false };         // whether to consider the change in cruise speed as a recovery option
   boost::posix_time::time_duration non_cruise_time{ Util::get_duration_from_minutes(30) };
   double orig_cruise_speed_ratio{ 1.02 };  // ratio of original cruise speed to MRC speed
   double max_cruise_speed_ratio{ 1.1 };   // maximum ratio of cruise speed to MRC speed

   int min_ground_time{ 30 };
};

struct AlgorithmConfig {
   Algorithm algorithm{ Algorithm::CCG };

   bool write_lp_to_file{ false };
   bool write_copies_to_file{ false };
   bool print_network{ false };
   bool print_alg_process{ false };
   bool print_solver_output{ false };

   boost::posix_time::time_duration run_time_limit{ boost::posix_time::hours(1) };
   double big_m{ 1e10 };
   double gap_tolerance{ 1e-4 };

   int time_interval{ 30 };
   int num_threads{ 1 };
   int lp_method{ 2 }; // Gurobi: -1 auto, 0 primal simplex, 1 dual simplex, 2 barrier.

   // ccg parameters
   int max_num_copies{ 10 };
   int min_num_copies{ 1 };
   bool dynamic_max_copies{ false };

   bool seq_copy_generation{ false };
   bool solve_cruise_time_decisions{ true };              // whether to solve time decisions in the sequential delay generation process (only true if csc is allowed)

   double score_threshold{ 0 };
   double seq_copy_score_threshold{ -10 };
   double seq_csc_copy_gap_threshold{ 0.05 };

   // iteration and final integer-program parameters
   double max_IP_run_time{ 60 };
   int max_iterations{ 50 };
   int min_iterations{ 1 };
   int iteration_to_start_copy_filter{ 0 };         // iteration when to start filter copies
};


class Config
{
private:
   static Config* cfg_instance;
   Config();


public:
   static Config* instance() {
      if (!cfg_instance) {
         cfg_instance = new Config();
      }
      return cfg_instance;
   }

   bool DEBUG_PRINT;

   MainConfig main_config;
   AlgorithmConfig alg_config;
   RuleConfig rule_config;

   void load_config(const std::string& path_to_runfolder);

   PositionRule string_to_position_rule(const std::string& pr_str);
   Algorithm string_to_algorithm(const std::string& alg_str);
};

