// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "utility/config.h"
#include "utility/util.h"
#include "boost/date_time/gregorian/greg_date.hpp"
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Config* Config::cfg_instance = nullptr;

Config::Config()
{
   DEBUG_PRINT = false;
}


void Config::load_config(const std::string& path_to_runfolder)
{
   std::string path_to_config = path_to_runfolder + "/config.json";

   std::ifstream config_file(path_to_config);
   if (!config_file) throw std::runtime_error("Cannot open configuration: " + path_to_config);
   json config_json = json::parse(config_file);
   auto& rule = config_json["rule_config"];
   auto& alg = config_json["algorithm_config"];

   main_config.path_to_instance_folder = path_to_runfolder + "/" + config_json["main_config"]["path_to_instance_folder"].get<std::string>();
   main_config.path_to_input = main_config.path_to_instance_folder + "/";
   
   rule_config.use_cruise_control = rule["use_cruise_control"].get<bool>();
   // Output directories are selected after command-line overrides in main().
   main_config.path_to_output = (std::filesystem::path(path_to_runfolder) /
      config_json["main_config"].value("path_to_results_folder", std::string("results"))).string();

   /* ---------------------- Rule configurations ---------------------- */
   rule_config.position_rule = string_to_position_rule(rule["position_rule"].get<std::string>());
   rule_config.cost_flt_cancel = rule["cost_flt_cancel"].get<float>();
   rule_config.cost_miss_endpos = rule["cost_miss_endpos"].get<float>();
   rule_config.cost_pos_family = rule["cost_pos_family"].get<float>();
   rule_config.cost_pos_model = rule["cost_pos_model"].get<float>();
   rule_config.cost_pos_config = rule["cost_pos_config"].get<float>();
   rule_config.max_flt_delay = Util::get_duration_from_minutes(rule["max_flt_delay"].get<int>());
   rule_config.cost_mtc_cancel = rule["cost_mtc_cancel"].get<float>();
   rule_config.max_mtc_delay = Util::get_duration_from_minutes(rule["max_mtc_delay"].get<int>());
   rule_config.cost_flt_delay = rule["cost_flt_delay"].get<float>();
   rule_config.cost_mtc_delay = rule["cost_mtc_delay"].get<float>();
   rule_config.cost_swap = rule["cost_swap"].get<float>();
   rule_config.cost_fuel = rule["cost_fuel"].get<float>();
   rule_config.cost_compression_a = rule["cost_compression_a"].get<float>();
   rule_config.cost_compression_b = rule["cost_compression_b"].get<float>();

   rule_config.non_cruise_time = Util::get_duration_from_minutes(rule["non_cruise_time"].get<int>());
   rule_config.orig_cruise_speed_ratio = rule["orig_cruise_speed_ratio"].get<float>();
   rule_config.max_cruise_speed_ratio = rule["max_cruise_speed_ratio"].get<float>();

   rule_config.min_ground_time = rule["min_ground_time"].get<int>();

   /* ---------------------- Algorithm configurations ---------------------- */
   alg_config.algorithm = string_to_algorithm(alg["algorithm"].get<std::string>());

   alg_config.write_lp_to_file = alg["write_lp_files"].get<bool>();
   alg_config.write_copies_to_file = alg["write_copies_to_file"].get<bool>();
   alg_config.print_network = alg["print_network"].get<bool>();
   alg_config.print_alg_process = alg["print_alg_process"].get<bool>();
   alg_config.print_solver_output = alg["print_solver_output"].get<bool>();
   alg_config.big_m = alg["big_m"].get<float>();
   alg_config.run_time_limit = Util::get_duration_from_minutes(alg["run_time_limit"].get<int>());
   alg_config.gap_tolerance = alg["gap_tolerance"].get<float>();

   alg_config.time_interval = alg["time_interval"].get<int>();

   alg_config.seq_copy_generation = alg["seq_copy_generation"].get<bool>();
   alg_config.solve_cruise_time_decisions = alg["solve_cruise_time_decisions"].get<bool>();

   alg_config.score_threshold = alg["score_threshold"].get<float>();
   alg_config.seq_copy_score_threshold = alg["seq_copy_score_threshold"].get<float>();
   alg_config.seq_csc_copy_gap_threshold = alg.value("seq_csc_copy_gap_threshold", alg_config.seq_csc_copy_gap_threshold);

   alg_config.max_num_copies = alg["max_num_copies"].get<int>();
   alg_config.min_num_copies = alg["min_num_copies"].get<int>();
   alg_config.dynamic_max_copies = alg["dynamic_max_copies"].get<bool>();

   alg_config.num_threads = alg["num_threads"].get<int>();
   alg_config.lp_method = alg.value("lp_method", alg_config.lp_method);
   if (alg_config.num_threads < 1) throw std::runtime_error("num_threads must be positive");
   if (alg_config.lp_method < -1 || alg_config.lp_method > 2)
      throw std::runtime_error("lp_method must be -1, 0, 1, or 2");

   alg_config.max_IP_run_time = alg["max_IP_run_time"].get<float>();
   alg_config.max_iterations = alg["max_iterations"].get<int>();
   alg_config.min_iterations = alg["min_iterations"].get<int>();
   alg_config.iteration_to_start_copy_filter = alg["iteration_to_start_copy_filter"].get<int>();
}

PositionRule Config::string_to_position_rule(const std::string& pr_str)
{
   std::string pr_str_lower = "";
   std::transform(pr_str.begin(), pr_str.end(), std::back_inserter(pr_str_lower), ::tolower);
   if (pr_str_lower == "hard") {
      return PositionRule::Hard;
   }
   else if (pr_str_lower == "relaxed") {
      return PositionRule::Relaxed;
   }
   else if (pr_str_lower == "balanced") {
      return PositionRule::Balanced;
   }
   else if (pr_str_lower == "specified") {
      return PositionRule::Specified;
   }
   else {
      throw std::runtime_error("Unknown position rule string: " + pr_str_lower);
   }
}

Algorithm Config::string_to_algorithm(const std::string& alg_str)
{
   std::string alg_str_lower = "";
   std::transform(alg_str.begin(), alg_str.end(), std::back_inserter(alg_str_lower), ::tolower);
   if (alg_str_lower == "ccg") {
      return Algorithm::CCG;
   }
   throw std::runtime_error("Only CCG is included in this repository; requested: " + alg_str);
}
