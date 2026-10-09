// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "engine.h"
#include "config.h"
#include "util.h"
#include <filesystem>

int main(int argc, char** argv) {
   PRINT_SECTION("Aircraft Recovery Problem Solver");
   try {
      if (argc != 2 && argc != 4) {
         std::cerr << "Usage: " << argv[0] << " RUN_FOLDER [INSTANCE CCG]\n";
         return -1;
      }
      std::string path_to_runfolder = argv[1];

      PRINT_SECTION("Loading Configuration");
      Config::instance()->load_config(path_to_runfolder);

      if (argc >= 4) {  // override config file
         Config::instance()->alg_config.algorithm = Config::instance()->string_to_algorithm(argv[3]);

         Config::instance()->main_config.path_to_instance_folder = path_to_runfolder + "/" + argv[2];
         Config::instance()->main_config.path_to_input = Config::instance()->main_config.path_to_instance_folder + "/";
      }
      auto& cfg = *Config::instance();
      const auto input = std::filesystem::path(cfg.main_config.path_to_instance_folder).lexically_normal();
      for (const auto* name : {"config.csv", "flights.csv", "aircraft.csv", "rotations.csv"}) {
         if (!std::filesystem::is_regular_file(input / name))
            throw std::runtime_error("Missing required input: " + (input / name).string());
      }
      const auto output = std::filesystem::path(cfg.main_config.path_to_output) / input.filename() /
         (cfg.rule_config.use_cruise_control ? "CCG-CSC" : "CCG");
      std::filesystem::create_directories(output);
      cfg.main_config.path_to_output = output.string() + "/";

      PRINT_SECTION("Run instance: " + Config::instance()->main_config.path_to_instance_folder);

      ARP::Engine engine;

      PRINT_SECTION("Reading input data");
      engine.load_input_data();

      PRINT_SECTION("Optimizing");
      engine.solve();

      PRINT_SECTION("Writing results");
      engine.write_results();

      PRINT_SECTION("Writing KPI");
      engine.write_kpi();

      return 0;
   } catch (const GRBException& e) {
      std::cerr << "Gurobi error " << e.getErrorCode() << ": " << e.getMessage() << '\n';
   } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << '\n';
   }
   return 1;
}
