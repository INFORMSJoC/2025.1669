// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "engine.h"
#include "config.h"
#include "datamngr.hpp"
#include "solmngr.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <ranges>

using namespace ARP;

Engine::Engine() 
{
   cpu_time = 0;
   upper_bound = Config::instance()->alg_config.big_m;
   lower_bound = 0;
   objective = 0;
}

void Engine::load_input_data()
{
   auto data_reg = DataRegistry::instance();

   data_reg->read_config();
   data_reg->read_flights();
   data_reg->read_aircraft();
   data_reg->read_rotations();
   data_reg->read_disruptions();

   data_reg->review_schedule();

   if (Config::instance()->rule_config.position_rule == PositionRule::Specified)
   {
      data_reg->read_position();
   }
   else if(Config::instance()->rule_config.position_rule == PositionRule::Balanced)
   {
      data_reg->review_positions();
   }

   data_reg->write_schedule();
}

void Engine::solve()
{
   PRINT_SECTION("Solving using CCG (copy and column generation) algorithm...");
   ccg.run();
}

void Engine::write_results() const
{
   auto data_reg = DataRegistry::instance();
   auto sol_reg = SolRegistry::instance();

   // write solution to result.out file showing the recovery schedule for each aircraft
   std::string filename = Config::instance()->main_config.path_to_output + "result.out";
   std::ofstream output;
   output.open(filename.c_str());

   output << "Objective:\t" << sol_reg->objective << std::endl;
   output << "Total CPU time:\t" << sol_reg->solve_time << std::endl;

   output << "================== Aircraft Recovery Schedule ==================" << std::endl;

   for (auto& aircraft : data_reg->aircrafts)
   {
      output << aircraft->get_reg_number() << "[" 
         << Util::get_minutes_from_duration(aircraft->get_min_ground_time()) << "]" << std::endl;

      std::vector<Leg*> prev_legs;
      std::vector<Leg*> next_legs;
      for (const auto& leg : aircraft->route)
      {
         if (leg->get_dep_time() < data_reg->recovery_start) {
            prev_legs.push_back(leg);
         }
         else if (leg->get_adj_arr_time(aircraft.get()) > aircraft->get_end_time()) {
            next_legs.push_back(leg);
         }
      }

      for (const auto& leg : prev_legs) {
         output << leg->to_string() << std::endl;
      }

      auto it = sol_reg->map_aircraft_id_to_route.find(aircraft->get_id());
      if (it == sol_reg->map_aircraft_id_to_route.end()) {
         continue;
      }

      std::sort(it->second.begin(), it->second.end(), [](const std::pair<Leg*, boost::posix_time::time_duration>& a,
         const std::pair<Leg*, boost::posix_time::time_duration>& b)->bool {
            return a.first->get_dep_time() + a.second < b.first->get_dep_time() + b.second;
         });

      output << aircraft->get_start_time() << std::endl;
      for (const auto& leg_info : it->second)
      {
         const auto leg = leg_info.first;
         output << leg->to_string(true) << std::endl;
      }
      output << aircraft->get_end_time() << std::endl;

      for (const auto& leg : next_legs) {
         output << leg->to_string() << std::endl;
      }
      output << "--------------------------------------------------------" << std::endl;
   }

   output << "================== Unassigned Flights ==================" << std::endl;
   for (const auto pLeg : sol_reg->unassigned_legs)
   {
      output << pLeg->to_string();
      output << std::endl;
   }

   output.close();

   // write solution to solution.out file showing the solution for each scheduled flights
   std::string solFile = Config::instance()->main_config.path_to_output + "solution.out";
   std::ofstream solOutput;
   solOutput.open(solFile);

   for (const auto& leg : data_reg->all_legs)
   {
      solOutput << leg->to_simple_string() << "\t" << leg->get_aircraft()->get_reg_number() << "\t";

      auto itrLeg = std::find(DataRegistry::instance()->legs.begin(), DataRegistry::instance()->legs.end(), leg.get());
      if (itrLeg == DataRegistry::instance()->legs.end()) {
         solOutput << leg->get_aircraft()->get_reg_number() << "\t-NA-" << std::endl;
      }
      else {
         auto itSol2 = sol_reg->map_leg_id_to_assignment.find(leg->get_id());
         if (itSol2 != sol_reg->map_leg_id_to_assignment.end())
         {
            solOutput << itSol2->second.first->get_reg_number() << "\t"
               << Util::get_minutes_from_duration(itSol2->second.second) << std::endl;
         }
         else
         {
            solOutput << "---\tUnassigned" << std::endl;
         }
      }
   }
   solOutput.close();
}

void Engine::write_kpi() const
{
   const auto data_reg = DataRegistry::instance();
   const auto sol_reg = SolRegistry::instance();
   const auto config = Config::instance();

   // flight cancellation and delays & aircraft swaps
   double cost_cancel = 0, cost_delay = 0;
   int num_delayed_flights = 0;
   double total_delay = 0, avg_delay = 0, min_delay = 0, max_delay = 0, cost_pos_violations = 0, cost_swaps = 0;
   int num_swaps = 0;
   for (const auto& leg : data_reg->all_legs)
   {
      if (std::ranges::find(data_reg->legs, leg.get()) == std::ranges::end(data_reg->legs)) continue;
      auto it = sol_reg->map_leg_id_to_assignment.find(leg->get_id());
      if (it == sol_reg->map_leg_id_to_assignment.end())
      {
         cost_cancel += leg->is_maintenance() ? config->rule_config.cost_mtc_cancel : config->rule_config.cost_flt_cancel;
         continue;
      }
      const auto delay = Util::get_minutes_from_duration(it->second.second);
      if (delay > 0) {
         num_delayed_flights++;
         total_delay += delay;
         if (delay < min_delay || min_delay == 0) min_delay = delay;
         if (delay > max_delay) max_delay = delay;
         cost_delay += delay * config->rule_config.cost_flt_delay;
      }
      if (it->second.first->get_reg_number() != leg->get_aircraft()->get_reg_number()) num_swaps++;
   }
   avg_delay = num_delayed_flights > 0 ? total_delay / num_delayed_flights : 0;
   cost_swaps = num_swaps * config->rule_config.cost_swap;

   for(int i = 0; i < sol_reg->position_violations.size(); i++) {
      const auto& position = data_reg->positions[i];
      const auto cost_pos_violation = position->get_type() == PositionType::Model ? config->rule_config.cost_pos_model :
         position->get_type() == PositionType::Family ? config->rule_config.cost_pos_family : 0;
      cost_pos_violations += sol_reg->position_violations[i] * cost_pos_violation;
   }

   std::string filename = Config::instance()->main_config.path_to_output + "kpi.out";
   std::ofstream output;
   output.open(filename.c_str());
   output.precision(4);

   output << "Run time(sec):" << std::setw(15) << sol_reg->solve_time.total_microseconds() / 1e6 << std::endl;
   output << "IP gap:" << sol_reg->gap << std::endl;
   if (config->alg_config.algorithm == Algorithm::CCG) {
      output << "\tTime for copy generation:\t" << sol_reg->copyGenTime << " seconds\n";
      output << "\tTime for column generation:\t" << sol_reg->colGenTime << " seconds\n";
      output << "\tTime for solving MP:\t" << sol_reg->solveMpTime << " seconds\n";
      output << "\tTime for solving IP:\t" << sol_reg->solveIpTime << " seconds\n";
      output << "\tTime for solving shortest paths:\t" << sol_reg->solveSppTime << " seconds\n";
      output << "\tTime for updating timespace networks:\t" << sol_reg->updateNetworkTime << " seconds\n";
      output << "\tShortest-path/network timings sum worker seconds and overlap generation timings.\n";
      output << "\tLabels considered:\t" << sol_reg->labels_considered << '\n';
      output << "\tPropagation leaves:\t" << sol_reg->propagation_leaves << '\n';
      output << "\tFinal network nodes:\t" << sol_reg->network_nodes << '\n';
      output << "\tMINLP solves:\t" << sol_reg->minlp_solves << '\n';
   }
   output << "======================= COST =======================" << std::endl;
   output << "======================= COST =======================" << std::endl;
   output << "Total:" << std::setw(23) << sol_reg->objective << std::endl;
   output << "Cancel:" << std::setw(22) << cost_cancel << std::endl;
   output << "Delay:" << std::setw(23) << cost_delay << std::endl;
   output << "Swaps:" << std::setw(23) << cost_swaps << std::endl;
   output << "PosViol:" << std::setw(21) << cost_pos_violations << std::endl;
   output << "CSC:" << std::setw(25) << sol_reg->objective - cost_cancel - cost_delay - cost_swaps - cost_pos_violations << std::endl;

   output << std::endl;
   output << "===================== DISRUPTION ===================" << std::endl;
   output << "# Cancelled:" << std::setw(17) << sol_reg->unassigned_legs.size() << std::endl;
   output << "# Delayed:" << std::setw(19) << num_delayed_flights << std::endl;
   output << "# Swaps:" << std::setw(21) << num_swaps << std::endl;
   output << "# CSCs:" << std::setw(22) << sol_reg->accelerated_legs.size() << std::endl;
   output << "Ttl delay(min):\t" << std::setw(13) << total_delay << std::endl;
   output << "Avg delay(min):\t" << std::setw(13) << int(avg_delay) << std::endl;
   output << "Min delay(min):\t" << std::setw(13) << min_delay << std::endl;
   output << "Max delay(min):\t" << std::setw(13) << max_delay << std::endl;

   output << std::endl;
   output << "================= AIRPORT CAPACITY =================" << std::endl;
   std::map<int, int> numLegsInSlots;
   for (const auto& slot : data_reg->slots) {
      numLegsInSlots[slot->get_id()] = 0;
   }
   for (const auto& leg : data_reg->all_legs)
   {
      const auto pLeg = leg.get();

      const auto itrLeg = std::find(DataRegistry::instance()->legs.begin(), DataRegistry::instance()->legs.end(), pLeg);
      if (itrLeg == data_reg->legs.end()) continue;                               // the flight is outside the recovery window 

      const auto it = sol_reg->map_leg_id_to_assignment.find(pLeg->get_id());
      if (it == sol_reg->map_leg_id_to_assignment.end())  continue;                          // the flight is cancelled        

      const auto delay = (it != sol_reg->map_leg_id_to_assignment.end()) ? it->second.second : Util::get_duration_from_minutes(0);
      for (const auto& slot : data_reg->slots) {
         const auto origin = pLeg->get_origin();
         const auto destination = pLeg->get_destination();
         const auto dep_time = pLeg->get_dep_time() + delay;
         const auto arr_time = pLeg->get_arr_time() + delay;
         const auto slotStart = slot->get_slot_interval().first;
         const auto slotEnd = slot->get_slot_interval().second;
         if ((slot->get_slot_type() == SlotType::Departure || slot->get_slot_type() == SlotType::Mixture) &&
            slot->get_station() == origin && slotStart <= dep_time && dep_time < slotEnd)
         {
            numLegsInSlots[slot->get_id()]++;
         }
         else if ((slot->get_slot_type() == SlotType::Arrival || slot->get_slot_type() == SlotType::Mixture) &&
            slot->get_station() == destination && slotStart <= arr_time && arr_time < slotEnd)
         {
            numLegsInSlots[slot->get_id()]++;
         }
      }
   }

   for (const auto& slot : data_reg->slots) {
      output << slot->get_station()->get_code() << " "
         << boost::posix_time::to_simple_string(slot->get_slot_interval().first) << " "
         << boost::posix_time::to_simple_string(slot->get_slot_interval().second) << " "
         << std::to_string(slot->get_slot_type()) << " "
         << slot->get_limit() << "(" << numLegsInSlots[slot->get_id()] << ")"
         << std::endl;
   }

   output << std::endl;
   output << "================= AIRCRAFT LOCATION ================" << std::endl;
   // aircraft, required end location and actual end location
   output << "Aircraft" << std::setw(15) << "PlannedEnd" << std::setw(15) << "ActualEnd" << std::endl;
   for (const auto& ac : data_reg->aircrafts)
   {
      const auto it = sol_reg->map_aircraft_id_to_route.find(ac->get_id());
      const auto planned_end_location = ac->get_end_station();
      auto actual_end_location = ac->get_start_station();
      if (it != sol_reg->map_aircraft_id_to_route.end() && !it->second.empty()) {
         actual_end_location = it->second.back().first->get_destination();
      }

      const auto planned_end_location_str = planned_end_location ? planned_end_location->get_code() : "N/A";
      const auto actual_end_location_str = actual_end_location ? actual_end_location->get_code() : "N/A";
      output << ac->get_reg_number() << std::setw(15) << planned_end_location_str << std::setw(15) << actual_end_location_str << std::endl;
   }

   output.close();
}
