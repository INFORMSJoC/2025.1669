// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "utility/datamngr.hpp"
#include "boost/algorithm/string.hpp"
#include <fstream>
#include <iostream>
#include "utility/config.h"

DataRegistry* DataRegistry::dataInstance = nullptr;

DataRegistry::DataRegistry()
{
   recovery_start = boost::posix_time::not_a_date_time;
   recovery_end = boost::posix_time::not_a_date_time;

   aircrafts.clear();
   slots.clear();
   all_legs.clear();
   legs.clear();
}

void DataRegistry::read_config()
{
   std::ifstream ifs;
   ifs.open(Config::instance()->main_config.path_to_input + "config.csv", std::ifstream::in);

   int row = 0;
   while (ifs.good())
   {
      row++;
      char lineChars[256];
      ifs.getline(lineChars, 256);

      std::string strLine(lineChars);

      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if (sVals.size() < 4)
      {
         break;
      }
      
      std::string startTimeStr = sVals[0] + " " + sVals[1];
      this->recovery_start = Util::string_to_ptime(startTimeStr);

      std::string endTimeStr = sVals[2] + " " + sVals[3];
      this->recovery_end = Util::string_to_ptime(endTimeStr);

      if(row >= 1)
         break; // TEMP: for ARP just read the first line to get the recovery period
   }
   ifs.close();
}

void DataRegistry::read_flights() 
{
   std::string formatStr = "%H:%M";

   std::ifstream ifs;
   ifs.open(Config::instance()->main_config.path_to_input + "flights.csv", std::ifstream::in);
   while (ifs.good())
   {
      char lineChars[256];
      ifs.getline(lineChars, 256);

      std::string strLine(lineChars);

      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if (sVals.size() < 6)
      {
         break;
      }
      int flt_number = std::stoi(sVals[0]);
      auto origin = getOrCreateStation(sVals[1]);
      auto destination = getOrCreateStation(sVals[2]);
      
      auto dep_time = Util::get_hr_min_time_from_string(sVals[3]);
      
      bool arrNextDay = false;
      std::string arrTimeStr = sVals[4];
      auto pos = arrTimeStr.find("+1");
      if (pos != std::string::npos) {
         arrNextDay = true;
         arrTimeStr.erase(pos, arrTimeStr.length());
      }
      auto arr_time = Util::get_hr_min_time_from_string(arrTimeStr);

      Flight* prev_flight = nullptr;
      int prevFltNumber = std::stoi(sVals[5]);
      if (prevFltNumber != 0) {
         auto itrFlight = this->map_flight_by_number.find(prevFltNumber);
         if (itrFlight == this->map_flight_by_number.end())
         {
            std::cerr << "Flight does not exist while reading prev flight!";
            exit(EXIT_FAILURE);        
         }
         else {
            prev_flight = itrFlight->second;
         }
      }

      auto pFlight = std::make_shared<Flight>(flt_number, origin, destination, dep_time, arr_time, arrNextDay, prev_flight);
      this->flights.push_back(pFlight);
      this->map_flight_by_number.emplace(flt_number, pFlight.get());
   }
   ifs.close();

   PRINT_LOG("number of flights: " + std::to_string(this->flights.size()));
}

void DataRegistry::read_aircraft() 
{
   std::ifstream ifs;
   ifs.open(Config::instance()->main_config.path_to_input + "aircraft.csv", std::ifstream::in);
   int numMtc = 0;

   while (ifs.good())
   {
      char lineChars[256];
      ifs.getline(lineChars, 256);

      std::string strLine(lineChars);

      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if (sVals.size() < 10)
      {
         break;
      }
      std::string tail = sVals[0];
      std::string model = sVals[1];
      std::string family = sVals[2];
      std::string seatConfig = sVals[3];
      double cost = std::stof(sVals[5]);
      int turn = std::stoi(sVals[6]);
      auto startStation = getOrCreateStation((sVals[8]));

      auto acFamily = this->get_or_create<AircraftFamily>(family, this->aircraft_families, this->map_aircraft_family_by_name);
      auto acModel = this->get_or_create<AircraftModel>(model, this->aircraft_models, this->map_aircraft_model_by_name);

      auto aircraft = std::make_shared<Aircraft>(tail, acFamily, acModel, startStation, turn, cost);
      this->aircrafts.push_back(aircraft);
      this->map_aircraft_by_reg.emplace(tail, aircraft.get());

      acFamily->aircrafts.push_back(aircraft.get());
      acModel->aircrafts.push_back(aircraft.get());
      acModel->family = acFamily;

      // maintenance data
      std::string &mainStr = sVals[9];
      if (mainStr != "NULL") {
         std::vector<std::string> maintData;
         boost::split(maintData, mainStr, boost::is_any_of("-"));
         auto station = getOrCreateStation(maintData[0]);
         auto startTimeStr = maintData[1] + " " + maintData[2];
         auto endTimeStr = maintData[3] + " " + maintData[4];
         auto start_time = Util::string_to_ptime(startTimeStr);
         auto end_time = Util::string_to_ptime(endTimeStr);
         auto maxFlyTime = Util::get_duration_from_minutes(std::stoi(maintData[5]));

         auto maintenance = std::make_shared<Maintenance>(static_cast<unsigned>(this->all_legs.size()), station, aircraft.get(), start_time, end_time, maxFlyTime);
         this->all_legs.push_back(maintenance);
         this->map_leg_by_id.emplace(maintenance->get_id(), maintenance.get());
         // aircraft->setMaintenance(maintenance.get());
         numMtc++;
      }

   }
   std::sort(this->all_legs.begin(), this->all_legs.end(), [](const auto& a, const auto& b) {
      return a->get_dep_time() < b->get_dep_time();
   });
   ifs.close();

   PRINT_LOG("number of aircraft: " + std::to_string(this->aircrafts.size()));
   PRINT_LOG("number of planned maintenance: " + std::to_string(numMtc));
}

void DataRegistry::read_rotations()
{
   std::ifstream ifs;
   ifs.open(Config::instance()->main_config.path_to_input + "rotations.csv", std::ifstream::in);
   while (ifs.good())
   {
      char lineChars[256];
      ifs.getline(lineChars, 256);

      std::string strLine(lineChars);

      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if (sVals.size() < 3)
      {
         break;
      }
      int flt_number = std::stoi(sVals[0]);
      auto depDate = Util::string_to_date(sVals[1]);

      auto fltItr = this->map_flight_by_number.find(flt_number);
      if (fltItr == this->map_flight_by_number.end()) {
         std::cerr << "The flight in the rotation file doesn't exist: " << flt_number;
         exit(EXIT_FAILURE);
      }
      const auto flight = fltItr->second;
      const auto flight_dep_time = flight->get_dep_time();
      const auto dep_time = boost::posix_time::ptime(depDate, boost::posix_time::hours(flight_dep_time.tm_hour) + boost::posix_time::minutes(flight_dep_time.tm_min));

      int arrNextDay = static_cast<int>(flight->is_arr_next_day());
      const auto arrDate = depDate + boost::gregorian::days(arrNextDay);
      const auto flight_arr_time = flight->get_arr_time();
      const auto arr_time = boost::posix_time::ptime(arrDate, boost::posix_time::hours(flight_arr_time.tm_hour) + boost::posix_time::minutes(flight_arr_time.tm_min));

      const auto itr_aircraft = this->map_aircraft_by_reg.find(sVals[2]);
      if (itr_aircraft == this->map_aircraft_by_reg.end()) {
         std::cerr << "The aircraft in the rotation file doesn't exist: " << sVals[2];
         exit(EXIT_FAILURE);
      }
      auto aircraft = itr_aircraft->second;
      if (aircraft->is_shuttle()) {
         continue;
      }

      auto leg = std::make_shared<Leg>(static_cast<unsigned>(this->all_legs.size()), flight, aircraft, dep_time, arr_time);

      this->all_legs.push_back(leg);
      this->map_leg_by_id.emplace(leg->get_id(), leg.get());
   }

   // sort all legs by time
   std::sort(this->all_legs.begin(), this->all_legs.end(), [](std::shared_ptr<Leg> &a, std::shared_ptr<Leg> &b)->bool {
      return a->get_dep_time() < b->get_dep_time();
      });
   ifs.close();

   PRINT_LOG("number of legs: " + std::to_string(this->all_legs.size()));
}

void DataRegistry::read_disruptions()
{
   std::ifstream ifs;

   // flight disruption
   ifs.open(Config::instance()->main_config.path_to_input + "alt_flights.csv", std::ifstream::in);
   int row = 0;
   int num_canceled_flights = 0;
   int num_delayed_flights = 0;
   while (ifs.good())
   { 
      char lineChars[256];
      ifs.getline(lineChars, 256);
      std::string strLine(lineChars);
      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if (boost::algorithm::starts_with(strLine, "#") || (sVals.size() < 3))
      {
         break;
      }
      row++;
      const auto flt_number = std::stoi(sVals[0]);
      const auto depDate = Util::string_to_date(sVals[1]);
      const int delay = std::stoi(sVals[2]);

      auto legItr = std::find_if(this->all_legs.begin(), this->all_legs.end(), [flt_number,depDate](const std::shared_ptr<Leg>& l)->bool{
         return (l->get_flight_number() == flt_number) && (l->get_dep_time().date() == depDate);
      });
      if (legItr == this->all_legs.end()) {
         std::cerr << "The leg in the rotation file doesn't exist: " << flt_number << "\t@" << depDate;
         exit(EXIT_FAILURE);
      }
      auto& leg = *legItr;

      if (delay == -1) {
         leg->set_disruption(Disruption::FlightCancel);
         num_canceled_flights++;
      }
      else {
         leg->set_disruption(Disruption::FlightDelay);
         leg->set_primary_delay(Util::get_duration_from_minutes(delay));
         leg->set_dep_time(leg->get_sch_dep_time() + leg->get_primary_delay());
         leg->set_arr_time(leg->get_sch_arr_time() + leg->get_primary_delay());
         num_delayed_flights++;
      }
   }
   ifs.close();

   PRINT_LOG("number of delayed flights: " + std::to_string(num_delayed_flights));
   PRINT_LOG("number of canceled flights: " + std::to_string(num_canceled_flights));

   // aircraft disruption
   row = 0;
   ifs.open(Config::instance()->main_config.path_to_input + "alt_aircraft.csv", std::ifstream::in);
   while (ifs.good())
   {  
      char lineChars[256];
      ifs.getline(lineChars, 256);
      std::string strLine(lineChars);
      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if ( boost::algorithm::starts_with(strLine, "#") || (sVals.size() < 5))
      {
         break;
      }
      row++;

      std::string code = sVals[0];
      const auto aircraft = this->map_aircraft_by_reg.at(code);
      std::string startTimeStr = sVals[1] + " " + sVals[2];
      std::string endTimeStr = sVals[3] + " " + sVals[4];
      const auto start_time = Util::string_to_ptime(startTimeStr);
      const auto end_time = Util::string_to_ptime(endTimeStr);
      const auto station = sVals.size() > 5 ? getOrCreateStation(sVals[5]) : nullptr;
      auto maintenance = std::make_shared<Maintenance>(static_cast<unsigned>(this->all_legs.size()), station, aircraft,
         start_time, end_time, Util::get_duration_from_minutes(0), true, false);
      aircraft->set_aog_maintenance(maintenance.get());
      maintenance->set_disruption(Disruption::AircraftUnavailable);
      this->all_legs.push_back(maintenance);
      this->map_leg_by_id.emplace(maintenance->get_id(), maintenance.get());
   }
   ifs.close();
   PRINT_LOG("number of AOG: " + std::to_string(row));

   // airport disruption
   ifs.open(Config::instance()->main_config.path_to_input + "alt_airports.csv", std::ifstream::in);
   int i = 0;
   while (ifs.good())
   {
      char lineChars[256];
      ifs.getline(lineChars, 256);
      std::string strLine(lineChars);
      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));
      if (boost::algorithm::starts_with(strLine, "#") || (sVals.size() < 7))
      {
         break;
      }

      const auto airport = getOrCreateStation(sVals[0]);
      std::string startTimeStr = sVals[1] + " " + sVals[2];
      const auto start_time = Util::string_to_ptime(startTimeStr);
      std::string endTimeStr = sVals[3] + " " + sVals[4];
      const auto end_time = Util::string_to_ptime(endTimeStr);

      if (sVals[5] != "None") {
         const int depLimit = std::stoi(sVals[5]);
         if (depLimit == 0) {
            this->slots.push_back(std::make_shared<Slot>(i++, start_time, end_time, airport, SlotType::Departure, depLimit));
         }
         else { // create multiple slots between startTime and endTime each w.r.t. one hour
            for (auto t = start_time; t < end_time; t += boost::posix_time::hours(1)) {
               this->slots.push_back(std::make_shared<Slot>(i++, t, t + boost::posix_time::hours(1), airport, SlotType::Departure, depLimit));
            }
         }
      }
      if (sVals[6] != "None") {
         const int arrLimit = std::stoi(sVals[6]);
         if (arrLimit == 0) {
            this->slots.push_back(std::make_shared<Slot>(i++, start_time, end_time, airport, SlotType::Arrival, arrLimit));
         }
         else { // create multiple slots between startTime and endTime each w.r.t. one hour
            for (auto t = start_time; t < end_time; t += boost::posix_time::hours(1)) {
               this->slots.push_back(std::make_shared<Slot>(i++, t, t + boost::posix_time::hours(1), airport, SlotType::Arrival, arrLimit));
            }
         }
      }
      if (sVals.size() > 7 && sVals[7] != "None") {
         const int mixLimit = std::stoi(sVals[7]);
         if (mixLimit == 0) {
            this->slots.push_back(std::make_shared<Slot>(i++, start_time, end_time, airport, SlotType::Mixture, mixLimit));
         }
         else { // create multiple slots between startTime and endTime each w.r.t. one hour
            for (auto t = start_time; t < end_time; t += boost::posix_time::hours(1)) {
               this->slots.push_back(std::make_shared<Slot>(i++, t, t + boost::posix_time::hours(1), airport, SlotType::Mixture, mixLimit));
            }
         }
      }
   }
   ifs.close();

   for (auto& it : slots) {
      map_slot_by_id.emplace(it->get_id(), it.get());
   }

   PRINT_LOG("number of airport capacity slots: " + std::to_string(this->slots.size()));
}

void DataRegistry::review_schedule()
{
   std::sort(this->all_legs.begin(), this->all_legs.end(), [](std::shared_ptr<Leg>& a, std::shared_ptr<Leg>& b)->bool {
      return a->get_dep_time() < b->get_dep_time();
      });

   for(auto &leg : all_legs){
      auto aircraft = leg->get_aircraft();
      aircraft->route.push_back(leg.get());

      if (leg->get_disruption() == Disruption::FlightCancel) {
         continue;
      }
      
      if (leg->get_dep_time() >= this->recovery_start && leg->get_arr_time() <= this->recovery_end) {
         this->legs.push_back(leg.get());
         aircraft->add_leg(leg.get());
      }
      else if (leg->get_dep_time() < this->recovery_start) {
         if ((aircraft->get_prev_leg() == nullptr) || (aircraft->get_prev_leg()->get_dep_time() < leg->get_dep_time()))
            aircraft->set_prev_leg(leg.get());
      }
      else if (leg->get_arr_time() > this->recovery_end) {
         if ((aircraft->get_next_leg() == nullptr) || (aircraft->get_next_leg()->get_dep_time() > leg->get_dep_time()))
            aircraft->set_next_leg(leg.get());
      }
   }

   for (auto& a : this->aircrafts) {
      std::sort(a->rotations.begin(), a->rotations.end(), Leg::compare_dep_time);

      // planned maintenance
      for (const auto l : a->rotations) {
         if (l->is_maintenance() && !(l->is_aog_maintenance())) {
            a->set_maintenance(static_cast<Maintenance*>(l));
            break;
         }
      }

      // set AOG maintenance
      if ((a->get_aog_maintenance() != nullptr) && (a->get_aog_maintenance()->get_location() == nullptr)) {
         Station* loc = nullptr;
         auto start_time = a->get_aog_maintenance()->get_dep_time();
         for (auto itr = a->rotations.begin(); itr != a->rotations.end(); itr++) {
            if ((*itr)->is_aog_maintenance()) continue;
            if ((*itr)->get_dep_time() >= start_time) {
               loc = (*itr)->get_origin();
               break;
            }
            auto nextItr = std::find_if(std::next(itr), a->rotations.end(), [](const Leg* l)->bool {
               return l->is_aog_maintenance() == false;
               });
            if ((*itr)->get_arr_time() <= start_time && (nextItr == a->rotations.end() || (*nextItr)->get_dep_time() >= start_time)) {
               loc = (*itr)->get_destination();
               break;
            }
         }
         a->get_aog_maintenance()->set_location(loc);
         assert(a->get_aog_maintenance()->get_location() != nullptr);
      }

      a->set_start_time(this->recovery_start);
      a->set_end_time(this->recovery_end + a->get_min_ground_time());
      // set aircraft start and end time/locations
      if (a->get_prev_leg() != nullptr) {
         a->set_start_time(std::max(a->get_prev_leg()->get_adj_arr_time(a.get()), this->recovery_start));
         a->set_start_station(a->get_prev_leg()->get_destination());
      }
      else {
         if (a->rotations.size() > 0) {
            auto firstFlight = a->rotations.front();
            if (firstFlight->get_origin() != a->get_start_station()) {
               std::cerr << "Aircraft start location revised: " << a->get_reg_number() << std::endl;
               a->set_start_station(firstFlight->get_origin());
            }
         }
      }

      if (a->get_next_leg() != nullptr) {
         a->set_end_time(std::min(a->get_next_leg()->get_adj_dep_time(), a->get_end_time()));
         a->set_end_station(a->get_next_leg()->get_origin());
      }
      else {
         if (a->rotations.size() == 0) {
            a->set_end_station(a->get_start_station());
         }
         else {
            auto lastFlight = a->rotations.back();
            a->set_end_station(lastFlight->get_destination());
         }
      }
   }

   // re-calculate the remaining hours for each maintenance
   for (const auto& aircraft : this->aircrafts) {
      const auto maintenance = aircraft->get_maintenance();
      if (maintenance == nullptr) continue;
      const auto remainingHours = maintenance->get_remaining_fly_time();
      boost::posix_time::time_duration extraHours = boost::posix_time::hours(0);
      for (const auto& leg : aircraft->rotations) {
         if (leg->get_arr_time() < maintenance->get_dep_time()) {
            if (leg->is_aog_maintenance()) continue;
            extraHours += leg->get_duration();
         }
      }
      maintenance->set_remaining_fly_time(remainingHours + extraHours);
   }
}

void DataRegistry::read_position()
{
   if (Config::instance()->rule_config.position_rule != PositionRule::Specified) return;
   std::ifstream ifs;
   ifs.open(Config::instance()->main_config.path_to_input + "position.csv", std::ifstream::in);

   while (ifs.good())
   {
      char lineChars[256];
      ifs.getline(lineChars, 256);
      std::string strLine(lineChars);
      if (boost::algorithm::starts_with(strLine, "#")) {
         break;
      }

      std::vector<std::string> sVals;
      boost::split(sVals, strLine, boost::is_any_of(" "));

      auto station = getOrCreateStation(sVals[0]);
      AircraftModel* acModel = nullptr;
      // AircraftConfig* acConfig = nullptr;
      for (int i = 1; i < sVals.size(); i++) {
         std::string& sVal = sVals[i];
         if (boost::algorithm::contains(sVal, "#")) {
            break;
         }
         if (i % 3 == 1) {
            auto it = map_aircraft_model_by_name.find(sVal);
            if (it != map_aircraft_model_by_name.end()) {
               acModel = it->second;
            }
         }
         /*else if (i % 3 == 2) {
            auto it = acConfigMap.find(sVal);
            if (it != acConfigMap.end()) {
               acConfig = it->second;
            }
         }*/
         else if (i % 3 == 0) {
            int count = std::stoi(sVal);
            if (acModel) {
               createOrUpdatePosition(station, PositionType::Family, count, acModel->family);
               createOrUpdatePosition(station, PositionType::Model, count, acModel->family, acModel);
               // createOrUpdatePosition(station, PositionType::Config, count, acModel->family, acModel, acConfig);
            }
            acModel = nullptr;
            // acConfig = nullptr;
         }
      }
   }

   ifs.close();
}

void DataRegistry::review_positions()
{
   if (Config::instance()->rule_config.position_rule != PositionRule::Balanced) return;

   for (const auto& aircraft : this->aircrafts) {
      const auto end_station = aircraft->get_end_station();
      if (end_station == nullptr) continue;
      const auto aircraft_family = aircraft->get_family();
      const auto aircraft_model = aircraft->get_model();
      auto it_pos_family = std::find_if(this->positions.begin(), this->positions.end(),
         [end_station, aircraft_family](const std::shared_ptr<Position>& pos)->bool {
            return (pos->get_station()->get_id() == end_station->get_id()) &&
               (pos->get_type() == PositionType::Family) &&
               (pos->get_family() == aircraft_family);
         });
      if (it_pos_family != this->positions.end()) {
         (*it_pos_family)->increment_desired_count();
      }
      else {
         auto position = std::make_shared<Position>(end_station, PositionType::Family, 1, aircraft_family, nullptr);
         this->positions.push_back(position);
      }

      auto it_pos_model = std::find_if(this->positions.begin(), this->positions.end(),
         [end_station, aircraft_model](const std::shared_ptr<Position>& pos)->bool {
            return (pos->get_station()->get_id() == end_station->get_id()) &&
               (pos->get_type() == PositionType::Model) &&
               (pos->get_model() == aircraft_model);
         });
      if (it_pos_model != this->positions.end()) {
         (*it_pos_model)->increment_desired_count();
      }
      else {
         auto position = std::make_shared<Position>(end_station, PositionType::Model, 1,
            aircraft_family, aircraft_model);
         this->positions.push_back(position);
      }
   }

   PRINT_LOG("number of positions after review: " + std::to_string(this->positions.size()));
}

void DataRegistry::write_schedule() 
{
   std::string filename = Config::instance()->main_config.path_to_output + "schedule.dat";
   std::ofstream output;
   output.open(filename.c_str());
   output << "Number of aircrafts: " << this->aircrafts.size() << std::endl;
   output << "Recovery window: " << this->recovery_start << " --- " << this->recovery_end << std::endl;
   for (const auto& aircraft : this->aircrafts) {
      output << "========================================" << std::endl;
      output << "AIRCRAFT: " << aircraft->to_string() << std::endl;

      bool enter_recovery = false, leave_recovery = false;
      for (const auto leg : aircraft->route) {
         if (!enter_recovery && leg->get_adj_dep_time() >= this->recovery_start) {
            enter_recovery = true;
            output << aircraft->get_start_station()->get_code() << "[" << aircraft->get_start_time() << "]" << std::endl;
         }
         if (!leave_recovery && leg->get_adj_arr_time(aircraft.get()) > aircraft->get_end_time()) {
            leave_recovery = true;
            output << aircraft->get_end_station()->get_code() << "[" << aircraft->get_end_time() << "]" << std::endl;
         }
         output << leg->to_string() << std::endl;
      }

      if (!enter_recovery) {
         output << aircraft->get_start_station()->get_code() << "[" << aircraft->get_start_time() << "]" << std::endl;
      }
      if (!leave_recovery) {
         output << aircraft->get_end_station()->get_code() << "[" << aircraft->get_end_time() << "]" << std::endl;
      }
   }
   output.close();
}

void DataRegistry::write_copies(int iteration, const std::map<size_t, std::vector<std::shared_ptr<LegCopy>>>& aircraft_to_new_copies,
   const std::map<size_t, std::vector<std::shared_ptr<LegCopy>>>& aircraft_to_reserved_copies, double obj)
{
   int num_generated_copies = 0;
   int num_individual = 0, num_multiple = 0, num_csc_individual = 0, num_csc_multiple = 0;
   int num_used_copies = 0;
   int num_used_individual = 0, num_used_multiple = 0, num_used_csc_individual = 0, num_used_csc_multiple = 0;

   for (const auto &it : aircraft_to_new_copies) {
      num_generated_copies += static_cast<int>(it.second.size());
      for (const auto& copy : it.second) {
         switch (copy->get_leg_copy_type()) {
            case LegCopyType::Individual:
               num_individual++;
               break;
            case LegCopyType::Sequential:
               num_multiple++;
               break;
            case LegCopyType::CruiseIndividual:
               num_csc_individual++;
               break;
            case LegCopyType::CruiseSequential:
               num_csc_multiple++;
               break;
            default:
               break;
         }
      }
   }
   for (const auto& it : aircraft_to_reserved_copies) {
      num_used_copies += static_cast<int>(it.second.size());
      num_generated_copies += static_cast<int>(it.second.size());
      for (const auto& copy : it.second) {
         switch (copy->get_leg_copy_type()) {
         case LegCopyType::Individual:
            num_individual++;
            num_used_individual++;
            break;
         case LegCopyType::Sequential:
            num_multiple++;
            num_used_multiple++;
            break;
         case LegCopyType::CruiseIndividual:
            num_csc_individual++;
            num_used_csc_individual++;
            break;
         case LegCopyType::CruiseSequential:
            num_csc_multiple++;
            num_used_csc_multiple++;
            break;
         default:
            break;
         }
      }
   }

   std::string filename = Config::instance()->main_config.path_to_output + "copy_gen.out";
   std::ofstream output;

   if(iteration == 1){
      output.open(filename.c_str(), std::ios_base::trunc); // overwrite the file for the first iteration
      output << "===== Copy Generation Summary =====" << std::endl;
      output << std::setw(5) << "#Iter"
         << std::setw(15) << "TotalGen"
         << std::setw(10) << "ind(reg)"
         << std::setw(10) << "mul(reg)"
         << std::setw(10) << "ind(csc)"
         << std::setw(10) << "mul(csc)"
         << std::setw(10) << "TotalUsed"
         << std::setw(10) << "ind(reg)"
         << std::setw(10) << "mul(reg)"
         << std::setw(10) << "ind(csc)"
         << std::setw(10) << "mul(csc)"
         << std::setw(15) << "obj" << std::endl;
   }
   else {
      output.open(filename.c_str(), std::ios_base::app); 
   }

   output.precision(4);
   output << std::setw(5) << iteration
      << std::setw(15) << num_generated_copies
      << std::setw(10) << num_individual
      << std::setw(10) << num_multiple
      << std::setw(10) << num_csc_individual
      << std::setw(10) << num_csc_multiple
      << std::setw(10) << num_used_copies
      << std::setw(10) << num_used_individual
      << std::setw(10) << num_used_multiple
      << std::setw(10) << num_used_csc_individual
      << std::setw(10) << num_used_csc_multiple
      << std::setw(15) << obj << std::endl;

   output.close();
}
