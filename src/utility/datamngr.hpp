// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include "util.h"
#include "Flight.hpp"
#include "Aircraft.hpp"
#include "Station.hpp"
#include "Slot.hpp"
#include <iostream>
#include <memory>
#include <unordered_map>
#include "gurobi_c++.h"

class Slot;
class Flight;
class Maintenance;


class DataRegistry
{
private:
   static DataRegistry* dataInstance;
   DataRegistry();
   ~DataRegistry() = default;

public:

   static DataRegistry* instance(){
     if(!dataInstance){
       dataInstance = new DataRegistry();
     }
     return dataInstance;
   }

   boost::posix_time::ptime recovery_start, recovery_end;

   std::vector<std::shared_ptr<Flight>> flights;     // all scheduled flights
   std::vector<std::shared_ptr<Leg>> all_legs;       // all legs (with aircraft and actual time information compared to flight) including those inside and outside the recovery window
   std::vector<Leg*> legs;                           // includes only legs inside recovery window
   std::vector<std::shared_ptr<Aircraft>> aircrafts;
   std::vector<std::shared_ptr<Station>> stations;
   std::vector<std::shared_ptr<Slot>> slots;

   std::map<int, Flight*> map_flight_by_number;             // map flight number to flight
   std::unordered_map<size_t, Leg*> map_leg_by_id;          // map leg id to leg
   std::unordered_map<std::string, Station*> map_station_by_iata;       // map airport IATA code to airport
   std::unordered_map<std::string, Aircraft*> map_aircraft_by_reg;      // map tail number to aircraft
   std::unordered_map<size_t, Slot*> map_slot_by_id;                    // map slot id to slot

   // aircraft families, models and configurations
   std::vector<std::shared_ptr<AircraftFamily>> aircraft_families;
   std::vector<std::shared_ptr<AircraftModel>> aircraft_models;
   std::unordered_map<std::string, AircraftFamily*> map_aircraft_family_by_name;    // map aircraft family to aircraft
   std::unordered_map<std::string, AircraftModel*> map_aircraft_model_by_name;      // map aircraft model to aircraft

   // aircraft positioning requirements
   std::vector<std::shared_ptr<Position>> positions;    

   void read_config();
   void read_flights();
   void read_aircraft();
   void read_rotations();
   void read_position();
   void read_disruptions();
   
   void review_schedule();
   void review_positions();

   void write_schedule();

   void write_copies(int iteration, const std::map<size_t, std::vector<std::shared_ptr<LegCopy>>> & aircraft_to_new_copies,
      const std::map<size_t, std::vector<std::shared_ptr<LegCopy>>>& aircraft_to_reserved_copies, double obj);

   Leg* get_leg(const std::string& flight_number, const boost::posix_time::ptime& dep_time) {
      auto it = std::find_if(this->all_legs.begin(), this->all_legs.end(),
         [&flight_number, &dep_time](const std::shared_ptr<Leg>& leg) {
            return leg->get_flight_number() == std::stoul(flight_number) && leg->get_dep_time().date() == dep_time.date();
         });
      if (it != this->all_legs.end()) {
         return it->get();
      }
      return nullptr;
   }

   std::vector<Leg*> get_legs(int flight_number, const boost::posix_time::ptime& dep_time,
      const Station* orig, const Station* dest)
   {
      std::vector<Leg*> legs_found;
      for (const auto& leg_ptr : this->all_legs)
      {
         auto leg_flight_num = leg_ptr->get_flight_number();
         if (leg_flight_num == flight_number && leg_ptr->get_dep_time().date() == dep_time.date())
         {
            legs_found.push_back(leg_ptr.get());
         }
      }
      if (legs_found.size() <= 1) {
         return legs_found;
      }
      // further filter by origin and destination
      std::sort(legs_found.begin(), legs_found.end(), Leg::compare_sch_dep_time);
      std::vector<Leg*> filtered_legs;
      bool fixed_link_started = false;
      bool fixed_link_ended = false;
      for (const auto& leg : legs_found) {
         if (fixed_link_ended) break;
         if (leg->get_origin() == orig || fixed_link_started) {
            filtered_legs.push_back(leg);
            fixed_link_started = true;
         }
         if (leg->get_destination() == dest) {
            fixed_link_ended = true;
         }
      }
      return filtered_legs;
   }

   Station* getOrCreateStation(const std::string& code) 
   {
      if (code.empty())
      {
         return nullptr;
      }

      auto itrStn = this->map_station_by_iata.find(code);
      if (itrStn == this->map_station_by_iata.end())
      {
         auto pStn = std::make_shared<Station>(code);
         this->stations.push_back(pStn);
         this->map_station_by_iata[code] = pStn.get();
         return pStn.get();
      }
      return itrStn->second;
   }

   Position* createOrUpdatePosition(Station* stn, PositionType t, int count, AircraftFamily* fm=nullptr, AircraftModel* mdl = nullptr)
   {
      const auto it = std::find_if(this->positions.begin(), this->positions.end(), [stn, t, fm, mdl](const std::shared_ptr<Position>& pos) {
         return pos->get_station() == stn && pos->get_type() == t && pos->get_family() == fm && 
            pos->get_model() == mdl;
      });
      if (it != this->positions.end())
      {
         (*it)->increment_desired_count(count);
         return it->get();
      }
      auto pos = std::make_shared<Position>(stn, t, count, fm, mdl);
      this->positions.push_back(pos);
      return pos.get();
   }

   // declare a function that get or create an obj
   template <typename T>
   T* get_or_create(const std::string& key, std::vector<std::shared_ptr<T>>& vec, std::unordered_map<std::string, T*>& map)
   {
      auto it = map.find(key);
      if (it == map.end())
      {
         std::shared_ptr<T> obj = std::make_shared<T>(key);
         vec.push_back(obj);
         map[key] = obj.get();
         return obj.get();
      }
      return it->second;
   }
};


