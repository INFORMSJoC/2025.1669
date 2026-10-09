// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include <vector>
#include "utility/util.h"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "utility/config.h"
#include "air/Station.hpp"

class Station;
class Leg;
class LegCopy;
class Maintenance;
class Aircraft;

struct CruiseFuelFactors
{
   double c1, c2, c3, c4;
   double maxRangeCrs;    // maximum range cruise speed (the most fuel efficient) in km/min

   CruiseFuelFactors(): c1(0.00002579), c2(0.154734277), c3(0.37911718), c4(2274.703078), maxRangeCrs(14.48)
   {}
};

struct AircraftFamily
{
   size_t id;
   std::string code;
   std::vector<Aircraft*> aircrafts;

   AircraftFamily(std::string code) : code(code), id(std::hash<std::string>{}(code)) {}

   bool operator==(const AircraftFamily& f) const {
      return (this->code == f.code);
   }
};

struct AircraftModel
{
   size_t id;
   std::string code;
   AircraftFamily* family;
   std::vector<Aircraft*> aircrafts;

   AircraftModel(std::string code) : code(code), family(nullptr), id(std::hash<std::string>{}(code)) {}

   bool operator==(const AircraftModel& m) const {
      return (this->id == m.id);
   }
};

class Aircraft {
private:
   size_t id;                                               // Aircraft ID

   std::string reg_number;                                  // registration number
   AircraftFamily* family;                                  // Aircraft Family
   AircraftModel* model;               

   boost::posix_time::ptime start_time;                     // Start Available Time
   boost::posix_time::ptime end_time;                       // End Available Time

   Station* start_station;                                  // Start Available Station
   Station* end_station;                                    // End Available Station

   double opn_cost;
   boost::posix_time::time_duration min_ground_time;

   Leg* prev_leg;                        // the leg before recovery window (could be airborne)
   Leg* next_leg;                        // the leg after recovery window

   Maintenance* maintenance;            // planned maintenance
   Maintenance* aog;                    // AOG maintenance

public:
   std::vector<Leg *> rotations;       // flights assigned to aircraft by schedule within the recovery window
   std::vector<Leg*> route;            // all scheduled legs assigned to aircraft (including those outside the recovery window)

   CruiseFuelFactors crf;

   Aircraft(std::string _reg, AircraftFamily* _family, AircraftModel* _model,  
      Station * _start, int mgt, double cost):
      reg_number(_reg),
      model(_model),
      family(_family),
      start_station(_start),
      end_station(nullptr),
      min_ground_time(Util::get_duration_from_minutes(mgt)),
      opn_cost(cost),
      prev_leg(nullptr),
      next_leg(nullptr),
      maintenance(nullptr),
      aog(nullptr),
      id(std::hash<std::string>{}(_reg))
   {
      rotations.clear();
   };

   bool operator==(const Aircraft& other) const {
      return (this->reg_number == other.reg_number);
   }

   size_t get_id() const {return id;}
   void set_id(size_t i) {id = i;}

   void add_leg(Leg * leg) {rotations.push_back(leg);}
   const std::vector<Leg*>& get_assigned_legs() const {return rotations;}

   int get_num_legs() const { return static_cast<int>(rotations.size()); }

   Station* get_start_station() const {return start_station;}
   Station* get_end_station() const {return end_station;}

   ptime get_start_time() const {return start_time;}
   ptime get_end_time() const {return end_time;}

   void set_start_station(Station* s) { start_station = s; }
   void set_end_station(Station* s) { end_station = s; }
   void set_start_time(ptime t) { start_time = t; }
   void set_end_time(ptime t) { end_time = t; }

   const std::string& get_reg_number() const {return reg_number; }
   AircraftFamily* get_family() const { return family; }
   AircraftModel* get_model() const { return model; }

   const std::string to_string() const { return reg_number + "(" + model->code + ")"; }

   bool is_legal_assignment (const LegCopy* copy) const;
   bool is_legal_assignment(const Leg* leg) const;
   double get_assignment_cost(const LegCopy* copy) const;

   bool is_shuttle() const { return family->code == "TranspCom"; }

   boost::posix_time::time_duration get_min_ground_time() const { 
      return min_ground_time; 
   }

   void set_prev_leg(Leg* l) { prev_leg = l; }
   void set_next_leg(Leg* l) { next_leg = l; }
   Leg* get_prev_leg() const { return prev_leg; }
   Leg* get_next_leg() const { return next_leg; }

   void set_maintenance(Maintenance* m) { maintenance = m; }
   Maintenance* get_maintenance() const { return maintenance; }

   void set_aog_maintenance(Maintenance* a) { aog = a; }
   Maintenance* get_aog_maintenance() const { return aog; }

   double get_orig_crs_speed() const { return Config::instance()->rule_config.orig_cruise_speed_ratio * crf.maxRangeCrs; }
   double get_crs_distance(const Leg* l) const;
   double get_crs_speed(const LegCopy* c) const;
   double get_crs_fuel_consumption(double cruiseDistance, double cruiseSpeed) const;

   boost::posix_time::ptime get_ready_time() const;
};

enum class PositionType
{
   Family,
   Model,
   Config
};

class Position
{
private:
   Station* station;

   PositionType type;			// position type

   AircraftFamily* family;		// aircraft family
   AircraftModel* model;		// aircraft model

   int count;      // desired number of aircrafts at the station

public:
   Position(Station* stn, PositionType t, int cnt = 0, AircraftFamily* fm = nullptr,
      AircraftModel* mdl = nullptr) :
      station(stn),
      type(t),
      family(fm),
      model(mdl),
      count(cnt)
   {}

   Station* get_station() { return station; }
   PositionType get_type() const { return type; }
   AircraftFamily* get_family() { return family; }
   AircraftModel* get_model() { return model; }
   int get_desired_count() const { return count; }
   void set_desired_count(int c) { count = c; }
   void increment_desired_count(int c = 1) { count += c; }

   bool is_aircraft_desired(const Aircraft* aircraft) {
      if (type == PositionType::Family) {
         return aircraft->get_family() == family;
      }
      else if (type == PositionType::Model) {
         return aircraft->get_model() == model;
      }
      return false;
   }

   std::string to_string() const {
      std::string str = station->get_code() + "-" + "Pos";
      if (type == PositionType::Family) {
         str += "Family-" + this->family->code;
      }
      else if (type == PositionType::Model) {
         str += "Model-" + this->model->code;
      }
      str += station->get_code() + "-" + std::to_string(count);
      return str;
   }
};
