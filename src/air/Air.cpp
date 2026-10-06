// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "Aircraft.hpp"
#include "Flight.hpp"
#include "utility/datamngr.hpp"
#include "timespace.hpp"

bool Aircraft::is_legal_assignment(const LegCopy* copy) const
{
   if (maintenance != nullptr && Timespace::SubNetwork::crosses_maintenance(copy, maintenance)) {
      return false;
   }
   if (aog != nullptr && Timespace::SubNetwork::crosses_maintenance(copy, aog)) {
      return false;
   }
   if (copy->get_adj_arr_time(this) > this->get_end_time())
   {
      return false;
   }
   if(copy->get_delay() > copy->get_leg()->get_max_delay())
   {
      return false;
   }
   return is_legal_assignment(copy->get_leg());
}

bool Aircraft::is_legal_assignment(const Leg* leg) const
{
   // maintenance not allowed to be swapped among aircraft
   if (leg->is_maintenance() && leg->get_aircraft() != this) {
      return false;
   }

   // swap only allowed between aircraft of the same family
   const auto& orig_family = leg->get_aircraft()->get_family();
   if (family != orig_family) {
      return false;
   }

   // shuttle not considered for swap
   if (is_shuttle() && get_id() != leg->get_aircraft()->get_id()) {
      return false;
   }

   return true;
}

double Aircraft::get_assignment_cost(const LegCopy* copy) const {
   const auto leg = copy->get_leg();
   const auto is_maint = leg->is_maintenance();

   double cost = 0;
   const auto sch_aircraft = leg->get_aircraft();
   if (sch_aircraft != this) {
      cost += Config::instance()->rule_config.cost_swap;
   }
   const auto delay = double(Util::get_minutes_from_duration(copy->get_delay()));
   const auto unit_delay_cost = is_maint ? Config::instance()->rule_config.cost_mtc_delay :
      Config::instance()->rule_config.cost_flt_delay;
   cost += delay * unit_delay_cost;

   if (Config::instance()->rule_config.use_cruise_control) {
      if (copy->get_duration() != leg->get_duration()) {
         const auto compression = Util::get_minutes_from_duration(leg->get_duration() - copy->get_duration());
         cost += Config::instance()->rule_config.cost_fuel * (compression * Config::instance()->rule_config.cost_compression_a +
            double(compression) * compression * Config::instance()->rule_config.cost_compression_b);
      }
   }

   return cost;
}

double Aircraft::get_crs_distance(const Leg* leg) const {
   return this->get_orig_crs_speed() * Util::get_minutes_from_duration(leg->get_duration());
}

double Aircraft::get_crs_speed(const LegCopy* copy) const {
   return this->get_crs_distance(copy->get_leg()) / Util::get_minutes_from_duration(copy->get_duration());
}

double Aircraft::get_crs_fuel_consumption(double cruise_distance, double cruise_speed) const {
   return cruise_distance * (crf.c1 * pow(cruise_speed, 2.0) + crf.c2 * cruise_distance + crf.c3 / pow(cruise_speed, 2.0) + crf.c4 / pow(cruise_speed, 3.0));
}

boost::posix_time::ptime Aircraft::get_ready_time() const
{
   if (prev_leg == nullptr) {
      return start_time;
   }
   else {
      return std::max(prev_leg->get_arr_time(), DataRegistry::instance()->recovery_start);
   }
}

boost::posix_time::time_duration Leg::get_max_delay() const
{
   if (this->is_aog_maintenance()) {
      return Util::get_duration_from_minutes(0);
   }
   else if (this->is_maintenance())
   {
      return Config::instance()->rule_config.max_mtc_delay;
   }
   else
   {
      return std::min(Config::instance()->rule_config.max_flt_delay,
         DataRegistry::instance()->recovery_end - this->get_arr_time());
   }
}
