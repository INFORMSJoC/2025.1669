// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include "Station.hpp"

enum SlotType
{
   Departure = 1,
   Arrival = 2,
   Mixture = 3
};

class Slot{
private:
   int id;
   ptime start_time;
   ptime end_time;
   Station* station;
   SlotType type;
   int limit;
 
public:
   Slot(int vId, ptime st, ptime et, Station* pStn, SlotType sType, int vLimit)
   : id(vId),
   start_time(st),
   end_time(et),
   station(pStn),
   type(sType),
   limit(vLimit)
   {}

   int get_id() const {return id;}

   std::pair<ptime, ptime> get_slot_interval() const {return std::make_pair(start_time, end_time);}
   Station* get_station() const {return station;}
   SlotType get_slot_type() const {return type;}
   int get_limit() const {return limit;}

   bool is_in_slot(const ptime& time, const Station* station) const {
      if (station != this->station) return false;
      return time >= start_time && time < end_time;
   }

   std::string to_simple_string() const {
      std::string str = "";
      str += station->get_code() + "-";
      str += std::to_string(type) + "-";
      str += "(" + std::to_string(limit) + ")";
      return str;
   }
};