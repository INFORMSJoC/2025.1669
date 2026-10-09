// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include "Station.hpp"
#include "Aircraft.hpp"
#include "Slot.hpp"
#include "util.h"
#include <iostream>

using namespace boost::posix_time;

class Slot;

class Flight {
private:
	int flt_number;

	std::tm dep_time;
	std::tm arr_time;
	bool arrNextDay;		// whether the flight arrives next day

	Station* origin;
	Station* destination;

	Flight* prev_flight;		// the precedding flight, when it is a part of a fixed link

public:
	Flight(int _fltNumber, Station* _depStation, Station* _arrStation, std::tm _depTime, std::tm _arrTime, bool _arrNextDay = false,
		Flight* _prevFlight = nullptr) :
		flt_number(_fltNumber),
		origin(_depStation),
		destination(_arrStation),
		dep_time(_depTime),
		arr_time(_arrTime),
		arrNextDay(_arrNextDay),
		prev_flight(_prevFlight)
	{};

	bool operator==(const Flight& flt) const {
		if (this->flt_number == flt.flt_number) {
			return true;
		}
		return false;
	}

	int get_flight_number() const { return flt_number; }
	std::tm get_dep_time() const { return dep_time; }
	std::tm get_arr_time() const { return arr_time; }
	bool is_arr_next_day() const { return arrNextDay; }
	Station* get_origin() const { return origin; }
	Station* get_destination() const { return destination; }
	Flight* get_prev_flight() const { return prev_flight; }
};

enum Disruption 
{
	NoDisruption = 0,
	FlightDelay = 1,
	FlightCancel = 2,
	AircraftUnavailable = 3,
	AirportRestriction = 4,
	MisConnect = 5
};

class Leg {
protected:
	unsigned leg_id; 

	Flight* flight;
	Aircraft* aircraft;		// the aircraft that was originally assigned to the scheduled flight
	Aircraft* new_aircraft;	//  the aircraft that was assigned to the flight in the recovery solution

	ptime dep_time;			// actual departure time takin into account the primary delay of the flight
	ptime arr_time;

	ptime sch_dep_time;
	ptime sch_arr_time;

	ptime new_dep_time;			// recovered departure time in zulu time
	ptime new_arr_time;			// recovered arrival time in zulu time

	// disruption related data
	Disruption disruption;
	time_duration primary_delay;

public:
	Leg(unsigned _id, Flight* _flight, Aircraft* _aircraft, ptime sch_dep_time, ptime sch_arr_time) :
		leg_id(_id),
		flight(_flight),
		aircraft(_aircraft),
		dep_time(sch_dep_time),
		arr_time(sch_arr_time),
		sch_dep_time(sch_dep_time),
		sch_arr_time(sch_arr_time),
		new_dep_time(sch_dep_time),
      new_arr_time(sch_arr_time),
		disruption(Disruption::NoDisruption),
		primary_delay(Util::get_duration_from_minutes(0)),
		new_aircraft(nullptr)
	{
	}

	Leg(): leg_id(0), flight(nullptr), aircraft(nullptr), new_aircraft(nullptr),
		disruption(Disruption::NoDisruption), primary_delay(Util::get_duration_from_minutes(0))
	{
   }

   virtual ~Leg() {
	}

   bool operator==(const Leg& leg) const {
		if (this->is_maintenance() != leg.is_maintenance()) return false;
		if (this->is_maintenance()) {
			return this->get_origin() == leg.get_origin() && this->aircraft == leg.get_aircraft() && this->dep_time == leg.dep_time;
		}
      else if (*(this->flight) == *(leg.flight) && this->dep_time == leg.dep_time) {
         return true;
      }
      return false;
   }

	ptime get_arr_time() const { return arr_time; }
	ptime get_dep_time() const { return dep_time; }
	ptime get_sch_arr_time() const { return sch_arr_time; }
	ptime get_sch_dep_time() const { return sch_dep_time; }

   void set_new_aircraft(Aircraft* ac) { new_aircraft = ac; }

	ptime get_new_dep_time() const { return new_dep_time; }
	ptime get_new_arr_time() const { return new_arr_time; }
	void set_new_dep_time(ptime t) { new_dep_time = t; }
	void set_new_arr_time(ptime t) { new_arr_time = t; }

	virtual unsigned get_id() const { return leg_id; }

	boost::posix_time::time_duration get_duration() const {
		auto duration = arr_time - dep_time;
		return Util::get_duration_from_minutes(int(duration.total_seconds() / 60));
	}

	boost::posix_time::time_duration get_crs_time() const {
		return this->get_duration() - Config::instance()->rule_config.non_cruise_time;
	}

	Aircraft * get_aircraft() const { return aircraft; }
	Aircraft* get_new_aircraft() const { return new_aircraft; }

	Disruption get_disruption() const { return disruption; }
	void set_disruption(Disruption d) { disruption = d; }

	time_duration get_primary_delay() const { return primary_delay; }
	void set_primary_delay(time_duration pd) { primary_delay = pd; }

	void set_dep_time(ptime dt) { dep_time = dt; }
	void set_arr_time(ptime at) { arr_time = at; }

	static bool compare_dep_time(Leg * a, Leg * b)
	{
		return (a->get_dep_time() < b->get_dep_time());
	};

	static bool compare_sch_dep_time(Leg* a, Leg* b)
	{
		return (a->get_sch_dep_time() < b->get_sch_dep_time());
   };

	virtual bool is_maintenance() const { return false; }
	virtual bool is_aog_maintenance() const { return false; }
	virtual bool is_retimeable() const { return true; }

	virtual int get_flight_number() const { return flight->get_flight_number(); }

	virtual Station* get_destination() const { return flight->get_destination(); }
	virtual Station* get_origin() const { return flight->get_origin(); }

	virtual ptime get_adj_dep_time() const { return dep_time; }
	virtual ptime get_adj_arr_time(const Aircraft* aircraft) const { return arr_time + aircraft->get_min_ground_time(); }

	boost::posix_time::time_duration get_max_delay() const;

	boost::posix_time::time_duration max_compression() const {
		if (this->is_maintenance() || !Config::instance()->rule_config.use_cruise_control) {
			return Util::get_duration_from_minutes(0);
		}
		const auto cruise_time = this->get_duration() - Config::instance()->rule_config.non_cruise_time;
		double compressionRate = 1.0 - double(Config::instance()->rule_config.orig_cruise_speed_ratio / Config::instance()->rule_config.max_cruise_speed_ratio);
		return Util::get_duration_from_minutes(int(Util::get_minutes_from_duration(cruise_time) * compressionRate));
	}

	boost::posix_time::time_duration get_min_duration() const {		// min limit of block time while using cruise speed change
		return this->get_duration() - this->max_compression();
	}

	boost::posix_time::ptime get_earliest_dep_time() const
	{
		return dep_time;
	}

	boost::posix_time::ptime get_latest_dep_time() const
	{
		return std::max<ptime>(dep_time, sch_dep_time + get_max_delay());
	}

	boost::posix_time::ptime get_earliest_arr_time() const
	{
		return std::min<ptime>(arr_time, this->get_earliest_dep_time() + this->get_min_duration());
	}

	boost::posix_time::ptime get_latest_arr_time() const
	{
		return this->get_latest_dep_time() + this->get_duration();
	}

	virtual bool is_in_slot(const Slot* slot) const	// whether a leg is located in the slot
	{
		if (aircraft == nullptr) return false;
		if (this->is_maintenance()) return false;
		if (aircraft->is_shuttle()) return false;

		const auto start_time = slot->get_slot_interval().first;
		const auto end_time = slot->get_slot_interval().second;
		const auto station = slot->get_station();

		const auto origin = get_origin();
		const auto destination = get_destination();

		switch (slot->get_slot_type()) {
		case SlotType::Departure:
			return ((origin == station) && (dep_time >= start_time) && (dep_time < end_time));
		case SlotType::Arrival:
			return ((destination == station) && (arr_time >= start_time) && (arr_time < end_time));
		case SlotType::Mixture:
			return (((destination == station) && (arr_time >= start_time) && (arr_time < end_time)) ||
				((origin == station) && (dep_time >= start_time) && (dep_time < end_time)));
		}

		return false;
	}

	bool is_in_slot(const Slot* slot, boost::posix_time::time_duration delay) const
	{
		if (this->get_aircraft() == nullptr) return false;
		if (this->get_aircraft()->is_shuttle()) return false;

		const auto start_time = slot->get_slot_interval().first;
		const auto end_time = slot->get_slot_interval().second;
		const auto station = slot->get_station();

		const auto origin = get_origin();
		const auto destination = get_destination();

		switch (slot->get_slot_type()) {
		case SlotType::Departure:
			return ((origin == station) && (dep_time + delay >= start_time) && (dep_time + delay < end_time));
		case SlotType::Arrival:
			return ((destination == station) && (arr_time + delay >= start_time) && (arr_time + delay < end_time));
		case SlotType::Mixture:
			return (((destination == station) && (arr_time + delay >= start_time) && (arr_time + delay < end_time)) ||
				((origin == station) && (dep_time + delay >= start_time) && (dep_time + delay < end_time)));
		}

		return false;
	}

	virtual std::string to_simple_string() const { return std::to_string(get_flight_number()) + "-" + Util::ptime_to_datestr(dep_time); }

	virtual std::string to_string(bool use_new_time = false) const {
		std::string res = "";
		const auto origin = this->get_origin();
      const auto destination = this->get_destination();
		if (!use_new_time) {
			res += std::to_string(this->get_flight_number());
			res += "(";
			res += Util::ptime_to_datestr(sch_dep_time);
			res += origin->get_code();
			res += Util::ptime_to_clock(sch_dep_time);

			const auto delay = Util::get_minutes_from_duration(primary_delay);
			if (delay > 0) {
				res += "+";
				res += std::to_string(delay);
			}

			res += "---";
			res += destination->get_code();
			res += Util::ptime_to_clock(sch_arr_time);

			if (delay > 0) {
				res += "+";
				res += std::to_string(delay);
			}

			res += ")";
		}
		else {
			const auto delay = Util::get_minutes_from_duration(new_dep_time - dep_time);
			const auto compression = Util::get_minutes_from_duration(this->get_duration() - (new_arr_time - new_dep_time));
			res += std::to_string(this->get_flight_number()) + "-d" + std::to_string(delay);
			if (compression > 0) {
				res += "-c" + std::to_string(compression);
         }
			res += "(";
			res += Util::ptime_to_datestr(new_dep_time);
			res += origin->get_code();
			res += Util::ptime_to_clock(new_dep_time);
			res += "---";
			res += destination->get_code();
			res += Util::ptime_to_clock(new_arr_time);
			res += ")";
		}
		return res;
	}
};

class Maintenance : public Leg {
private:
	Station* location;

	bool isRetime; 		// whether it can be retime
	bool isAog;

	time_duration remainingFlyTime;

public:
	Maintenance(unsigned _id, Station* _location, Aircraft* _aircraft, ptime _startTime, ptime _endTime, 
		time_duration _remainingFlyTime, bool _isAog=false, bool _isRetime=false) :
		location(_location),
		remainingFlyTime(_remainingFlyTime),
		isAog(_isAog),
		isRetime(_isRetime)
	{
		leg_id = _id;
		aircraft = _aircraft;
		dep_time = _startTime;
		arr_time = _endTime;
      new_dep_time = _startTime;
      new_arr_time = _endTime;
		sch_dep_time = _startTime;
		sch_arr_time = _endTime;

		if (isAog) {
			isRetime = false;
		}
	}

	std::string to_simple_string() const override { return aircraft->get_reg_number() + "_" + location->get_code() + "-" + 
		Util::ptime_to_clock(dep_time) + "/" + Util::ptime_to_clock(arr_time); }

	int get_flight_number() const override { return -1; }
	ptime get_adj_dep_time() const override { return dep_time + aircraft->get_min_ground_time(); }
	ptime get_adj_arr_time(const Aircraft* a = nullptr) const override { return arr_time; }

	void set_location(Station* s) { location = s; }
	Station* get_location() const { return location; }
	Station* get_destination() const override { return location; }
	Station* get_origin() const override { return location; }

	bool is_in_slot(const Slot* slot) const override { return false; }
	bool is_maintenance() const override { return true; }
	bool is_aog_maintenance() const override { return isAog; }

	time_duration get_remaining_fly_time() const { return remainingFlyTime; }
	void set_remaining_fly_time(time_duration r) { remainingFlyTime = r; }

	bool operator==(const Maintenance& mtc) const {
		if (this->location == mtc.get_location() && this->aircraft == mtc.get_aircraft() && this->dep_time == mtc.dep_time) {
			return true;
		}
		return false;
	}

	std::string to_string(bool use_new_time = false) const override {
		std::string res = this->to_simple_string();
		if (!use_new_time) {
			res += "(" + Util::ptime_to_datestr(dep_time)
				+ location->get_code()
				+ Util::ptime_to_clock(dep_time) + "---"
				+ Util::ptime_to_datestr(arr_time) + " "
				+ Util::ptime_to_clock(arr_time) + ")";
		}
		else {
			const auto delay = Util::get_minutes_from_duration(new_dep_time - dep_time);
			if (delay >= 0) {
				res += "-d" + std::to_string(delay);
			}
			else {
				res += "-p" + std::to_string(-delay);
			}
			res += "(" + Util::ptime_to_datestr(new_dep_time)
				+ location->get_code()
				+ Util::ptime_to_clock(new_dep_time) + "---"
				+ Util::ptime_to_datestr(new_arr_time) + " "
				+ Util::ptime_to_clock(new_arr_time) + ")";
		}
		return res;
	}
};

enum LegCopyType {
   Original,
   Propagation,
   Individual,
   Sequential,
	CruiseIndividual,
	CruiseSequential
};


class LegCopy
{
private:
	Leg* leg;

	ptime dep_time;
	ptime arr_time;

	double score;

   LegCopyType type;

public:
	LegCopy(const Leg* _leg, ptime _depTime, ptime _arrTime, LegCopyType _type = LegCopyType::Original) :
		leg(const_cast<Leg*>(_leg)),
		dep_time(_depTime),
		arr_time(_arrTime),
		score(0.0),
      type(_type)
	{};

	LegCopy(const LegCopy* copy)
	{
		leg = const_cast<Leg*>(copy->get_leg()); 
		dep_time = copy->dep_time;
		arr_time = copy->arr_time;
		score = copy->score;
		type = copy->type;
	}

	~LegCopy() 
	{}

	bool operator==(const LegCopy& leg) const {
		return (*(this->get_leg()) == *(leg.get_leg()) && 
			this->get_dep_time() == leg.get_dep_time() && this->get_arr_time() == leg.get_arr_time());
	}

	Leg* get_leg() const { return leg; }

	int get_flight_number() const { return leg->get_flight_number(); }

	Station* get_destination() const { return leg->get_destination(); }
	Station* get_origin() const { return leg->get_origin(); }

	ptime get_dep_time() const { return dep_time; }
	ptime get_arr_time() const { return arr_time; }

	ptime get_adj_dep_time(const Aircraft* ac) const { return leg->is_maintenance() ? dep_time + ac->get_min_ground_time(): dep_time; }
	ptime get_adj_arr_time(const Aircraft* ac) const { return leg->is_maintenance() ? arr_time : arr_time + ac->get_min_ground_time(); }
	
   ptime get_adj_earliest_arr_time(const Aircraft* ac) const { 
      const auto mgt = leg->is_maintenance() ? Util::get_duration_from_minutes(0) : ac->get_min_ground_time();
      return leg->get_earliest_arr_time() + this->get_delay() + mgt;
	}

	ptime get_sch_arr_time() const { return leg->get_sch_arr_time(); }
	ptime get_sch_dep_time() const { return leg->get_sch_dep_time(); }

	boost::posix_time::time_duration get_duration() const {
		auto duration = arr_time - dep_time;
		return Util::get_duration_from_minutes(int(duration.total_seconds() / 60));
	}

	Aircraft* get_aircraft() const { return  leg->get_aircraft(); }

	boost::posix_time::time_duration get_delay() const {
		return dep_time - leg->get_dep_time(); 
	}

	boost::posix_time::time_duration get_compression() const {
      return leg->get_duration() - this->get_duration();
	}

	static bool compare_dep_time(LegCopy* a, LegCopy* b)
	{
		return (a->get_dep_time() < b->get_dep_time());
	};

	void set_score(double _s) { score = _s; }
	double get_score() const { return score; }

   void set_leg_copy_type(LegCopyType t) { type = t; }
   LegCopyType get_leg_copy_type() const { return type; }

	bool is_in_slot(const Slot* slot) const {
		const auto schLeg = this->get_leg();
		if (schLeg->is_maintenance()) return false;
		if (schLeg->get_aircraft() == nullptr) return false;
		if (schLeg->get_aircraft()->is_shuttle()) return false;

		const auto start_time = slot->get_slot_interval().first;
		const auto end_time = slot->get_slot_interval().second;
		const auto station = slot->get_station();

		const auto dep_time = this->get_dep_time();
		const auto arr_time = this->get_arr_time();
		const auto origin = this->get_origin();
		const auto destination = this->get_destination();

		switch (slot->get_slot_type()) {
		case SlotType::Departure:
			return ((origin == station) && (dep_time >= start_time) && (dep_time < end_time));
		case SlotType::Arrival:
			return ((destination == station) && (arr_time >= start_time) && (arr_time < end_time));
		case SlotType::Mixture:
			return (((destination == station) && (arr_time >= start_time) && (arr_time < end_time)) ||
				((origin == station) && (dep_time >= start_time) && (dep_time < end_time)));
		}

		return false;
   }

	void print(bool print_score = true) const {
		const std::map<LegCopyType, std::string> LegCopyTypeMap{ {LegCopyType::Original, "-od"},
												{LegCopyType::Propagation, "-pd"},
												{LegCopyType::Individual, "-id"},
												{LegCopyType::Sequential, "-sd"},
												{LegCopyType::CruiseIndividual, "-ci"},
												{LegCopyType::CruiseSequential, "-cs"} };

		std::cout << this->get_flight_number() << LegCopyTypeMap.at(this->get_leg_copy_type());
		std::cout << Util::get_minutes_from_duration(this->get_delay());
		if (Config::instance()->rule_config.use_cruise_control && !this->get_leg()->is_maintenance() && this->get_leg()->get_duration() != this->get_duration()) {
         std::cout << "-c" << Util::get_minutes_from_duration(this->get_leg()->get_duration() - this->get_duration());
		}
		std::cout << "[" << this->get_origin()->get_code() << "(";
		std::cout << this->get_dep_time() << ")---";
		std::cout << this->get_destination()->get_code() << "(";
		std::cout <<this->get_arr_time() << ")]";
		if (print_score) {
			std::cout << " ===> Score=" << this->get_score();
		}
	}
};

// Equality intentionally follows LegCopy::operator==, including equivalent Leg objects.
struct LegCopyHash {
   size_t operator()(const LegCopy& copy) const {
      const auto leg = copy.get_leg();
      size_t h = leg->is_maintenance() ? std::hash<Aircraft*>{}(leg->get_aircraft()) :
         std::hash<int>{}(leg->get_flight_number());
      const auto combine = [&](size_t value) { h ^= value + 0x9e3779b9 + (h << 6) + (h >> 2); };
      if (leg->is_maintenance()) combine(std::hash<Station*>{}(leg->get_origin()));
      const ptime epoch(boost::gregorian::date(1970, 1, 1));
      combine(std::hash<long long>{}((leg->get_dep_time() - epoch).ticks()));
      combine(std::hash<long long>{}((copy.get_dep_time() - epoch).ticks()));
      combine(std::hash<long long>{}((copy.get_arr_time() - epoch).ticks()));
      return h;
   }
};


