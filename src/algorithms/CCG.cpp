// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "CCG.h"
#include "datamngr.hpp"
#include "solmngr.h"
#include "config.h"
#include "util.h"
#include <set>
#include <omp.h>

using namespace Timespace;
using namespace CCG;

void Solver::run() 
{
	PRINT_SECTION("Copy & Column Generation");
	const Util::Stopwatch timer;

	this->initialize();

	if (Config::instance()->alg_config.print_alg_process)
	{
		std::cout.precision(4);
		std::cout << std::setw(10) << "Iteration" << std::setw(15) << "Upper Bound" << std::setw(15) << "Lower Bound" 
			<< std::setw(15) << "Gap" << std::setw(15) << "Time(sec)" <<  std::endl;
	}

	iteration = 0;
	while (true)
	{
		iteration++;

		path_model.solve();
		SolRegistry::instance()->solveMpTime += path_model.master_mod->get(GRB_DoubleAttr_Runtime);
		if (path_model.master_mod->get(GRB_IntAttr_Status) != GRB_OPTIMAL)
		{
			std::cerr << "No solution found for MP!" << std::endl;
			path_model.master_mod->write(Config::instance()->main_config.path_to_output + "MP-failed.lp");
			exit(EXIT_FAILURE);
		}
		mp_ub = path_model.master_mod->get(GRB_DoubleAttr_ObjVal);

		this->get_mp_duals();

		this->set_network_arc_costs();

		this->copy_generation();

		this->column_generation();

		this->update_bounds();

		gap = Util::compute_gap(mp_ub, mp_lb);
		solve_time = Util::duration_from_seconds(timer.seconds());

		if (Config::instance()->alg_config.print_alg_process)
		{
			std::cout << std::setw(10) << "-" + std::to_string(iteration) + "-"
				<< std::setw(15) << mp_ub << std::setw(15) << mp_lb << std::setw(15) << gap 
				<< std::setw(15) << solve_time.total_seconds() << std::endl;
		}

		if (gap <= Config::instance()->alg_config.gap_tolerance) { break; }
		if (solve_time >= Config::instance()->alg_config.run_time_limit) { break; }
		if (iteration > Config::instance()->alg_config.max_iterations) break;

		this->manage_copies();
	}

	PRINT_SUBSECTION("parsing the recovery solution");
	get_solution();
	auto sol = SolRegistry::instance();
	sol->solve_time = Util::duration_from_seconds(timer.seconds());
	for (const auto& sub : network.sub_networks) {
		sol->solveSppTime += sub->label_seconds;
		sol->updateNetworkTime += sub->update_seconds;
		sol->labels_considered += sub->labels_considered;
		sol->propagation_leaves += sub->propagation_leaves;
		sol->network_nodes += sub->nodes.size();
		sol->minlp_solves += sub->minlp_solves;
	}
}

void Solver::initialize()
{
	const auto& aircrafts = DataRegistry::instance()->aircrafts;
	min_rd_cost.assign(aircrafts.size(), 0.0);
	eligible_legs.resize(aircrafts.size());
	existing_copies.resize(aircrafts.size());
	for (int i = 0; i < aircrafts.size(); ++i) {
		for (auto leg : DataRegistry::instance()->legs) {
			if (aircrafts[i]->is_legal_assignment(leg)) eligible_legs[i].push_back(leg);
		}
	}
	if (Config::instance()->rule_config.use_cruise_control &&
		Config::instance()->alg_config.seq_copy_generation &&
		Config::instance()->alg_config.solve_cruise_time_decisions) {
		// Gurobi environments must be initialized serially and owned by one worker.
		for (int i = 0; i < Config::instance()->alg_config.num_threads; ++i) {
			auto env = std::make_unique<GRBEnv>(true);
			env->set(GRB_IntParam_OutputFlag, 0);
			env->set(GRB_IntParam_Threads, 1);
			env->start();
			pricing_environments.push_back(std::move(env));
		}
	}
	for (const auto& ac : DataRegistry::instance()->aircrafts)
	{
		aircraft_to_all_copies.emplace(std::make_pair(ac->get_id(), std::vector<std::shared_ptr<LegCopy>>()));
		aircraft_to_new_copies.emplace(std::make_pair(ac->get_id(), std::vector<std::shared_ptr<LegCopy>>()));
		aircraft_to_reserve_copies.emplace(std::make_pair(ac->get_id(), std::vector<std::shared_ptr<LegCopy>>()));
	}

	// original leg copy (delay = 0)
	for (auto& ac : DataRegistry::instance()->aircrafts)
	{
		for (auto& leg : DataRegistry::instance()->legs)
		{
			if (ac->is_legal_assignment(leg) && leg->get_adj_arr_time(ac.get()) <= ac->get_end_time()) {
				auto copy = std::make_shared<LegCopy>(leg, leg->get_dep_time(), leg->get_arr_time());
				aircraft_to_all_copies[ac->get_id()].push_back(copy);
			}
		}
	}


   // propagation leg copy
	for (auto& ac : DataRegistry::instance()->aircrafts) {
		auto current_time = ac->get_start_time();
		for (auto p_leg : ac->get_assigned_legs()) {
			const auto max_delay = p_leg->get_max_delay();
			boost::posix_time::time_duration delay = std::max(current_time - p_leg->get_adj_dep_time(), Util::get_duration_from_minutes(0));

			bool is_valid = false;
			// pass only if the delay does not cause any slot violation
			while (!is_valid) {
				auto it = std::find_if(DataRegistry::instance()->slots.begin(), DataRegistry::instance()->slots.end(),
					[&](const std::shared_ptr<Slot>& slot) {
						return slot->get_limit() == 0 && !p_leg->is_maintenance() &&
							((slot->get_slot_type() == SlotType::Departure && slot->is_in_slot(p_leg->get_dep_time() + delay, p_leg->get_origin()))
								|| (slot->get_slot_type() == SlotType::Arrival && slot->is_in_slot(p_leg->get_arr_time() + delay, p_leg->get_destination())));
					});
				if (it != DataRegistry::instance()->slots.end()) {
					if ((*it)->get_slot_type() == SlotType::Departure) {
						delay = (*it)->get_slot_interval().second - p_leg->get_dep_time();
					}
					else {
						delay = (*it)->get_slot_interval().second - p_leg->get_arr_time();
					}
				}
				else {
					is_valid = true;
				}
			}

			if (delay > max_delay) break;
			current_time = p_leg->get_adj_arr_time(ac.get()) + delay;
			if (current_time > ac->get_end_time()) break;
			if (delay == Util::get_duration_from_minutes(0)) continue;

			auto copy = std::make_shared<LegCopy>(p_leg, p_leg->get_dep_time() + delay, p_leg->get_arr_time() + delay, LegCopyType::Propagation);
			aircraft_to_all_copies[ac->get_id()].push_back(copy);
		}
	}
	// initialize network
	network.build(aircraft_to_all_copies, true);

	// creating column generation master problem
	path_model.create_master_problem();
	for (const auto& ac : aircrafts) {
		auto& keys = existing_copies[path_model.get_index(ac.get())];
		for (const auto& copy : aircraft_to_all_copies.at(ac->get_id())) keys.insert(*copy);
		for (const auto& arc : network.get_sub_network(ac.get())->flight_arcs) {
			path_model.initialize_arc_cost(ac.get(), arc.get());
		}
	}

	// generating the first set of columns
	new_paths.clear();
	all_paths.clear();
	map_path_vars.clear();

	// dummy paths
	for (const auto& ac : DataRegistry::instance()->aircrafts) {
      auto dummy_path = std::make_shared<Path>(ac.get(), 0, std::numeric_limits<double>::infinity());
		if (Config::instance()->rule_config.position_rule == PositionRule::Hard && ac->get_start_station() != ac->get_end_station()) {
         dummy_path->set_cost(Config::instance()->rule_config.cost_miss_endpos);
		}
		dummy_path->set_id(int(all_paths.size()));

		all_paths.push_back(dummy_path);
		new_paths.push_back(dummy_path.get());
	}
	this->add_columns();
	new_paths.clear();

	// initial paths containing propagation delay copies
	for (const auto& ac : DataRegistry::instance()->aircrafts)
	{
		const auto sub_network = network.get_sub_network(ac.get());
		auto p_path = std::make_shared<Path>(ac.get(), 0, std::numeric_limits<double>::infinity());
		auto current_station = ac->get_start_station();
      auto current_time = ac->get_start_time();
		for (auto p_leg : ac->get_assigned_legs()) {
			if (p_leg->get_origin() != current_station) break;
			// find the propagation delay copy arc associated with the leg
			auto it = std::find_if(sub_network->flight_arcs.begin(), sub_network->flight_arcs.end(),
				[&](const auto& arc) {
               return arc->get_start_time() >= current_time && arc->get_leg_copy()->get_leg() == p_leg && 
						arc->get_leg_copy()->get_leg_copy_type() == LegCopyType::Propagation;
            });
			if (it == sub_network->flight_arcs.end()) {
				it = std::find_if(sub_network->flight_arcs.begin(), sub_network->flight_arcs.end(),
					[&](const auto& arc) {
						return arc->get_start_time() >= current_time && arc->get_leg_copy()->get_leg() == p_leg && 
							arc->get_leg_copy()->get_leg_copy_type() == LegCopyType::Original;
               });
			}
			if (it == sub_network->flight_arcs.end()) break;
			p_path->add_arc(it->get());
         current_station = p_leg->get_destination();
         current_time = it->get()->get_end_time();
		}

		p_path->set_id(int(all_paths.size()));
		p_path->compute_lof_cost();

		all_paths.push_back(p_path);
		new_paths.push_back(p_path.get());
	}
	this->add_columns();

	new_paths.clear();

	if (Config::instance()->alg_config.write_lp_to_file)
	{
		path_model.master_mod->write(Config::instance()->main_config.path_to_output + "RMP.lp");
	}
}

void Solver::get_mp_duals()
{
	const auto read_duals = [&](GRBConstr* constraints, std::vector<double>& values) {
		if (values.empty()) return;
		std::unique_ptr<double[]> duals(path_model.master_mod->get(GRB_DoubleAttr_Pi, constraints, static_cast<int>(values.size())));
		std::copy_n(duals.get(), values.size(), values.begin());
	};
	read_duals(path_model.path_select, path_model.dual_path_select);
	read_duals(path_model.flight_cover, path_model.dual_flight_cover);
	read_duals(path_model.flow_control, path_model.dual_flow_control);
	read_duals(path_model.position_req, path_model.dual_position_req);
}

void Solver::set_network_arc_costs()
{
	for (const auto& ac : DataRegistry::instance()->aircrafts)
	{
		const auto aircraft = ac.get();
		const auto sub_network = network.get_sub_network(aircraft);
		// assign cost to each arc
		try {
			for (auto& arc : sub_network->flight_arcs) {
				path_model.update_arc_cost(arc.get());
			}

			for (auto& arc : sub_network->ground_arcs) {
				double rc = 0.0;
				const auto head = arc->get_head_node();
				const auto tail = arc->get_tail_node();
				if (head == sub_network->get_sink_node()) {
					const auto station = tail->get_station();

					if (Config::instance()->rule_config.position_rule == PositionRule::Hard) {
						if (station != aircraft->get_end_station()) {
							rc += Config::instance()->rule_config.cost_miss_endpos;
						}
					}
					else {
						const auto pos_family_idx = path_model.get_index(station, PositionType::Family, aircraft->get_family());
						if (pos_family_idx >= 0) {
							rc -= path_model.dual_position_req[pos_family_idx];
						}
						const auto pos_model_idx = path_model.get_index(station, PositionType::Model, aircraft->get_family(), aircraft->get_model());
						if (pos_model_idx >= 0) {
							rc -= path_model.dual_position_req[pos_model_idx];
						}
					}
				}
				arc->set_cost(rc);
				arc->set_cost_exclude_slots(rc);
			}
		}
		catch (const char* msg) {
			std::cerr << msg << std::endl;
			exit(EXIT_FAILURE);
		}
	}
}

void Solver::get_solution()
{
	auto data_reg = DataRegistry::instance();
   auto sol_reg = SolRegistry::instance();

	try
	{
		path_model.num_paths = static_cast<int>(all_paths.size());

		path_model.solve();
		sol_reg->solveMpTime += path_model.master_mod->get(GRB_DoubleAttr_Runtime);
		sol_reg->lp_objective = path_model.master_mod->get(GRB_DoubleAttr_ObjVal);

		path_model.solve(true);
		if (path_model.master_mod->get(GRB_IntAttr_Status) != GRB_OPTIMAL)
		{
			throw std::runtime_error("No solution found for MP!");
		}
		sol_reg->solveIpTime = path_model.master_mod->get(GRB_DoubleAttr_Runtime);
		sol_reg->objective = path_model.master_mod->get(GRB_DoubleAttr_ObjVal);
		sol_reg->gap = Util::compute_gap(sol_reg->objective, sol_reg->lp_objective);

		for (const auto& aircraft : data_reg->aircrafts) {
			sol_reg->map_aircraft_id_to_route.emplace(aircraft->get_id(), std::vector<std::pair<Leg*, boost::posix_time::time_duration>>());
		}

		for (const auto& it : map_path_vars)
		{
			if (it.second->get(GRB_DoubleAttr_X) > 0.99)
			{
				const auto& path = all_paths[it.first];
				for (const auto arc : path->arc_list) {
               const auto leg = arc->get_leg();
					const auto copy = arc->get_leg_copy();
					leg->set_new_aircraft(path->get_aircraft());
					leg->set_new_dep_time(copy->get_dep_time());
					leg->set_new_arr_time(copy->get_arr_time());

					sol_reg->map_aircraft_id_to_route[path->get_aircraft()->get_id()].push_back(std::make_pair(leg, arc->get_delay()));
					sol_reg->map_leg_id_to_assignment.emplace(leg->get_id(), std::make_pair(path->get_aircraft(), arc->get_delay()));
					if (Config::instance()->rule_config.use_cruise_control) {
                  if (copy->get_duration() != leg->get_duration()) {
							sol_reg->accelerated_legs.emplace(leg->get_id(), leg->get_duration() - copy->get_duration());
                  }
					}
				}
			}
		}

		for (int i = 0; i < data_reg->legs.size(); i++)
		{
			if (path_model.var_cancel[i].get(GRB_DoubleAttr_X) > 0.99)
			{
				sol_reg->unassigned_legs.push_back(data_reg->legs[i]);
			}
		}

		for (int i = 0; i < data_reg->positions.size(); i++) {
         sol_reg->position_violations.push_back(static_cast<int>(path_model.var_miss_position[i].get(GRB_DoubleAttr_X)));
		}
	}
	catch (const GRBException& e)
	{
		std::cerr << "Exception caught: " << e.getMessage() << std::endl;
		throw;
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught!" << std::endl;
		throw;
	}
}

void Solver::manage_copies()
{
	// reserve the copies that have been used in new paths
	for (const auto& aircraft : DataRegistry::instance()->aircrafts)
	{
		auto sub_network = network.get_sub_network(aircraft.get());
		const auto aircraft_id = aircraft->get_id();
		auto& new_copies = aircraft_to_new_copies[aircraft_id];
		auto& reserved_copies = aircraft_to_reserve_copies[aircraft_id];
		std::unordered_set<const LegCopy*> used;
		for (const auto path : new_paths) {
			if (path->get_aircraft() != aircraft.get()) continue;
			for (const auto arc : path->arc_list) used.insert(arc->get_leg_copy());
		}
		std::unordered_set<const LegCopy*> discarded;
		auto out = new_copies.begin();
		for (auto it = new_copies.begin(); it != new_copies.end(); ++it) {
			auto& copy = *it;
			if (used.contains(copy.get())) reserved_copies.push_back(std::move(copy));
			else {
				discarded.insert(copy.get());
				if (out != it) *out = std::move(copy);
				++out;
			}
		}
		new_copies.erase(out, new_copies.end());
		if (discarded.empty()) continue;
		auto& arcs = sub_network->flight_arcs;
		arcs.erase(std::remove_if(arcs.begin(), arcs.end(), [&](const auto& arc) {
			return discarded.contains(arc->get_leg_copy());
		}), arcs.end());
		// Retain event times: copy evaluation also uses nodes without incident flight arcs.
		// Deleting them changes the candidate set, so only rebuild the affected topology.
		sub_network->update({}, &path_model, true);
	}

	if (Config::instance()->alg_config.write_copies_to_file)
	{
		DataRegistry::instance()->write_copies(iteration, aircraft_to_new_copies, aircraft_to_reserve_copies, mp_ub);
	}

	// update the pool of all copies and clean the other pools
	for (const auto& aircraft : DataRegistry::instance()->aircrafts)
	{
		for (auto& item : aircraft_to_reserve_copies[aircraft->get_id()])
		{
			existing_copies[path_model.get_index(aircraft.get())].insert(*item);
			aircraft_to_all_copies[aircraft->get_id()].push_back(std::move(item));
		}

		aircraft_to_new_copies[aircraft->get_id()].clear();
		aircraft_to_reserve_copies[aircraft->get_id()].clear();
	}
}
