// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "CCG.h"
#include "datamngr.hpp"
#include "config.h"
#include "util.h"
#include "solmngr.h"
#include <set>
#include <iostream>
#include <fstream>

using namespace Timespace;
using namespace CCG;


void Solver::column_generation()
{
	const Util::Stopwatch timer;
	new_paths.clear();

	std::vector<std::shared_ptr<Path>> generated(DataRegistry::instance()->aircrafts.size());
	std::fill(min_rd_cost.begin(), min_rd_cost.end(), 0.0);
#pragma omp parallel num_threads(Config::instance()->alg_config.num_threads)
	{
#pragma omp for schedule(dynamic)
		for (int idx = 0; idx < DataRegistry::instance()->aircrafts.size(); ++idx)
		{
			const auto aircraft = DataRegistry::instance()->aircrafts[idx].get();
			const auto ac_idx = path_model.get_index(aircraft);
			const auto sub_network = network.get_sub_network(aircraft);
			auto source = sub_network->get_source_node();
			auto sink = sub_network->get_sink_node();

			this->multi_label_algorithm(aircraft, true, false, false);

			// find the min-cost label among all fwdLabels of the sink node
			const auto it = std::min_element(sink->fwd_labels.begin(), sink->fwd_labels.end(),
				[](auto& a, auto& b) { return a->cost < b->cost; });
			if (it == sink->fwd_labels.end()) {
				continue;
			}
			auto path_rd_cost = (*it)->cost - path_model.dual_path_select[ac_idx];
			min_rd_cost[idx] = path_rd_cost;
			if (path_rd_cost >= 0) { continue; }

			// backward track to find the most negative rc path
			auto p_path = std::make_shared<Path>(aircraft, 0, path_rd_cost);
			auto curr_label = it->get();
			while ((curr_label->arc != nullptr) && (!sub_network->is_source_node(curr_label->arc->get_head_node())))
			{
				const auto pred_arc = curr_label->arc;
				if (pred_arc->is_flight_arc()) {
					p_path->add_arc(static_cast<FlightArc*>(pred_arc));
				}
				curr_label = curr_label->predecessor;
			}

			std::reverse(p_path->arc_list.begin(), p_path->arc_list.end());
			p_path->compute_lof_cost();

			generated[idx] = std::move(p_path);
		}
	}
	// Deterministic aircraft order, with no shared-container insertion in workers.
	for (auto& path : generated) {
		if (!path) continue;
		path->set_id(static_cast<int>(all_paths.size()));
		new_paths.push_back(path.get());
		all_paths.push_back(std::move(path));
	}

	this->add_columns();
	SolRegistry::instance()->colGenTime += timer.seconds();
}

void Solver::add_columns()
{
	try
	{
		for (auto path : new_paths)
		{
			const auto ac = path->get_aircraft();
			GRBColumn column;
			column.addTerm(1.0, path_model.path_select[path_model.get_index(ac)]);
			std::vector<int> slot_counts(DataRegistry::instance()->slots.size(), 0);
			for (const auto arc : path->arc_list) {
				column.addTerm(1.0, path_model.flight_cover[arc->leg_index]);
				for (int index : arc->slot_indices) ++slot_counts[index];
			}
			for (int i = 0; i < slot_counts.size(); ++i) {
				if (slot_counts[i]) column.addTerm(slot_counts[i], path_model.flow_control[i]);
			}
			// add the column to positioning constraints if the path ends with a desired location
			if (Config::instance()->rule_config.position_rule != PositionRule::Hard) {
				const auto end_station = path->get_destination();
				const auto family_idx = path_model.get_index(end_station, PositionType::Family, ac->get_family());
				if (family_idx >= 0) {
					column.addTerm(1.0, path_model.position_req[family_idx]);
				}
				const auto model_idx = path_model.get_index(end_station, PositionType::Model, ac->get_family(), ac->get_model());
				if (model_idx >= 0) {
					column.addTerm(1.0, path_model.position_req[model_idx]);
				}
			}
			auto new_var = std::make_shared<GRBVar>(path_model.master_mod->addVar(0, GRB_INFINITY,
				path->get_cost(), GRB_CONTINUOUS, column,
				"Assign(" + ac->get_reg_number() + "_" + std::to_string(path->get_id()) + ")"));
			path_model.var_assign.push_back(new_var);
			map_path_vars.emplace(path->get_id(), new_var.get());
		}
	}
	catch (const GRBException& e)
	{
		std::cerr << "Exception caught: " << e.getMessage() << std::endl;
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught!" << std::endl;
	}
}

void Solver::update_bounds()
{
	mp_new_lb = mp_ub;

	for (double reduced_cost : min_rd_cost) {
		mp_new_lb += std::min(reduced_cost, 0.0);
	}

	mp_lb = mp_new_lb;
}

void Solver::write_paths(const std::vector<Path*>& paths) const
{
	std::ofstream file;
	const auto path = Config::instance()->main_config.path_to_output + "copies.out";
	file.open(path, std::ios_base::app);
	if (!file.is_open())
	{
		std::cerr << "Error: Unable to open file " << path << std::endl;
		return;
	}

	for (const auto path : paths)
	{
		file << iteration << "\t" << path->get_aircraft()->get_reg_number() << "\t" << path->get_rd_cost() << "\t";

		for (const auto& arc : path->arc_list)
		{
			file << arc->to_simple_string() << " ";
		}
		file << std::endl;
	}
	file.close();
}