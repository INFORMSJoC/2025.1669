// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "CCG.h"
#include "datamngr.hpp"
#include "solmngr.h"
#include "config.h"
#include "util.h"
#include <string>
#include <set>
#include <omp.h>

using namespace Timespace;
using namespace CCG;

/* ====================== Evaluate csc individual copies ============================ */
void Solver::copy_evaluation_individual_csc(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies)
{
	if (!Config::instance()->rule_config.use_cruise_control) return;

	const auto sub_network = network.get_sub_network(aircraft);
	const auto& station_nodes_map = sub_network->station_nodes_map;

	for (const auto leg : eligible_legs[path_model.get_index(aircraft)])
	{
		if (leg->is_maintenance())  continue;

		// get the departure station and time window for generating copies
		const auto p_station = leg->get_origin();
		const auto max_delay = leg->get_max_delay();

		// find the first timespace node after the flight within the same departure station
		if (station_nodes_map.find(p_station->get_id()) == station_nodes_map.end()) {
			continue;
		}
		const auto& dep_sta_nodes = station_nodes_map.at(p_station->get_id());
		const auto node_itr = first_at_or_after(dep_sta_nodes, leg->get_dep_time());
		if (node_itr == dep_sta_nodes.end()) {
			continue;
		}

		if (station_nodes_map.find(leg->get_destination()->get_id()) == station_nodes_map.end()) {
			continue;
		}
		const auto& arr_sta_nodes = station_nodes_map.at(leg->get_destination()->get_id());

		for (auto itr = node_itr; itr != dep_sta_nodes.end(); ++itr) {
			const auto delay_node = *itr;
			if (std::all_of(delay_node->fwd_labels.begin(), delay_node->fwd_labels.end(), [](const auto& label) {
				return label->cost >= std::numeric_limits<double>::infinity(); })) continue;  // not accesible from source

			const auto delay_time = delay_node->get_time() - leg->get_dep_time();
			if (delay_time > max_delay) break;

			auto first_arr_sta_node_itr = last_before(arr_sta_nodes, delay_node->get_time() + leg->get_duration() + aircraft->get_min_ground_time());
			if (first_arr_sta_node_itr == arr_sta_nodes.rend()) continue;

			/* ********************************** evaluate CSC Delay Copies to catch next leg ***************************** */
			for (auto arr_itr = first_arr_sta_node_itr; arr_itr != arr_sta_nodes.rend(); arr_itr++) {
				const auto compression = leg->get_duration() - ((*arr_itr)->get_time()
					- aircraft->get_min_ground_time() - delay_node->get_time());
				if (Util::get_minutes_from_duration(compression) == 0) continue;
				if (compression > leg->get_duration() || compression > leg->max_compression()) break;

				const auto arrival_delay = (*arr_itr)->get_time() - leg->get_adj_arr_time(aircraft);
				if (leg->get_adj_arr_time(aircraft) + arrival_delay > aircraft->get_end_time()) break;

				LegCopy delay_copy(leg, delay_node->get_time(), leg->get_arr_time() + arrival_delay, LegCopyType::CruiseIndividual);
				if ((*arr_itr)->bwd_label < std::numeric_limits<double>::infinity()) {
					const auto score = this->scoring_copy(delay_copy, aircraft, *itr, *arr_itr);
					if (score < Config::instance()->alg_config.score_threshold) {
						candidate_copies.push_back(delay_copy);
					}
				}
			}
			/* ******************************** evaluate CSC Delay Copies falling within slots ***************************** */
			// - departure slot: delay leg to the end of the slot (already evaluated above)
			// - arrival slot: compress leg to one sec before the slot
			for (int slot_index : path_model.get_relevant_slots(leg)) {
				const auto& slot = DataRegistry::instance()->slots[slot_index];
				if (slot->get_slot_type() != SlotType::Arrival) continue;
				if (slot->get_station() != leg->get_destination()) continue;
				const auto start_time = slot->get_slot_interval().first;
				const auto new_arr_time = start_time - boost::posix_time::seconds(1);
				const auto slot_node = sub_network->get_node(start_time, slot->get_station());

				LegCopy csc_copy(leg, leg->get_dep_time() + delay_time, new_arr_time, LegCopyType::CruiseIndividual);
				if (csc_copy.get_compression() <= Util::get_duration_from_minutes(0) ||
					csc_copy.get_compression() > leg->max_compression()) continue;
				if (csc_copy.get_delay() > max_delay) continue;
				if (csc_copy.get_adj_arr_time(aircraft) > aircraft->get_end_time()) continue;
				const auto score = this->scoring_copy(csc_copy, aircraft, *itr, slot_node);
				if (score < Config::instance()->alg_config.score_threshold) {
					candidate_copies.push_back(csc_copy);
				}
			}
		}
	}

	std::sort(candidate_copies.begin(), candidate_copies.end(), [](LegCopy& a, LegCopy& b) {
		return a.get_score() < b.get_score();
		});
}

/* ====================== Evaluate csc multiple copies by propagating leg delays ============================ */
void Solver::copy_evaluation_multiple_csc_propagation(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies)
{
	if (!Config::instance()->rule_config.use_cruise_control) return;
	if (!Config::instance()->alg_config.seq_copy_generation) return;

	const auto sub_network = network.get_sub_network(aircraft);
	const auto source = sub_network->get_source_node();
	const auto sink = sub_network->get_sink_node();
	const auto& station_nodes_map = sub_network->station_nodes_map;

	for (const auto leg : eligible_legs[path_model.get_index(aircraft)])
	{
		if (leg->is_maintenance())  continue;

		// get the departure station and time window for generating copies
		const auto p_station = leg->get_origin();
		const auto max_delay = leg->get_max_delay();

		// find the first timespace node after the flight within the same departure station
		if (station_nodes_map.find(p_station->get_id()) == station_nodes_map.end()) {
			continue;
		}
		const auto& dep_sta_nodes = station_nodes_map.at(p_station->get_id());
		const auto node_itr = first_at_or_after(dep_sta_nodes, leg->get_dep_time());
		if (node_itr == dep_sta_nodes.end()) {
			continue;
		}

		if (station_nodes_map.find(leg->get_destination()->get_id()) == station_nodes_map.end()) {
			continue;
		}
		const auto& arr_sta_nodes = station_nodes_map.at(leg->get_destination()->get_id());

		for (auto itr = node_itr; itr != dep_sta_nodes.end(); ++itr) {
			const auto delay_node = *itr;
			if (std::all_of(delay_node->fwd_labels.begin(), delay_node->fwd_labels.end(), [](const auto& label) {
				return label->cost >= std::numeric_limits<double>::infinity(); })) continue;  // not accesible from source

			const auto delay_time = delay_node->get_time() - leg->get_dep_time();
			if (delay_time > max_delay) break;

			const auto first_arr_sta_node_itr = last_before(arr_sta_nodes, delay_node->get_time() + leg->get_duration() + aircraft->get_min_ground_time());
			if (first_arr_sta_node_itr == arr_sta_nodes.rend()) continue;

			/* ********************************** evaluate CSC Delay Copies to catch next leg ***************************** */
			for (auto arr_itr = first_arr_sta_node_itr; arr_itr != arr_sta_nodes.rend(); arr_itr++) {
				const auto compression = leg->get_duration() - ((*arr_itr)->get_time()
					- aircraft->get_min_ground_time() - delay_node->get_time());
				if (Util::get_minutes_from_duration(compression) == 0) continue;
				if (compression > leg->get_duration() || compression > leg->max_compression()) break;

				const auto arrival_delay = (*arr_itr)->get_time() - leg->get_adj_arr_time(aircraft);
				if (leg->get_adj_arr_time(aircraft) + arrival_delay > aircraft->get_end_time()) break;

				LegCopy delay_copy(leg, delay_node->get_time(), leg->get_arr_time() + arrival_delay, LegCopyType::CruiseIndividual);

				for (auto itr2 = first_arr_sta_node_itr; itr2 != arr_sta_nodes.rend(); ++itr2) {
					const auto left_node = *itr2;
					auto required_delay = delay_copy.get_adj_arr_time(aircraft) - left_node->get_time();
					if (required_delay > max_delay) break;

					for (const auto& fwd_label : delay_node->fwd_labels) {
						for (const auto& bwd_label : left_node->bwd_labels) {
							if (required_delay > bwd_label->allowed_delay) continue;
							scoring_propagation_copies(candidate_copies, aircraft, delay_copy, fwd_label.get(), bwd_label.get());
						}
					}
				}
			}
			/* ******************************** evaluate CSC Delay Copies falling within slots ***************************** */
			// - arrival slot: compress leg to one sec before the slot
			for (int slot_index : path_model.get_relevant_slots(leg)) {
				const auto& slot = DataRegistry::instance()->slots[slot_index];
				if (slot->get_slot_type() != SlotType::Arrival) continue;
				if (slot->get_station() != leg->get_destination()) continue;
				const auto start_time = slot->get_slot_interval().first;
				const auto new_arr_time = start_time - boost::posix_time::seconds(1);
				const auto slot_node = sub_network->get_node(start_time, slot->get_station());

				LegCopy csc_copy(leg, leg->get_dep_time() + delay_time, new_arr_time, LegCopyType::Sequential);
				if (csc_copy.get_compression() <= Util::get_duration_from_minutes(0) ||
					csc_copy.get_compression() > leg->max_compression()) continue;
				if (csc_copy.get_delay() > max_delay) continue;
				if (csc_copy.get_adj_arr_time(aircraft) > aircraft->get_end_time()) continue;
				
				for (auto itr2 = first_arr_sta_node_itr; itr2 != arr_sta_nodes.rend(); ++itr2) {
					const auto left_node = *itr2;
					auto required_delay = csc_copy.get_adj_arr_time(aircraft) - left_node->get_time();
					if (required_delay > max_delay) break;

					for (const auto& fwd_label : delay_node->fwd_labels) {
						for (const auto& bwd_label : left_node->bwd_labels) {
							if (required_delay > bwd_label->allowed_delay) continue;
							scoring_propagation_copies(candidate_copies, aircraft, csc_copy, fwd_label.get(), bwd_label.get());
						}
					}
				}
			}
		}
	}

	std::sort(candidate_copies.begin(), candidate_copies.end(), [](LegCopy& a, LegCopy& b) {
		return a.get_score() < b.get_score();
		});
}

/* ====================== Evaluate csc multiple copies by solving minlp model ======================== */
void Solver::copy_evaluation_multiple_csc_solve_minlp(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies)
{
	if (!Config::instance()->alg_config.solve_cruise_time_decisions) return;
	const auto& station_nodes_map = network.get_sub_network(aircraft)->station_nodes_map;

	for (const auto leg : eligible_legs[path_model.get_index(aircraft)])
	{
		const auto MAX_DELAY = leg->get_max_delay();

		/* ********************************** evaluate delay copies (including original leg and existing copies) ***************************** */
		const auto dep_station_nodes_itr = station_nodes_map.find(leg->get_origin()->get_id());
		if (dep_station_nodes_itr == station_nodes_map.end()) continue;
		const auto& dep_station_nodes = dep_station_nodes_itr->second;
		const auto arr_station_nodes_itr = station_nodes_map.find(leg->get_destination()->get_id());
		if (arr_station_nodes_itr == station_nodes_map.end()) continue;
		const auto& arr_station_nodes = arr_station_nodes_itr->second;

		const auto leg_dep_node_itr = first_at_or_after(dep_station_nodes, leg->get_adj_dep_time());

		for (auto itr = leg_dep_node_itr; itr != dep_station_nodes.end(); ++itr) {
			const auto delay_node = *itr;
			const auto delay_time = delay_node->get_time() - leg->get_adj_dep_time();
			if (delay_time > MAX_DELAY) break;
			if (std::all_of(delay_node->fwd_labels.begin(), delay_node->fwd_labels.end(), [](const auto& label) {
				return label->cost >= std::numeric_limits<double>::infinity(); })) continue;   // not accesible from source to delay node

			LegCopy delay_copy(leg, leg->get_dep_time() + delay_time, leg->get_arr_time() + delay_time, LegCopyType::Sequential);

			if (delay_copy.get_adj_earliest_arr_time(aircraft) > aircraft->get_end_time()) break;
			const auto first_arr_node_itr = last_before(arr_station_nodes, delay_copy.get_adj_arr_time(aircraft));
			for (auto itr2 = first_arr_node_itr; itr2 != arr_station_nodes.rend(); ++itr2) {
				const auto left_node = *itr2;
				auto required_delay = delay_copy.get_adj_earliest_arr_time(aircraft) - left_node->get_time();
				if (required_delay > MAX_DELAY) break;

				for (const auto& fwd_label : delay_node->fwd_labels) {
					for (const auto& bwd_label : left_node->bwd_labels) {
						if (required_delay > bwd_label->allowed_delay) continue;
						double score = scoring_csc_copies(candidate_copies, aircraft, delay_copy, fwd_label.get(), bwd_label.get());
						if (!candidate_copies.empty() && score < Config::instance()->alg_config.seq_copy_score_threshold)
						{
							return;
						}
					}
				}
			}
		}
	}

	std::sort(candidate_copies.begin(), candidate_copies.end(), [](LegCopy& a, LegCopy& b) {
		return a.get_score() < b.get_score();
		});
}

double Solver::scoring_csc_copies(std::vector<LegCopy>& candidate_copies, const Aircraft* aircraft, LegCopy& delay_copy,
	const Timespace::Label* fwd_label, const Timespace::Label* bwd_label)
{
	if (!Config::instance()->alg_config.solve_cruise_time_decisions) return 0;
	if (!Config::instance()->alg_config.seq_copy_generation) return 0;
	if (!Config::instance()->rule_config.use_cruise_control) return 0;

	// not accessible from source or to sink
	if (fwd_label->cost >= std::numeric_limits<double>::infinity() || bwd_label->cost >= std::numeric_limits<double>::infinity()) return 0;

	// invalid backward label
	if (bwd_label->arc == nullptr || !bwd_label->arc->is_flight_arc() || bwd_label->arc->get_leg() == delay_copy.get_leg()) return 0;
	const double base_score = fwd_label->cost + path_model.get_new_copy_rd_cost(aircraft, &delay_copy) -
		path_model.dual_path_select[path_model.get_index(aircraft)];
	if (base_score + bwd_label->cost >= Config::instance()->alg_config.score_threshold &&
		base_score + bwd_label->cost_exclude_slots >= Config::instance()->alg_config.score_threshold)
	{
		return 0;
	}

	const auto sub_network = network.get_sub_network(aircraft);
	double score = base_score + bwd_label->cost;

	std::vector<LegCopy> MultiLegCopies;
	auto current = bwd_label;
	if (current->arc == nullptr || !current->arc->is_flight_arc()) return 0;

	std::vector<LegCopy*> copies_to_adjust;
	copies_to_adjust.push_back(&delay_copy);
	while (current->arc != nullptr && current->predecessor != nullptr) {
		const auto successor_arc = current->arc;
		if (successor_arc->is_flight_arc()) {
			copies_to_adjust.push_back(successor_arc->get_leg_copy());
		}
		current = current->predecessor;
	}

	solve_time_decisions_with_minlp(copies_to_adjust, MultiLegCopies, aircraft);

	// calculate score for sequential copy (if any) and add to candidate copies if the score is good
	for (size_t i = 0; i < MultiLegCopies.size(); i++) {
		const auto orig_copy = copies_to_adjust[i];
		const auto new_copy = &(MultiLegCopies[i]);

		const auto cost_delay = Util::get_minutes_from_duration(new_copy->get_dep_time() -
			orig_copy->get_dep_time()) * Config::instance()->rule_config.cost_flt_delay;
		score += cost_delay;

		const auto orig_compression = Util::get_minutes_from_duration(orig_copy->get_leg()->get_duration() - orig_copy->get_duration());
		const auto curr_compression = Util::get_minutes_from_duration(new_copy->get_leg()->get_duration() - new_copy->get_duration());
		score += Config::instance()->rule_config.cost_fuel * (curr_compression * Config::instance()->rule_config.cost_compression_a +
			curr_compression * curr_compression * Config::instance()->rule_config.cost_compression_b);
		score -= Config::instance()->rule_config.cost_fuel * (orig_compression * Config::instance()->rule_config.cost_compression_a +
			orig_compression * orig_compression * Config::instance()->rule_config.cost_compression_b);

		score = path_model.adjust_slot_score(score, orig_copy, new_copy);
	}

	if (score < Config::instance()->alg_config.score_threshold) {
		for (auto& copy : MultiLegCopies) {
			copy.set_score(score);
			copy.set_leg_copy_type(LegCopyType::CruiseSequential);
			candidate_copies.push_back(copy);
		}
	}
	return score;
}

void Solver::solve_time_decisions_with_minlp(const std::vector<LegCopy*>& copies_to_adjust, std::vector<LegCopy>& copies_adjusted,
	const Aircraft* aircraft)
{
	if (!Config::instance()->alg_config.solve_cruise_time_decisions) return;
	const auto dataReg = DataRegistry::instance();
	const auto num_legs = copies_to_adjust.size();
	const auto START_TIME = copies_to_adjust[0]->get_dep_time();

	GRBModel grbModel(*pricing_environments.at(omp_get_thread_num()));
	grbModel.set(GRB_IntParam_OutputFlag, 0);

	GRBQuadExpr objective = 0;
	std::unique_ptr<GRBVar[]> varDepTime(grbModel.addVars(static_cast<int>(num_legs), GRB_CONTINUOUS));
	std::unique_ptr<GRBVar[]> varArrTime(grbModel.addVars(static_cast<int>(num_legs), GRB_CONTINUOUS));
	std::unique_ptr<GRBVar[]> varCompression(grbModel.addVars(static_cast<int>(num_legs), GRB_CONTINUOUS));
	struct SlotDecision {
		int slot_index;
		GRBVar in_slot, left, right;
	};
	std::vector<std::vector<SlotDecision>> slot_decisions(num_legs);
	double independent_slot_cost = 0;
	for (double dual : path_model.dual_flow_control) independent_slot_cost -= std::max(dual, 0.0);
	for (size_t i = 0; i < num_legs; ++i) {
		const auto leg = copies_to_adjust[i]->get_leg();
		// Omitted, unconstrained slot indicators contribute only a constant.
		objective += independent_slot_cost;
		if (leg->is_maintenance()) continue;
		for (int j : path_model.get_relevant_slots(leg)) {
			const auto& slot = dataReg->slots[j];
			const auto window = slot->get_slot_interval();
			if (slot->get_slot_type() == SlotType::Departure) {
				if (leg->get_latest_dep_time() < window.first || leg->get_dep_time() >= window.second) continue;
			}
			else if (slot->get_slot_type() == SlotType::Arrival) {
				if (leg->get_latest_arr_time() < window.first || leg->get_earliest_arr_time() > window.second) continue;
			}
			else continue; // The original time-decision model does not constrain mixture indicators.
			objective += std::max(path_model.dual_flow_control[j], 0.0);
			slot_decisions[i].push_back({j,
				grbModel.addVar(0, 1, 0, GRB_BINARY), grbModel.addVar(0, 1, 0, GRB_BINARY),
				grbModel.addVar(0, 1, 0, GRB_BINARY)});
		}
	}

	try {
		/* objective */
		for (size_t i = 0; i < num_legs; i++) {
			const auto copy = copies_to_adjust[i];
			// delay cost
			objective += (varDepTime[i] - Util::get_minutes_from_duration(copy->get_leg()->get_dep_time() - START_TIME)) * Config::instance()->rule_config.cost_flt_delay;
			// compression cost
			objective += Config::instance()->rule_config.cost_fuel * (varCompression[i] * Config::instance()->rule_config.cost_compression_a +
				varCompression[i] * varCompression[i] * Config::instance()->rule_config.cost_compression_b);
			// reduced cost associated with slots
			for (const auto& decision : slot_decisions[i]) {
				objective -= decision.in_slot * path_model.dual_flow_control[decision.slot_index];
			}
		}
		grbModel.setObjective(objective, GRB_MINIMIZE);

		/* constraints */
		// time relation for start and end of sequential copy arcs
		grbModel.addConstr(varDepTime[0] == 0);
		const auto adjust = copies_to_adjust.back()->get_leg()->is_maintenance() ? 0 : Util::get_minutes_from_duration(aircraft->get_min_ground_time());
		grbModel.addConstr(varArrTime[num_legs - 1] <= Util::get_minutes_from_duration(aircraft->get_end_time() - START_TIME) - adjust);
		for (size_t i = 0; i < num_legs; i++) {
			const auto leg = copies_to_adjust[i]->get_leg();

			GRBLinExpr lhs = varArrTime[i] - varDepTime[i] + varCompression[i];
			double rhs = Util::get_minutes_from_duration(leg->get_duration());

			grbModel.addConstr(lhs == rhs);

			varDepTime[i].set(GRB_DoubleAttr_LB, Util::get_minutes_from_duration(leg->get_dep_time() - START_TIME));
			varDepTime[i].set(GRB_DoubleAttr_UB, Util::get_minutes_from_duration(leg->get_latest_dep_time() - START_TIME));
			varArrTime[i].set(GRB_DoubleAttr_LB, Util::get_minutes_from_duration(leg->get_earliest_arr_time() - START_TIME));
			varArrTime[i].set(GRB_DoubleAttr_UB, Util::get_minutes_from_duration(leg->get_latest_arr_time() - START_TIME));

			varCompression[i].set(GRB_DoubleAttr_LB, 0);
			varCompression[i].set(GRB_DoubleAttr_UB, Util::get_minutes_from_duration(leg->max_compression()));
		}
		// minimum ground time constraints
		for (size_t i = 0; i < num_legs - 1; i++) {
			const auto curr_leg = copies_to_adjust[i]->get_leg();
			const auto next_leg = copies_to_adjust[i + 1]->get_leg();
			const auto mgt = (curr_leg->is_maintenance() || next_leg->is_maintenance()) ?
				0 : Util::get_minutes_from_duration(aircraft->get_min_ground_time());
			GRBLinExpr lhs = varDepTime[i + 1] - varArrTime[i];
			grbModel.addConstr(lhs >= mgt);
		}
		// airport capacity slot constraints
		if (!dataReg->slots.empty()) {
			for (size_t i = 0; i < num_legs; i++) {
				const auto leg = copies_to_adjust[i]->get_leg();
				if (leg->is_maintenance()) continue;
				for (const auto& decision : slot_decisions[i]) {
					const auto& slot = dataReg->slots[decision.slot_index];
					const auto slotWindow = slot->get_slot_interval();
					if (slot->get_slot_type() == SlotType::Departure) {
						if (leg->get_origin() != slot->get_station()) continue;
						if (leg->get_latest_dep_time() < slotWindow.first) continue;
						if (leg->get_dep_time() >= slotWindow.second) continue;
						// within the slot window
						GRBLinExpr lhs = Util::get_minutes_from_duration(slotWindow.first - START_TIME) * decision.in_slot +
							Util::get_minutes_from_duration(leg->get_dep_time() - START_TIME) * (1 - decision.in_slot);
						grbModel.addConstr(lhs <= varDepTime[i]);
						GRBLinExpr rhs = Util::get_minutes_from_duration(slotWindow.second - START_TIME) * decision.in_slot +
							Util::get_minutes_from_duration(leg->get_latest_dep_time() - START_TIME) * (1 - decision.in_slot);
						grbModel.addConstr(varDepTime[i] <= rhs);
						// outside the slot window
						GRBLinExpr rhs2 = (Util::get_minutes_from_duration(slotWindow.first - START_TIME) - 1) * decision.left +
							Util::get_minutes_from_duration(leg->get_latest_dep_time() - START_TIME) * (1 - decision.left);
						grbModel.addConstr(varDepTime[i] <= rhs2);
						GRBLinExpr lhs2 = Util::get_minutes_from_duration(slotWindow.second - START_TIME) * decision.right +
							Util::get_minutes_from_duration(leg->get_dep_time() - START_TIME) * (1 - decision.right);
						grbModel.addConstr(lhs2 <= varDepTime[i]);

						grbModel.addConstr(decision.in_slot + decision.left + decision.right == 1);
					}
					else if (slot->get_slot_type() == SlotType::Arrival) {
						if (leg->get_destination() != slot->get_station()) continue;
						if (leg->get_latest_arr_time() < slotWindow.first) continue;
						if (leg->get_earliest_arr_time() > slotWindow.second) continue;
						GRBLinExpr lhs = Util::get_minutes_from_duration(slotWindow.first - START_TIME) * decision.in_slot +
							Util::get_minutes_from_duration(leg->get_earliest_arr_time() - START_TIME) * (1 - decision.in_slot);
						grbModel.addConstr(lhs <= varArrTime[i]);
						GRBLinExpr rhs = Util::get_minutes_from_duration(slotWindow.second - START_TIME) * decision.in_slot +
							Util::get_minutes_from_duration(leg->get_latest_arr_time() - START_TIME) * (1 - decision.in_slot);
						grbModel.addConstr(varArrTime[i] <= rhs);
						// outside the slot window
						GRBLinExpr rhs2 = (Util::get_minutes_from_duration(slotWindow.first - START_TIME) - 1) * decision.left +
							Util::get_minutes_from_duration(leg->get_latest_arr_time() - START_TIME) * (1 - decision.left);
						grbModel.addConstr(varArrTime[i] <= rhs2);
						GRBLinExpr lhs2 = Util::get_minutes_from_duration(slotWindow.second - START_TIME) * decision.right +
							Util::get_minutes_from_duration(leg->get_earliest_arr_time() - START_TIME) * (1 - decision.right);
						grbModel.addConstr(lhs2 <= varArrTime[i]);

						grbModel.addConstr(decision.in_slot + decision.left + decision.right == 1);
					}
				}
			}
		}
	}
	catch (GRBException& e) {
		std::cout << "Gurobi error: " << e.getMessage() << std::endl;
	}
	catch (...) {
		std::cout << "Other error" << std::endl;
	}

	// solve the model
	grbModel.set(GRB_DoubleParam_TimeLimit, 10);
	grbModel.set(GRB_DoubleParam_MIPGap, 0.01);
	++network.get_sub_network(aircraft)->minlp_solves;
	grbModel.optimize();

	// retrieve the solution
	if (grbModel.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
		for (size_t i = 0; i < num_legs; i++) {
			const auto leg = copies_to_adjust[i]->get_leg();
			const auto dep_time = START_TIME + Util::get_duration_from_minutes(int(varDepTime[i].get(GRB_DoubleAttr_X)));
			const auto arr_time = START_TIME + Util::get_duration_from_minutes(int(varArrTime[i].get(GRB_DoubleAttr_X)));

			LegCopy copy(leg, dep_time, arr_time, LegCopyType::CruiseSequential);
			copies_adjusted.push_back(copy);
		}
	}
	// Variable handle arrays are released by RAII, including exceptional exits.
}
