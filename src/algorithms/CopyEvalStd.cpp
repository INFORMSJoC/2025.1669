// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "CCG.h"
#include "datamngr.hpp"
#include "config.h"
#include "util.h"
#include <string>
#include <set>
#include <omp.h>
#include <limits>

using namespace Timespace;
using namespace CCG;

/* ====================== individual copies ============================ */
void Solver::copy_evaluation_individual(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies)
{
	const auto sub_network = network.get_sub_network(aircraft);
	const auto& station_nodes_map = sub_network->station_nodes_map;

	for (const auto leg : eligible_legs[path_model.get_index(aircraft)])
	{
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

		/* ********************************** evaluate delay copies ***************************** */
		if (station_nodes_map.find(leg->get_destination()->get_id()) == station_nodes_map.end()) {
			continue;
		}
		const auto& arr_sta_nodes = station_nodes_map.at(leg->get_destination()->get_id());
		for (auto itr = node_itr; itr != dep_sta_nodes.end(); ++itr) {
			const auto delay_node = *itr;
			if (delay_node->get_time() <= leg->get_adj_dep_time()) continue;
			if (std::all_of(delay_node->fwd_labels.begin(), delay_node->fwd_labels.end(), [](const auto& label) {
				return label->cost >= std::numeric_limits<double>::infinity(); })) continue;

			const auto delay_time = delay_node->get_time() - leg->get_adj_dep_time();
			if (delay_time > max_delay) break;

			LegCopy delay_copy(leg, leg->get_dep_time() + delay_time, leg->get_arr_time() + delay_time, LegCopyType::Individual);
			const auto right_neighbor_itr = first_at_or_after(arr_sta_nodes, delay_copy.get_adj_arr_time(aircraft));

			if (right_neighbor_itr != arr_sta_nodes.end()) {
				const auto score = this->scoring_copy(delay_copy, aircraft, *itr, *right_neighbor_itr);
				if (score < Config::instance()->alg_config.score_threshold) {
					candidate_copies.push_back(delay_copy);
				}
			}
		}

		/* ********************************** evaluate copies for airport capacity arrival slots ***************************** */
		for (int slot_index : path_model.get_relevant_slots(leg)) {
			const auto& slot = DataRegistry::instance()->slots[slot_index];
			if (slot->get_slot_type() == SlotType::Departure) continue;
			if (slot->get_station() != leg->get_destination()) continue;
			const auto end_time = slot->get_slot_interval().second;
			const auto slot_node = sub_network->get_node(end_time, slot->get_station());
			if ((end_time <= leg->get_arr_time()) || (end_time > leg->get_arr_time() + max_delay)) continue;

			const auto delay_time = end_time - leg->get_arr_time();
			LegCopy delay_copy(leg, leg->get_dep_time() + delay_time, end_time, LegCopyType::Individual);
			if (delay_copy.get_adj_arr_time(aircraft) > aircraft->get_end_time()) continue;
			const auto left_neighbor_itr = last_at_or_before(dep_sta_nodes, delay_copy.get_dep_time());

			if (left_neighbor_itr == dep_sta_nodes.rend()) continue;
			const auto score = this->scoring_copy(delay_copy, aircraft, *left_neighbor_itr, slot_node);

			if (score < Config::instance()->alg_config.score_threshold) {
				candidate_copies.push_back(delay_copy);
			}
		}
	}
	std::sort(candidate_copies.begin(), candidate_copies.end(), [](LegCopy& a, LegCopy& b) {
		return a.get_score() < b.get_score();
		});
}

double Solver::scoring_copy(LegCopy& delay_copy, const Aircraft* aircraft, Timespace::Node* left, Timespace::Node* right)
{
	if (Timespace::SubNetwork::crosses_maintenance(&delay_copy, aircraft->get_maintenance())) return 0;
	if (delay_copy.get_adj_arr_time(aircraft) > aircraft->get_end_time()) return 0;
	if (delay_copy.get_dep_time() < aircraft->get_start_time()) return 0;
	// existing copies are not evaluated again
	if (has_copy(aircraft, delay_copy)) return 0;

	const auto rc = path_model.get_new_copy_rd_cost(aircraft, &delay_copy) - path_model.dual_path_select[path_model.get_index(aircraft)];
	auto legFlyTime = delay_copy.get_duration();

	boost::posix_time::time_duration maxFlyTime = boost::posix_time::hours(10000);
	if (aircraft->get_maintenance() != nullptr) {
		maxFlyTime = aircraft->get_maintenance()->get_remaining_fly_time();
		if (delay_copy.get_arr_time() > aircraft->get_maintenance()->get_dep_time()) {
			legFlyTime = boost::posix_time::seconds(0);
		}
	}

	double score = INFINITY;
	for (const auto& fl : left->fwd_labels) {
		for (const auto& bl : right->bwd_labels) {
			if ((aircraft->get_maintenance() != nullptr) && (fl->fly_time + bl->fly_time + legFlyTime > maxFlyTime)) continue;
			score = std::min<double>(fl->cost + bl->cost + rc, score);
		}
	}
	delay_copy.set_score(score);
	return score;
}

/* ======================= multiple copies ======================== */
void Solver::copy_evaluation_multiple(const Aircraft* aircraft, std::vector<LegCopy>& candidate_copies)
{
	const auto& station_nodes_map = network.get_sub_network(aircraft)->station_nodes_map;

	for (const auto leg : eligible_legs[path_model.get_index(aircraft)])
	{
		const auto MAX_DELAY = leg->get_max_delay();

		/* ********************************** evaluate delay copies ***************************** */
		const auto dep_station_nodes_itr = station_nodes_map.find(leg->get_origin()->get_id());
		if (dep_station_nodes_itr == station_nodes_map.end()) continue;
		const auto& dep_station_nodes = dep_station_nodes_itr->second;
		const auto arr_station_nodes_itr = station_nodes_map.find(leg->get_destination()->get_id());
		if (arr_station_nodes_itr == station_nodes_map.end()) continue;
		const auto& arr_station_nodes = arr_station_nodes_itr->second;

		const auto first_dep_node_itr = first_after(dep_station_nodes, leg->get_adj_dep_time());

		for (auto itr = first_dep_node_itr; itr != dep_station_nodes.end(); ++itr) {
			const auto delay_node = *itr;
			const auto delay_time = delay_node->get_time() - leg->get_adj_dep_time();
			if (delay_time > MAX_DELAY) break;
			if (std::all_of(delay_node->fwd_labels.begin(), delay_node->fwd_labels.end(), [](const auto& label) {
				return label->cost >= std::numeric_limits<double>::infinity(); })) continue;   // not accesible from source to delay node
			// This is the copy to be evaluated
			LegCopy delay_copy(leg, leg->get_dep_time() + delay_time, leg->get_arr_time() + delay_time, LegCopyType::Sequential);

			// stop if the copy arrival time exceeds aircraft end time
			if (delay_copy.get_adj_arr_time(aircraft) > aircraft->get_end_time()) break;

			// FOR ALL ARRIVAL station nodes
			const auto first_arr_node_itr = last_before(arr_station_nodes, delay_copy.get_adj_arr_time(aircraft));
			for (auto itr2 = first_arr_node_itr; itr2 != arr_station_nodes.rend(); ++itr2) {
				const auto left_node = *itr2;
				auto required_delay = delay_copy.get_adj_arr_time(aircraft) - left_node->get_time();
				if (required_delay > MAX_DELAY) break;

				for (const auto& fwd_label : delay_node->fwd_labels) {
					for (const auto& bwd_label : left_node->bwd_labels) {
						if (required_delay > bwd_label->allowed_delay) continue;
						scoring_propagation_copies(candidate_copies, aircraft, delay_copy, fwd_label.get(), bwd_label.get());
					}
				}
			}
		}

		/* ************************* evaluate copies arriving at slot begin and end node *********************** */
		for (int slot_index : path_model.get_relevant_slots(leg)) {
			const auto& slot = DataRegistry::instance()->slots[slot_index];
			if (slot->get_slot_type() == SlotType::Departure) continue;  // (ps: those from departure slots are evaluaved already above)
			if (slot->get_station() != leg->get_destination()) continue;
			const auto end_time = slot->get_slot_interval().second;
			if (end_time < leg->get_arr_time()) continue;
			const auto delay_time = end_time - leg->get_arr_time();
			if (delay_time > MAX_DELAY) continue;
			LegCopy delay_copy(leg, leg->get_dep_time() + delay_time, end_time);
			// stop if the copy arrival time exceeds aircraft end time
			if (delay_copy.get_adj_arr_time(aircraft) > aircraft->get_end_time()) break;
			// discard if the copy already exists
			if (has_copy(aircraft, delay_copy)) continue;

			const auto left_dep_neighbor_itr = last_at_or_before(dep_station_nodes, delay_copy.get_adj_dep_time(aircraft));
			if (left_dep_neighbor_itr == dep_station_nodes.rend()) continue;

			const auto first_arr_node_itr = last_before(arr_station_nodes, delay_copy.get_adj_arr_time(aircraft));
			for (auto itr2 = first_arr_node_itr; itr2 != arr_station_nodes.rend(); ++itr2) {
				const auto left_node = *itr2;
				auto required_delay = delay_copy.get_adj_arr_time(aircraft) - left_node->get_time();
				if (required_delay > MAX_DELAY) break;

				for (const auto& fwd_label : (*left_dep_neighbor_itr)->fwd_labels) {
					for (const auto& bwd_label : left_node->bwd_labels) {
						if (required_delay > bwd_label->allowed_delay) continue;
						scoring_propagation_copies(candidate_copies, aircraft, delay_copy, fwd_label.get(), bwd_label.get());
					}
				}
			}
		}
	}

	std::sort(candidate_copies.begin(), candidate_copies.end(), [](LegCopy& a, LegCopy& b) {
		return a.get_score() < b.get_score();
		});
}

double Solver::scoring_propagation_copies(std::vector<LegCopy>& candidate_copies, const Aircraft* aircraft, LegCopy& delay_copy,
	const Timespace::Label* fwd_label, const Timespace::Label* bwd_label)
{
	if (!Config::instance()->alg_config.seq_copy_generation) return 0;

	// not accessible from source or to sink
	if (fwd_label->cost >= std::numeric_limits<double>::infinity() ||
		bwd_label->cost >= std::numeric_limits<double>::infinity()) return 0;

	// invalid backward label
	if (bwd_label->arc == nullptr || !bwd_label->arc->is_flight_arc() ||
		bwd_label->arc->get_leg() == delay_copy.get_leg()) return 0;

	const double base_score = fwd_label->cost + path_model.get_new_copy_rd_cost(aircraft, &delay_copy) -
		path_model.dual_path_select[path_model.get_index(aircraft)];
	if (base_score + bwd_label->cost >= Config::instance()->alg_config.score_threshold &&
		base_score + bwd_label->cost_exclude_slots >= Config::instance()->alg_config.score_threshold)
	{
		return 0;
	}

	const auto sub_network = network.get_sub_network(aircraft);

	auto current = bwd_label;
	if (current->arc == nullptr || !current->arc->is_flight_arc()) return 0;

	std::vector<LegCopy*> copies_to_adjust;  // the leg copies whose timing will be adjusted 
	copies_to_adjust.push_back(&delay_copy);
	while (current->arc != nullptr && current->predecessor != nullptr) {
		const auto successor_arc = current->arc;
		if (successor_arc->is_flight_arc()) {
			copies_to_adjust.push_back(successor_arc->get_leg_copy());
		}
		current = current->predecessor;
	}

	/* ****************** multiple copies to be evaluated *********************** */
	double best_score = Config::instance()->alg_config.score_threshold;
	auto best_set_copies = std::vector<LegCopy>();
	std::vector<LegCopy> set_copies;
	set_copies.reserve(copies_to_adjust.size());
	best_set_copies.reserve(copies_to_adjust.size());
	depth_first_search_copies(aircraft, best_set_copies, set_copies, copies_to_adjust, 0,
		delay_copy.get_adj_dep_time(aircraft), base_score + bwd_label->cost, best_score,
		sub_network->propagation_leaves);

	for (auto& copy : best_set_copies) {
		copy.set_score(best_score);
		copy.set_leg_copy_type(LegCopyType::Sequential);
		candidate_copies.push_back(copy);
	}

	return best_score;
}

void Solver::depth_first_search_copies(const Aircraft* aircraft, std::vector<LegCopy>& best_copies,
	std::vector<LegCopy>& set_copies, const std::vector<LegCopy*>& copies_to_adjust,
	size_t i, boost::posix_time::ptime new_dep_time, double score, double& best_score, size_t& leaves)
{
	const auto copy_to_adjust = copies_to_adjust[i];
	const time_duration additional_delay = std::max<time_duration>(new_dep_time - copy_to_adjust->get_adj_dep_time(aircraft),
		Util::get_duration_from_minutes(0));
	LegCopy new_copy(copy_to_adjust->get_leg(), copy_to_adjust->get_dep_time() + additional_delay,
		copy_to_adjust->get_arr_time() + additional_delay, LegCopyType::Sequential);
	set_copies.push_back(new_copy);
	// Keep the original arithmetic/DFS order, including strict tie handling.
	if (i > 0) {
		score += Util::get_minutes_from_duration(new_copy.get_dep_time() - copy_to_adjust->get_dep_time()) *
			Config::instance()->rule_config.cost_flt_delay;
		score = path_model.adjust_slot_score(score, copy_to_adjust, &new_copy);
	}
	if (set_copies.size() == copies_to_adjust.size()) {
		++leaves;
		if (score < best_score) {
			best_score = score;
			best_copies = set_copies;
		}
	}
	else {
		const auto next_leg = copies_to_adjust[i + 1]->get_leg();
		const auto ready_time = new_copy.get_adj_arr_time(aircraft);
		// Slot boundaries depend on the scheduled leg, not on the DFS branch.
		auto next_times = path_model.get_propagation_times(next_leg);
		const auto pos = std::lower_bound(next_times.begin(), next_times.end(), ready_time);
		if (pos == next_times.end() || *pos != ready_time) next_times.insert(pos, ready_time);
		for (const auto& t : next_times) {
			if (t < ready_time || t < next_leg->get_adj_dep_time()) continue;
			if (t - next_leg->get_adj_dep_time() > next_leg->get_max_delay()) continue;
			if (t + next_leg->get_duration() > aircraft->get_end_time()) continue;
			depth_first_search_copies(aircraft, best_copies, set_copies, copies_to_adjust, i + 1,
				t, score, best_score, leaves);
		}
	}
	set_copies.pop_back();
}
