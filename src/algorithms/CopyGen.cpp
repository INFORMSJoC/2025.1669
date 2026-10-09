// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "CCG.h"
#include "datamngr.hpp"
#include "solmngr.h"
#include "config.h"
#include "util.h"
#include <omp.h>
#include <limits>

#ifdef _DEBUG
#include <iostream>
#endif

using namespace Timespace;
using namespace CCG;

void Solver::copy_generation()
{
	const Util::Stopwatch timer;
	for (const auto& ac : DataRegistry::instance()->aircrafts)
	{
		aircraft_to_new_copies[ac->get_id()].clear();
	}

	auto max_num_copies = Config::instance()->alg_config.max_num_copies;
	if (Config::instance()->alg_config.dynamic_max_copies)
	{
		max_num_copies = std::max(int(Config::instance()->alg_config.max_num_copies / static_cast<int>(std::pow(2, iteration / 10))),
			Config::instance()->alg_config.min_num_copies);
	}

#pragma omp parallel num_threads(Config::instance()->alg_config.num_threads)
	{
		std::vector<LegCopy> candidate_copies;

#pragma omp for schedule(dynamic)
		for (int idx = 0; idx < DataRegistry::instance()->aircrafts.size(); ++idx)
		{
			const auto aircraft = DataRegistry::instance()->aircrafts[idx].get();
			candidate_copies.clear();

			this->multi_label_algorithm(aircraft, true, true, false);

			this->copy_evaluation_individual(aircraft, candidate_copies);

			this->copy_evaluation_individual_csc(aircraft, candidate_copies);

			if (Config::instance()->alg_config.seq_copy_generation) {
            if (candidate_copies.empty() || candidate_copies.front().get_score() > Config::instance()->alg_config.seq_copy_score_threshold) {
					this->multi_label_algorithm(aircraft, false, true, true);
					this->copy_evaluation_multiple(aircraft, candidate_copies);
				}

				if (Config::instance()->rule_config.use_cruise_control) {
					if (candidate_copies.empty() || candidate_copies.front().get_score() > Config::instance()->alg_config.seq_copy_score_threshold) {
						this->copy_evaluation_multiple_csc_propagation(aircraft, candidate_copies);
					}
					if (gap < Config::instance()->alg_config.seq_csc_copy_gap_threshold &&
						 (candidate_copies.empty() || candidate_copies.front().get_score() >= Config::instance()->alg_config.seq_copy_score_threshold))
					{
						this->copy_evaluation_multiple_csc_solve_minlp(aircraft, candidate_copies);
					}
				}
			}

			std::sort(candidate_copies.begin(), candidate_copies.end(), [](LegCopy& a, LegCopy& b) {
				auto type_a = a.get_leg_copy_type();
				auto type_b = b.get_leg_copy_type();
				if (type_a != type_b) {
					return static_cast<int>(type_a) > static_cast<int>(type_b);
				}
				return a.get_score() < b.get_score();
				});

			// Preserve priority order while deduplicating by content, not shared_ptr identity.
			std::unordered_set<LegCopy, LegCopyHash> seen;
			seen.reserve(candidate_copies.size());
			const auto last = std::remove_if(candidate_copies.begin(), candidate_copies.end(),
				[&](const LegCopy& copy) { return has_copy(aircraft, copy) || !seen.insert(copy).second; });
			candidate_copies.erase(last, candidate_copies.end());

			auto prev_score = Config::instance()->alg_config.score_threshold;
			auto& new_copies = aircraft_to_new_copies.at(aircraft->get_id());
			std::vector<Leg*> legs_been_copied;
			int num_copies = 0;
			for (int i = 0; i < candidate_copies.size() && num_copies < max_num_copies; i++) {
				const auto& copy = candidate_copies[i];
				// check if the leg has been copied
				if (std::any_of(legs_been_copied.begin(), legs_been_copied.end(), [&](Leg* leg) {
					return leg == copy.get_leg();
					}))
				{
					continue;
				}
				legs_been_copied.push_back(copy.get_leg());
				auto new_copy = std::make_shared<LegCopy>(copy.get_leg(), copy.get_dep_time(), copy.get_arr_time(), copy.get_leg_copy_type());
				new_copy->set_score(copy.get_score());
				new_copies.push_back(new_copy);
				if (copy.get_leg_copy_type() == LegCopyType::Individual || copy.get_leg_copy_type() == LegCopyType::CruiseIndividual ||
					 ((copy.get_leg_copy_type() == LegCopyType::Sequential || copy.get_leg_copy_type() == LegCopyType::CruiseSequential) &&
						copy.get_score() != prev_score)) 
				{
					num_copies++;
				}

				prev_score = copy.get_score();
#ifdef _DEBUG
				if (new_copy->get_adj_dep_time(aircraft) >= new_copy->get_adj_arr_time(aircraft)) {
					new_copy->print();
					std::cout << std::endl << new_copy->get_adj_dep_time(aircraft) << std::endl;
					std::cout << new_copy->get_adj_arr_time(aircraft) << std::endl;
				}
#endif
			}
			legs_been_copied.clear();
			candidate_copies.clear();
		}

		// Each iteration owns a different aircraft's copies and network. The omp-for
		// barrier above finishes all evaluations before any network is modified.
#pragma omp for schedule(dynamic)
		for (int idx = 0; idx < DataRegistry::instance()->aircrafts.size(); ++idx) {
			const auto aircraft = DataRegistry::instance()->aircrafts[idx].get();
			network.get_sub_network(aircraft)->update(aircraft_to_new_copies.at(aircraft->get_id()), &path_model);
		}
	}
	SolRegistry::instance()->copyGenTime += timer.seconds();
}

void Solver::multi_label_algorithm(const Aircraft* aircraft, const bool forward, const bool backward, const bool sequential)
{
	const Util::Stopwatch timer;

	const auto sub_network = network.get_sub_network(aircraft);
	const auto p_source = sub_network->get_source_node();
	const auto p_sink = sub_network->get_sink_node();

	sub_network->reset_labels(forward, backward);

	if (forward) {
		for (auto p_arc : p_source->leaving_arcs)
		{
			for (auto& p_label : p_source->fwd_labels) {
				++sub_network->labels_considered;
				label_setting(p_arc, p_label.get(), aircraft, true, false);
			}
		}

		for (auto& node : sub_network->nodes) {
			if (sub_network->is_source_node(node.get())) continue;
			if (node->get_time() < p_source->get_time()) continue;
			for (auto p_arc : node->leaving_arcs)
			{
				for (auto& p_label : node->fwd_labels) {
					++sub_network->labels_considered;
					label_setting(p_arc, p_label.get(), aircraft, true, false);
				}
			}
		}
	}

	if (backward) {
		for (auto p_arc : p_sink->entering_arcs)
		{
			for (auto& p_label : p_sink->bwd_labels) {
				++sub_network->labels_considered;
				label_setting(p_arc, p_label.get(), aircraft, false, sequential);
			}
		}
		for (auto itr = sub_network->nodes.rbegin(); itr != sub_network->nodes.rend(); ++itr) {
			auto node = (*itr).get();
			if (sub_network->is_sink_node(node)) continue;
			if (node->get_time() >= p_sink->get_time())
			{
#ifdef _DEBUG
				std::cout << "Node time conflict in backward labeling: " << node->to_simple_string() << " " << p_sink->to_simple_string() << std::endl;
#endif
				continue;
			}
			for (auto p_arc : node->entering_arcs)
			{
				for (auto& p_label : node->bwd_labels) {
					++sub_network->labels_considered;
					label_setting(p_arc, p_label.get(), aircraft, false, sequential);
				}
			}
		}
	}
	sub_network->label_seconds += timer.seconds();
}

void Solver::label_setting(const Timespace::Arc* arc, const Timespace::Label* const predecessor, const Aircraft* aircraft, const bool forward, const bool sequential)
{
	if (predecessor == nullptr || predecessor->cost == std::numeric_limits<double>::infinity()) return;

	// backtrack all the flight legs to check if the current arc is a duplicate flight leg/maintenance
	if (predecessor != nullptr && predecessor->arc != nullptr && arc->get_leg() != nullptr)
	{
		auto curr_label = predecessor;
		auto leg = arc->get_leg();

		while (curr_label != nullptr && curr_label->arc != nullptr)
		{
			if (curr_label->arc->get_leg() != nullptr && *leg == *curr_label->arc->get_leg())
			{
				return;
			}
			curr_label = curr_label->predecessor;
		}
	}

	boost::posix_time::time_duration allowed_delay = predecessor->allowed_delay;

	if (!forward && sequential) {
		if (arc->is_flight_arc()) {
			boost::posix_time::time_duration allowed_flight_delay = std::max<boost::posix_time::time_duration>(
				arc->get_leg()->get_max_delay() - arc->get_delay(), Util::get_duration_from_minutes(0));
			allowed_delay = std::min<time_duration>(allowed_flight_delay, allowed_delay);
		}
		else {
			allowed_delay += arc->get_duration();
		}
		if (allowed_delay >= boost::posix_time::minutes(1)) {
			const auto sub_network = network.get_sub_network(aircraft);
			if (sub_network->is_sink_node(arc->get_head_node())) {
				allowed_delay -= boost::posix_time::minutes(1);
			}
		}
	}

	auto cost = predecessor->cost + arc->get_cost();
	auto cost_exclude_slots = predecessor->cost_exclude_slots + arc->get_cost_exclude_slots();
	auto node = forward ? arc->get_head_node() : arc->get_tail_node();
	auto& labels = forward ? node->fwd_labels : node->bwd_labels;

	Label new_label(predecessor, arc, cost, cost_exclude_slots, allowed_delay);

	for (auto it = labels.begin(); it != labels.end();) {
		if (Label::check_dominance(&new_label, it->get(), forward, sequential))
		{
			it = labels.erase(it);
		}
		else {
			if (Label::check_dominance(it->get(), &new_label, forward, sequential)) {
				return;
			}
			++it;
		}
	}
	labels.push_back(std::make_unique<Label>(new_label));
}


