// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "network/timespace.hpp"
#include "utility/config.h"
#include "utility/datamngr.hpp"
#include <PathModel.h>
#include <set>

using namespace Timespace;

bool SubNetwork::crosses_maintenance(const Arc* arc, const Maintenance* maintenance)
{
   if (maintenance == nullptr)
   {
      return false;
   }

   if(maintenance->get_adj_dep_time() > arc->get_start_time() &&
      maintenance->get_adj_dep_time() < arc->get_end_time())
   {
      return true;
   }
   if(maintenance->get_arr_time() > arc->get_start_time() &&
      maintenance->get_arr_time() < arc->get_end_time())
   {
      return true;
   }
   if(maintenance->get_adj_dep_time() < arc->get_start_time() &&
      maintenance->get_arr_time() > arc->get_end_time())
   {
      return true;
   }
   return false;
}

bool SubNetwork::crosses_maintenance(const LegCopy* copy, const Maintenance* maintenance)
{
   if (maintenance == nullptr)
   {
      return false;
   }
   const auto aircraft = maintenance->get_aircraft();
   if(maintenance->get_adj_dep_time() > copy->get_adj_dep_time(aircraft) &&
      maintenance->get_adj_dep_time() < copy->get_adj_arr_time(aircraft))
   {
      return true;
   }
   if(maintenance->get_arr_time() > copy->get_adj_dep_time(aircraft) &&
      maintenance->get_arr_time() < copy->get_adj_arr_time(aircraft))
   {
      return true;
   }
   if(maintenance->get_adj_dep_time() < copy->get_adj_dep_time(aircraft) &&
      maintenance->get_arr_time() > copy->get_adj_arr_time(aircraft))
   {
      return true;
   }
   return false;
}

void SubNetwork::update(const std::vector<std::shared_ptr<LegCopy>>& new_copies, const PathModel* path_model, bool topology_changed)
{
   if (new_copies.empty() && !topology_changed) return;
   const Util::Stopwatch timer;
   const auto aircraft = this->get_aircraft();
   // remove old ground arcs while keeping flight arcs
   this->ground_arcs.clear();

   // clear node arc relations
   for (auto& node : this->nodes) {
      node->leaving_arcs.clear();
      node->entering_arcs.clear();
   }

   // create new flight arcs and nodes based on new leg copies
   for (const auto& leg_copy : new_copies) {
      if (!aircraft->is_legal_assignment(leg_copy.get())) continue;
      auto dep_node = this->get_or_create_node(leg_copy->get_adj_dep_time(aircraft), leg_copy->get_origin());
      auto arr_node = this->get_or_create_node(leg_copy->get_adj_arr_time(aircraft), leg_copy->get_destination());
      auto flight_arc = std::make_shared<FlightArc>(dep_node, arr_node, leg_copy.get());
      path_model->initialize_arc_cost(aircraft, flight_arc.get());
      this->flight_arcs.push_back(flight_arc);
   }

   // recreate slot nodes
   for (const auto& slot : DataRegistry::instance()->slots) {
      this->get_or_create_node(slot->get_slot_interval().first, slot->get_station());
      this->get_or_create_node(slot->get_slot_interval().second, slot->get_station());
      if (slot->get_slot_type() != SlotType::Departure) {
         this->get_or_create_node(slot->get_slot_interval().second + aircraft->get_min_ground_time(), slot->get_station());
      }
   }

   // sort all the nodes by time
   std::sort(this->nodes.begin(), this->nodes.end(), [](auto& a, auto& b) -> bool
      {
         return a->get_time() < b->get_time();
      });

   // create station:nodes map
   this->station_nodes_map.clear();
   for (auto& node : this->nodes) {
      const auto p_stn = node->get_station();
      if (p_stn == nullptr) continue;  // dummy sink node   
      const auto& stn_id = p_stn->get_id();
      auto it = this->station_nodes_map.find(stn_id);
      if (it == this->station_nodes_map.end()) {
         this->station_nodes_map.emplace(stn_id, std::vector<Node*>());
      }
      this->station_nodes_map[stn_id].push_back(node.get());
   }

   // Each station's subsequence is already sorted by the global node order.

   /* create ground arcs between consecutive nodes */
   for (const auto& itr_map : this->station_nodes_map) {
      const auto& sta_nodes = itr_map.second;
      for (auto it = sta_nodes.begin(); it != sta_nodes.end(); it++) {
         auto it_next_node = std::next(it);
         if (it_next_node == sta_nodes.end()) {
            break;
         }
         this->ground_arcs.push_back(std::make_shared<Arc>(*it, *it_next_node));
      }
   }

   // connect last node in each station to dummy sink node
   for (const auto& it : this->station_nodes_map) {
      const auto last_node = it.second.back();
      if (last_node->get_time() != aircraft->get_end_time()) {
         std::cerr << "Last node time is incorrect!" << std::endl;
         std::cout << "Last node time: " << to_simple_string(last_node->get_time()) << std::endl;
         std::cout << "End time: " << to_simple_string(aircraft->get_end_time()) << std::endl;
         // exit(EXIT_FAILURE);
      }
      this->ground_arcs.push_back(std::make_shared<Arc>(last_node, this->get_sink_node()));
   }

   for (const auto& arc : this->flight_arcs) {
      arc->get_tail_node()->leaving_arcs.push_back(arc.get());
      arc->get_head_node()->entering_arcs.push_back(arc.get());
   }
   for (auto& arc : this->ground_arcs) {
      arc->get_tail_node()->leaving_arcs.push_back(arc.get());
      arc->get_head_node()->entering_arcs.push_back(arc.get());

      double rc = 0.0;
      const auto head = arc->get_head_node();
      const auto tail = arc->get_tail_node();
      const auto source = this->get_source_node();
      if (head == this->get_sink_node()) {
         const auto station = tail->get_station();

         if (Config::instance()->rule_config.position_rule == PositionRule::Hard) {
            if (station != aircraft->get_end_station()) {
               rc += Config::instance()->rule_config.cost_miss_endpos;
            }
         }
         else {
            const auto pos_family_idx = path_model->get_index(station, PositionType::Family, aircraft->get_family());
            if (pos_family_idx >= 0) {
               rc -= path_model->dual_position_req[pos_family_idx];
            }
            const auto pos_model_idx = path_model->get_index(station, PositionType::Model, aircraft->get_family(), aircraft->get_model());
            if (pos_model_idx >= 0) {
               rc -= path_model->dual_position_req[pos_model_idx];
            }
         }
      }
      arc->set_cost(rc);
      arc->set_cost_exclude_slots(rc);
   }
   update_seconds += timer.seconds();
}

void Network::build(const std::vector<LegCopy>& leg_copies, bool create_nodes_for_slots) 
{
   this->clean();
   const auto config = Config::instance();
   auto& aircrafts = DataRegistry::instance()->aircrafts;
   const auto data_reg = DataRegistry::instance();

   for (const auto& ac : aircrafts) {
      const auto aircraft = ac.get();
      auto sub_network = std::make_shared<SubNetwork>(ac.get());
      std::set<Station*> stations;

      // source and sink nodes
      auto source = sub_network->get_or_create_node(ac->get_start_time(), ac->get_start_station());
      auto sink = sub_network->get_or_create_node(ac->get_end_time() + boost::posix_time::minutes(1), nullptr);
      sub_network->set_source_node(source);
      sub_network->set_sink_node(sink);

      // flight arcs and their nodes
      for (auto& leg_copy : leg_copies)
      {
         auto p_copy = &leg_copy;
         if (!ac->is_legal_assignment(p_copy)) continue;
         if (p_copy->get_adj_arr_time(aircraft) > aircraft->get_end_time()) continue;
         auto dep_node = sub_network->get_or_create_node(p_copy->get_adj_dep_time(aircraft), p_copy->get_origin());
         auto arr_node = sub_network->get_or_create_node(p_copy->get_adj_arr_time(aircraft), p_copy->get_destination());
         sub_network->flight_arcs.push_back(std::make_shared<FlightArc>(dep_node, arr_node, p_copy));
      }

      // manually create nodes for airport capacity slots
      if (create_nodes_for_slots) {
         for (const auto& slot : data_reg->slots) {
            sub_network->get_or_create_node(slot->get_slot_interval().first, slot->get_station());
            sub_network->get_or_create_node(slot->get_slot_interval().second, slot->get_station());
         }
      }

      // create last node in each station
      for (const auto &it : sub_network->nodes) {
         const auto stn = it->get_station();
         if(stn == nullptr) 
            continue;
         stations.insert(it->get_station());
      }
      for (const auto station : stations) {
         sub_network->get_or_create_node(aircraft->get_end_time(), station);
      }

      // sort all the nodes by time
      std::sort(sub_network->nodes.begin(), sub_network->nodes.end(), [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) -> bool
         {
            return a->get_time() < b->get_time();
         });

      // create station:nodes map
      sub_network->station_nodes_map.clear();
      for (auto& node : sub_network->nodes) {
         const auto p_stn = node->get_station();
         if (p_stn == nullptr) continue;  // dummy sink node   
         const auto& stn_id = p_stn->get_id();
         auto it = sub_network->station_nodes_map.find(stn_id);
         if (it == sub_network->station_nodes_map.end()) {
            sub_network->station_nodes_map.emplace(stn_id, std::vector<Node*>());
         }
         sub_network->station_nodes_map[stn_id].push_back(node.get());
      }

      /* create ground arcs between consecutive nodes */
      for (const auto& itr_map : sub_network->station_nodes_map) {
         const auto& sta_nodes = itr_map.second;
         for (auto it = sta_nodes.begin(); it != sta_nodes.end(); it++) {
            auto it_next_node = std::next(it);
            if (it_next_node == sta_nodes.end()) {
               break;
            }
            sub_network->ground_arcs.push_back(std::make_shared<Arc>(*it, *it_next_node));
         }
      }

      // connect last node in each station to dummy sink node
      for (const auto& it : sub_network->station_nodes_map) {
         const auto last_node = it.second.back();
         if (last_node->get_time() != aircraft->get_end_time()) {
            std::cout << "Last node time: " << to_simple_string(last_node->get_time()) << std::endl;
            std::cout << "End time: " << to_simple_string(aircraft->get_end_time()) << std::endl;
            std::cerr << "Last node time is incorrect while building timespace network!";
         }
         sub_network->ground_arcs.push_back(std::make_shared<Arc>(last_node, sink));
      }

      // entering and leaving arcs for each node
      for (const auto& arc : sub_network->flight_arcs) {
         arc->get_tail_node()->leaving_arcs.push_back(arc.get());
         arc->get_head_node()->entering_arcs.push_back(arc.get());
      }
      for (const auto& arc : sub_network->ground_arcs) {
         arc->get_tail_node()->leaving_arcs.push_back(arc.get());
         arc->get_head_node()->entering_arcs.push_back(arc.get());
      }

      this->sub_networks.push_back(sub_network);
      this->network_map.emplace(aircraft->get_id(), sub_network.get());
   }
}

void Network::build(const std::map<size_t, std::vector<std::shared_ptr<LegCopy>>>& aircraft_to_all_copies, bool create_nodes_for_slots)
{
   const auto config = Config::instance();
   auto& aircrafts = DataRegistry::instance()->aircrafts;

   for (const auto& ac : aircrafts) {
      const auto aircraft = ac.get();
      auto sub_network = std::make_shared<SubNetwork>(ac.get());
      const auto & leg_copies = aircraft_to_all_copies.at(aircraft->get_id());
      std::set<Station*> stations;

      // source and sink nodes
      auto source = sub_network->get_or_create_node(ac->get_start_time(), ac->get_start_station());
      auto sink = sub_network->get_or_create_node(aircraft->get_end_time() + boost::posix_time::minutes(1), nullptr);
      sub_network->set_source_node(source);
      sub_network->set_sink_node(sink);

      // flight arcs and their nodes
      for (auto& leg_copy : leg_copies)
      {
         auto p_copy = leg_copy.get();
         if (!ac->is_legal_assignment(p_copy))
            continue;
         auto dep_node = sub_network->get_or_create_node(p_copy->get_adj_dep_time(aircraft), p_copy->get_origin());
         auto arr_node = sub_network->get_or_create_node(p_copy->get_adj_arr_time(aircraft), p_copy->get_destination());
         sub_network->flight_arcs.push_back(std::make_shared<FlightArc>(dep_node, arr_node, p_copy));
      }

      // manually create nodes for airport capacity slots
      if (create_nodes_for_slots) {
         const auto turn = aircraft->get_min_ground_time();
         for (const auto& slot : DataRegistry::instance()->slots) {
            sub_network->get_or_create_node(slot->get_slot_interval().first, slot->get_station());
            sub_network->get_or_create_node(slot->get_slot_interval().second, slot->get_station());
            if (slot->get_slot_type() != SlotType::Departure) {
               sub_network->get_or_create_node(slot->get_slot_interval().second + turn, slot->get_station());
            }
         }
      }

      // create last node in each station
      for (const auto& it : sub_network->nodes) {
         const auto stn = it->get_station();
         if (stn == nullptr)
            continue;
         stations.insert(it->get_station());
      }
      for (const auto station : stations) {
         sub_network->get_or_create_node(aircraft->get_end_time(), station);
      }

      // sort all the nodes by time
      std::sort(sub_network->nodes.begin(), sub_network->nodes.end(), [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) -> bool
         {
            return a->get_time() < b->get_time();
         });

      // create station:nodes map
      sub_network->station_nodes_map.clear();
      for (auto& node : sub_network->nodes) {
         const auto p_stn = node->get_station();
         if (p_stn == nullptr) continue;  // dummy sink node   
         const auto& stn_id = p_stn->get_id();
         auto it = sub_network->station_nodes_map.find(stn_id);
         if (it == sub_network->station_nodes_map.end()) {
            sub_network->station_nodes_map.emplace(stn_id, std::vector<Node*>());
         }
         sub_network->station_nodes_map[stn_id].push_back(node.get());
      }

      /* create ground arcs between consecutive nodes */
      for (const auto& itr_map : sub_network->station_nodes_map) {
         const auto& sta_nodes = itr_map.second;
         for (auto it = sta_nodes.begin(); it != sta_nodes.end(); it++) {
            auto it_next_node = std::next(it);
            if (it_next_node == sta_nodes.end()) {
               break;
            }
            sub_network->ground_arcs.push_back(std::make_shared<Arc>(*it, *it_next_node));
         }
      }

      // connect last node in each station to dummy sink node
      for (const auto& it : sub_network->station_nodes_map) {
         const auto last_node = it.second.back();
         if (last_node->get_time() != aircraft->get_end_time()) {
            std::cout << "Last node time: " << to_simple_string(last_node->get_time()) << std::endl;
            std::cout << "End time: " << to_simple_string(aircraft->get_end_time()) << std::endl;
            std::cerr << "Last node time is incorrect while building network on copies!";
            exit(EXIT_FAILURE);
         }
         sub_network->ground_arcs.push_back(std::make_shared<Arc>(last_node, sink));
      }

      // entering and leaving arcs for each node
      for (const auto& arc : sub_network->flight_arcs) {
         arc->get_tail_node()->leaving_arcs.push_back(arc.get());
         arc->get_head_node()->entering_arcs.push_back(arc.get());
      }
      for (const auto& arc : sub_network->ground_arcs) {
         arc->get_tail_node()->leaving_arcs.push_back(arc.get());
         arc->get_head_node()->entering_arcs.push_back(arc.get());
      }

      this->sub_networks.push_back(sub_network);
      this->network_map.emplace(aircraft->get_id(), sub_network.get());
   }
}
