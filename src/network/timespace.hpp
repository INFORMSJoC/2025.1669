// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include "util.h"
#include "Flight.hpp"
#include <vector>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>

class Station;
class Aircraft;
class PathModel;

namespace Timespace {
   class Node;
   class Arc;
}

namespace Timespace 
{
   struct Label {
      double cost;
      time_duration fly_time;

      boost::posix_time::time_duration allowed_delay;
      double cost_exclude_slots;

      Label* predecessor;
      Arc* arc;      // the arc from its predecessor node to the current node

      Label() :
         cost(0.0), 
         cost_exclude_slots(0.0),
         fly_time(Util::get_duration_from_minutes(0)),
         allowed_delay(Util::get_duration_from_minutes(0)),
         predecessor(nullptr),
         arc(nullptr)
      {};

      Label(const Label* _predecessor, const Arc* _arc, double _cost, double _cost_exclude_slots=0.0) :
         predecessor(const_cast<Label*>(_predecessor)),
         arc(const_cast<Arc*>(_arc)),
         cost(_cost),
         cost_exclude_slots(_cost_exclude_slots),
         allowed_delay(Util::get_duration_from_minutes(0)),
         fly_time(Util::get_duration_from_minutes(0))
      {
      };

      Label(const Label* _predecessor, const Arc* _arc, double _cost, double _cost_exclude_slots, time_duration _ad) :
         predecessor(const_cast<Label*>(_predecessor)),
         arc(const_cast<Arc*>(_arc)),
         cost(_cost),
         cost_exclude_slots(_cost_exclude_slots),
         allowed_delay(_ad),
         fly_time(Util::get_duration_from_minutes(0))
      {
      };

      Label(const Label* _predecessor, const Arc* _arc, double _cost, double _cost_exclude_slots, time_duration _ad, time_duration _fly_time) :
         predecessor(const_cast<Label*>(_predecessor)),
         arc(const_cast<Arc*>(_arc)),
         cost(_cost),
         cost_exclude_slots(_cost_exclude_slots),
         allowed_delay(_ad),
         fly_time(_fly_time)
      {
      };

      static bool check_dominance(const Label* a, const Label* b) {
         return ((a->cost <= b->cost) && (a->fly_time <= b->fly_time) && a->allowed_delay >= b->allowed_delay);
      }

      bool operator==(const Label& other) {
         return (this->arc == other.arc && this->cost == other.cost 
            && this->fly_time == other.fly_time && this->allowed_delay == other.allowed_delay);
      }

      static bool check_dominance(const Label* a, const Label* b, const bool forward, const bool sequential)
      {
         if (forward) {
            return a->cost <= b->cost;
         }
         else if (!sequential) {
            return (a->cost <= b->cost) && (a->allowed_delay >= b->allowed_delay);
         }
         else {      // backward and sequential
            return (a->cost <= b->cost) && (a->allowed_delay >= b->allowed_delay) &&
               (a->cost_exclude_slots <= b->cost_exclude_slots);
         }
      }
   };

   class Node
   {
   private:
      boost::posix_time::ptime time;
      Station* station;

      size_t node_id;

   public:
      double fwd_label;
      Arc* predecessor;

      double bwd_label;
      Arc* sucessor;

      std::vector<std::unique_ptr<Label>> fwd_labels;
      std::vector<std::unique_ptr<Label>> bwd_labels;    // used in CCG algorithm

      std::vector<Arc*> leaving_arcs;
      std::vector<Arc*> entering_arcs;

      Node(const boost::posix_time::ptime _t, Station* _s) :
         station(_s),
         time(_t),
         fwd_label(0),
         bwd_label(0),
         predecessor(nullptr),
         sucessor(nullptr),
         node_id(0)
      {};

      Node() :
         station(nullptr),
         time(boost::posix_time::not_a_date_time),
         node_id(0),
         fwd_label(0),
         bwd_label(0),
         predecessor(nullptr),
         sucessor(nullptr) {};

      ~Node() {}

      bool operator==(const Node& nd) {
         return (this->station == nd.station && this->time == nd.time);
      }

      size_t get_id() const { return node_id; }
      void set_id(size_t i) { node_id = i; }

      boost::posix_time::ptime get_time() const { return time; }
      Station* get_station() const { return station; }

      std::string to_simple_string() const {
         return (station == nullptr ? "Sink" : station->get_code()) + Util::ptime_to_string(time);
      }
   };

   inline auto first_at_or_after(const std::vector<Node*>& nodes, ptime time) {
      return std::lower_bound(nodes.begin(), nodes.end(), time,
         [](const Node* node, ptime t) { return node->get_time() < t; });
   }
   inline auto first_after(const std::vector<Node*>& nodes, ptime time) {
      return std::upper_bound(nodes.begin(), nodes.end(), time,
         [](ptime t, const Node* node) { return t < node->get_time(); });
   }
   inline auto last_before(const std::vector<Node*>& nodes, ptime time) {
      return std::make_reverse_iterator(first_at_or_after(nodes, time));
   }
   inline auto last_at_or_before(const std::vector<Node*>& nodes, ptime time) {
      return std::make_reverse_iterator(first_after(nodes, time));
   }

   class Arc {
   protected:
      Node* tail;
      Node* head;

      double cost;
      double cost_exclude_slots;

   public:
      Arc(Node* _tail, Node* _head) : cost(0.0), cost_exclude_slots(0.0)
      {
         tail = _tail;
         head = _head;
      };
      Arc() : tail(nullptr), head(nullptr), cost(0.0), cost_exclude_slots(0.0) {};
      ~Arc() {};
      Node* get_tail_node() const { return tail; }
      Node* get_head_node() const { return head; }
      void set_tail_node(Node* _tail) { tail = _tail; }
      void set_head_node(Node* _head) { head = _head; }

      boost::posix_time::ptime get_start_time() const { return tail->get_time(); }
      boost::posix_time::ptime get_end_time() const { return head->get_time(); }
      Station* get_origin() const { return tail->get_station(); }
      Station* get_destination() const { return head->get_station(); }

      boost::posix_time::time_duration get_duration() const {
         return head->get_time() - tail->get_time();
      }

      void set_cost(double c) { cost = c; }
      double get_cost() const { return cost; }

      virtual boost::posix_time::time_duration get_delay() const { return boost::posix_time::seconds(0); }
      virtual LegCopy* get_leg_copy() const { return nullptr; }
      virtual Leg* get_leg() const { return nullptr; }
      virtual bool is_maintenance() const { return false; }
      virtual bool is_flight_arc() const { return false; }
      virtual double get_assignment_cost(const Aircraft* aircraft) const { return 0.0; }

      void set_cost_exclude_slots(double c) { cost_exclude_slots = c; }
      double get_cost_exclude_slots() const { return cost_exclude_slots; }

      virtual std::string to_simple_string() const {
         return tail->to_simple_string() + "-" + head->to_simple_string();
      }

      virtual bool is_in_slot(const Slot* slot) const { return false; }

      virtual time_duration get_fly_time() const { return Util::get_duration_from_minutes(0); }
   };

   class FlightArc : public Arc {
   private:
      LegCopy* copy;

   public:
      double assignment_cost = 0;
      int leg_index = -1;
      std::vector<int> slot_indices;

      FlightArc() : copy(nullptr), Arc() {};
      FlightArc(Node* _tail, Node* _head, const LegCopy* _copy) : Arc()
      {
         tail = _tail;
         head = _head;
         copy = const_cast<LegCopy*>(_copy);
      };
      ~FlightArc() {};

      boost::posix_time::time_duration get_delay() const override { return copy->get_delay(); }
      Leg* get_leg() const override { return copy->get_leg(); }
      bool is_maintenance() const override { return copy->get_leg()->is_maintenance(); }
      bool is_flight_arc() const override { return true; }

      LegCopy* get_leg_copy() const override { return copy; };

      double get_assignment_cost(const Aircraft* aircraft) const override { return aircraft->get_assignment_cost(this->get_leg_copy()); }

      std::string to_simple_string() const override {
         std::stringstream ss;
         if (this->is_maintenance()) {
            ss << "MaintenanceArc-" << copy->get_leg()->to_simple_string() << "_" << Util::get_minutes_from_duration(this->get_delay());
         }
         else {
            ss << "FlightArc-" << copy->get_leg()->get_flight_number() << "_" << Util::get_minutes_from_duration(this->get_delay());
         }
         return ss.str();
      }

      bool is_in_slot(const Slot* slot) const override 
      {
         return copy->is_in_slot(slot);
      }

      time_duration get_fly_time() const override 
      {
         return this->get_leg()->get_duration();
      }
   };

   class SubNetwork {
   private:
      Aircraft* aircraft;          // every aircraft has a timespace network
      Node* source;
      Node* sink;
      struct NodeKey {
         ptime time;
         Station* station;
         bool operator==(const NodeKey& other) const { return time == other.time && station == other.station; }
      };
      struct NodeKeyHash {
         size_t operator()(const NodeKey& key) const {
            const auto ticks = (key.time - ptime(boost::gregorian::date(1970, 1, 1))).ticks();
            const auto h = std::hash<long long>{}(ticks);
            return h ^ (std::hash<Station*>{}(key.station) + 0x9e3779b9 + (h << 6) + (h >> 2));
         }
      };
      std::unordered_map<NodeKey, Node*, NodeKeyHash> node_index;
      size_t next_node_id = 0;

   public:
      double label_seconds = 0;
      double update_seconds = 0;
      size_t labels_considered = 0;
      size_t propagation_leaves = 0;
      size_t minlp_solves = 0;
      std::vector<std::shared_ptr<Node> > nodes;
      std::vector<std::shared_ptr<FlightArc> > flight_arcs;
      std::vector<std::shared_ptr<Arc> > ground_arcs;

      std::map<size_t, std::vector<Node*> > station_nodes_map;
      std::map<size_t, std::vector<Arc*>> flight_to_arcs;

      SubNetwork(Aircraft* _a) : aircraft(_a), source(nullptr), sink(nullptr)
      {
         nodes.clear();
         flight_arcs.clear();
         ground_arcs.clear();
      };
      ~SubNetwork() {}

      Node* get_source_node() const { return source; }
      Node* get_sink_node() const { return sink; }
      void set_source_node(Node* n) { source = n; }
      void set_sink_node(Node* n) { sink = n; }
      bool is_source_node(const Node* n) const { return n == source; }
      bool is_sink_node(const Node* n) const { return n == sink; }

      Node* get_or_create_node(ptime _time, Station* _station) {
         const NodeKey key{_time, _station};
         auto itr_node = node_index.find(key);
         if (itr_node == node_index.end()) {
            auto n = std::make_shared<Node>(_time, _station);
            n->set_id(next_node_id++);
            nodes.push_back(n);
            node_index.emplace(key, n.get());
            return n.get();
         }
         return itr_node->second;
      }

      Node* get_node(ptime _time, Station* _station) const {
         const auto it = node_index.find({_time, _station});
         return it == node_index.end() ? nullptr : it->second;
      }

      void delete_node(Node* n) {
         auto itr_node = std::find_if(nodes.begin(), nodes.end(), [n](const std::shared_ptr<Node>& node)->bool {
            return node.get() == n;
            });
         if (itr_node != nodes.end()) {
            node_index.erase({n->get_time(), n->get_station()});
            nodes.erase(itr_node);
         }
      }

      int get_num_nodes() const { return static_cast<int>(nodes.size()); }
      int get_num_arcs() const { return static_cast<int>(flight_arcs.size() + ground_arcs.size()); }
      int get_num_flight_arcs() const { return static_cast<int>(flight_arcs.size()); }
      int get_num_ground_arcs() const { return static_cast<int>(ground_arcs.size()); }

      Aircraft* get_aircraft() const { return aircraft; }

      void reset_labels(bool forward, bool backward) {
         if (forward) {
            for (auto& pNode : nodes) {
               pNode->fwd_labels.clear();
            }
            source->fwd_labels.push_back(std::make_unique<Label>());
         }

         if (backward) {
            for (auto& pNode : nodes) {
               pNode->bwd_labels.clear();
            }
            sink->bwd_labels.push_back(std::make_unique<Label>());
         }
      }

      static bool crosses_maintenance(const Arc* arc, const Maintenance* maintenance);
      static bool crosses_maintenance(const LegCopy* copy, const Maintenance* maintenance);

      void update(const std::vector<std::shared_ptr<LegCopy>>& new_copies, const PathModel* path_model, bool topology_changed = false);
   };

   struct Network
   {
      std::vector<std::shared_ptr<SubNetwork> > sub_networks;
      std::map<size_t, SubNetwork*> network_map;

      SubNetwork* get_sub_network(const Aircraft* aircraft) const { return network_map.at(aircraft->get_id()); }

      void clean() {
         sub_networks.clear();
         network_map.clear();
      }

      void build(const std::vector<LegCopy>& leg_copies, bool create_nodes_for_slots = false);
      void build(const std::map<size_t, std::vector<std::shared_ptr<LegCopy>>>& aircraft_to_all_copies, 
         bool create_nodes_for_slots = false);
   };
}

