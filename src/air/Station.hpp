// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include <string>
#include <vector>

class Leg;

class Station {
private:
	size_t id;
	std::string code;                          			// station IATA code
	
public:
	Station(std::string c): 
		code(c),
		id(std::hash<std::string>{}(c))
	{}

   bool operator==(const Station& s) const {
     return (this->code == s.code);
   }

	std::string& get_code() { return code; }
	size_t get_id() const { return id; }
};

