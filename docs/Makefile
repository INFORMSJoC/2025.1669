# Copyright (c) 2026 Zhouchun Huang.
# SPDX-License-Identifier: MIT
# GNU Make, Linux; C++20 and a compatible Gurobi C++ library are required.
CXX ?= g++
GUROBI_HOME ?= /opt/gurobi1301/linux64
GUROBI_LIB ?= gurobi130
CPPFLAGS += -Isrc -Isrc/utility -Isrc/air -Isrc/network -Isrc/models -Isrc/algorithms -I$(GUROBI_HOME)/include
ifneq ($(BOOST_ROOT),)
CPPFLAGS += -I$(BOOST_ROOT)
endif
ifneq ($(JSON_ROOT),)
CPPFLAGS += -I$(JSON_ROOT)
endif
CXXFLAGS ?= -O2 -std=c++20 -fopenmp
LDFLAGS += -L$(GUROBI_HOME)/lib -Wl,-rpath,$(GUROBI_HOME)/lib -fopenmp
LDLIBS += -lgurobi_c++ -l$(GUROBI_LIB) -lm -lpthread
SOURCES := src/engine.cpp src/main.cpp src/air/Air.cpp src/algorithms/CCG.cpp src/algorithms/ColGen.cpp src/algorithms/CopyEvalCrs.cpp src/algorithms/CopyEvalStd.cpp src/algorithms/CopyGen.cpp src/models/PathModel.cpp src/network/timespace.cpp src/utility/config.cpp src/utility/datamngr.cpp src/utility/solmngr.cpp
OBJECTS := $(SOURCES:src/%.cpp=build/%.o)
.PHONY: all clean test
all: bin/arp
bin/arp: $(OBJECTS)
	mkdir -p $(@D)
	$(CXX) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@
build/%.o: src/%.cpp
	mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@
-include $(OBJECTS:.o=.d)
build/tests/performance_tests.o: tests/performance_tests.cpp
	mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@
bin/performance_tests: build/tests/performance_tests.o $(filter-out build/main.o,$(OBJECTS))
	mkdir -p $(@D)
	$(CXX) $^ $(LDFLAGS) $(LDLIBS) -o $@
test: bin/performance_tests
	./bin/performance_tests
-include build/tests/performance_tests.d
clean:
	rm -rf build bin
