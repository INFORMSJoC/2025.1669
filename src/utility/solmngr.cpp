// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#include "solmngr.h"

SolRegistry* SolRegistry::sol_instance = nullptr;

SolRegistry::SolRegistry()
{
   objective = 0.0;
   solve_time = boost::posix_time::time_duration(0, 0, 0);

   solve_time = boost::posix_time::seconds(0);
   objective = 0;
   gap = 0.0;
   lp_objective = 0.0;

   env = new GRBEnv();

   copyGenTime = colGenTime = solveMpTime = solveSppTime = updateNetworkTime = 0;
}