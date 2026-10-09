// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include<vector>
#include<map>
#include "CCG.h"


namespace ARP {
   class Engine {
   private:
      double cpu_time;
      double upper_bound, lower_bound;
      double objective;

      CCG::Solver ccg;

   public:
      Engine();

      void load_input_data();
      void solve();
      void write_results() const;
      void write_kpi() const;
   };
}