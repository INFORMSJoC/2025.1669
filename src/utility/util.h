// Copyright (c) 2026 Zhouchun Huang.
// SPDX-License-Identifier: MIT
#pragma once

#include <time.h>
#include <algorithm>
#include<stdio.h>
#include<string>
#include<ctime>
#include<vector>
#include<ctime>
#include<map>
#include <chrono>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define PRINT_LOG(log) {std::cout << "    " << log << std::endl;}
#define PRINT_SECTION(log) {std::cout << "[INFO] " << log << "..." << std::endl;}
#define PRINT_SUBSECTION(log) {std::cout << "   " << log << std::endl;}
#define PRINT_SEGMENT(log) {std::cout << std::string(30, '-') << " " << log << " " << std::string(30, '-') << std::endl;}

using namespace boost::posix_time;
using namespace boost::gregorian;

namespace Util
{
   class Stopwatch {
      std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
   public:
      double seconds() const {
         return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
      }
   };

   inline boost::posix_time::time_duration duration_from_seconds(double seconds) {
      return boost::posix_time::microseconds(static_cast<long long>(seconds * 1e6));
   }

   std::tm inline get_hr_min_time_from_string(std::string& time_str)
   {
      std::istringstream time_stream{ time_str };  // Construct a stream from string
      std::tm time;  // Time structure
      time_stream >> std::get_time(&time, "%H:%M");  // Fill time struct

      time.tm_year = time.tm_mon = time.tm_mday = time.tm_yday = time.tm_wday = time.tm_sec = 0;
      time.tm_isdst = -1;

      return time;
   }

   boost::posix_time::ptime inline string_to_ptime(std::string& time_str, const char* time_format = "%d/%m/%y %H:%M")
   {
      std::istringstream time_stream{ time_str };  // Construct a stream from string
      std::tm time;  // Time structure
      time_stream >> std::get_time(&time, time_format);  // Fill time struct

      time.tm_sec = 0;
      time.tm_isdst = -1;

      return ptime_from_tm(time);
   }

   boost::gregorian::date inline string_to_date(std::string& date_str, const char* time_format = "%d/%m/%y")
   {
      std::istringstream time_stream{ date_str };  // Construct a stream from string
      std::tm time;  // Time structure
      time_stream >> std::get_time(&time, time_format);  // Fill time struct

      return boost::gregorian::date_from_tm(time);
   }

   std::string inline ptime_to_string(boost::posix_time::ptime time) {
      std::stringstream stream;
      boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
      facet->format("%m-%dT%H:%M");
      stream.imbue(std::locale(std::locale::classic(), facet));
      stream << time;

      return stream.str();
   }

   std::string inline ptime_to_clock(boost::posix_time::ptime time) {
      std::stringstream stream;
      boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
      facet->format("%H:%M");
      stream.imbue(std::locale(std::locale::classic(), facet));
      stream << time;

      return stream.str();
   }

   std::string inline ptime_to_datestr(boost::posix_time::ptime time) {
      return std::to_string(time.date().month().as_number()) + "/" + std::to_string(time.date().day().as_number());
   }

   boost::posix_time::time_duration inline get_duration_from_minutes(int m) {
      int hours = int(m / 60);
      int minutes = m % 60;

      return boost::posix_time::hours(hours) + boost::posix_time::minutes(minutes);
   }

   int inline get_minutes_from_duration(boost::posix_time::time_duration td) {
      return int(float(td.total_seconds()) / 60.0);
   }

   double inline compute_gap(double upper_bound, double lower_bound) {
      if (upper_bound == 0 && lower_bound == 0) {
         return 0;
      }
      else if (lower_bound == 0) {
         return std::numeric_limits<double>::infinity();
      }
      else {
         return std::abs(upper_bound - lower_bound) / std::abs(lower_bound);
      }
   }
}


