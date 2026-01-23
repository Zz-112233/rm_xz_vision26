#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace xz_vision
{
  class Tracker
  {
  public:
    Tracker(const std::string& config_path, Solver& solver);

    std::string state() const;

    std::list<Target> track(std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
                            bool use_enemy_color = true);

  private:
    Solver& solver_;
    Color enemy_color_;
    int min_detect_count_;
    int max_temp_lost_count_;
    int detect_count_;
    int temp_lost_count_;
    int outpost_max_temp_lost_count_;
    int normal_temp_lost_count_;
    std::string state_, pre_state_;
    Target target_;
    std::chrono::steady_clock::time_point last_timestamp_;

    void state_machine(bool found);

    bool set_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);

    bool update_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
  };

} // namespace xz_vision

#endif