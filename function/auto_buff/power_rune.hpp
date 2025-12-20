#pragma once

#include "rune_detector.hpp"

namespace auto_buff
{
  class PowerRune
  {
  public:
    PowerRune();
    bool run_once(const cv::Mat& img, double pitch, double yaw, double roll = 0.0);

  private:
    auto_buff::BuffDetection buff_detection;
  };
} // namespace auto_buff