#pragma once

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense> // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace auto_buff
{
  enum PowerRune_type { SMALL, BIG };
  enum FanBlade_type { _target, _unlight, _light };
  enum Track_status { TRACK, TEM_LOSE, LOSE };

  struct fan_blade // 扇叶
  {
    cv::Point2f center;              // 扇页中心
    std::vector<cv::Point2f> points; // 四个点从左上角开始逆时针
    double angle, width, height;
    FanBlade_type type; // 类型
  };

  struct lamp_arm // 灯臂
  {
  };

  class BuffArmour // 能量机关装甲模块
  {
  public:
    BuffArmour();

  private:
    fan_blade rarmor;
    lamp_arm lamparm;
  };

} // namespace auto_buff
