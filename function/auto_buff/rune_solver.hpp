#pragma once

#include <ceres/ceres.h>

#include "rune_detector.hpp"

#include <yaml-cpp/yaml.h>
#include <tools/logger.hpp>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <random>
#include <shared_mutex>
#include <thread>

namespace auto_buff
{
  class BuffSolver
  {
  public:
    BuffSolver(const std::string& config);
    ~BuffSolver();

    bool calculate(const Frame& frame, std::vector<cv::Point2f>& cameraPoints);

    void init_params();

  private:
    std::vector<cv::Point2f> m_camera_points;
    std::vector<cv::Point3f> m_world_points;
    auto_buff::Direction m_direction; // 旋转方向
    auto_buff::Convexity m_convexity; // 拟合数据凹凸性
    int m_total_shift;                // 总体的装甲板切换数
    double m_bullet_speed;            // 子弹速度
    bool m_first_detect;              // 第一次检测的标志位，第一次检测有效之后置为 true
    double m_angle_last;              // 上一帧相对于第一帧的旋转角度（不考虑装甲板切换)

    int m_direction_thresh;
    std::thread m_fit_thread;
    std::shared_mutex m_mutex;

    inline static int FPS;
  }
} // namespace auto_buff