#pragma once
#include "rune_detector.hpp"

#include <opencv2/opencv.hpp>

namespace auto_buff
{
  inline static float image_height = 1280, image_width = 1024;

  enum class Status {
    SUCCESS,
    ARROW_FAILURE,
    ARMOR_FAILURE,
    CENTER_FAILURE
  }; // 成功，箭头检测失败，装甲板检测失败，中心R检测失败

  /**
   * @brief 灯条
   */
  struct LightLine {
    LightLine() = default;
    LightLine(const std::vector<cv::Point>& contour, const cv::Rect2f& global_roi,
              const cv::Rect2f& local_roi = cv::Rect2f(0, 0, image_width, image_height));
    std::vector<cv::Point> m_contour; // 轮廓点集
    double m_contour_area;            // 轮廓面积
    double m_area;                    // 外接旋转矩形面积
    cv::RotatedRect m_rotated_rect;   // 外接旋转矩形
    cv::Point2f m_tl;                 // 左上角点
    cv::Point2f m_tr;                 // 右上角点
    cv::Point2f m_bl;                 // 左下角点
    cv::Point2f m_br;                 // 右下角点
    cv::Point2f m_center;             // 中心点
    double m_length;                  // 长度
    double m_width;                   // 宽度
    double m_x;                       // 中心点 x 坐标
    double m_y;                       // 中心点 y 坐标
    double m_angle;                   // 旋转矩形角度
    double m_aspect_ratio;            // 旋转矩形长宽比
  };

  /**
   * @brief 箭头
   */
  struct Arrow {
    Arrow() = default;
    void set(const std::vector<LightLine>& points, const cv::Point2f& roi);
    std::vector<cv::Point> m_contour; // 轮廓点集
    cv::RotatedRect m_rotated_rect;   // 外接旋转矩形
    double m_length;                  // 长度
    double m_width;                   // 宽度
    cv::Point2f m_center;             // 中心点
    double m_angle;                   // 角度
    double m_aspect_ratio;            // 长宽比
    double m_area;                    // 面积
    double m_fill_ratio;              // 填充比例
  };

  /**
   * @brief 中心 R
   */
  struct CenterR {
    CenterR() = default;
    void set(const LightLine& contour);
    LightLine m_lightline;    // 中心 R 灯条
    cv::Point2f m_center_R;   // 中心 R 点
    cv::Rect m_bounding_rect; // 中心 R 最小正矩形
    double m_x;               // 中心 R x 坐标
    double m_y;               // 中心 R y 坐标
  };

  // 封装一帧图像及其相关的姿态信息和时间戳
  struct Frame {
    Frame() = default;
    Frame(const cv::Mat& image, const std::chrono::steady_clock::time_point& time, double pitch,
          double yaw, double roll)
        : m_image{image}
        , m_time{time}
        , m_roll{roll}
        , m_pitch{pitch}
        , m_yaw{yaw}
    {
    }
    cv::Mat m_image;                              // 图像数据
    std::chrono::steady_clock::time_point m_time; // 图像捕获的时间戳（单调时钟，适合测时间间隔）
    double m_roll, m_pitch, m_yaw;                // 设备/相机的姿态（旋转角度，单位通常是弧度）
    void set(const cv::Mat& image, const std::chrono::steady_clock::time_point& time, double pitch,
             double yaw, double roll);
    void set(const cv::Mat& image, const std::chrono::steady_clock::time_point& time);
  };

} // namespace auto_buff
