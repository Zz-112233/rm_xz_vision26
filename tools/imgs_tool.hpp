#pragma once
#include <opencv2/opencv.hpp>

namespace tools
{
  void reset_roi(cv::Rect2f& roi, int w, int h);
  void reset_roi(cv::Rect2f& roi, const cv::Mat& img);
  void reset_roi(cv::Rect2f& roi, const cv::Rect2f& img_roi);
  bool in_rect(const cv::Point2f& p, const cv::Rect2f& r);
} // namespace tools
