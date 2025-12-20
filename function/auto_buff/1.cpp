#include "rune_armour.hpp"

#include <algorithm>

#include "tools/logger.hpp"

namespace auto_buff
{
  /**
   * @brief Construct a new Lightline:: Lightline object
   * @param[in] contour       轮廓点集
   * @param[in] roi           roi 用来设置正确的中心及角点
   */
  LightLine::LightLine(const std::vector<cv::Point>& contour, const cv::Rect2f& global_roi,
                       const cv::Rect2f& local_roi = cv::Rect2f(0, 0, image_width, image_height))
      : m_contour(contour)
      , m_contour_area(cv::contourArea(contour))
      , m_rotated_rect(cv::minAreaRect(contour))
  {

    // 长的为 length，短的为 width
    m_width = m_rotated_rect.size.width, m_length = m_rotated_rect.size.height;
    if (m_width > m_length) {
      std::swap(m_width, m_length);
    }
    m_aspect_ratio = m_length / m_width;
    m_center = m_rotated_rect.center;
    m_angle = m_rotated_rect.angle;
    m_area = m_rotated_rect.size.width * m_rotated_rect.size.height;
    std::array<cv::Point2f, 4> points;
    m_rotated_rect.points(points.begin());
    /**
     * OpenCV 中 RotatedRect::points() 角点顺序为顺时针，p[0]
     * 为纵坐标最大的点。若有多个纵坐标最大，则取其中横坐标最大的点。 p[0] 到 p[3] 的边为
     * width，其邻边为 height。
     * 根据上述关系可以确立四个角点位置。如果是装甲板灯条，则其还需要结合中心 R 来得到中心 R
     * 参照下的角点位置。
     */
    if (m_rotated_rect.size.width > m_rotated_rect.size.height) {
      m_tl = points[1];
      m_tr = points[2];
      m_bl = points[0];
      m_br = points[3];
    } else {
      m_tl = points[0];
      m_tr = points[1];
      m_bl = points[3];
      m_br = points[2];
    }
    // 得到相对原图的角点和中心位置
    m_tl += local_roi.tl() + global_roi.tl();
    m_tr += local_roi.tl() + global_roi.tl();
    m_bl += local_roi.tl() + global_roi.tl();
    m_br += local_roi.tl() + global_roi.tl();
    m_center += local_roi.tl() + global_roi.tl();
    m_x = m_center.x, m_y = m_center.y;
  };

  // 提供一种方式在对象创建后重新设置其内容，避免频繁构造新对象（可能用于性能优化或复用对象）
  void Frame::set(const cv::Mat& image, const std::chrono::steady_clock::time_point& time,
                  double pitch, double yaw, double roll)
  {
    m_image = image;
    m_time = time;
    m_roll = roll;
    m_pitch = pitch;
    m_yaw = yaw;
  }

  void Frame::set(const cv::Mat& image, const std::chrono::steady_clock::time_point& time)
  {
    m_image = image;
    m_time = time;
  }
} // namespace auto_buff