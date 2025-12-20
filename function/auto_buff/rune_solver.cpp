#include <atomic>
#include <mutex>

#include "rune_solver.hpp"
#include "rune_detector.hpp"

namespace auto_buff
{

  std::atomic<bool> STOP_THREAD(false);
  std::atomic<bool> VALID_PARAMS(false);
  extern std::mutex MUTEX;

  /**
   * @brief 初始化解算参数
   * @param[in]
   */
  BuffSolver::BuffSolver(const std::string& config) { auto yaml = YAML::LoadFile(config); }

  BuffSolver::~BuffSolver()
  {
    STOP_THREAD.store(true);
    m_fit_thread.join();
  };

  // Construct a new Calculator:: Calculator object
  void BuffSolver::init_params()
  {
    m_world_points = {
        {-0.5f * BuffDetection::armor_inside_width, BuffDetection::armor_inside_y, 0.0f},

        {0.5f * BuffDetection::armor_inside_width, BuffDetection::armor_inside_y, 0.0f},

        {0.0f, -BuffDetection::armor_outside_y - BuffDetection::armor_outside_height, 0.0f},

        {-0.5f * BuffDetection::armor_outside_width, -BuffDetection::armor_outside_y, 0.0f},

        {0.5f * BuffDetection::armor_outside_width, -BuffDetection::armor_outside_y, 0.0f},

        {0.0f, BuffDetection::power_rune_radius, 0.0f}};

    m_direction = auto_buff::Direction::UNKNOWN;
    m_convexity = auto_buff::Convexity::UNKNOWN;
    m_totalShift = 0;
    m_first_detect = true;
    m_angle_last = 0.0;

    const int interval_ms = 1000 / FPS;
    m_direction_thresh = std::max(100 / interval_ms, 2);
  }

} // namespace auto_buff