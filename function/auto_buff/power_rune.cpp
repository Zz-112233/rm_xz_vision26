#include "power_rune.hpp"

#include <thread>

namespace auto_buff
{
  extern std::mutex MUTEX;
  PowerRune::PowerRune();

  bool PowerRune::run_once(const cv::Mat& image, double pitch, double yaw, double roll)
  {
    Frame frame{image, std::chrono::steady_clock::now(), pitch, yaw, roll};
#if CONSOLE_OUTPUT >= 2
    MUTEX.lock();
    std::cout << "------" << std::endl;
    std::cout << "current time point: "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     frame.m_time.time_since_epoch())
                     .count()
              << std::endl;
    std::cout << "current row: " << roll << std::endl;
    std::cout << "current pitch: " << pitch << std::endl;
    std::cout << "current yaw: " << yaw << std::endl;
    MUTEX.unlock();
#endif
    if (buff_detection.detect(frame) == false) {
      return false;
    }
    auto cameraPoints{buff_detection.getCameraPoints()};
    bool result = m_calculator.calculate(frame, cameraPoints);
#if SHOW_IMAGE >= 1
    if (result == true) {
      m_detector.drawTargetPoint(m_calculator.getPredictPixel());
    }
    m_detector.visualize();
    char key = cv::waitKey(1);
    if (key == ' ') {
      cv::waitKey(0);
    } else if (key == 'q') {
      std::exit(1);
    }
#endif
    return result;
  };

} // namespace auto_buff