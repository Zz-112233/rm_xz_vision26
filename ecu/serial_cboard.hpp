#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "ecu/command.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"
#include "io/public_param.hpp"

enum Mode { idle, auto_aim, small_buff, big_buff, outpost };
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

namespace ecu
{
  class SerialBoard
  {
  public:
    double bullet_speed;
    Mode mode;
    // ShootMode shoot_mode; 烧饼
    double ft_angle;

    explicit SerialBoard(const std::string& config_path);
    ~SerialBoard();

    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
    void send(io::Command command) const;

  private:
    struct IMUData {
      Eigen::Quaterniond q;
      std::chrono::steady_clock::time_point timestamp;
    };

    tools::ThreadSafeQueue<IMUData> queue_;

    int fd_; // 串口文件描述符
    std::thread read_thread_;
    std::atomic<bool> running_;

    std::string port_;
    int baudrate_;

    IMUData data_ahead_;
    IMUData data_behind_;

    void readLoop();
    void parseFrame(const std::vector<uint8_t>& frame);
    std::string read_yaml(const std::string& config_path);
  };

} // namespace ecu
