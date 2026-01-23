#include <fmt/core.h>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip>

#include "function/auto_aim/detector.hpp"
#include "tools/exiter.hpp"
#include "tools/draw_tool.hpp"
#include "tools/logger.hpp"
#include "tools/math_tool.hpp"
#include "tools/plotter.hpp"
#include "ecu/camera.hpp"
#include "ecu/command.hpp"

using namespace cv;
using namespace std;

const std::string keys =
    "{help h usage ? |     | 输出命令行参数说明 }"
    "{@config-path c | /home/chaichai/project/rm_xz_vision26/configs/how_to_set_params.yaml | "
    "yaml配置文件的路径}";

int main(int argc, char* argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  ecu::Camera Camera(config_path);
  xz_vision::Detector detector(config_path, true);

  // --- 定义 FPS 统计变量 ---
  int frame_count = 0;            // 帧计数器
  double total_time = 0;          // 累积耗时
  const int stats_interval = 100; // 统计间隔（100帧）

  while (true) {
    cv::Mat raw_img;
    std::chrono::steady_clock::time_point timestamp;

    // 增加空帧检查，防止 cvtColor 报错
    Camera.read(raw_img, timestamp);
    if (raw_img.empty()) {
      continue;
    }

    // 1. 计时开始
    auto start = std::chrono::steady_clock::now();

    // 2. 算法处理
    auto armors = detector.detect(raw_img);

    // 3. 计时结束
    auto end = std::chrono::steady_clock::now();

    // 4. 统计逻辑
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    total_time += duration.count(); // 累加微秒
    frame_count++;

    // 5. 每隔 100 帧计算并输出一次
    if (frame_count >= stats_interval) {
      double avg_ms = (total_time / stats_interval) / 1000.0; // 100帧平均耗时（毫秒）
      double avg_fps = 1000.0 / avg_ms;                       // 平均FPS

      std::cout << "[INFO] Avg Time (last 100 frames): " << std::fixed << std::setprecision(2)
                << avg_ms << " ms | Avg FPS: " << avg_fps << std::endl;

      // 重置计数器
      frame_count = 0;
      total_time = 0;
    }

    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
  }
  return 0;
}