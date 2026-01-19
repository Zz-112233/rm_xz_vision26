#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "ecu/camera.hpp"
#include "function/auto_aim/detector.hpp"
#include "function/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/rotary_tool.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | /home/zzy/rm_xz_vision26/configs/how_to_set_params.yaml   | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  ecu::Camera camera(config_path);
  xz_vision::Detector detector(config_path, true);
  xz_vision::YOLO yolo(config_path, true);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<xz_vision::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}