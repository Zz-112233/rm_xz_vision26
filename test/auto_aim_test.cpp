#include <opencv2/opencv.hpp>
#include <fmt/core.h>

#include "ecu/camera.hpp"
#include "function/auto_aim/detector.hpp"
#include "function/auto_aim/solver.hpp"
#include "tools/exiter.hpp"

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

  tools::Exiter exiter;
  ecu::Camera camera(config_path);
  xz_vision::Detector detector(config_path);
  xz_vision::Solver solver(config_path);

  while (!exiter.exit()) {
    cv::Mat raw_img;
    std::chrono::steady_clock::time_point timestamp;

    camera.read(raw_img, timestamp);

    auto t_now = std::chrono::steady_clock::now();

    auto armors = detector.detect(raw_img); // 直接调用 detect 方法
    
    // 按 ESC 退出, 按空格暂停
    int key = cv::waitKey(30);
    if (key == 27) // ESC
      break;
    else if (key == 32) { // Space
      cv::waitKey(0);
    }
  }
  return 0;
}