#pragma once

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
  class Classifier
  {
  public:
    explicit Classifier(const std::string& config_path);

    void classify(Armor& armor);

    void ovclassify(Armor& armor);

  private:
    cv::dnn::Net net_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
  };

} // namespace auto_aim
