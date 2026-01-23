#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace xz_vision
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLOR = {"red", "blue", "extinguish", "purple"};

// 装甲板尺寸类型
enum ArmorType
{
  big,    // 大装甲板：230×127mm (英雄)
  small,  // 标准装甲板：135×125mm (步兵、工程)
  mini    // 迷你装甲板：比标准更小 (前哨站、基地)
};
const std::vector<std::string> ARMOR_TYPE = {"big", "small", "mini"};

enum ArmorName
{
  one = 0,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAME = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// 装甲板属性映射 (颜色, 名称, 类型)
// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  // 标准装甲板 (small): 步兵、工程
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, sentry, small},      {red, sentry, small},      {extinguish, sentry, small},
   
  // 迷你装甲板 (mini): 前哨站、基地
  {blue, outpost, mini},     {red, outpost, mini},     {extinguish, outpost, mini},
  {blue, base, mini},        {red, base, mini},        {extinguish, base, mini},    {purple, base, mini},
  
  // 大装甲板 (big): 英雄
  {blue, one, big},          {red, one, big},          {extinguish, one, big},
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}
};
// clang-format on

struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

struct Armor
{
  Color color;
  Lightbar left, right;
  cv::Point2f center;
  cv::Point2f center_norm;
  std::vector<cv::Point2f> points;

  double ratio;
  double side_ratio;
  double rectangular_error;

    ArmorType type;
    ArmorName name;
    ArmorPriority priority;

    int class_id = -1;       // 分类ID
    cv::Mat pattern;         // 装甲板识别图案
    double confidence = 0.0; // 模型置信度
    bool duplicated;         // 是否重复识别
    cv::Rect box;
    Armor(const Lightbar& left, const Lightbar& right); // 传统视觉构造函数
    double ComputeRectangularError(const Lightbar& left, const Lightbar& right);
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints);
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
    Armor(int color_id, int num_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints);
    Armor(int color_id, int num_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);

    // 神经网络构造函数
    Eigen::Vector3d xyz_in_camera; // 在相机坐标系下的位置
    Eigen::Vector3d xyz_in_gimbal; // 在云台坐标系下的位置
    Eigen::Vector3d xyz_in_world;  // 在世界坐标系下的位置

    // 姿态信息（单位：弧度）
    Eigen::Vector3d ypr_in_gimbal; // 在云台坐标系下的偏航(yaw)、俯仰(pitch)、滚转(roll)
    Eigen::Vector3d ypr_in_world;  // 在世界坐标系下的欧拉角
    Eigen::Vector3d ypd_in_world;  // 在世界坐标系下的偏航(yaw)、俯仰(pitch)、距离(distance)

    // 用于优化的原始数据
    double yaw_raw = 0.0;
  };
} // namespace xz_vision
