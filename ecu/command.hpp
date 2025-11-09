#pragma once

namespace ecu
{
  struct Command {
    bool control;
    bool shoot;                  // 是否射击
    double yaw;                  // 俯仰角
    double pitch;                // 偏航角
    double horizon_distance = 0; // 无人机专有
  };

} // namespace ecu