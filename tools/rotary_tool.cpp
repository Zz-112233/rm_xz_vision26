#include "rotary_tool.hpp"

#include <cmath>
#include <opencv2/core.hpp> // CV_PI

namespace tools
{
  double limit_rad(double angle)
  {
    while (angle > CV_PI)
      angle -= 2 * CV_PI;
    while (angle <= -CV_PI)
      angle += 2 * CV_PI;
    return angle;
  }

  Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
  {
    if (!extrinsic)
      std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper)
      k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
      a = xyzw[3];
      b = xyzw[i];
      c = xyzw[j];
      d = xyzw[k] * sign;
    } else {
      a = xyzw[3] - xyzw[j];
      b = xyzw[i] + xyzw[k] * sign;
      c = xyzw[j] + xyzw[3];
      d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2 = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
    auto safe = safe1 && safe2;
    if (safe) {
      eulers[0] = half_sum + half_diff;
      eulers[2] = half_sum - half_diff;
    } else {
      if (!extrinsic) {
        eulers[0] = 0;
        if (!safe1)
          eulers[2] = 2 * half_sum;
        if (!safe2)
          eulers[2] = -2 * half_diff;
      } else {
        eulers[2] = 0;
        if (!safe1)
          eulers[0] = 2 * half_sum;
        if (!safe2)
          eulers[0] = 2 * half_diff;
      }
    }

    for (int i = 0; i < 3; i++)
      eulers[i] = limit_rad(eulers[i]);

    if (!is_proper) {
      eulers[2] *= sign;
      eulers[1] -= CV_PI / 2;
    }

    if (!extrinsic)
      std::swap(eulers[0], eulers[2]);

    return eulers;
  }

  Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
  {
    Eigen::Quaterniond q(R);
    return eulers(q, axis0, axis1, axis2, extrinsic);
  }

  Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d& ypr)
  {
    double roll = ypr[2];
    double pitch = ypr[1];
    double yaw = ypr[0];
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_roll = cos(roll);
    double sin_roll = sin(roll);
    // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };
    // clang-format on
    return R;
  }

  Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz)
  {
    auto x = xyz[0], y = xyz[1], z = xyz[2];
    auto yaw = std::atan2(y, x);
    auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
    auto distance = std::sqrt(x * x + y * y + z * z);
    return {yaw, pitch, distance};
  }

  Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz)
  {
    auto x = xyz[0], y = xyz[1], z = xyz[2];

    auto dyaw_dx = -y / (x * x + y * y);
    auto dyaw_dy = x / (x * x + y * y);
    auto dyaw_dz = 0.0;

    auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

    auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

    // clang-format off
  Eigen::MatrixXd J{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };
    // clang-format on

    return J;
  }

  Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd)
  {
    auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
    auto x = distance * std::cos(pitch) * std::cos(yaw);
    auto y = distance * std::cos(pitch) * std::sin(yaw);
    auto z = distance * std::sin(pitch);
    return {x, y, z};
  }

  Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d& ypd)
  {
    auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);

    auto dx_dyaw = distance * cos_pitch * -sin_yaw;
    auto dy_dyaw = distance * cos_pitch * cos_yaw;
    auto dz_dyaw = 0.0;

    auto dx_dpitch = distance * -sin_pitch * cos_yaw;
    auto dy_dpitch = distance * -sin_pitch * sin_yaw;
    auto dz_dpitch = distance * cos_pitch;

    auto dx_ddistance = cos_pitch * cos_yaw;
    auto dy_ddistance = cos_pitch * sin_yaw;
    auto dz_ddistance = sin_pitch;

    // clang-format off
  Eigen::MatrixXd J{
    {dx_dyaw, dx_dpitch, dx_ddistance},
    {dy_dyaw, dy_dpitch, dy_ddistance},
    {dz_dyaw, dz_dpitch, dz_ddistance}
  };
    // clang-format on

    return J;
  }

  double delta_time(const std::chrono::steady_clock::time_point& a,
                    const std::chrono::steady_clock::time_point& b)
  {
    std::chrono::duration<double> c = a - b;
    return c.count();
  }

  double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2)
  {
    if (vec1.norm() == 0. || vec2.norm() == 0.) {
      return 0.;
    }
    return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
  }

  double limit_min_max(double input, double min, double max)
  {
    if (input > max)
      return max;
    else if (input < min)
      return min;
    return input;
  }

  // 角度限制在(-π, π]
  double limit_radian(double angle)
  {
    constexpr double TWO_PI = 2.0 * M_PI;
    angle = fmod(angle, TWO_PI);
    if (angle > M_PI)
      angle -= TWO_PI;
    if (angle <= -M_PI)
      angle += TWO_PI;
    return angle;
  }

  // 直角坐标转球坐标 (yaw, pitch, distance)
  Eigen::Vector3d convert_xyz_to_yaw_pitch_distance(const Eigen::Vector3d& xyz)
  {
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    double distance = std::sqrt(x * x + y * y + z * z);

    // 处理距离为0的情况
    if (distance < 1e-6) {
      return Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    double yaw = std::atan2(y, x);
    double pitch = std::asin(z / distance);

    return Eigen::Vector3d(yaw, pitch, distance);
  }

  // 计算xyz到ypd的雅可比矩阵
  Eigen::MatrixXd compute_xyz_to_yaw_pitch_distance_jacobian(const Eigen::Vector3d& xyz)
  {
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    double x2 = x * x;
    double y2 = y * y;
    double z2 = z * z;

    double d2 = x2 + y2 + z2; // 距离平方
    double d = std::sqrt(d2); // 距离

    double r2 = x2 + y2;      // 水平距离平方
    double r = std::sqrt(r2); // 水平距离

    Eigen::MatrixXd jacobian(3, 3);

    // 处理奇异点
    if (d < 1e-6 || r < 1e-6) {
      jacobian.setZero();
      jacobian(2, 2) = 1.0; // ddistance/dz = 1
      return jacobian;
    }

    double d3 = d * d2;

    // dyaw/dx, dyaw/dy, dyaw/dz
    jacobian(0, 0) = -y / r2;
    jacobian(0, 1) = x / r2;
    jacobian(0, 2) = 0.0;

    // dpitch/dx, dpitch/dy, dpitch/dz
    jacobian(1, 0) = -x * z / (d2 * r);
    jacobian(1, 1) = -y * z / (d2 * r);
    jacobian(1, 2) = r / d2;

    // ddistance/dx, ddistance/dy, ddistance/dz
    jacobian(2, 0) = x / d;
    jacobian(2, 1) = y / d;
    jacobian(2, 2) = z / d;

    return jacobian;
  }

  // 计算时间差 (秒)
  double calculate_delta_time(const std::chrono::steady_clock::time_point& current,
                              const std::chrono::steady_clock::time_point& previous)
  {
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(current - previous);
    return duration.count();
  }

} // namespace tools