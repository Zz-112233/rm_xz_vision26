#include "exiter.hpp"

#include <csignal>   // 包含信号处理相关的头文件，用于处理SIGINT等系统信号
#include <stdexcept> // 包含标准异常类，用于抛出runtime_error异常

namespace tools
{
  // 全局静态变量，用于标记Exiter类是否已经被初始化
  bool Exiter::exit_ = false;
  bool Exiter::exiter_inited_ = false;

  Exiter::Exiter()
  {
    // 确保整个程序中只存在一个Exiter实例，避免信号处理逻辑冲突
    if (exiter_inited_)
      throw std::runtime_error("Exiter: Only one instance is allowed!");

    // 注册SIGINT信号(通常是用户按下Ctrl+C)的处理函数
    // 当接收到SIGINT信号时，将exit_标记设为true，通知程序准备退出
    std::signal(SIGINT, [](int)
                { exit_ = true; });

    // 标记Exiter实例已初始化
    exiter_inited_ = true;
  }

  // exit()成员函数的实现
  // 返回exit_的当前值，用于判断是否需要退出程序
  bool Exiter::exit() const { return exit_; }

} // namespace tools