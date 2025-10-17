#pragma once

namespace tools
{
  // 定义一个Exiter类，用于处理程序退出相关的逻辑
  class Exiter
  {
  public:
    Exiter();

    // exit()成员函数声明，返回值为bool类型，且该函数不会修改类的成员变量（const修饰）
    bool exit() const;

  private:
    // 全局静态变量，用于标记是否需要退出程序
    static bool exit_;

    // 全局静态变量，用于标记Exiter类是否已经被初始化
    static bool exiter_inited_;
  };

}