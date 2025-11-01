#include "tools/exiter.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    try {
        // 创建 Exiter 实例（会自动注册信号处理）
        tools::Exiter exiter;

        // 程序主逻辑（示例：循环检测是否需要退出）
        std::cout << "程序运行中，按 Ctrl+C 退出..." << std::endl;
        while (!exiter.exit()) { // 调用 exit() 检查退出信号
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "运行中..." << std::endl;
        }

        // 退出前的清理逻辑
        std::cout << "收到退出信号，程序即将退出..." << std::endl;
    } catch (const std::exception& e) {
        // 捕获创建多实例时的异常
        std::cerr << "错误：" << e.what() << std::endl;
        return 1;
    }

    return 0;
}