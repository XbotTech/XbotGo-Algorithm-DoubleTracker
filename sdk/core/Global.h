#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdarg.h>
#include <stdio.h>

inline bool& printf_enabled()
{
    static bool my_printf_enabled = true; // 是否打印输出，默认为 true
    return my_printf_enabled;
}

// inline 函数定义，避免多重定义
inline void printfXbotGo(const char* format, ...)
{
    if (printf_enabled()) { // 检查是否启用打印
        va_list args;
        va_start(args, format);
        vprintf(format, args); // 打印输出
        va_end(args);
    }
}

// 提供接口用于修改是否启用打印功能
inline void setPrintfEnabled(bool enabled)
{
    if (printf_enabled() != enabled) { // 仅在状态变化时修改
        printf_enabled() = enabled;    // 修改全局变量的值
    }

    if (printf_enabled()) {
        printf("Enable print\n");
    } else {
        printf("Disable print\n");
    }
}

#endif // GLOBAL_H
