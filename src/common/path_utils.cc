// src/common/path_utils.cpp
#include "common/path_utils.h"

#ifdef _WIN32
    #include <windows.h>
    #include <libgen.h>  // 需要 mingw 或 cygwin
    #define PATH_MAX 260
#else
    #include <unistd.h>
    #include <libgen.h>
    #include <limits.h>
#endif

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace sad {

std::string GetExecutablePath() {
#ifdef _WIN32
    // Windows 方式
    char buffer[MAX_PATH];
    DWORD len = GetModuleFileNameA(NULL, buffer, sizeof(buffer));
    if (len == 0) {
        return "";
    }
    return std::string(buffer, len);
#else
    // Linux/Mac 方式
    char buffer[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len == -1) {
        return "";
    }
    buffer[len] = '\0';
    return std::string(buffer);
#endif
}

std::string GetExecutableDir() {
    std::string exe_path = GetExecutablePath();
    if (exe_path.empty()) {
        return "";
    }
    
    // dirname 可能会修改字符串，需要复制一份
    char* exe_path_cstr = strdup(exe_path.c_str());
    if (!exe_path_cstr) {
        return "";
    }
    
    std::string exe_dir = dirname(exe_path_cstr);
    free(exe_path_cstr);
    
    return exe_dir;
}

std::string GetProjectRoot() {
    std::string exe_dir = GetExecutableDir();
    if (exe_dir.empty()) {
        return "";
    }
    
    // 假设可执行文件在 build/bin/ 下，需要回到项目根目录
    // 从 build/bin 向上两级：build/bin -> build -> project_root
    char* exe_dir_cstr = strdup(exe_dir.c_str());
    if (!exe_dir_cstr) {
        return "";
    }
    
    // 先去掉 bin
    std::string parent = dirname(exe_dir_cstr);  // build
    free(exe_dir_cstr);
    
    // 再去掉 build
    char* parent_cstr = strdup(parent.c_str());
    if (!parent_cstr) {
        return "";
    }
    std::string project_root = dirname(parent_cstr);  // project_root
    free(parent_cstr);
    
    return project_root;
}

std::string JoinPath(const std::string& base, const std::string& relative) {
    if (base.empty()) {
        return relative;
    }
    if (relative.empty()) {
        return base;
    }
    
    // 检查 base 是否以 / 结尾
    bool base_has_slash = (base.back() == '/');
    bool rel_has_slash = (relative.front() == '/');
    
    if (base_has_slash && rel_has_slash) {
        // 都有的情况：去掉 relative 开头的 /
        return base + relative.substr(1);
    } else if (!base_has_slash && !rel_has_slash) {
        // 都没有的情况：添加 /
        return base + "/" + relative;
    } else {
        // 其中一个有
        return base + relative;
    }
}

} // namespace sad