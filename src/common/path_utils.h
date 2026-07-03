// src/common/path_utils.h
#pragma once

#include <string>

namespace sad {

/**
 * @brief 获取当前可执行文件的绝对路径
 * @return 可执行文件的完整路径，失败返回空字符串
 */
std::string GetExecutablePath();

/**
 * @brief 获取当前可执行文件所在目录
 * @return 可执行文件所在目录的绝对路径，失败返回空字符串
 */
std::string GetExecutableDir();

/**
 * @brief 获取项目根目录（假设可执行文件在 build/bin/ 下）
 * @return 项目根目录的绝对路径
 */
std::string GetProjectRoot();

/**
 * @brief 拼接路径（自动处理斜杠）
 * @param base 基础路径
 * @param relative 相对路径
 * @return 拼接后的完整路径
 */
std::string JoinPath(const std::string& base, const std::string& relative);

} // namespace sad