/**
 * @file order_payload.hpp
 * @brief 订单载荷（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_tasks {

/**
 * @brief 订单载荷（OrderPayload）
 */
class OrderPayload {
 public:
  /**
   * @brief 判断是否包含 Has。
   * @param key 字符串
   * @return 满足条件返回 true，否则返回 false。
   */
  bool Has(const std::string& key) const;
  /**
   * @brief 获取 GetString。
   * @param key 字符串
   * @param fallback 字符串
   * @return 描述性字符串。
   */
  std::string GetString(const std::string& key, const std::string& fallback = "") const;
  /**
   * @brief 获取 GetInt。
   * @param key 字符串
   * @param fallback 参数
   * @return 计算结果。
   */
  int GetInt(const std::string& key, int fallback = 0) const;
  double GetDouble(const std::string& key, double fallback = 0.0) const;
  /**
   * @brief 获取 GetBool。
   * @param key 字符串
   * @param fallback 布尔标志
   * @return 满足条件返回 true，否则返回 false。
   */
  bool GetBool(const std::string& key, bool fallback = false) const;
  /**
   * @brief 获取 GetStringList。
   * @param key 字符串
   * @param fallback 字符串
   * @return 描述性字符串。
   */
  std::vector<std::string> GetStringList(
      const std::string& key, const std::vector<std::string>& fallback = {}) const;

  /**
   * @brief 设置 SetString。
   * @param key 字符串
   * @param value 字符串
   */
  void SetString(const std::string& key, const std::string& value);
  /**
   * @brief 设置 SetList。
   * @param key 字符串
   * @param values 字符串
   */
  void SetList(const std::string& key, const std::vector<std::string>& values);

 private:
  std::unordered_map<std::string, std::string> values_;
  std::unordered_map<std::string, std::vector<std::string>> lists_;
};

/**
 * @brief 解析订单载荷。
 * @param text 字符串
 * @param message 字符串
 * @return 计算结果。
 */
std::optional<OrderPayload> ParseOrderPayload(const std::string& text, std::string* message);

}  // namespace robot_tasks
