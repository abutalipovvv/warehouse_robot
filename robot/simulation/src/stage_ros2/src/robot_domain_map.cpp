#include "stage_ros2/robot_domain_map.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <regex>
#include <set>
#include <stdexcept>
#include <system_error>

namespace
{

std::string trim(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(), [](unsigned char character) {return std::isspace(character);});
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char character) {
        return std::isspace(character);
      }).base();
  return first < last ? std::string(first, last) : std::string();
}

}  // namespace

std::map<std::string, size_t> stage_ros2::parse_robot_domain_map(const std::string & config)
{
  std::map<std::string, size_t> robot_domains;
  std::set<size_t> domains;
  size_t start = 0;
  while (start <= config.size()) {
    const size_t end = config.find(',', start);
    const std::string entry = trim(config.substr(start, end - start));
    if (!entry.empty()) {
      const size_t separator = entry.find('=');
      if (separator == std::string::npos || entry.find('=', separator + 1) != std::string::npos) {
        throw std::invalid_argument(
          "Invalid robot_domain_map entry '" + entry + "'; expected robot_id=domain_id");
      }

      const std::string robot_id = trim(entry.substr(0, separator));
      const std::string domain_text = trim(entry.substr(separator + 1));
      if (!std::regex_match(robot_id, std::regex("[A-Za-z][A-Za-z0-9_-]{1,63}"))) {
        throw std::invalid_argument(
                "Invalid robot_id in robot_domain_map: '" + robot_id + "'");
      }
      size_t domain_id = 0;
      const auto result =
        std::from_chars(domain_text.data(), domain_text.data() + domain_text.size(), domain_id);
      if (
        robot_id.empty() || domain_text.empty() || result.ec != std::errc() ||
        result.ptr != domain_text.data() + domain_text.size() || domain_id > 232)
      {
        throw std::invalid_argument(
          "Invalid robot_domain_map entry '" + entry +
          "'; domain_id must be an integer between 0 and 232");
      }
      if (!robot_domains.emplace(robot_id, domain_id).second) {
        throw std::invalid_argument("Duplicate robot_id in robot_domain_map: " + robot_id);
      }
      if (!domains.emplace(domain_id).second) {
        throw std::invalid_argument(
          "Duplicate ROS domain in robot_domain_map: " + std::to_string(domain_id));
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return robot_domains;
}
