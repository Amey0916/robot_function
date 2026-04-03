#pragma once
#include <yaml-cpp/yaml.h>
#include "utility/common.h"
namespace timoo
{
namespace lidar
{
template <typename T>
inline void yamlReadAbort(const YAML::Node& yaml, const std::string& key, T& out_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    TM_ERROR << " : Not set " << key;
    TM_ERROR << " value, Aborting!!!" << TM_REND;
    exit(-1);
  }
  else
  {
    out_val = yaml[key].as<T>();
  }
}

template <typename T>
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, T& out_val, const T& default_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    out_val = default_val;
    return false;
  }
  else
  {
    out_val = yaml[key].as<T>();
    return true;
  }
}

inline YAML::Node yamlSubNodeAbort(const YAML::Node& yaml, const std::string& node)
{
  YAML::Node ret = yaml[node.c_str()];
  if (!ret)
  {
    TM_ERROR << " : Cannot find subnode " << node << ". Aborting!!!" << TM_REND;
    exit(-1);
  }
  return std::move(ret);
}

}  // namespace lidar
}  // namespace timoo
