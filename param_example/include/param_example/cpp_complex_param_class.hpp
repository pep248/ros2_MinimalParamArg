#ifndef CPP_COMPLEX_PARAM_CLASS_HPP
#define CPP_COMPLEX_PARAM_CLASS_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class MyCppComplexParamClass : public rclcpp::Node
{
public:
    MyCppComplexParamClass(const std::string & node_name);

    void start();

private:
    geometry_msgs::msg::PoseStamped pose_stamped_;

    geometry_msgs::msg::PoseStamped loadPoseStamped(const std::string & config_file);
};

#endif // CPP_COMPLEX_PARAM_CLASS_HPP
