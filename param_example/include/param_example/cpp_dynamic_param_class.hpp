#ifndef CPP_PARAM_CLASS_HPP
#define CPP_PARAM_CLASS_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>

using namespace std;

class MyCppDynamicParamClass : public rclcpp::Node
{
public:
    MyCppDynamicParamClass(const std::string & node_name);

    void start();

    void print_param();

private:
    std::string param1_;
    int param2_;
    bool param3_;

    rcl_interfaces::msg::SetParametersResult ParamUpdateCallback(const std::vector<rclcpp::Parameter> & parameters);
    OnSetParametersCallbackHandle::SharedPtr param_cb_;


};

#endif // CPP_PARAM_CLASS_HPP
