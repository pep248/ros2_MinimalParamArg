#ifndef CPP_PARAM_CLASS_HPP
#define CPP_PARAM_CLASS_HPP

#include "rclcpp/rclcpp.hpp"

using namespace std;

class MyCppParamClass : public rclcpp::Node
{
public:
    MyCppParamClass(const std::string & node_name);

    void start();

private:
    std::string param1_;
    int param2_;
    bool param3_;
};

#endif // CPP_PARAM_CLASS_HPP
