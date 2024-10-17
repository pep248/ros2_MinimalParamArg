#include <param_example/cpp_param_class.hpp>

MyCppParamClass::MyCppParamClass(const std::string & node_name)
: Node(node_name)
{
    param1_ = this->declare_parameter<std::string>("param1", "default_param1");
    param2_ = this->declare_parameter<int>("param2", 0);
    param3_ = this->declare_parameter<bool>("param3", false);
}

void MyCppParamClass::start()
{
    RCLCPP_INFO(this->get_logger(), "param1: %s", param1_.c_str());
    RCLCPP_INFO(this->get_logger(), "param2: %d", param2_);
    RCLCPP_INFO(this->get_logger(), "param3: %s", param3_ ? "true" : "false");
}
