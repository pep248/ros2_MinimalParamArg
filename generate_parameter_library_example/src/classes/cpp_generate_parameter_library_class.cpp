#include <generate_parameter_library_example/cpp_generate_parameter_library_class.hpp>

MyCppGenerateParameterLibraryClass::MyCppGenerateParameterLibraryClass(const std::string & node_name)
: Node(node_name)
{
    config1_params_listener_ = std::make_shared<config_1_params::ParamListener>(shared_from_this());
    config1_params_ = std::make_shared<config_1_params::Params>(config1_params_listener_->get_params());

    config2_params_listener_ = std::make_shared<config_2_params::ParamListener>(shared_from_this());
    config2_params_ = std::make_shared<config_2_params::Params>(config2_params_listener_->get_params());
}

void MyCppGenerateParameterLibraryClass::start()
{
    // RCLCPP_INFO(this->get_logger(), "param1: %s", param1_.c_str());
    // RCLCPP_INFO(this->get_logger(), "param2: %d", param2_);
    // RCLCPP_INFO(this->get_logger(), "param3: %s", param3_ ? "true" : "false");
}
