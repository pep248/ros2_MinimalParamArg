#include <generate_parameter_library_example/cpp_generate_parameter_library_class.hpp>

MyCppGenerateParameterLibraryClass::MyCppGenerateParameterLibraryClass(const std::string & node_name)
: Node(node_name)
{
    config1_params_listener_ = std::make_shared<config_1_params::ParamListener>(this->get_node_parameters_interface());
    config1_params_ = std::make_shared<config_1_params::Params>(config1_params_listener_->get_params());

    config2_params_listener_ = std::make_shared<config_2_params::ParamListener>(this->get_node_parameters_interface());
    config2_params_ = std::make_shared<config_2_params::Params>(config2_params_listener_->get_params());
}

void MyCppGenerateParameterLibraryClass::start()
{
    // Print all parameters from config_1_params
    RCLCPP_INFO(this->get_logger(), "Config 1 Parameters:");
    RCLCPP_INFO(this->get_logger(), "  param1: %s", config1_params_->param1.c_str());
    RCLCPP_INFO(this->get_logger(), "  param2: %f", config1_params_->param2);

    // Print all parameters from config_2_params
    RCLCPP_INFO(this->get_logger(), "Config 2 Parameters:");
    RCLCPP_INFO(this->get_logger(), "  param_field_1.param1: %s", config2_params_->param_field_1.param1.c_str());
    RCLCPP_INFO(this->get_logger(), "  param_field_1.param2: %f", config2_params_->param_field_1.param2);
    RCLCPP_INFO(this->get_logger(), "  param_field_2.paramA: %ld", config2_params_->param_field_2.paramA);
    RCLCPP_INFO(this->get_logger(), "  param_field_2.paramB: %s", config2_params_->param_field_2.paramB ? "true" : "false");
}