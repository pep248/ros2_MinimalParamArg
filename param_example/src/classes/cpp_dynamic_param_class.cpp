#include <param_example/cpp_dynamic_param_class.hpp>

MyCppDynamicParamClass::MyCppDynamicParamClass(const std::string & node_name)
: Node(node_name)
{
    param1_ = this->declare_parameter<std::string>("param1", "default_param1");
    param2_ = this->declare_parameter<int>("param2", 0);
    param3_ = this->declare_parameter<bool>("param3", false);

    param_cb_ = this->add_on_set_parameters_callback(std::bind(&MyCppDynamicParamClass::ParamUpdateCallback, this, std::placeholders::_1));
    
}


void MyCppDynamicParamClass::start()
{
    this->print_param();
    while(1);
}


void MyCppDynamicParamClass::print_param()
{
    RCLCPP_INFO(this->get_logger(), "param1: %s", param1_.c_str());
    RCLCPP_INFO(this->get_logger(), "param2: %d", param2_);
    RCLCPP_INFO(this->get_logger(), "param3: %s", param3_ ? "true" : "false");
}


// ros2 param set /cpp_param_node param1 "\"cpp_param_node: This is a modified param\""
// ros2 param set /cpp_param_node param2 34
// ros2 param set /cpp_param_node param3 false
rcl_interfaces::msg::SetParametersResult MyCppDynamicParamClass::ParamUpdateCallback(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const rclcpp::Parameter & param : parameters)
    {
        if (param.get_name() == "param1")
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            {
                result.successful = false;
                result.reason = "my_param_name must be a string";
                break;
            }
            else
            {
                this->param1_ = param.as_string();
            }
        }
       
        else if (param.get_name() == "param2")
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                result.successful = false;
                result.reason = "my_param_name must be an integer";
                break;
            }
            else
            {
                this->param2_ = param.as_int();
            }
        }
        
        else if (param.get_name() == "param3")
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                result.successful = false;
                result.reason = "my_param_name must be a boolean";
                break;
            }
            else
            {
                this->param3_ = param.as_bool();
            }
        }
        
        else
        {
            result.successful = false;
            result.reason = "my_param_name is not a parameter of this node";
            break;
        }
    }

    this->print_param();

    return result;
}