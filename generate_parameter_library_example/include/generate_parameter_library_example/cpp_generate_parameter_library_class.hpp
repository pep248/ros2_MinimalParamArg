#ifndef CPP_GENERATE_PARAMENTER_LIBRARY_CLASS_HPP
#define CPP_GENERATE_PARAMENTER_LIBRARY_CLASS_HPP

#include "rclcpp/rclcpp.hpp"
#include <generate_parameter_library_example/cpp_config_1_parameters.hpp>
#include <generate_parameter_library_example/cpp_config_2_parameters.hpp>

using namespace std;

class MyCppGenerateParameterLibraryClass : public rclcpp::Node
{
public:
    MyCppGenerateParameterLibraryClass(const std::string & node_name);

    void start();

private:
    std::shared_ptr<config_1_params::ParamListener> config1_params_listener_;
    std::shared_ptr<config_1_params::Params> config1_params_;

    std::shared_ptr<config_2_params::ParamListener> config2_params_listener_;
    std::shared_ptr<config_2_params::Params> config2_params_;
};

#endif // CPP_GENERATE_PARAMENTER_LIBRARY_CLASS_HPP
