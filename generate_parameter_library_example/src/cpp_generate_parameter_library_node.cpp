#include "rclcpp/rclcpp.hpp"
#include <generate_parameter_library_example/cpp_generate_parameter_library_class.hpp>
#include <thread>

void executor_spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Unknown exception");
    }
    executor.remove_node(node);
    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto cpp_generate_parameter_library_node = std::make_shared<MyCppGenerateParameterLibraryClass>("cpp_generate_parameter_library_node");

    std::thread ros_thread(executor_spin, cpp_generate_parameter_library_node);
    ros_thread.detach();

    cpp_generate_parameter_library_node->start();

    return 0;
}
