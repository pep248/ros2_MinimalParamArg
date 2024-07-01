// Include necessary ROS 2 headers
#include "rclcpp/rclcpp.hpp"
#include <param_example/cpp_complex_param_class.hpp>
#include <thread>

// Function to spin the executor in a separate thread
void executor_spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        // Add the node to the executor
        executor.add_node(node);
        // Spin the executor to process incoming messages
        executor.spin();
    } catch (const std::exception & e) {
        // Log any exceptions that occur
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    } catch (...) {
        // Log any unknown exceptions that occur
        RCLCPP_ERROR(node->get_logger(), "Unknown exception");
    }
    // Remove the node from the executor
    executor.remove_node(node);
    // Shutdown ROS 2
    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a shared pointer to the MyCppComplexParamClass node
    auto cpp_param_node = std::make_shared<MyCppComplexParamClass>("cpp_param_node");

    // Create a new thread to spin the executor
    std::thread ros_thread(executor_spin, cpp_param_node);
    // Detach the thread to allow it to run independently
    ros_thread.detach();

    // Start the node's main loop
    cpp_param_node->start();

    return 0;
}
