#include <param_example/cpp_complex_param_class.hpp>

MyCppComplexParamClass::MyCppComplexParamClass(const std::string & node_name)
: Node(node_name)
{
    // Get parameter file path
    auto param_file = this->declare_parameter<std::string>("complex_config_file", "/random/default/path");

    // Load PoseStamped from YAML file
    this->pose_stamped_ = loadPoseStamped(param_file);
    RCLCPP_INFO(this->get_logger(), "PoseStamped: header.frame_id=%s, pose.position=(%f, %f, %f), pose.orientation=(%f, %f, %f, %f)",
                this->pose_stamped_.header.frame_id.c_str(),
                this->pose_stamped_.pose.position.x,
                this->pose_stamped_.pose.position.y,
                this->pose_stamped_.pose.position.z,
                this->pose_stamped_.pose.orientation.x,
                this->pose_stamped_.pose.orientation.y,
                this->pose_stamped_.pose.orientation.z,
                this->pose_stamped_.pose.orientation.w);
}

geometry_msgs::msg::PoseStamped MyCppComplexParamClass::loadPoseStamped(const std::string & config_file)
{
    YAML::Node config = YAML::LoadFile(config_file);

    geometry_msgs::msg::PoseStamped pose_stamped;

    // Load header
    pose_stamped.header.frame_id = config["pose_stamped"]["header"]["frame_id"].as<std::string>();

    // Load pose
    pose_stamped.pose.position.x = config["pose_stamped"]["pose"]["position"]["x"].as<double>();
    pose_stamped.pose.position.y = config["pose_stamped"]["pose"]["position"]["y"].as<double>();
    pose_stamped.pose.position.z = config["pose_stamped"]["pose"]["position"]["z"].as<double>();

    pose_stamped.pose.orientation.x = config["pose_stamped"]["pose"]["orientation"]["x"].as<double>();
    pose_stamped.pose.orientation.y = config["pose_stamped"]["pose"]["orientation"]["y"].as<double>();
    pose_stamped.pose.orientation.z = config["pose_stamped"]["pose"]["orientation"]["z"].as<double>();
    pose_stamped.pose.orientation.w = config["pose_stamped"]["pose"]["orientation"]["w"].as<double>();

    return pose_stamped;
}

void MyCppComplexParamClass::start()
{
    RCLCPP_INFO(this->get_logger(), "Header: frame_id=%s", this->pose_stamped_.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose:");
    RCLCPP_INFO(this->get_logger(), "  Position: (%f, %f, %f)",
                this->pose_stamped_.pose.position.x,
                this->pose_stamped_.pose.position.y,
                this->pose_stamped_.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  Orientation: (%f, %f, %f, %f)",
                this->pose_stamped_.pose.orientation.x,
                this->pose_stamped_.pose.orientation.y,
                this->pose_stamped_.pose.orientation.z,
                this->pose_stamped_.pose.orientation.w);
}
