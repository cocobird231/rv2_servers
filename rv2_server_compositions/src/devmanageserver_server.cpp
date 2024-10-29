#include <memory>
#include <rv2_server_components/devmanageserver_component.h>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    options.use_intra_process_comms(true);
    auto serverNode = std::make_shared<rv2_interfaces::DevManageServer>(options);
    exec.add_node(serverNode);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
