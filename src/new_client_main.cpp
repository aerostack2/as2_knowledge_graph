#include "as2_knowledge_graph_client.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto service_client = std::make_shared<ServiceClient>();
    while (!service_client->is_service_done()) {
        rclcpp::spin_some(service_client);
    }
    rclcpp::shutdown();
    return 0;

}