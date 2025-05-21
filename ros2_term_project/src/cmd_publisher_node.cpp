#include "rclcpp/rclcpp.hpp"
#include "cmd_publisher.h"

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // ✅ use_sim_time 파라미터를 명시적으로 설정하여 충돌 방지
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});

    auto node = std::make_shared<CmdPublisher>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
