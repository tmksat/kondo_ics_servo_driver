#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "kondo_ics_servo_driver/kondo_ics_servo_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KondoIcsServoDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
