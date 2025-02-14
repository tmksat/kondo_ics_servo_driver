#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

// サービスヘッダ（自作srv）
#include "kondo_ics_servo_driver/srv/set_position.hpp"
#include "kondo_ics_servo_driver/srv/get_position.hpp"
#include "kondo_ics_servo_driver/srv/set_id.hpp"
#include "kondo_ics_servo_driver/srv/get_id.hpp"
#include "kondo_ics_servo_driver/srv/free.hpp"

#include "kondo_ics_servo_driver/ics_driver.hpp"

using namespace std::chrono_literals;

// 角度変換定数
constexpr double RAD_TO_POS_SCALE = 1697.67; // pos = 7500 + rad * 1697.67
constexpr int NEUTRAL_POS = 7500;            // サーボの中立位置

class KondoIcsServoDriverNode : public rclcpp::Node
{
public:
    KondoIcsServoDriverNode();

private:
    // サービスコールバック
    void setPositionCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::SetPosition::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::SetPosition::Response> response);

    void getPositionCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::GetPosition::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::GetPosition::Response> response);

    void setIdCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::SetID::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::SetID::Response> response);

    void getIdCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::GetID::Request> /*request*/,
        std::shared_ptr<kondo_ics_servo_driver::srv::GetID::Response> response);

    void freeCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::Free::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::Free::Response> response);

    std::shared_ptr<IcsDriver> ics_driver_;
    rclcpp::Service<kondo_ics_servo_driver::srv::SetPosition>::SharedPtr set_pos_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::GetPosition>::SharedPtr get_pos_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::SetID>::SharedPtr set_id_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::GetID>::SharedPtr get_id_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::Free>::SharedPtr free_;
};