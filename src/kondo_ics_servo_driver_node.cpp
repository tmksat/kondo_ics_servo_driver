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

#include "kondo_ics_servo_driver/kondo_ics_servo_driver.hpp"

using namespace std::chrono_literals;

// 角度変換定数
constexpr double RAD_TO_POS_SCALE = 1697.67; // pos = 7500 + rad * 1697.67
constexpr int NEUTRAL_POS = 7500;            // サーボの中立位置

class KondoIcsServoDriverNode : public rclcpp::Node
{
public:
    KondoIcsServoDriverNode()
        : Node("kondo_ics_servo_driver_node")
    {
        // シリアルポート初期化（ポート名/baud rateは適宜変更）
        ics_driver_ = std::make_shared<IcsDriver>("/dev/ttyUSB0", 115200);
        if (!ics_driver_->isOpen())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Serial port opened");
        }

        // 各サービスサーバの生成
        set_pos_service_ = create_service<kondo_ics_servo_driver::srv::SetPosition>(
            "set_position",
            std::bind(&KondoIcsServoDriverNode::setPositionCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        get_pos_service_ = create_service<kondo_ics_servo_driver::srv::GetPosition>(
            "get_position",
            std::bind(&KondoIcsServoDriverNode::getPositionCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        set_id_service_ = create_service<kondo_ics_servo_driver::srv::SetID>(
            "set_id",
            std::bind(&KondoIcsServoDriverNode::setIdCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        get_id_service_ = create_service<kondo_ics_servo_driver::srv::GetID>(
            "get_id",
            std::bind(&KondoIcsServoDriverNode::getIdCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Kondo ICS Servo Driver Node has been started!");
    }

private:
    // サービスコールバック
    void setPositionCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::SetPosition::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::SetPosition::Response> response)
    {
        // 入力角度[radian]をサーボ位置に変換
        int pos = NEUTRAL_POS + static_cast<int>(RAD_TO_POS_SCALE * request->angle);
        RCLCPP_INFO(get_logger(), "SetPosition: servo_id=%d, angle=%f [rad] -> pos=%d", request->servo_id, request->angle, pos);

        bool success = ics_driver_->setPositionCmd(static_cast<uint8_t>(request->servo_id), pos);
        response->success = success;
    }

    void getPositionCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::GetPosition::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::GetPosition::Response> response)
    {
        int pos = ics_driver_->getPositionCmd(static_cast<uint8_t>(request->servo_id));
        if (pos < 0)
        {
            RCLCPP_ERROR(get_logger(), "GetPosition: failed for servo_id=%d", request->servo_id);
            response->angle = -1.0; // エラーの場合は負の値
        }
        else
        {
            double angle = static_cast<double>(pos - NEUTRAL_POS) / RAD_TO_POS_SCALE;
            response->angle = angle;
            RCLCPP_INFO(get_logger(), "GetPosition: servo_id=%d, pos=%d -> angle=%f [rad]", request->servo_id, pos, angle);
        }
    }

    void setIdCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::SetID::Request> request,
        std::shared_ptr<kondo_ics_servo_driver::srv::SetID::Response> response)
    {
        RCLCPP_INFO(get_logger(), "SetID: new_id=%d", request->new_id);
        bool success = ics_driver_->setIdCmd(static_cast<uint8_t>(request->new_id));
        response->success = success;
    }

    void getIdCallback(
        const std::shared_ptr<kondo_ics_servo_driver::srv::GetID::Request> /*request*/,
        std::shared_ptr<kondo_ics_servo_driver::srv::GetID::Response> response)
    {
        int id = ics_driver_->getIdCmd();
        response->servo_id = id;
        RCLCPP_INFO(get_logger(), "GetID: servo_id=%d", id);
    }

    std::shared_ptr<IcsDriver> ics_driver_;
    rclcpp::Service<kondo_ics_servo_driver::srv::SetPosition>::SharedPtr set_pos_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::GetPosition>::SharedPtr get_pos_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::SetID>::SharedPtr set_id_service_;
    rclcpp::Service<kondo_ics_servo_driver::srv::GetID>::SharedPtr get_id_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KondoIcsServoDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
