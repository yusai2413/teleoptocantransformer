#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sa_msgs/msg/proto_adapter.hpp>
#include "teleoptocantransformer/msg/vehicle_command.hpp"
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <sstream>
#include <map>
#include <functional>
#include "simple_json_parser.hpp"

// Protobuf 头文件
#include "control_msgs/control_cmd.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"
#include "common_msgs/basic_msgs/header.pb.h"
#include <sa_msgs/msg/chassis_status.hpp>

using namespace std::chrono_literals;

class Teleop2CanTransformerXiaosong : public rclcpp::Node
{
public:
    Teleop2CanTransformerXiaosong() : Node("teleop2can_transformer_xiaosong")
    {
        // 声明参数
        this->declare_parameter<double>("arm_deadzone", 0.05);      // 大臂死区
        this->declare_parameter<double>("stick_deadzone", 0.05);    // 斗杆死区
        this->declare_parameter<double>("bucket_deadzone", 0.05);    // 铲斗死区
        this->declare_parameter<double>("swing_deadzone", 0.05);    // 回转死区
        this->declare_parameter<double>("track_deadzone", 0.05);    // 履带死区
        // 各发布开关（yaml 可配置）
        this->declare_parameter<bool>("publish_vehicle_command", true);         // 是否发布 /vehicle_command
        this->declare_parameter<bool>("publish_vehicle_command_debug", true); // 是否发布 /vehicle_command_debug
        this->declare_parameter<bool>("publish_chassis_feedback", true);      // 是否发布 cannode/chassis_feedback
        this->declare_parameter<bool>("verbose_log", false);                  // 是否打印详细日志，默认关闭以降低CPU
        
        // 电流映射范围（mA）
        // xiaosong 协议：所有控制都是直接电流控制，范围 0~700mA
        this->declare_parameter<double>("max_current", 700.0);      // 最大电流 (mA)
        
        // 获取参数
        arm_deadzone_ = this->get_parameter("arm_deadzone").as_double();
        stick_deadzone_ = this->get_parameter("stick_deadzone").as_double();
        bucket_deadzone_ = this->get_parameter("bucket_deadzone").as_double();
        swing_deadzone_ = this->get_parameter("swing_deadzone").as_double();
        track_deadzone_ = this->get_parameter("track_deadzone").as_double();
        max_current_ = this->get_parameter("max_current").as_double();
        publish_vehicle_command_ = this->get_parameter("publish_vehicle_command").as_bool();
        publish_vehicle_command_debug_ = this->get_parameter("publish_vehicle_command_debug").as_bool();
        publish_chassis_feedback_ = this->get_parameter("publish_chassis_feedback").as_bool();
        verbose_log_ = this->get_parameter("verbose_log").as_bool();
        
        // 创建订阅者（订阅远程端控制指令）
        // 使用 BEST_EFFORT QoS 以匹配发布者
        rclcpp::QoS teleop_qos(10);
        teleop_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        teleop_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        teleop_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        teleop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/controls/teleop", 
            teleop_qos,
            std::bind(&Teleop2CanTransformerXiaosong::teleop_callback, this, std::placeholders::_1)
        );
        
        // 创建发布者（发布到 cannode）
        // 使用 RELIABLE QoS 以匹配 cannode 的订阅者
        rclcpp::QoS vehicle_cmd_qos(10);
        vehicle_cmd_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        vehicle_cmd_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        vehicle_cmd_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        vehicle_cmd_pub_ = this->create_publisher<sa_msgs::msg::ProtoAdapter>(
            "/vehicle_command",
            vehicle_cmd_qos
        );
        
        // 创建非序列化消息发布者（用于调试和查看）
        vehicle_cmd_debug_pub_ = this->create_publisher<teleoptocantransformer::msg::VehicleCommand>(
            "/vehicle_command_debug",
            vehicle_cmd_qos
        );

        // 订阅 cannode 输出的底盘状态并转成 JSON 后发布
        rclcpp::QoS chassis_qos(10);
        chassis_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        chassis_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        chassis_qos.history(rclcpp::HistoryPolicy::KeepLast);

        chassis_status_sub_ = this->create_subscription<sa_msgs::msg::ChassisStatus>(
            "/chassis_status",
            chassis_qos,
            std::bind(&Teleop2CanTransformerXiaosong::chassis_status_callback, this, std::placeholders::_1)
        );

        chassis_feedback_pub_ = this->create_publisher<std_msgs::msg::String>(
            "cannode/chassis_feedback",
            chassis_qos
        );
        
        RCLCPP_INFO(this->get_logger(), "QoS 配置: /controls/teleop (BEST_EFFORT), /vehicle_command (RELIABLE)");
        RCLCPP_INFO(this->get_logger(), "订阅话题: /controls/teleop");
        RCLCPP_INFO(this->get_logger(), "发布话题: /vehicle_command (序列化)");
        RCLCPP_INFO(this->get_logger(), "发布话题: /vehicle_command_debug (非序列化，可用 ros2 topic echo 查看)");
        
        // 使用定时器定期检查连接状态
        auto connection_check_timer = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                size_t pub_count = teleop_sub_->get_publisher_count();
                size_t sub_count = vehicle_cmd_pub_->get_subscription_count();
                RCLCPP_INFO(this->get_logger(), "连接状态检查: /controls/teleop 发布者数=%zu, /vehicle_command 订阅者数=%zu", 
                           pub_count, sub_count);
            }
        );
        
        // 初始化控制命令
        last_arm_up_current_ = 0.0;
        last_arm_down_current_ = 0.0;
        last_stick_retract_current_ = 0.0;
        last_stick_extend_current_ = 0.0;
        last_bucket_close_current_ = 0.0;
        last_bucket_dump_current_ = 0.0;
        last_rotate_left_current_ = 0.0;
        last_rotate_right_current_ = 0.0;
        last_left_track_forward_current_ = 0.0;
        last_left_track_backward_current_ = 0.0;
        last_right_track_forward_current_ = 0.0;
        last_right_track_backward_current_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Teleop2CanTransformerXiaosong 节点已启动（适配小松协议）");
        RCLCPP_INFO(this->get_logger(), "死区设置: arm=%.3f, stick=%.3f, bucket=%.3f, swing=%.3f, track=%.3f",
                    arm_deadzone_, stick_deadzone_, bucket_deadzone_, swing_deadzone_, track_deadzone_);
        RCLCPP_INFO(this->get_logger(), "电流范围: 最大电流=%.1f mA",
                    max_current_);
        RCLCPP_INFO(this->get_logger(), "发布开关: publish_vehicle_command=%s, publish_vehicle_command_debug=%s, publish_chassis_feedback=%s",
                    publish_vehicle_command_ ? "true" : "false",
                    publish_vehicle_command_debug_ ? "true" : "false",
                    publish_chassis_feedback_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "详细日志: verbose_log=%s (默认关闭以降低CPU)", verbose_log_ ? "true" : "false");
    }

private:
    // 死区处理函数
    double apply_deadzone(double value, double deadzone)
    {
        if (std::abs(value) < deadzone) {
            return 0.0;
        }
        // 线性缩放，使死区外的值映射到 [0, 1] 或 [-1, 1]
        if (value > 0) {
            return (value - deadzone) / (1.0 - deadzone);
        } else {
            return (value + deadzone) / (1.0 - deadzone);
        }
    }
    
    // 限制值在范围内
    double clamp(double value, double min_val, double max_val)
    {
        return std::max(min_val, std::min(max_val, value));
    }
    
    // 将输入值转换为电流值
    // -1 到 0：映射到 max_current 到 0（下降/收回方向）
    // 0 到 1：映射到 0 到 max_current（上升/伸出方向）
    double input_to_current(double input, double deadzone)
    {
        // 应用死区（仅用于输入死区，不影响电流映射）
        input = apply_deadzone(input, deadzone);
        
        // 将输入值映射到电流值
        // -1 -> max_current, 0 -> 0, +1 -> max_current
        double current;
        if (input < 0) {
            // 负值范围 [-1, 0] 映射到 [max_current, 0]
            current = (-input) * max_current_;
        } else {
            // 正值范围 [0, 1] 映射到 [0, max_current]
            current = input * max_current_;
        }
        
        return clamp(current, 0.0, max_current_);
    }

    // 将 ChassisStatus 转成 JSON 字符串
    std::string chassis_status_to_json(const sa_msgs::msg::ChassisStatus & msg)
    {
        auto bool_to_str = [](bool v) { return v ? "true" : "false"; };
        std::ostringstream oss;
        oss << "{";
        oss << "\"header\":{";
        oss << "\"stamp\":{\"sec\":" << msg.header.stamp.sec << ",\"nanosec\":" << msg.header.stamp.nanosec << "},";
        oss << "\"frame_id\":\"" << msg.header.frame_id << "\"";
        oss << "},";

        // 基本状态
        oss << "\"high_voltage_status\":" << bool_to_str(msg.high_voltage_status) << ",";
        oss << "\"parking_brake\":" << bool_to_str(msg.parking_brake) << ",";
        oss << "\"horn_status\":" << bool_to_str(msg.horn_status) << ",";
        oss << "\"left_turn_signal\":" << bool_to_str(msg.left_turn_signal) << ",";
        oss << "\"right_turn_signal\":" << bool_to_str(msg.right_turn_signal) << ",";
        oss << "\"walk_motor_mode\":" << static_cast<int>(msg.walk_motor_mode) << ",";
        oss << "\"wet_brake_alarm\":" << bool_to_str(msg.wet_brake_alarm) << ",";
        oss << "\"emergency_stop\":" << bool_to_str(msg.emergency_stop) << ",";
        oss << "\"gear_signal\":" << static_cast<int>(msg.gear_signal) << ",";
        oss << "\"rotation_alarm\":" << static_cast<int>(msg.rotation_alarm) << ",";
        oss << "\"heartbeat_status\":" << bool_to_str(msg.heartbeat_status) << ",";
        oss << "\"brake_control\":" << msg.brake_control << ",";
        oss << "\"front_rear_angle\":" << msg.front_rear_angle << ",";
        oss << "\"battery_level\":" << static_cast<int>(msg.battery_level) << ",";
        oss << "\"charging_status\":" << bool_to_str(msg.charging_status) << ",";
        oss << "\"hydraulic_lock\":" << bool_to_str(msg.hydraulic_lock) << ",";
        oss << "\"fault_level\":" << static_cast<int>(msg.fault_level) << ",";
        oss << "\"turtle_rabbit_gear\":" << bool_to_str(msg.turtle_rabbit_gear) << ",";
        oss << "\"work_light\":" << bool_to_str(msg.work_light) << ",";
        oss << "\"vehicle_mode\":" << static_cast<int>(msg.vehicle_mode) << ",";

        // 电流和压力
        oss << "\"boom_lift_current\":" << msg.boom_lift_current << ",";
        oss << "\"boom_lower_current\":" << msg.boom_lower_current << ",";
        oss << "\"bucket_close_current\":" << msg.bucket_close_current << ",";
        oss << "\"bucket_open_current\":" << msg.bucket_open_current << ",";
        oss << "\"boom_big_pressure\":" << msg.boom_big_pressure << ",";
        oss << "\"boom_small_pressure\":" << msg.boom_small_pressure << ",";
        oss << "\"bucket_big_pressure\":" << msg.bucket_big_pressure << ",";
        oss << "\"bucket_small_pressure\":" << msg.bucket_small_pressure << ",";

        // 电机状态
        oss << "\"hydraulic_motor_speed\":" << msg.hydraulic_motor_speed << ",";
        oss << "\"hydraulic_motor_torque\":" << msg.hydraulic_motor_torque << ",";
        oss << "\"hydraulic_motor_current\":" << msg.hydraulic_motor_current << ",";
        oss << "\"hydraulic_motor_enable\":" << bool_to_str(msg.hydraulic_motor_enable) << ",";
        oss << "\"walk_motor_current\":" << msg.walk_motor_current << ",";
        oss << "\"walk_motor_torque\":" << msg.walk_motor_torque << ",";
        oss << "\"walk_motor_speed\":" << msg.walk_motor_speed << ",";
        oss << "\"vehicle_speed\":" << msg.walk_motor_speed * 3.6 << ",";
        oss << "\"walk_motor_enable\":" << bool_to_str(msg.walk_motor_enable) << ",";

        // 角度信息
        oss << "\"boom_angle\":" << msg.boom_angle << ",";
        oss << "\"bucket_angle\":" << msg.bucket_angle;

        oss << "}";
        return oss.str();
    }

    // 底盘状态订阅回调
    void chassis_status_callback(const sa_msgs::msg::ChassisStatus::SharedPtr msg)
    {
        if (publish_chassis_feedback_) {
            std_msgs::msg::String json_msg;
            json_msg.data = chassis_status_to_json(*msg);
            chassis_feedback_pub_->publish(json_msg);
        }
    }
    
    void teleop_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (verbose_log_) {
            RCLCPP_INFO(this->get_logger(), "🔔 回调函数被调用！");
        }
        try {
            // 打印接收到的原始输入
            if (verbose_log_) {
                RCLCPP_INFO(this->get_logger(), "============================================================");
                RCLCPP_INFO(this->get_logger(), "📥 收到控制指令: %s", msg->data.c_str());
                RCLCPP_INFO(this->get_logger(), "📥 消息长度: %zu 字节", msg->data.length());
            }
            
            // 解析 JSON
            std::map<std::string, std::string> data = SimpleJsonParser::parse(msg->data);
            
            // 创建 ControlCommand protobuf 消息
            control::ControlCommand cmd;
            
            // 设置 header
            auto* header = cmd.mutable_header();
            auto now = this->now();
            header->set_timestamp_sec(now.seconds());
            header->set_frame_id("base_link");
            
            // 处理大臂控制 (boom: -1..1 -> arm_up_current / arm_down_current)
            // -1 到 0：下降方向，映射到 arm_down_current 700~0
            // 0 到 1：抬升方向，映射到 arm_up_current 0~700
            if (data.find("boom") != data.end()) {
                double boom_input = clamp(SimpleJsonParser::get_double(data["boom"]), -1.0, 1.0);
                
                if (boom_input > 0) {
                    // 抬升：0~1 映射到 0~700
                    double current = input_to_current(boom_input, arm_deadzone_);
                    cmd.set_arm_up_current(current);
                    cmd.set_arm_down_current(0.0);
                    last_arm_up_current_ = current;
                    last_arm_down_current_ = 0.0;
                } else if (boom_input < 0) {
                    // 下降：-1~0 映射到 700~0
                    double current = input_to_current(boom_input, arm_deadzone_);
                    cmd.set_arm_up_current(0.0);
                    cmd.set_arm_down_current(current);
                    last_arm_up_current_ = 0.0;
                    last_arm_down_current_ = current;
                } else {
                    // 无输入
                    cmd.set_arm_up_current(0.0);
                    cmd.set_arm_down_current(0.0);
                    last_arm_up_current_ = 0.0;
                    last_arm_down_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_arm_up_current(last_arm_up_current_);
                cmd.set_arm_down_current(last_arm_down_current_);
            }
            
            // 处理斗杆控制 (stick: -1..1 -> stick_retract_current / stick_extend_current)
            // -1 到 0：收回方向，映射到 stick_retract_current 700~0
            // 0 到 1：伸出方向，映射到 stick_extend_current 0~700
            if (data.find("stick") != data.end()) {
                double stick_input = clamp(SimpleJsonParser::get_double(data["stick"]), -1.0, 1.0);
                
                if (stick_input > 0) {
                    // 伸出：0~1 映射到 0~700
                    double current = input_to_current(stick_input, stick_deadzone_);
                    cmd.set_stick_retract_current(0.0);
                    cmd.set_stick_extend_current(current);
                    last_stick_retract_current_ = 0.0;
                    last_stick_extend_current_ = current;
                } else if (stick_input < 0) {
                    // 收回：-1~0 映射到 700~0
                    double current = input_to_current(stick_input, stick_deadzone_);
                    cmd.set_stick_retract_current(current);
                    cmd.set_stick_extend_current(0.0);
                    last_stick_retract_current_ = current;
                    last_stick_extend_current_ = 0.0;
                } else {
                    // 无输入
                    cmd.set_stick_retract_current(0.0);
                    cmd.set_stick_extend_current(0.0);
                    last_stick_retract_current_ = 0.0;
                    last_stick_extend_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_stick_retract_current(last_stick_retract_current_);
                cmd.set_stick_extend_current(last_stick_extend_current_);
            }
            
            // 处理铲斗控制 (bucket: -1..1 -> bucket_close_current / bucket_dump_current)
            // -1 到 0：收斗方向，映射到 bucket_close_current 700~0
            // 0 到 1：翻斗方向，映射到 bucket_dump_current 0~700
            if (data.find("bucket") != data.end()) {
                double bucket_input = clamp(SimpleJsonParser::get_double(data["bucket"]), -1.0, 1.0);
                
                if (bucket_input > 0) {
                    // 翻斗：0~1 映射到 0~700
                    double current = input_to_current(bucket_input, bucket_deadzone_);
                    cmd.set_bucket_close_current(0.0);
                    cmd.set_bucket_dump_current(current);
                    last_bucket_close_current_ = 0.0;
                    last_bucket_dump_current_ = current;
                } else if (bucket_input < 0) {
                    // 收斗：-1~0 映射到 700~0
                    double current = input_to_current(bucket_input, bucket_deadzone_);
                    cmd.set_bucket_close_current(current);
                    cmd.set_bucket_dump_current(0.0);
                    last_bucket_close_current_ = current;
                    last_bucket_dump_current_ = 0.0;
                } else {
                    // 无输入
                    cmd.set_bucket_close_current(0.0);
                    cmd.set_bucket_dump_current(0.0);
                    last_bucket_close_current_ = 0.0;
                    last_bucket_dump_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_bucket_close_current(last_bucket_close_current_);
                cmd.set_bucket_dump_current(last_bucket_dump_current_);
            }
            
            // 处理回转控制 (swing: -1..1 -> rotate_left_current / rotate_right_current)
            // -1 到 0：左转方向，映射到 rotate_left_current 700~0
            // 0 到 1：右转方向，映射到 rotate_right_current 0~700
            if (data.find("swing") != data.end()) {
                double swing_input = clamp(SimpleJsonParser::get_double(data["swing"]), -1.0, 1.0);
                
                if (swing_input > 0) {
                    // 右转：0~1 映射到 0~700
                    double current = input_to_current(swing_input, swing_deadzone_);
                    cmd.set_rotate_left_current(0.0);
                    cmd.set_rotate_right_current(current);
                    last_rotate_left_current_ = 0.0;
                    last_rotate_right_current_ = current;
                } else if (swing_input < 0) {
                    // 左转：-1~0 映射到 700~0
                    double current = input_to_current(swing_input, swing_deadzone_);
                    cmd.set_rotate_left_current(current);
                    cmd.set_rotate_right_current(0.0);
                    last_rotate_left_current_ = current;
                    last_rotate_right_current_ = 0.0;
                } else {
                    // 无输入
                    cmd.set_rotate_left_current(0.0);
                    cmd.set_rotate_right_current(0.0);
                    last_rotate_left_current_ = 0.0;
                    last_rotate_right_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_rotate_left_current(last_rotate_left_current_);
                cmd.set_rotate_right_current(last_rotate_right_current_);
            }
            
            // 处理左履带控制 (leftTrack: -1..1 -> left_track_forward_current / left_track_backward_current)
            // -1 到 0：后退方向，映射到 left_track_backward_current 700~0
            // 0 到 1：前进方向，映射到 left_track_forward_current 0~700
            if (data.find("leftTrack") != data.end()) {
                double left_track_input = clamp(SimpleJsonParser::get_double(data["leftTrack"]), -1.0, 1.0);
                
                if (left_track_input > 0) {
                    // 前进：0~1 映射到 0~700
                    double current = input_to_current(left_track_input, track_deadzone_);
                    cmd.set_left_track_forward_current(current);
                    cmd.set_left_track_backward_current(0.0);
                    last_left_track_forward_current_ = current;
                    last_left_track_backward_current_ = 0.0;
                } else if (left_track_input < 0) {
                    // 后退：-1~0 映射到 700~0
                    double current = input_to_current(left_track_input, track_deadzone_);
                    cmd.set_left_track_forward_current(0.0);
                    cmd.set_left_track_backward_current(current);
                    last_left_track_forward_current_ = 0.0;
                    last_left_track_backward_current_ = current;
                } else {
                    // 无输入
                    cmd.set_left_track_forward_current(0.0);
                    cmd.set_left_track_backward_current(0.0);
                    last_left_track_forward_current_ = 0.0;
                    last_left_track_backward_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_left_track_forward_current(last_left_track_forward_current_);
                cmd.set_left_track_backward_current(last_left_track_backward_current_);
            }
            
            // 处理右履带控制 (rightTrack: -1..1 -> right_track_forward_current / right_track_backward_current)
            // -1 到 0：后退方向，映射到 right_track_backward_current 700~0
            // 0 到 1：前进方向，映射到 right_track_forward_current 0~700
            if (data.find("rightTrack") != data.end()) {
                double right_track_input = clamp(SimpleJsonParser::get_double(data["rightTrack"]), -1.0, 1.0);
                
                if (right_track_input > 0) {
                    // 前进：0~1 映射到 0~700
                    double current = input_to_current(right_track_input, track_deadzone_);
                    cmd.set_right_track_forward_current(current);
                    cmd.set_right_track_backward_current(0.0);
                    last_right_track_forward_current_ = current;
                    last_right_track_backward_current_ = 0.0;
                } else if (right_track_input < 0) {
                    // 后退：-1~0 映射到 700~0
                    double current = input_to_current(right_track_input, track_deadzone_);
                    cmd.set_right_track_forward_current(0.0);
                    cmd.set_right_track_backward_current(current);
                    last_right_track_forward_current_ = 0.0;
                    last_right_track_backward_current_ = current;
                } else {
                    // 无输入
                    cmd.set_right_track_forward_current(0.0);
                    cmd.set_right_track_backward_current(0.0);
                    last_right_track_forward_current_ = 0.0;
                    last_right_track_backward_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_right_track_forward_current(last_right_track_forward_current_);
                cmd.set_right_track_backward_current(last_right_track_backward_current_);
            }
            
            // 处理紧急停止
            if (data.find("emergency_stop") != data.end()) {
                bool estop = SimpleJsonParser::get_bool(data["emergency_stop"]);
                cmd.set_estop(estop);
            }
            
            // 处理驻车制动
            if (data.find("parking_brake") != data.end()) {
                cmd.set_parking_brake(SimpleJsonParser::get_bool(data["parking_brake"]));
            }
            
            // 处理开关量位图 (switch_bits: uint32)
            if (data.find("switch_bits") != data.end()) {
                uint32_t switch_bits = static_cast<uint32_t>(SimpleJsonParser::get_int(data["switch_bits"]));
                cmd.set_switch_bits(switch_bits);
            }
            
            // 打印转换后的输出信息（仅在 verbose 模式下打印）
            if (verbose_log_) {
                RCLCPP_INFO(this->get_logger(), "📤 转换后的控制命令 (xiaosong 协议):");
                RCLCPP_INFO(this->get_logger(), "   大臂: 抬升=%.1f mA, 下降=%.1f mA", 
                           cmd.arm_up_current(), cmd.arm_down_current());
                RCLCPP_INFO(this->get_logger(), "   斗杆: 收回=%.1f mA, 伸出=%.1f mA", 
                           cmd.stick_retract_current(), cmd.stick_extend_current());
                RCLCPP_INFO(this->get_logger(), "   铲斗: 收斗=%.1f mA, 翻斗=%.1f mA", 
                           cmd.bucket_close_current(), cmd.bucket_dump_current());
                RCLCPP_INFO(this->get_logger(), "   回转: 左转=%.1f mA, 右转=%.1f mA", 
                           cmd.rotate_left_current(), cmd.rotate_right_current());
                RCLCPP_INFO(this->get_logger(), "   左履带: 前进=%.1f mA, 后退=%.1f mA", 
                           cmd.left_track_forward_current(), cmd.left_track_backward_current());
                RCLCPP_INFO(this->get_logger(), "   右履带: 前进=%.1f mA, 后退=%.1f mA", 
                           cmd.right_track_forward_current(), cmd.right_track_backward_current());
                if (cmd.has_switch_bits()) {
                    RCLCPP_INFO(this->get_logger(), "   开关量位图: 0x%08X", cmd.switch_bits());
                }
                if (cmd.has_estop()) {
                    RCLCPP_INFO(this->get_logger(), "   紧急停止: %s", cmd.estop() ? "是" : "否");
                }
                if (cmd.has_parking_brake()) {
                    RCLCPP_INFO(this->get_logger(), "   驻车制动: %s", cmd.parking_brake() ? "是" : "否");
                }
            }
            
            // 序列化 protobuf 消息
            std::string serialized_data;
            cmd.SerializeToString(&serialized_data);
            
            // 创建 ROS2 消息（序列化版本）
            if (publish_vehicle_command_) {
                auto ros_msg = sa_msgs::msg::ProtoAdapter();
                ros_msg.pb.assign(serialized_data.begin(), serialized_data.end());
                vehicle_cmd_pub_->publish(ros_msg);
            }
            
            // 创建并发布非序列化消息（用于调试和查看）
            if (publish_vehicle_command_debug_) {
                auto debug_msg = teleoptocantransformer::msg::VehicleCommand();
                debug_msg.header.stamp = this->now();
                debug_msg.header.frame_id = "base_link";
                // 注意：VehicleCommand 消息可能没有 xiaosong 专用的字段，这里只设置通用字段
                debug_msg.estop = cmd.has_estop() ? cmd.estop() : false;
                debug_msg.parking_brake = cmd.has_parking_brake() ? cmd.parking_brake() : false;
                vehicle_cmd_debug_pub_->publish(debug_msg);
            }
            
            if (verbose_log_) {
                RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command (protobuf 大小: %zu 字节)", serialized_data.size());
                RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command_debug (非序列化消息)");
                RCLCPP_INFO(this->get_logger(), "============================================================");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理控制命令时出错: %s", e.what());
        }
    }
    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_sub_;
    rclcpp::Publisher<sa_msgs::msg::ProtoAdapter>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<teleoptocantransformer::msg::VehicleCommand>::SharedPtr vehicle_cmd_debug_pub_;
    rclcpp::Subscription<sa_msgs::msg::ChassisStatus>::SharedPtr chassis_status_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chassis_feedback_pub_;
    
    // 死区参数
    double arm_deadzone_;
    double stick_deadzone_;
    double bucket_deadzone_;
    double swing_deadzone_;
    double track_deadzone_;
    
    // 电流参数
    double max_current_;           // 最大电流 (mA)
    
    // 是否发布/vehicle_command
    bool publish_vehicle_command_;
    bool publish_vehicle_command_debug_;
    bool publish_chassis_feedback_;
    // 是否打印详细日志（降低CPU时可关闭）
    bool verbose_log_;
    
    // 上次的值（用于保持状态）
    double last_arm_up_current_;
    double last_arm_down_current_;
    double last_stick_retract_current_;
    double last_stick_extend_current_;
    double last_bucket_close_current_;
    double last_bucket_dump_current_;
    double last_rotate_left_current_;
    double last_rotate_right_current_;
    double last_left_track_forward_current_;
    double last_left_track_backward_current_;
    double last_right_track_forward_current_;
    double last_right_track_backward_current_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop2CanTransformerXiaosong>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

