#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sa_msgs/msg/proto_adapter.hpp>
#include "teleoptocantransformer/msg/vehicle_command.hpp"
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <map>
#include "simple_json_parser.hpp"

// Protobuf 头文件
#include "control_msgs/control_cmd.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"
#include "common_msgs/basic_msgs/header.pb.h"

using namespace std::chrono_literals;

class Teleop2CanTransformer : public rclcpp::Node
{
public:
    Teleop2CanTransformer() : Node("teleop2can_transformer")
    {
        // 声明参数
        this->declare_parameter<double>("steering_deadzone", 0.05);
        this->declare_parameter<double>("throttle_deadzone", 0.05);
        this->declare_parameter<double>("brake_deadzone", 0.05);
        this->declare_parameter<double>("boom_deadzone", 0.05);
        this->declare_parameter<double>("bucket_deadzone", 0.05);
        
        // 角度映射范围（度）
        // 大臂范围：-800~800
        // 铲斗范围：-800~800
        this->declare_parameter<double>("arm_angle_min", -800.0);
        this->declare_parameter<double>("arm_angle_max", 800.0);
        this->declare_parameter<double>("shovel_angle_min", -800.0);
        this->declare_parameter<double>("shovel_angle_max", 800.0);
        
        // 速度限制（m/s）
        this->declare_parameter<double>("max_speed", 3.0);
        
        // 获取参数
        steering_deadzone_ = this->get_parameter("steering_deadzone").as_double();
        throttle_deadzone_ = this->get_parameter("throttle_deadzone").as_double();
        brake_deadzone_ = this->get_parameter("brake_deadzone").as_double();
        boom_deadzone_ = this->get_parameter("boom_deadzone").as_double();
        bucket_deadzone_ = this->get_parameter("bucket_deadzone").as_double();
        arm_angle_min_ = this->get_parameter("arm_angle_min").as_double();
        arm_angle_max_ = this->get_parameter("arm_angle_max").as_double();
        shovel_angle_min_ = this->get_parameter("shovel_angle_min").as_double();
        shovel_angle_max_ = this->get_parameter("shovel_angle_max").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        
        // 创建订阅者（订阅远程端控制指令）
        // 使用 BEST_EFFORT QoS 以匹配发布者（参考 keyboard_piston_joint_publisher_2_updated.py）
        rclcpp::QoS teleop_qos(10);
        teleop_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        teleop_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        teleop_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        teleop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/controls/teleop", 
            teleop_qos,
            std::bind(&Teleop2CanTransformer::teleop_callback, this, std::placeholders::_1)
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
        last_gear_ = control::canbus::Chassis::GEAR_NEUTRAL;
        last_steering_ = 0.0;
        last_throttle_ = 0.0;
        last_brake_ = 0.0;
        last_arm_angle_ = 0.0;
        last_shovel_angle_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Teleop2CanTransformer 节点已启动");
        RCLCPP_INFO(this->get_logger(), "死区设置: steering=%.3f, throttle=%.3f, brake=%.3f, boom=%.3f, bucket=%.3f",
                    steering_deadzone_, throttle_deadzone_, brake_deadzone_, boom_deadzone_, bucket_deadzone_);
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
    
    // 将档位字符串转换为 GearPosition 枚举
    control::canbus::Chassis::GearPosition string_to_gear(const std::string& gear_str)
    {
        if (gear_str == "D" || gear_str == "d") {
            return control::canbus::Chassis::GEAR_DRIVE;
        } else if (gear_str == "R" || gear_str == "r") {
            return control::canbus::Chassis::GEAR_REVERSE;
        } else if (gear_str == "N" || gear_str == "n") {
            return control::canbus::Chassis::GEAR_NEUTRAL;
        } else {
            return control::canbus::Chassis::GEAR_NEUTRAL;
        }
    }
    
    void teleop_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "🔔 回调函数被调用！");
        try {
            // 打印接收到的原始输入
            RCLCPP_INFO(this->get_logger(), "============================================================");
            RCLCPP_INFO(this->get_logger(), "📥 收到控制指令: %s", msg->data.c_str());
            RCLCPP_INFO(this->get_logger(), "📥 消息长度: %zu 字节", msg->data.length());
            
            // 解析 JSON
            std::map<std::string, std::string> data = SimpleJsonParser::parse(msg->data);
            
            // 创建 ControlCommand protobuf 消息
            control::ControlCommand cmd;
            
            // 设置 header
            auto* header = cmd.mutable_header();
            auto now = this->now();
            header->set_timestamp_sec(now.seconds());
            header->set_frame_id("base_link");
            
            // 处理转向控制 (steering: -1..1 -> steering_target: -800..800)
            // 支持新格式的 steering 字段
            if (data.find("steering") != data.end()) {
                double steering_input = clamp(SimpleJsonParser::get_double(data["steering"]), -1.0, 1.0);
                
                // 应用死区
                steering_input = apply_deadzone(steering_input, steering_deadzone_);
                
                // 映射到角度范围 [-800, 800]
                // 注意：根据 Python 脚本，转向是反向的
                double steering_target = -steering_input * 800.0;
                cmd.set_steering_target(steering_target);
                last_steering_ = steering_target;
            } else {
                // 如果没有输入，保持上次值或设为0
                cmd.set_steering_target(last_steering_);
            }
            
            // 处理左履带 (leftTrack: -1..1) - 保留格式但不映射到输出
            if (data.find("leftTrack") != data.end()) {
                double left_track = clamp(SimpleJsonParser::get_double(data["leftTrack"]), -1.0, 1.0);
                RCLCPP_DEBUG(this->get_logger(), "收到 leftTrack: %.2f (保留格式，不映射)", left_track);
            }
            
            // 处理右履带 (rightTrack: -1..1) - 保留格式但不映射到输出
            if (data.find("rightTrack") != data.end()) {
                double right_track = clamp(SimpleJsonParser::get_double(data["rightTrack"]), -1.0, 1.0);
                RCLCPP_DEBUG(this->get_logger(), "收到 rightTrack: %.2f (保留格式，不映射)", right_track);
            }
            
            // 处理驾驶室旋转 (swing: -1..1) - 保留格式但不映射到输出
            if (data.find("swing") != data.end()) {
                double swing = clamp(SimpleJsonParser::get_double(data["swing"]), -1.0, 1.0);
                RCLCPP_DEBUG(this->get_logger(), "收到 swing: %.2f (保留格式，不映射)", swing);
            }
            
            // 处理小臂 (stick: -1..1) - 保留格式但不映射到输出（cannode没有小臂字段）
            if (data.find("stick") != data.end()) {
                double stick = clamp(SimpleJsonParser::get_double(data["stick"]), -1.0, 1.0);
                RCLCPP_DEBUG(this->get_logger(), "收到 stick: %.2f (保留格式，不映射)", stick);
            }
            
            // 处理油门控制 (throttle: -1..1 -> throttle: 200..0，反向映射)
            if (data.find("throttle") != data.end()) {
                double throttle_input = clamp(SimpleJsonParser::get_double(data["throttle"]), -1.0, 1.0);
                throttle_input = apply_deadzone(throttle_input, throttle_deadzone_);
                // 映射：-1 -> 200, 1 -> 0
                double throttle_value = (1.0 - throttle_input) / 2.0 * 200.0;
                cmd.set_throttle(throttle_value);
                last_throttle_ = throttle_value;
            } else {
                cmd.set_throttle(last_throttle_);
            }
            
            // 处理刹车控制 (brake: -1..1 -> brake: 1000..0，反向映射)
            if (data.find("brake") != data.end()) {
                double brake_input = clamp(SimpleJsonParser::get_double(data["brake"]), -1.0, 1.0);
                brake_input = apply_deadzone(brake_input, brake_deadzone_);
                // 映射：-1 -> 1000, 1 -> 0
                double brake_value = (1.0 - brake_input) / 2.0 * 1000.0;
                cmd.set_brake(brake_value);
                last_brake_ = brake_value;
            } else {
                cmd.set_brake(last_brake_);
            }
            
            // 处理档位控制
            if (data.find("gear") != data.end()) {
                std::string gear_str = data["gear"];
                control::canbus::Chassis::GearPosition gear = string_to_gear(gear_str);
                cmd.set_gear_location(gear);
                last_gear_ = gear;
            } else {
                cmd.set_gear_location(last_gear_);
            }
            
            // 处理驻车制动
            if (data.find("parking_brake") != data.end()) {
                cmd.set_parking_brake(SimpleJsonParser::get_bool(data["parking_brake"]));
            }
            
            // 处理大臂控制 (boom: -1..1 -> arm_angle: -800~800度)
            if (data.find("boom") != data.end()) {
                double boom_input = clamp(SimpleJsonParser::get_double(data["boom"]), -1.0, 1.0);
                boom_input = apply_deadzone(boom_input, boom_deadzone_);
                
                // 映射到角度范围（度）：-1 -> -800，1 -> 800
                double arm_angle = boom_input * (arm_angle_max_ - arm_angle_min_) / 2.0 + 
                                  (arm_angle_max_ + arm_angle_min_) / 2.0;
                cmd.set_arm_angle(arm_angle);
                cmd.set_arm_enable(true);
                last_arm_angle_ = arm_angle;
            } else {
                cmd.set_arm_angle(last_arm_angle_);
                cmd.set_arm_enable(false);
            }
            
            // 处理铲斗控制 (bucket: -1..1 -> shovel_angle: -800~800度)
            if (data.find("bucket") != data.end()) {
                double bucket_input = clamp(SimpleJsonParser::get_double(data["bucket"]), -1.0, 1.0);
                bucket_input = apply_deadzone(bucket_input, bucket_deadzone_);
                
                // 映射到角度范围（度）：-1 -> 800（因为反向），1 -> -800
                // 注意：根据 Python 脚本，bucket 是反向的
                double shovel_angle = -bucket_input * (shovel_angle_max_ - shovel_angle_min_) / 2.0 + 
                                     (shovel_angle_max_ + shovel_angle_min_) / 2.0;
                cmd.set_shovel_angle(shovel_angle);
                cmd.set_shovel_enable(true);
                last_shovel_angle_ = shovel_angle;
            } else {
                cmd.set_shovel_angle(last_shovel_angle_);
                cmd.set_shovel_enable(false);
            }
            
            // 处理紧急停止
            if (data.find("emergency_stop") != data.end()) {
                bool estop = SimpleJsonParser::get_bool(data["emergency_stop"]);
                cmd.set_estop(estop);
            }
            
            // 处理发动机开关 (power_enable: boolean -> engine_on_off)
            if (data.find("power_enable") != data.end()) {
                cmd.set_engine_on_off(SimpleJsonParser::get_bool(data["power_enable"]));
            }
            
            // 处理喇叭 (horn: boolean) - 使用deprecated字段但保留格式
            if (data.find("horn") != data.end()) {
                bool horn = SimpleJsonParser::get_bool(data["horn"]);
                // ControlCommand 的 horn 字段是 deprecated，但可以设置
                // 注意：protobuf 可能不支持直接设置 deprecated 字段，这里仅记录日志
                RCLCPP_DEBUG(this->get_logger(), "收到 horn: %s (保留格式)", horn ? "true" : "false");
            }
            
            // 处理速度模式 (speed_mode: 'turtle'|'rabbit') - 影响max_speed但不直接映射
            double effective_max_speed = max_speed_;
            if (data.find("speed_mode") != data.end()) {
                std::string speed_mode = data["speed_mode"];
                if (speed_mode == "rabbit") {
                    effective_max_speed = max_speed_;  // 兔子模式：最大速度
                } else if (speed_mode == "turtle") {
                    effective_max_speed = max_speed_ * 0.5;  // 乌龟模式：一半速度
                }
                RCLCPP_DEBUG(this->get_logger(), "速度模式: %s, 有效最大速度: %.2f m/s", 
                           speed_mode.c_str(), effective_max_speed);
            }
            
            // 处理灯光代码 (light_code: number) - 保留格式但不映射到输出
            if (data.find("light_code") != data.end()) {
                int light_code = SimpleJsonParser::get_int(data["light_code"]);
                RCLCPP_DEBUG(this->get_logger(), "收到 light_code: 0x%02X (保留格式，不映射)", light_code);
            }
            
            // 处理液压锁 (hydraulic_lock: boolean) - 保留格式但不映射到输出
            if (data.find("hydraulic_lock") != data.end()) {
                bool hydraulic_lock = SimpleJsonParser::get_bool(data["hydraulic_lock"]);
                RCLCPP_DEBUG(this->get_logger(), "收到 hydraulic_lock: %s (保留格式，不映射)", 
                           hydraulic_lock ? "true" : "false");
            }
            
            // 根据油门和档位计算目标速度
            // 速度模式会影响最大速度
            if (cmd.has_throttle() && cmd.has_gear_location()) {
                double speed = 0.0;
                if (cmd.gear_location() == control::canbus::Chassis::GEAR_DRIVE) {
                    speed = (cmd.throttle() / 200.0) * effective_max_speed;
                } else if (cmd.gear_location() == control::canbus::Chassis::GEAR_REVERSE) {
                    speed = -(cmd.throttle() / 200.0) * effective_max_speed;
                }
                cmd.set_speed(speed);
            }
            
            // 打印转换后的输出信息
            RCLCPP_INFO(this->get_logger(), "📤 转换后的控制命令:");
            
            // 转向 (steering_target: [-800, 800])
            double steering_target = cmd.steering_target();
            RCLCPP_INFO(this->get_logger(), "   转向 (steering_target): %.2f", steering_target);
            if (steering_target < -800.0 || steering_target > 800.0) {
                RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: steering_target 超出范围 [-800, 800]");
            }
            
            // 油门 (throttle: [0, 200])
            double throttle = cmd.throttle();
            RCLCPP_INFO(this->get_logger(), "   油门 (throttle): %.2f", throttle);
            if (throttle < 0.0 || throttle > 200.0) {
                RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: throttle 超出范围 [0, 200]");
            }
            
            // 刹车 (brake: [0, 1000])
            double brake = cmd.brake();
            RCLCPP_INFO(this->get_logger(), "   刹车 (brake): %.2f", brake);
            if (brake < 0.0 || brake > 1000.0) {
                RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: brake 超出范围 [0, 1000]");
            }
            
            // 档位字符串
            std::string gear_str = "UNKNOWN";
            switch(cmd.gear_location()) {
                case control::canbus::Chassis::GEAR_DRIVE: gear_str = "D(前进)"; break;
                case control::canbus::Chassis::GEAR_REVERSE: gear_str = "R(后退)"; break;
                case control::canbus::Chassis::GEAR_NEUTRAL: gear_str = "N(空档)"; break;
                default: gear_str = "N(空档)"; break;
            }
            RCLCPP_INFO(this->get_logger(), "   档位 (gear_location): %s (%d)", gear_str.c_str(), static_cast<int>(cmd.gear_location()));
            
            if (cmd.has_speed()) {
                RCLCPP_INFO(this->get_logger(), "   目标速度 (speed): %.2f m/s", cmd.speed());
            }
            
            // 大臂角度 (arm_angle: [-800, 800]°)
            if (cmd.has_arm_angle()) {
                double arm_angle = cmd.arm_angle();
                if (cmd.arm_enable()) {
                    RCLCPP_INFO(this->get_logger(), "   大臂角度 (arm_angle): %.2f° [启用]", arm_angle);
                } else {
                    RCLCPP_INFO(this->get_logger(), "   大臂角度 (arm_angle): %.2f° [禁用]", arm_angle);
                }
                if (arm_angle < -800.0 || arm_angle > 800.0) {
                    RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: arm_angle 超出范围 [-800, 800]°");
                }
            }
            
            // 铲斗角度 (shovel_angle: [-800, 800]°)
            if (cmd.has_shovel_angle()) {
                double shovel_angle = cmd.shovel_angle();
                if (cmd.shovel_enable()) {
                    RCLCPP_INFO(this->get_logger(), "   铲斗角度 (shovel_angle): %.2f° [启用]", shovel_angle);
                } else {
                    RCLCPP_INFO(this->get_logger(), "   铲斗角度 (shovel_angle): %.2f° [禁用]", shovel_angle);
                }
                if (shovel_angle < -800.0 || shovel_angle > 800.0) {
                    RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: shovel_angle 超出范围 [-800, 800]°");
                }
            }
            
            if (cmd.has_estop()) {
                RCLCPP_INFO(this->get_logger(), "   紧急停止 (estop): %s", cmd.estop() ? "是" : "否");
            }
            
            if (cmd.has_parking_brake()) {
                RCLCPP_INFO(this->get_logger(), "   驻车制动 (parking_brake): %s", cmd.parking_brake() ? "是" : "否");
            }
            
            if (cmd.has_engine_on_off()) {
                RCLCPP_INFO(this->get_logger(), "   发动机 (engine_on_off): %s", cmd.engine_on_off() ? "开启" : "关闭");
            }
            
            // 打印新格式字段（保留格式但不映射到输出）
            if (data.find("leftTrack") != data.end() || data.find("rightTrack") != data.end() ||
                data.find("swing") != data.end() || data.find("stick") != data.end() ||
                data.find("speed_mode") != data.end() || data.find("light_code") != data.end() ||
                data.find("hydraulic_lock") != data.end() || data.find("horn") != data.end()) {
                RCLCPP_INFO(this->get_logger(), "   新格式字段（保留格式，不映射到输出）:");
                if (data.find("leftTrack") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     leftTrack: %.2f", SimpleJsonParser::get_double(data["leftTrack"]));
                }
                if (data.find("rightTrack") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     rightTrack: %.2f", SimpleJsonParser::get_double(data["rightTrack"]));
                }
                if (data.find("swing") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     swing: %.2f", SimpleJsonParser::get_double(data["swing"]));
                }
                if (data.find("stick") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     stick: %.2f", SimpleJsonParser::get_double(data["stick"]));
                }
                if (data.find("speed_mode") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     speed_mode: %s", data["speed_mode"].c_str());
                }
                if (data.find("light_code") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     light_code: 0x%02X", SimpleJsonParser::get_int(data["light_code"]));
                }
                if (data.find("hydraulic_lock") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     hydraulic_lock: %s", SimpleJsonParser::get_bool(data["hydraulic_lock"]) ? "true" : "false");
                }
                if (data.find("horn") != data.end()) {
                    RCLCPP_INFO(this->get_logger(), "     horn: %s", SimpleJsonParser::get_bool(data["horn"]) ? "true" : "false");
                }
            }
            
            // 序列化 protobuf 消息
            std::string serialized_data;
            cmd.SerializeToString(&serialized_data);
            
            // 创建 ROS2 消息并发布（序列化版本）
            auto ros_msg = sa_msgs::msg::ProtoAdapter();
            ros_msg.pb.assign(serialized_data.begin(), serialized_data.end());
            vehicle_cmd_pub_->publish(ros_msg);
            
            // 创建并发布非序列化消息（用于调试和查看）
            auto debug_msg = teleoptocantransformer::msg::VehicleCommand();
            debug_msg.header.stamp = this->now();
            debug_msg.header.frame_id = "base_link";
            debug_msg.steering_target = cmd.steering_target();
            debug_msg.throttle = cmd.throttle();
            debug_msg.brake = cmd.brake();
            debug_msg.gear_location = static_cast<int32_t>(cmd.gear_location());
            debug_msg.speed = cmd.has_speed() ? cmd.speed() : 0.0;
            debug_msg.arm_angle = cmd.has_arm_angle() ? cmd.arm_angle() : 0.0;
            debug_msg.arm_enable = cmd.has_arm_angle() ? cmd.arm_enable() : false;
            debug_msg.shovel_angle = cmd.has_shovel_angle() ? cmd.shovel_angle() : 0.0;
            debug_msg.shovel_enable = cmd.has_shovel_angle() ? cmd.shovel_enable() : false;
            debug_msg.estop = cmd.has_estop() ? cmd.estop() : false;
            debug_msg.parking_brake = cmd.has_parking_brake() ? cmd.parking_brake() : false;
            debug_msg.engine_on_off = cmd.has_engine_on_off() ? cmd.engine_on_off() : false;
            vehicle_cmd_debug_pub_->publish(debug_msg);
            
            RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command (protobuf 大小: %zu 字节)", serialized_data.size());
            RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command_debug (非序列化消息)");
            RCLCPP_INFO(this->get_logger(), "============================================================");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理控制命令时出错: %s", e.what());
        }
    }
    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_sub_;
    rclcpp::Publisher<sa_msgs::msg::ProtoAdapter>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<teleoptocantransformer::msg::VehicleCommand>::SharedPtr vehicle_cmd_debug_pub_;
    
    // 死区参数
    double steering_deadzone_;
    double throttle_deadzone_;
    double brake_deadzone_;
    double boom_deadzone_;
    double bucket_deadzone_;
    
    // 角度映射范围
    double arm_angle_min_;
    double arm_angle_max_;
    double shovel_angle_min_;
    double shovel_angle_max_;
    
    // 速度限制
    double max_speed_;
    
    // 上次的值（用于保持状态）
    control::canbus::Chassis::GearPosition last_gear_;
    double last_steering_;
    double last_throttle_;
    double last_brake_;
    double last_arm_angle_;
    double last_shovel_angle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop2CanTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
