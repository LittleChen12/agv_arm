#include "agv_hardware/agv_hardware.hpp"

namespace agv_hardware {
    
hardware_interface::CallbackReturn AgvHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    remoteIP = "192.168.2.160";
    localIP = "192.168.2.100";

    robot_ = std::make_shared<rokae::xMateRobot>();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    try {
        robot_->connectToRobot(remoteIP, localIP);

        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                    "Connected to robot at %s", remoteIP.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    catch (const rokae::NetworkException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                     "Network error connecting to robot at %s: %s",
                     remoteIP.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    catch (const rokae::ExecutionException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                     "Execution error connecting to robot at %s: %s",
                     remoteIP.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 上电并切换到实时模式
    robot_->setRtNetworkTolerance(20, ec);
    robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    robot_->setPowerState(true, ec);

    // 启动状态接收
    robot_->startReceiveRobotState(std::chrono::milliseconds(1),
        {rokae::RtSupportedFields::jointPos_m});

    rt_controller_ = robot_->getRtMotionController().lock();
    if (!rt_controller_) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                     "获取 RT 控制器失败");
        return hardware_interface::CallbackReturn::ERROR;
    }

    rt_controller_->startMove(rokae::RtControllerMode::jointPosition);

    RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                "硬件激活完成，RT 循环已启动");

    // 读取初始关节位置
    joint_positions_ = robot_->jointPos(ec); 

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    if (rt_controller_) {
        rt_controller_->stopLoop();
        rt_controller_->stopMove();
        rt_controller_.reset();
    }

    robot_->stopReceiveRobotState();
    robot_->setPowerState(false, ec);
    robot_->disconnectFromRobot(ec);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AgvHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    set_state("arm1_joint1/position", joint_positions_[0]);
    set_state("arm1_joint2/position", joint_positions_[1]);
    set_state("arm1_joint3/position", joint_positions_[2]);
    set_state("arm1_joint4/position", joint_positions_[3]);
    set_state("arm1_joint5/position", joint_positions_[4]);
    set_state("arm1_joint6/position", joint_positions_[5]);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvHardwareInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // 首次进入时设置 RT 控制循环
    if (callback_init == 0)
    {
        static std::atomic<int> counter{0};
        rt_controller_->setControlLoop<rokae::JointPosition>(
            [this]() -> rokae::JointPosition {
                rokae::JointPosition cmd;
                cmd.joints.resize(6);

                // 每次回调都读取当前角度
                std::array<double, 6> current = robot_->jointPos(ec);

                for (size_t i = 0; i < 6; ++i) {
                    joint_positions_[i] = current[i];
                    // 防御：若命令非有限值则回退到当前角度
                    cmd.joints[i] = std::isfinite(joint_commands_[i]) ?
                                    joint_commands_[i] : current[i];
                }
                return cmd;
            },
            0,
            true
        );

        rt_controller_->startLoop(false); // 启动循环（非阻塞）
        callback_init = 1;
    }

    std::lock_guard<std::mutex> lock(io_mutex);

    // 读取控制器命令
    double cmds[6] = {
        get_command("arm1_joint1/position"),
        get_command("arm1_joint2/position"),
        get_command("arm1_joint3/position"),
        get_command("arm1_joint4/position"),
        get_command("arm1_joint5/position"),
        get_command("arm1_joint6/position")
    };

    // 清洗：若非有限值则回退到当前状态
    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(cmds[i])) {
            cmds[i] = joint_positions_[i];  // 使用当前角度作为安全值
        }
        joint_commands_[i] = cmds[i];
    }

    return hardware_interface::return_type::OK;
}

} // namespace agv_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    agv_hardware::AgvHardwareInterface,
    hardware_interface::SystemInterface)