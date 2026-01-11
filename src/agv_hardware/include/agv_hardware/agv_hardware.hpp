#ifndef AGV_HARDWARE_HPP
#define AGV_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include "rokae/robot.h"

namespace agv_hardware {

class AgvHardwareInterface : public hardware_interface::SystemInterface {
public:
    // lifecycle node overrides
    hardware_interface::CallbackReturn 
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface overrides
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareInfo & params) override;
    hardware_interface::return_type 
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type 
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;   
private:
    std::string remoteIP;
    std::string localIP;
    error_code ec;
    std::shared_ptr<rokae::xMateRobot> robot_;

    std::array<double, 6> joint_positions_{};
    std::array<double, 6> joint_commands_{};
    std::mutex io_mutex;

    std::shared_ptr<rokae::RtMotionControlCobot<6>> rt_controller_;

    bool callback_init = 0;
};

}; // namespace agv_hardware


#endif // AGV_HARDWARE_HPP
