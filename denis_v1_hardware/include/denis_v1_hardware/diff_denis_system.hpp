#ifndef DENIS_DIFF_SYSTEM_HPP
#define DENIS_DIFF_SYSTEM_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "denis_v1_hardware/arduino_comms.hpp"
#include "denis_v1_hardware/wheel.hpp"

namespace denis_diff_hardware
{
    struct Config
    {
        std::string left_wheel_name = "";
        std::string right_wheel_name = "";
        float loop_rate = 0.0;
        std::string device = "";
        int baud_rate = 0;
        int timeout_ms = 0;
        int enc_counts_per_rev = 0;
        int pid_p = 0;
        int pid_d = 0;
        int pid_i = 0;
        int pid_o = 0;
    };

    class DiffDenisSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        ArduinoComms comms_;
        Config cfg_;
        Wheel wheel_l_;
        Wheel wheel_r_;
    };

} // namespace denis_diff

#endif