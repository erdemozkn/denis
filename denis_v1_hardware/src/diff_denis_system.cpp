#include "denis_v1_hardware/diff_denis_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace denis_diff_hardware
{
    hardware_interface::CallbackReturn DiffDenisSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baut_rate = std::stoi(info_.hardware_parameters["baut_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

        wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
    }

    std::vector<hardware_interface::StateInterface> DiffDenisSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffDenisSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));
    }

    hardware_interface::CallbackReturn DiffDenisSystemHardware::on_configure(
        const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(this->get_logger("DiffDenisSystemHardware"), "Activating...");

        comms_.connect(cfg_.device, cfg_.timeout_ms);

        RCLCPP_INFO(this->get_logger("DiffDenisSystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDenisSystemHardware::on_cleanup(
        rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(this->get_logger("DiffDenisSystemHardware"), "Deactivating...");

        comms_.disconnect();

        RCLCPP_Info(this->get_logger("DiffDenisSystemHardware"), "Successfully deactivated!")
    }

    hardware_interface::CallbackReturn DiffDenisSystemHardware::on_activate(
        const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(this->get_logger("DiffDenisSystemHardware"), "Activating...");

        comms_.set_pid_values(30,20,0,100);

        RCLCPP_INFO(this->get_logger("DiffDenisSystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffDenisSystemHardware::on_deactivate(
        rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(this->get_logger("DiffDenisSystemHardware"), "Deactivating...");
        // nothing to do for now
        RCLCPP_Info(this->get_logger("DiffDenisSystemHardware"), "Successfully deactivated!")
    }

    hardware_interface::return_type DiffDenisSystemHardware::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

        double delta_seconds = period.seconds;

        // left wheel
        double pos_prev = wheel_l_.pos;

        wheel_l_.pos = wheel_l_.calcEncAngle();
        wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

        // right wheel
        pos_prev = wheel_r_.pos;

        wheel_r_.pos = wheel_r_.calcEncAngle();
        wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffDenisSystemHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
        int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
        comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
        return hardware_interface::return_type::OK;
    }
} // namespace denis_diff_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    denis_diff_hardware::DiffDenisSystemHardware, hardware_interface::SystemInterface)