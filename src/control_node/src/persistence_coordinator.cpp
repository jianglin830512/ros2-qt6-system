#include "control_node/persistence_coordinator.hpp"

PersistenceCoordinator::PersistenceCoordinator(
    StateManager* state_manager,
    std::shared_ptr<ControlNodeContext> context)
    : state_manager_(state_manager),
    context_(context)
{
    RCLCPP_INFO(context_->logging_interface->get_logger(), "Persistence Coordinator initialized.");
}

void PersistenceCoordinator::save_system_settings(PersistenceCallback callback)
{
    auto client = context_->save_system_settings_client;
    if (!client) {
        RCLCPP_ERROR(context_->logging_interface->get_logger(), "Save system settings client is not initialized!");
        if (callback) callback(false, "Client not initialized");
        return;
    }

    if (!client->service_is_ready()) {
        RCLCPP_WARN(context_->logging_interface->get_logger(), "Save system settings service not available.");
        if (callback) callback(false, "Service not available");
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetSystemSettings::Request>();
    request->settings = state_manager_->get_system_settings();

    auto response_callback = [this, callback](rclcpp::Client<ros2_interfaces::srv::SetSystemSettings>::SharedFuture future) {
        if (!callback) return;
        try {
            auto result = future.get();
            callback(result->success, result->message);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(context_->logging_interface->get_logger(), "Exception while calling save_system_settings service: %s", e.what());
            callback(false, "Service call failed with exception");
        }
    };

    client->async_send_request(request, response_callback);
}

void PersistenceCoordinator::save_regulator_settings(uint8_t regulator_id, PersistenceCallback callback)
{
    auto client = context_->save_regulator_settings_client;
    if (!client) {
        RCLCPP_ERROR(context_->logging_interface->get_logger(), "Save regulator settings client is not initialized!");
        if (callback) callback(false, "Client not initialized");
        return;
    }

    if (!client->service_is_ready()) {
        RCLCPP_WARN(context_->logging_interface->get_logger(), "Save regulator settings service not available.");
        if (callback) callback(false, "Service not available");
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetRegulatorSettings::Request>();
    request->regulator_id = regulator_id;
    request->settings = state_manager_->get_regulator_settings(regulator_id);

    auto response_callback = [this, callback, regulator_id](rclcpp::Client<ros2_interfaces::srv::SetRegulatorSettings>::SharedFuture future) {
        if (!callback) return;
        try {
            auto result = future.get();
            callback(result->success, result->message);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(context_->logging_interface->get_logger(), "Exception while calling save_regulator_settings for ID %u: %s", regulator_id, e.what());
            callback(false, "Service call failed with exception");
        }
    };

    client->async_send_request(request, response_callback);
}

void PersistenceCoordinator::save_circuit_settings(uint8_t circuit_id, PersistenceCallback callback)
{
    auto client = context_->save_circuit_settings_client;
    if (!client) {
        RCLCPP_ERROR(context_->logging_interface->get_logger(), "Save circuit settings client is not initialized!");
        if (callback) callback(false, "Client not initialized");
        return;
    }

    if (!client->service_is_ready()) {
        RCLCPP_WARN(context_->logging_interface->get_logger(), "Save circuit settings service is not available.");
        if (callback) callback(false, "Service not available");
        return;
    }

    auto request = std::make_shared<ros2_interfaces::srv::SetCircuitSettings::Request>();
    request->circuit_id = circuit_id;
    request->settings = state_manager_->get_circuit_settings(circuit_id);

    auto response_callback = [this, callback, circuit_id](rclcpp::Client<ros2_interfaces::srv::SetCircuitSettings>::SharedFuture future) {
        if (!callback) return;
        try {
            auto result = future.get();
            callback(result->success, result->message);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(context_->logging_interface->get_logger(), "Exception while calling save_circuit_settings for ID %u: %s", circuit_id, e.what());
            callback(false, "Service call failed with exception");
        }
    };

    client->async_send_request(request, response_callback);
}
