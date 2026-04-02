#ifndef TCP_HARDWARE_DRIVER_HPP_
#define TCP_HARDWARE_DRIVER_HPP_

#include "hardware_node/i_hardware_driver.hpp"
#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep

// 前向声明
class SimpleTcpClient;

class TcpHardwareDriver : public IHardwareDriver
{
public:
    explicit TcpHardwareDriver(rclcpp::Logger logger,
                               std::string plc_ip, int plc_port,
                               std::string temp_ip, int temp_port);
    ~TcpHardwareDriver() override;

    void update() override;

    // --- Service Handlers ---
    void handle_set_hardware_regulator_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request,
        AsyncCallback callback) override;

    void handle_set_hardware_circuit_settings_request(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request,
        AsyncCallback callback) override;

    void handle_regulator_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request,
        AsyncCallback callback) override;

    void handle_circuit_breaker_command(
        const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request,
        AsyncCallback callback) override;

    void handle_set_control_mode(
        const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request,
        AsyncCallback callback) override;

    void handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg) override;
    void handle_clear_alarm() override;

    // --- Getters ---
    bool get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status) override;
    bool get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status) override;
    bool get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) override;
    bool get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) override;
    bool get_system_status(ros2_interfaces::msg::HardwareSystemStatus& status) override;

private:
    void initialize_default_states();

    // --- Internal Logic ---
    void read_plc_data();
    void read_temp_monitor_data();
    void parse_plc_buffer(const std::vector<uint8_t>& buffer);
    void parse_temp_buffer(const std::vector<uint8_t>& buffer);

    // 高频电压控制线程 (15ms)
    void voltage_keep_alive_loop();

    // Modbus Helpers
    bool modbus_read_holding_registers(SimpleTcpClient* client, uint8_t unit_id, uint16_t start_addr, uint16_t count, std::vector<uint8_t>& out_data);
    bool modbus_write_single_register(SimpleTcpClient* client, uint8_t unit_id, uint16_t addr, uint16_t value);
    float parse_float_abcd(const uint8_t* ptr);
    uint16_t parse_uint16(const uint8_t* ptr);

    rclcpp::Logger logger_;
    std::mutex data_mutex_;

    std::unique_ptr<SimpleTcpClient> client_plc_;
    std::unique_ptr<SimpleTcpClient> client_temp_;

    uint64_t update_tick_count_ = 0;
    uint64_t log_throttle_count_ = 0; // 用于控制日志打印频率

    // Data Cache
    std::map<uint8_t, ros2_interfaces::msg::RegulatorStatus> cache_reg_status_;
    std::map<uint8_t, ros2_interfaces::msg::HardwareCircuitStatus> cache_circ_status_;
    std::map<uint8_t, ros2_interfaces::msg::RegulatorSettings> cache_reg_settings_;
    std::map<uint8_t, ros2_interfaces::msg::HardwareCircuitSettings> cache_circ_settings_;
    ros2_interfaces::msg::HardwareSystemStatus cache_system_status_; // [新增]

    // --- Voltage Control State ---
    std::atomic<bool> keep_alive_running_;
    std::thread keep_alive_thread_;

    // 存储当前的调压指令: key=regulator_id (1,2), value=command (1=Up, 2=Down, 0/3=Stop)
    std::map<uint8_t, uint8_t> active_voltage_cmd_;
    std::mutex cmd_mutex_;

    // --- PLC Register Address Constants (Updated) ---
    // 1. Commands (Write Only - Single Pulse 256 or Keep Alive 256)
    static const uint16_t ADDR_CMD_REG1_UP   = 0x0000;
    static const uint16_t ADDR_CMD_REG1_DOWN = 0x0001;
    static const uint16_t ADDR_CMD_REG2_UP   = 0x0002;
    static const uint16_t ADDR_CMD_REG2_DOWN = 0x0003;

    static const uint16_t ADDR_CMD_REG1_CLOSE = 0x0004; // 主调压器合闸
    static const uint16_t ADDR_CMD_REG1_OPEN  = 0x0005;
    static const uint16_t ADDR_CMD_REG2_CLOSE = 0x0006; // 辅调压器合闸
    static const uint16_t ADDR_CMD_REG2_OPEN  = 0x0007;

    static const uint16_t ADDR_CMD_C1_TEST_CLOSE = 0x0008; // 1#回路试验合闸
    static const uint16_t ADDR_CMD_C1_TEST_OPEN  = 0x0009;
    static const uint16_t ADDR_CMD_C2_TEST_CLOSE = 0x000A; // 2#回路试验合闸
    static const uint16_t ADDR_CMD_C2_TEST_OPEN  = 0x000B;
    static const uint16_t ADDR_CMD_C1_SIM_CLOSE  = 0x000C; // 1#回路模拟合闸
    static const uint16_t ADDR_CMD_C1_SIM_OPEN   = 0x000D;
    static const uint16_t ADDR_CMD_C2_SIM_CLOSE  = 0x000E; // 2#回路模拟合闸
    static const uint16_t ADDR_CMD_C2_SIM_OPEN   = 0x000F;

    // 2. Settings / Mode (Read/Write)
    static const uint16_t ADDR_MODE_C1_TEST = 0x0010;
    static const uint16_t ADDR_MODE_C1_SIM  = 0x0011;
    static const uint16_t ADDR_MODE_C2_TEST = 0x0012;
    static const uint16_t ADDR_MODE_C2_SIM  = 0x0013;

    // 3. Status Read Range
    // Start reading from 0x0010 (Mode) to cover everything up to 0x005D (Last Float Aux OCP)
    static const uint16_t ADDR_DATA_START = 0x0010;
    // End is VD184 (0x005C + 0x005D).
    // Length = 0x005D - 0x0010 + 1 = 78 registers.
    static const uint16_t ADDR_DATA_LEN   = 78;
};

#endif // TCP_HARDWARE_DRIVER_HPP_
