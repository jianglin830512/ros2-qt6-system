#include "hardware_node/tcp_hardware_driver.hpp"

// Windows / Linux Socket includes
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close
#endif

#include <thread>
#include <vector>
#include <cstring>
// #include <cmath>
#include <algorithm>
// #include <iostream> // For formatting logs if needed

// ============================================================================
// Internal Helper Class: SimpleTcpClient (Same as before)
// ============================================================================
class SimpleTcpClient {
public:
    SimpleTcpClient(rclcpp::Logger logger, std::string ip, int port)
        : logger_(logger), ip_(ip), port_(port), sock_(INVALID_SOCKET) {
        // 初始化尝试时间为过去，确保启动时能立即尝试第一次连接
        last_connect_attempt_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    }

    ~SimpleTcpClient() { disconnect(); }

    // 核心连接逻辑：处理长连接保持与失败后的3秒重连
    bool ensure_connected() {
        // 1. 如果连接正常，直接返回 true (保持长连接)
        if (sock_ != INVALID_SOCKET) return true;

        // 2. 检查是否处于重连冷却期 (3秒)
        auto now = std::chrono::steady_clock::now();
        if ((now - last_connect_attempt_) < std::chrono::seconds(3)) {
            return false; // 还在冷却中，暂不尝试
        }

        // 更新尝试时间
        last_connect_attempt_ = now;

        // 3. 打印尝试连接日志
        RCLCPP_WARN(logger_, "TCP: Attempting to connect/reconnect to %s:%d ...", ip_.c_str(), port_);

        // 4. 创建 Socket
        sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock_ == INVALID_SOCKET) {
            RCLCPP_ERROR(logger_, "TCP: Failed to create socket for %s:%d", ip_.c_str(), port_);
            return false;
        }

        // 5. 设置非阻塞模式 (为了控制连接超时)
#ifdef _WIN32
        u_long mode = 1; ioctlsocket(sock_, FIONBIO, &mode);
#else
        int flags = fcntl(sock_, F_GETFL, 0); fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
#endif

        struct sockaddr_in serv_addr;
        std::memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port_);
        inet_pton(AF_INET, ip_.c_str(), &serv_addr.sin_addr);

        // 6. 发起连接
        int res = connect(sock_, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

        // 7. 使用 select 等待连接建立 (超时 500ms)
        fd_set write_fds; FD_ZERO(&write_fds); FD_SET(sock_, &write_fds);
        struct timeval tv = {0, 500000};
        int sel_res = select((int)sock_ + 1, NULL, &write_fds, NULL, &tv);

        if (sel_res > 0) {
            // 进一步检查 Socket 错误状态 (确保不是被拒绝)
            int so_error = 0;
            socklen_t len = sizeof(so_error);
#ifdef _WIN32
            getsockopt(sock_, SOL_SOCKET, SO_ERROR, (char*)&so_error, &len);
#else
            getsockopt(sock_, SOL_SOCKET, SO_ERROR, &so_error, &len);
#endif
            if (so_error == 0) {
                // 连接成功！
                // 恢复为阻塞模式或设置接收超时 (这里设置为 200ms 超时)
#ifdef _WIN32
                mode = 0; ioctlsocket(sock_, FIONBIO, &mode);
                DWORD timeout = 200; setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
                flags = fcntl(sock_, F_GETFL, 0); fcntl(sock_, F_SETFL, flags & ~O_NONBLOCK);
                struct timeval r_tv = {0, 200000}; setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &r_tv, sizeof(r_tv));
#endif
                RCLCPP_INFO(logger_, "TCP: Successfully connected to %s:%d", ip_.c_str(), port_);
                return true;
            }
        }

        // 8. 连接失败处理
        RCLCPP_ERROR(logger_, "TCP: Connection failed to %s:%d. Retrying in 3s...", ip_.c_str(), port_);
        close_socket_internal();
        return false;
    }

    void disconnect() {
        if (sock_ != INVALID_SOCKET) {
            RCLCPP_WARN(logger_, "TCP: Disconnecting from %s:%d", ip_.c_str(), port_);
            close_socket_internal();
        }
    }

    bool write_bytes(const std::vector<uint8_t>& data) {
        if (!ensure_connected()) return false;

        if (send(sock_, (const char*)data.data(), (int)data.size(), 0) == SOCKET_ERROR) {
            RCLCPP_ERROR(logger_, "TCP: Send error to %s:%d", ip_.c_str(), port_);
            disconnect(); // 发送失败视为连接断开
            return false;
        }
        return true;
    }

    bool read_bytes(std::vector<uint8_t>& buffer, size_t expected_size) {
        if (!ensure_connected()) return false;

        buffer.resize(expected_size);
        size_t total = 0;
        while (total < expected_size) {
            int n = recv(sock_, (char*)buffer.data() + total, (int)(expected_size - total), 0);
            if (n <= 0) {
                // n=0: 对端关闭; n<0: 错误
                RCLCPP_ERROR(logger_, "TCP: Recv error/closed from %s:%d", ip_.c_str(), port_);
                disconnect(); // 读取失败视为连接断开
                return false;
            }
            total += n;
        }
        return true;
    }

private:
    void close_socket_internal() {
        if (sock_ != INVALID_SOCKET) {
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
        }
    }

    rclcpp::Logger logger_;
    std::string ip_;
    int port_;
    SOCKET sock_;
    std::chrono::steady_clock::time_point last_connect_attempt_;
};

// ============================================================================
// Modbus Write Float Helper
// ============================================================================
bool modbus_write_float(SimpleTcpClient* client, uint8_t unit_id, uint16_t addr, float value) {
    uint32_t raw;
    std::memcpy(&raw, &value, 4);

    uint8_t b0 = (raw >> 24) & 0xFF;
    uint8_t b1 = (raw >> 16) & 0xFF;
    uint8_t b2 = (raw >> 8) & 0xFF;
    uint8_t b3 = raw & 0xFF;

    std::vector<uint8_t> req(17);
    req[0] = 0; req[1] = 0;
    req[2] = 0; req[3] = 0;
    req[4] = 0; req[5] = 11;
    req[6] = unit_id;
    req[7] = 0x10;              // FC 16
    req[8] = (addr >> 8) & 0xFF; req[9] = addr & 0xFF;
    req[10] = 0; req[11] = 2;   // 2 Regs
    req[12] = 4;
    req[13] = b0; req[14] = b1;
    req[15] = b2; req[16] = b3;

    if (!client->write_bytes(req)) return false;
    std::vector<uint8_t> resp;
    return client->read_bytes(resp, 12);
}

// ============================================================================
// TcpHardwareDriver Implementation
// ============================================================================

TcpHardwareDriver::TcpHardwareDriver(rclcpp::Logger logger,
                                     std::string plc_ip, int plc_port,
                                     std::string temp_ip, int temp_port)
    : logger_(logger)
{
#ifdef _WIN32
    WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    initialize_default_states();

    client_plc_ = std::make_unique<SimpleTcpClient>(logger_, plc_ip, plc_port);
    client_temp_ = std::make_unique<SimpleTcpClient>(logger_, temp_ip, temp_port);

    active_voltage_cmd_[1] = 0;
    active_voltage_cmd_[2] = 0;

    keep_alive_running_ = true;
    keep_alive_thread_ = std::thread(&TcpHardwareDriver::voltage_keep_alive_loop, this);

    RCLCPP_INFO(logger_, "TcpHardwareDriver Started. PLC: %s:%d (15ms cycle)", plc_ip.c_str(), plc_port);
}

TcpHardwareDriver::~TcpHardwareDriver()
{
    keep_alive_running_ = false;
    if (keep_alive_thread_.joinable()) {
        keep_alive_thread_.join();
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

void TcpHardwareDriver::initialize_default_states()
{
    for (uint8_t id = 1; id <= 2; ++id) {
        cache_reg_status_[id].regulator_id = id;
        cache_circ_status_[id].circuit_id = id;
        cache_circ_status_[id].test_loop.temperature_array.fill(0.0);
        cache_circ_status_[id].ref_loop.temperature_array.fill(0.0);
    }
}

// ----------------------------------------------------------------------------
// Voltage Keep-Alive Thread (Updated to 15ms)
// ----------------------------------------------------------------------------
void TcpHardwareDriver::voltage_keep_alive_loop()
{
    while (keep_alive_running_) {
        // Updated frequency: 15ms
        auto next_wake = std::chrono::steady_clock::now() + std::chrono::milliseconds(15);

        std::map<uint8_t, uint8_t> cmds;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            cmds = active_voltage_cmd_;
        }

        for (auto const& [id, cmd] : cmds) {
            uint16_t addr = 0xFFFF;
            // cmd: 1=Up, 2=Down
            if (cmd == 1) {
                addr = (id == 1) ? ADDR_CMD_REG1_UP : ADDR_CMD_REG2_UP;
            }
            else if (cmd == 2) {
                addr = (id == 1) ? ADDR_CMD_REG1_DOWN : ADDR_CMD_REG2_DOWN;
            }

            if (addr != 0xFFFF) {
                // Send 256 constantly to keep moving
                modbus_write_single_register(client_plc_.get(), 1, addr, 256);
            }
        }

        std::this_thread::sleep_until(next_wake);
    }
}

// ----------------------------------------------------------------------------
// Main Update Loop (Runs at ~5Hz / 200ms)
// ----------------------------------------------------------------------------
void TcpHardwareDriver::update()
{
    update_tick_count_++;

    // Read PLC Status (0x0010 to 0x0058)
    read_plc_data();

    // Read Temp (Every 1s -> 5 * 200ms)
    if (update_tick_count_ % 5 == 0) {
        read_temp_monitor_data();
    }
}

void TcpHardwareDriver::read_plc_data()
{
    std::vector<uint8_t> data;
    // Read from 0x0010, Length 74
    if (modbus_read_holding_registers(client_plc_.get(), 1, ADDR_DATA_START, ADDR_DATA_LEN, data)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        parse_plc_buffer(data);
    }
}

void TcpHardwareDriver::read_temp_monitor_data()
{
    std::vector<uint8_t> data;
    if (modbus_read_holding_registers(client_temp_.get(), 1, 0, 80, data)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        parse_temp_buffer(data);
    }
}

// ----------------------------------------------------------------------------
// Parsers
// ----------------------------------------------------------------------------
void TcpHardwareDriver::parse_plc_buffer(const std::vector<uint8_t>& buffer)
{
    // Expected bytes: 74 regs * 2 = 148 bytes
    if (buffer.size() < 148) return;

    // Helper: Offset relative to 0x0010
    auto get_ptr = [&](uint16_t addr) -> const uint8_t* {
        int offset_reg = addr - ADDR_DATA_START;
        if (offset_reg < 0) return nullptr;
        return buffer.data() + (offset_reg * 2);
    };

    auto& r1 = cache_reg_status_[1];
    auto& c1 = cache_circ_status_[1];
    auto& r2 = cache_reg_status_[2];
    auto& c2 = cache_circ_status_[2];

    // ========================================================================
    // 1. Control Mode & Source (0x0010 - 0x0013)
    // ========================================================================
    // C1 Mode (VW32 -> 0x0010)
    uint16_t c1_mode = parse_uint16(get_ptr(0x0010));
    c1.plc_control_mode = (c1_mode == 2) ? 2 : 1; // 1=Manual, 2=Auto

    // C2 Mode (VW34 -> 0x0011)
    uint16_t c2_mode = parse_uint16(get_ptr(0x0011));
    c2.plc_control_mode = (c2_mode == 2) ? 2 : 1;

    // C1 Source (VW36 -> 0x0012)
    uint16_t c1_src = parse_uint16(get_ptr(0x0012));
    c1.plc_control_source = (c1_src == 2) ? 2 : 1; // 1=Test, 2=Ref

    // C2 Source (VW38 -> 0x0013)
    uint16_t c2_src = parse_uint16(get_ptr(0x0013));
    c2.plc_control_source = (c2_src == 2) ? 2 : 1;

    // ========================================================================
    // 2. Bit Parsing (0x0014, 0x0015)
    // ========================================================================
    // Register 0x0014 (VW40) => [HighByte: V40][LowByte: V41]
    uint16_t w_v40 = parse_uint16(get_ptr(0x0014));
    uint8_t v40 = (w_v40 >> 8) & 0xFF; // High
    uint8_t v41 = w_v40 & 0xFF;        // Low

    // Register 0x0015 (VW42) => [HighByte: V42][LowByte: V43]
    uint16_t w_v42 = parse_uint16(get_ptr(0x0015));
    uint8_t v42 = (w_v42 >> 8) & 0xFF; // High

    // Mapping based on Table:
    // V40.3 Main Closed, V40.4 Main Open
    // V40.5 Aux Closed,  V40.6 Aux Open
    r1.breaker_closed_switch_ack = (v40 >> 3) & 0x01;
    r1.breaker_opened_switch_ack = (v40 >> 4) & 0x01;
    r2.breaker_closed_switch_ack = (v40 >> 5) & 0x01;
    r2.breaker_opened_switch_ack = (v40 >> 6) & 0x01;

    // V40.7 Main Test Close, V41.0 Main Test Open
    c1.test_loop.breaker_closed_switch_ack = (v40 >> 7) & 0x01;
    c1.test_loop.breaker_opened_switch_ack = (v41 >> 0) & 0x01;

    // V41.1 Main Ref Close, V41.2 Main Ref Open
    c1.ref_loop.breaker_closed_switch_ack = (v41 >> 1) & 0x01;
    c1.ref_loop.breaker_opened_switch_ack = (v41 >> 2) & 0x01;

    // V41.3 Aux Test Close, V41.4 Aux Test Open
    c2.test_loop.breaker_closed_switch_ack = (v41 >> 3) & 0x01;
    c2.test_loop.breaker_opened_switch_ack = (v41 >> 4) & 0x01;

    // V41.5 Aux Ref Close, V41.6 Aux Ref Open
    c2.ref_loop.breaker_closed_switch_ack = (v41 >> 5) & 0x01;
    c2.ref_loop.breaker_opened_switch_ack = (v41 >> 6) & 0x01;

    // V41.7 Main Limit Up, V42.0 Main Limit Down
    r1.upper_limit_switch_on = (v41 >> 7) & 0x01;
    r1.lower_limit_switch_on = (v42 >> 0) & 0x01;

    // V42.1 Aux Limit Up, V42.2 Aux Limit Down
    r2.upper_limit_switch_on = (v42 >> 1) & 0x01;
    r2.lower_limit_switch_on = (v42 >> 2) & 0x01;

    // Voltage Direction Logic (New)
    // V42.3 Main Up, V42.4 Main Down
    bool m_up = (v42 >> 3) & 0x01;
    bool m_down = (v42 >> 4) & 0x01;
    if (m_up && !m_down) r1.voltage_direction = 1;      // Increasing
    else if (!m_up && m_down) r1.voltage_direction = -1;// Decreasing
    else if (!m_up && !m_down) r1.voltage_direction = 0;// Static
    else r1.voltage_direction = -2; // Error state

    // V42.5 Aux Up, V42.6 Aux Down
    bool a_up = (v42 >> 5) & 0x01;
    bool a_down = (v42 >> 6) & 0x01;
    if (a_up && !a_down) r2.voltage_direction = 1;
    else if (!a_up && a_down) r2.voltage_direction = -1;
    else if (!a_up && !a_down) r2.voltage_direction = 0;
    else r2.voltage_direction = -2;

    // ========================================================================
    // 3. Floats (0x002E - 0x0058)
    // ========================================================================

    // -- Current Change Range (Readback in %, need * 100 ?) --
    // The PLC sends Float 0.0-1.0 or 0-100? The user said "Need convert to float then /100 when writing".
    // This implies PLC stores 0.XX for percentage.
    // So when Reading: 0.50 -> 50(int32).
    auto parse_pct = [&](uint16_t addr) -> int32_t {
        float f = parse_float_abcd(get_ptr(addr));
        return static_cast<int32_t>(f * 100.0f);
    };

    cache_circ_settings_[1].test_loop.current_change_range_percent = parse_pct(0x002E);
    cache_circ_settings_[1].ref_loop.current_change_range_percent  = parse_pct(0x0030);
    cache_circ_settings_[2].test_loop.current_change_range_percent = parse_pct(0x0032);
    cache_circ_settings_[2].ref_loop.current_change_range_percent  = parse_pct(0x0034);

    // -- Readings --
    r1.voltage_reading = parse_float_abcd(get_ptr(0x003A));
    r2.voltage_reading = parse_float_abcd(get_ptr(0x003C));
    r1.current_reading = parse_float_abcd(get_ptr(0x003E));
    r2.current_reading = parse_float_abcd(get_ptr(0x0040));

    c1.test_loop.current = parse_float_abcd(get_ptr(0x0042));
    c1.ref_loop.current  = parse_float_abcd(get_ptr(0x0044));
    c2.test_loop.current = parse_float_abcd(get_ptr(0x0046));
    c2.ref_loop.current  = parse_float_abcd(get_ptr(0x0048));

    // -- Settings Readback (Optional, but good for validation) --
    // Const Current Settings
    cache_circ_settings_[1].test_loop.start_current_a = (int32_t)parse_float_abcd(get_ptr(0x004A));
    cache_circ_settings_[1].ref_loop.start_current_a  = (int32_t)parse_float_abcd(get_ptr(0x004C));
    cache_circ_settings_[2].test_loop.start_current_a = (int32_t)parse_float_abcd(get_ptr(0x004E));
    cache_circ_settings_[2].ref_loop.start_current_a  = (int32_t)parse_float_abcd(get_ptr(0x0050));

    // Max Current Settings
    cache_circ_settings_[1].test_loop.max_current_a = (int32_t)parse_float_abcd(get_ptr(0x0052));
    cache_circ_settings_[1].ref_loop.max_current_a  = (int32_t)parse_float_abcd(get_ptr(0x0054));
    cache_circ_settings_[2].test_loop.max_current_a = (int32_t)parse_float_abcd(get_ptr(0x0056));
    cache_circ_settings_[2].ref_loop.max_current_a  = (int32_t)parse_float_abcd(get_ptr(0x0058));

    // ========================================================================
    // 4. Logging Block (Every 10th parse)
    // ========================================================================
    log_throttle_count_++;
    if (log_throttle_count_ % 10 == 0) {
        // --- START LOGGING BLOCK ---
        RCLCPP_INFO(logger_, "=== PLC Status Dump (Tick %lu) ===", log_throttle_count_);
        RCLCPP_INFO(logger_, "Modes: C1=%d(Src%d), C2=%d(Src%d)",
                    c1.plc_control_mode, c1.plc_control_source,
                    c2.plc_control_mode, c2.plc_control_source);

        RCLCPP_INFO(logger_, "Main Reg: V=%.2f A=%.2f Dir=%d Breaker=%d/%d Limits=%d/%d",
                    r1.voltage_reading, r1.current_reading, r1.voltage_direction,
                    r1.breaker_closed_switch_ack, r1.breaker_opened_switch_ack,
                    r1.upper_limit_switch_on, r1.lower_limit_switch_on);

        RCLCPP_INFO(logger_, "C1 Loop: TestA=%.2f RefA=%.2f | Brk Test=%d/%d Ref=%d/%d",
                    c1.test_loop.current, c1.ref_loop.current,
                    c1.test_loop.breaker_closed_switch_ack, c1.test_loop.breaker_opened_switch_ack,
                    c1.ref_loop.breaker_closed_switch_ack, c1.ref_loop.breaker_opened_switch_ack);

        // Uncomment to see Raw Hex if needed
        // RCLCPP_INFO(logger_, "Raw V40: 0x%02X, V41: 0x%02X, V42: 0x%02X", v40, v41, v42);
        // --- END LOGGING BLOCK ---
    }
}

void TcpHardwareDriver::parse_temp_buffer(const std::vector<uint8_t>& buffer)
{
    if (buffer.size() < 160) return;

    auto get_temp = [&](int channel_idx) -> float {
        return parse_float_abcd(buffer.data() + (channel_idx * 4));
    };

    for(int i=0; i<8; ++i) cache_circ_status_[1].test_loop.temperature_array[i] = get_temp(i);
    for(int i=0; i<8; ++i) cache_circ_status_[1].ref_loop.temperature_array[i] = get_temp(8 + i);
    for(int i=0; i<8; ++i) cache_circ_status_[2].test_loop.temperature_array[i] = get_temp(16 + i);
    for(int i=0; i<8; ++i) cache_circ_status_[2].ref_loop.temperature_array[i] = get_temp(24 + i);
}

// ----------------------------------------------------------------------------
// Modbus Helpers (Same as before)
// ----------------------------------------------------------------------------
bool TcpHardwareDriver::modbus_read_holding_registers(SimpleTcpClient* client, uint8_t unit_id, uint16_t start_addr, uint16_t count, std::vector<uint8_t>& out_data)
{
    std::vector<uint8_t> req(12);
    req[0] = 0; req[1] = 0;
    req[2] = 0; req[3] = 0;
    req[4] = 0; req[5] = 6;
    req[6] = unit_id;
    req[7] = 0x03;
    req[8] = (start_addr >> 8) & 0xFF; req[9] = start_addr & 0xFF;
    req[10] = (count >> 8) & 0xFF; req[11] = count & 0xFF;

    if (!client->write_bytes(req)) return false;

    std::vector<uint8_t> header;
    if (!client->read_bytes(header, 9)) return false;

    if (header[7] != 0x03) return false;
    uint8_t byte_count = header[8];
    if (byte_count != count * 2) return false;

    return client->read_bytes(out_data, byte_count);
}

bool TcpHardwareDriver::modbus_write_single_register(SimpleTcpClient* client, uint8_t unit_id, uint16_t addr, uint16_t value)
{
    std::vector<uint8_t> req(12);
    req[0] = 0; req[1] = 0;
    req[2] = 0; req[3] = 0;
    req[4] = 0; req[5] = 6;
    req[6] = unit_id;
    req[7] = 0x06;
    req[8] = (addr >> 8) & 0xFF; req[9] = addr & 0xFF;
    req[10] = (value >> 8) & 0xFF; req[11] = value & 0xFF;

    if (!client->write_bytes(req)) return false;
    std::vector<uint8_t> resp;
    return client->read_bytes(resp, 12);
}

float TcpHardwareDriver::parse_float_abcd(const uint8_t* ptr)
{
    if (!ptr) return 0.0f;
    uint32_t tmp = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
    float res;
    std::memcpy(&res, &tmp, 4);
    return res;
}

uint16_t TcpHardwareDriver::parse_uint16(const uint8_t* ptr)
{
    if (!ptr) return 0;
    return (ptr[0] << 8) | ptr[1];
}

// ----------------------------------------------------------------------------
// Command Handlers (Updated Addresses)
// ----------------------------------------------------------------------------

void TcpHardwareDriver::handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (msg->command == 3) {
        active_voltage_cmd_[msg->regulator_id] = 0; // Stop
    } else {
        active_voltage_cmd_[msg->regulator_id] = msg->command; // 1=Up, 2=Down
    }
}

void TcpHardwareDriver::handle_regulator_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, AsyncCallback callback)
{
    uint16_t addr = 0xFFFF;
    if (request->regulator_id == 1) {
        addr = (request->command == 1) ? ADDR_CMD_MAIN_CLOSE : ADDR_CMD_MAIN_OPEN;
    } else if (request->regulator_id == 2) {
        addr = (request->command == 1) ? ADDR_CMD_AUX_CLOSE : ADDR_CMD_AUX_OPEN;
    }

    if (addr == 0xFFFF) {
        callback(false, "Invalid ID or Command");
        return;
    }

    std::thread([this, addr, callback]() {
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, 256);
        callback(ok, ok ? "Breaker CMD Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_circuit_breaker_command(
    const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, AsyncCallback callback)
{
    uint16_t addr = 0xFFFF;
    if (request->circuit_id == 1) {
        switch(request->command) {
        case 1: addr = ADDR_CMD_MAIN_TEST_CLOSE; break;
        case 2: addr = ADDR_CMD_MAIN_TEST_OPEN; break;
        case 3: addr = ADDR_CMD_MAIN_REF_CLOSE; break;
        case 4: addr = ADDR_CMD_MAIN_REF_OPEN; break;
        }
    } else if (request->circuit_id == 2) {
        switch(request->command) {
        case 1: addr = ADDR_CMD_AUX_TEST_CLOSE; break;
        case 2: addr = ADDR_CMD_AUX_TEST_OPEN; break;
        case 3: addr = ADDR_CMD_AUX_REF_CLOSE; break;
        case 4: addr = ADDR_CMD_AUX_REF_OPEN; break;
        }
    }

    if (addr == 0xFFFF) {
        callback(false, "Invalid ID or Command");
        return;
    }

    std::thread([this, addr, callback]() {
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, 256);
        callback(ok, ok ? "Loop Breaker CMD Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_set_control_mode(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request, AsyncCallback callback)
{
    // Write Mode: 1=Manual, 2=Auto
    uint16_t addr = (request->circuit_id == 1) ? ADDR_MODE_C1 : ADDR_MODE_C2;
    uint16_t val = (uint16_t)request->mode;

    std::thread([this, addr, val, callback]() {
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, val);
        callback(ok, ok ? "Control Mode Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_set_control_source(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlSource::Request> request, AsyncCallback callback)
{
    // Write Source: 1=Test, 2=Ref
    uint16_t addr = (request->circuit_id == 1) ? ADDR_SOURCE_C1 : ADDR_SOURCE_C2;
    uint16_t val = (uint16_t)request->source;

    std::thread([this, addr, val, callback]() {
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, val);
        callback(ok, ok ? "Control Source Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_set_hardware_regulator_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, AsyncCallback callback)
{
    uint16_t addr;
    if (request->settings.regulator_id == 1) addr = 0x0028;
    else if (request->settings.regulator_id == 2) addr = 0x0029;
    else { callback(false, "Invalid ID"); return; }

    int32_t speed_pct = std::clamp(request->settings.voltage_up_speed_percent, 0, 100);
    uint16_t plc_val = static_cast<uint16_t>(speed_pct * 5); // 0-100% -> 0-500

    std::thread([this, addr, plc_val, request, callback]() {
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, plc_val);
        if(ok) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            cache_reg_settings_[request->settings.regulator_id] = request->settings;
            callback(true, "Regulator speed updated");
        } else {
            callback(false, "Modbus Write Failed");
        }
    }).detach();
}

void TcpHardwareDriver::handle_set_hardware_circuit_settings_request(
    const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, AsyncCallback callback)
{
    uint8_t id = request->settings.circuit_id;
    if (id != 1 && id != 2) { callback(false, "Invalid Circuit ID"); return; }

    std::thread([this, id, request, callback]() {
        bool all_ok = true;

        // 1. CT Ratio (UINT16)
        uint16_t addr_ct_test = (id == 1) ? 0x002A : 0x002C;
        uint16_t addr_ct_ref  = (id == 1) ? 0x002B : 0x002D;
        if (!modbus_write_single_register(client_plc_.get(), 1, addr_ct_test, (uint16_t)request->settings.test_loop.ct_ratio)) all_ok = false;
        if (!modbus_write_single_register(client_plc_.get(), 1, addr_ct_ref,  (uint16_t)request->settings.ref_loop.ct_ratio)) all_ok = false;

        // 2. Change Range (FLOAT) - Addresses Updated to 4 distinct values
        // ID=1 -> Test=0x2E, Ref=0x30
        // ID=2 -> Test=0x32, Ref=0x34
        uint16_t addr_range_test = (id == 1) ? 0x002E : 0x0032;
        uint16_t addr_range_ref  = (id == 1) ? 0x0030 : 0x0034;

        float range_test_f = (float)request->settings.test_loop.current_change_range_percent / 100.0f;
        float range_ref_f  = (float)request->settings.ref_loop.current_change_range_percent / 100.0f;
        if (!modbus_write_float(client_plc_.get(), 1, addr_range_test, range_test_f)) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_range_ref,  range_ref_f))  all_ok = false;

        // 3. Const Current Setting (FLOAT)
        // ID=1 -> Test=0x4A, Ref=0x4C
        // ID=2 -> Test=0x4E, Ref=0x50
        uint16_t addr_const_test = (id == 1) ? 0x004A : 0x004E;
        uint16_t addr_const_ref  = (id == 1) ? 0x004C : 0x0050;
        if (!modbus_write_float(client_plc_.get(), 1, addr_const_test, (float)request->settings.test_loop.start_current_a)) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_const_ref,  (float)request->settings.ref_loop.start_current_a)) all_ok = false;

        // 4. Max Current (FLOAT)
        // ID=1 -> Test=0x52, Ref=0x54
        // ID=2 -> Test=0x56, Ref=0x58
        uint16_t addr_max_test = (id == 1) ? 0x0052 : 0x0056;
        uint16_t addr_max_ref  = (id == 1) ? 0x0054 : 0x0058;
        if (!modbus_write_float(client_plc_.get(), 1, addr_max_test, (float)request->settings.test_loop.max_current_a)) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_max_ref,  (float)request->settings.ref_loop.max_current_a)) all_ok = false;

        if (all_ok) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            cache_circ_settings_[id] = request->settings;
            callback(true, "Circuit settings updated");
        } else {
            callback(false, "Partial PLC Write Failed");
        }
    }).detach();
}

void TcpHardwareDriver::handle_clear_alarm() {}

// --- Getters ---
bool TcpHardwareDriver::get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_reg_status_.count(regulator_id)) {
        status = cache_reg_status_[regulator_id];
        return true;
    }
    return false;
}
bool TcpHardwareDriver::get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_circ_status_.count(circuit_id)) {
        status = cache_circ_status_[circuit_id];
        return true;
    }
    return false;
}
bool TcpHardwareDriver::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_reg_settings_.count(regulator_id)) {
        settings = cache_reg_settings_[regulator_id];
        return true;
    }
    return false;
}
bool TcpHardwareDriver::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_circ_settings_.count(circuit_id)) {
        settings = cache_circ_settings_[circuit_id];
        return true;
    }
    return false;
}
