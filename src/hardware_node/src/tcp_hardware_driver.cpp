#include "hardware_node/tcp_hardware_driver.hpp"

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
#include <algorithm>
#include <map>
#include <chrono>

// ============================================================================
// Internal Helper Class: SimpleTcpClient
// ============================================================================
class SimpleTcpClient {
public:
    // 【修改】接受自定义超时参数
    SimpleTcpClient(rclcpp::Logger logger, std::string ip, int port, int conn_timeout, int recv_timeout)
        : logger_(logger), ip_(ip), port_(port), sock_(INVALID_SOCKET),
        connect_timeout_ms_(conn_timeout), recv_timeout_ms_(recv_timeout) {
        last_connect_attempt_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    }

    ~SimpleTcpClient() { disconnect(); }

    std::mutex& get_tx_mutex() { return tx_mutex_; }

    bool ensure_connected() {
        if (sock_ != INVALID_SOCKET) return true;

        auto now = std::chrono::steady_clock::now();
        if ((now - last_connect_attempt_) < std::chrono::seconds(3)) {
            return false;
        }

        last_connect_attempt_ = now;
        RCLCPP_WARN(logger_, "TCP: Attempting to connect/reconnect to %s:%d ...", ip_.c_str(), port_);

        sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock_ == INVALID_SOCKET) {
            RCLCPP_ERROR(logger_, "TCP: Failed to create socket for %s:%d", ip_.c_str(), port_);
            return false;
        }

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

        int res = connect(sock_, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

        fd_set write_fds; FD_ZERO(&write_fds); FD_SET(sock_, &write_fds);

        // 【修改】使用 yaml 配置的 Connect Timeout (代替 500ms 写死)
        struct timeval tv = {connect_timeout_ms_ / 1000, (connect_timeout_ms_ % 1000) * 1000};
        int sel_res = select((int)sock_ + 1, NULL, &write_fds, NULL, &tv);

        if (sel_res > 0) {
            int so_error = 0;
            socklen_t len = sizeof(so_error);
#ifdef _WIN32
            getsockopt(sock_, SOL_SOCKET, SO_ERROR, (char*)&so_error, &len);
#else
            getsockopt(sock_, SOL_SOCKET, SO_ERROR, &so_error, &len);
#endif
            if (so_error == 0) {
#ifdef _WIN32
                mode = 0; ioctlsocket(sock_, FIONBIO, &mode);
                // 【修改】使用 yaml 配置的 Recv Timeout (代替 200ms)
                DWORD timeout = recv_timeout_ms_;
                setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
                flags = fcntl(sock_, F_GETFL, 0); fcntl(sock_, F_SETFL, flags & ~O_NONBLOCK);
                // 【修改】使用 yaml 配置的 Recv Timeout
                struct timeval r_tv = {recv_timeout_ms_ / 1000, (recv_timeout_ms_ % 1000) * 1000};
                setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &r_tv, sizeof(r_tv));
#endif
                RCLCPP_INFO(logger_, "TCP: Successfully connected to %s:%d", ip_.c_str(), port_);
                return true;
            }
        }

        RCLCPP_ERROR(logger_, "TCP: Connection failed to %s:%d. Retrying in 3s...", ip_.c_str(), port_);
        close_socket_internal();
        return false;
    }

    void disconnect() {
        if (sock_ != INVALID_SOCKET) {
            RCLCPP_WARN(logger_, "TCP: Disconnecting from %s:%d (to flush OS buffer)", ip_.c_str(), port_);
            close_socket_internal();
        }
    }

    bool write_bytes(const std::vector<uint8_t>& data) {
        if (!ensure_connected()) return false;

        if (send(sock_, (const char*)data.data(), (int)data.size(), 0) == SOCKET_ERROR) {
            RCLCPP_ERROR(logger_, "TCP: Send error to %s:%d", ip_.c_str(), port_);
            disconnect();
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
                RCLCPP_ERROR(logger_, "TCP: Recv error/closed from %s:%d", ip_.c_str(), port_);
                disconnect();
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
    int connect_timeout_ms_; // [新增]
    int recv_timeout_ms_;    // [新增]
    std::chrono::steady_clock::time_point last_connect_attempt_;
    std::mutex tx_mutex_;
};

// ============================================================================
// Modbus Write Float Helper
// ============================================================================
bool modbus_write_float(SimpleTcpClient* client, uint8_t unit_id, uint16_t addr, float value) {
    std::lock_guard<std::mutex> lock(client->get_tx_mutex());

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
    req[7] = 0x10;
    req[8] = (addr >> 8) & 0xFF; req[9] = addr & 0xFF;
    req[10] = 0; req[11] = 2;
    req[12] = 4;
    req[13] = b0; req[14] = b1;
    req[15] = b2; req[16] = b3;

    if (!client->write_bytes(req)) return false;
    std::vector<uint8_t> resp;
    if (!client->read_bytes(resp, 12)) return false;

    if (resp[7] != 0x10) {
        client->disconnect();
        return false;
    }
    return true;
}

// ============================================================================
// TcpHardwareDriver Implementation
// ============================================================================

struct PendingWriteGuard {
    std::atomic<int>& count_;
    PendingWriteGuard(std::atomic<int>& c) : count_(c) { count_++; }
    ~PendingWriteGuard() { count_--; }
};

TcpHardwareDriver::TcpHardwareDriver(rclcpp::Logger logger,
                                     std::string plc_ip, int plc_port,
                                     std::string temp_ip, int temp_port,
                                     int tcp_connect_timeout_ms,
                                     int tcp_recv_timeout_ms,
                                     int regulator_cmd_timeout_ms)
    : logger_(logger), regulator_cmd_timeout_ms_(regulator_cmd_timeout_ms)
{
#ifdef _WIN32
    WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    initialize_default_states();

    client_plc_ = std::make_unique<SimpleTcpClient>(logger_, plc_ip, plc_port, tcp_connect_timeout_ms, tcp_recv_timeout_ms);
    client_temp_ = std::make_unique<SimpleTcpClient>(logger_, temp_ip, temp_port, tcp_connect_timeout_ms, tcp_recv_timeout_ms);

    active_voltage_cmd_[1] = 0;
    active_voltage_cmd_[2] = 0;

    last_voltage_cmd_time_[1] = std::chrono::steady_clock::now();
    last_voltage_cmd_time_[2] = std::chrono::steady_clock::now();

    keep_alive_running_ = true;
    keep_alive_thread_ = std::thread(&TcpHardwareDriver::voltage_keep_alive_loop, this);

    RCLCPP_INFO(logger_, "TcpHardwareDriver Started. PLC: %s:%d (15ms loop max)", plc_ip.c_str(), plc_port);
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
        cache_reg_settings_[id].regulator_id = id;
        cache_circ_settings_[id].circuit_id = id;
    }
    cache_system_status_.is_remote = false;
    cache_system_status_.emergency_stop_on = false;
    cache_system_status_.plc_connected = false;
    cache_system_status_.temp_monitor_connected = false;
}

void TcpHardwareDriver::voltage_keep_alive_loop()
{
    while (keep_alive_running_) {
        // 【核心修复】：在每次循环起始处获取基准时间，确保周期绝对锁定为 50ms
        auto next_wake = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);

        // 如果有高优先级的指令排队（如合闸、下发参数），仅挂起 10ms 快速让渡
        if (pending_writes_.load() > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::map<uint8_t, uint8_t> cmds;
        {
            std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
            auto now = std::chrono::steady_clock::now();

            // 超时停止安全校验
            for (auto& kv : active_voltage_cmd_) {
                uint8_t id = kv.first;
                if (kv.second != 0) {
                    auto last_time = last_voltage_cmd_time_[id];
                    // 使用 yaml 配置里的保护断开超时时间 (当前为 150ms)
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() > regulator_cmd_timeout_ms_) {
                        RCLCPP_WARN(logger_, "Regulator %u op cmd timeout (>%dms). Auto-stopping.", id, regulator_cmd_timeout_ms_);
                        kv.second = 0;
                    }
                }
            }
            cmds = active_voltage_cmd_;
        }

        bool sent_command = false;
        for (auto const&[id, cmd] : cmds) {
            uint16_t addr = 0xFFFF;
            if (cmd == 1) addr = (id == 1) ? ADDR_CMD_REG1_UP : ADDR_CMD_REG2_UP;
            else if (cmd == 2) addr = (id == 1) ? ADDR_CMD_REG1_DOWN : ADDR_CMD_REG2_DOWN;

            if (addr != 0xFFFF) {
                // 执行单次升/降压 Modbus 写入
                modbus_write_single_register(client_plc_.get(), 1, addr, 256);
                sent_command = true;
            }
        }

        if (sent_command) {
            // 【核心修复】：废弃松散的 sleep_for，使用绝对时间 sleep_until。
            // 不管上面的 Modbus 写入耗费了 2ms 还是 5ms，都能严格贴紧 50ms 周期执行下一次发包。
            std::this_thread::sleep_until(next_wake);
        } else {
            // 如果当前没有按键按下，20ms 快速轮询等待 Control Node 随时到来的指令
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

void TcpHardwareDriver::update()
{
    update_tick_count_++;
    if (pending_writes_.load() <= 0) {
        read_plc_data();
    }
    if (update_tick_count_ % 5 == 0) {
        read_temp_monitor_data();
    }
}

void TcpHardwareDriver::read_plc_data()
{
    std::vector<uint8_t> data;
    if (modbus_read_holding_registers(client_plc_.get(), 1, ADDR_DATA_START, ADDR_DATA_LEN, data)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cache_system_status_.plc_connected = true;
        parse_plc_buffer(data);
    } else {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cache_system_status_.plc_connected = false;
    }
}

void TcpHardwareDriver::read_temp_monitor_data()
{
    std::vector<uint8_t> data;
    // 【修改】读取48个通道，每个通道占2个Modbus寄存器(单精度浮点数)，共需读取 48 * 2 = 96 个寄存器
    if (modbus_read_holding_registers(client_temp_.get(), 1, 0, 96, data)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cache_system_status_.temp_monitor_connected = true;
        parse_temp_buffer(data);
    } else {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cache_system_status_.temp_monitor_connected = false;
    }
}

void TcpHardwareDriver::parse_plc_buffer(const std::vector<uint8_t>& buffer)
{
    if (buffer.size() < 156) return;

    auto get_ptr = [&](uint16_t addr) -> const uint8_t* {
        int offset_reg = addr - ADDR_DATA_START;
        if (offset_reg < 0) return nullptr;
        return buffer.data() + (offset_reg * 2);
    };

    auto& r1 = cache_reg_status_[1];
    auto& c1 = cache_circ_status_[1];
    auto& r2 = cache_reg_status_[2];
    auto& c2 = cache_circ_status_[2];

    uint16_t c1_test_mode = parse_uint16(get_ptr(0x0010));
    c1.test_loop.plc_control_mode = c1_test_mode;
    uint16_t c1_sim_mode = parse_uint16(get_ptr(0x0011));
    c1.ref_loop.plc_control_mode = c1_sim_mode;
    uint16_t c2_test_mode = parse_uint16(get_ptr(0x0012));
    c2.test_loop.plc_control_mode = c2_test_mode;
    uint16_t c2_sim_mode = parse_uint16(get_ptr(0x0013));
    c2.ref_loop.plc_control_mode = c2_sim_mode;

    auto correct_plc_value = [this](uint16_t current_val, uint16_t addr, const char* name) {
        if (current_val != 1 && current_val != 2) {
            static std::map<uint16_t, std::chrono::steady_clock::time_point> last_correction;
            auto now = std::chrono::steady_clock::now();
            if (now - last_correction[addr] > std::chrono::seconds(2)) {
                last_correction[addr] = now;
                RCLCPP_WARN(logger_, "PLC Warn: Auto-correcting %s (Addr 0x%04X) to 1.", name, addr);
                std::thread([this, addr]() {
                    modbus_write_single_register(client_plc_.get(), 1, addr, 1);
                }).detach();
            }
        }
    };
    correct_plc_value(c1_test_mode, ADDR_MODE_C1_TEST, "C1 Test Mode");
    correct_plc_value(c1_sim_mode, ADDR_MODE_C1_SIM, "C1 Sim Mode");
    correct_plc_value(c2_test_mode, ADDR_MODE_C2_TEST, "C2 Test Mode");
    correct_plc_value(c2_sim_mode, ADDR_MODE_C2_SIM, "C2 Sim Mode");

    uint16_t w_v40 = parse_uint16(get_ptr(0x0014));
    uint8_t v40 = (w_v40 >> 8) & 0xFF;
    uint8_t v41 = w_v40 & 0xFF;

    uint16_t w_v42 = parse_uint16(get_ptr(0x0015));
    uint8_t v42 = (w_v42 >> 8) & 0xFF;
    uint8_t v43 = w_v42 & 0xFF;

    cache_system_status_.is_remote = (v40 >> 1) & 0x01;
    cache_system_status_.emergency_stop_on = (v40 >> 2) & 0x01;

    r1.breaker_closed_switch_ack = (v40 >> 3) & 0x01;
    r1.breaker_opened_switch_ack = (v40 >> 4) & 0x01;
    r2.breaker_closed_switch_ack = (v40 >> 5) & 0x01;
    r2.breaker_opened_switch_ack = (v40 >> 6) & 0x01;

    c1.test_loop.breaker_closed_switch_ack = (v40 >> 7) & 0x01;
    c1.test_loop.breaker_opened_switch_ack = (v41 >> 0) & 0x01;
    c2.test_loop.breaker_closed_switch_ack = (v41 >> 1) & 0x01;
    c2.test_loop.breaker_opened_switch_ack = (v41 >> 2) & 0x01;

    c1.ref_loop.breaker_closed_switch_ack = (v41 >> 3) & 0x01;
    c1.ref_loop.breaker_opened_switch_ack = (v41 >> 4) & 0x01;
    c2.ref_loop.breaker_closed_switch_ack = (v41 >> 5) & 0x01;
    c2.ref_loop.breaker_opened_switch_ack = (v41 >> 6) & 0x01;

    r1.upper_limit_switch_on = (v41 >> 7) & 0x01;
    r1.lower_limit_switch_on = (v42 >> 0) & 0x01;
    r2.upper_limit_switch_on = (v42 >> 1) & 0x01;
    r2.lower_limit_switch_on = (v42 >> 2) & 0x01;
    r1.auto_reduce_opening = (v42 >> 7) & 0x01;
    r2.auto_reduce_opening = (v43 >> 0) & 0x01;

    bool m_up = (v42 >> 3) & 0x01;
    bool m_down = (v42 >> 4) & 0x01;
    if (m_up && !m_down) r1.voltage_direction = 1;
    else if (!m_up && m_down) r1.voltage_direction = -1;
    else if (!m_up && !m_down) r1.voltage_direction = 0;
    else r1.voltage_direction = -2;

    bool a_up = (v42 >> 5) & 0x01;
    bool a_down = (v42 >> 6) & 0x01;
    if (a_up && !a_down) r2.voltage_direction = 1;
    else if (!a_up && a_down) r2.voltage_direction = -1;
    else if (!a_up && !a_down) r2.voltage_direction = 0;
    else r2.voltage_direction = -2;

    uint16_t alarm1 = parse_uint16(get_ptr(0x0016));
    uint16_t alarm2 = parse_uint16(get_ptr(0x0017));
    r1.over_current_on = (alarm1 == 256);
    r2.over_current_on = (alarm2 == 256);

    uint16_t speed1_val = parse_uint16(get_ptr(0x0028));
    uint16_t speed2_val = parse_uint16(get_ptr(0x0029));

    cache_reg_settings_[1].voltage_up_speed_percent = speed1_val / 5;
    cache_reg_settings_[1].voltage_down_speed_percent = speed1_val / 5;
    cache_reg_settings_[2].voltage_up_speed_percent = speed2_val / 5;
    cache_reg_settings_[2].voltage_down_speed_percent = speed2_val / 5;

    cache_circ_settings_[1].test_loop.ct_ratio = parse_uint16(get_ptr(0x002A));
    cache_circ_settings_[2].test_loop.ct_ratio = parse_uint16(get_ptr(0x002B));
    cache_circ_settings_[1].ref_loop.ct_ratio  = parse_uint16(get_ptr(0x002C));
    cache_circ_settings_[2].ref_loop.ct_ratio  = parse_uint16(get_ptr(0x002D));

    auto parse_float_to_int32 = [&](uint16_t addr) -> int32_t {
        float f = parse_float_abcd(get_ptr(addr));
        return static_cast<int32_t>(f);
    };

    cache_circ_settings_[1].test_loop.current_change_range_percent = parse_float_to_int32(0x002E);
    cache_circ_settings_[2].test_loop.current_change_range_percent = parse_float_to_int32(0x0030);
    cache_circ_settings_[1].ref_loop.current_change_range_percent  = parse_float_to_int32(0x0032);
    cache_circ_settings_[2].ref_loop.current_change_range_percent  = parse_float_to_int32(0x0034);

    r1.voltage_reading = parse_float_abcd(get_ptr(0x003A));
    r2.voltage_reading = parse_float_abcd(get_ptr(0x003C));
    r1.current_reading = parse_float_abcd(get_ptr(0x003E));
    r2.current_reading = parse_float_abcd(get_ptr(0x0040));

    c1.test_loop.current = parse_float_abcd(get_ptr(0x0042));
    c2.test_loop.current = parse_float_abcd(get_ptr(0x0044));
    c1.ref_loop.current  = parse_float_abcd(get_ptr(0x0046));
    c2.ref_loop.current  = parse_float_abcd(get_ptr(0x0048));

    cache_circ_settings_[1].test_loop.start_current_a = (int32_t)parse_float_abcd(get_ptr(0x004A));
    cache_circ_settings_[2].test_loop.start_current_a = (int32_t)parse_float_abcd(get_ptr(0x004C));
    cache_circ_settings_[1].ref_loop.start_current_a  = (int32_t)parse_float_abcd(get_ptr(0x004E));
    cache_circ_settings_[2].ref_loop.start_current_a  = (int32_t)parse_float_abcd(get_ptr(0x0050));

    cache_circ_settings_[1].test_loop.max_current_a = (int32_t)parse_float_abcd(get_ptr(0x0052));
    cache_circ_settings_[2].test_loop.max_current_a = (int32_t)parse_float_abcd(get_ptr(0x0054));
    cache_circ_settings_[1].ref_loop.max_current_a  = (int32_t)parse_float_abcd(get_ptr(0x0056));
    cache_circ_settings_[2].ref_loop.max_current_a  = (int32_t)parse_float_abcd(get_ptr(0x0058));

    cache_reg_settings_[1].over_current_a = (int32_t)parse_float_abcd(get_ptr(0x005A));
    cache_reg_settings_[2].over_current_a = (int32_t)parse_float_abcd(get_ptr(0x005C));
}

void TcpHardwareDriver::parse_temp_buffer(const std::vector<uint8_t>& buffer)
{
    // 【修改】48个通道 * 4字节(单精度浮点数) = 192字节。数据长度不满足则丢弃。
    if (buffer.size() < 192) return;

    // 【修改】获取温度的 Lambda 表达式，增加“超过1000显示为0”的过滤逻辑
    auto get_temp = [&](int channel_idx) -> float {
        float temp = parse_float_abcd(buffer.data() + (channel_idx * 4));
        // TP1100 断线时通常显示 1999.9，这里如果超过1000直接视为无效数据并归零
        if (temp > 1000.0f || temp < -200.0f) {
            return 0.0f;
        }
        return temp;
    };

    // 1. 前 16 个通道 (索引 0 ~ 15) 赋值给 回路1 的 试验支路
    for(int i = 0; i < 16; ++i) {
        cache_circ_status_[1].test_loop.temperature_array[i] = get_temp(i);
    }

    // 2. 接下来 8 个通道 (索引 16 ~ 23) 赋值给 回路1 的 参考支路
    for(int i = 0; i < 8; ++i) {
        cache_circ_status_[1].ref_loop.temperature_array[i] = get_temp(16 + i);
    }

    // 3. 接下来 16 个通道 (索引 24 ~ 39) 赋值给 回路2 的 试验支路
    for(int i = 0; i < 16; ++i) {
        cache_circ_status_[2].test_loop.temperature_array[i] = get_temp(24 + i);
    }

    // 4. 接下来 8 个通道 (索引 40 ~ 47) 赋值给 回路2 的 参考支路
    for(int i = 0; i < 8; ++i) {
        cache_circ_status_[2].ref_loop.temperature_array[i] = get_temp(40 + i);
    }
}

bool TcpHardwareDriver::modbus_read_holding_registers(SimpleTcpClient* client, uint8_t unit_id, uint16_t start_addr, uint16_t count, std::vector<uint8_t>& out_data)
{
    std::lock_guard<std::mutex> lock(client->get_tx_mutex());
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

    if (header[7] != 0x03) {
        client->disconnect();
        return false;
    }
    uint8_t byte_count = header[8];
    if (byte_count != count * 2) {
        client->disconnect();
        return false;
    }
    return client->read_bytes(out_data, byte_count);
}

bool TcpHardwareDriver::modbus_write_single_register(SimpleTcpClient* client, uint8_t unit_id, uint16_t addr, uint16_t value)
{
    std::lock_guard<std::mutex> lock(client->get_tx_mutex());
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
    if (!client->read_bytes(resp, 12)) return false;

    if (resp[7] != 0x06) {
        client->disconnect();
        return false;
    }
    return true;
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

void TcpHardwareDriver::handle_regulator_operation_command(const ros2_interfaces::msg::RegulatorOperationCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (msg->command == 3) {
        active_voltage_cmd_[msg->regulator_id] = 0;
    } else {
        active_voltage_cmd_[msg->regulator_id] = msg->command;
    }
    last_voltage_cmd_time_[msg->regulator_id] = std::chrono::steady_clock::now();
}

void TcpHardwareDriver::handle_regulator_breaker_command(const std::shared_ptr<ros2_interfaces::srv::RegulatorBreakerCommand::Request> request, AsyncCallback callback)
{
    uint16_t addr = 0xFFFF;
    if (request->regulator_id == 1) {
        if (request->command == 1) addr = ADDR_CMD_REG1_CLOSE;
        else if (request->command == 2) addr = ADDR_CMD_REG1_OPEN;
        else if (request->command == 3) addr = ADDR_CMD_REG1_AUTO_REDUCE_OPEN;
    }
    else if (request->regulator_id == 2) {
        if (request->command == 1) addr = ADDR_CMD_REG2_CLOSE;
        else if (request->command == 2) addr = ADDR_CMD_REG2_OPEN;
        else if (request->command == 3) addr = ADDR_CMD_REG2_AUTO_REDUCE_OPEN;
    }
    if (addr == 0xFFFF) { callback(false, "Invalid ID or Command"); return; }
    std::thread([this, addr, callback]() {
        PendingWriteGuard guard(pending_writes_);
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, 256);
        callback(ok, ok ? "Breaker CMD Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_circuit_breaker_command(const std::shared_ptr<ros2_interfaces::srv::CircuitBreakerCommand::Request> request, AsyncCallback callback)
{
    uint16_t addr = 0xFFFF;
    if (request->circuit_id == 1) {
        switch(request->command) {
        case 1: addr = ADDR_CMD_C1_TEST_CLOSE; break;
        case 2: addr = ADDR_CMD_C1_TEST_OPEN; break;
        case 3: addr = ADDR_CMD_C1_SIM_CLOSE; break;
        case 4: addr = ADDR_CMD_C1_SIM_OPEN; break;
        }
    } else if (request->circuit_id == 2) {
        switch(request->command) {
        case 1: addr = ADDR_CMD_C2_TEST_CLOSE; break;
        case 2: addr = ADDR_CMD_C2_TEST_OPEN; break;
        case 3: addr = ADDR_CMD_C2_SIM_CLOSE; break;
        case 4: addr = ADDR_CMD_C2_SIM_OPEN; break;
        }
    }
    if (addr == 0xFFFF) { callback(false, "Invalid ID or Command"); return; }
    std::thread([this, addr, callback]() {
        PendingWriteGuard guard(pending_writes_);
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, 256);
        callback(ok, ok ? "Loop Breaker CMD Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_set_control_mode(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitControlMode::Request> request, AsyncCallback callback)
{
    uint16_t addr = 0xFFFF;
    if (request->circuit_id == 1) addr = (request->loop_type == 1) ? ADDR_MODE_C1_TEST : ADDR_MODE_C1_SIM;
    else if (request->circuit_id == 2) addr = (request->loop_type == 1) ? ADDR_MODE_C2_TEST : ADDR_MODE_C2_SIM;
    uint16_t val = (request->mode == 2) ? 2 : 1;

    std::thread([this, addr, val, circ_id = request->circuit_id, callback]() {
        PendingWriteGuard guard(pending_writes_);
        bool ok = modbus_write_single_register(client_plc_.get(), 1, addr, val);
        callback(ok, ok ? "Control Mode Sent" : "Modbus Write Failed");
    }).detach();
}

void TcpHardwareDriver::handle_set_hardware_regulator_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetRegulatorSettings::Request> request, AsyncCallback callback)
{
    uint8_t id = request->settings.regulator_id;
    if (id != 1 && id != 2) { callback(false, "Invalid ID"); return; }
    uint16_t addr_speed = (id == 1) ? 0x0028 : 0x0029;
    uint16_t addr_ocp   = (id == 1) ? 0x005A : 0x005C;
    int32_t speed_pct = std::clamp(request->settings.voltage_up_speed_percent, 0, 100);
    uint16_t plc_val = static_cast<uint16_t>(speed_pct * 5);
    float ocp_val = static_cast<float>(request->settings.over_current_a);

    std::thread([this, id, addr_speed, addr_ocp, plc_val, ocp_val, request, callback]() {
        PendingWriteGuard guard(pending_writes_);
        bool all_ok = true;
        if (!modbus_write_single_register(client_plc_.get(), 1, addr_speed, plc_val)) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_ocp, ocp_val)) all_ok = false;
        if(all_ok) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            cache_reg_settings_[id] = request->settings;
            callback(true, "Regulator settings updated");
        } else {
            callback(false, "Partial PLC Write Failed");
        }
    }).detach();
}

void TcpHardwareDriver::handle_set_hardware_circuit_settings_request(const std::shared_ptr<ros2_interfaces::srv::SetHardwareCircuitSettings::Request> request, AsyncCallback callback)
{
    uint8_t id = request->settings.circuit_id;
    if (id != 1 && id != 2) { callback(false, "Invalid Circuit ID"); return; }
    std::thread([this, id, request, callback]() {
        PendingWriteGuard guard(pending_writes_);
        bool all_ok = true;
        uint16_t addr_ct_test = (id == 1) ? 0x002A : 0x002B;
        uint16_t addr_ct_ref  = (id == 1) ? 0x002C : 0x002D;
        if (!modbus_write_single_register(client_plc_.get(), 1, addr_ct_test, (uint16_t)request->settings.test_loop.ct_ratio)) all_ok = false;
        if (!modbus_write_single_register(client_plc_.get(), 1, addr_ct_ref,  (uint16_t)request->settings.ref_loop.ct_ratio)) all_ok = false;
        uint16_t addr_range_test = (id == 1) ? 0x002E : 0x0030;
        uint16_t addr_range_ref  = (id == 1) ? 0x0032 : 0x0034;
        if (!modbus_write_float(client_plc_.get(), 1, addr_range_test, static_cast<float>(request->settings.test_loop.current_change_range_percent))) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_range_ref,  static_cast<float>(request->settings.ref_loop.current_change_range_percent)))  all_ok = false;
        uint16_t addr_const_test = (id == 1) ? 0x004A : 0x004C;
        uint16_t addr_const_ref  = (id == 1) ? 0x004E : 0x0050;
        if (!modbus_write_float(client_plc_.get(), 1, addr_const_test, static_cast<float>(request->settings.test_loop.start_current_a))) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_const_ref,  static_cast<float>(request->settings.ref_loop.start_current_a))) all_ok = false;
        uint16_t addr_max_test = (id == 1) ? 0x0052 : 0x0054;
        uint16_t addr_max_ref  = (id == 1) ? 0x0056 : 0x0058;
        if (!modbus_write_float(client_plc_.get(), 1, addr_max_test, static_cast<float>(request->settings.test_loop.max_current_a))) all_ok = false;
        if (!modbus_write_float(client_plc_.get(), 1, addr_max_ref,  static_cast<float>(request->settings.ref_loop.max_current_a))) all_ok = false;
        if (all_ok) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            cache_circ_settings_[id] = request->settings;
            callback(true, "Circuit settings updated");
        } else {
            callback(false, "Partial PLC Write Failed");
        }
    }).detach();
}

void TcpHardwareDriver::handle_clear_alarm() {
    std::thread([this]() {
        PendingWriteGuard guard(pending_writes_);
        bool ok1 = modbus_write_single_register(client_plc_.get(), 1, 0x0016, 0);
        bool ok2 = modbus_write_single_register(client_plc_.get(), 1, 0x0017, 0);
    }).detach();
}

bool TcpHardwareDriver::get_regulator_status(uint8_t regulator_id, ros2_interfaces::msg::RegulatorStatus& status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_reg_status_.count(regulator_id)) {
        status = cache_reg_status_[regulator_id];
        status.regulator_id = regulator_id;
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_circuit_status(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitStatus& status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_circ_status_.count(circuit_id)) {
        status = cache_circ_status_[circuit_id];
        status.circuit_id = circuit_id;
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_regulator_settings(uint8_t regulator_id, ros2_interfaces::msg::RegulatorSettings& settings) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_reg_settings_.count(regulator_id)) {
        settings = cache_reg_settings_[regulator_id];
        settings.regulator_id = regulator_id;
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_circuit_settings(uint8_t circuit_id, ros2_interfaces::msg::HardwareCircuitSettings& settings) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cache_circ_settings_.count(circuit_id)) {
        settings = cache_circ_settings_[circuit_id];
        settings.circuit_id = circuit_id;
        return true;
    }
    return false;
}

bool TcpHardwareDriver::get_system_status(ros2_interfaces::msg::HardwareSystemStatus& status) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    status = cache_system_status_;
    return true;
}
