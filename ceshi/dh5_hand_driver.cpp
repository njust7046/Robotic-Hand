/**
 * DH-5-6 灵巧手完整驱动与动作库
 * 
 * 硬件：右手，从机ID=0x01，RS485，115200 8N1
 * 协议：Modbus-RTU (0x03读/0x06写单个/0x10写多个)
 * 
 * 轴映射（右手）：
 *   轴1: 拇指左右摆动
 *   轴2: 食指屈伸
 *   轴3: 中指屈伸
 *   轴4: 无名指屈伸
 *   轴5: 小拇指屈伸
 *   轴6: 拇指上下摆动
 */

#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

// ============================================================================
// 常量定义（严格依据手册）
// ============================================================================

// Modbus 从机地址
constexpr uint8_t DEVICE_ID = 0x01;

// 寄存器地址（手册 6.4.2 设置参数）
constexpr uint16_t REG_HOMING          = 0x0100;  // 回零控制
constexpr uint16_t REG_POS_AXIS1       = 0x0101;  // 轴1位置 (拇指左右)
constexpr uint16_t REG_POS_AXIS2       = 0x0102;  // 轴2位置 (食指)
constexpr uint16_t REG_POS_AXIS3       = 0x0103;  // 轴3位置 (中指)
constexpr uint16_t REG_POS_AXIS4       = 0x0104;  // 轴4位置 (无名指)
constexpr uint16_t REG_POS_AXIS5       = 0x0105;  // 轴5位置 (小拇指)
constexpr uint16_t REG_POS_AXIS6       = 0x0106;  // 轴6位置 (拇指上下)

constexpr uint16_t REG_FORCE_AXIS1     = 0x0107;  // 轴1力百分比 (20~100)
constexpr uint16_t REG_FORCE_AXIS2     = 0x0108;
constexpr uint16_t REG_FORCE_AXIS3     = 0x0109;
constexpr uint16_t REG_FORCE_AXIS4     = 0x010A;
constexpr uint16_t REG_FORCE_AXIS5     = 0x010B;
constexpr uint16_t REG_FORCE_AXIS6     = 0x010C;

constexpr uint16_t REG_SPEED_AXIS1     = 0x010D;  // 轴1速度百分比 (1~100)
constexpr uint16_t REG_SPEED_AXIS2     = 0x010E;
constexpr uint16_t REG_SPEED_AXIS3     = 0x010F;
constexpr uint16_t REG_SPEED_AXIS4     = 0x0110;
constexpr uint16_t REG_SPEED_AXIS5     = 0x0111;
constexpr uint16_t REG_SPEED_AXIS6     = 0x0112;

// 寄存器地址（手册 6.4.3 反馈参数）
constexpr uint16_t REG_HOMING_STATUS   = 0x0200;  // 回零状态
constexpr uint16_t REG_STATE_AXIS1     = 0x0201;  // 轴1运行状态 (0运动/1到位/2堵转)
constexpr uint16_t REG_STATE_AXIS2     = 0x0202;
constexpr uint16_t REG_STATE_AXIS3     = 0x0203;
constexpr uint16_t REG_STATE_AXIS4     = 0x0204;
constexpr uint16_t REG_STATE_AXIS5     = 0x0205;
constexpr uint16_t REG_STATE_AXIS6     = 0x0206;

constexpr uint16_t REG_CURPOS_AXIS1    = 0x0207;  // 轴1当前位置
constexpr uint16_t REG_CURPOS_AXIS2    = 0x0208;
constexpr uint16_t REG_CURPOS_AXIS3    = 0x0209;
constexpr uint16_t REG_CURPOS_AXIS4    = 0x020A;
constexpr uint16_t REG_CURPOS_AXIS5    = 0x020B;
constexpr uint16_t REG_CURPOS_AXIS6    = 0x020C;

constexpr uint16_t REG_CURRENT_AXIS1   = 0x0213;  // 轴1电流 (mA)
constexpr uint16_t REG_CURRENT_AXIS2   = 0x0214;
constexpr uint16_t REG_CURRENT_AXIS3   = 0x0215;
constexpr uint16_t REG_CURRENT_AXIS4   = 0x0216;
constexpr uint16_t REG_CURRENT_AXIS5   = 0x0217;

// 系统寄存器（手册 6.4.4）
constexpr uint16_t REG_CLEAR_ERROR     = 0x0501;  // 清除错误 (写1)
constexpr uint16_t REG_SYSTEM_RESTART  = 0x0503;  // 系统重启 (写1)

// 回零模式（手册 6.4.2）
constexpr uint16_t HOMING_CLOSE_ALL    = 0x0555;  // 6轴全部闭合回零
constexpr uint16_t HOMING_OPEN_ALL     = 0x0AAA;  // 6轴全部张开回零

// 运行状态值
constexpr uint16_t STATE_MOVING        = 0;
constexpr uint16_t STATE_ARRIVED       = 1;
constexpr uint16_t STATE_BLOCKED       = 2;

// 回零状态值（每轴2bit）
constexpr uint8_t HOMING_NOT_INIT      = 0;  // 00
constexpr uint8_t HOMING_SUCCESS       = 1;  // 01
constexpr uint8_t HOMING_IN_PROGRESS   = 2;  // 10

// ============================================================================
// CRC16 (Modbus-RTU) 低字节在前
// ============================================================================
uint16_t crc16_modbus(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;//防出错校验码
}

// ============================================================================
// Linux 串口封装 (Ubuntu 22.04)
// ============================================================================
class LinuxSerialPort {
private:
    int fd = -1;
    string portName;

    speed_t toSpeed(uint32_t baudRate) {
        switch (baudRate) {
            case 4800:   return B4800;
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            default:     return B115200;
        }
    }

public:
    bool open(const string& port, uint32_t baudRate = 115200) {
        portName = port;

        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            cerr << "❌ 无法打开串口 " << port << " (" << strerror(errno) << ")" << endl;
            return false;
        }

        termios tty {};
        if (tcgetattr(fd, &tty) != 0) {
            cerr << "❌ 获取串口状态失败 (" << strerror(errno) << ")" << endl;
            close();
            return false;
        }

        // 原始模式 + 115200 8N1 + 无流控
        cfmakeraw(&tty);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;  // 100ms

        speed_t speed = toSpeed(baudRate);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            cerr << "❌ 设置串口参数失败 (" << strerror(errno) << ")" << endl;
            close();
            return false;
        }

        tcflush(fd, TCIOFLUSH);

        cout << "✅ 串口已打开: " << port << " @" << baudRate << " bps" << endl;
        return true;
    }

    void close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    int write(const uint8_t* data, size_t length) {
        if (fd < 0) return -1;

        size_t totalWritten = 0;
        while (totalWritten < length) {
            ssize_t n = ::write(fd, data + totalWritten, length - totalWritten);
            if (n < 0) {
                if (errno == EINTR) continue;
                return -1;
            }
            totalWritten += static_cast<size_t>(n);
        }
        return static_cast<int>(totalWritten);
    }

    int read(uint8_t* buffer, size_t maxLen) {
        if (fd < 0) return -1;

        ssize_t n = ::read(fd, buffer, maxLen);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) return 0;
            return -1;
        }
        return static_cast<int>(n);
    }

    ~LinuxSerialPort() {
        close();
    }
};

// ============================================================================
// DhHand 驱动类
// ============================================================================
class DhHand {
private:
    LinuxSerialPort serial;
    uint8_t deviceID = DEVICE_ID;
    int retryCount = 2;  // 重试次数
    int readTimeoutMs = 300;
    
    // 发送并接收 Modbus 响应
    bool sendAndReceive(const vector<uint8_t>& txFrame, vector<uint8_t>& rxFrame, int expectedLen = 0) {
        for (int attempt = 0; attempt <= retryCount; attempt++) {
            if (attempt > 0) {
                cout << "  ⚠️ 重试 " << attempt << "/" << retryCount << endl;
            }

            // 发送
            int sent = serial.write(txFrame.data(), txFrame.size());
            if (sent != (int)txFrame.size()) {
                cerr << "  ❌ 发送失败" << endl;
                this_thread::sleep_for(chrono::milliseconds(50));
                continue;
            }

            // 等待并拼接响应
            vector<uint8_t> rxBuffer;
            rxBuffer.reserve(256);
            auto start = chrono::steady_clock::now();
            int targetLen = expectedLen;

            while (true) {
                auto elapsedMs = chrono::duration_cast<chrono::milliseconds>(
                    chrono::steady_clock::now() - start).count();
                if (elapsedMs > readTimeoutMs) break;

                uint8_t tmp[64] = {0};
                int n = serial.read(tmp, sizeof(tmp));
                if (n < 0) {
                    cerr << "  ❌ 串口读取失败" << endl;
                    break;
                }
                if (n == 0) {
                    this_thread::sleep_for(chrono::milliseconds(5));
                    continue;
                }

                rxBuffer.insert(rxBuffer.end(), tmp, tmp + n);

                // 未指定长度时，根据 Modbus 头推断长度
                if (targetLen == 0 && rxBuffer.size() >= 3) {
                    uint8_t fn = rxBuffer[1];
                    if (fn == 0x03) {
                        targetLen = 5 + rxBuffer[2];
                    } else if (fn == 0x06 || fn == 0x10 || (fn & 0x80)) {
                        targetLen = 8;
                    }
                }

                if (targetLen > 0 && (int)rxBuffer.size() >= targetLen) {
                    break;
                }

                // 收到数据后延长等待窗口，避免分包
                start = chrono::steady_clock::now();
            }

            if (rxBuffer.empty()) {
                cerr << "  ❌ 接收超时" << endl;
                continue;
            }

            // 检查最小长度
            if (rxBuffer.size() < 5) {
                cerr << "  ❌ 响应过短 (" << rxBuffer.size() << " bytes)" << endl;
                continue;
            }

            if (targetLen > 0 && (int)rxBuffer.size() < targetLen) {
                cerr << "  ❌ 响应长度不足 (期望 " << targetLen
                     << ", 实际 " << rxBuffer.size() << ")" << endl;
                continue;
            }

            if (targetLen > 0 && (int)rxBuffer.size() > targetLen) {
                rxBuffer.resize(targetLen);
            }

            // 校验 CRC
            uint16_t recvCrc = rxBuffer[rxBuffer.size() - 2] | (rxBuffer[rxBuffer.size() - 1] << 8);
            uint16_t calcCrc = crc16_modbus(rxBuffer.data(), rxBuffer.size() - 2);
            
            if (recvCrc != calcCrc) {
                cerr << "  ❌ CRC 校验失败 (recv=0x" << hex << recvCrc 
                     << " calc=0x" << calcCrc << dec << ")" << endl;
                continue;
            }

            // 检查异常响应（功能码最高位为1）
            if (rxBuffer[1] & 0x80) {
                cerr << "  ❌ Modbus 异常响应: 0x" << hex << (int)rxBuffer[2] << dec << endl;
                return false;
            }

            rxFrame = rxBuffer;
            return true;
        }

        cerr << "  ❌ 通信失败，已重试 " << retryCount << " 次" << endl;
        return false;
    }

    // 构建 Modbus 帧并添加 CRC
    vector<uint8_t> buildFrame(const vector<uint8_t>& data) {
        vector<uint8_t> frame = data;
        uint16_t crc = crc16_modbus(frame.data(), frame.size());
        frame.push_back(crc & 0xFF);      // CRC 低字节
        frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
        return frame;
    }

public:
    // 打开串口
    bool open(const string& port, uint32_t baudRate = 115200) {
        return serial.open(port, baudRate);
    }

    // 关闭串口
    void close() {
        serial.close();
    }

    // ========================================================================
    // Modbus 0x06: 写单个保持寄存器
    // ========================================================================
    bool writeSingleReg(uint16_t regAddr, uint16_t value) {
        vector<uint8_t> tx = {
            deviceID,
            0x06,
            (uint8_t)(regAddr >> 8),
            (uint8_t)(regAddr & 0xFF),
            (uint8_t)(value >> 8),
            (uint8_t)(value & 0xFF)
        };
        
        auto frame = buildFrame(tx);
        vector<uint8_t> rx;
        
        if (!sendAndReceive(frame, rx, 8)) {
            return false;
        }

        // 响应应该是回显（地址+功能码+寄存器+值+CRC）
        if (rx.size() != 8 || rx[1] != 0x06) {
            cerr << "  ❌ 0x06 响应格式错误" << endl;
            return false;
        }

        return true;
    }

    // ========================================================================
    // Modbus 0x10: 写多个保持寄存器
    // ========================================================================
    bool writeMultiRegs(uint16_t startAddr, const vector<uint16_t>& values) {
        if (values.empty() || values.size() > 123) {
            cerr << "❌ 寄存器数量无效: " << values.size() << endl;
            return false;
        }

        uint16_t regCount = (uint16_t)values.size();
        uint8_t byteCount = regCount * 2;

        vector<uint8_t> tx = {
            deviceID,
            0x10,
            (uint8_t)(startAddr >> 8),
            (uint8_t)(startAddr & 0xFF),
            (uint8_t)(regCount >> 8),
            (uint8_t)(regCount & 0xFF),
            byteCount
        };

        for (auto val : values) {
            tx.push_back((uint8_t)(val >> 8));
            tx.push_back((uint8_t)(val & 0xFF));
        }

        auto frame = buildFrame(tx);
        vector<uint8_t> rx;

        if (!sendAndReceive(frame, rx, 8)) {
            return false;
        }

        if (rx.size() != 8 || rx[1] != 0x10) {
            cerr << "  ❌ 0x10 响应格式错误" << endl;
            return false;
        }

        return true;
    }

    // ========================================================================
    // Modbus 0x03: 读保持寄存器
    // ========================================================================
    bool readHoldingRegs(uint16_t startAddr, uint16_t count, vector<uint16_t>& outValues) {
        if (count == 0 || count > 125) {
            cerr << "❌ 读取数量无效: " << count << endl;
            return false;
        }

        vector<uint8_t> tx = {
            deviceID,
            0x03,
            (uint8_t)(startAddr >> 8),
            (uint8_t)(startAddr & 0xFF),
            (uint8_t)(count >> 8),
            (uint8_t)(count & 0xFF)
        };

        auto frame = buildFrame(tx);
        vector<uint8_t> rx;

        if (!sendAndReceive(frame, rx)) {
            return false;
        }

        // 响应格式: [ID][0x03][字节数][数据...][CRC]
        if (rx.size() < 5 || rx[1] != 0x03) {
            cerr << "  ❌ 0x03 响应格式错误" << endl;
            return false;
        }

        uint8_t byteCount = rx[2];
        if (byteCount != count * 2 || rx.size() != (size_t)(5 + byteCount)) {
            cerr << "  ❌ 0x03 数据长度不匹配" << endl;
            return false;
        }

        outValues.clear();
        for (int i = 0; i < count; i++) {
            uint16_t val = (rx[3 + i * 2] << 8) | rx[4 + i * 2];
            outValues.push_back(val);
        }

        return true;
    }

    // ========================================================================
    // 清除错误（写 0x0501 = 1）
    // ========================================================================
    bool clearError() {
        cout << "🔧 清除错误..." << endl;
        return writeSingleReg(REG_CLEAR_ERROR, 1);
    }

    // ========================================================================
    // 回零（闭合模式）并等待完成
    // ========================================================================
    bool homeCloseAllAxes(int timeoutSec = 10) {
        cout << "🔄 开始回零（闭合模式）..." << endl;

        // 发送回零指令
        if (!writeSingleReg(REG_HOMING, HOMING_CLOSE_ALL)) {
            cerr << "❌ 回零指令发送失败" << endl;
            return false;
        }

        // 等待回零完成
        auto startTime = chrono::steady_clock::now();
        while (true) {
            auto elapsed = chrono::duration_cast<chrono::seconds>(
                chrono::steady_clock::now() - startTime).count();
            
            if (elapsed > timeoutSec) {
                cerr << "❌ 回零超时 (" << timeoutSec << "s)" << endl;
                return false;
            }

            // 读回零状态 0x0200
            vector<uint16_t> status;
            if (!readHoldingRegs(REG_HOMING_STATUS, 1, status)) {
                cerr << "  ⚠️ 读取回零状态失败，继续等待..." << endl;
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            uint16_t homingStatus = status[0];
            
            // 检查所有6个轴是否都回零成功（每轴2bit，值为01表示成功）
            bool allSuccess = true;
            for (int axis = 0; axis < 6; axis++) {
                int bits = (homingStatus >> (axis * 2)) & 0x03;
                if (bits != HOMING_SUCCESS) {
                    allSuccess = false;
                    break;
                }
            }

            if (allSuccess) {
                cout << "✅ 回零完成！" << endl;
                // 回零完成后额外等待，确保手指到位
                this_thread::sleep_for(chrono::milliseconds(1000));
                return true;
            }

            this_thread::sleep_for(chrono::milliseconds(200));
        }
    }

    // ========================================================================
    // 回零（张开模式）并等待完成
    // ========================================================================
    bool homeOpenAllAxes(int timeoutSec = 10) {
        cout << "🔄 开始回零（张开模式）..." << endl;

        // 发送回零指令
        if (!writeSingleReg(REG_HOMING, HOMING_OPEN_ALL)) {
            cerr << "❌ 回零指令发送失败" << endl;
            return false;
        }

        // 等待回零完成
        auto startTime = chrono::steady_clock::now();
        while (true) {
            auto elapsed = chrono::duration_cast<chrono::seconds>(
                chrono::steady_clock::now() - startTime).count();
            
            if (elapsed > timeoutSec) {
                cerr << "❌ 回零超时 (" << timeoutSec << "s)" << endl;
                return false;
            }

            // 读回零状态 0x0200
            vector<uint16_t> status;
            if (!readHoldingRegs(REG_HOMING_STATUS, 1, status)) {
                cerr << "  ⚠️ 读取回零状态失败，继续等待..." << endl;
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            uint16_t homingStatus = status[0];
            
            // 检查所有6个轴是否都回零成功（每轴2bit，值为01表示成功）
            bool allSuccess = true;
            for (int axis = 0; axis < 6; axis++) {
                int bits = (homingStatus >> (axis * 2)) & 0x03;
                if (bits != HOMING_SUCCESS) {
                    allSuccess = false;
                    break;
                }
            }

            if (allSuccess) {
                cout << "✅ 回零完成！" << endl;
                // 回零完成后额外等待，确保手指到位
                this_thread::sleep_for(chrono::milliseconds(1000));
                return true;
            }

            this_thread::sleep_for(chrono::milliseconds(200));
        }
    }

    // ========================================================================
    // 设置 6 轴位置（单位：0.01mm）带范围校验
    // ========================================================================
    bool setAxesPositions6(const uint16_t pos[6]) {
        // 手册未明确给出最大行程，这里假设 0~1706 (0~17.06mm) 为安全范围
        // 实际应用中需要根据标定确定
        for (int i = 0; i < 6; i++) {
            if (pos[i] > 1706) {
                cerr << "❌ 轴" << (i+1) << " 位置超限: " << pos[i] << " (最大1706)" << endl;
                return false;
            }
        }

        vector<uint16_t> values(pos, pos + 6);
        return writeMultiRegs(REG_POS_AXIS1, values);
    }

    // ========================================================================
    // 设置 6 轴速度（百分比 1~100）
    // ========================================================================
    bool setAxesSpeeds6(const uint16_t spd[6]) {
        for (int i = 0; i < 6; i++) {
            if (spd[i] < 1 || spd[i] > 100) {
                cerr << "❌ 轴" << (i+1) << " 速度超限: " << spd[i] << " (范围1~100)" << endl;
                return false;
            }
        }

        vector<uint16_t> values(spd, spd + 6);
        return writeMultiRegs(REG_SPEED_AXIS1, values);
    }

    // ========================================================================
    // 设置 6 轴力（百分比 20~100）
    // ========================================================================
    bool setAxesForces6(const uint16_t force[6]) {
        for (int i = 0; i < 6; i++) {
            if (force[i] < 20 || force[i] > 100) {
                cerr << "❌ 轴" << (i+1) << " 力百分比超限: " << force[i] << " (范围20~100)" << endl;
                return false;
            }
        }

        vector<uint16_t> values(force, force + 6);
        return writeMultiRegs(REG_FORCE_AXIS1, values);
    }

    // ========================================================================
    // 读取 6 轴运行状态（0运动/1到位/2堵转）
    // ========================================================================
    bool readAxisStates6(uint16_t states[6]) {
        vector<uint16_t> values;
        if (!readHoldingRegs(REG_STATE_AXIS1, 6, values)) {
            return false;
        }
        for (int i = 0; i < 6; i++) {
            states[i] = values[i];
        }
        return true;
    }

    // ========================================================================
    // 读取 6 轴当前位置（0.01mm）
    // ========================================================================
    bool readAxesPositions6(uint16_t positions[6]) {
        vector<uint16_t> values;
        if (!readHoldingRegs(REG_CURPOS_AXIS1, 6, values)) {
            return false;
        }
        for (int i = 0; i < 6; i++) {
            positions[i] = values[i];
        }
        return true;
    }

    // ========================================================================
    // 读取 5 轴电流（mA，有符号，手册只给到轴1~5）
    // ========================================================================
    bool readCurrents5(int16_t currents[5]) {
        vector<uint16_t> values;
        if (!readHoldingRegs(REG_CURRENT_AXIS1, 5, values)) {
            return false;
        }
        for (int i = 0; i < 5; i++) {
            currents[i] = (int16_t)values[i];  // 有符号转换
        }
        return true;
    }
};

// ============================================================================
// HandActions 动作层
// ============================================================================
class HandActions {
private:
    DhHand& hand;

    // 可配置参数（需要根据实际硬件标定）
    struct Config {
        uint16_t openPos[6]   = {869, 1706, 1682, 1690, 1687, 854};  // 张开位置（默认值，需标定）
        uint16_t closePos[6]  = {30, 30, 30, 30, 30, 600}; // 闭合位置（握拳参考）
        uint16_t speed[6]     = {30, 30, 30, 30, 30, 30};     // 默认速度50%
        uint16_t force[6]     = {60, 60, 60, 60, 60, 60};     // 默认力60%
        int16_t  currentThreshold = 500;  // 电流阈值 (mA)
    } config;

    // 等待运动完成或堵转（带超时）
    bool waitMotionDone(int timeoutSec = 5) {
        auto startTime = chrono::steady_clock::now();
        
        while (true) {
            auto elapsed = chrono::duration_cast<chrono::seconds>(
                chrono::steady_clock::now() - startTime).count();
            
            if (elapsed > timeoutSec) {
                cerr << "  ⚠️ 运动超时" << endl;
                return false;
            }

            uint16_t states[6];
            if (!hand.readAxisStates6(states)) {
                this_thread::sleep_for(chrono::milliseconds(50));
                continue;
            }

            // 检查是否有堵转（警告但继续，因为到达极限位置堵转是正常的）
            bool hasBlocked = false;
            for (int i = 0; i < 6; i++) {
                if (states[i] == STATE_BLOCKED) {
                    cerr << "  ⚠️ 轴" << (i+1) << " 到达极限位置" << endl;
                    hasBlocked = true;
                }
            }
            
            // 如果有堵转，认为已经到位了
            if (hasBlocked) {
                return true;
            }

            // 检查是否全部到位
            bool allArrived = true;
            for (int i = 0; i < 6; i++) {
                if (states[i] != STATE_ARRIVED) {
                    allArrived = false;
                    break;
                }
            }

            if (allArrived) {
                return true;
            }

            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

public:
    HandActions(DhHand& h) : hand(h) {}

    // 设置动作参数（供外部标定使用）
    void setOpenPositions(const uint16_t pos[6]) {
        memcpy(config.openPos, pos, sizeof(config.openPos));
    }
    
    void setClosePositions(const uint16_t pos[6]) {
        memcpy(config.closePos, pos, sizeof(config.closePos));
    }

    void setDefaultSpeed(const uint16_t spd[6]) {
        memcpy(config.speed, spd, sizeof(config.speed));
    }

    void setDefaultForce(const uint16_t force[6]) {
        memcpy(config.force, force, sizeof(config.force));
    }

    void setCurrentThreshold(int16_t threshold) {
        config.currentThreshold = threshold;
    }

    // ========================================================================
    // 张开手（使用配置的张开位置）
    // ========================================================================
    bool openHand() {
        cout << "🖐️  执行动作：张开手" << endl;
        cout << "  目标位置: ";
        for (int i = 0; i < 6; i++) {
            cout << "轴" << (i+1) << "=" << config.openPos[i] << " ";
        }
        cout << endl;

        if (!hand.setAxesSpeeds6(config.speed)) return false;
        if (!hand.setAxesForces6(config.force)) return false;
        if (!hand.setAxesPositions6(config.openPos)) return false;

        return waitMotionDone();
    }

    // ========================================================================
    // 握拳（使用配置的闭合位置）
    // ========================================================================
    bool fist() {
        cout << "✊ 执行动作：握拳" << endl;
        cout << "  目标位置: ";
        for (int i = 0; i < 6; i++) {
            cout << "轴" << (i+1) << "=" << config.closePos[i] << " ";
        }
        cout << endl;

        if (!hand.setAxesSpeeds6(config.speed)) return false;
        if (!hand.setAxesForces6(config.force)) return false;
        if (!hand.setAxesPositions6(config.closePos)) return false;

        return waitMotionDone();
    }

    // ========================================================================
    // 布（手掌张开）
    // ========================================================================
    bool paper() {
        cout << "✋ 执行动作：布（张开）" << endl;
        return openHand();
    }

    // ========================================================================
    // 剪刀（食指+中指伸直，无名指+小拇指弯曲）
    // ========================================================================
    bool scissors() {
        cout << "✌️ 执行动作：剪刀" << endl;

        uint16_t pos[6] = {
            300,               // 轴1 拇指左右：向内收
            config.openPos[1], // 轴2 食指伸直
            config.openPos[2], // 轴3 中指伸直
            config.closePos[3],// 轴4 无名指弯曲
            config.closePos[4],// 轴5 小拇指弯曲
            700                // 轴6 拇指上下：中间位
        };

        if (!hand.setAxesSpeeds6(config.speed)) return false;
        if (!hand.setAxesForces6(config.force)) return false;
        if (!hand.setAxesPositions6(pos)) return false;

        return waitMotionDone();
    }

    // ========================================================================
    // 对捏（拇指+食指）
    // ========================================================================
    bool pinch() {
        cout << "👌 执行动作：对捏（拇指+食指）" << endl;

        // 其他手指张开，拇指和食指闭合
        uint16_t pinchPos[6] = {
            600,  // 轴1 拇指左右 - 向内
            700,  // 轴2 食指 - 弯曲
            50,   // 轴3 中指 - 张开
            50,   // 轴4 无名指 - 张开
            50,   // 轴5 小拇指 - 张开
            600   // 轴6 拇指上下 - 向下
        };

        if (!hand.setAxesSpeeds6(config.speed)) return false;
        if (!hand.setAxesForces6(config.force)) return false;
        if (!hand.setAxesPositions6(pinchPos)) return false;

        return waitMotionDone();
    }

    // ========================================================================
    // 抓瓶子（开环策略 + 电流保护）
    // 
    // 策略说明：
    // 1. 先张开到初始位置
    // 2. 拇指预摆位
    // 3. 四指一次性连续闭合（流畅）
    // 4. 高频检测电流/堵转，单轴触发后锁住该轴，其他轴继续
    // 5. 四指都触发停止或超时后结束
    // ========================================================================
    bool graspBottle(int pollIntervalMs = 60, int maxCycles = 180) {
        cout << "🍶 执行动作：抓瓶子（连续闭合 + 接触停指）" << endl;

        // Step 1: 张开到初始位置
        cout << "  📍 Step 1: 张开到初始位置" << endl;
        if (!openHand()) {
            cerr << "  ❌ 张开失败" << endl;
            return false;
        }
        this_thread::sleep_for(chrono::milliseconds(250));

        // Step 2: 设置抓取参数（四指速度更快，更接近握拳的流畅感）
        uint16_t graspSpeed[6] = {35, 30, 30,30, 30, 35};
        uint16_t graspForce[6] = {55, 55, 55, 55, 55, 55};
        const int blockedConfirmCount = 2;    // 连续2次堵转再确认
        const int currentConfirmCount = 2;    // 连续2次过流再确认
        const int currentRiseThreshold = 90;  // 相对基线电流增量阈值(mA)
        const int nearTargetTolerance = 12;   // 接近最终目标阈值(0.12mm)

        if (!hand.setAxesSpeeds6(graspSpeed)) return false;
        if (!hand.setAxesForces6(graspForce)) return false;

        // Step 3: 拇指预定位（轴1/轴6）
        cout << "  📍 Step 2: 拇指预定位..." << endl;
        uint16_t currentPos[6];
        if (!hand.readAxesPositions6(currentPos)) {
            cerr << "  ❌ 读取位置失败" << endl;
            return false;
        }
        currentPos[0] = config.closePos[0];
        currentPos[5] = config.closePos[5];
        if (!hand.setAxesPositions6(currentPos)) {
            cerr << "  ❌ 拇指预定位失败" << endl;
            return false;
        }
        this_thread::sleep_for(chrono::milliseconds(350));

        // Step 4: 构造连续闭合目标（轴2~5 直接向闭合位运动）
        uint16_t targetPos[6];
        memcpy(targetPos, config.closePos, sizeof(targetPos));
        if (!hand.readAxesPositions6(currentPos)) {
            cerr << "  ❌ 读取初始位置失败" << endl;
            return false;
        }

        uint16_t cmdPos[6];
        memcpy(cmdPos, currentPos, sizeof(cmdPos));
        cmdPos[0] = targetPos[0];
        cmdPos[5] = targetPos[5];
        for (int i = 1; i <= 4; i++) {
            cmdPos[i] = targetPos[i];
        }

        // 每轴是否停止（仅轴2~5参与）
        bool axisFinished[6] = {true, false, false, false, false, true};
        uint16_t holdPos[6];
        memcpy(holdPos, currentPos, sizeof(holdPos));
        holdPos[0] = cmdPos[0];
        holdPos[5] = cmdPos[5];
        int blockedCount[6] = {0, 0, 0, 0, 0, 0};
        int overCurrentCount[6] = {0, 0, 0, 0, 0, 0};

        // 记录空载基线电流（轴1~5）
        int baselineCurrent[5] = {0, 0, 0, 0, 0};
        {
            int16_t currents[5];
            if (hand.readCurrents5(currents)) {
                for (int i = 0; i < 5; i++) baselineCurrent[i] = abs(currents[i]);
            }
        }

        cout << "  📍 Step 3: 四指连续闭合，接触即停..." << endl;
        if (!hand.setAxesPositions6(cmdPos)) {
            cerr << "  ❌ 下发连续闭合目标失败" << endl;
            return false;
        }

        // Step 5: 高频轮询，触发后锁定该轴当前位置
        for (int cycle = 0; cycle < maxCycles; cycle++) {
            this_thread::sleep_for(chrono::milliseconds(pollIntervalMs));

            uint16_t states[6];
            if (!hand.readAxisStates6(states)) {
                continue;
            }
            if (!hand.readAxesPositions6(currentPos)) {
                continue;
            }

            int16_t currents[5];
            bool hasCurrent = hand.readCurrents5(currents);
            bool needResendCommand = false;

            for (int i = 1; i <= 4; i++) {  // 仅轴2~5
                if (axisFinished[i]) continue;

                bool nearTarget = abs((int)currentPos[i] - (int)targetPos[i]) <= nearTargetTolerance;
                if (nearTarget) {
                    axisFinished[i] = true;
                    holdPos[i] = currentPos[i];
                    needResendCommand = true;
                    cout << "      ✅ 轴" << (i+1) << " 接近闭合上限，停止该轴" << endl;
                    continue;
                }

                if (states[i] == STATE_BLOCKED) {
                    blockedCount[i]++;
                } else {
                    blockedCount[i] = 0;
                }

                bool hitCurrent = false;
                int rise = 0;
                if (hasCurrent && i < 5) {
                    int currAbs = abs(currents[i]);
                    rise = currAbs - baselineCurrent[i];
                    bool hitHardLimit = currAbs >= config.currentThreshold;
                    bool hitRiseLimit = rise >= currentRiseThreshold;
                    hitCurrent = hitHardLimit || hitRiseLimit;
                    if (hitCurrent) {
                        overCurrentCount[i]++;
                    } else {
                        overCurrentCount[i] = 0;
                        baselineCurrent[i] = (baselineCurrent[i] * 3 + currAbs) / 4;
                    }
                }

                if (blockedCount[i] >= blockedConfirmCount ||
                    overCurrentCount[i] >= currentConfirmCount) {
                    axisFinished[i] = true;
                    holdPos[i] = currentPos[i];
                    needResendCommand = true;
                    cout << "      🤏 轴" << (i+1) << " 触发接触停止";
                    if (hasCurrent && i < 5) {
                        cout << " (I=" << currents[i]
                             << "mA, 基线=" << baselineCurrent[i]
                             << "mA, 增量=" << rise << "mA)";
                    }
                    cout << endl;
                }
            }

            if (needResendCommand) {
                for (int i = 1; i <= 4; i++) {
                    cmdPos[i] = axisFinished[i] ? holdPos[i] : targetPos[i];
                }
                if (!hand.setAxesPositions6(cmdPos)) {
                    cerr << "  ❌ 更新停指目标失败" << endl;
                    return false;
                }
            }

            bool fingersDone = axisFinished[1] && axisFinished[2] &&
                               axisFinished[3] && axisFinished[4];
            if (fingersDone) {
                cout << "  ✅ 四指已接触停止，抓瓶完成" << endl;
                return true;
            }
        }

        cout << "⚠️ 抓瓶超时：未在规定时间内全部触发停止" << endl;
        return false;
    }

    // ========================================================================
    // 标定方法：逐步测试位置范围
    // 
    // 使用方法：
    // 1. 手动调用此函数，指定轴号和起始位置
    // 2. 函数会逐步增加位置，同时监测电流
    // 3. 当电流超过阈值或位置到达上限时停止
    // 4. 输出最大安全位置供配置使用
    // ========================================================================
    bool calibrateAxis(int axisIndex, uint16_t startPos = 0, uint16_t step = 50, uint16_t maxPos = 1000) {
        cout << "🔧 标定轴" << (axisIndex + 1) << " (起始=" << startPos 
             << ", 步进=" << step << ", 最大=" << maxPos << ")" << endl;

        if (axisIndex < 0 || axisIndex >= 6) {
            cerr << "❌ 轴索引无效" << endl;
            return false;
        }

        // 设置慢速
        uint16_t speed[6] = {20, 20, 20, 20, 20, 20};
        uint16_t force[6] = {40, 40, 40, 40, 40, 40};
        
        hand.setAxesSpeeds6(speed);
        hand.setAxesForces6(force);

        uint16_t pos[6];
        hand.readAxesPositions6(pos);
        pos[axisIndex] = startPos;

        for (uint16_t testPos = startPos; testPos <= maxPos; testPos += step) {
            pos[axisIndex] = testPos;
            
            cout << "  测试位置: " << testPos << " ... ";
            
            if (!hand.setAxesPositions6(pos)) {
                cout << "设置失败" << endl;
                return false;
            }

            this_thread::sleep_for(chrono::milliseconds(500));

            // 检查状态
            uint16_t states[6];
            if (hand.readAxisStates6(states)) {
                if (states[axisIndex] == STATE_BLOCKED) {
                    cout << "堵转！" << endl;
                    cout << "⚠️ 最大安全位置: " << (testPos - step) << endl;
                    return true;
                }
            }

            // 检查电流（仅前5轴）
            if (axisIndex < 5) {
                int16_t currents[5];
                if (hand.readCurrents5(currents)) {
                    cout << "电流=" << currents[axisIndex] << "mA";
                    if (abs(currents[axisIndex]) > config.currentThreshold) {
                        cout << " (超阈值)" << endl;
                        cout << "⚠️ 最大安全位置: " << testPos << endl;
                        return true;
                    }
                }
            }

            cout << "OK" << endl;
        }

        cout << "✅ 标定完成，未触及限位" << endl;
        return true;
    }
};

// ============================================================================
// main() 演示
// ============================================================================
int main(int argc, char* argv[]) {
    cout << "========================================" << endl;
    cout << "  DH-5-6 灵巧手驱动演示" << endl;
    cout << "========================================" << endl;

    // 默认 Linux 串口，可通过命令行参数覆盖
    // 用法:
    //   ./dh5_hand_driver /dev/ttyUSB0 fist
    //   ./dh5_hand_driver /dev/ttyUSB0 scissors
    //   ./dh5_hand_driver /dev/ttyUSB0 paper
    //   ./dh5_hand_driver /dev/ttyUSB0 bottle
    //   ./dh5_hand_driver /dev/ttyUSB0 demo
    string port = "/dev/ttyUSB0";
    string action = "demo";

    if (argc > 1) {
        string arg1 = argv[1];
        if (arg1.rfind("/dev/", 0) == 0) {
            port = arg1;
        } else {
            action = arg1;
        }
    }
    if (argc > 2) {
        action = argv[2];
    }

    // 初始化驱动
    DhHand hand;
    if (!hand.open(port)) {
        cerr << "❌ 初始化失败" << endl;
        return 1;
    }

    // Step 1: 清除错误
    if (!hand.clearError()) {
        cerr << "❌ 清除错误失败" << endl;
    }
    this_thread::sleep_for(chrono::milliseconds(200));

    // Step 2: 回零（使用张开模式，让手处于张开状态）
    cout << "\n提示：使用张开模式回零，手会向张开方向运动" << endl;
    if (!hand.homeOpenAllAxes(15)) {
        cerr << "❌ 回零失败，程序退出" << endl;
        return 1;
    }
    cout << "⏳ 回零完成，等待手指完全到位..." << endl;
    this_thread::sleep_for(chrono::milliseconds(3000));
    
    // 读取回零后的位置
    uint16_t currentPos[6];
    if (hand.readAxesPositions6(currentPos)) {
        cout << "\n📍 回零后各轴位置: ";
        for (int i = 0; i < 6; i++) {
            cout << "轴" << (i+1) << "=" << currentPos[i] << " ";
        }
        cout << endl;
    }

    // Step 3: 创建动作控制器
    HandActions actions(hand);

    // （可选）如果已经标定过，可以在这里设置自定义参数
    // uint16_t customOpenPos[6] = {800, 50, 50, 50, 50, 400};
    // actions.setOpenPositions(customOpenPos);

    bool ok = true;
    if (action == "fist") {
        cout << "\n--- 执行: 握拳 ---" << endl;
        ok = actions.fist();
    } else if (action == "scissors") {
        cout << "\n--- 执行: 剪刀 ---" << endl;
        ok = actions.scissors();
    } else if (action == "paper" || action == "open") {
        cout << "\n--- 执行: 布（张开） ---" << endl;
        ok = actions.paper();
    } else if (action == "bottle" || action == "grasp") {
        cout << "\n--- 执行: 抓瓶子 ---" << endl;
        actions.setCurrentThreshold(600);
        ok = actions.graspBottle(60, 180);
    } else if (action == "demo") {
        cout << "\n--- 演示: 握拳 -> 剪刀 -> 布 ---" << endl;
        ok = actions.fist();
        this_thread::sleep_for(chrono::milliseconds(1200));
        ok = ok && actions.scissors();
        this_thread::sleep_for(chrono::milliseconds(1200));
        ok = ok && actions.paper();
    } else {
        cerr << "❌ 未知动作: " << action << endl;
        cerr << "可用动作: fist | scissors | paper | bottle | demo" << endl;
        hand.close();
        return 2;
    }

    if (!ok) {
        cerr << "⚠️ 动作执行失败或未完全到位" << endl;
    }
    this_thread::sleep_for(chrono::milliseconds(1500));
    
    // 读取张开后的位置
    if (hand.readAxesPositions6(currentPos)) {
        cout << "📍 张开后各轴位置: ";
        for (int i = 0; i < 6; i++) {
            cout << "轴" << (i+1) << "=" << currentPos[i] << " ";
        }
        cout << endl;
    }
    
    cout << "\n✅ 执行完成！" << endl;
    cout << "\n📝 标定说明：" << endl;
    cout << "  - 默认位置参数可能需要根据实际硬件调整" << endl;
    cout << "  - 可使用 actions.calibrateAxis(轴索引) 进行逐步标定" << endl;
    cout << "  - 例如: actions.calibrateAxis(1, 0, 50, 1000) 标定食指" << endl;

    hand.close();

    return 0;
}
