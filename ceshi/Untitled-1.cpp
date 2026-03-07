// #include <iostream>
// #include <vector>
// #include <stdint.h>
// #include <string.h>

// #ifdef _WIN32
// #include <windows.h>
// #else
// #include <fcntl.h>
// #include <termios.h>
// #include <unistd.h>
// #endif

// using namespace std;

// // ------------------------ CRC16 (Modbus) ------------------------
// uint16_t CRC16_Modbus(const uint8_t* data, uint16_t length)
// {
//     uint16_t crc = 0xFFFF;
//     for (uint16_t i = 0; i < length; i++) {
//         crc ^= data[i];
//         for (uint16_t j = 0; j < 8; j++) {
//             if (crc & 0x01)
//                 crc = (crc >> 1) ^ 0xA001;
//             else
//                 crc >>= 1;
//         }
//     }
//     return crc;
// }

// // ------------------------ 串口类（Win/Linux 自动切换） ------------------------
// class SerialPort {
// public:
// #ifdef _WIN32
//     HANDLE hComm;
// #else
//     int fd;
// #endif

//     bool openPort(const char* port, int baudrate = 115200) {
// #ifdef _WIN32
//         hComm = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
//         if (hComm == INVALID_HANDLE_VALUE) return false;

//         DCB dcb = {0};
//         dcb.DCBlength = sizeof(DCB);
//         GetCommState(hComm, &dcb);
//         dcb.BaudRate = baudrate;
//         dcb.ByteSize = 8;
//         dcb.StopBits = ONESTOPBIT;
//         dcb.Parity   = NOPARITY;
//         SetCommState(hComm, &dcb);

//         return true;
// #else
//         fd = ::open(port, O_RDWR | O_NOCTTY | O_SYNC);
//         if (fd < 0) return false;

//         struct termios tty;
//         memset(&tty, 0, sizeof tty);
//         tcgetattr(fd, &tty);

//         cfsetospeed(&tty, B115200);
//         cfsetispeed(&tty, B115200);

//         tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
//         tty.c_cflag &= ~(PARENB | CSTOPB);
//         tty.c_cflag |= CREAD | CLOCAL;

//         tcsetattr(fd, TCSANOW, &tty);
//         return true;
// #endif
//     }

//     int writeData(const uint8_t* buffer, int size) {
// #ifdef _WIN32
//         DWORD written;
//         WriteFile(hComm, buffer, size, &written, NULL);
//         return written;
// #else
//         return ::write(fd, buffer, size);
// #endif
//     }
// };

// // ------------------------ 组帧（0x6A 写多个参数） ------------------------
// vector<uint8_t> buildFrame_6A(
//     uint8_t deviceID,
//     vector<pair<uint16_t, uint16_t>> params   // (寄存器, 值)
// ) {
//     vector<uint8_t> frame;

//     // ---- 地址码 ----
//     frame.push_back(deviceID);

//     // 先占位，为了之后再回填长度
//     frame.push_back(0x00); // length high
//     frame.push_back(0x00); // length low

//     // ---- 功能码 ----
//     frame.push_back(0x6A);

//     // ---- 数据长度（稍后计算） ----
//     frame.push_back(0x00);
//     frame.push_back(0x00);

//     // ---- 参数数量 ----
//     frame.push_back(params.size());

//     // ---- 参数内容 ----
//     for (auto& p : params) {
//         uint16_t reg = p.first;
//         uint16_t val = p.second;

//         frame.push_back(reg >> 8);
//         frame.push_back(reg & 0xFF);

//         frame.push_back(val >> 8);
//         frame.push_back(val & 0xFF);
//     }

//     // ---- 回填数据长度(参数数量 + 参数内容) ----
//     uint16_t dataLen = 1 + params.size() * 4;
//     frame[4] = dataLen >> 8;
//     frame[5] = dataLen & 0xFF;

//     // ---- 回填整个帧长度 = 地址码到 CRC 前所有字节 + CRC2字节 ----
//     uint16_t frameLen = frame.size() + 2;
//     frame[1] = frameLen >> 8;
//     frame[2] = frameLen & 0xFF;

//     // ---- CRC ----
//     uint16_t crc = CRC16_Modbus(frame.data(), frame.size());
//     frame.push_back(crc & 0xFF);
//     frame.push_back((crc >> 8) & 0xFF);

//     return frame;
// }

// void homing(SerialPort& sp)
// {
//     // 主机ID=01, 功能码=06, 寄存器=0x0100, 值=0x0AAA
//     uint8_t frame[] = { 
//         0x01, 0x06, 
//         0x01, 0x00, 
//         0x0A, 0xAA,
//         0x00, 0x00  // CRC占位
//     };
    
//     // 计算CRC
//     uint16_t crc = CRC16_Modbus(frame, 6);
//     frame[6] = crc & 0xFF;
//     frame[7] = crc >> 8;

//     sp.writeData(frame, 8);

//     cout << "已发送回零指令（Homing）..." << endl;

//     // 给设备一点时间执行回零动作（2 秒通常够了）
//     Sleep(3000);
// }

// // ------------------------ 主程序示例 ------------------------
// int main() {
//     SerialPort sp;

// #ifdef _WIN32
//     const char* port = "\\\\.\\COM3";   // 根据你的 RS485 转换器修改
// #else
//     const char* port = "/dev/ttyUSB0";
// #endif

//     if (!sp.openPort(port)) {
//         cout << "⚠️ 打开串口失败！" << endl;
//         return -1;
//     }

//     cout << "✅ 串口已打开: " << port << endl;
//      homing(sp);
//      Sleep(1000);
//     // 示例：轴 2（食指）设置：位置=5mm、力100%、速度100%
//     vector<pair<uint16_t, uint16_t>> params = {
//         {0x0102, 500},   // 位置 = 500 = 5mm (0.01mm单位)
//         {0x0108, 80},   // 力 = 100%
//         {0x010E, 40}    // 速度 = 100%
//     };

//     auto frame = buildFrame_6A(0x01, params);

//     cout << "发送指令：" << endl;
//     for (auto b : frame) printf("%02X ", b);
//     cout << endl;

//     sp.writeData(frame.data(), frame.size());

//     cout << "👍 已发送 0x6A 指令到灵巧手！" << endl;

//     return 0;
// }


// #include <iostream>
// #include <vector>
// #include <stdint.h>
// #include <string.h>

// #ifdef _WIN32
// #include <windows.h>
// #else
// #include <fcntl.h>
// #include <termios.h>
// #include <unistd.h>
// #define Sleep(msec) usleep((msec) * 1000)
// #endif

// using namespace std;

// // ---------------------------------------------------------------
// // CRC16 (Modbus) 计算函数
// // ---------------------------------------------------------------
// uint16_t CRC16_Modbus(const uint8_t* data, uint16_t length)
// {
//     uint16_t crc = 0xFFFF;
//     for (uint16_t i = 0; i < length; i++) {
//         crc ^= data[i];
//         for (uint16_t j = 0; j < 8; j++) {
//             crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
//         }
//     }
//     return crc;
// }

// // ---------------------------------------------------------------
// // 串口类（跨平台）
// // ---------------------------------------------------------------
// class SerialPort {
// public:
// #ifdef _WIN32
//     HANDLE hComm;
// #else
//     int fd;
// #endif

//     bool openPort(const char* port, int baudrate = 115200) {
// #ifdef _WIN32
//         hComm = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
//         if (hComm == INVALID_HANDLE_VALUE) return false;

//         DCB dcb = { 0 };
//         dcb.DCBlength = sizeof(DCB);
//         GetCommState(hComm, &dcb);
//         dcb.BaudRate = baudrate;
//         dcb.ByteSize = 8;
//         dcb.StopBits = ONESTOPBIT;
//         dcb.Parity = NOPARITY;
//         SetCommState(hComm, &dcb);

//         return true;
// #else
//         fd = ::open(port, O_RDWR | O_NOCTTY | O_SYNC);
//         if (fd < 0) return false;

//         struct termios tty;
//         memset(&tty, 0, sizeof tty);
//         tcgetattr(fd, &tty);
//         cfsetospeed(&tty, B115200);
//         cfsetispeed(&tty, B115200);

//         tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
//         tty.c_cflag &= ~(PARENB | CSTOPB);
//         tty.c_cflag |= CREAD | CLOCAL;

//         tcsetattr(fd, TCSANOW, &tty);
//         return true;
// #endif
//     }

//     int writeData(const uint8_t* buffer, int size) {
// #ifdef _WIN32
//         DWORD written;
//         WriteFile(hComm, buffer, size, &written, NULL);
//         return written;
// #else
//         return ::write(fd, buffer, size);
// #endif
//     }

//     int readData(uint8_t* buffer, int maxLen, int timeoutMs = 50) {
// #ifdef _WIN32
//         COMMTIMEOUTS timeouts = { 0 };
//         timeouts.ReadIntervalTimeout = timeoutMs;
//         timeouts.ReadTotalTimeoutConstant = timeoutMs;
//         timeouts.ReadTotalTimeoutMultiplier = 10;
//         SetCommTimeouts(hComm, &timeouts);

//         DWORD bytesRead = 0;
//         ReadFile(hComm, buffer, maxLen, &bytesRead, NULL);
//         return bytesRead;
// #else
//         return ::read(fd, buffer, maxLen);
// #endif
//     }
// };

// // ---------------------------------------------------------------
// // HandController 类 —— 主要控制接口
// // ---------------------------------------------------------------
// class HandController {
// private:
//     SerialPort sp;

// public:
//     HandController(const char* portName) {
//         if (!sp.openPort(portName)) {
//             cout << "❌ 打开串口失败！" << endl;
//             exit(1);
//         }
//         cout << "✅ 串口已打开：" << portName << endl;
//     }

//     // -----------------------------------------------------------
//     // 自动组帧（0x6A 写多个参数）
//     // -----------------------------------------------------------
//     vector<uint8_t> buildFrame_6A(uint8_t deviceID,
//         vector<pair<uint16_t, uint16_t>> params)
//     {
//         vector<uint8_t> frame;

//         frame.push_back(deviceID);
//         frame.push_back(0x00);
//         frame.push_back(0x00);
//         frame.push_back(0x6A);

//         frame.push_back(0x00);
//         frame.push_back(0x00);

//         frame.push_back(params.size());

//         for (auto& p : params)
//         {
//             frame.push_back(p.first >> 8);
//             frame.push_back(p.first & 0xFF);
//             frame.push_back(p.second >> 8);
//             frame.push_back(p.second & 0xFF);
//         }

//         uint16_t dataLen = 1 + params.size() * 4;
//         frame[4] = dataLen >> 8;
//         frame[5] = dataLen & 0xFF;

//         uint16_t frameLen = frame.size() + 2;
//         frame[1] = frameLen >> 8;
//         frame[2] = frameLen & 0xFF;

//         uint16_t crc = CRC16_Modbus(frame.data(), frame.size());
//         frame.push_back(crc & 0xFF);
//         frame.push_back(crc >> 8);

//         return frame;
//     }

//     vector<uint8_t> buildFrame_69(uint8_t deviceID, vector<uint16_t> regs)
//     {
//         vector<uint8_t> frame;

//         frame.push_back(deviceID);
//         frame.push_back(0x00);
//         frame.push_back(00);
//         frame.push_back(0x69);

//         uint16_t dataLen = 1 + regs.size() * 2;
//         frame.push_back(dataLen >> 8);
//         frame.push_back(dataLen & 0xFF);

//         frame.push_back(regs.size());

//         for (auto r : regs) {
//             frame.push_back(r >> 8);
//             frame.push_back(r & 0xFF);
//         }

//         uint16_t frameLen = frame.size() + 2;
//         frame[1] = frameLen >> 8;
//         frame[2] = frameLen & 0xFF;

//         uint16_t crc = CRC16_Modbus(frame.data(), frame.size());
//         frame.push_back(crc & 0xFF);
//         frame.push_back(crc >> 8);

//         return frame;
//     }

//     // -----------------------------------------------------------
//     // 正确的回零指令（0x0100 = 0x0555）
//     // -----------------------------------------------------------
//     void homing() {

//         // 闭合回零模式：0x0555
//         uint8_t frame[] = {
//             0x01, 0x06,
//             0x01, 0x00,       // 0x0100 回零寄存器
//             0x05, 0x55,       // 每轴 01 = 闭合回零
//             0x00, 0x00        // CRC
//         };

//         uint16_t crc = CRC16_Modbus(frame, 6);
//         frame[6] = crc & 0xFF;
//         frame[7] = crc >> 8;

//         sp.writeData(frame, 8);

//         cout << "🔄 已发送 回零指令（闭合回零 0x0555）..." << endl;
//         Sleep(100);
//     }

//     // -----------------------------------------------------------
//     // 等待回零完成（读取 0x0200 状态）
//     // -----------------------------------------------------------
//     void waitHomingDone()
//     {
//         cout << "⏳ 等待回零完成..." << endl;

//         while (true)
//         {
//             auto frame = buildFrame_69(0x01, { 0x0200 });
//             sp.writeData(frame.data(), frame.size());

//             Sleep(20);

//             uint8_t recv[64] = { 0 };
//             int len = sp.readData(recv, sizeof(recv));

//             if (len < 10)
//             {
//                 Sleep(50);
//                 continue;
//             }

//             // 数据区：recv[7] 高字节，recv[8] 低字节
//             uint16_t status = (recv[7] << 8) | recv[8];

//             bool allReady = true;
//             for (int axis = 0; axis < 6; axis++)
//             {
//                 int bits = (status >> (axis * 2)) & 0x03;

//                 // 01 = 回零成功
//                 if (bits != 0x01)
//                     allReady = false;
//             }

//             if (allReady) break;

//             Sleep(50);
//         }

//         cout << "✅ 回零完成！" << endl;
//     }

//     // -----------------------------------------------------------
//     // 控制任意手指（位置 mm，力%，速度%）
//     // -----------------------------------------------------------
//     void moveFinger(int finger, float pos_mm, int force, int speed)
//     {
//         uint16_t posValue = (uint16_t)(pos_mm * 100);  // mm → 0.01mm

//         uint16_t posReg = 0x0100 + finger;     // 0x0101~0x0106
//         uint16_t forceReg = 0x0106 + finger;   // 0x0107~0x010C
//         uint16_t speedReg = 0x010C + finger;   // 0x010D~0x0112

//         vector<pair<uint16_t, uint16_t>> params = {
//             { posReg, posValue },
//             { forceReg, force },
//             { speedReg, speed }
//         };

//         auto frame = buildFrame_6A(0x01, params);
//         sp.writeData(frame.data(), frame.size());

//         cout << "👉 发送动作：轴 " << finger
//             << " | 位置 = " << pos_mm << " mm"
//             << " | 力 = " << force
//             << " | 速度 = " << speed << endl;
//     }
// };

// // ---------------------------------------------------------------
// // main()
// // ---------------------------------------------------------------
// int main() {

//     HandController hand("\\\\.\\COM3");

//     hand.homing();             // 设置回零模式
//     hand.waitHomingDone();     // 等待六轴全部回零

//     //hand.moveFinger(2, 5, 80, 40); // 食指
//     //hand.moveFinger(3, 5, 70, 60); // 中指

//     return 0;
// }

#include <iostream>
#include <vector>
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000)
#endif

using namespace std;

// ---------------------------------------------------------------
// CRC16
// ---------------------------------------------------------------
uint16_t CRC16_Modbus(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint16_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------
class SerialPort {
public:
#ifdef _WIN32
    HANDLE hComm;
#else
    int fd;
#endif

    bool openPort(const char* port, int baudrate = 115200) {
#ifdef _WIN32
        hComm = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
        if (hComm == INVALID_HANDLE_VALUE) return false;

        DCB dcb = { 0 };
        dcb.DCBlength = sizeof(DCB);
        GetCommState(hComm, &dcb);
        dcb.BaudRate = baudrate;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        SetCommState(hComm, &dcb);

        return true;
#else
        fd = ::open(port, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return false;

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~(PARENB | CSTOPB);
        tty.c_cflag |= CREAD | CLOCAL;

        tcsetattr(fd, TCSANOW, &tty);
        return true;
#endif
    }

    int writeData(const uint8_t* buffer, int size) {
#ifdef _WIN32
        DWORD written;
        WriteFile(hComm, buffer, size, &written, NULL);
        return written;
#else
        return ::write(fd, buffer, size);
#endif
    }

    int readData(uint8_t* buffer, int maxLen, int timeoutMs = 50) {
#ifdef _WIN32
        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout = timeoutMs;
        timeouts.ReadTotalTimeoutConstant = timeoutMs;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        SetCommTimeouts(hComm, &timeouts);

        DWORD bytesRead = 0;
        ReadFile(hComm, buffer, maxLen, &bytesRead, NULL);
        return bytesRead;
#else
        return ::read(fd, buffer, maxLen);
#endif
    }
};

// ---------------------------------------------------------------
class HandController {
private:
    SerialPort sp;

public:
    HandController(const char* portName) {
        if (!sp.openPort(portName)) {
            cout << " 打开串口失败！" << endl;
            exit(1);
        }
        cout << " 串口已打开：" << portName << endl;
    }
// 读取任意寄存器（使用 0x69）
bool readRegs(vector<uint16_t> regs, uint8_t* recvBuf, int& recvLen)
{
    // 构造 0x69 帧
    vector<uint8_t> frame;

    frame.push_back(0x01);  // ID
    frame.push_back(0x00);  // frame len high (稍后填充)
    frame.push_back(0x00);  // frame len low
    frame.push_back(0x69);  // function code

    uint16_t dataLen = 1 + regs.size()*2;
    frame.push_back(dataLen >> 8);
    frame.push_back(dataLen & 0xFF);

    frame.push_back(regs.size());

    for(auto r : regs){
        frame.push_back(r >> 8);
        frame.push_back(r & 0xFF);
    }

    // 计算总长度
    uint16_t frameLen = frame.size() + 2;
    frame[1] = frameLen >> 8;
    frame[2] = frameLen & 0xFF;

    uint16_t crc = CRC16_Modbus(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    // 发送
    sp.writeData(frame.data(), frame.size());

    Sleep(20);

    recvLen = sp.readData(recvBuf, 64);
    return (recvLen > 0);
}

    // -----------------------------------------------------------
    // 回零
    // -----------------------------------------------------------
    void homing() {
        uint8_t frame[] = {
            0x01, 0x06,
            0x01, 0x00,
            //0x05, 0x55,   //  闭合回零（兼容全部手指）
             0x0A, 0xAA,   //  张开回零模式
            0x00, 0x00
        };

        uint16_t crc = CRC16_Modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = crc >> 8;

        sp.writeData(frame, 8);
        Sleep(100);
        cout << " 已发送回零指令" << endl;
    }

    void waitHomingDone() {
        cout << " 等待回零完成..." << endl;
        while (true) {
            uint8_t cmd[] = { 0x01,0x00,0x0B,0x69,0x00,0x03,0x01,0x02,0x00,0x00,0x00 };
            uint16_t crc = CRC16_Modbus(cmd, 9);
            cmd[9] = crc & 0xFF;
            cmd[10] = crc >> 8;

            sp.writeData(cmd, 11);
            Sleep(20);

            uint8_t recv[32] = {0};
            int len = sp.readData(recv, sizeof(recv));
            if (len < 10) continue;

            uint16_t status = (recv[7] << 8) | recv[8];

            bool ok = true;
            for(int i=0;i<6;i++){
                int bits = (status >> (i*2)) & 0x03;
                if(bits != 1) ok = false;
            }
            if(ok) break;
            Sleep(50);
        }
        cout << " 全部完成回零！" << endl;
    }

    // -----------------------------------------------------------
    // 读取 6 个手指当前位置 (0x0207–0x020C)
    // -----------------------------------------------------------
    vector<uint16_t> readAllPositions() {
        vector<uint16_t> result(6, 0);

        // 构建 0x69 读多个寄存器（从 0x0207 开始读 6 个）
        vector<uint8_t> frame = {
            0x01,0x00,0x00,0x69,
            0x00,0x0D, // data len
            0x06,      // 参数数量
            0x02,0x07, // 地址 0207
            0x02,0x08,
            0x02,0x09,
            0x02,0x0A,
            0x02,0x0B,
            0x02,0x0C,
        };

        uint16_t l = frame.size()+2;
        frame[1] = l >> 8;
        frame[2] = l & 0xFF;

        uint16_t crc = CRC16_Modbus(frame.data(), frame.size());
        frame.push_back(crc & 0xFF);
        frame.push_back(crc >> 8);

        sp.writeData(frame.data(), frame.size());
        Sleep(30);

        uint8_t recv[128] = {0};
        int len = sp.readData(recv, sizeof(recv));

        if(len < 20){
            cout<<" 读取位置失败（长度不足）"<<endl;
            return result;
        }

        // recv[7..] 开始是数据
        int index = 7;
        for(int i=0;i<6;i++){
            uint16_t v = (recv[index] << 8) | recv[index+1];
            result[i] = v;
            index += 2;
        }

        return result;
    }

    void moveSingleFinger(int finger, float pos_mm, int force, int speed)
    {
        finger -= 1;

        uint16_t pos = (uint16_t)(pos_mm * 100);
        uint16_t regPos    = 0x0101 + finger;
        uint16_t regForce  = 0x0107 + finger;
        uint16_t regSpeed  = 0x010D + finger;

        vector<uint8_t> frame;

        frame.push_back(0x01);         // ID
        frame.push_back(0x00);         // frame len hi (placeholder)
        frame.push_back(0x00);         // frame len lo
        frame.push_back(0x6A);         // function code

        uint16_t dataLen = 1 + 3 * 4;  // paramCount + 3*(index+value)
        frame.push_back(dataLen >> 8);
        frame.push_back(dataLen & 0xFF);

        frame.push_back(3);            // 参数数量 = 3

        // 写位置
        frame.push_back(regPos >> 8);
        frame.push_back(regPos & 0xFF);
        frame.push_back(pos >> 8);
        frame.push_back(pos & 0xFF);

        // 写力
        frame.push_back(regForce >> 8);
        frame.push_back(regForce & 0xFF);
        frame.push_back(force >> 8);
        frame.push_back(force & 0xFF);

        // 写速度
        frame.push_back(regSpeed >> 8);
        frame.push_back(regSpeed & 0xFF);
        frame.push_back(speed >> 8);
        frame.push_back(speed & 0xFF);

        // 填 frameLen
        uint16_t frameLen = frame.size() + 2;
        frame[1] = frameLen >> 8;
        frame[2] = frameLen & 0xFF;

        uint16_t crc = CRC16_Modbus(frame.data(), frame.size());
        frame.push_back(crc & 0xFF);
        frame.push_back(crc >> 8);

        sp.writeData(frame.data(), frame.size());
    }
};

void debugReadHomingStatus(HandController& hand)
{
    uint8_t recv[64]={0};
    int len = 0;

    if(!hand.readRegs({0x0200}, recv, len)){
        cout << " 读取失败（无数据）" << endl;
        return;
    }

    cout << " Recv len = " << len << endl;
    cout << " Raw = ";
    for(int i=0;i<len;i++) printf("%02X ", recv[i]);
    cout << endl;

    if(len < 10){
        cout << " 数据太短，无法解析" << endl;
        return;
    }

    uint16_t status = (recv[7]<<8) | recv[8];
    printf(" 回零寄存器 0x0200 = 0x%04X\n", status);

    for(int axis=0; axis<6; axis++){
        int bits = (status >> (axis*2)) & 0x03;
        printf("  轴%d 状态 = %d\n", axis+1, bits);
    }
}


// ---------------------------------------------------------------
int main()
{
    HandController hand("\\\\.\\COM3");

     hand.homing();
     Sleep(5000);
    //debugReadHomingStatus(hand);
     //hand.waitHomingDone();

    //  只动食指：5mm，力80，速度40
    //hand.moveSingleFinger(1, 1, 20, 20);
    //Sleep(1);
    //hand.moveSingleFinger(6, 30, 20, 20);
    //Sleep(1);
    hand.moveSingleFinger(2, 1, 20, 20);
    Sleep(1);
    //  只动中指：8mm，力60，速度30
    hand.moveSingleFinger(3, 1, 20, 20);
        Sleep(1);
    hand.moveSingleFinger(4, 1, 20, 20);
        Sleep(1);
    hand.moveSingleFinger(5, 1, 20, 20);
     Sleep(1);
    //hand.moveSingleFinger(5, 5, 80, 40);

    return 0;
}
