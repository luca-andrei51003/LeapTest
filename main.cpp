#include "LeapC.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <cstdint>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")

extern "C" {
#include "third_party/mavlink/include/mavlink/v2.0/common/mavlink.h"
}

#define RAD_TO_DEG 57.2958f
using namespace std;

// ======================= CONFIG =======================
// Serial (SiK USB radio)
static const char* SERIAL_PORT_NAME = "COM6";

// IMPORTANT: SiK V3 default is 57600. Pixhawk TELEM1 commonly 57600.
// Use 57600 unless you reconfigure BOTH radios AND SERIAL1_BAUD on Pixhawk.
static const DWORD SERIAL_BAUDRATE = CBR_57600;

// MAVLink identity: make ArduPilot treat you like a real GCS.
// SYSID_MYGCS on ArduPilot should be set to 255.
static const uint8_t SYS_ID  = 255;
static const uint8_t COMP_ID = MAV_COMP_ID_MISSIONPLANNER; // 190

// Vehicle SYSID usually 1 (ArduPilot default)
static const uint8_t VEHICLE_SYSID = 1;

// UDP mirror: Mission Planner listens by default on 14550
static const char* UDP_IP   = "127.0.0.1";
static const uint16_t UDP_PORT = 14550;

// ======================= UDP MIRROR =======================
class UdpMirror {
public:
    bool start(const char* ip, uint16_t port) {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
            std::cerr << "WSAStartup failed\n";
            return false;
        }

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET) {
            std::cerr << "UDP socket() failed\n";
            WSACleanup();
            return false;
        }

        // Bind to an ephemeral local port so we can receive replies from Mission Planner
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(0); // ephemeral
        if (bind(sock, (sockaddr*)&local, sizeof(local)) != 0) {
            std::cerr << "UDP bind() failed, WSA=" << WSAGetLastError() << "\n";
            closesocket(sock);
            sock = INVALID_SOCKET;
            WSACleanup();
            return false;
        }

        memset(&mpAddr, 0, sizeof(mpAddr));
        mpAddr.sin_family = AF_INET;
        mpAddr.sin_port = htons(port);
        if (inet_pton(AF_INET, ip, &mpAddr.sin_addr) != 1) {
            std::cerr << "inet_pton failed for " << ip << "\n";
            closesocket(sock);
            sock = INVALID_SOCKET;
            WSACleanup();
            return false;
        }

        running.store(true);
        return true;
    }

    void stop() {
        running.store(false);
        if (sock != INVALID_SOCKET) {
            closesocket(sock);
            sock = INVALID_SOCKET;
        }
        WSACleanup();
    }

    // Send a fully formed MAVLink packet buffer
    void sendPacket(const uint8_t* data, size_t len) {
        if (!running.load() || sock == INVALID_SOCKET) return;
        if (!data || len == 0) return;
        sendto(sock, (const char*)data, (int)len, 0, (sockaddr*)&mpAddr, sizeof(mpAddr));
    }

    // Convenience: send a mavlink_message_t by serializing it to a packet
    void sendMessage(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        sendPacket(buf, len);
    }

    // Receive raw UDP datagrams from Mission Planner (non-blocking with short select)
    int recvPacket(uint8_t* out, int maxLen) {
        if (!running.load() || sock == INVALID_SOCKET) return 0;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 2000; // 2 ms

        int r = select(0, &fds, nullptr, nullptr, &tv);
        if (r <= 0) return 0;

        sockaddr_in from{};
        int fromLen = sizeof(from);
        int n = recvfrom(sock, (char*)out, maxLen, 0, (sockaddr*)&from, &fromLen);
        if (n <= 0) return 0;

        return n;
    }

private:
    SOCKET sock = INVALID_SOCKET;
    sockaddr_in mpAddr{};
    std::atomic<bool> running{false};
};

// ======================= SERIAL PORT =======================
class SerialPort {
public:
    SerialPort() : hComm(INVALID_HANDLE_VALUE) {}
    ~SerialPort() { close(); }

    bool open(const char* portName, DWORD baudRate) {
        // For COM10+ you must use "\\\\.\\COM10". COM6 is fine as-is,
        // but we handle both cases robustly:
        std::string fullName = portName;
        if (fullName.rfind("COM", 0) == 0 && fullName.size() > 4) { // COM10 or more
            fullName = "\\\\.\\" + fullName;
        }

        hComm = CreateFileA(
            fullName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,              // exclusive access (IMPORTANT)
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr
        );

        if (hComm == INVALID_HANDLE_VALUE) {
            cerr << "Failed to open serial port " << portName
                 << " (error " << GetLastError() << ")\n";
            return false;
        }

        SetupComm(hComm, 4096, 4096);

        COMMTIMEOUTS timeouts{};
        // Non-blocking-ish: ReadFile returns quickly
        timeouts.ReadIntervalTimeout         = 20;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.ReadTotalTimeoutConstant    = 20;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant   = 50;
        SetCommTimeouts(hComm, &timeouts);

        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(hComm, &dcb)) {
            cerr << "GetCommState failed (error " << GetLastError() << ")\n";
            close();
            return false;
        }

        dcb.BaudRate = baudRate;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;

        // No hardware flow control (unless you know RTS/CTS is wired end-to-end)
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl  = DTR_CONTROL_DISABLE;
        dcb.fRtsControl  = RTS_CONTROL_DISABLE;

        // No software flow control
        dcb.fInX  = FALSE;
        dcb.fOutX = FALSE;

        if (!SetCommState(hComm, &dcb)) {
            cerr << "SetCommState failed (error " << GetLastError() << ")\n";
            close();
            return false;
        }

        PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
        cout << "Serial port " << portName << " opened at " << baudRate << " bps\n";
        return true;
    }

    bool writeBytes(const uint8_t* data, size_t len) {
        if (hComm == INVALID_HANDLE_VALUE) return false;
        DWORD bytesWritten = 0;
        if (!WriteFile(hComm, data, (DWORD)len, &bytesWritten, nullptr)) {
            cerr << "Serial WriteFile failed (error " << GetLastError() << ")\n";
            return false;
        }
        return (bytesWritten == len);
    }

    // Read up to maxLen bytes, returns number of bytes read (0..maxLen)
    size_t readBytes(uint8_t* out, size_t maxLen) {
        if (hComm == INVALID_HANDLE_VALUE) return 0;
        DWORD bytesRead = 0;
        if (!ReadFile(hComm, out, (DWORD)maxLen, &bytesRead, nullptr)) {
            // timeouts can still return false in some cases; treat as no data
            return 0;
        }
        return (size_t)bytesRead;
    }

    void close() {
        if (hComm != INVALID_HANDLE_VALUE) {
            CloseHandle(hComm);
            hComm = INVALID_HANDLE_VALUE;
        }
    }

private:
    HANDLE hComm;
};

// ======================= MAVLINK UTILS =======================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline int16_t clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// 0..100 (%) -> 0..1000 (MANUAL_CONTROL thrust field)
static inline int16_t thrustPercentToAxis(float pct) {
    pct = clampf(pct, 0.0f, 100.0f);
    return (int16_t)lround(pct * 10.0f);
}

// ======================= MAVLINK SERIAL + UDP MIRROR =======================
class MavlinkBridge {
public:
    bool start(const char* portName, DWORD baudRate, const char* udpIp, uint16_t udpPort) {
        if (!serial.open(portName, baudRate)) return false;
        if (!udp.start(udpIp, udpPort)) {
            cerr << "UDP mirror start failed\n";
            serial.close();
            return false;
        }

        running.store(true);

        // RX thread: read bytes from serial, parse MAVLink, mirror to UDP
        rxThread = std::thread(&MavlinkBridge::rxLoop, this);

        // Heartbeat thread: send GCS heartbeat
        hbThread = std::thread(&MavlinkBridge::heartbeatLoop, this);

        udpRxThread = std::thread(&MavlinkBridge::udpToSerialLoop, this);

        return true;
    }

    void udpToSerialLoop() {
        uint8_t buf[2048];
        while (running.load()) {
            int n = udp.recvPacket(buf, (int)sizeof(buf));
            if (n > 0) {
                serial.writeBytes(buf, (size_t)n);  // forward MP commands to Pixhawk
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }


    void stop() {
        running.store(false);
        if (rxThread.joinable()) rxThread.join();
        if (hbThread.joinable()) hbThread.join();
        udp.stop();
        serial.close();
    }

    // Send a MAVLink message to serial AND mirror it to UDP (so MP sees it)
    bool sendMessage(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        // 1) serial -> drone
        bool ok = serial.writeBytes(buf, len);

        // 2) mirror -> Mission Planner
        udp.sendPacket(buf, len);

        return ok;
    }

    // Send MANUAL_CONTROL (mirrored to UDP too)
    void sendManualControl(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons = 0) {
        mavlink_manual_control_t payload{};
        payload.target = VEHICLE_SYSID;
        payload.x = x;
        payload.y = y;
        payload.z = z;
        payload.r = r;

        payload.buttons  = buttons;
        payload.buttons2 = 0;

        // Extensions: keep disabled (safe)
        payload.enabled_extensions = 0;
        payload.s = 0;
        payload.t = 0;
        payload.aux1 = payload.aux2 = payload.aux3 = payload.aux4 = payload.aux5 = payload.aux6 = 0;

        mavlink_message_t msg{};
        mavlink_msg_manual_control_encode(SYS_ID, COMP_ID, &msg, &payload);
        sendMessage(msg);
    }

    // Optional: know whether we see vehicle heartbeats (useful for debugging)
    bool hasSeenVehicleHeartbeat() const { return seenVehicleHeartbeat.load(); }

private:
    void heartbeatLoop() {
        while (running.load()) {
            mavlink_message_t hb{};
            mavlink_msg_heartbeat_pack(
                SYS_ID,
                COMP_ID,
                &hb,
                MAV_TYPE_GCS,
                MAV_AUTOPILOT_INVALID,
                0,
                0,
                MAV_STATE_ACTIVE
            );
            sendMessage(hb);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void rxLoop() {
        mavlink_message_t msg{};
        mavlink_status_t status{};

        uint8_t buf[512];

        while (running.load()) {
            size_t n = serial.readBytes(buf, sizeof(buf));
            if (n == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            for (size_t i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    // We received a complete MAVLink message from the Pixhawk.
                    // Mirror it to Mission Planner over UDP:
                    udp.sendMessage(msg);

                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        // Mark that we are seeing a vehicle heartbeat from SYSID=1
                        if (msg.sysid == VEHICLE_SYSID) {
                            seenVehicleHeartbeat.store(true);
                        }
                    }
                }
            }
        }
    }

    SerialPort serial;
    UdpMirror udp;

    std::atomic<bool> running{false};
    std::atomic<bool> seenVehicleHeartbeat{false};

    std::thread rxThread;
    std::thread hbThread;
    std::thread udpRxThread;
};

// ======================= MAIN =======================
int main() {
    // Bridge: COM6 <-> Pixhawk, and mirror everything to UDP:14550 for Mission Planner
    MavlinkBridge mav;
    if (!mav.start(SERIAL_PORT_NAME, SERIAL_BAUDRATE, UDP_IP, UDP_PORT)) {
        cerr << "Failed to start MAVLink bridge on " << SERIAL_PORT_NAME << "\n";
        return -1;
    }

    // Leap Motion connection
    LEAP_CONNECTION connection = nullptr;
    if (LeapCreateConnection(nullptr, &connection) != eLeapRS_Success) {
        cerr << "Failed to create Leap connection.\n";
        mav.stop();
        return -1;
    }

    if (LeapOpenConnection(connection) != eLeapRS_Success) {
        cerr << "Failed to open Leap connection.\n";
        LeapDestroyConnection(connection);
        mav.stop();
        return -1;
    }

    cout << "Leap Motion connected.\n";
    cout << "MAVLink bridge running: Serial(" << SERIAL_PORT_NAME << " @57600) -> Pixhawk, mirrored to UDP 127.0.0.1:14550\n";
    cout << "IMPORTANT: Mission Planner must connect via UDP (NOT COM6).\n";

    LEAP_CONNECTION_MESSAGE msg;
    auto lastPrint = chrono::steady_clock::now();
    auto lastSend  = chrono::steady_clock::now();

    float thrust_pct = 50.0f;      // 0..100
    const float thrust_min  = 20.0f;
    const float thrust_max  = 90.0f;

    while (true) {
        if (LeapPollConnection(connection, 10, &msg) == eLeapRS_Success &&
            msg.type == eLeapEventType_Tracking) {

            const LEAP_TRACKING_EVENT* frame = msg.tracking_event;

            const LEAP_HAND* mainHand = nullptr;
            const LEAP_HAND* altHand  = nullptr;

            for (uint32_t i = 0; i < frame->nHands; i++) {
                const LEAP_HAND& hand = frame->pHands[i];
                if (hand.type == eLeapHandType_Right) mainHand = &hand;
                else                                  altHand  = &hand;
            }

            if (mainHand) {
                float grab = mainHand->grab_strength;
                LEAP_VECTOR dir  = mainHand->palm.direction;
                LEAP_VECTOR norm = mainHand->palm.normal;

                float pitch = atan2f(dir.y, -dir.z) * RAD_TO_DEG;
                float roll  = atan2f(norm.x, norm.y) * RAD_TO_DEG;
                float yaw   = atan2f(dir.x, dir.z) * RAD_TO_DEG;

                // Altitude via secondary hand: open -> higher thrust, fist -> lower thrust
                if (altHand) {
                    float grab2 = altHand->grab_strength; // 0=open, 1=fist
                    const float step_size = 0.05f;

                    float inv = 1.0f - grab2;
                    int step_index = static_cast<int>(inv / step_size); // 0..20
                    float thrust_from_step =
                        thrust_min + step_index * ((thrust_max - thrust_min) * step_size);

                    thrust_pct = clampf(thrust_from_step, thrust_min, thrust_max);
                }

                // Send MANUAL_CONTROL @ ~20 Hz
                auto now = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::milliseconds>(now - lastSend).count() >= 30) {
                    int16_t x = 0, y = 0, r = 0;

                    // Dead-man: reset thrust to neutral and neutral sticks
                    if (grab > 0.9f) {
                        thrust_pct = 50.0f;
                        x = 0; y = 0; r = 0;
                    } else {
                        if (pitch < -4.5f)      x = +400;   // forward
                        else if (pitch > 4.5f)  x = -400;   // backward

                        if (roll < 0 && roll > -175)        y = +400;   // right
                        else if (roll > 1 && roll < 175)    y = -400;   // left

                        // yaw optional
                        (void)yaw;
                    }

                    int16_t z = thrustPercentToAxis(thrust_pct);

                    // Safety clamps
                    x = clampi(x, -1000, 1000);
                    y = clampi(y, -1000, 1000);
                    r = clampi(r, -1000, 1000);
                    z = clampi(z, 0, 1000);

                    mav.sendManualControl(x, y, z, r);
                    lastSend = now;
                }

                // Console print
                auto nowPrint = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::milliseconds>(nowPrint - lastPrint).count() > 50) {
                    cout << "Pitch: " << pitch
                         << " | Roll: "  << roll
                         << " | Thrust%: " << thrust_pct
                         << (grab > 0.9f ? "  [DEAD-MAN]\n" : "\n");

                    if (grab > 0.9f) {
                        cout << "MAIN HAND CLENCHED -> DRONE HOVER MODE\n";
                    } else {
                        if (pitch > 4.5)  cout << "BACKWARD\n";
                        else if (pitch < -4.5) cout << "FORWARD\n";
                        if (roll > 1 && roll < 175)      cout << "LEFT\n";
                        else if (roll < 0 && roll > -175) cout << "RIGHT\n";
                    }

                    if (!mav.hasSeenVehicleHeartbeat()) {
                        cout << "WARNING: not seeing vehicle heartbeat yet. Check TELEM1 wiring/params/baud.\n";
                    }

                    lastPrint = nowPrint;
                }
            }
        }
    }

    // Not reached normally
    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);
    mav.stop();
    return 0;
}
