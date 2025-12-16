#include "LeapC.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <windows.h>

extern "C" {
    #include "third_party/mavlink/include/mavlink/v2.0/common/mavlink.h"
}

#define RAD_TO_DEG 57.2958f
using namespace std;

// ======= CONFIG: CHANGE THIS TO YOUR PIXHAWK COM PORT =======
static const char* SERIAL_PORT_NAME = "COM6";  // <-- change to your COM number
static const DWORD SERIAL_BAUDRATE = CBR_57600;

static const uint8_t SYS_ID        = 255; // our app
static const uint8_t COMP_ID       = MAV_COMP_ID_MISSIONPLANNER;
static const uint8_t VEHICLE_SYSID = 1;   // Pixhawk system ID (usually 1)

// ======= SIMPLE SERIAL WRAPPER (WRITE-ONLY) =======
class SerialPort {
public:
    SerialPort() : hComm(INVALID_HANDLE_VALUE) {}
    ~SerialPort() { close(); }

    bool open(const char* portName, DWORD baudRate) {
        hComm = CreateFileA(
            portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );
        if (hComm == INVALID_HANDLE_VALUE) {
            cerr << "Failed to open serial port " << portName << endl;
            return false;
        }

        // Configure timeouts
        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout         = 50;
        timeouts.ReadTotalTimeoutMultiplier  = 10;
        timeouts.ReadTotalTimeoutConstant    = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant   = 50;
        SetCommTimeouts(hComm, &timeouts);

        // Configure serial params: 8N1
        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(hComm, &dcb)) {
            cerr << "GetCommState failed\n";
            close();
            return false;
        }

        dcb.BaudRate = baudRate;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl  = DTR_CONTROL_DISABLE;
        dcb.fRtsControl  = RTS_CONTROL_DISABLE;

        if (!SetCommState(hComm, &dcb)) {
            cerr << "SetCommState failed\n";
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
            cerr << "Serial WriteFile failed\n";
            return false;
        }
        return (bytesWritten == len);
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

// ======= MAVLINK HELPERS =======
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline int16_t clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// 0..100 (%) → 0..1000 (MANUAL_CONTROL thrust)
static inline int16_t thrustPercentToAxis(float pct) {
    pct = clampf(pct, 0.0f, 100.0f);
    return (int16_t)lround(pct * 10.0f);
}

class MavlinkSerial {
public:
    bool start(const char* portName, DWORD baudRate) {
        if (!serial.open(portName, baudRate)) return false;
        running = true;
        hbThread = std::thread(&MavlinkSerial::heartbeatLoop, this);
        return true;
    }

    void stop() {
        running = false;
        if (hbThread.joinable()) hbThread.join();
        serial.close();
    }

    bool sendMessage(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        return serial.writeBytes(buf, len);
    }

    void sendManualControl(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons = 0) {
        mavlink_manual_control_t payload{};

        payload.x = x;
        payload.y = y;
        payload.z = z;
        payload.r = r;

        payload.buttons  = buttons;   // low 16 joystick buttons
        payload.buttons2 = 0;         // you can ignore extended buttons

        payload.target = VEHICLE_SYSID;

        // These must be zero unless explicitly used
        payload.enabled_extensions = 0;
        payload.s = 0;
        payload.t = 0;
        payload.aux1 = 0;
        payload.aux2 = 0;
        payload.aux3 = 0;
        payload.aux4 = 0;
        payload.aux5 = 0;
        payload.aux6 = 0;

        mavlink_message_t msg{};
        mavlink_msg_manual_control_encode(
            SYS_ID,        // system ID of your GCS/app
            COMP_ID,       // component ID of your app
            &msg,
            &payload
        );

        sendMessage(msg); // your existing function that writes to serial/UDP
    }




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
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0,
                MAV_STATE_ACTIVE
            );
            sendMessage(hb);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    SerialPort serial;
    std::atomic<bool> running{false};
    std::thread hbThread;
};

// ======= MAIN =======

int main() {
    // 1) Start MAVLink over serial
    MavlinkSerial mav;
    if (!mav.start(SERIAL_PORT_NAME, SERIAL_BAUDRATE)) {
        cerr << "Failed to start MAVLink on " << SERIAL_PORT_NAME << endl;
        return -1;
    }

    // 2) Leap Motion connection
    LEAP_CONNECTION connection = nullptr;
    if (LeapCreateConnection(nullptr, &connection) != eLeapRS_Success) {
        cerr << "Failed to create Leap connection." << endl;
        mav.stop();
        return -1;
    }

    LeapOpenConnection(connection);
    cout << "Leap Motion connected. Use one or two hands to control the drone..." << endl;

    LEAP_CONNECTION_MESSAGE msg;
    auto lastPrint = chrono::steady_clock::now();
    auto lastSend  = chrono::steady_clock::now();

    // Thrust controlled by secondary hand
    float thrust_pct = 50.0f;      // 0..100
    const float thrust_step = 0.2f;
    const float thrust_min  = 20.0f;
    const float thrust_max  = 90.0f;  // avoid 100% by default

    while (true) {
        if (LeapPollConnection(connection, 10, &msg) == eLeapRS_Success &&
            msg.type == eLeapEventType_Tracking) {

            const LEAP_TRACKING_EVENT* frame = msg.tracking_event;

            const LEAP_HAND* mainHand = nullptr;
            const LEAP_HAND* altHand  = nullptr;

            // Identify hands
            for (uint32_t i = 0; i < frame->nHands; i++) {
                const LEAP_HAND& hand = frame->pHands[i];
                if (hand.type == eLeapHandType_Right)
                    mainHand = &hand;   // right hand = control
                else
                    altHand = &hand;    // left hand = altitude
            }

            if (mainHand) {
                float grab = mainHand->grab_strength;
                LEAP_VECTOR dir  = mainHand->palm.direction;
                LEAP_VECTOR norm = mainHand->palm.normal;

                float pitch = atan2f(dir.y, -dir.z) * RAD_TO_DEG;
                float roll  = atan2f(norm.x, norm.y) * RAD_TO_DEG;
                float yaw   = atan2f(dir.x, dir.z) * RAD_TO_DEG;

                // Altitude via secondary hand
                // Altitude via secondary hand (open = ascend, fist = descend)
                if (altHand) {
                    float grab2 = altHand->grab_strength;   // 0.0 = open, 1.0 = fist

                    const float min_thrust = 20.0f;         // %
                    const float max_thrust = 90.0f;         // %
                    const float step_size  = 0.05f;         // grab-strength step

                    // Invert so 0.0 (open) -> highest thrust, 1.0 (fist) -> lowest thrust
                    float inv = 1.0f - grab2;

                    // Quantize in 0.05 steps: 0, 0.05, 0.10, ... , 1.0  →  0..20 steps
                    int step_index = static_cast<int>(inv / step_size);   // 0..20

                    // Each step adds 70 / 20 = 3.5 % thrust across the 20→90 range
                    float thrust_from_step =
                        min_thrust + step_index * ((max_thrust - min_thrust) * step_size); // 3.5% per step

                    thrust_pct = clampf(thrust_from_step, min_thrust, max_thrust);
                }



                // Send MANUAL_CONTROL @ ~20 Hz
                auto now = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::milliseconds>(now - lastSend).count() >= 50) {

                    int16_t x = 0, y = 0, r = 0;

                    // DEAD-MAN = return thrust to 50%
                    if (grab > 0.9f) {
                        thrust_pct = 50.0f;     // <── reset to neutral throttle
                        x = 0;
                        y = 0;
                        r = 0;
                    }
                    else {
                        // Direction controls
                        if (pitch < -4.5f)      x = +400;   // FORWARD
                        else if (pitch > 4.5f)  x = -400;   // BACKWARD

                        if (roll < 0 && roll > -175)        y = +400;   // RIGHT
                        else if (roll > 1 && roll < 175)    y = -400;   // LEFT

                        // Optional yaw:
                        // r = map_angle_to_axis(yaw);
                    }

                    // Convert thrust% (20..90 normally, but 50 in dead-man)
                    int16_t z = thrustPercentToAxis(thrust_pct);

                    // Safety clamps
                    x = clampi(x, -1000, 1000);
                    y = clampi(y, -1000, 1000);
                    r = clampi(r, -1000, 1000);
                    z = clampi(z, 0,      1000);

                    mav.sendManualControl(x, y, z, r);
                    lastSend = now;
                }


                // Console print
                auto nowPrint = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::milliseconds>(nowPrint - lastPrint).count() > 200) {
                    cout << "Pitch: " << pitch
                         << " | Roll: "  << roll
                         << " | Yaw: "   << yaw
                         << " | Thrust%: " << thrust_pct
                         << (grab > 0.9f ? "  [HOLD]\n" : "\n");

                    if (grab > 0.9f) {
                        cout << "MAIN HAND CLENCHED -> DRONE HOVER MODE\n";
                    } else {
                        if (pitch > 4.5)  cout << "BACKWARD\n";
                        else if (pitch < -4.5) cout << "FORWARD\n";
                        if (roll > 1 && roll < 175)      cout << "LEFT\n";
                        else if (roll < 0 && roll > -175) cout << "RIGHT\n";
                    }

                    if (altHand) {
                        float grab2 = altHand->grab_strength;
                        if (grab2 < 0.10f)      cout << "SECOND HAND OPEN -> ASCEND\n";
                        else if (grab2 > 0.95f) cout << "SECOND HAND FIST -> DESCEND\n";
                        else                    cout << "SECOND HAND RELAXED -> HOLD ALTITUDE\n";
                    }

                    cout << "\n";
                    lastPrint = nowPrint;
                }
            }
        }
    }

    mav.stop();
    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);
    return 0;
}
