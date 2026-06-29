#pragma once
#include <atomic>
#include <cstdint>

// Shared, lock-free snapshot of everything the control loop knows.
// The control loop (runControlLoop) writes; the GUI reads. All fields are
// std::atomic so no mutex is needed for these scalar values.
struct TelemetryState {
    // ---- Link / connection status ----
    std::atomic<bool> serialOpen{false};       // SiK radio serial port opened
    std::atomic<bool> leapConnected{false};    // Leap connection opened
    std::atomic<bool> vehicleHeartbeat{false}; // seen a HEARTBEAT from the vehicle

    // ---- Hand presence ----
    std::atomic<bool> mainHandPresent{false};  // right hand (attitude)
    std::atomic<bool> altHandPresent{false};   // left hand (thrust)

    // ---- Attitude (degrees) from the right hand ----
    std::atomic<float> pitch{0.0f};
    std::atomic<float> roll{0.0f};
    std::atomic<float> yaw{0.0f};

    // ---- Thrust ----
    std::atomic<float> thrustPct{0.0f};        // 0..100

    // ---- Raw gesture inputs (handy for debugging/tuning) ----
    std::atomic<float> mainGrab{0.0f};         // right-hand grab strength 0..1
    std::atomic<float> altGrabAngle{0.0f};     // left-hand grab angle (0 open .. ~pi fist)

    // ---- Last MANUAL_CONTROL actually sent (x/y/r: -1000..1000, z: 0..1000) ----
    std::atomic<int> sentX{0}; // pitch
    std::atomic<int> sentY{0}; // roll
    std::atomic<int> sentZ{0}; // thrust
    std::atomic<int> sentR{0}; // yaw

    // ---- Flags ----
    std::atomic<bool> deadMan{false};          // right hand clenched -> hover

    // ---- Health counters ----
    std::atomic<uint64_t> framesProcessed{0};  // Leap tracking frames handled
    std::atomic<uint64_t> packetsSent{0};      // MANUAL_CONTROL packets sent
    std::atomic<long long> lastFrameMs{0};     // steady-clock ms of last tracking frame
};

// Runs the Leap -> MAVLink control logic until `running` becomes false.
// Continuously updates `state`. Pass verbose=true to also print to the console.
void runControlLoop(TelemetryState& state, std::atomic<bool>& running, bool verbose);
