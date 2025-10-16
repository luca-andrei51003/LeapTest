#include "LeapC.h"
#include <iostream>
#include <cmath>
#include <chrono>

#define RAD_TO_DEG 57.2958f
using namespace std;
int main() {
    LEAP_CONNECTION connection = nullptr;
    eLeapRS result = LeapCreateConnection(nullptr, &connection);
    if (result != eLeapRS_Success) {
        std::cerr << "Failed to create connection." << std::endl;
        return -1;
    }

    LeapOpenConnection(connection);
    std::cout << "Leap Motion connected. Move your hand..." << std::endl;

    LEAP_CONNECTION_MESSAGE msg;
    auto lastPrint = std::chrono::steady_clock::now();

    while (true) {
        result = LeapPollConnection(connection, 10, &msg);
        if (result == eLeapRS_Success && msg.type == eLeapEventType_Tracking) {
            const LEAP_TRACKING_EVENT* frame = msg.tracking_event;

            if (frame->nHands > 0) {
                const LEAP_HAND& hand = frame->pHands[0];

                // Get vectors
                LEAP_VECTOR dir = hand.palm.direction;
                LEAP_VECTOR norm = hand.palm.normal;

                // Compute orientation
                float pitch = atan2f(dir.y, -dir.z) * RAD_TO_DEG;     // Forward/back tilt
                float roll  = atan2f(norm.x, norm.y) * RAD_TO_DEG;    // Left/right tilt
                float yaw   = atan2f(dir.x, dir.z) * RAD_TO_DEG;
                float fist;

                // Print once every ~0.2 sec
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPrint).count() > 200) {
                    std::cout << "Pitch: " << pitch << "°"
                              << " | Roll: " << roll << "°"
                              << " | Yaw: " << yaw << "°" << std::endl;
                    //cout<<"Grab Strength is: "<<fist<<endl;
                    lastPrint = now;
                }

                // Example thresholds for drone direction
                /*
                if (pitch > 15) std::cout << "Move BACKWARD\n";
                else if (pitch < -15) std::cout << "Move FORWARD\n";
                if (roll > 15) std::cout << "Move LEFT\n";
                else if (roll < -15) std::cout << "Move RIGHT\n";*/
            }
        }
    }

    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);
    return 0;
}
