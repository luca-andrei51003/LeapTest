#include "LeapC.h"
#include <iostream>
#include <cmath>
#include <chrono>

#define RAD_TO_DEG 57.2958f
using namespace std;

int main() {
    LEAP_CONNECTION connection = nullptr;
    if (LeapCreateConnection(nullptr, &connection) != eLeapRS_Success) {
        cerr << "Failed to create connection." << endl;
        return -1;
    }

    LeapOpenConnection(connection);
    cout << "Leap Motion connected. Move your hand..." << endl;

    LEAP_CONNECTION_MESSAGE msg;
    auto lastPrint = chrono::steady_clock::now();

    while (true) {
        if (LeapPollConnection(connection, 10, &msg) == eLeapRS_Success &&
            msg.type == eLeapEventType_Tracking) {

            const LEAP_TRACKING_EVENT* frame = msg.tracking_event;

            const LEAP_HAND* mainHand = nullptr;
            const LEAP_HAND* altHand = nullptr;

            if (frame->nHands > 0) {
                const LEAP_HAND& hand = frame->pHands[0];

                float grab = hand.grab_strength;
                LEAP_VECTOR dir = hand.palm.direction;
                LEAP_VECTOR norm = hand.palm.normal;
                LEAP_VECTOR pos = hand.palm.position;

                // Compute angles
                float pitch = atan2f(dir.y, -dir.z) * RAD_TO_DEG;
                float roll  = atan2f(norm.x, norm.y) * RAD_TO_DEG;
                float yaw   = atan2f(dir.x, dir.z) * RAD_TO_DEG;

                auto now = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::milliseconds>(now - lastPrint).count() > 20) {

                    if (grab > 0.8f) {
                        cout << "Fist clenched DRONE HOLD / HOVER MODE" << endl;
                    } else {
                        cout << "Pitch: " << pitch
                             << " | Roll: " << roll
                             << " | Yaw: " << yaw << endl;

                        // Directions
                        if (pitch > 15) cout << "FORWARD" << endl;
                        else if (pitch < -15) cout << "BACKWARD" << endl;
                        if (roll > 15) cout << "RIGHT" << endl;
                        else if (roll < -15) cout << "LEFT" << endl;

                        // Altitude control
                        if (pos.y > 220) cout << "Ascend" << endl;
                        else if (pos.y < 150) cout << "Descend" << endl;
                        else cout << "Maintain altitude" << endl;
                    }

                    lastPrint = now;
                }
            }
        }
    }

    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);
    return 0;
}
