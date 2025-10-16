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
    cout << "Leap Motion connected. Use one or two hands to control the drone..." << endl;

    LEAP_CONNECTION_MESSAGE msg;
    auto lastPrint = chrono::steady_clock::now();

    while (true) {
        if (LeapPollConnection(connection, 10, &msg) == eLeapRS_Success &&
            msg.type == eLeapEventType_Tracking) {

            const LEAP_TRACKING_EVENT* frame = msg.tracking_event;

            const LEAP_HAND* mainHand = nullptr;
            const LEAP_HAND* altHand = nullptr;

            // Identify hands
            for (uint32_t i = 0; i < frame->nHands; i++) {
                const LEAP_HAND& hand = frame->pHands[i];
                if (hand.type == eLeapHandType_Right)
                    mainHand = &hand;   // right hand = control
                else
                    altHand = &hand;    // left hand = altitude
            }

            // Process only if we have at least one hand
            if (mainHand) {
                float grab = mainHand->grab_strength;
                LEAP_VECTOR dir = mainHand->palm.direction;
                LEAP_VECTOR norm = mainHand->palm.normal;

                float pitch = atan2f(dir.y, -dir.z) * RAD_TO_DEG;
                float roll  = atan2f(norm.x, norm.y) * RAD_TO_DEG;
                float yaw   = atan2f(dir.x, dir.z) * RAD_TO_DEG;

                auto now = chrono::steady_clock::now();
                if (chrono::duration_cast<chrono::milliseconds>(now - lastPrint).count() > 200) {

                    if (grab > 0.8f) {
                        cout << "MAIN HAND CLENCHED -> DRONE HOVER MODE" << endl;
                    } else {
                        /*cout << "Pitch: " << pitch
                             << " | Roll: " << roll
                             << " | Yaw: " << yaw << endl;*/

                        if (pitch > 15) cout << "BACKWARD" << endl;
                        else if (pitch < -15) cout << "FORWARD" << endl;
                        if (roll > 15) cout << "LEFT" << endl;
                        else if (roll < -15) cout << "RIGHT" << endl;
                    }

                    // ✋ Secondary hand controls altitude
                    if (altHand) {
                        float grab2 = altHand->grab_strength;

                        if (grab2 < 0.3f)       cout << "SECOND HAND OPEN -> ASCEND" << endl;
                        else if (grab2 > 0.8f)  cout << "️ SECOND HAND FIST -> DESCEND" << endl;
                        else                    cout << "SECOND HAND RELAXED -> HOLD ALTITUDE" << endl;
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
