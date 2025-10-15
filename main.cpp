#include "LeapC.h"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;
int main() {
    LEAP_CONNECTION connection = nullptr;
    eLeapRS result = LeapCreateConnection(nullptr, &connection);
    if (result != eLeapRS_Success) {
        cerr << "Failed to create connection." << endl;
        return -1;
    }

    LeapOpenConnection(connection);
    cout << "Leap Motion connection opened. Move your hand..." << endl;

    LEAP_CONNECTION_MESSAGE msg;  // <-- structure
    while (true) {
        result = LeapPollConnection(connection, 1000, &msg);  // <-- use '&msg'
        if (result == eLeapRS_Success && msg.type == eLeapEventType_Tracking) {
            auto* frame = msg.tracking_event;
            cout << "Frame " << frame->info.frame_id
                      << " | Hands detected: " << frame->nHands << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);
    return 0;
}
