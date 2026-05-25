// Exercises the real TCPSender + SDR++ net:: stack against a live local server.
// Verifies: connect, reconnect-on-failure (server comes up late), queueing,
// and that lines are delivered intact.
#include "tcp_sender.h"
#include <cstdio>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    int port = (argc > 1) ? atoi(argv[1]) : 19999;

    TCPSender s;
    s.configure("127.0.0.1", port);
    s.start();

    // Wait until connected (server may start slightly after us, exercising reconnect).
    for (int i = 0; i < 50 && !s.isConnected(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("connected=%d\n", (int)s.isConnected());

    // Send a few JSON-ish records identical in shape to what the module emits.
    for (int i = 0; i < 5; i++) {
        char line[256];
        snprintf(line, sizeof(line),
            "{\"name\":\"TEST %d\",\"mmsi\":22700676%d,\"date\":\"2026-05-25\",\"time\":\"12:00:0%d\",\"lat\":43.295000,\"lon\":5.370000,\"type\":\"AIS\",\"speed\":10.0,\"info\":\"MMSI=22700676%d msg=1\"}",
            i, i, i, i);
        s.send(line);
    }

    // Give the worker time to flush.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("sent=%llu dropped=%llu\n", (unsigned long long)s.getSentCount(), (unsigned long long)s.getDroppedCount());

    s.stop();
    return 0;
}
