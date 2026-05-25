#pragma once
#include <utils/net.h>
#include <utils/flog.h>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>

// TCPSender: minimal, robust TCP *client* used to push decoded AIS records to
// an external collector (e.g. the Django map backend).
//
// Design constraints:
//   - send() is called from the DSP/decoder thread and MUST NOT block on
//     network I/O. It only enqueues a line and wakes the worker.
//   - A dedicated worker thread owns the socket, handles (re)connection with
//     back-off, and drains the queue. The socket is never touched from another
//     thread, so no socket-level locking is needed.
//   - net::connect() throws on failure (unknown host / refused), so the worker
//     wraps it in try/catch and retries.
//
// Records are newline-terminated; the collector reads one record per line.
class TCPSender {
public:
    ~TCPSender() { stop(); }

    // Set/replace the destination. Safe to call at any time; if running and the
    // host/port actually changed, the current connection is dropped so the
    // worker reconnects to the new endpoint.
    void configure(const std::string& host, int port) {
        std::lock_guard<std::mutex> lck(mtx);
        if (host == this->host && port == this->port) { return; }
        this->host = host;
        this->port = port;
        endpointChanged = true;
        cnd.notify_all();
    }

    void start() {
        if (running) { return; }
        running = true;
        workerThread = std::thread(&TCPSender::worker, this);
    }

    void stop() {
        if (!running) { return; }
        running = false;
        cnd.notify_all();
        if (workerThread.joinable()) { workerThread.join(); }
        // Drop any pending data so a later start() begins clean.
        std::lock_guard<std::mutex> lck(mtx);
        queue.clear();
    }

    // Non-blocking. Enqueues a record (a '\n' is appended if missing).
    void send(const std::string& line) {
        std::lock_guard<std::mutex> lck(mtx);
        if (queue.size() >= MAX_QUEUE) { queue.pop_front(); droppedCount++; }
        if (!line.empty() && line.back() == '\n') { queue.push_back(line); }
        else { queue.push_back(line + "\n"); }
        cnd.notify_all();
    }

    bool isConnected() const { return connected; }
    uint64_t getSentCount() const { return sentCount; }
    uint64_t getDroppedCount() const { return droppedCount; }

private:
    static constexpr size_t MAX_QUEUE = 2048;        // ~ generous for AIS rates
    static constexpr int RECONNECT_DELAY_MS = 3000;  // back-off between attempts
    static constexpr int SEND_RETRY_MS = 20;         // pause when send buffer is full

    void worker() {
        std::shared_ptr<net::Socket> sock;
        while (running) {
            // --- Ensure we have a live connection -------------------------
            std::string h; int p;
            {
                std::lock_guard<std::mutex> lck(mtx);
                h = host; p = port;
                endpointChanged = false;
            }

            if (h.empty() || p <= 0) {
                std::unique_lock<std::mutex> lck(mtx);
                cnd.wait_for(lck, std::chrono::milliseconds(RECONNECT_DELAY_MS),
                             [&] { return !running || endpointChanged; });
                continue;
            }

            try {
                sock = net::connect(h, p);
            }
            catch (const std::exception& e) {
                sock.reset();
                connected = false;
                flog::warn("[AIS][TCP] connect to {}:{} failed: {} (retrying)", h, p, e.what());
                std::unique_lock<std::mutex> lck(mtx);
                cnd.wait_for(lck, std::chrono::milliseconds(RECONNECT_DELAY_MS),
                             [&] { return !running || endpointChanged; });
                continue;
            }

            if (!sock || !sock->isOpen()) {
                connected = false;
                std::unique_lock<std::mutex> lck(mtx);
                cnd.wait_for(lck, std::chrono::milliseconds(RECONNECT_DELAY_MS),
                             [&] { return !running || endpointChanged; });
                continue;
            }

            connected = true;
            flog::info("[AIS][TCP] connected to {}:{}", h, p);

            // --- Drain the queue while the link stays up ------------------
            while (running && sock->isOpen()) {
                std::string line;
                {
                    std::unique_lock<std::mutex> lck(mtx);
                    cnd.wait(lck, [&] { return !running || endpointChanged || !queue.empty(); });
                    if (!running || endpointChanged) { break; }
                    line = std::move(queue.front());
                    queue.pop_front();
                }

                int n = sock->sendstr(line);
                if (n <= 0) {
                    // The SDR++ socket is non-blocking. A return of <= 0 can mean
                    // either (a) the send buffer is momentarily full (EWOULDBLOCK)
                    // — the socket stays open — or (b) a real error/peer close —
                    // the socket gets closed by Socket::send(). We use isOpen() to
                    // tell them apart: if still open, it was just back-pressure, so
                    // we re-queue the line and retry after a short pause instead of
                    // tearing down the connection (which previously caused a
                    // reconnect storm, especially with two instances feeding a slow
                    // reader like `nc`).
                    {
                        std::lock_guard<std::mutex> lck(mtx);
                        queue.push_front(line);
                    }
                    if (sock->isOpen()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_MS));
                        continue;            // retry same line, keep the connection
                    }
                    break;                   // real disconnect: reconnect
                }
                if ((size_t)n < line.size()) {
                    // Partial write on the non-blocking socket: re-queue the
                    // unsent remainder so it goes out next, preserving framing.
                    std::lock_guard<std::mutex> lck(mtx);
                    queue.push_front(line.substr(n));
                    continue;
                }
                sentCount++;
            }

            connected = false;
            if (sock) { sock->close(); sock.reset(); }
        }

        if (sock) { sock->close(); }
        connected = false;
    }

    std::thread workerThread;
    std::atomic<bool> running{ false };
    std::atomic<bool> connected{ false };
    std::atomic<uint64_t> sentCount{ 0 };
    std::atomic<uint64_t> droppedCount{ 0 };

    std::mutex mtx;
    std::condition_variable cnd;
    std::deque<std::string> queue;

    std::string host = "127.0.0.1";
    int port = 0;
    bool endpointChanged = false;
};
