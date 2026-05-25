#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <gui/main_window.h>
#include <gui/tuner.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <utils/optionlist.h>
#include <dsp/sink/handler_sink.h>

#include "dsp.h"
#include "ais/decoder.h"
#include "tcp_sender.h"

#include <mutex>
#include <map>
#include <vector>
#include <ctime>
#include <cstdio>
#include <algorithm>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

// AIS physical layer: 9600 baud GMSK on two VHF channels.
//   AIS 1 (87B): 161.975 MHz   |   AIS 2 (88B): 162.025 MHz
#define AIS_BAUDRATE    9600
#define AIS_SAMPLERATE  (AIS_BAUDRATE * 5)   // 48 kHz, 5 samples/symbol
#define AIS_BANDWIDTH   22000.0
#define AIS_CH1_FREQ    161975000.0
#define AIS_CH2_FREQ    162025000.0

SDRPP_MOD_INFO{
    /* Name:            */ "ais_decoder",
    /* Description:     */ "AIS (marine ship tracking) decoder with TCP output",
    /* Author:          */ "SDR++ community",
    /* Version:         */ 1, 0, 0,
    /* Max instances    */ -1
};

ConfigManager config;

// ---------------------------------------------------------------------------
// Small lookups for human-readable "info diverse".
// ---------------------------------------------------------------------------
static const char* navStatusText(int s) {
    static const char* tbl[] = {
        "Under way (engine)", "At anchor", "Not under command", "Restricted manoeuvrability",
        "Constrained by draught", "Moored", "Aground", "Fishing", "Under way (sailing)",
        "Reserved(9)", "Reserved(10)", "Towing astern", "Pushing ahead",
        "Reserved(13)", "AIS-SART", "Undefined"
    };
    if (s < 0 || s > 15) { return ""; }
    return tbl[s];
}

static std::string shipTypeText(int t) {
    if (t <= 0) { return ""; }
    if (t >= 20 && t <= 29) { return "Wing in ground"; }
    if (t == 30) { return "Fishing"; }
    if (t == 31 || t == 32) { return "Towing"; }
    if (t == 33) { return "Dredging"; }
    if (t == 34) { return "Diving"; }
    if (t == 35) { return "Military"; }
    if (t == 36) { return "Sailing"; }
    if (t == 37) { return "Pleasure craft"; }
    if (t >= 40 && t <= 49) { return "High speed craft"; }
    if (t == 50) { return "Pilot vessel"; }
    if (t == 51) { return "Search and rescue"; }
    if (t == 52) { return "Tug"; }
    if (t == 53) { return "Port tender"; }
    if (t == 55) { return "Law enforcement"; }
    if (t == 58) { return "Medical transport"; }
    if (t >= 60 && t <= 69) { return "Passenger"; }
    if (t >= 70 && t <= 79) { return "Cargo"; }
    if (t >= 80 && t <= 89) { return "Tanker"; }
    if (t >= 90 && t <= 99) { return "Other"; }
    return "";
}

// Escape a string for inclusion inside a JSON double-quoted value.
static std::string jsonEscape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:
            if ((unsigned char)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); out += b; }
            else { out += c; }
        }
    }
    return out;
}

// A vessel/contact as accumulated from one or more AIS messages.
struct Contact {
    uint32_t mmsi = 0;
    std::string name;        // filled by static msgs (5/19/21/24); "" until known
    bool hasPos = false;
    double lat = 0.0, lon = 0.0;
    bool hasSpeed = false;
    double sog = 0.0;
    bool hasCog = false;
    double cog = 0.0;
    int heading = -1;
    int navStatus = -1;
    int shipType = -1;
    int lastType = 0;        // last AIS message type seen
    uint64_t msgCount = 0;   // number of CRC-valid messages received for this MMSI
    time_t lastUpdate = 0;   // reception time (UTC, system clock)
};

class AISDecoderModule : public ModuleManager::Instance {
public:
    AISDecoderModule(std::string name) {
        this->name = name;

        // Restore config
        config.acquire();
        if (config.conf[name].contains("tcpHost")) { strncpy(hostBuf, config.conf[name]["tcpHost"].get<std::string>().c_str(), sizeof(hostBuf) - 1); }
        if (config.conf[name].contains("tcpPort")) { port = config.conf[name]["tcpPort"]; }
        if (config.conf[name].contains("tcpEnabled")) { tcpEnabled = config.conf[name]["tcpEnabled"]; }
        if (config.conf[name].contains("showContactsWindow")) { showContactsWindow = config.conf[name]["showContactsWindow"]; }
        config.release();

        // DSP front-end
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, AIS_BANDWIDTH, AIS_SAMPLERATE, AIS_BANDWIDTH, AIS_BANDWIDTH, true);
        vfo->setSnapInterval(1);

        dsp.init(vfo->output, AIS_SAMPLERATE, AIS_BAUDRATE);
        dataHandler.init(&dsp.out, _dataHandler, this);

        // Hook decoded-message callback
        decoder.onMessage = [this](const ais::Message& m) { this->onMessage(m); };

        // TCP output
        sender.configure(hostBuf, port);
        sender.start();

        // Start the DSP chain now if the module is enabled. This mirrors the
        // pager_decoder, which starts its decoder in the constructor. Without
        // this, the module reports enabled=true but never drains the VFO output
        // stream, so on Play the VFO buffer fills up and stalls the whole signal
        // path (the waterfall never starts). Starting here keeps the module in a
        // consistent "enabled and running" state from launch.
        if (enabled) {
            decoder.reset();
            dsp.start();
            dataHandler.start();
        }

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~AISDecoderModule() {
        gui::menu.removeEntry(name);
        if (enabled) {
            dsp.stop();
            dataHandler.stop();
            sigpath::vfoManager.deleteVFO(vfo);
        }
        sender.stop();
    }

    void postInit() {}

    void enable() {
        double bw = gui::waterfall.getBandwidth();
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, std::clamp<double>(0, -bw / 2.0, bw / 2.0), AIS_BANDWIDTH, AIS_SAMPLERATE, AIS_BANDWIDTH, AIS_BANDWIDTH, true);
        vfo->setSnapInterval(1);
        dsp.setInput(vfo->output);

        decoder.reset();
        dsp.start();
        dataHandler.start();
        enabled = true;
    }

    void disable() {
        dsp.stop();
        dataHandler.stop();
        sigpath::vfoManager.deleteVFO(vfo);
        enabled = false;
    }

    bool isEnabled() { return enabled; }

private:
    // --- DSP stream handlers (run in the sink threads) ---------------------
    static void _dataHandler(uint8_t* data, int count, void* ctx) {
        AISDecoderModule* _this = (AISDecoderModule*)ctx;
        _this->decoder.process(data, count);
    }

    // Called for every CRC-valid AIS message (decoder/handler thread).
    void onMessage(const ais::Message& msg) {
        time_t now = time(nullptr);

        std::lock_guard<std::mutex> lck(contactsMtx);

        Contact& c = contacts[msg.mmsi];
        c.mmsi = msg.mmsi;
        c.lastType = msg.type;
        c.msgCount++;
        c.lastUpdate = now;
        if (!msg.name.empty())          { c.name = msg.name; }
        if (msg.shipType >= 0)          { c.shipType = msg.shipType; }
        if (msg.navStatus >= 0)         { c.navStatus = msg.navStatus; }
        if (msg.hasSpeed)               { c.hasSpeed = true; c.sog = msg.sog; }
        if (msg.hasCog)                 { c.hasCog = true; c.cog = msg.cog; }
        if (msg.heading >= 0)           { c.heading = msg.heading; }
        if (msg.hasPosition)            { c.hasPos = true; c.lat = msg.lat; c.lon = msg.lon; }

        // Only emit a TCP record when we have coordinates to plot.
        if (msg.hasPosition && tcpEnabled) {
            emitRecord(c, now);
        }
    }

    // Build the JSON line and hand it to the TCP sender (non-blocking).
    // Schema (one record per line):
    //   name, mmsi, date, time (UTC), lat, lon, type ("AIS"),
    //   speed   : SOG in knots, or null   (kept for backward compatibility)
    //   sog     : SOG in knots, or null   (same value, explicit name)
    //   cog     : Course Over Ground in degrees, or null
    //   hdg     : true Heading in degrees, or null
    //   shiptype: ship type label, or null
    //   navstatus: navigational status label, or null
    //   msgtype : last AIS message type number
    //   count   : number of decoded messages for this MMSI
    //   info    : compact human-readable summary (kept for backward compatibility)
    void emitRecord(const Contact& c, time_t now) {
        struct tm tmv;
#ifdef _WIN32
        gmtime_s(&tmv, &now);
#else
        gmtime_r(&now, &tmv);
#endif
        char dateBuf[16], timeBuf[16];
        strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &tmv);
        strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &tmv);

        std::string objName = c.name.empty() ? ("MMSI:" + std::to_string(c.mmsi)) : c.name;

        // "info diverse": compact, human readable, but still machine-friendly.
        std::string info = "MMSI=" + std::to_string(c.mmsi) + " msg=" + std::to_string(c.lastType);
        if (c.hasCog)        { char b[32]; snprintf(b, sizeof(b), " COG=%.1f", c.cog);  info += b; }
        if (c.heading >= 0)  { info += " HDG=" + std::to_string(c.heading); }
        if (c.navStatus >= 0){ std::string ns = navStatusText(c.navStatus); if (!ns.empty()) { info += " nav=" + ns; } }
        if (c.shipType >= 0) { std::string st = shipTypeText(c.shipType);  if (!st.empty()) { info += " ship=" + st; } }

        char latBuf[24], lonBuf[24];
        snprintf(latBuf, sizeof(latBuf), "%.6f", c.lat);
        snprintf(lonBuf, sizeof(lonBuf), "%.6f", c.lon);

        // Numeric fields: emit the value or the literal null.
        std::string sogField = "null", cogField = "null", hdgField = "null";
        if (c.hasSpeed)     { char b[24]; snprintf(b, sizeof(b), "%.1f", c.sog); sogField = b; }
        if (c.hasCog)       { char b[24]; snprintf(b, sizeof(b), "%.1f", c.cog); cogField = b; }
        if (c.heading >= 0) { hdgField = std::to_string(c.heading); }

        // String label fields: emit a quoted string or null.
        std::string shipField = "null", navField = "null";
        if (c.shipType >= 0) { std::string st = shipTypeText(c.shipType); if (!st.empty()) { shipField = "\"" + jsonEscape(st) + "\""; } }
        if (c.navStatus >= 0){ std::string ns = navStatusText(c.navStatus); if (!ns.empty()) { navField = "\"" + jsonEscape(ns) + "\""; } }

        std::string json = "{";
        json += "\"name\":\""    + jsonEscape(objName) + "\",";
        json += "\"mmsi\":"      + std::to_string(c.mmsi) + ",";
        json += "\"date\":\""    + std::string(dateBuf) + "\",";
        json += "\"time\":\""    + std::string(timeBuf) + "\",";
        json += "\"lat\":"       + std::string(latBuf) + ",";
        json += "\"lon\":"       + std::string(lonBuf) + ",";
        json += "\"type\":\"AIS\",";
        json += "\"speed\":"     + sogField + ",";      // backward-compatible alias of sog
        json += "\"sog\":"       + sogField + ",";
        json += "\"cog\":"       + cogField + ",";
        json += "\"hdg\":"       + hdgField + ",";
        json += "\"shiptype\":"  + shipField + ",";
        json += "\"navstatus\":" + navField + ",";
        json += "\"msgtype\":"   + std::to_string(c.lastType) + ",";
        json += "\"count\":"     + std::to_string((unsigned long long)c.msgCount) + ",";
        json += "\"info\":\""    + jsonEscape(info) + "\"";
        json += "}";

        sender.send(json);
    }

    // --- GUI ---------------------------------------------------------------
    static void menuHandler(void* ctx) {
        AISDecoderModule* _this = (AISDecoderModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!_this->enabled) { style::beginDisabled(); }

        // Quick-tune to the two AIS channels.
        ImGui::TextUnformatted("Channel");
        if (ImGui::Button(("AIS 1 (161.975)##ais_ch1_" + _this->name).c_str(), ImVec2(menuWidth / 2.0f - 4, 0))) {
            tuner::normalTuning(_this->name, AIS_CH1_FREQ);
        }
        ImGui::SameLine();
        if (ImGui::Button(("AIS 2 (162.025)##ais_ch2_" + _this->name).c_str(), ImVec2(menuWidth / 2.0f - 4, 0))) {
            tuner::normalTuning(_this->name, AIS_CH2_FREQ);
        }

        if (!_this->enabled) { style::endDisabled(); }

        // --- TCP output settings (work even while disabled) ---
        ImGui::Separator();
        ImGui::TextUnformatted("TCP output");

        ImGui::LeftLabel("Host");
        ImGui::FillWidth();
        if (ImGui::InputText(("##ais_tcp_host_" + _this->name).c_str(), _this->hostBuf, sizeof(_this->hostBuf))) {
            _this->applyTcpConfig();
        }

        ImGui::LeftLabel("Port");
        ImGui::FillWidth();
        if (ImGui::InputInt(("##ais_tcp_port_" + _this->name).c_str(), &_this->port)) {
            if (_this->port < 0) { _this->port = 0; }
            if (_this->port > 65535) { _this->port = 65535; }
            _this->applyTcpConfig();
        }

        if (ImGui::Checkbox(("Send decoded contacts##ais_tcp_en_" + _this->name).c_str(), &_this->tcpEnabled)) {
            _this->applyTcpConfig();
        }

        const char* connTxt = _this->sender.isConnected() ? "Connected" : "Disconnected";
        ImGui::TextUnformatted("Status:"); ImGui::SameLine();
        if (_this->sender.isConnected()) { ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "%s", connTxt); }
        else                             { ImGui::TextColored(ImVec4(0.8f, 0.4f, 0.2f, 1.0f), "%s", connTxt); }
        ImGui::Text("Sent: %llu  Dropped: %llu", (unsigned long long)_this->sender.getSentCount(), (unsigned long long)_this->sender.getDroppedCount());

        // --- Decoder statistics ---
        ImGui::Separator();
        ImGui::Text("Frames OK: %llu", (unsigned long long)_this->decoder.framesOk);
        ImGui::Text("Messages:  %llu", (unsigned long long)_this->decoder.messagesParsed);

        // --- Contacts: opened in a separate, movable window ---
        ImGui::Separator();
        {
            std::lock_guard<std::mutex> lck(_this->contactsMtx);
            ImGui::Text("Contacts: %d", (int)_this->contacts.size());
        }
        const char* btnLabel = _this->showContactsWindow ? "Hide contacts window" : "Show contacts window";
        if (ImGui::Button((std::string(btnLabel) + "##ais_toggle_win_" + _this->name).c_str(), ImVec2(menuWidth, 0))) {
            _this->showContactsWindow = !_this->showContactsWindow;
            config.acquire();
            config.conf[_this->name]["showContactsWindow"] = _this->showContactsWindow;
            config.release(true);
        }

        // The contacts table lives in its own movable/resizable ImGui window.
        // Drawn here (as falcon9_decoder does); the lockWaterfallControls flag
        // inside drawContactsWindow() stops window drags from moving the VFO.
        _this->drawContactsWindow();
    }

    // Updates the comment so it matches reality (drawn from menuHandler).
    void drawContactsWindow() {
        if (!showContactsWindow) { return; }

        std::string title = "AIS Contacts (" + name + ")###ais_contacts_win_" + name;
        ImGui::SetNextWindowSize(ImVec2(640, 420), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(title.c_str(), &showContactsWindow)) {
            ImGui::End();
            return;
        }

        // Prevent the waterfall from reacting to clicks/drags that originate
        // inside our window. The waterfall input handler uses raw mouse state
        // plus a geometric hit test that ignores overlapping ImGui windows, so
        // without this, dragging our title bar over the waterfall would move
        // the VFO. The core resets lockWaterfallControls to showCredits at the
        // start of every MainWindow::draw, so we only assert it while our
        // window is actually engaged. We cover three cases: (1) the cursor is
        // over our window or its children, (2) an item inside is active, and
        // (3) our window is focused while a mouse button is held — this last
        // one catches title-bar drags where the cursor briefly leaves the
        // window rectangle and would otherwise let the waterfall grab the drag.
        if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem |
                                   ImGuiHoveredFlags_AllowWhenBlockedByPopup |
                                   ImGuiHoveredFlags_ChildWindows) ||
            ImGui::IsAnyItemActive() ||
            (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
             (ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Left)))) {
            gui::mainWindow.lockWaterfallControls = true;
        }

        if (ImGui::Button(("Clear##ais_clear_" + name).c_str())) {
            std::lock_guard<std::mutex> lck(contactsMtx);
            contacts.clear();
        }
        ImGui::SameLine();
        {
            std::lock_guard<std::mutex> lck(contactsMtx);
            ImGui::Text("%d contact(s)", (int)contacts.size());
        }

        // Legend for the column acronyms.
        ImGui::TextDisabled("SOG = Speed Over Ground (knots)  |  COG = Course Over Ground / direction (deg)  |  HDG = Heading (deg)");

        drawContactsTable();

        ImGui::End();
    }

    void drawContactsTable() {
        std::lock_guard<std::mutex> lck(contactsMtx);

        // Snapshot + sort by most recent.
        std::vector<const Contact*> list;
        list.reserve(contacts.size());
        for (auto& kv : contacts) { list.push_back(&kv.second); }
        std::sort(list.begin(), list.end(), [](const Contact* a, const Contact* b) { return a->lastUpdate > b->lastUpdate; });

        if (ImGui::BeginTable(("##ais_contacts_tbl_" + name).c_str(), 10,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable |
                              ImGuiTableFlags_Reorderable)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Name / MMSI");
            ImGui::TableSetupColumn("Time (UTC)");
            ImGui::TableSetupColumn("Lat (deg)");
            ImGui::TableSetupColumn("Lon (deg)");
            ImGui::TableSetupColumn("SOG (kn)");        // Speed Over Ground, knots
            ImGui::TableSetupColumn("COG (deg)");       // Course Over Ground (direction), degrees
            ImGui::TableSetupColumn("HDG (deg)");       // true Heading, degrees
            ImGui::TableSetupColumn("Type");            // ship type (human readable)
            ImGui::TableSetupColumn("Msgs");            // count of decoded messages
            ImGui::TableSetupColumn("Status");// navigational status
            ImGui::TableHeadersRow();

            for (const Contact* c : list) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(c->name.empty() ? ("MMSI:" + std::to_string(c->mmsi)).c_str() : c->name.c_str());

                ImGui::TableSetColumnIndex(1);
                {
                    char tb[16] = "-";
                    if (c->lastUpdate) {
                        struct tm tmv;
#ifdef _WIN32
                        gmtime_s(&tmv, &c->lastUpdate);
#else
                        gmtime_r(&c->lastUpdate, &tmv);
#endif
                        strftime(tb, sizeof(tb), "%H:%M:%S", &tmv);
                    }
                    ImGui::TextUnformatted(tb);
                }

                ImGui::TableSetColumnIndex(2);
                if (c->hasPos) { ImGui::Text("%.5f", c->lat); } else { ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(3);
                if (c->hasPos) { ImGui::Text("%.5f", c->lon); } else { ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(4);
                if (c->hasSpeed) { ImGui::Text("%.1f", c->sog); } else { ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(5);
                if (c->hasCog) { ImGui::Text("%.1f", c->cog); } else { ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(6);
                if (c->heading >= 0 && c->heading != 511) { ImGui::Text("%d", c->heading); } else { ImGui::TextUnformatted("-"); }
                ImGui::TableSetColumnIndex(7);
                {
                    std::string st = (c->shipType >= 0) ? shipTypeText(c->shipType) : "";
                    ImGui::TextUnformatted(st.empty() ? "-" : st.c_str());
                }
                ImGui::TableSetColumnIndex(8);
                ImGui::Text("%llu", (unsigned long long)c->msgCount);
                ImGui::TableSetColumnIndex(9);
                {
                    std::string ns = (c->navStatus >= 0) ? navStatusText(c->navStatus) : "";
                    ImGui::TextUnformatted(ns.empty() ? "-" : ns.c_str());
                }
            }
            ImGui::EndTable();
        }
    }

    void applyTcpConfig() {
        sender.configure(hostBuf, port);
        config.acquire();
        config.conf[name]["tcpHost"] = std::string(hostBuf);
        config.conf[name]["tcpPort"] = port;
        config.conf[name]["tcpEnabled"] = tcpEnabled;
        config.release(true);
    }

    std::string name;
    bool enabled = true;

    // DSP chain
    VFOManager::VFO* vfo = nullptr;
    AISDSP dsp;
    dsp::sink::Handler<uint8_t> dataHandler;
    ais::Decoder decoder;

    // TCP output
    TCPSender sender;
    char hostBuf[128] = "127.0.0.1";
    int port = 10110;            // common AIS/NMEA-over-TCP default
    bool tcpEnabled = true;
    bool showContactsWindow = false;

    // Contacts (shared between decoder thread and GUI thread)
    std::mutex contactsMtx;
    std::map<uint32_t, Contact> contacts;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/ais_decoder_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new AISDecoderModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (AISDecoderModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
