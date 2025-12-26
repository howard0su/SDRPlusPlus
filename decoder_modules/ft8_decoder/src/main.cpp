#include <config.h>
#include <core.h>
#include <filesystem>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/widgets/image.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>

#include <dsp/demod/quadrature.h>
#include <dsp/sink/handler_sink.h>
#include <dsp/window/hann.h>
#include <dsp/loop/fast_agc.h>

#include "monitor.h"
#include "ft8/message.h"

#include "imosm_rich.h"

#include "utils/flog.h"

#include <chrono>
#include <cstring>
#include <cmath>
#include <unordered_map>
#include <vector>

SDRPP_MOD_INFO{ /* Name:            */ "ft8_decoder",
                /* Description:     */ "FT8 decoder for SDR-888",
                /* Author:          */ "SDDC Lab",
                /* Version:         */ 0, 1, 0,
                /* Max instances    */ -1 };

#define SAMPLE_RATE 12000.0f
#define BANDWIDTH   3000.0f

// FT8 configuration constants (from demo.c)
const int kMin_score = 5;
const int kMax_candidates = 256;
const int kLDPC_iterations = 25;
const int kMax_decoded_messages = 50;
const int kFreq_osr = 2;
const int kTime_osr = 4;

// Callsign hash table entry
struct CallsignHashEntry {
    char callsign[12]; // Up to 11 symbols + null terminator
    uint32_t hash;     // 8 MSBs: age, 22 LSBs: hash value
};

// Global callsign hash map used by the FT8 message code via the C interface.
static std::mutex g_callsign_mutex;
static std::unordered_map<uint32_t, std::pair<std::string, uint8_t>> g_callsign_map; // n22 -> (callsign, age)

static bool callsign_lookup_impl(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign_out) {
    std::lock_guard<std::mutex> lk(g_callsign_mutex);

    if (hash_type == FTX_CALLSIGN_HASH_22_BITS) {
        auto it = g_callsign_map.find(hash);
        if (it == g_callsign_map.end())
            return false;
        strncpy(callsign_out, it->second.first.c_str(), 11);
        callsign_out[11] = '\0';
        return true;
    }

    // For 12- and 10-bit hashes, search for a matching prefix
    for (const auto& kv : g_callsign_map) {
        uint32_t n22 = kv.first;
        if (hash_type == FTX_CALLSIGN_HASH_12_BITS) {
            if ((n22 >> 10) == hash) {
                strncpy(callsign_out, kv.second.first.c_str(), 11);
                callsign_out[11] = '\0';
                return true;
            }
        }
        else { // 10 bits
            if ((n22 >> 12) == hash) {
                strncpy(callsign_out, kv.second.first.c_str(), 11);
                callsign_out[11] = '\0';
                return true;
            }
        }
    }

    return false;
}

static void callsign_save_impl(const char* callsign, uint32_t n22) {
    std::lock_guard<std::mutex> lk(g_callsign_mutex);

    std::string s = callsign;
    if (s.size() > 11) s.resize(11);
    auto it = g_callsign_map.find(n22);
    if (it == g_callsign_map.end()) {
        g_callsign_map.emplace(n22, std::make_pair(s, (uint8_t)0));
    }
    else {
        it->second.first = s;
        it->second.second = 0; // reset age
    }
}

class FT8DecoderModule : public ModuleManager::Instance {
public:
    FT8DecoderModule(std::string name) {
        this->name = name;

        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_LOWER, 0, BANDWIDTH, SAMPLE_RATE, SAMPLE_RATE, SAMPLE_RATE, true);

        agc.init(vfo->output, 1.0f, 1e6, 0.001f, 1.0f);
        sink.init(&agc.out, handler, this);

        agc.start();
        sink.start();

        // Initialize monitor
        monitor_config_t mon_cfg;
        mon_cfg.f_min = 300;
        mon_cfg.f_max = BANDWIDTH;
        mon_cfg.sample_rate = (int)SAMPLE_RATE;
        mon_cfg.time_osr = kTime_osr;
        mon_cfg.freq_osr = kFreq_osr;
        mon_cfg.protocol = FTX_PROTOCOL_FT8;

        monitor_init(&monitor, &mon_cfg);
        // Ensure sample buffer is empty at start
        sampleBuffer.clear();

        _mapPlot.setBoundsGeo(ImOsm::MIN_LAT, ImOsm::MAX_LAT, ImOsm::MIN_LON,
                              ImOsm::MAX_LON);

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~FT8DecoderModule() {
        disable();
        monitor_free(&monitor);
        gui::menu.removeEntry(name);
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        agc.stop();
        sink.stop();

        if (vfo) {
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }

        // clear any buffered samples
        sampleBuffer.clear();

        enabled = false;
    }

    bool isEnabled() { return enabled; }

private:
    static void menuHandler(void* ctx) {
        FT8DecoderModule* _this = (FT8DecoderModule*)ctx;

        if (!_this->enabled) {
            style::beginDisabled();
        }

        ImGui::Text("FT8 Decoder");
        ImGui::Text("Status: %s", _this->state == WAIT_FOR_START ? "Waiting for slot" : "Collecting symbols");

        if (!_this->enabled) {
            style::endDisabled();
        }

        if (_this->_markItems.size() > 0) {
            double minLat = ImOsm::MAX_LAT, maxLat = ImOsm::MIN_LAT, minLon = ImOsm::MAX_LON, maxLon = ImOsm::MIN_LON;

            // caculate max min/max lat/lon from mark items
            for (auto item : _this->_markItems) {
                ImOsm::GeoCoords coords = item.second->geoCoords();
                if (coords.lat < minLat) minLat = coords.lat;
                if (coords.lat > maxLat) maxLat = coords.lat;
                if (coords.lon < minLon) minLon = coords.lon;
                if (coords.lon > maxLon) maxLon = coords.lon;
            }

            _this->_mapPlot.setBoundsGeo(minLat, maxLat, minLon, maxLon);

            {
                std::lock_guard<std::mutex> lk(_this->_mapMutex);
                _this->_mapPlot.paint();
            }
        }
    }

    static void handler(dsp::complex_t* data, int count, void* ctx) {
        FT8DecoderModule* _this = (FT8DecoderModule*)ctx;
        _this->handle((dsp::complex_t*)data, count);
    }

    void handle(dsp::complex_t* data, int count) {
        if (state == WAIT_FOR_START) {
            // Check if now is an FT8 window start
            // FT8 windows are 15 seconds: at 0, 15, 30, 45 seconds
            using namespace std::chrono;
            const auto now = system_clock::now();
            const auto ms_epoch = duration_cast<milliseconds>(now.time_since_epoch()).count();
            int sec = (int)((ms_epoch / 1000) % 60);

            if ((sec % 15) == 0) {
                monitor_reset(&monitor);
                state = COLLECT_SYMBOLS;
                slotStartTime = now;
                flog::info("FT8DecoderModule: COLLECT_SYMBOLS, sec = {}", sec);
            }

            return;
        }
        else if (state == COLLECT_SYMBOLS) {
            // COLLECT_SYMBOLS state: buffer incoming samples until we have full blocks
            // Append incoming samples to buffer
            sampleBuffer.insert(sampleBuffer.end(), data, data + count);

            // Process full blocks from the buffer
            while ((int)sampleBuffer.size() >= monitor.block_size) {
                // Check if we have enough waterfall data
                if (monitor.wf.num_blocks >= monitor.wf.max_blocks) {
                    flog::info("FT8DecoderModule: Decoding slot, collected {} blocks", monitor.wf.num_blocks);
                    // Enough data collected, decode and reset
                    state = DECODING;
                    std::thread decodeThread([this]() {
                        try {
                            this->decodeSlot();
                            this->state = WAIT_FOR_START;
                            this->sampleBuffer.clear();
                        }
                        catch (const std::exception& e) {
                            flog::error("FT8DecoderModule decode thread exception: {}", e.what());
                        }
                        catch (...) {
                            flog::error("FT8DecoderModule decode thread unknown exception");
                        }
                    });
                    decodeThread.detach();
                    return;
                }

                monitor_process(&monitor, reinterpret_cast<const monitor_complex_t*>(sampleBuffer.data()));

                // remove consumed samples from the front of the buffer
                sampleBuffer.erase(sampleBuffer.begin(), sampleBuffer.begin() + monitor.block_size);
            }
        }
    }

    void decodeSlot() {
        const ftx_waterfall_t* wf = &monitor.wf;

        // Candidate list
        ftx_candidate_t candidate_list[kMax_candidates];

        // Message hash table for deduplication
        std::unordered_map<uint32_t, ftx_message_t> message_map;

        for (int pass = 0; pass < 3; pass++) {
            int num_candidates = ftx_find_candidates(wf, kMax_candidates / (pass + 1), candidate_list, kMin_score);

            // Go over candidates and attempt to decode messages
            for (int idx = 0; idx < num_candidates; ++idx) {
                const ftx_candidate_t* cand = &candidate_list[idx];

                float freq_hz = (monitor.min_bin + cand->freq_offset + (float)cand->freq_sub / wf->freq_osr) / monitor.symbol_period;
                float time_sec = (cand->time_offset + (float)cand->time_sub / wf->time_osr) * monitor.symbol_period;

                ftx_message_t message;
                ftx_decode_status_t status;

                if (!ftx_decode_candidate(wf, cand, kLDPC_iterations, &message, &status)) {
                    // Decoding failed
                    continue;
                }

                // Check for duplicate using C++ unordered_map
                uint32_t msg_hash = message.hash;
                if (message_map.find(msg_hash) != message_map.end()) {
                    // Duplicate found
                    continue;
                }

                // Decode the message text
                char text[FTX_MAX_MESSAGE_LENGTH];
                ftx_message_rc_t unpack_status = ftx_message_decode(&message, getCallsignHashInterface(), text);
                if (unpack_status != FTX_MESSAGE_RC_OK) {
                    snprintf(text, sizeof(text), "Error [%d] unpacking", (int)unpack_status);
                    continue;
                }

                // Store the message
                message_map[msg_hash] = message;
                {
                    // decode std message to append callsign and grid for map plotting
                    char call_to[12] = { 0 };
                    char call_de[12] = { 0 };
                    char grid[12] = { 0 };
                    ftx_message_rc_t std_status = ftx_message_decode_std(&message, getCallsignHashInterface(), call_to, call_de, grid);
                    if (std_status != FTX_MESSAGE_RC_OK || _markItems.find(call_de) != _markItems.end()) {
                        continue;
                    }

                    std::string grid_str = std::string(grid);
                    if (grid_str.find('+') == std::string::npos && grid_str.find('-') == std::string::npos) {
                        double lat, lon;

                        flog::debug("FT8DecoderModule: Parsing grid with offset: {}", grid_str);
                        if (grid[0] == 'R' && grid[1] == ' ') {
                            grid_str = grid_str.substr(2);
                        }

                        if (maidenhead_to_latlon(grid_str.data(), &lat, &lon) == 0) {
                            std::lock_guard<std::mutex> lk(_mapMutex);
                            auto item = std::make_shared<ImOsm::Rich::MarkItem>(ImOsm::GeoCoords(lat, lon), call_de);
                            item->style().markerSize = 2.0f;
                            _markItems[call_de] = item;
                            _mapPlot.addItem(item);
                            flog::info("FT8DecoderModule: Plotted callsign {} at grid {} (lat {:.4f}, lon {:.4f})", call_de, grid_str, lat, lon);
                        }
                    }
                }
            }
        }

        // Cleanup callsign hash table (age > 10)
        cleanupCallsignHashTable(10);
    }

    static ftx_callsign_hash_interface_t* getCallsignHashInterface() {
        static ftx_callsign_hash_interface_t callsign_if;
        callsign_if.lookup_hash = &callsign_lookup_impl;
        callsign_if.save_hash = &callsign_save_impl;
        return &callsign_if;
    }

    /*
     * Convert Maidenhead Grid Locator (4 or 6 chars) to latitude/longitude
     * lat, lon returned in degrees
     * Center of grid square is returned
     *
     * Returns 0 on success, -1 on invalid input
     */
    static int maidenhead_to_latlon(const char* grid, double* lat, double* lon) {
        if (!grid || !lat || !lon)
            return -1;

        int len = strlen(grid);
        if (len != 4 && len != 6)
            return -1;

        char g[7] = { 0 };
        for (int i = 0; i < len; i++)
            g[i] = tolower((unsigned char)grid[i]);

        /* Validate basic ranges */
        if (g[0] < 'a' || g[0] > 'r') return -1;
        if (g[1] < 'a' || g[1] > 'r') return -1;
        if (g[2] < '0' || g[2] > '9') return -1;
        if (g[3] < '0' || g[3] > '9') return -1;
        if (len == 6) {
            if (g[4] < 'a' || g[4] > 'x') return -1;
            if (g[5] < 'a' || g[5] > 'x') return -1;
        }

        /* Start from southwest corner */
        double longitude = -180.0;
        double latitude = -90.0;

        /* Field */
        longitude += (g[0] - 'a') * 20.0;
        latitude += (g[1] - 'a') * 10.0;

        /* Square */
        longitude += (g[2] - '0') * 2.0;
        latitude += (g[3] - '0') * 1.0;

        if (len == 6) {
            /* Subsquare */
            longitude += (g[4] - 'a') * (5.0 / 60.0);
            latitude += (g[5] - 'a') * (2.5 / 60.0);

            /* Center of subsquare */
            longitude += (5.0 / 60.0) / 2.0;
            latitude += (2.5 / 60.0) / 2.0;
        }
        else {
            /* Center of square */
            longitude += 1.0;
            latitude += 0.5;
        }

        *lat = latitude;
        *lon = longitude;
        return 0;
    }

    void cleanupCallsignHashTable(uint8_t max_age) {
        std::lock_guard<std::mutex> lk(g_callsign_mutex);

        // Increment age for all entries and remove entries older than max_age
        for (auto it = g_callsign_map.begin(); it != g_callsign_map.end();) {
            it->second.second++;
            if (it->second.second > max_age) {
                it = g_callsign_map.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    std::string name;
    bool enabled = true;

    VFOManager::VFO* vfo = NULL;
    dsp::loop::FastAGC<dsp::complex_t> agc;
    dsp::sink::Handler<dsp::complex_t> sink;

    monitor_t monitor = {};
    std::chrono::system_clock::time_point slotStartTime;

    enum {
        WAIT_FOR_START,
        COLLECT_SYMBOLS,
        DECODING,
    } state = WAIT_FOR_START;

    // Message storage using hash for deduplication
    // Buffer for incoming samples when count < monitor.block_size
    std::vector<dsp::complex_t> sampleBuffer;
    std::unordered_map<std::string, CallsignHashEntry> callsignHashTable;
    ImOsm::Rich::RichMapPlot _mapPlot;

    std::mutex _mapMutex;
    std::unordered_map<std::string, std::shared_ptr<ImOsm::Rich::MarkItem>> _markItems;
};

MOD_EXPORT void _INIT_() {}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) { return new FT8DecoderModule(name); }

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) { delete (FT8DecoderModule*)instance; }

MOD_EXPORT void _END_() {}