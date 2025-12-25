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
    char callsign[12];    // Up to 11 symbols + null terminator
    uint32_t hash;        // 8 MSBs: age, 22 LSBs: hash value
};

// Global callsign hash map used by the FT8 message code via the C interface.
static std::mutex g_callsign_mutex;
static std::unordered_map<uint32_t, std::pair<std::string, uint8_t>> g_callsign_map; // n22 -> (callsign, age)

static bool callsign_lookup_impl(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign_out)
{
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
    for (const auto &kv : g_callsign_map) {
        uint32_t n22 = kv.first;
        if (hash_type == FTX_CALLSIGN_HASH_12_BITS) {
            if ((n22 >> 10) == hash) {
                strncpy(callsign_out, kv.second.first.c_str(), 11);
                callsign_out[11] = '\0';
                return true;
            }
        } else { // 10 bits
            if ((n22 >> 12) == hash) {
                strncpy(callsign_out, kv.second.first.c_str(), 11);
                callsign_out[11] = '\0';
                return true;
            }
        }
    }

    return false;
}

static void callsign_save_impl(const char* callsign, uint32_t n22)
{
    std::lock_guard<std::mutex> lk(g_callsign_mutex);

    std::string s = callsign;
    if (s.size() > 11) s.resize(11);
    auto it = g_callsign_map.find(n22);
    if (it == g_callsign_map.end()) {
        g_callsign_map.emplace(n22, std::make_pair(s, (uint8_t)0));
    } else {
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
        ImGui::Text("Decoded Messages: %d", (int)_this->decodedMessages.size());

        if (!_this->enabled) {
            style::endDisabled();
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
                        } catch (const std::exception &e) {
                            flog::error("FT8DecoderModule decode thread exception: {}", e.what());
                        } catch (...) {
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
                decodedMessages[msg_hash] = message;

                // Log the decoded message
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                struct tm* tm_now = gmtime(&time_t_now);

                flog::info("[FT8] {:02d}{:02d}{:02d} {:+05.1f} {:+4.2f} {:4.0f} ~ {}",
                    tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec,
                    message.snr / 2.0f - 22.0f, time_sec, freq_hz, text);
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

    void cleanupCallsignHashTable(uint8_t max_age) {
        std::lock_guard<std::mutex> lk(g_callsign_mutex);

        // Increment age for all entries and remove entries older than max_age
        for (auto it = g_callsign_map.begin(); it != g_callsign_map.end();) {
            it->second.second++;
            if (it->second.second > max_age) {
                it = g_callsign_map.erase(it);
            } else {
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
    std::unordered_map<uint32_t, ftx_message_t> decodedMessages;
    std::unordered_map<std::string, CallsignHashEntry> callsignHashTable;
};

MOD_EXPORT void _INIT_() {}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) { return new FT8DecoderModule(name); }

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) { delete (FT8DecoderModule*)instance; }

MOD_EXPORT void _END_() {}