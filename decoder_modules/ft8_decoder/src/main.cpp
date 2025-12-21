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

#include "utils/flog.h"

#include <chrono>
#include <cstring>
#include <cmath>
#include <unordered_map>

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

class FT8DecoderModule : public ModuleManager::Instance {
public:
    FT8DecoderModule(std::string name) {
        this->name = name;

        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, BANDWIDTH, SAMPLE_RATE, SAMPLE_RATE, SAMPLE_RATE, true);

        agc.init(vfo->output, 1.0f, 1e6, 0.001f, 1.0f);
        sink.init(&agc.out, handler, this);

        agc.start();
        sink.start();

        // Initialize monitor
        monitor_config_t mon_cfg = {
            .f_min = 300,
            .f_max = BANDWIDTH,
            .sample_rate = (int)SAMPLE_RATE,
            .time_osr = kTime_osr,
            .freq_osr = kFreq_osr,
            .protocol = FTX_PROTOCOL_FT8
        };
        monitor_init(&monitor, &mon_cfg);

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
        if (vfo) {
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }
        agc.stop();
        sink.stop();

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
            }
            return;
        }

        // COLLECT_SYMBOLS state: process each block of samples directly
        for (int frame_pos = 0; frame_pos + monitor.block_size <= count; frame_pos += monitor.block_size) {
            // Check if we have enough waterfall data
            if (monitor.wf.num_blocks >= monitor.wf.max_blocks) {
                // Enough data collected, decode and reset
                decodeSlot();
                state = WAIT_FOR_START;
                return;
            }

            // Process the IQ frame
            monitor_process(&monitor, reinterpret_cast<const monitor_complex_t*>(data + frame_pos));
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
        // Use a stub interface for now - callsign hashing not implemented
        static ftx_callsign_hash_interface_t interface = {
            .lookup_hash = nullptr,
            .save_hash = nullptr
        };
        return &interface;
    }

    void cleanupCallsignHashTable(uint8_t max_age) {
        // Cleanup not needed in current implementation
        // In future, could implement proper callsign hash management
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
    } state = WAIT_FOR_START;

    // Message storage using hash for deduplication
    std::unordered_map<uint32_t, ftx_message_t> decodedMessages;
    std::unordered_map<std::string, CallsignHashEntry> callsignHashTable;
};

MOD_EXPORT void _INIT_() {}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) { return new FT8DecoderModule(name); }

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) { delete (FT8DecoderModule*)instance; }

MOD_EXPORT void _END_() {}