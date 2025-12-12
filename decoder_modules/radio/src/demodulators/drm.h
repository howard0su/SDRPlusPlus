#pragma once

#pragma once
#include "../demod.h"

namespace demod {
    class DRM : public Demodulator {
    public:
        DRM() {}

        DRM(std::string name, ConfigManager* config, dsp::stream<dsp::complex_t>* input, double bandwidth, double audioSR) {
            init(name, config, input, bandwidth, audioSR);
        }

        ~DRM() {
            stop();
        }

        void init(std::string name, ConfigManager* config, dsp::stream<dsp::complex_t>* input, double bandwidth, double audioSR) {
            this->name = name;
            audioSampleRate = audioSR;

            // Define structure
            c2s.init(input);
        }

        void start() {
            c2s.start();
        }

        void stop() {
            c2s.stop();
        }

        void showMenu() {}

        void setBandwidth(double bandwidth) {}

        void setInput(dsp::stream<dsp::complex_t>* input) {
            c2s.setInput(input);
        }

        void AFSampRateChanged(double newSR) {
            audioSampleRate = newSR;
        }


        // ============= INFO =============

        const char* getName() { return "DRM"; }
        double getIFSampleRate() { return audioSampleRate; }
        double getAFSampleRate() { return audioSampleRate; }
        double getDefaultBandwidth() { return audioSampleRate; }
        double getMinBandwidth() { return audioSampleRate; }
        double getMaxBandwidth() { return audioSampleRate; }
        bool getBandwidthLocked() { return true; }
        double getDefaultSnapInterval() { return 2500.0; }
        int getVFOReference() { return ImGui::WaterfallVFO::REF_CENTER; }
        bool getDeempAllowed() { return false; }
        bool getPostProcEnabled() { return false; }
        int getDefaultDeemphasisMode() { return DEEMP_MODE_NONE; }
        bool getFMIFNRAllowed() { return false; }
        bool getNBAllowed() { return true; }
        dsp::stream<dsp::stereo_t>* getOutput() { return &c2s.out; }

    private:
        double audioSampleRate;
        DRMDemodulation c2s;

        std::string name;
    };
}