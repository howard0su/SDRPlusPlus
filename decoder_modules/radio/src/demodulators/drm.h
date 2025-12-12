#pragma once

#pragma once
#include "../demod.h"
#include "../drm/DRMReceiver.h"

namespace demod {
    class DRM : public Demodulator {
    public:
        DRM() {}

        DRM(std::string name, ConfigManager* config, dsp::stream<dsp::complex_t>* input, double bandwidth, double audioSR) {
            init(name, config, input, bandwidth, audioSR);
        }

        ~DRM() {
            stop();

            if (drmReceiver) {
                delete drmReceiver;
            }
        }

        void init(std::string name, ConfigManager* config, dsp::stream<dsp::complex_t>* input, double bandwidth, double audioSR) {
            this->name = name;
            audioSampleRate = audioSR;

            drmReceiver = new CDRMReceiver(&Settings);
            drmReceiver->init(input);
            drmReceiver->LoadSettings();

            Settings.Put("Receiver", "reverb", true);
            Settings.Put("Receiver", "samplerateaud", (int)getIFSampleRate());
            Settings.Put("Receiver", "sampleratesig", (int)getIFSampleRate());
        }

        void start() {
            drmReceiver->start();
        }

        void stop() {
            drmReceiver->stop();
        }
 
        void showMenu() {
            // Display DRM Receiver Parameters
            auto Parameters = drmReceiver->GetParameters();
            auto ReceiveStatus = Parameters->ReceiveStatus;

            // use a leds to display each status
            DrawIndicator("FS", ReceiveStatus.FSync.GetStatus());
            ImGui::SameLine();
            DrawIndicator("TS", ReceiveStatus.TSync.GetStatus());
            ImGui::SameLine();
            DrawIndicator("FAC", ReceiveStatus.FAC.GetStatus());
            ImGui::SameLine();
            DrawIndicator("SDC", ReceiveStatus.SDC.GetStatus());
            ImGui::SameLine();
            DrawIndicator("SL", ReceiveStatus.SLAudio.GetStatus());
            ImGui::SameLine();
            DrawIndicator("LL", ReceiveStatus.LLAudio.GetStatus());
            ImGui::NewLine();
            
            // Display signal quality metrics
            float snr = Parameters->GetSNR();
            ImGui::Text("SNR: %.1f dB", snr);
            
            // Display MER (Modulation Error Ratio)
            ImGui::Text("MER: %.1f dB  WMER-MSC: %.1f dB  WMER-FAC: %.1f dB", 
                        Parameters->rMER, Parameters->rWMERMSC, Parameters->rWMERFAC);
            
            // Display all service info
            for (int curService = 0; curService < (int)Parameters->Service.size(); curService++) {
                auto& service = Parameters->Service[curService];

                if (!service.IsActive())
                    continue;
                
                if (service.eAudDataFlag == CService::SF_AUDIO) {
                    // Display audio codec information
                    auto& audioParam = service.AudioParam;
                    if (audioParam.iStreamID != STREAM_ID_NOT_USED) {
                        const char* codec = "Unknown";
                        switch (audioParam.eAudioCoding) {
                            case CAudioParam::AC_AAC: codec = "AAC"; break;
                            case CAudioParam::AC_OPUS: codec = "OPUS"; break;
                            case CAudioParam::AC_xHE_AAC: codec = "xHE-AAC"; break;
                            default: codec = "Unknown"; break;
                        }
                        
                        const char* mode = "Mono";
                        switch (audioParam.eAudioMode) {
                            case CAudioParam::AM_MONO: mode = "Mono"; break;
                            case CAudioParam::AM_P_STEREO: mode = "P-Stereo"; break;
                            case CAudioParam::AM_STEREO: mode = "Stereo"; break;
                            default: mode = "Unknown"; break;
                        }
                        
                        ImGui::Text("Audio: [%s] %s %s%s", 
                                    service.strLabel.c_str(),
                                    codec, mode, 
                                    audioParam.eSBRFlag == CAudioParam::SBR_USED ? " +SBR" : "");
                    }
                    else 
                    {
                        continue;
                    }
                }
                else if (service.eAudDataFlag == CService::SF_DATA) {
                        // get data information
                        auto& dataParam = service.DataParam;
                        ImGui::SameLine();
                        ImGui::Text("Data: [%s] Stream ID: %d", service.strLabel.c_str(), dataParam.iStreamID);
                }
                ImGui::NewLine();
            }
        }

        void setBandwidth(double bandwidth) {
            // drmReceiver->setBandwidth(bandwidth);
        }

        void setInput(dsp::stream<dsp::complex_t>* input) {
            drmReceiver->setInput(input);
        }

        void AFSampRateChanged(double newSR) {
            audioSampleRate = newSR;
        }

        // ============= INFO =============

        const char* getName() { return "DRM"; }
        double getIFSampleRate() { return 48000.0; }
        double getAFSampleRate() { return 48000.0; }
        double getDefaultBandwidth() { return 9000.0; }
        double getMinBandwidth() { return 9000.0; }
        double getMaxBandwidth() { return 20000.0; }
        bool getBandwidthLocked() { return false; }
        double getDefaultSnapInterval() { return 500.0; }
        int getVFOReference() { return ImGui::WaterfallVFO::REF_CENTER; }
        bool getDeempAllowed() { return false; }
        bool getPostProcEnabled() { return false; }
        int getDefaultDeemphasisMode() { return DEEMP_MODE_NONE; }
        bool getFMIFNRAllowed() { return false; }
        bool getNBAllowed() { return false; }
        dsp::stream<dsp::stereo_t>* getOutput() { return &drmReceiver->out; }

    private:
        static ImVec4 GetIndicatorColor(ETypeRxStatus state) {
            switch (state) {
                case ETypeRxStatus::NOT_PRESENT:        return ImVec4(0.5f, 0.5f, 0.5f, 1.0f); // gray
                case ETypeRxStatus::CRC_ERROR:  return ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // red
                case ETypeRxStatus::DATA_ERROR: return ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // orange
                case ETypeRxStatus::RX_OK:        return ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // green
                default:
                    return ImVec4(0.0f, 0.0f, 0.0f, 1.0f); // black
            }
        }

        void DrawIndicator(const char* id, ETypeRxStatus state, float radius = 16.0f)
        {
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 pos = ImGui::GetCursorScreenPos();
            ImVec2 center = ImVec2(pos.x + radius, pos.y + radius);

            ImVec4 col = GetIndicatorColor(state);
            draw_list->AddCircleFilled(center, radius, ImColor(col));

            // Reserve space for the circle + spacing + text
            ImGui::Dummy(ImVec2(radius * 2 + 6, radius * 2));
            ImGui::SameLine();

            ImGui::TextUnformatted(id);
        }
    private:
        double audioSampleRate;
        CDRMReceiver *drmReceiver = nullptr;
        CSettings Settings;

        std::string name;

    };
}