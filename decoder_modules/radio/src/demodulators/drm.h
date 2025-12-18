#pragma once

#pragma once
#include "../demod.h"
#include "../drm/DRMReceiver.h"
#include "implot.h"

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
            auto plotManager = drmReceiver->GetPlotManager();
            auto ReceiveStatus = Parameters->ReceiveStatus;

            // Display signal quality metrics
            float snr = Parameters->GetSNR();
            ImGui::Text("SNR: %.1f dB", snr);

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

            if (snr == 0.0f)
                return;

            if (ImPlot::BeginPlot("Constellation", ImVec2(-1, 300), ImPlotFlags_NoMouseText | ImPlotFlags_NoInputs)) {
                ImPlot::SetupAxes("", "", ImPlotAxisFlags_NoTickLabels, ImPlotAxisFlags_NoTickLabels);
                ImPlot::SetupAxisLimits(ImAxis_X1, -1.5, 1.5, ImPlotCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5, 1.5, ImPlotCond_Always);

                std::vector<ImPlotPoint> points;
                CVector<_COMPLEX> vecConstellation;
                drmReceiver->GetMSCMLC()->GetVectorSpace(vecConstellation);
                // convert complex to ImPlot points
                for (const auto& c : vecConstellation) {
                    points.emplace_back(c.real(), c.imag());
                }
                ImPlot::PlotScatter("MSC", &points[0].x, &points[0].y, (int)points.size());

                vecConstellation.clear();
                drmReceiver->GetFACMLC()->GetVectorSpace(vecConstellation);
                // convert complex to ImPlot points
                points.clear();
                for (const auto& c : vecConstellation) {
                    points.emplace_back(c.real(), c.imag());
                }

                ImPlot::PlotScatter("FAC", &points[0].x, &points[0].y, (int)points.size());

                vecConstellation.clear();
                drmReceiver->GetSDCMLC()->GetVectorSpace(vecConstellation);
                // convert complex to ImPlot points
                points.clear();
                for (const auto& c : vecConstellation) {
                    points.emplace_back(c.real(), c.imag());
                }

                ImPlot::PlotScatter("SDC", &points[0].x, &points[0].y, (int)points.size());

                ImPlot::EndPlot();
            }

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
                        case CAudioParam::AC_AAC:
                            codec = "AAC";
                            break;
                        case CAudioParam::AC_OPUS:
                            codec = "OPUS";
                            break;
                        case CAudioParam::AC_xHE_AAC:
                            codec = "xHE-AAC";
                            break;
                        default:
                            codec = "Unknown";
                            break;
                        }

                        const char* mode = "Mono";
                        switch (audioParam.eAudioMode) {
                        case CAudioParam::AM_MONO:
                            mode = "Mono";
                            break;
                        case CAudioParam::AM_P_STEREO:
                            mode = "P-Stereo";
                            break;
                        case CAudioParam::AM_STEREO:
                            mode = "Stereo";
                            break;
                        default:
                            mode = "Unknown";
                            break;
                        }

                        ImGui::Text("Audio: [%s] %s %s%s",
                                    service.strLabel.c_str(),
                                    codec, mode,
                                    audioParam.eSBRFlag == CAudioParam::SBR_USED ? " +SBR" : "");
                    }
                    else {
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
            case ETypeRxStatus::NOT_PRESENT:
                return ImVec4(0.25f, 0.25f, 0.25f, 1.0f); // gray
            case ETypeRxStatus::CRC_ERROR:
                return ImVec4(0.5f, 0.0f, 0.0f, 1.0f); // red
            case ETypeRxStatus::DATA_ERROR:
                return ImVec4(0.5f, 0.25f, 0.0f, 1.0f); // orange
            case ETypeRxStatus::RX_OK:
                return ImVec4(0.0f, 0.5f, 0.0f, 1.0f); // green
            default:
                return ImVec4(0.0f, 0.0f, 0.0f, 1.0f); // black
            }
        }

        void DrawIndicator(const char* id, ETypeRxStatus state, float radius = 16.0f) {
            // Account for High-DPI scaling
            float scaled_radius = radius * style::uiScale;

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 pos = ImGui::GetCursorScreenPos();
            ImVec2 center = ImVec2(pos.x + scaled_radius, pos.y + scaled_radius);

            ImVec4 col = GetIndicatorColor(state);
            draw_list->AddCircleFilled(center, scaled_radius, ImColor(col));

            // Draw text centered in the circle
            ImVec2 text_size = ImGui::CalcTextSize(id);
            ImVec2 text_pos = ImVec2(center.x - text_size.x * 0.5f, center.y - text_size.y * 0.5f);
            draw_list->AddText(text_pos, IM_COL32(255, 255, 255, 255), id);

            // Reserve space for the circle
            ImGui::Dummy(ImVec2(scaled_radius * 2, scaled_radius * 2));
        }

    private:
        double audioSampleRate;
        CDRMReceiver* drmReceiver = nullptr;
        CSettings Settings;

        std::string name;
    };
}