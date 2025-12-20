#ifdef _WIN32
#include <winsock2.h>
#include <ws2ipdef.h>
#include <ws2tcpip.h>
inline void usleep(int micros) {
    Sleep(micros / 1000);
}
#endif

#include <imgui.h>
#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <config.h>
#include "kiwisdr.h"
#include "gui/smgui.h"
#include "imgui_stdlib.h"
#include <filesystem>
#include <chrono>
#include <fstream>


SDRPP_MOD_INFO{
    /* Name:            */ "web888_source",
    /* Description:     */ "Web888 source module for SDR-888",
    /* Author:          */ "SDDC Lab",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};


ConfigManager config;

struct Web888SourceModule : public ModuleManager::Instance {
    std::string web888Site = "servername:8073";
    KiwiSDRClient kiwiSdrClient;

    Web888SourceModule(std::string name) {
        this->name = name;

        config.acquire();
        if (config.conf.contains("web888_site")) {
            web888Site = config.conf["web888_site"];
        }
        config.release(false);

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;

        kiwiSdrClient.onConnected = [&]() {
            connected = true;
            tune(lastTuneFrequency, this);
        };

        kiwiSdrClient.onDisconnected = [&]() {
            connected = false;
            running = false;
            gui::mainWindow.setPlayState(false);
        };

        // Load config
        sigpath::sourceManager.registerSource("Web-888", &handler);
    }

    ~Web888SourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("Web-888");
    }

    void postInit() {
    }

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

    static void menuSelected(void* ctx) {
        Web888SourceModule* _this = (Web888SourceModule*)ctx;
        core::setInputSampleRate(12000); // fixed for web888
        flog::info("Web888SourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        Web888SourceModule* _this = (Web888SourceModule*)ctx;
        flog::info("Web888SourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        Web888SourceModule* _this = (Web888SourceModule*)ctx;
        if (_this->running) { return; }

        
        _this->kiwiSdrClient.init(_this->web888Site);
        _this->running = true;
        _this->kiwiSdrClient.start();
        _this->nextSend = 0;
        _this->timeSet = false;
        std::thread feeder([=]() {
            double nextSend = 0;
            while (_this->running) {
                _this->kiwiSdrClient.iqDataLock.lock();
                auto bufsize = _this->kiwiSdrClient.iqData.size();
                _this->kiwiSdrClient.iqDataLock.unlock();
                double now = (double)KiwiSDRClient::currentTimeMillis();
                if (nextSend == 0) {
                    if (bufsize < 200) {
                        usleep(16000); // some sleep
                        continue;      // waiting for initial batch
                    }
                    nextSend = now;
                }
                else {
                    auto delay = nextSend - now;
                    double sleepTime = delay * 1000;
                    if (sleepTime > 0) {
                        usleep(sleepTime);
                    }
                }
                std::vector<std::complex<float>> toSend;
                size_t bufferSize = 0;
                _this->kiwiSdrClient.iqDataLock.lock();
                if (_this->kiwiSdrClient.iqData.size() >= 200) {
                    for (int i = 0; i < 200; i++) {
                        toSend.emplace_back(_this->kiwiSdrClient.iqData[i]);
                    }
                    _this->kiwiSdrClient.iqData.erase(_this->kiwiSdrClient.iqData.begin(), _this->kiwiSdrClient.iqData.begin() + 200);
                    bufferSize = _this->kiwiSdrClient.iqData.size();
                }
                _this->kiwiSdrClient.iqDataLock.unlock();
                if (bufferSize > _this->kiwiSdrClient.NETWORK_BUFFER_SIZE) {
                    nextSend += 1000.0 / 120.0;
                }
                else {
                    nextSend += 1000.0 / 60.0;
                }
                int64_t ctm = KiwiSDRClient::currentTimeMillis();
                if (!toSend.empty()) {
                    memcpy(_this->stream.writeBuf, toSend.data(), toSend.size() * sizeof(dsp::complex_t));
                    _this->stream.swap((int)toSend.size());
                }
                else {
                    nextSend = 0;
                }
            }
        });
        feeder.detach();

        _this->running = true;
        flog::info("Web888SourceModule '{0}': Start!", _this->name);
    }

    static void stop(void* ctx) {
        Web888SourceModule* _this = (Web888SourceModule*)ctx;
        if (!_this->running) { return; }
        _this->kiwiSdrClient.stop();

        _this->running = false;
        flog::info("Web888SourceModule '{0}': Stop!", _this->name);
    }

    std::vector<dsp::complex_t> incomingBuffer;

    double nextSend = 0;

    void incomingSample(double i, double q) {
        incomingBuffer.emplace_back(dsp::complex_t{ (float)q, (float)i });
        if (incomingBuffer.size() >= 200) { // 60 times per second
            double now = (double)KiwiSDRClient::currentTimeMillis();
            if (nextSend == 0) {
                nextSend = now;
            }
            else {
                auto delay = nextSend - now;
                double sleepTime = delay * 1000;
                if (sleepTime > 0) {
                    usleep(sleepTime);
                }
            }
            //            flog::info("Sending samples: {}", incomingBuffer.size());
            nextSend += 1000.0 / 60.0;
            incomingBuffer.clear();
        }
    }


    double lastTuneFrequency = 14.100;

    static void tune(double freq, void* ctx) {
        Web888SourceModule* _this = (Web888SourceModule*)ctx;
        _this->lastTuneFrequency = freq;
        if (_this->running && _this->connected) {
            _this->kiwiSdrClient.tune(freq, KiwiSDRClient::TUNE_IQ);
        }
        flog::info("Web888SourceModule '{0}': Tune: {1}!", _this->name, freq);
    }


    static void menuHandler(void* ctx) {
        Web888SourceModule* _this = (Web888SourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }
        ImGui::InputText("Web888 Site", &_this->web888Site);
        if (_this->running) { SmGui::EndDisabled(); }
        
        ImGui::Text("Status: %s", _this->kiwiSdrClient.getConnectionStatus().c_str());
    }


    std::string name;
    bool enabled = true;
    bool running = false;
    bool connected = false;
    bool timeSet = false;

    double freq;
    bool serverBusy = false;

    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;

    std::shared_ptr<KiwiSDRClient> client;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/web888_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new Web888SourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (Web888SourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}