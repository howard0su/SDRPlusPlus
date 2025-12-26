#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <gui/smgui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <utils/optionlist.h>
#include <atomic>
#include <assert.h>
#include "libsddc.h"

#include "fft_rx_vfo.h"

SDRPP_MOD_INFO{
    /* Name:            */ "sddc_source",
    /* Description:     */ "SDDC Source Module",
    /* Author:          */ "Howard Su",
    /* Version:         */ 1, 0, 0,
    /* Max instances    */ -1
};

ConfigManager config;

#define CONCAT(a, b) ((std::string(a) + b).c_str())

// The following count have to be choose as
// 3 * N = 4 * BufferCount + 1
// where N is integer, so that the FFT bins align correctly
// the values we can use 41, 80, 92， 101
#define SDDC_ACCUMRATE_BUFFER_COUNT 101
#define SDDC_BUFFER_SIZE (16 * 1024 / 2)

#define TUNER_IF_FREQUENCY 4570000.0

// GAINFACTORS to be adjusted with lab reference source measured with HDSDR Smeter rms mode  
#define BBRF103_GAINFACTOR 	(7.800e-8f)       // BBRF103
#define HF103_GAINFACTOR   	(1.140e-8f)      // HF103
#define RX888_GAINFACTOR   	(0.695e-8f)     // RX888
#define RX888mk2_GAINFACTOR (1.080e-8f)      // RX888mk2

class SDDCSourceModule : public ModuleManager::Instance {
public:
    SDDCSourceModule(std::string name) {
        this->name = name;

        sampleRate = 128 * 1000 * 1000.0;

        // Initialize the DDC
        ddc.init(&dataIn, RX888mk2_GAINFACTOR);

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &ddc.out;

        // Refresh devices
        refresh();

        // Select device from config
        config.acquire();
        std::string devSerial = config.conf["device"];
        config.release();
        select(devSerial);

        sigpath::sourceManager.registerSource("RX-888", &handler);
    }

    ~SDDCSourceModule() {
        sigpath::sourceManager.unregisterSource("SDDC");
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

    enum Port {
        PORT_VHF,
        PORT_HF
    };

private:
    std::string getBandwdithScaled(double bw) {
        char buf[1024];
        if (bw >= 1000000.0) {
            snprintf(buf, 1024, "%.1lfMHz", bw / 1000000.0);
        }
        else if (bw >= 1000.0) {
            snprintf(buf, 1024, "%.1lfKHz", bw / 1000.0);
        }
        else {
            snprintf(buf, 1024, "%.1lfHz", bw);
        }
        return std::string(buf);
    }

    void refresh() {
        devices.clear();

        int devCount = sddc_get_device_count();

        // If no device, give up
        if (!devCount) { return; }

        for (int i = 0; i < devCount; ++i) {
            char serial[256];
            int err = sddc_get_device_usb_strings(i, NULL, NULL, serial);
            if (err == 0) {
                devices.define(std::string(serial), std::string(serial), i);
            }
        }
    }

    void select(const std::string& serial) {

        // If there are no devices, give up
        if (devices.empty()) {
            selectedSerial.clear();
            return;
        }

        int index = sddc_get_index_by_serial(serial.c_str());

        // if the serial was not found, select the first device
        if (index < 0) {
            select(devices.key(0));
            return;
        }

        // Get the ID in the list
        int id = devices.keyId(serial);
        selectedDevId = devices[id];

        // Define the ports
        ports.clear();
        ports.define("hf", "HF", PORT_HF);
        ports.define("vhf", "VHF", PORT_VHF);

        // Save serial number
        selectedSerial = serial;
        selectedDevId = id;

        // some API needs the device specific information
        sddc_dev_t* dev;
        int err = sddc_open(&dev, selectedDevId);
        if (err) {
            flog::error("Failed to open device: {}", err);
            return;
        }

        // determine the device type
        char product[256];
        sddc_get_usb_strings(dev, NULL, product, NULL);

        float gainFactor = RX888_GAINFACTOR;
        if (strstr(product, "RX888mk2")) {
            gainFactor = RX888mk2_GAINFACTOR;
        }
        else if (strstr(product, "RX888")) {
            gainFactor = RX888_GAINFACTOR;
        }

        ddc.setGainFactor(gainFactor);

        // Load default options
        port = PORT_HF;
        portId = ports.valueId(port);
        rfGain = 0;
        ifGain = 0;
        bias = false;

        // Load config
        config.acquire();
        if (config.conf["devices"][selectedSerial].contains("port")) {
            std::string desiredPort = config.conf["devices"][selectedSerial]["port"];
            if (ports.keyExists(desiredPort)) {
                portId = ports.keyId(desiredPort);
                port = ports[portId];
            }
        }

        sddc_set_direct_sampling(dev, (port == PORT_HF) ? 1 : 0);

        if (port == PORT_HF) {

            // define supported samplerates
            samplerates.clear();
            sampleRate = 64e6;
            for (int i = 1; i <= 6; ++i) {
                samplerates.define(sampleRate, getBandwdithScaled(sampleRate), sampleRate);
                sampleRate /= 2;
            }

            sampleRate = 32e6;
            srId = samplerates.valueId(sampleRate);
        }
        else {
            // define supported samplerates
            samplerates.clear();
            sampleRate = 8e6;
            for (int i = 1; i <= 4; ++i) {
                samplerates.define(sampleRate, getBandwdithScaled(sampleRate), sampleRate);
                sampleRate /= 2;
            }

            sampleRate = 8e6;
            srId = samplerates.valueId(sampleRate);
        }

        if (config.conf["devices"][selectedSerial].contains("samplerate")) {
            int desiredSr = config.conf["devices"][selectedSerial]["samplerate"];
            if (samplerates.keyExists(desiredSr)) {
                srId = samplerates.keyId(desiredSr);
                sampleRate = samplerates[srId];
            }
        }

        if (config.conf["devices"][selectedSerial].contains("rfGain")) {
            float min, max;
            sddc_get_rf_gain_range(dev, &min, &max);
            rf_gain_max = max;
            rf_gain_min = min;
            rfGain = std::clamp<int>(config.conf["devices"][selectedSerial]["rfGain"], rf_gain_min, rf_gain_max);
        }
        if (config.conf["devices"][selectedSerial].contains("ifGain")) {
            float min, max;
            sddc_get_if_gain_range(dev, &min, &max);
            if_gain_max = max;
            if_gain_min = min;
            ifGain = std::clamp<int>(config.conf["devices"][selectedSerial]["ifGain"], if_gain_min, if_gain_max);
        }
        if (config.conf["devices"][selectedSerial].contains("bias")) {
            bias = config.conf["devices"][selectedSerial]["bias"];
        }
        config.release();

        sddc_close(dev);

        // Update the samplerate
        core::setInputSampleRate(sampleRate);

        // Update freq select limits
        if (port == PORT_HF) {
            gui::freqSelect.minFreq = 0;
            gui::freqSelect.maxFreq = sampleRate <= 32e6 ? 32e6 : 64e6;
        }
        else {
            gui::freqSelect.minFreq = 30e6;
            gui::freqSelect.maxFreq = 2e9;
        }
        gui::freqSelect.limitFreq = true;
    }

    static void menuSelected(void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
        core::setInputSampleRate(_this->sampleRate);
        flog::debug("SDDCSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
        flog::debug("SDDCSourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
        if (_this->running) { return; }

        _this->start();
    }

    void start() {
        // Open the device
        int err = sddc_open(&openDev, selectedDevId);
        if (err) {
            flog::error("Failed to open device: {}", (int)err);
            return;
        }

        xtal_freq = sampleRate * 2;
        if (sampleRate < 32e6) {
            xtal_freq = 64e6;
        } else {
            xtal_freq = sampleRate * 2;
        }

        // set HF or VHF first
        if (port == PORT_HF) {
            sddc_set_direct_sampling(openDev, 1);
            sddc_enable_bias_tee(openDev, bias ? 1 : 0);

            // Configure and start the DDC for decimation only
            ddc.setInSamplerate(xtal_freq);
            ddc.setOutSamplerate(sampleRate, sampleRate);
            ddc.setOffset(freq);
            ddc.start();
        }
        else {
            sddc_set_direct_sampling(openDev, 0);
            sddc_enable_bias_tee(openDev, bias ? 0x02 : 0);

            sddc_set_center_freq64(openDev, (uint64_t)freq);

            // Configure and start the DDC for decimation only
            ddc.setInSamplerate(xtal_freq);
            ddc.setOutSamplerate(sampleRate, sampleRate);
            ddc.setOffset(-TUNER_IF_FREQUENCY);
            ddc.start();
        }

        sddc_set_xtal_freq(openDev, xtal_freq);
        
        float min, max;
        sddc_get_rf_gain_range(openDev, &min, &max);
        rf_gain_max = max;
        rf_gain_min = min;
        rfGain = std::clamp<int>(rfGain, rf_gain_min, rf_gain_max);
        sddc_get_if_gain_range(openDev, &min, &max);
        if_gain_max = max;
        if_gain_min = min;
        ifGain = std::clamp<int>(ifGain, if_gain_min, if_gain_max);

        sddc_set_if_gain(openDev, ifGain);
        float val;
        sddc_get_if_gain(openDev, &val);
        ifGain = (int)val;
        sddc_set_rf_gain(openDev, rfGain);
        sddc_get_rf_gain(openDev, &val);
        rfGain = (int)val;

        buffercount = 0;

        running = true;
        sddc_read_async(openDev, &sddc_async_callback, this);

        flog::info("SDDCSourceModule '{0}': Start!", name);
    }

    static void stop(void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
        if (!_this->running) { return; }

        _this->stop();
    }

    void stop() {
        running = false;

        dataIn.stopWriter();
        sddc_cancel_async(openDev);
        dataIn.clearWriteStop();

        // Stop the DDC
        ddc.stop();

        // Close the device
        sddc_close(openDev);

        flog::info("SDDCSourceModule '{0}': Stop!", name);
    }

    static void tune(double freq, void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
        if (_this->running) {
            if (_this->port == PORT_VHF) {
                sddc_set_center_freq64(_this->openDev, (uint64_t)freq);
            }
            else {
                if (freq < _this->xtal_freq / 2)
                {
                    _this->ddc.setOffset(freq);
                }
            }
        }
        _this->freq = freq;
        flog::debug("SDDCSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }

        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_sddc_dev_sel_", _this->name), &_this->selectedDevId, _this->devices.txt)) {
            _this->select(_this->devices.key(_this->selectedDevId));
            core::setInputSampleRate(_this->sampleRate);
            config.acquire();
            config.conf["device"] = _this->selectedSerial;
            config.release(true);
        }

        if (SmGui::Combo(CONCAT("##_sddc_sr_sel_", _this->name), &_this->srId, _this->samplerates.txt)) {
            _this->sampleRate = _this->samplerates.value(_this->srId);
            core::setInputSampleRate(_this->sampleRate);
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["samplerate"] = _this->samplerates.key(_this->srId);
                config.release(true);
            }
        }

        SmGui::SameLine();
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Button(CONCAT("Refresh##_sddc_refr_", _this->name))) {
            _this->refresh();
            _this->select(_this->selectedSerial);
            core::setInputSampleRate(_this->sampleRate);
        }

        SmGui::LeftLabel("Antenna Port");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_sddc_port_", _this->name), &_this->portId, _this->ports.txt)) {
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["port"] = _this->ports.key(_this->portId);
                config.release(true);

                _this->select(_this->selectedSerial);
            }
        }

        if (_this->running) { SmGui::EndDisabled(); }

        if (SmGui::Checkbox(CONCAT("Bias-T##_sddc_bias_", _this->name), &_this->bias)) {
            if (_this->running) {
                int flag = _this->port == PORT_HF ? 1 : 2;
                sddc_enable_bias_tee(_this->openDev, _this->bias ? flag : 0);
            }
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["bias"] = _this->bias;
                config.release(true);
            }
        }

        SmGui::LeftLabel("RF Gain");
        SmGui::FillWidth();
        if (SmGui::SliderInt(CONCAT("##_sddc_rf_gain_", _this->name), &_this->rfGain, _this->rf_gain_min, _this->rf_gain_max)) {
            if (_this->running) {
                sddc_set_rf_gain(_this->openDev, _this->rfGain);
            }
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["rfGain"] = _this->rfGain;
                config.release(true);
            }
        }

        SmGui::LeftLabel("IF Gain");
        SmGui::FillWidth();
        if (SmGui::SliderInt(CONCAT("##_sddc_if_gain_", _this->name), &_this->ifGain, _this->if_gain_min, _this->if_gain_max)) {
            if (_this->running) {
                sddc_set_if_gain(_this->openDev, _this->ifGain);
            }
            if (!_this->selectedSerial.empty()) {
                config.acquire();
                config.conf["devices"][_this->selectedSerial]["ifGain"] = _this->ifGain;
                config.release(true);
            }
        }
    }

    static void sddc_async_callback(const int16_t* buffer, uint32_t count, void* ctx) {
        SDDCSourceModule* _this = (SDDCSourceModule*)ctx;

        assert(SDDC_BUFFER_SIZE == count);

        memcpy(_this->dataIn.writeBuf + _this->buffercount * SDDC_BUFFER_SIZE, buffer, count * sizeof(int16_t));
        
        _this->buffercount++;
        // If buffer is full, swap and reset fill
        if (_this->buffercount == SDDC_ACCUMRATE_BUFFER_COUNT) {
            _this->dataIn.swap(SDDC_BUFFER_SIZE * SDDC_ACCUMRATE_BUFFER_COUNT);
            _this->buffercount = 0;
        }
    }

    std::string name;
    bool enabled = true;
    SourceManager::SourceHandler handler;
    bool running = false;
    double freq;

    OptionList<std::string, int> devices;
    int selectedDevId = 0;

    uint32_t xtal_freq;
    OptionList<int, double> samplerates;
    int srId = 0;
    double sampleRate;

    OptionList<std::string, Port> ports;
    int portId = 0;
    Port port;

    int rfGain = 0;
    int ifGain = 0;
    std::string selectedSerial;

    sddc_dev_t* openDev;

    int buffercount;
    std::thread workerThread;
    std::atomic<bool> run = false;

    bool bias;
    int rf_gain_min = 0, rf_gain_max = 0;
    int if_gain_min = 0, if_gain_max = 0;

    dsp::channel::FFTRxVFO ddc;
    dsp::stream<int16_t> dataIn;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["devices"] = json({});
    def["device"] = "";
    config.setPath(core::args["root"].s() + "/sddc_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new SDDCSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (SDDCSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}