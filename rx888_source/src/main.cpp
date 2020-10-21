
#pragma once
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>

#include "openFX3.h"
#include "Si5351.h"

#define ADC_FREQ 4000000

#define USE_FS_4 1

#if USE_FS_4
#define SAMPLE_RATE (ADC_FREQ/2)
#else
#define SAMPLE_RATE (ADC_FREQ)
#endif

ConfigManager config;

MOD_INFO{
    /* Name:        */ "rx888",
    /* Description: */ "RX888 input module for SDR++",
    /* Author:      */ "Howard Su",
    /* Version:     */ "0.1.0"};

typedef struct _range
{
    float _minimum;
    float _maximum;

    float minimum() { return _minimum; }
    float maximum() { return _maximum; }
} range;

using namespace dsp;

class RX888Module
{
public:
    RX888Module(std::string name)
    {

        this->name = name;
        if (!openFX3())
        {
            return;
        }

        Si5351init();
        initialized = true;
        currentGains = new float[1];

        in.init(64000);
        vfo.init(&in, SAMPLE_RATE, SAMPLE_RATE / 2, SAMPLE_RATE / 2, SAMPLE_RATE / 4, blockSize);
        output = vfo.output;
        refresh();
        if (devList.size() == 0)
        {
            return;
        }
        setDevice(devList[0]);

        for (int i = 1; i <= 16; i = i * 2)
        {
            sampleRates.push_back(i * 2000000);
        }

        for (int i = 0; i < sampleRates.size(); i++)
        {
            txtSampleRateList += std::to_string((int)sampleRates[i]);
            txtSampleRateList += '\0';
        }

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = output;
        sigpath::sourceManager.registerSource("RX888", &handler);
        spdlog::info("RX888MODULE '{0}': Instance created!", name);
    }

    ~RX888Module()
    {
        if (initialized)
        {
            closeFX3();
        }
        spdlog::info("RX888MODULE '{0}': Instance deleted!", name);
    }

#define SEL0 (8)  		//   SEL0  GPIO26
#define SEL1 (16) 		//   SEL1  GPIO27

    void start()
    {
        uint8_t Bgpio[2];
        Bgpio[0] = 0x17 | SEL0 & (~SEL1);
	    Bgpio[1] = 0x00;

        if (devList.size() == 0)
        {
            return;
        }
        if (running)
        {
            return;
        }

        // start device
        si5351aSetFrequency(ADC_FREQ, 0);
        core::setInputSampleRate(SAMPLE_RATE);

        vfo.start();
        fx3Control(GPIOFX3, Bgpio);

        running = true;
        _workerThread = std::thread(_worker, this);
    }

    void stop()
    {
        if (!running)
        {
            return;
        }
        running = false;
        _workerThread.join();
    }

    void refresh()
    {
        if (running)
        {
            return;
        }

        txtDevList += "HF";
        txtDevList += '\0';
        txtDevList += "VHF";
        txtDevList += '\0';

        devList.push_back((void *)1);
        devList.push_back((void *)2);
    }

    void setSampleRate(float sampleRate)
    {
        _bandWidth = sampleRate;
        vfo.setOutputSampleRate(sampleRate);
        vfo.setBandwidth(sampleRate);
    }

    void setFrequency(float freq)
    {
        freqency = freq;
        vfo.setOffset(freq - SAMPLE_RATE / 4);
    }

    void setGain(int gainId, float gain)
    {
    }

    bool isRunning()
    {
        return running;
    }

    void setDevice(void *devArgs)
    {
        if (running)
        {
            return;
        }

        // TODO: handle HF and VHF
    }

    std::vector<void *> devList;
    std::string txtDevList;
    std::vector<double> sampleRates;
    std::string txtSampleRateList;

    std::vector<std::string> gainList;
    std::vector<range> gainRanges;
    float *currentGains;

    dsp::stream<dsp::complex_t> *output;

private:
    static void _worker(RX888Module *_this)
    {
        _this->worker();
    }

    void worker()
    {
        // block size 131072
        OVERLAPPED inOvLap;
        auto buffer = new short[blockSize * 2];
        auto outbuf = new complex_t[blockSize];

        long pktSize = EndPt->MaxPktSize;
        EndPt->SetXferSize(blockSize * 2);
        long ppx = blockSize * 2 / pktSize;

        inOvLap.hEvent = CreateEvent(NULL, false, false, NULL);

        auto context = EndPt->BeginDataXfer((PUCHAR)buffer, blockSize, &inOvLap);

        fx3Control(STARTFX3);

        while (running)
        {
            LONG rLen = blockSize * 2; // Reset this each time through because
            // FinishDataXfer may modify it
            if (!EndPt->WaitForXfer(&inOvLap, 5000))
            {                   // block on transfer
                EndPt->Abort(); // abort if timeout
                if (EndPt->LastError == ERROR_IO_PENDING)
                    WaitForSingleObject(inOvLap.hEvent, 5000);
                break;
            }

            if (EndPt->Attributes == 2)
            { // BULK Endpoint
                if (EndPt->FinishDataXfer((PUCHAR)buffer, rLen, &inOvLap, context))
                {
                    rLen = rLen / sizeof(short);
#if USE_FS_4
                    int i;
                    int k = 0;
                    for (i = 0; i < rLen - 3;)
                    {
                        outbuf[k].i = (float)buffer[i*2] / 32768.0f;
                        outbuf[k].q = -(float)buffer[i*2 + 1] / 32768.0f;;
                        outbuf[k + 1].i = -(float)buffer[i*2 + 2] / 32768.0f;;
                        outbuf[k + 1].q = (float)buffer[i*2 + 3] / 32768.0f;;
                        k += 2;
                        i += 4;
                    }
                    in.write(outbuf, k);
#else
                    for (int i = 0; i < rLen; i ++)
                    {
                        outbuf[i].q = (float)buffer[i] / 32768.0f;
                        outbuf[i].i = 0;
                    }
                    in.write(outbuf, rLen);
#endif
                }
            }

            context = EndPt->BeginDataXfer((PUCHAR)buffer, blockSize, &inOvLap);
        }

        delete[] buffer;
        delete[] outbuf;
    }

private:
    static void menuSelected(void *ctx)
    {
        RX888Module *_this = (RX888Module *)ctx;
        spdlog::info("RX888Module '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void *ctx)
    {
        RX888Module *_this = (RX888Module *)ctx;
        spdlog::info("RX888Module '{0}': Menu Deselect!", _this->name);
    }

    static void start(void *ctx)
    {
        RX888Module *_this = (RX888Module *)ctx;
        spdlog::info("RX888Module '{0}': Start!", _this->name);
        _this->start();
    }

    static void stop(void *ctx)
    {
        RX888Module *_this = (RX888Module *)ctx;
        spdlog::info("RX888Module '{0}': Stop!", _this->name);
        _this->stop();
    }

    static void tune(double freq, void *ctx)
    {
        RX888Module *_this = (RX888Module *)ctx;
        spdlog::info("RX888Module '{0}': Tune: {1}!", _this->name, freq);
        _this->setFrequency(freq);
    }

    static void menuHandler(void *ctx)
    {
        RX888Module *_this = (RX888Module *)ctx;
        ImGui::Text("Hi from %s!", _this->name.c_str());
    }

    std::string name;
    std::thread _workerThread;
    bool running = false;
    bool initialized = false;
    int samplerateidx = 0;
    int blockSize = 131072 / 2;
    float freqency = 16000000;
    float _bandWidth;

    stream<complex_t> in;
    VFO vfo;

    SourceManager::SourceHandler handler;
};

MOD_EXPORT void _INIT_()
{
    config.setPath(ROOT_DIR "/rx888_source_config.json");
    json defConf;
    defConf["device"] = "";
    defConf["devices"] = json({});
    config.load(defConf);
    config.enableAutoSave();
}

MOD_EXPORT void *_CREATE_INSTANCE_(std::string name)
{
    return new RX888Module(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void *instance)
{
    delete (RX888Module *)instance;
}

MOD_EXPORT void _STOP_()
{
    config.disableAutoSave();
    config.save();
}