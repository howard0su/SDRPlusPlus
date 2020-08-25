
#pragma once
#include <string>
#include <dsp/stream.h>
#include <dsp/types.h>
#include <dsp/math.h>
#include <spdlog/spdlog.h>

#include "openFX3.h"
#include "Si5351.h"

#define ADC_FREQ 64000000

typedef struct _range {
    float _minimum;
    float _maximum;

    float minimum() { return _minimum; }
    float maximum() { return _maximum; }
}range;

using namespace dsp;

namespace io {
    class SoapyWrapper {
        public:
        SoapyWrapper() {
            if (!openFX3())
            {
                return;
            }

            Si5351init();
            initialized = true;
            currentGains = new float[1];

            in.init(64000);
            vfo.init(&in, ADC_FREQ/2, ADC_FREQ/4, ADC_FREQ/4, 0, blockSize);
            output = vfo.output;
            refresh();
            if (devList.size() == 0) {
                return;
            }
            setDevice(devList[0]);

            for (int i = 1; i <= 16; i = i * 2) {
                sampleRates.push_back(i * 2000000);
            }

            for (int i = 0; i < sampleRates.size(); i++) {
                txtSampleRateList += std::to_string((int)sampleRates[i]);
                txtSampleRateList += '\0';
            }
        }

        ~SoapyWrapper()
        {
            if (initialized) {
                closeFX3();
            }
        }

        void start()
        {
            if (devList.size() == 0) {
                return;
            }
            if (running) {
                return;
            }

            // start device
            si5351aSetFrequency(ADC_FREQ, 0);

            vfo.start();

            running = true;
            _workerThread = std::thread(_worker, this);
        }

        void stop()
        {
            if (!running) {
                return;
            }
            running = false;
            _workerThread.join();
        }

        void refresh()
        {
            if (running) {
                return;
            }
            
            txtDevList += "HF";
            txtDevList += '\0';
            txtDevList += "VHF";
            txtDevList += '\0';

            devList.push_back((void*)1);
            devList.push_back((void*)2);
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
            vfo.setOffset(freq - ADC_FREQ / 4);
        }

        void setGain(int gainId, float gain)
        {
        }

        bool isRunning()
        {
            return running;
        }

        void setDevice(void* devArgs)
        {
            if (running) {
                return;
            }

            // TODO: handle HF and VHF
        }

        std::vector<void*> devList;
        std::string txtDevList;
        std::vector<double> sampleRates;
        std::string txtSampleRateList;

        std::vector<std::string> gainList;
        std::vector<range> gainRanges;
        float* currentGains;

        dsp::stream<dsp::complex_t> *output;
 
        private:
        static void _worker(SoapyWrapper* _this) {
            _this->worker();
        }

        void worker() {
            // block size 131072
            OVERLAPPED inOvLap;
            auto buffer = new unsigned short[blockSize * 2];
            auto outbuf = new complex_t[blockSize];

            long pktSize = EndPt->MaxPktSize;
            EndPt->SetXferSize(blockSize * 2);
            long ppx = blockSize * 2/ pktSize;

		    inOvLap.hEvent = CreateEvent(NULL, false, false, NULL);

            auto context = EndPt->BeginDataXfer((PUCHAR)buffer, blockSize, &inOvLap);

            fx3Control(STARTFX3);

            while(running) {
                LONG rLen = blockSize * 2;	// Reset this each time through because
        		// FinishDataXfer may modify it
		        if (!EndPt->WaitForXfer(&inOvLap, 5000)) { // block on transfer
			        EndPt->Abort(); // abort if timeout
			        if (EndPt->LastError == ERROR_IO_PENDING)
				        WaitForSingleObject(inOvLap.hEvent, 5000);
			        break;
		        }
#define DDC_LPF 

                if (EndPt->Attributes == 2) { // BULK Endpoint
                    if (EndPt->FinishDataXfer((PUCHAR)buffer, rLen, &inOvLap, context)) {
#ifdef DDC_LPF
                    rLen = rLen / sizeof(unsigned short);
                    for (int i = 0; i < rLen; i++)
                    {
                        switch(i & 0x03) {
                        case 0:
                            outbuf[i].i = (float)(buffer[i] - 32767)/65535.0;
                            outbuf[i].q = 0.0f;
                            break;
                        case 1:
                            outbuf[i].i = 0.0f;
                            outbuf[i].q = -(float)(buffer[i] - 32767)/65535.0;
                            break;
                        case 2:
                            outbuf[i].i = -(float)(buffer[i] - 32767)/65535.0;
                            outbuf[i].q = 0.0f;
                            break;
                        case 3:
                            outbuf[i].i = 0.0f;
                            outbuf[i].q = (float)(buffer[i] - 32767)/65535.0;
                            break;
                        }
                    }
                    in.write(outbuf, rLen);
#endif
#ifdef DDC_BERCILE
#endif
                    }
                }

                context = EndPt->BeginDataXfer((PUCHAR)buffer, blockSize, &inOvLap);
            }

            delete[] buffer;
            delete[] outbuf;
        }

        std::thread _workerThread;
        bool running = false;
        bool initialized = false;
        int samplerateidx = 0;
        int blockSize = 131072 / 2;
        float freqency = 16000000;
        float _bandWidth;

        stream<complex_t> in;
        VFO vfo;
    };
}