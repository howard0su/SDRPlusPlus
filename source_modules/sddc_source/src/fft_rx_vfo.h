#pragma once
#include <signal_path/signal_path.h>
#include <fftw3.h>
#include <cstring>
#include <algorithm>

#include "kaiser.h"

#define NDECIDX 7 // Support decimation ratios: 2,4,8,16,32,64,128

namespace dsp::channel {
    /**
     * FFT-based RX VFO using overlap-save method for high-performance processing
     * - Input: Real (float) samples
     * - Output: Complex (IQ) samples
     * - Uses FFTW for efficient FFT/IFFT operations
     * - Minimum decimation rate of 2
     * - Optimized for high sample rates (up to 64M samples/s)
     * 
     * The overlap-save method performs:
     * 1. Real-to-complex FFT on input blocks
     * 2. Frequency domain shift and filtering
     * 3. Complex IFFT with decimation
     */
    class FFTRxVFO : public Processor<int16_t, complex_t> {
        using base_type = Processor<int16_t, complex_t>;
    public:
        FFTRxVFO() {}

        ~FFTRxVFO() override {
            if (!base_type::_block_init) { return; }
            base_type::stop();
            cleanup();
        }

        void init(stream<int16_t>* in, float gain, int fftSize = 8192) {
            _inSamplerate = 64000000;
            _outSamplerate = 32000000;
            _fftSize = fftSize;

            mratio[0] = 1;  // 1,2,4,8,16
            mfftdim[0] = fftSize / 2;
            for (int i = 1; i < NDECIDX; i++)
            {
                mratio[i] = mratio[i - 1] * 2;
                mfftdim[i] = mfftdim[i - 1] / 2;
            }
            
            _mtunebin = fftSize / 2 / 4;

            // create each filters
            generateFreqFilters(gain);

            decimate_count = 0;

            base_type::init(in);
        }

        void setInSamplerate(double inSamplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            
            _inSamplerate = inSamplerate;
            _decimationIndex = (int)(log2(_inSamplerate / _outSamplerate));
            if (_decimationIndex < 1) {
                _decimationIndex = 1;
                _outSamplerate = _inSamplerate / 2.0;
            }
            
            _decimationIndex -= 1; // index starts from zero
            base_type::tempStart();
        }

        void setOutSamplerate(double outSamplerate, double bandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            
            _outSamplerate = outSamplerate;
            _decimationIndex = (int)(log2(_inSamplerate / _outSamplerate));
            if (_decimationIndex < 1) {
                _decimationIndex = 1;
                _outSamplerate = _inSamplerate / 2.0;
            }

            _decimationIndex -= 1; // index starts from zero
            base_type::tempStart();
        }

        void setOffset(double offset) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            std::lock_guard<std::mutex> lck2(_filterMtx);

            _lsb = offset < 0;

            if (offset < 0)
                offset = -offset;

            offset = offset / _inSamplerate / 2.0f;
            // align to 1/4 of halfft
            int halfFft = _fftSize / 2;
            _mtunebin = int(offset * halfFft / 4) * 4;  // mtunebin step 4 bin  ?
            // TODO: how to handle the small freq drift
            // float delta = ((float)this->mtunebin  / halfFft) - offset;
            // float ret = delta * getRatio(); // ret increases with higher decimation
            // DbgPrintf("offset %f mtunebin %d delta %f (%f)\n", offset, this->mtunebin, delta, ret);
            // return ret;
        }

        void reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            decimate_count = 0;
            base_type::tempStart();
        }

        inline int process(int count, const int16_t* in, complex_t* out) {
            int decimate;
            int mtunebin;
            
            {
                std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
                std::lock_guard<std::mutex> lck2(_filterMtx);
                decimate= _decimationIndex;
                mtunebin = _mtunebin;
            }

            int halfFft = _fftSize / 2;
            fftwf_plan *plan_f2t_c2c;          // fftw plan buffers Time to Freq real to complex per buffer
            plan_f2t_c2c = &plans_f2t_c2c[decimate];

            // holds the FFT size for the current decimation level
        	int mfft = this->mfftdim[decimate];	// = halfFft / 2^mdecimation
	        fftwf_complex* filter = filterHw[decimate];

            // when arriving here, we have 'count' new samples in 'in'
            // and we have halffft samples overlapped from last time in ADCInTime

            // Convert input into ADCInTime buffer
            convert_float(in, &ADCinTime[halfFft], count);
            
            // Calculate the parameters for the first half
		    auto shift_count = std::min(mfft/2, halfFft - mtunebin);
            auto source = &ADCinFreq[mtunebin];
            // Calculate the parameters for the second half
            auto start = std::max(0, mfft / 2 - mtunebin);
            auto source2 = &ADCinFreq[mtunebin - mfft / 2];
            auto dest = &inFreqTmp[mfft / 2];
            auto filter2 = &filter[halfFft - mfft / 2];
            complex_t* origin_output = out; 

            // calcuate how many times we should run fft/ifft pair
            // we can estimate how many output samples we can get
            int fftPerBuf = count / (3 * halfFft / 2) + 1; // number of ffts per buffer with 256|768 overlap

            for(int k = 0; k < fftPerBuf; k++)
            {
                // core of fast convolution including filter and decimation
			    //   main part is 'overlap-scrap' (IMHO better name for 'overlap-save'), see
			    //   https://en.wikipedia.org/wiki/Overlap%E2%80%93save_method
                {
                    // FFT first stage: time to frequency, real to complex
				    // 'full' transformation size: 2 * halfFft
                    fftwf_execute_dft_r2c(plan_t2f_r2c, ADCinTime + (3 * halfFft / 2) * k, ADCinFreq);
                    // result now in ADCinFreq[]
                    
                    // circular shift (mixing in full bins) and low/bandpass filtering (complex multiplication)
                    {
                        // circular shift tune fs/2 first half array into inFreqTmp[]
                        shift_freq(inFreqTmp, source, filter, 0, shift_count);
                        if (mfft / 2 != shift_count)
                            memset(inFreqTmp[shift_count], 0, sizeof(float) * 2 * (mfft / 2 - shift_count));

                        // circular shift tune fs/2 second half array
                        shift_freq(dest, source2, filter2, start, mfft/2);
                        if (start != 0)
                            memset(inFreqTmp[mfft / 2], 0, sizeof(float) * 2 * start);
                    }
                    // result now in inFreqTmp[]

                    // 'shorter' inverse FFT transform (decimation); frequency (back) to COMPLEX time domain
                    // transform size: mfft = mfftdim[k] = halfFft / 2^k with k = mdecimation
                    fftwf_execute_dft(*plan_f2t_c2c, inFreqTmp, inFreqTmp);     //  c2c decimation
                    // result now in inFreqTmp[]
                }

                // postprocessing
                if (_lsb) // lower sideband
                {
                    // mirror just by negating the imaginary Q of complex I/Q
                    if (k == 0)
                    {
                        copy<true>((fftwf_complex*)out, &inFreqTmp[mfft / 4], mfft/2);
                        out += (mfft / 2);
                    }
                    else
                    {
                        copy<true>((fftwf_complex*)out, &inFreqTmp[0], (3 * mfft / 4));
                        out += (3 * mfft / 4);
                    }
                }
                else // upper sideband
                {
                    if (k == 0)
                    {
                        copy<false>((fftwf_complex*)out, &inFreqTmp[mfft / 4], mfft/2);
                        out += (mfft / 2);
                    }
                    else
                    {
                        copy<false>((fftwf_complex*)out, &inFreqTmp[0], (3 * mfft / 4));
                        out += (3 * mfft / 4);
                    }
                }
                // result now in this->obuffers[]
            }

            {
                // save last overlap samples for next time
                memmove(ADCinTime, &ADCinTime[count - halfFft], sizeof(float) * halfFft);
            }

            return out - origin_output;
        }

        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            int outCount = process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            base_type::_in->flush();
            if (outCount > 0) {
                if (!base_type::out.swap(outCount)) { return -1; }
            }
            return outCount;
        }

    protected:
        static void convert_float(const int16_t *input, float* output, int size)
        {
            for(int m = 0; m < size; m++)
            {
                output[m] = float(input[m]);
            }
        }

        static void shift_freq(fftwf_complex* dest, const fftwf_complex* source1, const fftwf_complex* source2, int start, int end)
        {
            for (int m = start; m < end; m++)
            {
                // besides circular shift, do complex multiplication with the lowpass filter's spectrum
                dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
                dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
            }
        }

        template<bool flip> static void copy(fftwf_complex* dest, const fftwf_complex* source, int count)
        {
            if (flip)
            {
                for (int i = 0; i < count; i++)
                {
                    dest[i][0] = source[i][0];
                    dest[i][1] = -source[i][1];
                }
            }
            else
            {
                for (int i = 0; i < count; i++)
                {
                    dest[i][0] = source[i][0];
                    dest[i][1] = source[i][1];
                }
            }
        }

        void generateFreqFilters(float gain) {
            int halfFft = _fftSize / 2;
            fftwf_plan filterplan_t2f_c2c; // time to frequency fft

            // filters
            fftwf_complex *pfilterht;       // time filter ht
            pfilterht = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*halfFft);     // halfFft
            filterHw = (fftwf_complex**)fftwf_malloc(sizeof(fftwf_complex*)*NDECIDX);
            for (int d = 0; d < NDECIDX; d++)
            {
                filterHw[d] = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*halfFft);     // halfFft
            }

            filterplan_t2f_c2c = fftwf_plan_dft_1d(halfFft, pfilterht, filterHw[0], FFTW_FORWARD, FFTW_MEASURE);
            float *pht = new float[halfFft / 4 + 1];
            const float Astop = 120.0f;
            const float relPass = 0.85f;  // 85% of Nyquist should be usable
            const float relStop = 1.1f;   // 'some' alias back into transition band is OK
            for (int d = 0; d < NDECIDX; d++)	// @todo when increasing NDECIDX
            {
                // @todo: have dynamic bandpass filter size - depending on decimation
                //   to allow same stopband-attenuation for all decimations
                float Bw = 64.0f / mratio[d];
                // Bw *= 0.8f;  // easily visualize Kaiser filter's response
                KaiserWindow(halfFft / 4 + 1, Astop, relPass * Bw / 128.0f, relStop * Bw / 128.0f, pht);

                float gainadj = gain * 2048.0f / (float)_fftSize; // reference is FFTN_R_ADC == 2048

                for (int t = 0; t < halfFft; t++)
                {
                    pfilterht[t][0] = pfilterht[t][1]= 0.0F;
                }
            
                for (int t = 0; t < (halfFft/4+1); t++)
                {
                    pfilterht[halfFft-1-t][0] = gainadj * pht[t];
                }

                fftwf_execute_dft(filterplan_t2f_c2c, pfilterht, filterHw[d]);
            }
            delete[] pht;
            fftwf_destroy_plan(filterplan_t2f_c2c);
            fftwf_free(pfilterht);

            ADCinTime = (float*)fftwf_malloc(1024 * 1024 * sizeof(float) + sizeof(float) * halfFft); // large enough buffer

            ADCinFreq = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*(halfFft + 1)); // 1024+1
            inFreqTmp = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*(halfFft));    // 1024

            plan_t2f_r2c = fftwf_plan_dft_r2c_1d(2 * halfFft, ADCinTime, ADCinFreq, FFTW_MEASURE);
            for (int d = 0; d < NDECIDX; d++)
            {
                plans_f2t_c2c[d] = fftwf_plan_dft_1d(mfftdim[d], inFreqTmp, inFreqTmp, FFTW_BACKWARD, FFTW_MEASURE);
            }
        }

        void cleanup() {
            if (filterHw == nullptr)
                return;

            for (int d = 0; d < NDECIDX; d++)
            {
                fftwf_free(filterHw[d]);     // 4096
            }
            fftwf_free(filterHw);
            filterHw = nullptr;

            fftwf_destroy_plan(plan_t2f_r2c);
            for (int d = 0; d < NDECIDX; d++)
            {
                fftwf_destroy_plan(plans_f2t_c2c[d]);
            }

            fftwf_free(ADCinTime);
            fftwf_free(ADCinFreq);
            fftwf_free(inFreqTmp);
        }

private:
        int mratio[NDECIDX];  // ratio dimentions: mratio[k] = 2^k
        int mfftdim[NDECIDX]; // FFT N dimensions: mfftdim[k] = halfFft / 2^k

        float GainScale;

        uint32_t decimate_count;

        fftwf_complex **filterHw;       // Hw complex to each decimation ratio

        fftwf_plan plan_t2f_r2c;          // fftw plan buffers Freq to Time complex to complex per decimation ratio
        fftwf_plan plans_f2t_c2c[NDECIDX];

        float *ADCinTime;                // point to each threads input buffers [nftt]
	    fftwf_complex *ADCinFreq;         // buffers in frequency
    	fftwf_complex *inFreqTmp;         // tmp decimation output buffers (after tune shift)

        // Parameters
        double _inSamplerate;
        double _outSamplerate;
        double _bandwidth;
        double _offset;
        int _fftSize;
        int _blockSize;
        int _overlap;
        int _outputSize;
        int _decimationIndex; // the index of decimation, log of 2
        bool _lsb;
        int _mtunebin;
        
        std::mutex _filterMtx;
    };
}
