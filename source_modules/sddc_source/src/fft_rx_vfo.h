#pragma once
#include <signal_path/signal_path.h>
#include <fftw3.h>
#include <cstring>
#include <algorithm>
#include <volk/volk.h>

#include "pffft/pf_mixer.h"
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
            _outSamplerate = 0;
            halfFft = fftSize / 2;

            _mtunebin = fftSize / 2 / 4;
            GainScale = gain;

            filter = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * halfFft);                  // halfFft
            ADCinTime = (float*)fftwf_malloc(1024 * 1024 * sizeof(float) + sizeof(float) * halfFft); // large enough buffer
            // make sure initial overlap region is zeroed to avoid reading uninitialized samples
            memset(ADCinTime, 0, sizeof(float) * halfFft);

            ADCinFreq = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (halfFft + 1)); // 1024+1
            inFreqTmp = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (halfFft));     // 1024

            plan_t2f_r2c = fftwf_plan_dft_r2c_1d(2 * halfFft, ADCinTime, ADCinFreq, FFTW_PATIENT);
            plan_f2t_c2c = fftwf_plan_dft_1d(halfFft, inFreqTmp, inFreqTmp, FFTW_BACKWARD, FFTW_PATIENT);

            setOutSamplerate(32000000, 16000000);
            base_type::init(in);
        }

        void setGainFactor(float gain) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if (GainScale != gain) {
                generateFreqFilter(gain, _decimationIndex);
            }
        }

        void setInSamplerate(double inSamplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);

            if (_inSamplerate == inSamplerate) {
                return;
            }

            base_type::tempStop();

            _inSamplerate = inSamplerate;
            _decimationIndex = (int)(log2(_inSamplerate / _outSamplerate));
            if (_decimationIndex < 1) {
                _decimationIndex = 1;
                _outSamplerate = _inSamplerate / 2.0;
            }

            _decimationIndex -= 1; // index starts from zero

            // create each filters
            generateFreqFilter(GainScale, _decimationIndex);

            base_type::tempStart();
        }

        void setOutSamplerate(double outSamplerate, double bandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);

            if (_outSamplerate == outSamplerate) {
                return;
            }
            base_type::tempStop();
            _outSamplerate = outSamplerate;
            _decimationIndex = (int)(log2(_inSamplerate / _outSamplerate));
            if (_decimationIndex < 1) {
                _decimationIndex = 1;
                _outSamplerate = _inSamplerate / 2.0;
            }

            _decimationIndex -= 1; // index starts from zero
            generateFreqFilter(GainScale, _decimationIndex);
            base_type::tempStart();
        }

        void setOffset(double offset) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if (offset > _inSamplerate)
                return;

            _lsb = offset < 0;

            if (offset < 0)
                offset = -offset;

            offset = offset / (_inSamplerate / 2.0f);
            // align to 1/4 of halfft
            _mtunebin = int(offset * halfFft / 4) * 4; // mtunebin step 4 bin  ?
            // handle the small freq drift
            float delta = ((float)_mtunebin / halfFft) - offset;
            float fc = delta * (1 << _decimationIndex); // ret increases with higher decimation
            // DbgPrintf("offset %f mtunebin %d delta %f (%f)\n", offset, this->mtunebin, delta, ret);
            if (_lsb) fc = -fc;

            if (this->fc != fc) {
                stateFineTune = shift_limited_unroll_C_sse_init(fc, 0.0F);
                this->fc = fc;
            }
        }

        void reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            base_type::tempStart();
        }

        inline int process(int count, const int16_t* in, complex_t* out) {
            int decimate;
            int mtunebin;
            bool lsb;

            {
                std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
                decimate = _decimationIndex;
                mtunebin = _mtunebin;
                lsb = _lsb;
            }

            // holds the FFT size for the current decimation level
            int mfft = halfFft / (1 << decimate); // = halfFft / 2^mdecimation

            // when arriving here, we have 'count' new samples in 'in'
            // and we have halffft samples overlapped from last time in ADCInTime

            // Convert input into ADCInTime buffer
            convert_float(in, &ADCinTime[halfFft], count);

            base_type::_in->flush();

            // Calculate the parameters for the first half
            size_t shift_count = std::min(mfft / 2, halfFft - mtunebin);
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
            float* real_input = ADCinTime;
            {
                // k == 0, first case, have to do differently as
                // the offset is different, discard samples are different
                {
                    // FFT first stage: time to frequency, real to complex
                    // 'full' transformation size: 2 * halfFft
                    fftwf_execute_dft_r2c(plan_t2f_r2c, real_input, ADCinFreq);
                    real_input += 3 * halfFft / 2;
                    // result now in ADCinFreq[]
                    ADCinFreq[0][0] = 0;
                    ADCinFreq[0][1] = 0;
                    // circular shift (mixing in full bins) and low/bandpass filtering (complex multiplication)
                    {
                        // circular shift tune fs/2 first half array into inFreqTmp[]
                        shift_freq(inFreqTmp, source, filter, shift_count);
                        if (mfft / 2 != shift_count)
                            memset(inFreqTmp[shift_count], 0, sizeof(*inFreqTmp) * (mfft / 2 - shift_count));

                        // circular shift tune fs/2 second half array
                        shift_freq(&dest[start], &source2[start], &filter2[start], mfft / 2 - start);
                        if (start != 0)
                            memset(inFreqTmp[mfft / 2], 0, sizeof(*inFreqTmp) * start);
                    }
                    // result now in inFreqTmp[]

                    // 'shorter' inverse FFT transform (decimation); frequency (back) to COMPLEX time domain
                    // transform size: mfft = mfftdim[k] = halfFft / 2^k with k = mdecimation
                    fftwf_execute_dft(plan_f2t_c2c, inFreqTmp, inFreqTmp); //  c2c decimation
                    // result now in inFreqTmp[]
                }

                // postprocessing
                if (lsb) // lower sideband
                {
                    // mirror just by negating the imaginary Q of complex I/Q
                    copy<true>((fftwf_complex*)out, &inFreqTmp[mfft / 4], mfft / 2);
                }
                else // upper sideband
                {
                    copy<false>((fftwf_complex*)out, &inFreqTmp[mfft / 4], mfft / 2);
                }
                out += mfft / 2;
            }

            int output_step = 3 * mfft / 4;
            for (int k = 1; k < fftPerBuf; k++) {
                // core of fast convolution including filter and decimation
                //   main part is 'overlap-scrap' (IMHO better name for 'overlap-save'), see
                //   https://en.wikipedia.org/wiki/Overlap%E2%80%93save_method
                // FFT first stage: time to frequency, real to complex
                // 'full' transformation size: 2 * halfFft
                fftwf_execute_dft_r2c(plan_t2f_r2c, real_input, ADCinFreq);
                real_input += 3 * halfFft / 2;
                // result now in ADCinFreq[]
                ADCinFreq[0][0] = 0;
                ADCinFreq[0][1] = 0;

                // circular shift (mixing in full bins) and low/bandpass filtering (complex multiplication)
                {
                    // circular shift tune fs/2 first half array into inFreqTmp[]
                    shift_freq(inFreqTmp, source, filter, shift_count);
                    if (mfft / 2 != shift_count)
                        memset(inFreqTmp[shift_count], 0, sizeof(*inFreqTmp) * (mfft / 2 - shift_count));

                    // circular shift tune fs/2 second half array
                    shift_freq(&dest[start], &source2[start], &filter2[start], mfft / 2 - start);
                    if (start != 0)
                        memset(inFreqTmp[mfft / 2], 0, sizeof(*inFreqTmp) * start);
                }
                // result now in inFreqTmp[]

                fftwf_execute_dft(plan_f2t_c2c, inFreqTmp, (fftwf_complex*)out); //  c2c decimation

                // postprocessing
                if (lsb) // lower sideband
                {
                    // mirror just by negating the imaginary Q of complex I/Q
                    int i;
                    for (i = 0; i < output_step; i += 4) {
                        out[i].im = -out[i].im;
                        out[i + 1].im = -out[i + 1].im;
                        out[i + 2].im = -out[i + 2].im;
                        out[i + 3].im = -out[i + 3].im;
                    }
                    for (; i < output_step; i++) {
                        out[i].im = -out[i].im;
                    }
                }
                out += output_step;
                // result now in this->obuffers[]
            }

            {
                // save last overlap samples for next time
                memmove(ADCinTime, &ADCinTime[count], sizeof(*ADCinTime) * halfFft);
            }

            int len = out - origin_output;
            if (this->fc != 0.0f) {
                shift_limited_unroll_C_sse_inp_c((complexf*)origin_output, len, &stateFineTune);
            }

            return len;
        }

        int run() override {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            int outCount = process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            if (outCount > 0) {
                if (!base_type::out.swap(outCount)) { return -1; }
            }
            return outCount;
        }

    protected:
        static inline void convert_float(const int16_t* input, float* output, int count) {
            volk_16i_s32f_convert_32f(output, input, 1.0f, count);
        }

        static inline void shift_freq(fftwf_complex* dest, const fftwf_complex* source1, const fftwf_complex* source2, size_t count) {
            // Use VOLK for complex multiplication
            volk_32fc_x2_multiply_32fc((lv_32fc_t*)(dest), (lv_32fc_t*)(source1), (lv_32fc_t*)(source2), count);
        }

        template <bool flip>
        static inline void copy(fftwf_complex* dest, const fftwf_complex* source, size_t count) {
            if constexpr (!flip) {
                memcpy(dest, source, count * sizeof(fftwf_complex));
            }
            else {
                // VOLK does not provide a direct function to negate imaginary part, so use a loop
                for (size_t i = 0; i < count; i++) {
                    dest[i][0] = source[i][0];
                    dest[i][1] = -source[i][1];
                }
            }
        }

        void generateFreqFilter(float gain, int index) {
            fftwf_plan filterplan_t2f_c2c; // time to frequency fft
            fftwf_complex* pfilterht;      // time filter ht

            pfilterht = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * halfFft); // halfFft
            filterplan_t2f_c2c = fftwf_plan_dft_1d(halfFft, pfilterht, filter, FFTW_FORWARD, FFTW_ESTIMATE);

            float* pht = new float[halfFft / 4 + 1];
            const float Astop = 120.0f;
            const float relPass = 0.85f; // 85% of Nyquist should be usable
            const float relStop = 1.1f;  // 'some' alias back into transition band is OK

            {
                // @todo: have dynamic bandpass filter size - depending on decimation
                //   to allow same stopband-attenuation for all decimations
                float Bw = 64.0f / (1 << index); // bandwidth relative to fs=64MHz
                // Bw *= 0.8f;  // easily visualize Kaiser filter's response
                KaiserWindow(halfFft / 4 + 1, Astop, relPass * Bw / 128.0f, relStop * Bw / 128.0f, pht);

                float gainadj = gain * 2048.0f / (float)(halfFft * 2); // reference is FFTN_R_ADC == 2048

                for (int t = 0; t < halfFft; t++) {
                    pfilterht[t][0] = pfilterht[t][1] = 0.0F;
                }

                for (int t = 0; t < (halfFft / 4 + 1); t++) {
                    pfilterht[halfFft - 1 - t][0] = gainadj * pht[t];
                }

                fftwf_execute_dft(filterplan_t2f_c2c, pfilterht, filter);
            }
            delete[] pht;
            fftwf_destroy_plan(filterplan_t2f_c2c);
            fftwf_free(pfilterht);

            fftwf_destroy_plan(plan_f2t_c2c);
            plan_f2t_c2c = fftwf_plan_dft_1d(halfFft / (1 << index), inFreqTmp, inFreqTmp, FFTW_BACKWARD, FFTW_PATIENT);
        }

        void cleanup() {
            if (inFreqTmp == nullptr) {
                return;
            }

            if (filter != nullptr) {
                fftwf_free(filter);
                filter = nullptr;
            }

            fftwf_destroy_plan(plan_t2f_r2c);
            fftwf_destroy_plan(plan_f2t_c2c);

            fftwf_free(ADCinTime);
            fftwf_free(ADCinFreq);
            fftwf_free(inFreqTmp);
            inFreqTmp = nullptr;
        }

    private:
        // fftwf_complex* filterHw[NDECIDX]; // Hw complex to each decimation ratio
        fftwf_complex* filter = nullptr;
        fftwf_plan plan_f2t_c2c;
        fftwf_plan plan_t2f_r2c; // fftw plan buffers Freq to Time complex to complex per decimation ratio

        float* ADCinTime;         // point to each threads input buffers [nftt]
        fftwf_complex* ADCinFreq; // buffers in frequency
        fftwf_complex* inFreqTmp; // tmp decimation output buffers (after tune shift)

        int halfFft;

        // Hardware scale factor
        float GainScale;

        // Parameters
        double _inSamplerate;
        double _outSamplerate;
        double _bandwidth;
        double _offset;
        int _blockSize;
        int _overlap;
        int _outputSize;
        int _decimationIndex = 1; // the index of decimation, log of 2
        bool _lsb;
        int _mtunebin;

        float fc;
        shift_limited_unroll_C_sse_data_t stateFineTune;
    };
}
