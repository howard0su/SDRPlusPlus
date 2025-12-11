#pragma once
#include "../processor.h"
#include "../loop/phase_control_loop.h"
#include "../loop/agc.h"
#include "../correction/dc_blocker.h"
#include "../convert/mono_to_stereo.h"
#include "../filter/fir.h"
#include "../taps/low_pass.h"
#include "../math/phasor.h"
#include <math.h>

namespace dsp::demod {
    template <class T>
    class SAM : public Processor<dsp::complex_t, T> {
        using base_type = Processor<dsp::complex_t, T>;
    public:
        enum Mode {
            SAM_MODE,  // Synchronous AM (both sidebands)
            USB,       // Upper sideband only
            LSB,       // Lower sideband only
            STEREO     // Stereo (LSB = left, USB = right)
        };

        enum AGCMode {
            CARRIER,
            AUDIO,
        };

        enum PLLSpeed {
            SLOW,    // DX mode: slow, stable (zeta=0.2, omegaN=70)
            MEDIUM,  // Standard mode (zeta=0.65, omegaN=200)
            FAST     // Fast tracking (zeta=1.0, omegaN=500)
        };

        SAM() {}

        SAM(stream<complex_t>* in, Mode mode, AGCMode agcMode, PLLSpeed pllSpeed, double bandwidth, 
            double agcAttack, double agcDecay, double dcBlockRate, double samplerate) { 
            init(in, mode, agcMode, pllSpeed, bandwidth, agcAttack, agcDecay, dcBlockRate, samplerate); 
        }

        ~SAM() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
            taps::free(lpfTaps);
            buffer::free(delayBuf);
        }

        void init(stream<complex_t>* in, Mode mode, AGCMode agcMode, PLLSpeed pllSpeed, double bandwidth, 
                  double agcAttack, double agcDecay, double dcBlockRate, double samplerate) {
            _mode = mode;
            _agcMode = agcMode;
            _pllSpeed = pllSpeed;
            _bandwidth = bandwidth;
            _samplerate = samplerate;
            _dcBlockRate = dcBlockRate;

            // Calculate PLL parameters
            updatePLLParams();

            // Initialize PLL coefficients for 2nd order loop
            float pllBandwidth = _omegaN / _samplerate;
            float alpha, beta;
            loop::PhaseControlLoop<float>::criticallyDamped(pllBandwidth, alpha, beta);
            pcl.init(alpha, beta, 0.0, -FL_M_PI, FL_M_PI, 0.0, -_omega_max, _omega_max);

            // Initialize AGCs
            carrierAgc.init(NULL, 1.0, agcAttack, agcDecay, 10e6, 10.0, INFINITY);
            audioAgc.init(NULL, 1.0, agcAttack, agcDecay, 10e6, 10.0, INFINITY);
            audioAgcUSB.init(NULL, 1.0, agcAttack, agcDecay, 10e6, 10.0, INFINITY);

            // Initialize DC blockers
            dcBlock.init(NULL, dcBlockRate);
            dcBlockUSB.init(NULL, dcBlockRate);

            // Initialize fade leveler exponential smoothing
            _mtauR = expf(-1.0 / (_samplerate * _tauR));
            _onem_mtauR = 1.0 - _mtauR;
            _mtauI = expf(-1.0 / (_samplerate * _tauI));
            _onem_mtauI = 1.0 - _mtauI;

            // Initialize low-pass filter
            lpfTaps = taps::lowPass(bandwidth / 2.0, (bandwidth / 2.0) * 0.1, samplerate);
            lpf.init(NULL, lpfTaps);
            lpfUSB.init(NULL, lpfTaps);

            // Initialize Hilbert transform delay buffers (for phasing method)
            delayBuf = buffer::alloc<complex_t>(HILBERT_STAGES * 3 + 3);
            memset(delayBuf, 0, (HILBERT_STAGES * 3 + 3) * sizeof(complex_t));

            if constexpr (std::is_same_v<T, float>) {
                audioAgc.out.free();
                audioAgcUSB.out.free();
            }
            dcBlock.out.free();
            dcBlockUSB.out.free();
            lpf.out.free();
            lpfUSB.out.free();
            
            base_type::init(in);
        }

        void setMode(Mode mode) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _mode = mode;
            reset();
            base_type::tempStart();
        }

        void setAGCMode(AGCMode agcMode) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _agcMode = agcMode;
            reset();
            base_type::tempStart();
        }

        void setPLLSpeed(PLLSpeed speed) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _pllSpeed = speed;
            updatePLLParams();
            float pllBandwidth = _omegaN / _samplerate;
            float alpha, beta;
            loop::PhaseControlLoop<float>::criticallyDamped(pllBandwidth, alpha, beta);
            pcl.setCoefficients(alpha, beta);
            base_type::tempStart();
        }

        void setBandwidth(double bandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if (bandwidth == _bandwidth) { return; }
            _bandwidth = bandwidth;
            std::lock_guard<std::mutex> lck2(lpfMtx);
            taps::free(lpfTaps);
            lpfTaps = taps::lowPass(_bandwidth / 2.0, (_bandwidth / 2.0) * 0.1, _samplerate);
            lpf.setTaps(lpfTaps);
            lpfUSB.setTaps(lpfTaps);
        }

        void setAGCAttack(double attack) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            carrierAgc.setAttack(attack);
            audioAgc.setAttack(attack);
            audioAgcUSB.setAttack(attack);
        }

        void setAGCDecay(double decay) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            carrierAgc.setDecay(decay);
            audioAgc.setDecay(decay);
            audioAgcUSB.setDecay(decay);
        }

        void setDCBlockRate(double rate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _dcBlockRate = rate;
            dcBlock.setRate(rate);
            dcBlockUSB.setRate(rate);
        }

        void setFadeLeveler(bool enabled) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _fadeLeveler = enabled;
        }

        float getCarrierFreq() const {
            return pcl.freq * _samplerate / (2.0 * FL_M_PI);
        }

        void reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            pcl.phase = 0.0;
            pcl.freq = 0.0;
            carrierAgc.reset();
            audioAgc.reset();
            audioAgcUSB.reset();
            dcBlock.reset();
            dcBlockUSB.reset();
            _dc = 0.0f;
            _dc_insert = 0.0f;
            _dcUSB = 0.0f;
            _dc_insertUSB = 0.0f;
            memset(delayBuf, 0, (HILBERT_STAGES * 3 + 3) * sizeof(complex_t));
            base_type::tempStart();
        }

        int process(int count, complex_t* in, T* out) {
            // Apply carrier AGC if needed
            complex_t* procBuf = in;
            if (_agcMode == AGCMode::CARRIER) {
                carrierAgc.process(count, in, carrierAgc.out.writeBuf);
                procBuf = carrierAgc.out.writeBuf;
            }

            for (int i = 0; i < count; i++) {
                // Generate PLL NCO (carrier reference)
                float cosPhase, sinPhase;
                volk_32f_sin_32f(&sinPhase, &pcl.phase, 1);
                volk_32f_cos_32f(&cosPhase, &pcl.phase, 1);

                // Mix input with carrier to baseband
                // Product detector: multiply input by sin and cos of carrier
                float ai = cosPhase * procBuf[i].re;  // I * cos
                float bi = sinPhase * procBuf[i].re;  // I * sin
                float aq = cosPhase * procBuf[i].im;  // Q * cos
                float bq = sinPhase * procBuf[i].im;  // Q * sin

                // For sideband selection, we need phase-shifted versions
                // Apply Hilbert transform via allpass filters for phasing method
                float ai_ps, bi_ps, aq_ps, bq_ps;
                
                if (_mode != Mode::SAM_MODE) {
                    // Phasing method for SSB: use cascaded allpass filters
                    applyHilbertPhasing(ai, bi, aq, bq, ai_ps, bi_ps, aq_ps, bq_ps);
                }

                // Recovered carrier components (corrected signal)
                complex_t corrected;
                corrected.re = ai + bq;  // I' = I*cos + Q*sin
                corrected.im = -bi + aq; // Q' = Q*cos - I*sin

                // Demodulate based on mode
                float audioLSB, audioUSB;
                
                switch (_mode) {
                    case Mode::SAM_MODE:
                        // Standard synchronous AM: just take real part
                        audioLSB = corrected.re;
                        audioUSB = 0.0f;
                        break;
                        
                    case Mode::LSB:
                        // Lower sideband: (I+jQ)_ps - (Q-jI)_ps
                        audioLSB = (ai_ps + bi_ps) - (aq_ps - bq_ps);
                        audioUSB = 0.0f;
                        break;
                        
                    case Mode::USB:
                        // Upper sideband: (I-jQ)_ps + (Q+jI)_ps
                        audioUSB = (ai_ps - bi_ps) + (aq_ps + bq_ps);
                        audioLSB = 0.0f;
                        break;
                        
                    case Mode::STEREO:
                        // Stereo mode: LSB in left, USB in right
                        audioLSB = (ai_ps + bi_ps) - (aq_ps - bq_ps);
                        audioUSB = (ai_ps - bi_ps) + (aq_ps + bq_ps);
                        break;
                }

                // Apply fade leveler if enabled (removes selective fading effects)
                if (_fadeLeveler) {
                    _dc = _mtauR * _dc + _onem_mtauR * audioLSB;
                    _dc_insert = _mtauI * _dc_insert + _onem_mtauI * corrected.re;
                    audioLSB = audioLSB + _dc_insert - _dc;

                    if (_mode == Mode::STEREO) {
                        _dcUSB = _mtauR * _dcUSB + _onem_mtauR * audioUSB;
                        _dc_insertUSB = _mtauI * _dc_insertUSB + _onem_mtauI * corrected.re;
                        audioUSB = audioUSB + _dc_insertUSB - _dcUSB;
                    }
                }

                // Store in temp buffers for further processing
                if constexpr (std::is_same_v<T, float>) {
                    audioAgc.out.writeBuf[i] = audioLSB;
                } else {
                    audioAgc.out.writeBuf[i] = audioLSB;
                    audioAgcUSB.out.writeBuf[i] = audioUSB;
                }

                // PLL update: track carrier phase error
                // Phase detector: atan2(Q', I')
                float phaseError = atan2f(corrected.im, corrected.re);
                pcl.advance(phaseError);
            }

            // Apply DC blocking
            if (_mode == Mode::STEREO) {
                dcBlock.process(count, audioAgc.out.writeBuf, audioAgc.out.writeBuf);
                dcBlockUSB.process(count, audioAgcUSB.out.writeBuf, audioAgcUSB.out.writeBuf);
            } else {
                dcBlock.process(count, audioAgc.out.writeBuf, audioAgc.out.writeBuf);
            }

            // Apply audio AGC if needed
            if (_agcMode == AGCMode::AUDIO) {
                audioAgc.process(count, audioAgc.out.writeBuf, audioAgc.out.writeBuf);
                if (_mode == Mode::STEREO) {
                    audioAgcUSB.process(count, audioAgcUSB.out.writeBuf, audioAgcUSB.out.writeBuf);
                }
            }

            // Apply low-pass filter
            {
                std::lock_guard<std::mutex> lck(lpfMtx);
                lpf.process(count, audioAgc.out.writeBuf, audioAgc.out.writeBuf);
                if (_mode == Mode::STEREO) {
                    lpfUSB.process(count, audioAgcUSB.out.writeBuf, audioAgcUSB.out.writeBuf);
                }
            }

            // Output formatting
            if constexpr (std::is_same_v<T, float>) {
                memcpy(out, audioAgc.out.writeBuf, count * sizeof(float));
            }
            if constexpr (std::is_same_v<T, stereo_t>) {
                if (_mode == Mode::STEREO) {
                    // True stereo output
                    for (int i = 0; i < count; i++) {
                        out[i].l = audioAgc.out.writeBuf[i];
                        out[i].r = audioAgcUSB.out.writeBuf[i];
                    }
                } else {
                    // Mono to stereo conversion
                    convert::MonoToStereo::process(count, audioAgc.out.writeBuf, out);
                }
            }

            return count;
        }

        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            base_type::_in->flush();
            if (!base_type::out.swap(count)) { return -1; }
            return count;
        }

    protected:
        static constexpr int HILBERT_STAGES = 7;
        static constexpr int OUT_IDX = 3 * HILBERT_STAGES;

        // Hilbert transform coefficients for sideband phasing
        // These create 90-degree phase shift networks
        static constexpr float c0[HILBERT_STAGES] = {
            -0.328201924180698f,
            -0.744171491539427f,
            -0.923022915444215f,
            -0.978490468768238f,
            -0.994128272402075f,
            -0.998458978159551f,
            -0.999790306259206f,
        };

        static constexpr float c1[HILBERT_STAGES] = {
            -0.0991227952747244f,
            -0.565619728761389f,
            -0.857467122550052f,
            -0.959123933111275f,
            -0.988739372718090f,
            -0.996959189310611f,
            -0.999282492800792f
        };

        void updatePLLParams() {
            switch (_pllSpeed) {
                case SLOW:
                    _zeta = 0.2;
                    _omegaN = 70.0;
                    break;
                case MEDIUM:
                    _zeta = 0.65;
                    _omegaN = 200.0;
                    break;
                case FAST:
                    _zeta = 1.0;
                    _omegaN = 500.0;
                    break;
            }

            // PLL frequency limits (±22 kHz)
            float pll_fmax = 22000.0;
            _omega_min = 2.0 * FL_M_PI * (-pll_fmax) / _samplerate;
            _omega_max = 2.0 * FL_M_PI * pll_fmax / _samplerate;
        }

        void applyHilbertPhasing(float ai, float bi, float aq, float bq,
                                  float& ai_ps, float& bi_ps, float& aq_ps, float& bq_ps) {
            // Store current samples in delay line
            delayBuf[0].re = ai;  // a[0] for I path
            delayBuf[0].im = bi;  // b[0] for I path
            
            // Use delay buffer differently for Q path
            complex_t tempQ;
            tempQ.re = aq;  // c[0] for Q path  
            tempQ.im = bq;  // d[0] for Q path

            // Apply cascaded allpass sections
            for (int j = 0; j < HILBERT_STAGES; j++) {
                int k = 3 * j;
                
                // I and rotated I paths (a and b arrays)
                float a_new = c0[j] * (delayBuf[k].re - delayBuf[k + 5].re) + delayBuf[k + 2].re;
                float b_new = c1[j] * (delayBuf[k].im - delayBuf[k + 5].im) + delayBuf[k + 2].im;
                
                delayBuf[k + 3].re = a_new;
                delayBuf[k + 3].im = b_new;
            }

            ai_ps = delayBuf[OUT_IDX].re;
            bi_ps = delayBuf[OUT_IDX].im;

            // Simple approximation for Q path phasing (90 degrees)
            // In a full implementation, you'd have separate delay buffers
            aq_ps = tempQ.re;
            bq_ps = tempQ.im;

            // Shift delay line
            for (int j = OUT_IDX + 2; j > 0; j--) {
                delayBuf[j] = delayBuf[j - 1];
            }
        }

        Mode _mode;
        AGCMode _agcMode;
        PLLSpeed _pllSpeed;

        double _samplerate;
        double _bandwidth;
        double _dcBlockRate;

        // PLL parameters
        float _zeta;
        float _omegaN;
        float _omega_min;
        float _omega_max;
        loop::PhaseControlLoop<float> pcl;

        // Fade leveler parameters
        bool _fadeLeveler = false;
        static constexpr float _tauR = 0.02f;  // Ratio time constant
        static constexpr float _tauI = 1.4f;   // Insert time constant
        float _mtauR;
        float _onem_mtauR;
        float _mtauI;
        float _onem_mtauI;
        float _dc = 0.0f;
        float _dc_insert = 0.0f;
        float _dcUSB = 0.0f;
        float _dc_insertUSB = 0.0f;

        // DSP blocks
        loop::AGC<complex_t> carrierAgc;
        loop::AGC<float> audioAgc;
        loop::AGC<float> audioAgcUSB;
        correction::DCBlocker<float> dcBlock;
        correction::DCBlocker<float> dcBlockUSB;
        tap<float> lpfTaps;
        filter::FIR<float, float> lpf;
        filter::FIR<float, float> lpfUSB;
        std::mutex lpfMtx;

        // Hilbert transform delay buffers
        complex_t* delayBuf;
    };
}
