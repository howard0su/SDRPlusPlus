#pragma once
#include "../processor.h"
#include "../buffer/buffer.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <volk/volk.h>

/*
Implement the algorithm described in:
https://rand.pepabo.com/papers/ieicesr202111-jj1bdx-mpfilter.pdf

SDRImplementation of Analog FM Broadcast Multipath Filter
*/

namespace dsp::filter {
    class MultipathFIR : public Processor<complex_t, complex_t> {
        using base_type = Processor<complex_t, complex_t>;
    public:
        MultipathFIR() {}

        MultipathFIR(stream<complex_t>* in, unsigned int stages, float targetLevel = 1.0f, float alpha = 0.01f, uint32_t updateMask = 0x03u) {
            init(in, stages, targetLevel, alpha, updateMask);
        }

        ~MultipathFIR() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
            if (stateBuffer) { buffer::free(stateBuffer); }
            if (coeffBuffer) { buffer::free(coeffBuffer); }
            if (magScratchBuffer) { buffer::free(magScratchBuffer); }
        }

        void init(stream<complex_t>* in, unsigned int stages, float targetLevel = 1.0f, float alpha = 0.01f, uint32_t updateMask = 0x03u) {
            _targetLevel = targetLevel;
            _alpha = alpha;
            _updateMask = updateMask;

            unsigned int finalStages = (stages > 0) ? stages : 1;
            setStageCountInternal(finalStages);
            base_type::init(in);
        }

        void setStages(unsigned int stages) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            unsigned int finalStages = (stages > 0) ? stages : 1;
            setStageCountInternal(finalStages);
            base_type::tempStart();
        }

        void setAlpha(float alpha) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _alpha = alpha;
        }

        void setTargetLevel(float targetLevel) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _targetLevel = targetLevel;
        }

        void setUpdateMask(uint32_t updateMask) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _updateMask = updateMask;
        }

        void reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            resetState();
            base_type::tempStart();
        }

        inline int process(int count, const complex_t* in, complex_t* out) {
            if (!_filterOrder) { return count; }

            for (int i = 0; i < count; i++) {
                complex_t y = singleProcess(in[i]);
                out[i] = y;

                if (((_sampleCount++) & _updateMask) == 0u) {
                    updateCoefficients(y);
                }
            }
            return count;
        }

        DEFAULT_PROC_RUN

    private:

        inline void updateSignalPresence(const std::complex<float>& x)
        {
            // envelope^2
            float mag2 = std::norm(x);

            // IIR Smooth（about 5~10 ms window, alpha 0.001 at fs=384kHz）
            constexpr float alpha = 0.001f;

            sp_energy_avg += alpha * (mag2 - sp_energy_avg);

            float mag = std::sqrt(mag2);
            sp_env_avg += alpha * (mag - sp_env_avg);

            float diff = mag - sp_env_avg;
            sp_env_var += alpha * ((diff * diff) - sp_env_var);

            if (++sp_count >= 256) {
                sp_count = 0;

                constexpr float ENERGY_MIN = 1e-4f;
                constexpr float VAR_MAX    = 0.6f;

                signal_present =
                    (sp_energy_avg > ENERGY_MIN) &&
                    (sp_env_var    < VAR_MAX);
            }
        }

        void setStageCountInternal(unsigned int stages) {
            _stages = stages;
            _filterOrder = (_stages * 4u) + 1u;
            _refIndex = (_stages * 3u) + 1u;
            if (_refIndex >= _filterOrder) {
                _refIndex = (_filterOrder ? _filterOrder - 1u : 0u);
            }

            // Free old buffers
            if (stateBuffer) { buffer::free(stateBuffer); }
            if (coeffBuffer) { buffer::free(coeffBuffer); }
            if (magScratchBuffer) { buffer::free(magScratchBuffer); }

            // Allocate new buffers
            stateBuffer = buffer::alloc<complex_t>(_filterOrder);
            coeffBuffer = buffer::alloc<complex_t>(_filterOrder);
            magScratchBuffer = buffer::alloc<float>(_filterOrder);

            resetState();
        }

        void resetState() {
            if (!_filterOrder || !stateBuffer || !coeffBuffer) { return; }

            buffer::clear(stateBuffer, _filterOrder);
            buffer::clear(coeffBuffer, _filterOrder);
            _sampleCount = 0;
            _lastError = 0.0f;

            coeffBuffer[_refIndex] = complex_t{ 1.0f, 0.0f };
            _mu = _alpha / static_cast<float>(_filterOrder);
        }

        inline complex_t singleProcess(const complex_t& sample) {
            memmove(stateBuffer, stateBuffer + 1, (_filterOrder - 1) * sizeof(complex_t));
            stateBuffer[_filterOrder - 1] = sample;

            complex_t output{};
            volk_32fc_x2_dot_prod_32fc((lv_32fc_t*)&output, (lv_32fc_t*)stateBuffer, (lv_32fc_t*)coeffBuffer, _filterOrder);

            if (!std::isfinite(output.re) || !std::isfinite(output.im)) {
                output = complex_t{ 0.0f, 0.0f };
            }

            return output;
        }

        inline void updateCoefficients(const complex_t& y) {
            volk_32fc_magnitude_squared_32f(magScratchBuffer, (lv_32fc_t*)stateBuffer, _filterOrder);
            float magSum = 0.0f;
            volk_32f_accumulator_s32f(&magSum, magScratchBuffer, _filterOrder);

            _mu = _alpha / (magSum + 1e-10f);

            const float env = (y.re * y.re) + (y.im * y.im);
            _lastError = _targetLevel - env;
            const float factor = _lastError * _mu;
            const complex_t factorTimesResult{ factor * y.re, factor * y.im };

#if VOLK_VERSION < 030100
            volk_32fc_x2_s32fc_multiply_conjugate_add_32fc((lv_32fc_t*)coeffBuffer, (lv_32fc_t*)coeffBuffer, (lv_32fc_t*)stateBuffer, *(lv_32fc_t*)&factorTimesResult, _filterOrder);
#else
            volk_32fc_x2_s32fc_multiply_conjugate_add2_32fc((lv_32fc_t*)coeffBuffer, (lv_32fc_t*)coeffBuffer, (lv_32fc_t*)stateBuffer, (lv_32fc_t*)&factorTimesResult, _filterOrder);
#endif

            coeffBuffer[_refIndex] = complex_t{ 1.0f, 0.0f };
        }

        unsigned int _stages = 1;
        unsigned int _filterOrder = 0;
        unsigned int _refIndex = 0;

        uint32_t _updateMask = 0x03u;
        float _alpha = 0.01f;
        float _targetLevel = 1.0f;
        float _mu = 0.0f;
        float _lastError = 0.0f;
        uint64_t _sampleCount = 0;

        complex_t* stateBuffer = NULL;
        complex_t* coeffBuffer = NULL;
        float* magScratchBuffer = NULL;

        // Signal presence detection
        float sp_energy_avg = 0.0f;
        float sp_env_avg = 0.0f;
        float sp_env_var = 0.0f;
        uint32_t sp_count = 0;
        bool signal_present = false;
    };
}
