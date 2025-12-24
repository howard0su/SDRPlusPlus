#pragma once
#include "../processor.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

namespace dsp::noise_reduction {
    class ANR : public Processor<stereo_t, stereo_t> {
        using base_type = Processor<stereo_t, stereo_t>;
    public:
        ANR() {}

        void init(stream<stereo_t>* in, int intensity) {
            _maxTaps = 128;
            _maxDelay = 64;
            _bufSize = _maxTaps + _maxDelay;

            _wL.assign(_maxTaps, 0.0f);
            _wR.assign(_maxTaps, 0.0f);
            _xL.assign(_bufSize, 0.0f);
            _xR.assign(_bufSize, 0.0f);
            
            _pos = 0;
            _lpfL = 0; _lpfR = 0;
            _envL = 0; _envR = 0;

            setIntensity(intensity);
            base_type::init(in);
        }

        void setIntensity(int intensity) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _intensity = std::clamp(intensity, 0, 30);
            if (_intensity <= 0) return;

            float norm = (float)_intensity / 30.0f;

            // 1. 动态参数：针对白噪声压制优化
            _taps = (_intensity < 15) ? 64 : 128; 
            _delay = 32 + (int)(32.0f * norm);   // 增加延迟，彻底打破白噪声自相关
            _muBase = 0.01f + (0.1f * norm);      // 适当加大步长，提高对信号的捕捉速度
            _leakage = 1.0f - (0.001f * norm);    // 维持系数稳定
            _lpfAlpha = 1.0f - (0.6f * norm);     // 随强度收窄带宽
        }

        inline int process(int count, stereo_t* in, stereo_t* out) {
            if (_intensity <= 0) {
                memcpy(out, in, count * sizeof(stereo_t));
                return count;
            }

            for (int i = 0; i < count; i++) {
                _xL[_pos] = in[i].l;
                _xR[_pos] = in[i].r;

                int u_idx = (_pos + _delay) % _bufSize;
                float yL = 0, yR = 0;
                float energyL = 1e-6f, energyR = 1e-6f;

                // --- 1. LMS 预测过程 ---
                for (int j = 0; j < _taps; j++) {
                    int idx = (u_idx + j) % _bufSize;
                    yL += _wL[j] * _xL[idx];
                    yR += _wR[j] * _xR[idx];
                    energyL += _xL[idx] * _xL[idx];
                    energyR += _xR[idx] * _xR[idx];
                }

                // --- 2. 权重更新 ---
                float errL = in[i].l - yL;
                float errR = in[i].r - yR;
                float stepL = _muBase / energyL;
                float stepR = _muBase / energyR;

                for (int j = 0; j < _taps; j++) {
                    int idx = (u_idx + j) % _bufSize;
                    _wL[j] = (_wL[j] * _leakage) + (stepL * errL * _xL[idx]);
                    _wR[j] = (_wR[j] * _leakage) + (stepR * errR * _xR[idx]);
                }

                // --- 3. 增强版相干性检测 ---
                // 快速追踪输入包络
                _envL = (_envL * 0.98f) + (std::abs(in[i].l) * 0.02f);
                _envR = (_envR * 0.98f) + (std::abs(in[i].r) * 0.02f);

                // 计算原始相干性：y 与输入的比例
                float gL = std::abs(yL) / (_envL + 1e-6f);
                float gR = std::abs(yR) / (_envR + 1e-6f);

                // --- 4. 非线性压制逻辑 (孔雀石压死白噪声的杀手锏) ---
                // 强度越强，压制函数越陡峭
                float hardness = 1.0f + ( (float)_intensity / 10.0f ); // 1.0 到 4.0

                // 使用幂函数压制：如果是白噪声，gL 会很小（如 0.2），gL^4 会变得极其微弱
                float finalGainL = std::pow(std::clamp(gL, 0.0f, 1.0f), hardness);
                float finalGainR = std::pow(std::clamp(gR, 0.0f, 1.0f), hardness);

                // 限制最低增益，防止完全静音导致听感不自然
                float minG = 0.01f; // 对应 -40dB 的背景压制
                if (finalGainL < minG) finalGainL = minG;
                if (finalGainR < minG) finalGainR = minG;

                // --- 5. 最终输出 ---
                // 此时输出的是经过大幅度非线性压制的预测信号
                float resL = yL * finalGainL;
                float resR = yR * finalGainR;

                // 最后的低通平滑
                _lpfL = (_lpfAlpha * resL) + ((1.0f - _lpfAlpha) * _lpfL);
                _lpfR = (_lpfAlpha * resR) + ((1.0f - _lpfAlpha) * _lpfR);

                out[i].l = _lpfL;
                out[i].r = _lpfR;

                _pos = (_pos == 0) ? (_bufSize - 1) : (_pos - 1);
            }
            return count;
        }

        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            base_type::ctrlMtx.lock();
            process(count, base_type::_in->readBuf, base_type::out.writeBuf);
            base_type::ctrlMtx.unlock();

            base_type::_in->flush();
            if (!base_type::out.swap(count)) { return -1; }
            return count;
        }

    protected:
        int _intensity = 0, _taps = 64, _delay = 48;
        float _muBase, _lpfAlpha, _leakage;
        float _lpfL = 0, _lpfR = 0, _envL = 0, _envR = 0;
        std::vector<float> _wL, _wR, _xL, _xR;
        int _pos = 0, _maxTaps, _maxDelay, _bufSize;
    };
}