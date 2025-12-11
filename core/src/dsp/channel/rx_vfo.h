#pragma once
#include <volk/volk.h>
#include "frequency_xlator.h"
#include "../multirate/rational_resampler.h"
// Optional two-stage pipeline using streams to split resampling and filtering
// across two threads when the block is started. Direct calls to process()
// remain single-threaded for compatibility.

namespace dsp::channel {
    class RxVFO : public Processor<complex_t, complex_t> {
        using base_type = Processor<complex_t, complex_t>;
    public:
        RxVFO() {}

        RxVFO(stream<complex_t>* in, double inSamplerate, double outSamplerate, double bandwidth, double offset) { init(in, inSamplerate, outSamplerate, bandwidth, offset); }

        ~RxVFO() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
            taps::free(ftaps);
        }

        void init(stream<complex_t>* in, double inSamplerate, double outSamplerate, double bandwidth, double offset) {
            _inSamplerate = inSamplerate;
            _outSamplerate = outSamplerate;
            _bandwidth = bandwidth;
            _offset = offset;
            filterNeeded = (_bandwidth != _outSamplerate);
            ftaps.taps = NULL;

            xlator.init(NULL, -_offset, _inSamplerate);
            resamp.init(NULL, _inSamplerate, _outSamplerate);
            generateTaps();
            filter.init(NULL, ftaps);

            stage1.init(in, &mid);          // input -> mid stream
            stage2.init(&mid, &out);        // mid stream -> public out

            base_type::init(in);
        }

        void setInSamplerate(double inSamplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _inSamplerate = inSamplerate;
            xlator.setOffset(-_offset, _inSamplerate);
            resamp.setInSamplerate(_inSamplerate);
            base_type::tempStart();
        }

        void setOutSamplerate(double outSamplerate, double bandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _outSamplerate = outSamplerate;
            _bandwidth = bandwidth;
            filterNeeded = (_bandwidth != _outSamplerate);
            resamp.setOutSamplerate(_outSamplerate);
            if (filterNeeded) {
                generateTaps();
                filter.setTaps(ftaps);
            }
            base_type::tempStart();
        }

        void setBandwidth(double bandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            std::lock_guard<std::mutex> lck2(filterMtx);
            _bandwidth = bandwidth;
            filterNeeded = (_bandwidth != _outSamplerate);
            if (filterNeeded) {
                generateTaps();
                filter.setTaps(ftaps);
            }
        }

        void setOffset(double offset) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _offset = offset;
            xlator.setOffset(-_offset, _inSamplerate);
        }

        void reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            xlator.reset();
            resamp.reset();
            filter.reset();
            base_type::tempStart();
        }

        inline int process(int count, const complex_t* in, complex_t* out) {
            count = processStage1(count, in, out);
            if (count <= 0) { return count; }
            return processStage2(count, out, out);
        }

        // run() is unused when the block is started (we override doStart/doStop
        // to launch two stage workers). It is kept for compatibility and uses
        // the single-threaded path.
        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            int outCount = process(count, base_type::_in->readBuf, out.writeBuf);

            // Swap if some data was generated
            base_type::_in->flush();
            if (outCount) {
                if (!out.swap(outCount)) { return -1; }
            }
            return outCount;
        }

    protected:
        // Split stage 1: translate + resample
        inline int processStage1(int count, const complex_t* in, complex_t* out) {
            if (_offset != 0.0) {
                xlator.process(count, in, out);
                in = out;
            }

            return resamp.process(count, in, out);
        }

        // Split stage 2: optional low-pass filter / bypass copy
        inline int processStage2(int count, const complex_t* in, complex_t* out) {
            if (!filterNeeded) {
                if (out != in) {
                    memcpy(out, in, count * sizeof(lv_32fc_t));
                }
                return count;
            }

            std::lock_guard<std::mutex> lck(filterMtx);
            filter.process(count, in, out);
            return count;
        }

        void doStart() override {
            // Start consumer first to avoid stalling the mid stream
            stage2.start();
            stage1.start();
        }

        void doStop() override {
            stage1.stop();
            stage2.stop();
        }

        void generateTaps() {
            taps::free(ftaps);
            double filterWidth = _bandwidth / 2.0;
            ftaps = taps::lowPass(filterWidth, filterWidth * 0.1, _outSamplerate);
        }

        class Stage1Worker : public block {
        public:
            Stage1Worker(RxVFO& p) : parent(p) {}

            void init(stream<complex_t>* in, stream<complex_t>* midOut) {
                _in = in;
                _out = midOut;
                registerInput(_in);
                registerOutput(_out);
                _block_init = true;
            }

            int run() override {
                int count = _in->read();
                if (count < 0) { return -1; }

                int outCount = parent.processStage1(count, _in->readBuf, _out->writeBuf);

                _in->flush();
                if (outCount) {
                    if (!_out->swap(outCount)) { return -1; }
                }
                return outCount;
            }

        private:
            RxVFO& parent;
            stream<complex_t>* _in = nullptr;
            stream<complex_t>* _out = nullptr;
        };

        class Stage2Worker : public block {
        public:
            Stage2Worker(RxVFO& p) : parent(p) {}

            void init(stream<complex_t>* in, stream<complex_t>* finalOut) {
                _in = in;
                _out = finalOut;
                registerInput(_in);
                registerOutput(_out);
                _block_init = true;
            }

            int run() override {
                int count = _in->read();
                if (count < 0) { return -1; }

                int outCount = parent.processStage2(count, _in->readBuf, _out->writeBuf);

                _in->flush();
                if (outCount) {
                    if (!_out->swap(outCount)) { return -1; }
                }
                return outCount;
            }

        private:
            RxVFO& parent;
            stream<complex_t>* _in = nullptr;
            stream<complex_t>* _out = nullptr;
        };

        stream<complex_t> mid;
        Stage1Worker stage1{*this};
        Stage2Worker stage2{*this};

        FrequencyXlator xlator;
        multirate::RationalResampler<complex_t> resamp;
        filter::FIR<complex_t, float> filter;
        tap<float> ftaps;
        bool filterNeeded;

        double _inSamplerate;
        double _outSamplerate;
        double _bandwidth;
        double _offset;

        std::mutex filterMtx;
    };
}