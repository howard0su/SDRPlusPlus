#pragma once

#include <stdint.h>
#include <cstddef>
#include <string>

class AudioReader {
public:
    virtual ~AudioReader() {}
    virtual uint16_t getBitDepth() = 0;
    virtual uint16_t getChannelCount() = 0;
    virtual uint32_t getSampleRate() = 0;
    virtual bool isValid() = 0;
    virtual void readSamples(void* data, size_t size) = 0;
    virtual void rewind() = 0;
    virtual void close() = 0;
};
