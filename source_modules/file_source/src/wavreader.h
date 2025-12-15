#pragma once

#include "reader.h"
#include <stdint.h>
#include <string.h>
#include <fstream>

#define WAV_SIGNATURE       "RIFF"
#define WAV_TYPE            "WAVE"
#define WAV_FORMAT_MARK     "fmt "
#define WAV_DATA_MARK       "data"
#define WAV_SAMPLE_TYPE_PCM 1

class WavReader : public AudioReader {
public:
    WavReader(std::string path) {
        file = std::ifstream(path.c_str(), std::ios::binary);
        file.read((char*)&hdr, sizeof(WavHeader_t));
        valid = false;
        if (memcmp(hdr.signature, "RIFF", 4) != 0) { return; }
        if (memcmp(hdr.fileType, "WAVE", 4) != 0) { return; }
        valid = true;
        dataOffset = sizeof(WavHeader_t);
        dataSize = hdr.dataSize;
    }

    uint16_t getBitDepth() override {
        return hdr.bitDepth;
    }

    uint16_t getChannelCount() override {
        return hdr.channelCount;
    }

    uint32_t getSampleRate() override {
        return hdr.sampleRate;
    }

    bool isValid() override {
        return valid;
    }

    void readSamples(void* data, size_t size) override {
        char* _data = (char*)data;
        file.read(_data, size);
        int read = file.gcount();
        if (read < (int)size) {
            file.clear();
            file.seekg(dataOffset);
            file.read(&_data[read], size - read);
            read += file.gcount();
        }
        bytesRead += read;
    }

    void rewind() override {
        file.clear();
        file.seekg(dataOffset);
    }

    void close() {
        file.close();
    }

private:
    struct WavHeader_t {
        char signature[4];           // "RIFF"
        uint32_t fileSize;           // data bytes + sizeof(WavHeader_t) - 8
        char fileType[4];            // "WAVE"
        char formatMarker[4];        // "fmt "
        uint32_t formatHeaderLength; // Always 16
        uint16_t sampleType;         // PCM (1)
        uint16_t channelCount;
        uint32_t sampleRate;
        uint32_t bytesPerSecond;
        uint16_t bytesPerSample;
        uint16_t bitDepth;
        char dataMarker[4]; // "data"
        uint32_t dataSize;
    };

    bool valid = false;
    std::ifstream file;
    size_t bytesRead = 0;
    WavHeader_t hdr;
    uint32_t dataOffset = 0;
    uint32_t dataSize = 0;
};