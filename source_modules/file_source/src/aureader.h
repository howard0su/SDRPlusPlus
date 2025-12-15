#pragma once

#include "reader.h"
#include <fstream>

class AuReader : public AudioReader {
public:
    AuReader(std::string path) {
        file.open(path.c_str(), std::ios::binary);
        if (!file.is_open()) { return; }

        file.read((char*)&hdr, sizeof(AuHeader_t));
        if (file.gcount() < (int)sizeof(AuHeader_t)) { return; }

        uint32_t dataOff = be32(hdr.dataOffset);
        uint32_t dataSz = be32(hdr.dataSize);
        uint32_t encoding = be32(hdr.encoding);
        uint32_t srate = be32(hdr.sampleRate);
        uint32_t channels = be32(hdr.channels);

        // encoding 3 = 16-bit linear (big-endian), 2 = 8-bit linear
        if (encoding == 3) {
            bitDepth = 16;
        } else if (encoding == 2) {
            bitDepth = 8;
        } else {
            // unsupported
            return;
        }

        dataOffset = dataOff;
        dataSize = dataSz;
        sampleRate = srate;
        channelCount = (uint16_t)channels;

        // seek to data
        file.seekg(dataOffset);
        valid = true;
    }

    ~AuReader() { close(); }

    uint16_t getBitDepth() override { return bitDepth; }
    uint16_t getChannelCount() override { return channelCount; }
    uint32_t getSampleRate() override { return sampleRate; }
    bool isValid() override { return valid; }
    void readSamples(void* data, size_t size) override {
        char* _data = (char*)data;
        file.read(_data, size);
        int read = file.gcount();
        if (read < (int)size) {
            // loop back
            file.clear();
            file.seekg(dataOffset);
            file.read(&_data[read], size - read);
            read += file.gcount();
        }

        // if 16-bit BE, swap to LE
        if (bitDepth == 16) {
            for (int i = 0; i + 1 < read; i += 2) {
                char a = _data[i];
                _data[i] = _data[i+1];
                _data[i+1] = a;
            }
        }
    }

    void rewind() override { file.clear(); file.seekg(dataOffset); }
    void close() override { if (file.is_open()) file.close(); }

private:
    struct AuHeader_t {
        char magic[4];
        uint32_t dataOffset;
        uint32_t dataSize;
        uint32_t encoding;
        uint32_t sampleRate;
        uint32_t channels;
    };

    std::ifstream file;
    AuHeader_t hdr;
    uint32_t dataOffset = 0;
    uint32_t dataSize = 0;
    uint16_t bitDepth = 0;
    uint16_t channelCount = 0;
    uint32_t sampleRate = 0;
    bool valid = false;

    static uint32_t be32(const uint32_t v) {
        unsigned char* b = (unsigned char*)&v;
        return (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | (uint32_t)b[3];
    }
};
