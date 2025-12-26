#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <module.h>
#include <gui/widgets/menu.h>

class DecoderManager {
public:
    DecoderManager();
    ~DecoderManager();

    // Register a decoder module
    void registerDecoder(const std::string& name, void (*drawHandler)(void* ctx), void* ctx);

    // Remove a decoder module
    void removeDecoder(const std::string& name);

    // Set active decoder
    void setActiveDecoder(const std::string& name);

    // Get active decoder
    std::string getActiveDecoder();

    // Draw tabbed interface
    void drawInterface();

    // Draw decoder content for active tab
    void drawActiveDecoder();

    // Check if decoder is active
    bool isActive(const std::string& name);

private:
    // Get all decoder names, sorted with radio decoders first
    std::vector<std::string> getDecoderNames();

    // No special radio tracking - just uses isRadioDecoder() for ordering
    bool isRadioDecoder(const std::string& name);

    struct DecoderInfo {
        std::string name;
        void* ctx;
        void (*drawHandler)(void* ctx);
        bool active;
        std::string vfoName;
    };

    std::map<std::string, DecoderInfo> decoders;
    std::string activeDecoderName;
};
