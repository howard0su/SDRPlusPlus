#include "decoder_manager.h"
#include <imgui.h>
#include <gui/style.h>
#include <utils/flog.h>
#include <gui/widgets/waterfall.h>
#include <signal_path/vfo_manager.h>
#include <gui/gui.h>

DecoderManager::DecoderManager() {
}

DecoderManager::~DecoderManager() {
}

void DecoderManager::registerDecoder(const std::string& name, void (*drawHandler)(void* ctx), void* ctx) {
    flog::info("Registering decoder '{}'", name);

    if (decoders.find(name) != decoders.end()) {
        flog::warn("Decoder '{}' already registered", name);
        return;
    }

    DecoderInfo info;
    info.name = name;
    info.ctx = ctx;
    info.drawHandler = drawHandler;
    info.vfoName = name; // VFO name matches decoder name
    info.active = isRadioDecoder(name);

    decoders[name] = info;
    flog::info("Successfully registered decoder '{}'", name);

    // If this is the first decoder or is a radio decoder and we don't have an active decoder yet,
    // automatically activate it
    if (isRadioDecoder(name)) {
        flog::info("Auto-activating decoder '{}'", name);
        setActiveDecoder(name);
    }
}

void DecoderManager::removeDecoder(const std::string& name) {
    auto it = decoders.find(name);
    if (it != decoders.end()) {
        bool wasActive = it->second.active;
        decoders.erase(it);

        if (wasActive) {
            setActiveDecoder("Radio");
        }
        flog::info("Removed decoder '{}'", name);
    }
}

void DecoderManager::setActiveDecoder(const std::string& name) {
    DecoderInfo* currentActive = nullptr;
    DecoderInfo* newActive = nullptr;

    for (auto& [decName, info] : decoders) {
        if (info.active) {
            currentActive = &info;
        }

        if (decName == name) {
            newActive = &info;
        }
    }

    if (newActive == nullptr) {
        flog::warn("Tried to activate non-existent decoder '{}'", name);
        return;
    }

    currentActive->active = false;
    newActive->active = true;

    // Update waterfall to select the VFO for this decoder
    if (gui::waterfall.vfos.find(name) != gui::waterfall.vfos.end()) {
        if (gui::waterfall.selectedVFO != name) {
            gui::waterfall.selectedVFO = name;
            gui::waterfall.selectedVFOChanged = true;
            flog::info("Switched to decoder '{}' VFO", name);
        }
    }
}

void DecoderManager::drawInterface() {
    if (decoders.empty()) {
        ImGui::Text("No decoders active");
        return;
    }

    ImVec4 activeColor = ImVec4(0.20f, 0.55f, 0.90f, 1.0f); // obvious blue
    // Get available width
    float availWidth = ImGui::GetContentRegionAvail().x;
    int tabCount = decoders.size();
    float tabWidth = availWidth / tabCount;

    // Draw tabs
    std::vector<std::string> names = getDecoderNames();
    for (int i = 0; i < names.size(); i++) {
        auto& decoder = decoders[names[i]];
        bool isActive = decoder.active;

        ImGui::PushID(i);

        if (ImGui::Button(names[i].c_str(), ImVec2(tabWidth, 0))) {
            setActiveDecoder(names[i]);
        }

        if (isActive) {
            ImVec2 min = ImGui::GetItemRectMin();
            ImVec2 max = ImGui::GetItemRectMax();

            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddLine(
                ImVec2(min.x, max.y - 1),
                ImVec2(max.x, max.y - 1),
                ImGui::GetColorU32(activeColor),
                2.0f);
        }

        ImGui::PopID();

        if (i < names.size() - 1) {
            ImGui::SameLine();
        }
    }
}

void DecoderManager::drawActiveDecoder() {
    for (auto& [name, decoder] : decoders) {
        if (decoder.active) {
            if (decoder.drawHandler) {
                decoder.drawHandler(decoder.ctx);
            }
            return;
        }
    }
}

bool DecoderManager::isActive(const std::string& name) {
    auto it = decoders.find(name);
    if (it != decoders.end()) {
        return it->second.active;
    }
    return false;
}

std::vector<std::string> DecoderManager::getDecoderNames() {
    std::vector<std::string> names;

    names.push_back("Radio");
    // Collect all decoder names
    for (auto& [name, info] : decoders) {
        if (isRadioDecoder(name)) {
            continue;
        }
        names.push_back(name);
    }

    return names;
}

bool DecoderManager::isRadioDecoder(const std::string& name) {
    bool isRadio = name.find("radio") == 0 || name.find("Radio") != std::string::npos ||
                   name.find("RADIO") != std::string::npos;
    if (isRadio) {
        flog::debug("Detected '{}' as radio decoder", name);
    }
    return isRadio;
}
