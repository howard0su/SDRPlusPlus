#include <gui/icons.h>
#include <stdint.h>
#include <config.h>
#include "backend.h"

#define STB_IMAGE_IMPLEMENTATION
#include <imgui/stb_image.h>
#include <filesystem>
#include <utils/flog.h>

namespace icons {
    ImTextureID LOGO;
    ImTextureID PLAY;
    ImTextureID STOP;
    ImTextureID MENU;
    ImTextureID MUTED;
    ImTextureID UNMUTED;
    ImTextureID NORMAL_TUNING;
    ImTextureID CENTER_TUNING;

    static ImTextureID loadTexture(std::string path) {
        int w, h, n;
        stbi_uc* data = stbi_load(path.c_str(), &w, &h, &n, 0);
        ImTextureID texId = backend::createTexture(w, h, data);
        stbi_image_free(data);
        return texId;
    }

    bool load(std::string resDir) {
        if (!std::filesystem::is_directory(resDir)) {
            flog::error("Invalid resource directory: {0}", resDir);
            return false;
        }

        LOGO = loadTexture(resDir + "/icons/sdrpp.png");
        PLAY = loadTexture(resDir + "/icons/play.png");
        STOP = loadTexture(resDir + "/icons/stop.png");
        MENU = loadTexture(resDir + "/icons/menu.png");
        MUTED = loadTexture(resDir + "/icons/muted.png");
        UNMUTED = loadTexture(resDir + "/icons/unmuted.png");
        NORMAL_TUNING = loadTexture(resDir + "/icons/normal_tuning.png");
        CENTER_TUNING = loadTexture(resDir + "/icons/center_tuning.png");

        return true;
    }
}