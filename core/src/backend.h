#pragma once
#include <string>
#include "imgui.h"

namespace backend {
    int init(std::string resDir = "");
    void beginFrame();
    void render(bool vsync = true);
    void getMouseScreenPos(double& x, double& y);
    void setMouseScreenPos(double x, double y);
    int renderLoop();
    int end();

    ImTextureID createTexture(int width, int height, const void* data);
    void updateTexture(ImTextureID textureId, int width, int height, const void* data);
    void deleteTexture(ImTextureID textureId);
}