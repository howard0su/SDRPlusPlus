#include <gui/widgets/image.h>
#include "backend.h"

namespace ImGui {
    ImageDisplay::ImageDisplay(int width, int height) {
        _width = width;
        _height = height;
        buffer = malloc(_width * _height * 4);
        activeBuffer = malloc(_width * _height * 4);
        memset(buffer, 0, _width * _height * 4);
        memset(activeBuffer, 0, _width * _height * 4);

        textureId = backend::createTexture(_width, _height, nullptr);
    }

    ImageDisplay::~ImageDisplay() {
        free(buffer);
        free(activeBuffer);
    }

    void ImageDisplay::draw(const ImVec2& size_arg) {
        std::lock_guard<std::mutex> lck(bufferMtx);

        ImGuiWindow* window = GetCurrentWindow();
        ImGuiStyle& style = GetStyle();
        ImVec2 min = window->DC.CursorPos;

        // Calculate scale
        float width = CalcItemWidth();
        float height = roundf((width / (float)_width) * (float)_height);

        ImVec2 size = CalcItemSize(size_arg, CalcItemWidth(), height);
        ImRect bb(min, ImVec2(min.x + size.x, min.y + size.y));

        ItemSize(size, style.FramePadding.y);
        if (!ItemAdd(bb, 0)) {
            return;
        }

        if (newData) {
            newData = false;
            backend::updateTexture(textureId, _width, _height, activeBuffer);
        }

        window->DrawList->AddImage((void*)(intptr_t)textureId, min, ImVec2(min.x + width, min.y + height));
    }

    void ImageDisplay::swap() {
        std::lock_guard<std::mutex> lck(bufferMtx);
        void* tmp = activeBuffer;
        activeBuffer = buffer;
        buffer = tmp;
        newData = true;
        memset(buffer, 0, _width * _height * 4);
    }
}