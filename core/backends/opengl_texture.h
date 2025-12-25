#include "imgui.h"
#include <GLFW/glfw3.h>

namespace backend {
    ImTextureID createTexture(int width, int height, const void* data)
    {
        GLuint texId;
        glGenTextures(1, &texId);

        if (data) {
            glBindTexture(GL_TEXTURE_2D, texId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (uint8_t*)data);
        }

        return texId;
    }
    
    static void getTextureSize(ImTextureID texId, int& width, int& height)
    {
        GLuint id = (GLuint)(uintptr_t)texId;
        glBindTexture(GL_TEXTURE_2D, id);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
    }

    void updateTexture(ImTextureID texId, const void* data)
    {
        int width;
        int height;
        GLuint id = (GLuint)(uintptr_t)texId;
        getTextureSize(texId, width, height);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (uint8_t*)data);
    }

    void deleteTexture(ImTextureID texId)
    {
        GLuint id = (GLuint)(uintptr_t)texId;
        glDeleteTextures(1, &id);
    }
}