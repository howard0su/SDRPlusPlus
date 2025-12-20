#include <gui/dialogs/credits.h>
#include <imgui.h>
#include <gui/icons.h>
#include <gui/style.h>
#include <config.h>
#include <version.h>

namespace credits {
    ImFont* bigFont;
    ImVec2 imageSize(128.0f, 128.0f);

    void init() {
        imageSize = ImVec2(128.0f * style::uiScale, 128.0f * style::uiScale);
    }

    void show() {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f, 20.0f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
        ImVec2 dispSize = ImGui::GetIO().DisplaySize;
        ImVec2 center = ImVec2(dispSize.x / 2.0f, dispSize.y / 2.0f);
        ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::OpenPopup("Credits");
        ImGui::BeginPopupModal("Credits", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove);

        ImGui::PushFont(style::hugeFont);
        ImGui::TextUnformatted("SDR-888          ");
        ImGui::PopFont();
        ImGui::SameLine();
        ImGui::Image(icons::LOGO, imageSize);
        ImGui::Spacing();
        ImGui::Spacing();

        ImGui::TextUnformatted("This software is based on SDR++ from Alexandre Rouma (ON5RYZ) . Modified by SDDC Lab.");

        ImGui::Spacing();
        ImGui::TextUnformatted("SDR-888 v" VERSION_STR " (Built at " __TIME__ ", " __DATE__ ")");

        ImGui::EndPopup();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }
}