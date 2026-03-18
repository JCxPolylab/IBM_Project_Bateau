#include "webui.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

int main()
{
    std::atomic<bool> missionRunning{ false };
    std::atomic<int> speed{ 40 };
    std::string lastAction = "none";
    std::string direction = "stop";

    CATJ_webui::WebUiServer web;

    CATJ_webui::WebUiConfig cfg;
    cfg.bindAddress = "0.0.0.0";
    cfg.port = 8080;
    cfg.documentRoot = "./webui_demo";
    cfg.indexFile = "index.html";

    web.setStateProvider([&]() {
        std::ostringstream oss;
        oss << "{";
        oss << "\"ok\":true,";
        oss << "\"missionRunning\":" << (missionRunning.load() ? "true" : "false") << ",";
        oss << "\"speed\":" << speed.load() << ",";
        oss << "\"direction\":\"" << CATJ_webui::WebUiServer::jsonEscape(direction) << "\",";
        oss << "\"lastAction\":\"" << CATJ_webui::WebUiServer::jsonEscape(lastAction) << "\"";
        oss << "}";
        return oss.str();
    });

    web.setActionHandler([&](const CATJ_webui::UiEvent& ev) {
        lastAction = ev.action;

        auto itSpeed = ev.params.find("speed");
        if (itSpeed != ev.params.end()) {
            try { speed = std::stoi(itSpeed->second); }
            catch (...) {}
        }

        if (ev.action == "move") {
            auto itDir = ev.params.find("dir");
            if (itDir != ev.params.end()) {
                direction = itDir->second;
            }
        }
        else if (ev.action == "mission") {
            auto itCmd = ev.params.find("cmd");
            if (itCmd != ev.params.end()) {
                if (itCmd->second == "start") missionRunning = true;
                if (itCmd->second == "pause") missionRunning = false;
            }
        }
        else if (ev.action == "horn") {
            std::cout << "BEEP" << std::endl;
        }

        std::cout << "[WEBUI] action=" << ev.action << " from " << ev.remoteIp << ':' << ev.remotePort << '\n';
        for (const auto& kv : ev.params) {
            std::cout << "  - " << kv.first << " = " << kv.second << '\n';
        }
    });

    if (!web.start(cfg)) {
        std::cerr << "Impossible de lancer le serveur WebUI" << std::endl;
        return -1;
    }

    std::cout << "Serveur WebUI lance sur: http://<IP_RASPI>:" << cfg.port << std::endl;
    std::cout << "Ctrl+C pour quitter." << std::endl;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
