#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "../Communication/uart/uart.h"

namespace CATJ_robot {

    enum class MoveCommand {
        Stop,
        Forward,
        Backward
    };

    enum class RobotMode {
        Stop,
        Labyrinthe,
        Ramassage,
        Course,
        Debug
    };

    enum class CatchAck {
        Unknown,
        Ok,
        Failed
    };

    enum class BallKind {
        Unknown,
        Score,
        Trash
    };

    struct MotherboardStatus {
        bool valid = false;
        bool etat = false;
        CatchAck catchAck = CatchAck::Unknown;
        BallKind balle = BallKind::Unknown;
        int speedVal = 0;
        std::string rawLine;
    };

    struct MotherboardConfig {
        CATJ_uart::UartConfig uart;
        int commandTimeoutMs = 100;
        int statusTimeoutMs = 20;
    };

    class MotherboardLink {
    public:
        MotherboardLink() = default;
        explicit MotherboardLink(const MotherboardConfig& cfg);

        bool open(const MotherboardConfig& cfg);
        void close();
        bool isOpen() const;

        bool sendTurnDeg(float angleDeg);
        bool sendMove(MoveCommand move);
        bool sendSpeedPct(int speedPct);
        bool sendCatch();
        bool sendMode(RobotMode mode);
        bool sendObjective(int predictedScore);
        bool sendStopAll();
        bool sendHeartbeat();

        bool requestStatus();
        std::optional<MotherboardStatus> readStatusOnce(int timeoutMs = -1);
        const MotherboardStatus& lastStatus() const { return lastStatus_; }

        static std::string moveToString(MoveCommand move);
        static std::string modeToString(RobotMode mode);
        static std::string catchAckToString(CatchAck v);
        static std::string ballKindToString(BallKind v);

    private:
        bool sendLine_(const std::string& line);
        std::optional<MotherboardStatus> parseLine_(const std::string& line) const;
        static std::string trim_(std::string s);
        static std::string upper_(std::string s);
        static bool startsWith_(const std::string& s, const std::string& prefix);

        MotherboardConfig cfg_{};
        CATJ_uart::Uart uart_{};
        mutable std::mutex mutex_;
        MotherboardStatus lastStatus_{};
    };

} // namespace CATJ_robot
