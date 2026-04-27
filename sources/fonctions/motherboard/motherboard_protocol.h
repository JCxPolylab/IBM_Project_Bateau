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

    enum class LockAck {
        Unknown,
        Ok,
        Failed
    };

    enum class BallKind {
        Unknown,
        Score,          // evenement positif generique (+10 par defaut)
        Trash,          // evenement negatif generique (-10 par defaut)
        PingPongOrange, // -5 par defaut
        PingPongWhite,  // -5 par defaut, configurable
        PiscineRed,     // +10 par defaut
        PiscineOther    // -10 par defaut
    };

    struct MotherboardStatus {
        bool valid = false;
        bool etat = false;
        CatchAck catchAck = CatchAck::Unknown;
        LockAck lockAck = LockAck::Unknown;
        BallKind balle = BallKind::Unknown;
        int speedVal = 0;
        bool scoreValid = false;       // score absolu envoye par la carte mere
        int score = 0;
        bool scoreDeltaValid = false;  // variation ponctuelle envoye par la carte mere
        int scoreDelta = 0;
        bool headingValid = false;     // cap / yaw gyro en degres
        float headingDeg = 0.0f;
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
        bool sendLockCollector();
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
        static std::string lockAckToString(LockAck v);
        static std::string ballKindToString(BallKind v);

    private:
        bool sendLine_(const std::string& line);
        std::optional<MotherboardStatus> parseLine_(const std::string& line) const;
        static std::string trim_(std::string s);
        static std::string upper_(std::string s);
        static bool startsWith_(const std::string& s, const std::string& prefix);
        static bool parseInt_(const std::string& s, int& out);
        static bool parseFloat_(const std::string& s, float& out);
        static BallKind parseBallKind_(const std::string& s);

        MotherboardConfig cfg_{};
        CATJ_uart::Uart uart_{};
        mutable std::mutex mutex_;
        MotherboardStatus lastStatus_{};
    };

} // namespace CATJ_robot
