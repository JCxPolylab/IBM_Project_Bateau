#pragma once

#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "../ball/ball_logic.h"
#include "../../motherboard/motherboard_protocol.h"

namespace CATJ_robot {

    class ModeCourse {
    public:
        struct Config {
            int scoreBaseCollecte = 20;
            int scoreBonusPonton = 100;
            int scoreBonusAfficheur = 50;
            float distCollecteRapideM = 0.50f;
            float catchAngleGateDeg = 18.0f;
            bool catchOnlyPositiveTargets = true;
            int conveyorNormalPct = 80;
            int conveyorRapidePct = 100;
            int scoreGenericPositive = 10;
            int scoreGenericTrash = -10;
            int scorePingPongOrange = -5;
            int scorePingPongWhite = -5;
            int scorePiscineRed = 10;
            int scorePiscineOther = -10;
            std::chrono::milliseconds antiRebond{ 1500 };
        };

        ModeCourse();
        explicit ModeCourse(const Config& cfg);

        void prepare(int predictedScore);
        void start();
        void stop();
        bool running() const { return running_; }

        void update(const std::vector<RobotBall>& visibleBalls,
            float rearDistanceMm,
            MotherboardLink& board);

        void processMotherboardStatus(const MotherboardStatus& status, MotherboardLink& board);
        bool lockCollection(MotherboardLink& board);

        int score() const { return score_; }
        int predictedScore() const { return predictedScore_; }
        bool pontonDetected() const { return retourPonton_; }
        void setPontoonMonitoring(bool enabled) { allowPontoonBonus_ = enabled; }
        bool pontoonMonitoringEnabled() const { return allowPontoonBonus_; }

    private:
        void registerBall_(BallType type);
        void registerMotherboardBall_(BallKind kind);
        void setScoreFromBoard_(int score);
        int scoreForBallKind_(BallKind kind) const;

        Config cfg_{};
        bool running_ = false;
        bool firstBall_ = false;
        bool retourPonton_ = false;
        bool verrouille_ = false;
        bool allowPontoonBonus_ = false;
        int score_ = 0;
        int predictedScore_ = 0;
        std::chrono::steady_clock::time_point lastCollect_{};
        std::map<BallType, int> counts_{};
    };

    struct CourseLidarDirections {
        float frontMm = INFINITY;
        float frontRightMm = INFINITY;
        float rightMm = INFINITY;
        float leftMm = INFINITY;
        float frontLeftMm = INFINITY;
        float rearMm = INFINITY;
    };

    class ModeCourseDemiTour {
    public:
        struct Config {
            float markerApproachMm = 1600.0f;
            float markerCaptureMm = 950.0f;
            float markerTooCloseMm = 420.0f;
            float homeDetectMm = 420.0f;
            float straightWallBalanceMm = 220.0f;
            float straightHeadingCorrectionDeg = 6.0f;
            float arcTurnDeg = 34.0f;
            float arcTightTurnDeg = 48.0f;
            float counterTurnDeg = 14.0f;
            float searchTurnDeg = 12.0f;
            float frontContactMm = 220.0f;
            float sideContactMm = 140.0f;
            int speedStraightPct = 40;
            int speedApproachPct = 28;
            int speedArcPct = 24;
            int speedCounterPct = 28;
            int speedReturnPct = 40;
            int speedBackupPct = 18;
            int speedSearchPct = 24;
            bool clockwiseTurn = true;
            bool useGyroHeading = true;
            float headingKp = 0.45f;
            float maxHeadingCorrectionDeg = 18.0f;
            float headingDeadbandDeg = 3.0f;
            float returnHeadingToleranceDeg = 18.0f;
            double fallbackMarkerSec = 8.0;
            double minHomeDetectSec = 6.0;
            double minReturnTravelSec = 2.0;
            int homeConfirmCycles = 3;
            std::chrono::milliseconds backupDuration{ 500 };
            std::chrono::milliseconds arcDuration{ 2600 };
            std::chrono::milliseconds counterDuration{ 1100 };
            std::chrono::milliseconds searchFlipPeriod{ 1200 };
            std::chrono::milliseconds contactDebounce{ 1000 };
        };

        ModeCourseDemiTour();
        explicit ModeCourseDemiTour(const Config& cfg);

        void start();
        void stop();
        bool running() const { return running_; }

        void update(const CourseLidarDirections& d, MotherboardLink& board, std::optional<float> headingDeg = std::nullopt);
        bool courseComplete() const { return courseComplete_; }
        bool markerSeen() const { return markerSeen_; }
        bool homeSeen() const { return homeSeen_; }
        int contacts() const { return contacts_; }
        double elapsedSec() const;
        std::string stateName() const;

    private:
        enum class State {
            Idle,
            StraightOut,
            BackupClear,
            ArcAround,
            CounterSteer,
            ReturnHome,
            Finish
        };

        void registerContact_(const CourseLidarDirections& d);
        void setMotion_(MotherboardLink& board, MoveCommand move, int speedPct, float turnDeg);
        void transitionTo_(State next, MotherboardLink& board);
        float headingCorrection_(float headingDeg, float targetDeg) const;
        static float normalizeAngleDeg_(float angleDeg);
        static bool isFinitePositive_(float v);

        Config cfg_{};
        bool running_ = false;
        bool courseComplete_ = false;
        bool markerSeen_ = false;
        bool homeSeen_ = false;
        int contacts_ = 0;
        int homeSeenCount_ = 0;
        int searchTurnSign_ = 1;
        bool startHeadingValid_ = false;
        float startHeadingDeg_ = 0.0f;
        State state_ = State::Idle;
        MoveCommand lastMove_ = MoveCommand::Stop;
        int lastSpeedPct_ = -1;
        float lastTurnDeg_ = 9999.0f;
        std::chrono::steady_clock::time_point startTs_{};
        std::chrono::steady_clock::time_point stateTs_{};
        std::chrono::steady_clock::time_point lastContactTs_{};
        std::chrono::steady_clock::time_point lastSearchFlipTs_{};
    };

} // namespace CATJ_robot
