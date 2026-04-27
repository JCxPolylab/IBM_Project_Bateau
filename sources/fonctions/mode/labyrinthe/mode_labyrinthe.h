#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "../../motherboard/motherboard_protocol.h"

namespace CATJ_robot {

    struct LidarDirections {
        float frontMm = INFINITY;
        float frontRightMm = INFINITY;
        float rightMm = INFINITY;
        float leftMm = INFINITY;
        float frontLeftMm = INFINITY;
        float rearMm = INFINITY;
    };

    class ModeLabyrinthe {
    public:
        struct Config {
            float frontAvoidMm = 700.0f;
            float frontCriticalMm = 240.0f;
            float sideContactMm = 130.0f;
            float wallFollowMm = 420.0f;
            float rearReturnMm = 320.0f;
            float wallToleranceMm = 90.0f;
            float turnAngleDeg = 22.0f;
            float wallCorrectionDeg = 9.0f;
            bool useGyroWhenNoWall = true;
            bool requireHeadingForFinish = false;
            float headingKp = 0.35f;
            float maxHeadingCorrectionDeg = 12.0f;
            float headingDeadbandDeg = 4.0f;
            float finishHeadingToleranceDeg = 60.0f;
            int speedPct = 45;
            int avoidSpeedPct = 28;
            double minRunBeforeReturnSec = 15.0;
            int returnConfirmCycles = 3;
            std::chrono::milliseconds contactDebounce{ 1000 };
        };

        ModeLabyrinthe();
        explicit ModeLabyrinthe(const Config& cfg);

        void start();
        void stop();
        bool running() const { return running_; }

        void update(const LidarDirections& d, MotherboardLink& board, std::optional<float> headingDeg = std::nullopt);
        int contacts() const { return contacts_; }
        bool circuitComplete() const { return circuitComplete_; }
        double elapsedSec() const;
        std::string stateName() const;

    private:
        enum class State {
            Idle,
            WallFollow,
            Avoid,
            Finish
        };

        void wallFollow_(const LidarDirections& d, MotherboardLink& board, std::optional<float> headingDeg);
        void registerContact_(const LidarDirections& d);
        float headingCorrection_(float headingDeg, float targetDeg) const;
        static float normalizeAngleDeg_(float angleDeg);
        static bool isFinitePositive_(float v);

        Config cfg_{};
        bool running_ = false;
        bool circuitComplete_ = false;
        int contacts_ = 0;
        int returnSeenCount_ = 0;
        bool startHeadingValid_ = false;
        float startHeadingDeg_ = 0.0f;
        State state_ = State::Idle;
        std::chrono::steady_clock::time_point startTs_{};
        std::chrono::steady_clock::time_point lastContactTs_{};
    };

} // namespace CATJ_robot
