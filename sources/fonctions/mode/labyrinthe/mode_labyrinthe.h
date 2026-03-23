#pragma once

#include <cmath>

#include "../../motherboard/motherboard_protocol.h"

namespace CATJ_robot {

    struct LidarDirections {
        float frontMm = INFINITY;
        float rightMm = INFINITY;
        float leftMm = INFINITY;
        float rearMm = INFINITY;
    };

    class ModeLabyrinthe {
    public:
        struct Config {
            float frontAvoidMm = 500.0f;
            float wallFollowMm = 400.0f;
            float rearReturnMm = 300.0f;
            float wallToleranceMm = 80.0f;
            float turnAngleDeg = 18.0f;
            int speedPct = 55;
        };

        ModeLabyrinthe();
        explicit ModeLabyrinthe(const Config& cfg);

        void start();
        void stop();
        bool running() const { return running_; }

        void update(const LidarDirections& d, MotherboardLink& board);
        int contacts() const { return contacts_; }
        bool circuitComplete() const { return circuitComplete_; }

    private:
        void wallFollow_(float rightMm, MotherboardLink& board);

        Config cfg_{};
        bool running_ = false;
        bool circuitComplete_ = false;
        int contacts_ = 0;
    };

} // namespace CATJ_robot
