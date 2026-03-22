#pragma once

#include <chrono>
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
            float distBalleCollecteeM = 0.20f;
            int conveyorNormalPct = 80;
            int conveyorRapidePct = 100;
            std::chrono::milliseconds antiRebond{1500};
        };

        explicit ModeCourse(Config cfg = {});

        void prepare(int predictedScore);
        void start();
        void stop();
        bool running() const { return running_; }

        void update(const std::vector<RobotBall>& visibleBalls,
                    float rearDistanceMm,
                    MotherboardLink& board);

        int score() const { return score_; }
        int predictedScore() const { return predictedScore_; }
        bool pontonDetected() const { return retourPonton_; }

    private:
        void registerBall_(BallType type);

        Config cfg_{};
        bool running_ = false;
        bool firstBall_ = false;
        bool retourPonton_ = false;
        bool verrouille_ = false;
        int score_ = 0;
        int predictedScore_ = 0;
        std::chrono::steady_clock::time_point lastCollect_{};
        std::map<BallType, int> counts_{};
    };

} // namespace CATJ_robot
