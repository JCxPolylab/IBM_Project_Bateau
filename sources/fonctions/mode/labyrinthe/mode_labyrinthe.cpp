#include "mode_labyrinthe.h"

#include <cmath>

namespace CATJ_robot {

ModeLabyrinthe::ModeLabyrinthe(Config cfg) : cfg_(cfg) {}

void ModeLabyrinthe::start()
{
    running_ = true;
    circuitComplete_ = false;
    contacts_ = 0;
}

void ModeLabyrinthe::stop()
{
    running_ = false;
}

void ModeLabyrinthe::update(const LidarDirections& d, MotherboardLink& board)
{
    if (!running_) return;

    if (d.rearMm < cfg_.rearReturnMm) {
        circuitComplete_ = true;
        board.sendStopAll();
        running_ = false;
        return;
    }

    if (d.frontMm < cfg_.frontAvoidMm) {
        board.sendSpeedPct(cfg_.speedPct);
        if (d.leftMm > d.rightMm) {
            board.sendTurnDeg(-cfg_.turnAngleDeg);
        } else {
            board.sendTurnDeg(+cfg_.turnAngleDeg);
        }
        board.sendMove(MoveCommand::Forward);
        contacts_ += (d.frontMm < 200.0f) ? 1 : 0;
        return;
    }

    wallFollow_(d.rightMm, board);
}

void ModeLabyrinthe::wallFollow_(float rightMm, MotherboardLink& board)
{
    board.sendSpeedPct(cfg_.speedPct);
    if (!std::isfinite(rightMm)) {
        board.sendTurnDeg(+8.0f);
        board.sendMove(MoveCommand::Forward);
        return;
    }

    const float error = cfg_.wallFollowMm - rightMm;
    if (std::fabs(error) < cfg_.wallToleranceMm) {
        board.sendTurnDeg(0.0f);
        board.sendMove(MoveCommand::Forward);
        return;
    }

    if (error > 0.0f) {
        board.sendTurnDeg(-8.0f); // trop proche du mur droit -> corriger à gauche
    } else {
        board.sendTurnDeg(+8.0f); // trop loin du mur droit -> corriger à droite
    }
    board.sendMove(MoveCommand::Forward);
}

} // namespace CATJ_robot
