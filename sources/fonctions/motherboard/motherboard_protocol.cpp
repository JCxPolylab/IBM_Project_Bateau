#include "motherboard_protocol.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace CATJ_robot {

MotherboardLink::MotherboardLink(const MotherboardConfig& cfg)
{
    open(cfg);
}

bool MotherboardLink::open(const MotherboardConfig& cfg)
{
    std::scoped_lock lock(mutex_);
    cfg_ = cfg;
    lastStatus_ = {};
    return uart_.open(cfg_.uart);
}

void MotherboardLink::close()
{
    std::scoped_lock lock(mutex_);
    uart_.close();
}

bool MotherboardLink::isOpen() const
{
    return uart_.isOpen();
}

bool MotherboardLink::sendTurnDeg(float angleDeg)
{
    std::ostringstream oss;
    oss << "TURN " << std::lround(angleDeg);
    return sendLine_(oss.str());
}

bool MotherboardLink::sendMove(MoveCommand move)
{
    return sendLine_(std::string("MOVE ") + moveToString(move));
}

bool MotherboardLink::sendSpeedPct(int speedPct)
{
    speedPct = std::clamp(speedPct, 0, 100);
    std::ostringstream oss;
    oss << "SPEED " << speedPct;
    return sendLine_(oss.str());
}

bool MotherboardLink::sendCatch()
{
    return sendLine_("CATCH");
}

bool MotherboardLink::sendMode(RobotMode mode)
{
    return sendLine_(std::string("MODE ") + modeToString(mode));
}

bool MotherboardLink::sendObjective(int predictedScore)
{
    std::ostringstream oss;
    oss << "OBJECTIF " << predictedScore;
    return sendLine_(oss.str());
}

bool MotherboardLink::sendStopAll()
{
    return sendLine_("MOVE STOP") && sendLine_("SPEED 0");
}

bool MotherboardLink::sendHeartbeat()
{
    return sendLine_("CATJ");
}

bool MotherboardLink::requestStatus()
{
    return sendLine_("GET_STATUS");
}

std::optional<MotherboardStatus> MotherboardLink::readStatusOnce(int timeoutMs)
{
    std::scoped_lock lock(mutex_);
    if (!uart_.isOpen()) {
        return std::nullopt;
    }

    std::string line;
    const int effectiveTimeout = (timeoutMs >= 0) ? timeoutMs : cfg_.statusTimeoutMs;
    if (!uart_.readLine(line, '\n', effectiveTimeout, 256)) {
        return std::nullopt;
    }

    auto st = parseLine_(line);
    if (st) {
        lastStatus_ = *st;
    }
    return st;
}

bool MotherboardLink::sendLine_(const std::string& line)
{
    std::scoped_lock lock(mutex_);
    if (!uart_.isOpen()) {
        return false;
    }
    return uart_.writeString(line + "\n") > 0;
}

std::optional<MotherboardStatus> MotherboardLink::parseLine_(const std::string& line) const
{
    MotherboardStatus st;
    st.valid = true;
    st.rawLine = trim_(line);

    std::istringstream iss(st.rawLine);
    std::string key;
    iss >> key;
    key = upper_(key);

    if (key == "ETAT") {
        int value = 0;
        iss >> value;
        st.etat = (value != 0);
        return st;
    }

    if (key == "ACK") {
        std::string sub;
        std::string result;
        iss >> sub >> result;
        sub = upper_(sub);
        result = upper_(result);
        if (sub == "CATCH") {
            st.catchAck = (result == "OK") ? CatchAck::Ok : CatchAck::Failed;
            return st;
        }
    }

    if (key == "BALLE") {
        std::string kind;
        iss >> kind;
        kind = upper_(kind);
        if (kind == "SCORE") {
            st.balle = BallKind::Score;
        } else if (kind == "TRASH") {
            st.balle = BallKind::Trash;
        }
        return st;
    }

    if (key == "SPEEDVAL") {
        iss >> st.speedVal;
        return st;
    }

    // Variante simple: STATUS etat=1 catch=ok balle=trash speed=42
    if (key == "STATUS") {
        std::string token;
        while (iss >> token) {
            auto pos = token.find('=');
            if (pos == std::string::npos) continue;
            std::string lhs = upper_(token.substr(0, pos));
            std::string rhs = upper_(token.substr(pos + 1));
            if (lhs == "ETAT") st.etat = (rhs == "1" || rhs == "TRUE" || rhs == "OK");
            else if (lhs == "CATCH") st.catchAck = (rhs == "OK") ? CatchAck::Ok : CatchAck::Failed;
            else if (lhs == "BALLE") st.balle = (rhs == "SCORE") ? BallKind::Score : (rhs == "TRASH") ? BallKind::Trash : BallKind::Unknown;
            else if (lhs == "SPEED") st.speedVal = std::atoi(rhs.c_str());
        }
        return st;
    }

    return st; // on conserve la ligne brute même si format partiel
}

std::string MotherboardLink::moveToString(MoveCommand move)
{
    switch (move) {
    case MoveCommand::Forward: return "FORWARD";
    case MoveCommand::Backward: return "BACKWARD";
    case MoveCommand::Stop:
    default: return "STOP";
    }
}

std::string MotherboardLink::modeToString(RobotMode mode)
{
    switch (mode) {
    case RobotMode::Labyrinthe: return "LABYRINTHE";
    case RobotMode::Ramassage:  return "RAMASSAGE";
    case RobotMode::Course:     return "COURSE";
    case RobotMode::Debug:      return "DEBUG";
    case RobotMode::Stop:
    default: return "STOP";
    }
}

std::string MotherboardLink::catchAckToString(CatchAck v)
{
    switch (v) {
    case CatchAck::Ok: return "OK";
    case CatchAck::Failed: return "FAILED";
    default: return "UNKNOWN";
    }
}

std::string MotherboardLink::ballKindToString(BallKind v)
{
    switch (v) {
    case BallKind::Score: return "SCORE";
    case BallKind::Trash: return "TRASH";
    default: return "UNKNOWN";
    }
}

std::string MotherboardLink::trim_(std::string s)
{
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

std::string MotherboardLink::upper_(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

bool MotherboardLink::startsWith_(const std::string& s, const std::string& prefix)
{
    return s.rfind(prefix, 0) == 0;
}

} // namespace CATJ_robot
