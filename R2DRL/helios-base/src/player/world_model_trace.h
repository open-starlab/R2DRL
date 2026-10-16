// Read-only diagnostics. This is NOT a restorable WorldModel checkpoint.
#ifndef R2DRL_WORLD_MODEL_TRACE_H
#define R2DRL_WORLD_MODEL_TRACE_H

#include <rcsc/player/world_model.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/action_effector.h>
#include <rcsc/player/player_command.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <unistd.h>

namespace r2drl {
namespace wm_trace_detail {

inline void string(std::ostream & out, const std::string & value)
{
    static const char hex[] = "0123456789abcdef";
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 0x20) out << "\\u00" << hex[c >> 4] << hex[c & 15];
        else out << char(c);
    }
    out << '"';
}

inline void number(std::ostream & out, double value)
{
    if (std::isfinite(value)) out << value;
    else out << "null";
}

inline void vector(std::ostream & out, const rcsc::Vector2D & value)
{
    if (!value.isValid()) { out << "null"; return; }
    out << '[';
    number(out, value.x); out << ','; number(out, value.y);
    out << ']';
}

inline char side(rcsc::SideID value)
{
    return value == rcsc::LEFT ? 'l' : value == rcsc::RIGHT ? 'r' : '-';
}

inline void playerFields(std::ostream & out, const rcsc::AbstractPlayerObject & p)
{
    out << "\"side\":\"" << side(p.side()) << "\",\"unum\":" << p.unum()
        << ",\"goalie\":" << (p.goalie() ? "true" : "false") << ",\"position\":";
    vector(out, p.pos()); out << ",\"velocity\":"; vector(out, p.vel());
    out << ",\"body_deg\":"; number(out, p.body().degree());
    out << ",\"face_deg\":"; number(out, p.face().degree());
    out << ",\"unum_count\":" << p.unumCount()
        << ",\"pos_count\":" << p.posCount()
        << ",\"seen_pos_count\":" << p.seenPosCount()
        << ",\"heard_pos_count\":" << p.heardPosCount()
        << ",\"vel_count\":" << p.velCount()
        << ",\"body_count\":" << p.bodyCount()
        << ",\"card\":" << int(p.card());
}

inline void playerIdentity(std::ostream & out, const rcsc::PlayerObject * p)
{
    if (!p) { out << "null"; return; }
    out << "{\"side\":\"" << side(p->side()) << "\",\"unum\":" << p->unum() << '}';
}

template <typename Command>
inline void command(std::ostream & out, const Command * value)
{
    if (!value) { out << "null"; return; }
    std::ostringstream text;
    text.imbue(std::locale::classic());
    value->toCommandString(text);
    string(out, text.str());
}

} // namespace wm_trace_detail

// Uses public const accessors only. Never requests fullstate or alters the WM,
// action mask, planner, shared-memory layout, or any policy random generator.
inline void writeWorldModelTrace(std::ostream & out, const rcsc::WorldModel & wm,
                                const char * phase, const std::string & mode,
                                const std::string & source, unsigned long long decision,
                                int base_action, int hybrid_action, double u0, double u1,
                                const rcsc::ActionEffector * effector = nullptr)
{
    using namespace wm_trace_detail;
    out.imbue(std::locale::classic());
    out << std::setprecision(17)
        << "{\"schema\":\"r2drl_wm_trace_v1\",\"phase\":";
    string(out, phase);
    out << ",\"coordinate_frame\":\"librcsc_world_model\",\"cycle\":" << wm.time().cycle()
        << ",\"stopped\":" << wm.time().stopped()
        << ",\"decision_index\":" << decision << ",\"pid\":" << ::getpid()
        << ",\"team\":"; string(out, wm.ourTeamName());
    out << ",\"side\":\"" << side(wm.ourSide()) << "\",\"source_shm\":"; string(out, source);
    out << ",\"mode\":"; string(out, mode);
    out << ",\"game_mode_id\":" << int(wm.gameMode().type())
        << ",\"base_action_id\":";
    if (base_action < 0) out << "null"; else out << base_action;
    out << ",\"hybrid_action_id\":";
    if (hybrid_action < 0) out << "null"; else out << hybrid_action;
    out << ",\"hybrid_parameters\":";
    if (hybrid_action < 0) out << "null";
    else { out << '['; number(out,u0); out << ','; number(out,u1); out << ']'; }

    const auto & self = wm.self();
    const auto & ball = wm.ball();
    out << ",\"self\":{"; playerFields(out, self);
    out << ",\"neck_deg\":"; number(out, self.neck().degree());
    out << ",\"pos_valid\":" << (self.posValid() ? "true" : "false")
        << ",\"vel_valid\":" << (self.velValid() ? "true" : "false")
        << ",\"face_valid\":" << (self.faceValid() ? "true" : "false");
    out << ",\"stamina\":"; number(out, self.stamina());
    out << ",\"kickable\":" << (self.isKickable() ? "true" : "false")
        << ",\"frozen\":" << (self.isFrozen() ? "true" : "false")
        << ",\"tackle_expires\":" << self.tackleExpires()
        << ",\"charged_expires\":" << self.chargedExpires()
        << ",\"tackle_probability\":"; number(out, self.tackleProbability());
    out << ",\"foul_probability\":"; number(out, self.foulProbability());
    out << ",\"catch_probability\":"; number(out, self.catchProbability());
    out << "},\"ball\":{\"position\":"; vector(out, ball.pos());
    out << ",\"velocity\":"; vector(out, ball.vel());
    out << ",\"relative_position\":"; vector(out, ball.rpos());
    out << ",\"seen_position\":"; vector(out, ball.seenPos());
    out << ",\"heard_position\":"; vector(out, ball.heardPos());
    out << ",\"pos_valid\":" << (ball.posValid() ? "true" : "false")
        << ",\"vel_valid\":" << (ball.velValid() ? "true" : "false")
        << ",\"rpos_valid\":" << (ball.rposValid() ? "true" : "false")
        << ",\"pos_count\":" << ball.posCount()
        << ",\"rpos_count\":" << ball.rposCount()
        << ",\"seen_pos_count\":" << ball.seenPosCount()
        << ",\"heard_pos_count\":" << ball.heardPosCount()
        << ",\"vel_count\":" << ball.velCount()
        << ",\"seen_vel_count\":" << ball.seenVelCount()
        << ",\"heard_vel_count\":" << ball.heardVelCount() << '}';

    const auto & intercept = wm.interceptTable();
    out << ",\"intercept\":{\"self_steps\":" << intercept.selfStep()
        << ",\"teammate_steps\":" << intercept.teammateStep()
        << ",\"opponent_steps\":" << intercept.opponentStep()
        << ",\"first_teammate\":"; playerIdentity(out, intercept.firstTeammate());
    out << ",\"first_opponent\":"; playerIdentity(out, intercept.firstOpponent());
    out << "},\"kickable_teammate\":"; playerIdentity(out, wm.kickableTeammate());
    out << ",\"kickable_opponent\":"; playerIdentity(out, wm.kickableOpponent());
    out << ",\"players\":[";
    bool first = true;
    for (const auto * player : wm.allPlayers()) {
        if (!player) continue;
        if (!first) out << ',';
        first = false;
        out << '{'; playerFields(out, *player); out << '}';
    }
    out << "],\"commands\":";
    if (!effector) out << "null";
    else {
        out << "{\"body\":"; command(out, effector->bodyCommand());
        out << ",\"turn_neck\":"; command(out, effector->turnNeckCommand());
        out << ",\"change_view\":"; command(out, effector->changeViewCommand());
        out << ",\"status\":\"queued_not_server_acknowledged\"}";
    }
    out << "}\n";
}

class WorldModelTrace {
    bool initialized_ = false;
    bool enabled_ = false;
    bool sampled_ = false;
    std::string directory_, mode_, source_;
    char selected_side_ = '-';
    unsigned long long every_ = 1, decision_ = 0;
    int base_action_ = -1, hybrid_action_ = -1;
    double u0_ = 0, u1_ = 0;
    std::ofstream file_;

    void fail(const char * reason)
    {
        std::cerr << "[WM_TRACE] disabled: " << reason << '\n';
        enabled_ = false;
        sampled_ = false;
        if (file_.is_open()) file_.close();
    }

    void initialize()
    {
        initialized_ = true;
        const char * directory = std::getenv("R2DRL_WM_TRACE_DIR");
        if (!directory || !directory[0]) return; // Default: no file or directory I/O.
        directory_ = directory;
        const char * interval = std::getenv("R2DRL_WM_TRACE_EVERY");
        if (interval) {
            char * end = nullptr;
            const unsigned long value = std::strtoul(interval, &end, 10);
            if (!interval[0] || *end || interval[0] == '-' || value == 0
                || value > 1000000) { fail("R2DRL_WM_TRACE_EVERY must be 1..1000000"); return; }
            every_ = value;
        }
        const char * selection = std::getenv("R2DRL_WM_TRACE_SIDE");
        if (selection && std::string(selection) != "all") {
            if (std::string(selection) == "l" || std::string(selection) == "r") selected_side_ = selection[0];
            else { fail("R2DRL_WM_TRACE_SIDE must be l, r, or all"); return; }
        }
        enabled_ = true;
    }

public:
    void begin(const rcsc::WorldModel & wm, const std::string & mode, const std::string & source)
    {
        if (!initialized_) initialize();
        if (!enabled_) return;
        sampled_ = false;
        if (selected_side_ != '-' && selected_side_ != wm_trace_detail::side(wm.ourSide())) return;
        ++decision_;
        if ((decision_ - 1) % every_) return;
        sampled_ = true;
        mode_ = mode; source_ = source;
        base_action_ = hybrid_action_ = -1;
        if (!file_.is_open()) {
            const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            std::ostringstream path;
            path << directory_ << "/wm_" << wm_trace_detail::side(wm.ourSide())
                 << "_u" << wm.self().unum() << "_pid" << ::getpid() << '_' << stamp << ".jsonl";
            file_.open(path.str(), std::ios::out | std::ios::app);
            if (!file_) { fail("cannot open output; create R2DRL_WM_TRACE_DIR first"); return; }
            std::cerr << "[WM_TRACE] " << path.str() << '\n';
        }
        writeWorldModelTrace(file_, wm, "pre_decision", mode_, source_, decision_, -1, -1, 0, 0);
        file_.flush();
        if (!file_) fail("write failed");
    }

    void baseAction(int action) { if (sampled_) base_action_ = action; }
    void hybridAction(int action, double u0, double u1)
    { if (sampled_) { hybrid_action_ = action; u0_ = u0; u1_ = u1; } }

    void end(const rcsc::WorldModel & wm, const rcsc::ActionEffector & effector)
    {
        if (!enabled_ || !sampled_) return;
        // Called at the end of communicationImpl, after neck/view decisions
        // but BEFORE makeCommand deletes commands. Not the next server state.
        writeWorldModelTrace(file_, wm, "post_decision", mode_, source_, decision_,
                             base_action_, hybrid_action_, u0_, u1_, &effector);
        file_.flush();
        sampled_ = false;
        if (!file_) fail("write failed");
    }
};

} // namespace r2drl
#endif
