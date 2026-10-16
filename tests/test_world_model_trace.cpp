// Synthetic objects only: no server, socket connection, policy or match.
#include "world_model_trace.h"
#include <rcsc/player/player_agent.h>
#include <rcsc/player/localization_default.h>
#include <rcsc/common/audio_memory.h>
#include <rcsc/random.h>
#include <cassert>
#include <limits>
#include <memory>

class TraceFixture : public rcsc::PlayerAgent {
public:
    explicit TraceFixture(rcsc::SideID side)
    {
        M_worldmodel.setLocalization(std::make_shared<rcsc::LocalizationDefault>());
        M_worldmodel.setAudioMemory(std::make_shared<rcsc::AudioMemory>());
        assert(M_worldmodel.init("trace_test", side, 6, false, 18.0));
    }
    void actionImpl() override {}
    void communicationImpl() override {}
    void queueCommands()
    {
        M_effector.setMove(-1.25, 2.5);
        M_effector.setTurnNeck(rcsc::AngleDeg(15));
    }
    void clearCommands() { M_effector.clearAllCommands(); }
};

int main(int argc, char ** argv)
{
    assert(argc >= 2);
    const std::string mode = argv[1];
    TraceFixture agent(argc > 2 && std::string(argv[2]) == "r" ? rcsc::RIGHT : rcsc::LEFT);
    if (mode == "fixture") {
        agent.queueCommands();
        r2drl::writeWorldModelTrace(std::cout, agent.world(), "post_decision", "Base", "quote\"\n\\tab\t", 1,
                                   17, -1, 0, 0, &agent.effector());
        return 0;
    }
    if (mode == "primitives") {
        using namespace r2drl::wm_trace_detail;
        std::cout << '[';
        string(std::cout, "quote\"\n\\tab\t"); std::cout << ',';
        number(std::cout, std::numeric_limits<double>::quiet_NaN()); std::cout << ',';
        number(std::cout, std::numeric_limits<double>::infinity()); std::cout << ',';
        vector(std::cout, rcsc::Vector2D::INVALIDATED);
        std::cout << "]\n";
        return 0;
    }
    assert(mode == "trace");
    r2drl::WorldModelTrace trace;
    std::srand(12345);
    std::ostringstream engine_before;
    engine_before << rcsc::RandomEngine::instance().get();
    const int expected_next_rand = std::rand();
    std::srand(12345);
    for (int i = 0; i < 3; ++i) {
        trace.begin(agent.world(), i == 2 ? "Hybrid" : "Base", "synthetic_fixture");
        if (i == 2) trace.hybridAction(2, 0.25, 0.75);
        else trace.baseAction(17);
        agent.queueCommands();
        const auto * original_body = agent.effector().bodyCommand();
        trace.end(agent.world(), agent.effector());
        assert(agent.effector().bodyCommand() == original_body);
        agent.clearCommands();
    }
    assert(std::rand() == expected_next_rand);
    std::ostringstream engine_after;
    engine_after << rcsc::RandomEngine::instance().get();
    assert(engine_before.str() == engine_after.str());
}
