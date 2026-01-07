// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <errno.h>

#include "sample_player.h"

#include "strategy.h"
#include "field_analyzer.h"

#include "action_chain_holder.h"
#include "sample_field_evaluator.h"

#include "soccer_role.h"

#include "sample_communication.h"
#include "keepaway_communication.h"
#include "sample_freeform_message_parser.h"

#include "bhv_penalty_kick.h"
#include "bhv_set_play.h"
#include "bhv_set_play_kick_in.h"
#include "bhv_set_play_indirect_free_kick.h"

#include "bhv_custom_before_kick_off.h"
#include "bhv_strict_check_shoot.h"

#include "view_tactical.h"

#include "intention_receive.h"

#include "bhv_basic_tackle.h"
#include "bhv_basic_move.h"


#include "basic_actions/basic_actions.h"
#include "basic_actions/body_hold_ball2008.h"
#include "basic_actions/bhv_emergency.h"
#include "basic_actions/body_go_to_point.h"
#include "basic_actions/body_intercept.h"
#include "basic_actions/body_kick_one_step.h"
#include "basic_actions/neck_scan_field.h"
#include "basic_actions/neck_turn_to_ball_or_scan.h"
#include "basic_actions/view_synch.h"
#include "basic_actions/kick_table.h"
#include "basic_actions/body_pass.h"
#include "basic_actions/body_dribble2008.h"
#include "basic_actions/body_advance_ball2009.h"

#include <rcsc/formation/formation.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/say_message_builder.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/player_agent.h>

#include <rcsc/common/abstract_client.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/audio_memory.h>
#include <rcsc/common/say_message_parser.h>

#include <rcsc/param/param_map.h>
#include <rcsc/param/cmd_line_parser.h>
#include <rcsc/common/server_param.h>
#include <cmath>

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <random>
#include <cassert>
#include <rcsc/player/player_object.h>
#include <atomic>

#include <thread>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cstdlib>  // 顶部
using namespace rcsc;


#include <sys/mman.h>   // shm_open/mmap 所需
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>      // std::memset

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::SamplePlayer()
    : PlayerAgent(),
      M_communication(),
      shm_ptr(nullptr)
{
    M_field_evaluator = createFieldEvaluator();
    M_action_generator = createActionGenerator();

    std::shared_ptr< AudioMemory > audio_memory( new AudioMemory );
    action_mask.fill(true);
    M_worldmodel.setAudioMemory( audio_memory );

    //
    // set communication message parser
    //
    addSayMessageParser( new BallMessageParser( audio_memory ) );
    addSayMessageParser( new PassMessageParser( audio_memory ) );
    addSayMessageParser( new InterceptMessageParser( audio_memory ) );
    addSayMessageParser( new GoalieMessageParser( audio_memory ) );
    addSayMessageParser( new GoalieAndPlayerMessageParser( audio_memory ) );
    addSayMessageParser( new OffsideLineMessageParser( audio_memory ) );
    addSayMessageParser( new DefenseLineMessageParser( audio_memory ) );
    addSayMessageParser( new WaitRequestMessageParser( audio_memory ) );
    addSayMessageParser( new PassRequestMessageParser( audio_memory ) );
    addSayMessageParser( new DribbleMessageParser( audio_memory ) );
    addSayMessageParser( new BallGoalieMessageParser( audio_memory ) );
    addSayMessageParser( new OnePlayerMessageParser( audio_memory ) );
    addSayMessageParser( new TwoPlayerMessageParser( audio_memory ) );
    addSayMessageParser( new ThreePlayerMessageParser( audio_memory ) );
    addSayMessageParser( new SelfMessageParser( audio_memory ) );
    addSayMessageParser( new TeammateMessageParser( audio_memory ) );
    addSayMessageParser( new OpponentMessageParser( audio_memory ) );
    addSayMessageParser( new BallPlayerMessageParser( audio_memory ) );
    addSayMessageParser( new StaminaMessageParser( audio_memory ) );
    addSayMessageParser( new RecoveryMessageParser( audio_memory ) );

    // addSayMessageParser( new FreeMessageParser< 9 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 8 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 7 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 6 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 5 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 4 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 3 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 2 >( audio_memory ) );
    // addSayMessageParser( new FreeMessageParser< 1 >( audio_memory ) );

    //
    // set freeform message parser
    //
    addFreeformMessageParser( new OpponentPlayerTypeMessageParser( M_worldmodel ) );

    //
    // set communication planner
    //
    M_communication = Communication::Ptr( new SampleCommunication() );
}

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::~SamplePlayer()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
bool
SamplePlayer::initImpl( CmdLineParser & cmd_parser )
{
    // 0) 先把现在 cmd line 的内容打出来（可选）

    // 1) 调用基类 initImpl
    bool result = PlayerAgent::initImpl( cmd_parser );

    // 2) Strategy 初始化
    bool strat_ok = Strategy::instance().init( cmd_parser );
    result &= strat_ok;

    // 3) 定义 ParamMap & 注册参数
    rcsc::ParamMap my_params( "Additional options" );

    my_params.add()
        ( "shm-name", "",       &SHM_NAME,   "shared memory name" )
        ( "mode",     "Helios", &RUN_MODE_,  "run mode: Base|Helios|Hybrid" );

    // 4) 解析附加参数
    cmd_parser.parse( my_params );


    // 5) 归一化 RUN_MODE_ → mode_
    std::string m = RUN_MODE_;
    std::transform( m.begin(), m.end(), m.begin(),
                    []( unsigned char c ){ return static_cast<char>( std::tolower( c ) ); } );

    if ( m == "base" )
    {
        mode_ = Mode::Base;
    }
    else if ( m == "hybrid" )
    {
        mode_ = Mode::Hybrid;
    }
    else
    {
        mode_ = Mode::Helios;  // 默认兜底
    }

    // 6) 处理 help 选项
    int help_cnt = cmd_parser.count( "help" );
    if ( help_cnt > 0 )
    {
        return false;
    }

    // 7) 检查是否有不支持的选项
    bool failed = cmd_parser.failed();
    if ( failed )
    {
    }

    // 8) 检查前面 init 的总结果
    if ( ! result )
    {
        return false;
    }

    // 9) 读取 team strategy
    std::string cfg_dir = config().configDir();
    if ( ! Strategy::instance().read( cfg_dir ) )
    {
        return false;
    }
    std::cerr << "[INIT] team=" << config().teamName()
        << " mode=" << RUN_MODE_
        << " cfg_dir=" << cfg_dir
        << " shm_name=" << SHM_NAME
        << std::endl;

    // 10) 读取 kick table
    std::string kick_path = cfg_dir + "/kick-table";
    if ( KickTable::instance().read( kick_path ) )
    {
    }
    else
    {
    }

    // 11) 打印关键信息

    // 12) 共享内存相关
    if ( mode_ == Mode::Base || mode_ == Mode::Hybrid )
    {

        if ( SHM_NAME.empty() )
        {
            return false;
        }


        if ( ! initSharedMemory() )
        {
            return false;
        }
    }
    else
    {
    }

    // 13) 最终确认 world/self 信息


    return true;
}


// ------- 小工具：算“球相对身体”的角度（度） -------
// 加了详细日志
static inline double relDirToBallDeg(const rcsc::WorldModel & wm){
    const int cyc = wm.time().cycle();
    const rcsc::Vector2D self_pos = wm.self().pos();
    const rcsc::Vector2D ball_pos = wm.ball().pos();

    rcsc::AngleDeg to_ball = (ball_pos - self_pos).th();
    double rel_deg = (to_ball - wm.self().body()).degree();  // 范围约在 [-180, 180]


    return rel_deg;
}

// 0:TURN, 1:DASH, 2:KICK, 3:CATCH
// 返回 true 表示本帧已发送了1条身体指令；false 表示没发
bool SamplePlayer::takeHybridAction(int a, double u0, double u1)
{
    const rcsc::WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();

    const bool frozen    = wm.self().isFrozen();
    const bool kickable  = wm.self().isKickable();
    const bool is_goalie = wm.self().goalie();


    // ---- 仅在本函数内可见的归一化 → 物理量 映射 ----
    auto clamp01 = [](double x)->double {
        double y = std::max(0.0, std::min(1.0, x));
        return y;
    };
    auto u_to_deg180 = [&](double u)->double {               // [-180, 180]°
        double u_c = clamp01(u);
        double val = (u_c * 2.0 - 1.0) * 180.0;
        return val;
    };
    auto u_to_dash = [&](double u)->double {                 // [-maxDash, +maxDash]
        const double maxDash = rcsc::ServerParam::i().maxDashPower(); // 通常 100
        double u_c = clamp01(u);
        double val = (u_c * 2.0 - 1.0) * maxDash;
        return val;
    };
    auto u_to_kick_power = [&](double u)->double {           // [-maxPower, +maxPower]
        const double maxP = rcsc::ServerParam::i().maxPower();         // 通常 100
        double u_c = clamp01(u);
        double val = (u_c * 2.0 - 1.0) * maxP;
        return val;
    };

    bool sent = false;

    // ---- 只发“一条”身体指令（turn/dash/kick/catch 之一）----
    switch(a){
        case 0: { // TURN(moment°) ；u0∈[0,1] → [-180,180]°
            const double moment_deg = u_to_deg180(u0);
            this->doTurn(moment_deg);
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            sent = true;
            break;
        }
        case 1: { // DASH
            const double power = 1.0 + 99.0 * clamp01(u0); // [1,100]
            this->doDash(power);                            // 只给正功率
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            sent = true;
            break;
        }
        case 2: { // KICK(power,dir)；u0→力度, u1→方向（相对身体，度）
            if (!wm.self().isKickable()) {
                sent = false;
                break;
            }
            const double power   = u_to_kick_power(u0);
            const double dir_deg = u_to_deg180(u1);
            this->doKick(power, dir_deg);
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            sent = true;
            break;
        }
        case 3: { // CATCH(dir)（当前无参 doCatch()）
            if (!wm.self().goalie()) {
                sent = false;
                break;
            }
            this->doCatch();
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            sent = true;
            break;
        }
        default:
            sent = false;
            break;
    }


    return sent;
}

void SamplePlayer::writeHybridMaskToSharedMemory()
{
    const int cyc = world().time().cycle();


    if (!shm_ptr) {
        return;
    }

    auto* base       = static_cast<uint8_t*>(shm_ptr);
    uint8_t* hy_mask = base + OFFSET_HYBRID_MASK;

    const auto m = getHybridActionMask();


    for (int i = 0; i < 4; ++i) {
        hy_mask[i] = static_cast<uint8_t>(m[i]);
    }


}


bool SamplePlayer::readHybridActionFromSharedMemory(int &a, float &u0, float &u1, int timeout_ms)
{
    const int my_cycle = world().time().cycle();


    if (!shm_ptr) {
        throw std::runtime_error("shm_ptr null (hybrid)");
    }

    auto* base                 = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A   = base + OFFSET_FLAG_A;
    volatile uint8_t* flag_B   = base + OFFSET_FLAG_B;
    int32_t* shm_cycle         = reinterpret_cast<int32_t*>(base + OFFSET_CYCLE);
    int32_t* shm_hybrid_action = reinterpret_cast<int32_t*>(base + OFFSET_HYBRID_ACT);
    float*   shm_u0            = reinterpret_cast<float*>(base + OFFSET_HYBRID_U0);
    float*   shm_u1            = reinterpret_cast<float*>(base + OFFSET_HYBRID_U1);


    // 和 Base 的握手保持一致：等待 Python 写完 (A==1,B==0)
    const auto start = std::chrono::steady_clock::now();
    const auto TO    = std::chrono::milliseconds(timeout_ms);
    int loop_cnt = 0;

    while (!(*flag_A == 1 && *flag_B == 0)) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ++loop_cnt;

        if (loop_cnt % 10000 == 0) {
        }
        if (std::chrono::steady_clock::now() - start > TO) {
            std::ostringstream oss;
            oss << "[Hybrid] wait action timeout at cycle=" << my_cycle
                << " flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << " shm_cycle=" << *shm_cycle;
            dlog.addText(Logger::TEAM,"[Hybrid][ERROR] ... wait action timeout");

            return false; // 超时：让上层兜底
        }
    }

    std::atomic_thread_fence(std::memory_order_acquire);

    a  = *shm_hybrid_action;
    u0 = *shm_u0;
    u1 = *shm_u1;


    // 保护：a 合法、u0/u1 合法
    if (a < 0 || a > 3) {
        dlog.addText(Logger::TEAM,"[Hybrid][ERROR] ... invalid action");

        return false;
    }

    auto clamp01 = [](float x){ return std::max(0.0f, std::min(1.0f, x)); };
    u0 = clamp01(u0);
    u1 = clamp01(u1);



    return true;
}


void SamplePlayer::writeSharedMemory()
{
    const int my_cycle = world().time().cycle();


    if (!shm_ptr) {
        dlog.addText(Logger::TEAM,"[Hybrid][ERROR] ... shm_ptr null in writeSharedMemory");
        throw std::runtime_error("shm_ptr is null — shared memory write aborted");
    }

    // 指针区
    auto* base = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A = base + OFFSET_FLAG_A;
    volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
    uint8_t*  shm_mask  = base + OFFSET_MASK;
    int32_t*  shm_cycle = reinterpret_cast<int32_t*>(base + OFFSET_CYCLE);
    float*    shm_state = reinterpret_cast<float*>(base + OFFSET_STATE);
    int32_t*  shm_action= reinterpret_cast<int32_t*>(base + OFFSET_ACTION);


    // 1) 写 mask
    const auto& mask = getActionMask();
    for (int i = 0; i < BASE_ACTION_NUM; ++i) {
        shm_mask[i] = static_cast<uint8_t>(mask[i] ? 1 : 0);
    }
    for (int i = 0; i < std::min(static_cast<int>(BASE_ACTION_NUM), 17); ++i) {
    }

    // 2) 写 cycle
    const int32_t cycle = static_cast<int32_t>(world().time().cycle());
    *shm_cycle = cycle;

    // 3) 写 obs
    const auto state = getAllState();
    for (size_t i = 0; i < state.size(); ++i) {
        shm_state[i] = state[i];
    }
    for (int i = 0; i < 5 && i < static_cast<int>(state.size()); ++i) {
    }

    // 这里不改 flag 协议，只打印当前 flag 值

}


// --- 放在合适的位置，例如与其它 mask/utility 函数放一起 ---

// 仅用于 Hybrid 四个底层动作：0:TURN, 1:DASH, 2:KICK, 3:CATCH
std::array<bool,4> SamplePlayer::getHybridActionMask() const
{
    const rcsc::WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();

    const bool frozen    = wm.self().isFrozen();
    const bool kickable  = wm.self().isKickable();
    const bool is_goalie = wm.self().goalie();


    std::array<bool,4> m{};
    m[0] = true;                    // TURN: 总是允许
    m[1] = !frozen;                 // DASH: 冻结时不允许
    m[2] = (!frozen && kickable);   // KICK: 要能踢球且未冻结
    m[3] = (!frozen && is_goalie && kickable);  // CATCH



    return m;
}

// 如果你仍然用全局的 action_mask(std::array<bool, BASE_ACTION_NUM>) 往共享内存写：
// 把前4位写为 Hybrid 掩码，其它清 False（避免 Python 侧误读）。
void SamplePlayer::setHybridActionMask()
{
    const int cyc = world().time().cycle();


    const auto m = getHybridActionMask();

    // 打印旧的前 8 个 mask 方便比对
    for (int i = 0; i < std::min(8, (int)BASE_ACTION_NUM); ++i) {
    }

    action_mask.fill(false);
    if (BASE_ACTION_NUM > 0) action_mask[0] = m[0]; // TURN
    if (BASE_ACTION_NUM > 1) action_mask[1] = m[1]; // DASH
    if (BASE_ACTION_NUM > 2) action_mask[2] = m[2]; // KICK
    if (BASE_ACTION_NUM > 3) action_mask[3] = m[3]; // CATCH

    for (int i = 0; i < std::min(8, (int)BASE_ACTION_NUM); ++i) {
    }

}

void
SamplePlayer::actionImpl()
{
    const int unum = world().self().unum();
    const int cyc = world().time().cycle();
    const int gm  = static_cast<int>(world().gameMode().type());
    std::cerr << "[HB] pid=" << ::getpid()
            << " unum=" << unum
            << " cyc=" << cyc
            << " gm=" << int(world().gameMode().type())
            << " mode=" << int(mode_)
            << "\n";
    auto shm_base = [&]() -> uint8_t* {
        return shm_ptr ? static_cast<uint8_t*>(shm_ptr) : nullptr;
    };

    auto shm_flags = [&](volatile uint8_t*& A, volatile uint8_t*& B) -> bool {
        uint8_t* base = shm_base();
        if (!base) return false;
        A = base + OFFSET_FLAG_A;
        B = base + OFFSET_FLAG_B;
        return true;
    };

    // write B then fence then A (same ordering as your Python helper)
    auto shm_set_flags = [&](volatile uint8_t* A, volatile uint8_t* B, uint8_t a, uint8_t b) {
        *B = b;
        std::atomic_thread_fence(std::memory_order_release);
        *A = a;
    };

    auto cerr_flags = [&](const char* tag) {
        if (!shm_ptr) {
            std::cerr << "[SHM][" << tag << "] cyc=" << cyc
                      << " gm=" << gm
                      << " mode=" << int(mode_)
                      << " shm_ptr=null"
                      << " kickable=" << int(world().self().isKickable())
                      << " frozen=" << int(world().self().isFrozen())
                      << " goalie=" << int(world().self().goalie())
                      << "\n";
            return;
        }
        volatile uint8_t* A = nullptr;
        volatile uint8_t* B = nullptr;
        if (!shm_flags(A, B)) {
            std::cerr << "[SHM][" << tag << "] cyc=" << cyc
                      << " gm=" << gm
                      << " mode=" << int(mode_)
                      << " shm_flags=fail\n";
            return;
        }
        std::cerr << "[SHM][" << tag << "] cyc=" << cyc
                  << " gm=" << gm
                  << " mode=" << int(mode_)
                  << " flags=(" << int(*A) << "," << int(*B) << ")"
                  << " kickable=" << int(world().self().isKickable())
                  << " frozen=" << int(world().self().isFrozen())
                  << " goalie=" << int(world().self().goalie())
                  << "\n";
    };

    // only spam around PlayOn entry
    if (world().gameMode().type() == GameMode::PlayOn || cyc < 5) {
        cerr_flags("enter");
    }

    // =========================
    // PlayOn + Base
    // =========================
    if (world().gameMode().type() == GameMode::PlayOn && mode_ == Mode::Base)
    {
        if (!shm_ptr) throw std::runtime_error("shm_ptr null in Base mode");

        cerr_flags("Base:begin");

        setActionMask();
        writeSharedMemory();

        volatile uint8_t* flag_A = nullptr;
        volatile uint8_t* flag_B = nullptr;
        if (!shm_flags(flag_A, flag_B)) throw std::runtime_error("shm_flags failed (Base)");

        // obs ready => (0,1)
        shm_set_flags(flag_A, flag_B, 0, 1);
        cerr_flags("Base:obs_ready");

        const int act = getActionFromSharedMemory(); // TIMEOUT inside changed to 45000ms

        takeAction(act);

        // done => (1,1)
        shm_set_flags(flag_A, flag_B, 1, 1);
        cerr_flags("Base:done");
        return;
    }

    // =========================
    // PlayOn + Hybrid
    // =========================
    if (world().gameMode().type() == GameMode::PlayOn && mode_ == Mode::Hybrid)
    {
        if (!shm_ptr) throw std::runtime_error("shm_ptr null in Hybrid mode");

        cerr_flags("Hybrid:begin");

        setHybridActionMask();
        writeSharedMemory();
        writeHybridMaskToSharedMemory();

        volatile uint8_t* flag_A = nullptr;
        volatile uint8_t* flag_B = nullptr;
        if (!shm_flags(flag_A, flag_B)) throw std::runtime_error("shm_flags failed (Hybrid)");

        // obs ready => (0,1)
        shm_set_flags(flag_A, flag_B, 0, 1);
        cerr_flags("Hybrid:obs_ready");

        int   a  = -1;
        float u0 = 0.5f;
        float u1 = 0.5f;

        if (!readHybridActionFromSharedMemory(a, u0, u1, /*timeout_ms=*/45000)) {
            std::ostringstream oss;
            oss << "[Hybrid][ERROR] read action timeout/invalid at cycle=" << world().time().cycle();
            throw std::runtime_error(oss.str());
        }

        const auto m = getHybridActionMask();
        if (a < 0 || a > 3 || !m[a]) {
            std::ostringstream oss;
            oss << "[Hybrid][ERROR] action blocked/invalid a=" << a
                << " mask={" << int(m[0]) << "," << int(m[1]) << ","
                << int(m[2]) << "," << int(m[3]) << "}";
            throw std::runtime_error(oss.str());
        }

        if (!takeHybridAction(a, double(u0), double(u1))) {
            std::ostringstream oss;
            oss << "[Hybrid][ERROR] takeHybridAction failed a=" << a
                << " u0=" << u0 << " u1=" << u1
                << " kickable=" << world().self().isKickable()
                << " goalie=" << world().self().goalie();
            throw std::runtime_error(oss.str());
        }

        this->setNeckAction(new Neck_TurnToBallOrScan(0));

        // done => (1,1)
        shm_set_flags(flag_A, flag_B, 1, 1);
        cerr_flags("Hybrid:done");
        return;
    }

    // =========================
    // Non-PlayOn (Helios original)
    // =========================
    cerr_flags("NonPlayOn:begin");


    if (shm_ptr) {
        volatile uint8_t* flag_A = nullptr;
        volatile uint8_t* flag_B = nullptr;
        if (shm_flags(flag_A, flag_B)) {
            shm_set_flags(flag_A, flag_B, 0, 0);
            std::cerr << "[SHM][NonPlayOn] cyc=" << cyc << " gm=" << gm << " set flags (0,0)\n";
        }
    }

    if (this->audioSensor().trainerMessageTime() == world().time()) {
        // noop
    }

    Strategy::instance().update(world());
    FieldAnalyzer::instance().update(world());

    M_field_evaluator  = createFieldEvaluator();
    M_action_generator = createActionGenerator();
    ActionChainHolder::instance().setFieldEvaluator(M_field_evaluator);
    ActionChainHolder::instance().setActionGenerator(M_action_generator);

    if (doPreprocess()) {
        std::cerr << "[HELIOS][preprocess] cyc=" << cyc << " gm=" << gm << "\n";
        return;
    }

    ActionChainHolder::instance().update(world());

    SoccerRole::Ptr role_ptr =
        Strategy::i().createRole(world().self().unum(), world());

    if (!role_ptr) {
        M_client->setServerAlive(false);
        std::cerr << "No role assigned to player unum=" << world().self().unum() << std::endl;
        return;
    }

    if (role_ptr->acceptExecution(world())) {
        role_ptr->execute(this);
        return;
    }

    if (world().gameMode().type() == GameMode::PlayOn && mode_ == Mode::Helios) {
        role_ptr->execute(this);
        return;
    }

    if (world().gameMode().isPenaltyKickMode()) {
        Bhv_PenaltyKick().execute(this);
        return;
    }

    Bhv_SetPlay().execute(this);
}



// sample_player.cpp
int SamplePlayer::getActionFromSharedMemory()
{
    const int my_cycle = world().time().cycle(); //获取当前cycle

    if (!shm_ptr) {
        throw std::runtime_error("shm_ptr null");
    }

    auto* base        = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A = base + OFFSET_FLAG_A;    //动作储存标志
    volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
    auto* shm_cycle   = reinterpret_cast<int32_t*>(base + OFFSET_CYCLE); //当前帧号
    auto* shm_action  = reinterpret_cast<int32_t*>(base + OFFSET_ACTION); //动作
    

    // C++ 端等待 Python 写完动作（flag 清零），并且最多等 5000 毫秒（5 秒）。
    const auto start   = std::chrono::steady_clock::now();
    const auto TIMEOUT = std::chrono::milliseconds(36000000); // 10 小时，基本上不会超时
    int loop_cnt = 0;
    while (!(*flag_A == 1 && *flag_B == 0)) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ++loop_cnt;

        if (loop_cnt % 10000 == 0) {
        }
        if (std::chrono::steady_clock::now() - start > TIMEOUT) {
            std::ostringstream oss;
            oss << "wait action timeout at cycle=" << my_cycle
                << " flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << " shm_cycle=" << *shm_cycle;
            throw std::runtime_error(oss.str());  // ✅ 允许报错
        }
    }

    std::atomic_thread_fence(std::memory_order_acquire);

    const int act = *shm_action; //拿出动作并检查

    if (!(0 <= act && act < BASE_ACTION_NUM)) {
        std::ostringstream oss;
        oss << "invalid action value: " << act << " at cycle=" << my_cycle;
        throw std::runtime_error(oss.str());      // ✅ 允许报错
    }

    return act;
}


/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionStart()
{
    const int cyc = world().time().cycle();

    // 这里目前没逻辑，纯占位方便之后加东西

}

std::vector<float> SamplePlayer::GetState()
{
    const WorldModel& wm = this->world();
    std::vector<float> state;

    const int cyc = wm.time().cycle();

    // --- 1. 自身状态 ---
    state.push_back(wm.self().pos().x);        // 自己位置 x
    state.push_back(wm.self().pos().y);        // 自己位置 y
    state.push_back(wm.self().vel().x);        // 自己速度 x
    state.push_back(wm.self().vel().y);        // 自己速度 y
    state.push_back(wm.self().stamina());      // 自己体力
    state.push_back(wm.self().isKickable() ? 1.0f : 0.0f);  // 是否能踢球

    // --- 2. 球状态 ---
    state.push_back(wm.ball().pos().x);        // 球位置 x
    state.push_back(wm.ball().pos().y);        // 球位置 y
    state.push_back(wm.ball().vel().x);        // 球速度 x
    state.push_back(wm.ball().vel().y);        // 球速度 y

    // --- 3. 当前 GameMode 类型（作为 float 存）
    state.push_back(static_cast<float>(wm.gameMode().type()));



    return state;
}


/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionEnd()
{
    const int cyc = world().time().cycle();

    if ( world().self().posValid() )
    {
#if 0
        const ServerParam & SP = ServerParam::i();
        //
        // inside of pitch
        //

        // top,lower
        debugClient().addLine( Vector2D( world().ourOffenseLineX(),
                                         -SP.pitchHalfWidth() ),
                               Vector2D( world().ourOffenseLineX(),
                                         -SP.pitchHalfWidth() + 3.0 ) );
        // top,lower
        debugClient().addLine( Vector2D( world().ourDefenseLineX(),
                                         -SP.pitchHalfWidth() ),
                               Vector2D( world().ourDefenseLineX(),
                                         -SP.pitchHalfWidth() + 3.0 ) );

        // bottom,upper
        debugClient().addLine( Vector2D( world().theirOffenseLineX(),
                                         +SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().theirOffenseLineX(),
                                         +SP.pitchHalfWidth() ) );
        //
        debugClient().addLine( Vector2D( world().offsideLineX(),
                                         world().self().pos().y - 15.0 ),
                               Vector2D( world().offsideLineX(),
                                         world().self().pos().y + 15.0 ) );

        // outside of pitch

        // top,upper
        debugClient().addLine( Vector2D( world().ourOffensePlayerLineX(),
                                         -SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().ourOffensePlayerLineX(),
                                         -SP.pitchHalfWidth() ) );
        // top,upper
        debugClient().addLine( Vector2D( world().ourDefensePlayerLineX(),
                                         -SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().ourDefensePlayerLineX(),
                                         -SP.pitchHalfWidth() ) );
        // bottom,lower
        debugClient().addLine( Vector2D( world().theirOffensePlayerLineX(),
                                         +SP.pitchHalfWidth() ),
                               Vector2D( world().theirOffensePlayerLineX(),
                                         +SP.pitchHalfWidth() + 3.0 ) );
        // bottom,lower
        debugClient().addLine( Vector2D( world().theirDefensePlayerLineX(),
                                         +SP.pitchHalfWidth() ),
                               Vector2D( world().theirDefensePlayerLineX(),
                                         +SP.pitchHalfWidth() + 3.0 ) );
#else
        // top,lower
        debugClient().addLine( Vector2D( world().ourDefenseLineX(),
                                         world().self().pos().y - 2.0 ),
                               Vector2D( world().ourDefenseLineX(),
                                         world().self().pos().y + 2.0 ) );

        //
        debugClient().addLine( Vector2D( world().offsideLineX(),
                                         world().self().pos().y - 15.0 ),
                               Vector2D( world().offsideLineX(),
                                         world().self().pos().y + 15.0 ) );
#endif
    }

    //
    // ball position & velocity
    //
    dlog.addText( Logger::WORLD,
                  "WM: BALL pos=(%lf, %lf), vel=(%lf, %lf, r=%lf, ang=%lf)",
                  world().ball().pos().x,
                  world().ball().pos().y,
                  world().ball().vel().x,
                  world().ball().vel().y,
                  world().ball().vel().r(),
                  world().ball().vel().th().degree() );


    dlog.addText( Logger::WORLD,
                  "WM: SELF move=(%lf, %lf, r=%lf, th=%lf)",
                  world().self().lastMove().x,
                  world().self().lastMove().y,
                  world().self().lastMove().r(),
                  world().self().lastMove().th().degree() );

    if ( world().prevBall().rpos().isValid() )
    {
        Vector2D diff = world().ball().rpos() - world().prevBall().rpos();
        dlog.addText( Logger::WORLD,
                      "WM: BALL rpos=(%lf %lf) prev_rpos=(%lf %lf) diff=(%lf %lf)",
                      world().ball().rpos().x,
                      world().ball().rpos().y,
                      world().prevBall().rpos().x,
                      world().prevBall().rpos().y,
                      diff.x,
                      diff.y );

        Vector2D ball_move = diff + world().self().lastMove();
        Vector2D diff_vel = ball_move * ServerParam::i().ballDecay();
        dlog.addText( Logger::WORLD,
                      "---> ball_move=(%lf %lf) vel=(%lf, %lf, r=%lf, th=%lf)",
                      ball_move.x,
                      ball_move.y,
                      diff_vel.x,
                      diff_vel.y,
                      diff_vel.r(),
                      diff_vel.th().degree() );
    }

}


/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleInitMessage()
{

    {
        // Initializing the order of penalty kickers
        std::vector< int > unum_order_pk_kickers = { 10, 9, 2, 11, 3, 4, 1, 5, 6, 7, 8 };
        for (size_t i = 0; i < unum_order_pk_kickers.size(); ++i) {
        }

        M_worldmodel.setPenaltyKickTakerOrder( unum_order_pk_kickers );
    }

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleServerParam()
{

    if ( ServerParam::i().keepawayMode() )
    {
        M_communication = Communication::Ptr( new KeepawayCommunication() );
    }
    else
    {
    }

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerParam()
{

    if  ( KickTable::instance().createTables() )
    {
    }
    else
    {
        M_client->setServerAlive( false );
    }

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerType()
{

    // 目前没有逻辑，这里只是占位
    // 以后如果加类型相关初始化，可以在这里加 log

}

/*-------------------------------------------------------------------*/
/*!
  communication decision.
  virtual method in super class
*/
void
SamplePlayer::communicationImpl()
{

    if ( M_communication )
    {
        M_communication->execute( this );
    }
    else
    {
    }

}


/*-------------------------------------------------------------------*/
/*!
*/
bool
SamplePlayer::doPreprocess()
{
    const WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();

    GameMode::Type gm = wm.gameMode().type();
    bool frozen       = wm.self().isFrozen();
    bool self_pos_ok  = wm.self().posValid();
    int  ball_pos_cnt = wm.ball().posCount();
    int  ball_seen_cnt= wm.ball().seenPosCount();
    bool goalie       = wm.self().goalie();


    dlog.addText( Logger::TEAM,
                  __FILE__": (doPreProcess)" );

    //
    // freezed by tackle effect
    //
    if ( frozen )
    {

        dlog.addText( Logger::TEAM,
                      __FILE__": tackle wait. expires= %d",
                      wm.self().tackleExpires() );
        // face neck to ball
        this->setViewAction( new View_Tactical() );
        this->setNeckAction( new Neck_TurnToBallOrScan( 0 ) );

        return true;
    }

    //
    // BeforeKickOff or AfterGoal. jump to the initial position
    //
    if ( gm == GameMode::BeforeKickOff
         || gm == GameMode::AfterGoal_ )
    {

        dlog.addText( Logger::TEAM,
                      __FILE__": before_kick_off" );
        Vector2D move_point = Strategy::i().getPosition( wm.self().unum() );

        Bhv_CustomBeforeKickOff( move_point ).execute( this );

        this->setViewAction( new View_Tactical() );

        return true;
    }

    //
    // self localization error
    //
    if ( ! self_pos_ok )
    {

        dlog.addText( Logger::TEAM,
                      __FILE__": invalid my pos" );
        Bhv_Emergency().execute( this ); // includes change view

        return true;
    }

    //
    // ball localization error
    //
    const int count_thr = ( goalie ? 10 : 5 );
    bool ball_bad_pos =
        ( ball_pos_cnt > count_thr
          || ( gm != GameMode::PlayOn
               && ball_seen_cnt > count_thr + 10 ) );


    if ( ball_bad_pos )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": search ball" );

        this->setViewAction( new View_Tactical() );

        Bhv_NeckBodyToBall().execute( this );

        return true;
    }

    //
    // set default change view
    //
    this->setViewAction( new View_Tactical() );

    //
    // check shoot chance
    //
    if ( doShoot() )
    {
        return true;
    }

    //
    // check queued action
    //
    if ( this->doIntention() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": do queued intention" );
        return true;
    }

    //
    // check simultaneous kick
    //
    if ( doForceKick() )
    {
        return true;
    }

    //
    // check pass message
    //
    if ( doHeardPassReceive() )
    {
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doShoot()
{
    const WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();


    GameMode::Type gm = wm.gameMode().type();
    int stopped = wm.time().stopped();
    bool kickable = wm.self().isKickable();


    bool strict_ok = false;
    if ( gm != GameMode::IndFreeKick_
         && stopped == 0
         && kickable )
    {
        Bhv_StrictCheckShoot bhv;
        strict_ok = bhv.execute( this );
    }
    else
    {
    }

    if ( gm != GameMode::IndFreeKick_
         && stopped == 0
         && kickable
         && strict_ok )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": shooted" );

        // reset intention
        this->setIntention( static_cast< SoccerIntention * >( 0 ) );

        return true;
    }

    return false;
}

bool
SamplePlayer::isDoShootExecutable()
{
    const WorldModel & wm = this->world(); //获取当前世界状态
    const int cyc = wm.time().cycle();

    GameMode::Type gm = wm.gameMode().type();
    int stopped = wm.time().stopped();
    bool kickable = wm.self().isKickable();


    bool strict_ok = false;
    if ( gm != GameMode::IndFreeKick_
         && stopped == 0
         && kickable )
    {
        Bhv_StrictCheckShoot bhv;
        strict_ok = bhv.isExecutable( this );
    }
    else
    {
    }

    bool ok = ( gm != GameMode::IndFreeKick_
                && stopped == 0
                && kickable
                && strict_ok );

    return ok;
}

/*-------------------------------------------------------------------*/
/*! helper: 由方向编号算目标点 */

static rcsc::Vector2D computeTargetByDirection(const rcsc::PlayerAgent* agent, int dir_id)
{
    const rcsc::Vector2D current_pos = agent->world().self().pos();
    const double step_length = 3.0;


    static const rcsc::Vector2D direction_offsets[4] = {
        rcsc::Vector2D(0.0,  1.0),  // 上
        rcsc::Vector2D(0.0, -1.0),  // 下
        rcsc::Vector2D(-1.0, 0.0),  // 左
        rcsc::Vector2D(1.0,  0.0)   // 右
    };

    rcsc::Vector2D target = current_pos;

    if (dir_id >= 0 && dir_id <= 3)
    {
        const rcsc::Vector2D &offset = direction_offsets[dir_id];
        target = current_pos + offset * step_length;
    }
    else if (dir_id == 4)
    {
        target = current_pos;
    }
    else
    {
        target = current_pos;
    }

    return target;
}

bool
SamplePlayer::doMoveTo(int n) // n: 0=上, 1=下, 2=左, 3=右, 4=原地
{
    const WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();

    GameMode::Type gm = wm.gameMode().type();
    bool kickable = wm.self().isKickable();


    if ( gm == GameMode::PlayOn
         && !kickable )
    {
        const rcsc::Vector2D target_point = computeTargetByDirection(this, n);
        double dist_thr = 1.0;
        double dash_power = 50.0;


        bool ok = Body_GoToPoint( target_point, dist_thr, dash_power ).execute( this );

        if ( !ok )
        {
            Body_TurnToBall().execute( this );
        }

        this->setNeckAction( new Neck_ScanField() );

        return true;
    }

    return false;
}

bool
SamplePlayer::doForceKick()
{
    const WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();

    GameMode::Type gm = wm.gameMode().type();
    bool goalie = wm.self().goalie();
    bool kickable = wm.self().isKickable();
    const rcsc::PlayerObject *opp = wm.kickableOpponent();


    if ( gm == GameMode::PlayOn
         && !goalie
         && kickable
         && opp )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": simultaneous kick" );
        this->debugClient().addMessage( "SimultaneousKick" );

        Vector2D goal_pos( ServerParam::i().pitchHalfLength(), 0.0 );

        if ( wm.self().pos().x > 36.0
             && wm.self().pos().absY() > 10.0 )
        {
            goal_pos.x = 45.0;
            dlog.addText( Logger::TEAM,
                          __FILE__": simultaneous kick cross type" );
        }


        Body_KickOneStep( goal_pos,
                          ServerParam::i().ballSpeedMax()
                          ).execute( this );

        this->setNeckAction( new Neck_ScanField() );

        return true;
    }

    return false;
}

bool
SamplePlayer::isDoForceKickExecutable() //当你在比赛中 能踢球，且敌人也能踢球，且比赛是正常状态（PlayOn），就立即执行 Body_KickOneStep() 把球解围或射门，防止丢球。
{
    const WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();

    GameMode::Type gm = wm.gameMode().type();
    bool goalie = wm.self().goalie();
    bool kickable = wm.self().isKickable();
    const rcsc::PlayerObject *opp = wm.kickableOpponent();


    bool ok = ( gm == GameMode::PlayOn
                && !goalie
                && kickable );

    return ok;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doHeardPassReceive()
{
    const WorldModel & wm = this->world();
    const int cyc = wm.time().cycle();


    const auto & audio = wm.audioMemory();
    int pass_cycle   = audio.passTime().cycle();
    bool pass_empty  = audio.pass().empty();
    int receiver_id  = pass_empty ? -1 : audio.pass().front().receiver_;


    if ( audio.passTime() != wm.time()
         || pass_empty
         || receiver_id != wm.self().unum() )
    {
        return false;
    }

    int self_min = wm.interceptTable().selfStep();
    Vector2D intercept_pos = wm.ball().inertiaPoint( self_min );
    Vector2D heard_pos = audio.pass().front().receive_pos_;


    dlog.addText( Logger::TEAM,
                  __FILE__":  (doHeardPassReceive) heard_pos(%.2f %.2f) intercept_pos(%.2f %.2f)",
                  heard_pos.x, heard_pos.y,
                  intercept_pos.x, intercept_pos.y );

    bool has_kickable_tm = ( wm.kickableTeammate() != nullptr );
    bool ball_pos_ok = ( wm.ball().posCount() <= 1 );
    bool ball_vel_ok = ( wm.ball().velCount() <= 1 );
    bool step_ok     = ( self_min < 20 );


    if ( ! has_kickable_tm
         && ball_pos_ok
         && ball_vel_ok
         && step_ok
         //&& intercept_pos.dist( heard_pos ) < 3.0 ) //5.0 )
         )
    {

        dlog.addText( Logger::TEAM,
                      __FILE__": (doHeardPassReceive) intercept cycle=%d. intercept",
                      self_min );
        this->debugClient().addMessage( "Comm:Receive:Intercept" );

        Body_Intercept().execute( this );

        this->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {

        dlog.addText( Logger::TEAM,
                      __FILE__": (doHeardPassReceive) intercept cycle=%d. go to receive point",
                      self_min );
        this->debugClient().setTarget( heard_pos );
        this->debugClient().addMessage( "Comm:Receive:GoTo" );

        Body_GoToPoint( heard_pos,
                        0.5,
                        ServerParam::i().maxDashPower()
                        ).execute( this );

        this->setNeckAction( new Neck_TurnToBall() );
    }

    this->setIntention( new IntentionReceive( heard_pos,
                                              ServerParam::i().maxDashPower(),
                                              0.9,
                                              5,
                                              wm.time() ) );

    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::getFieldEvaluator() const
{
    const int cyc = world().time().cycle();
    return M_field_evaluator;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::createFieldEvaluator() const
{
    const int cyc = world().time().cycle();

    FieldEvaluator *raw = new SampleFieldEvaluator;


    FieldEvaluator::ConstPtr ptr( raw );


    return ptr;
}



/*-------------------------------------------------------------------*/
/*!
*/
#include "actgen_cross.h"
#include "actgen_direct_pass.h"
#include "actgen_self_pass.h"
#include "actgen_strict_check_pass.h"
#include "actgen_short_dribble.h"
#include "actgen_simple_dribble.h"
#include "actgen_shoot.h"
#include "actgen_action_chain_length_filter.h"

ActionGenerator::ConstPtr
SamplePlayer::createActionGenerator() const
{
    const int cyc = world().time().cycle();

    CompositeActionGenerator * g = new CompositeActionGenerator();

    int gen_idx = 0;

    //
    // shoot
    //
    g->addGenerator( new ActGen_RangeActionChainLengthFilter
                     ( new ActGen_Shoot(),
                       2, ActGen_RangeActionChainLengthFilter::MAX ) );
    ++gen_idx;

    //
    // strict check pass
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_StrictCheckPass(), 1 ) );
    ++gen_idx;

    //
    // cross
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_Cross(), 1 ) );
    ++gen_idx;

    //
    // direct pass (当前注释掉，仅打印说明未启用)
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_DirectPass(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // short dribble
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_ShortDribble(), 1 ) );
    ++gen_idx;

    //
    // self pass (long dribble)
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_SelfPass(), 1 ) );
    ++gen_idx;

    //
    // simple dribble (当前注释掉，仅打印说明未启用)
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_SimpleDribble(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    ActionGenerator::ConstPtr ptr(g);

    return ptr;
}


const std::array<bool, BASE_ACTION_NUM>& SamplePlayer::getActionMask() const {
    return action_mask;
}

std::vector<float> SamplePlayer::getAllState() const
{
    const WorldModel & wm = this->world();

    const int cyc = wm.time().cycle();

    std::vector<float> state;
    state.reserve(97);

    //---------------- 1. 自身 ----------------
    {
        const rcsc::Vector2D &sp  = wm.self().pos();
        const rcsc::Vector2D &sv  = wm.self().vel();
        const double          sta = wm.self().stamina();
        const bool            kik = wm.self().isKickable();


        state.push_back(static_cast<float>(sp.x));
        state.push_back(static_cast<float>(sp.y));
        state.push_back(static_cast<float>(sv.x));
        state.push_back(static_cast<float>(sv.y));
        state.push_back(static_cast<float>(sta));
        state.push_back(kik ? 1.f : 0.f);
    }

    //---------------- 2. 球 ------------------
    {
        const rcsc::Vector2D &bp = wm.ball().pos();
        const rcsc::Vector2D &bv = wm.ball().vel();


        state.push_back(static_cast<float>(bp.x));
        state.push_back(static_cast<float>(bp.y));
        state.push_back(static_cast<float>(bv.x));
        state.push_back(static_cast<float>(bv.y));
    }

    //---------------- 3. 对手 11 人 ----------
    {
        int opp_cnt = 0;

        for (const auto *opp_base : wm.theirPlayers()) {
            if (opp_cnt >= 11) break;

            const rcsc::PlayerObject *opp =
                dynamic_cast<const rcsc::PlayerObject *>(opp_base);

            if (opp && opp->posValid()) {
                const rcsc::Vector2D &op = opp->pos();
                const rcsc::Vector2D &ov = opp->vel();


                state.push_back(static_cast<float>(op.x));
                state.push_back(static_cast<float>(op.y));
                state.push_back(static_cast<float>(ov.x));
                state.push_back(static_cast<float>(ov.y));
            } else {

                state.insert(state.end(), 4, 0.f);
            }
            ++opp_cnt;
        }

        for (; opp_cnt < 11; ++opp_cnt) {
            state.insert(state.end(), 4, 0.f);
        }
    }

    //---------------- 4. 队友（除自己）10 人 --
    {
        int mate_cnt = 0;
        const int self_unum = wm.self().unum();

        for (const auto *mate_base : wm.ourPlayers()) {
            const rcsc::PlayerObject *mate =
                dynamic_cast<const rcsc::PlayerObject *>(mate_base);

            if (!mate) {
                continue;
            }
            if (mate->unum() == self_unum) {
                continue;
            }
            if (mate_cnt >= 10) {
                break;
            }

            if (mate->posValid()) {
                const rcsc::Vector2D &mp = mate->pos();
                const rcsc::Vector2D &mv = mate->vel();

                state.push_back(static_cast<float>(mp.x));
                state.push_back(static_cast<float>(mp.y));
                state.push_back(static_cast<float>(mv.x));
                state.push_back(static_cast<float>(mv.y));
            } else {

                state.insert(state.end(), 4, 0.f);
            }
            ++mate_cnt;
        }

        for (; mate_cnt < 10; ++mate_cnt) {
            state.insert(state.end(), 4, 0.f);
        }
    }

    //---------------- 5. 当前 GameMode / side / goalie -------
    {
        const int gm_type = static_cast<int>(wm.gameMode().type());
        const bool our_left = (wm.ourSide() == rcsc::LEFT);
        const bool is_goalie = wm.self().goalie();


        state.push_back(static_cast<float>(gm_type));
        state.push_back(our_left ? 0.0f : 1.0f);
        state.push_back(is_goalie ? 1.0f : 0.0f);
    }

    // -------- 完整性检查 ----------
    std::size_t sz = state.size();

    assert(state.size() == STATE_NUM && "getAllState(): length mismatch");


    return state;
}



void SamplePlayer::takeAction(int n) {
    const WorldModel & wm = this->world();
    Bhv_BasicMove        move_behavior;
    Body_HoldBall2008    hold_ball;
    Body_Pass            pass_action;
    Body_AdvanceBall2009 advance_ball_action;

    const int cyc = world().time().cycle();

    switch (n) {
        case 0: {
            move_behavior.doTackle(this);
            break;
        }

        case 1: {
            bool can_shoot = isDoShootExecutable();
            if (can_shoot) {
                doShoot();   // 严格射门，可行时优先
            } else {
                bool fk = doForceKick();  // 如果严格射门条件不满足，就执行强制射门
            }
            break;
        }

        case 2: {
            move_behavior.doIntercept(this);  // 内部会判断是否需要拦截
            break;
        }

        case 3: {
            advance_ball_action.execute(this);
            break;
        }

        // ------- 三类传球 -------
        case 4: { // Direct Pass
            Body_Pass pa;
            pa.DirectPass(this);
            break;
        }

        case 5: { // Lead Pass
            Body_Pass pa;
            pa.LeadPass(this);
            break;
        }

        case 6: { // Through Pass
            Body_Pass pa;
            pa.ThroughPass(this);
            break;
        }

        case 7: {
            hold_ball.execute(this);
            break;
        }

        case 8: {
            bool exec_ok = isDoCatchExecutable();
            doCatch();
            break;
        }

        case 9: {
            int dribble_dir = 0;  // 上
            rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
            Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
            dribble_action.execute(this);
            break;
        }

        case 10: {
            int dribble_dir = 1;  // 下
            rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
            Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
            dribble_action.execute(this);
            break;
        }

        case 11: {
            int dribble_dir = 2;  // 左
            rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
            Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
            dribble_action.execute(this);
            break;
        }

        case 12: {
            int dribble_dir = 3;  // 右
            rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
            Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
            dribble_action.execute(this);
            break;
        }

        case 13: {
            bool ok = doMoveTo(0);
            break;
        }

        case 14: {
            bool ok = doMoveTo(1);
            break;
        }

        case 15: {
            bool ok = doMoveTo(2);
            break;
        }

        case 16: {
            bool ok = doMoveTo(3);
            break;
        }

        default: {
            break;
        }
    }

}


/*-------------------------------------------------------------------*/
/*!
  Set the current action mask (for RL usage)
*/

void SamplePlayer::setActionMask() {
    const WorldModel & wm = this->world();
    Bhv_BasicMove      move_behavior;
    Body_HoldBall2008  hold_ball;
    Body_Pass          pass_action;
    Body_AdvanceBall2009 advance_ball_action;

    const int cyc = world().time().cycle();

    // 先全部清 0，避免残留
    for (int i = 0; i < BASE_ACTION_NUM; ++i) {
        action_mask[i] = false;
    }

    // 0 铲球
    action_mask[0] = move_behavior.isTackleExecutable(this);

    // 1 射门
    action_mask[1] = isDoShootExecutable();

    // 2 追球：只要自己不能踢球，就允许追球
    action_mask[2] = !wm.self().isKickable();

    // 3 解围
    action_mask[3] = advance_ball_action.isExecutable(this);

    // 4–6: 各种传球（暂时共用 isExecutable）
    action_mask[4] = pass_action.isExecutable(this);  // Direct Pass

    action_mask[5] = pass_action.isExecutable(this);  // Lead Pass

    action_mask[6] = pass_action.isExecutable(this);  // Through Pass

    // 7 控球 HoldBall
    action_mask[7] = hold_ball.isExecutable(this);

    // 8 Catch（门将接球）
    action_mask[8] = isDoCatchExecutable();

    // 9–12: 带球推进（是否能踢球决定）
    bool can_dribble = wm.self().isKickable();
    action_mask[9]  = can_dribble;
    action_mask[10] = can_dribble;
    action_mask[11] = can_dribble;
    action_mask[12] = can_dribble;

    // --- 无球移动动作（13–16）---
    const rcsc::ServerParam & SP  = rcsc::ServerParam::i();
    const rcsc::Vector2D      cur = wm.self().pos();
    const double step = 0.3;
    const double x_min = -SP.pitchHalfLength();
    const double x_max =  SP.pitchHalfLength();
    const double y_min = -SP.pitchHalfWidth();
    const double y_max =  SP.pitchHalfWidth();


    bool can_move_up    = (cur.y + step <= y_max);
    bool can_move_down  = (cur.y - step >= y_min);
    bool can_move_left  = (cur.x - step >= x_min);
    bool can_move_right = (cur.x + step <= x_max);


    // 如果当前能踢球（kickable），则禁止无球移动
    if (wm.self().isKickable()) {
        can_move_up = can_move_down = can_move_left = can_move_right = false;
    } else {
    }

    action_mask[13] = can_move_up;
    action_mask[14] = can_move_down;
    action_mask[15] = can_move_left;
    action_mask[16] = can_move_right;


    // 汇总打印前若干个 mask（方便对齐 Python 侧日志）
    for (int i = 0; i <= 16 && i < BASE_ACTION_NUM; ++i) {
    }

}


bool SamplePlayer::initSharedMemory() {


    if (SHM_NAME.empty()) {
        return false;
    }

    if (SHM_NAME[0] != '/') {
        SHM_NAME = "/" + SHM_NAME;
    }

    int shm_fd = -1;
    const int max_retry = 2000;
    for (int i = 0; i < max_retry; ++i) {
        shm_fd = shm_open(SHM_NAME.c_str(), O_RDWR, 0666);
        if (shm_fd != -1) {
            break;
        }
        if (errno != ENOENT) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (shm_fd == -1) {
        return false;
    }

    struct stat st;
    if (fstat(shm_fd, &st) == -1) {
        close(shm_fd);
        return false;
    }

    if ((size_t)st.st_size < SHM_SIZE) {
        close(shm_fd);
        return false;
    }

    void* p = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    close(shm_fd);

    if (p == MAP_FAILED) {
        shm_ptr = nullptr;
        return false;
    }
    shm_ptr = p;

    auto need = [&](size_t off, size_t bytes, const char* tag){
        if (off + bytes > SHM_SIZE) {
            return false;
        }
        return true;
    };

    bool ok = true;
    ok &= need(OFFSET_FLAG_A, 1, "FLAG_A");
    ok &= need(OFFSET_FLAG_B, 1, "FLAG_B");
    ok &= need(OFFSET_MASK, BASE_ACTION_NUM, "MASK");
    ok &= need(OFFSET_CYCLE, sizeof(int32_t), "CYCLE");
    ok &= need(OFFSET_ACTION, sizeof(int32_t), "ACTION");
    ok &= need(OFFSET_STATE, STATE_NUM * sizeof(float), "STATE");
    ok &= need(OFFSET_HYBRID_MASK, 4, "HYBRID_MASK");
    ok &= need(OFFSET_HYBRID_ACT, sizeof(int32_t), "HYBRID_ACT");
    ok &= need(OFFSET_HYBRID_U0, sizeof(float), "HYBRID_U0");
    ok &= need(OFFSET_HYBRID_U1, sizeof(float), "HYBRID_U1");


    if (!ok) {
        munmap(shm_ptr, SHM_SIZE);
        shm_ptr = nullptr;
        return false;
    }

    std::atomic_thread_fence(std::memory_order_acquire);
    return true;
}





bool SamplePlayer::isDoCatchExecutable() const {
    const rcsc::WorldModel & wm = this->world();
    return (!wm.self().isFrozen())
        && wm.self().goalie()
        && wm.self().isKickable()
        && inOurPenaltyArea()   // 🟢 必须在己方禁区
        && (wm.gameMode().type() == rcsc::GameMode::PlayOn
            || wm.gameMode().type() == rcsc::GameMode::PenaltyTaken_)
        && wm.ball().rposValid();
}

bool SamplePlayer::inOurPenaltyArea() const {

    const rcsc::WorldModel   &wm = this->world();

    const rcsc::ServerParam  &SP = rcsc::ServerParam::i();

    const rcsc::Vector2D     &p  = wm.self().pos();

    // 服务器参数（默认：pitchHalfLength=52.5, penaltyAreaLength=16.5, penaltyAreaWidth=40.32）
    const double x_goal_line   = SP.pitchHalfLength();
    const double pa_len        = SP.penaltyAreaLength();
    const double pa_half_width = SP.penaltyAreaWidth() * 0.5;

    // 给一点容差，避免贴线误差
    const double eps = 1e-6;

    double x_min, x_max;

    // 当前实现：总是用“左侧禁区”坐标
    x_min = -x_goal_line - eps;
    x_max = -x_goal_line + pa_len + eps;

    const double abs_y = std::fabs(p.y);
    const double y_limit = pa_half_width + eps;

    const bool cond_x_min = (p.x >= x_min);
    const bool cond_x_max = (p.x <= x_max);
    const bool cond_y     = (abs_y <= y_limit);


    const bool inside =
        (cond_x_min && cond_x_max && cond_y);

    return inside;
}
