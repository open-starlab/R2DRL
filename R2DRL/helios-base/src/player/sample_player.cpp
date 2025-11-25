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
      M_communication()
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
    std::cerr << "[SamplePlayer::initImpl] START" << std::endl;

    bool result = PlayerAgent::initImpl( cmd_parser );

    // read additional options
    result &= Strategy::instance().init( cmd_parser );

    // ✅ 定义 ParamMap，再 add 参数
    rcsc::ParamMap my_params( "Additional options" );
    my_params.add()
        ( "shm-name", "",       &SHM_NAME,   "shared memory name" )
        ( "mode",     "Helios", &RUN_MODE_,  "run mode: Base|Helios|Hybrid" );

    // ✅ 解析附加参数
    cmd_parser.parse( my_params );

    // ✅ 归一化 RUN_MODE_ → mode_
    std::string m = RUN_MODE_;
    std::transform( m.begin(), m.end(), m.begin(),
                    []( unsigned char c ){ return static_cast<char>( std::tolower( c ) ); } );

    if ( m == "base" )        mode_ = Mode::Base;
    else if ( m == "hybrid" ) mode_ = Mode::Hybrid;
    else                      mode_ = Mode::Helios;  // 默认兜底

    if ( cmd_parser.count( "help" ) > 0 )
    {
        my_params.printHelp( std::cerr );
        return false;
    }

    if ( cmd_parser.failed() )
    {
        std::cerr << "player: ***WARNING*** detected unsupported options: ";
        cmd_parser.print( std::cerr );
        std::cerr << std::endl;
    }

    if ( ! result )
    {
        std::cerr << "[SamplePlayer::initImpl] PlayerAgent::initImpl / Strategy::init failed" << std::endl;
        return false;
    }

    if ( ! Strategy::instance().read( config().configDir() ) )
    {
        std::cerr << "***ERROR*** Failed to read team strategy." << std::endl;
        return false;
    }

    if ( KickTable::instance().read( config().configDir() + "/kick-table" ) )
    {
        std::cerr << "Loaded the kick table: [" << config().configDir() << "/kick-table]" << std::endl;
    }

    // ======== 关键信息日志 ========
    std::cerr << "[SamplePlayer::initImpl] team=" << config().teamName()
              << " RUN_MODE_=" << RUN_MODE_
              << " normalized_mode=" << ( mode_ == Mode::Base ? "Base"
                                       : mode_ == Mode::Hybrid ? "Hybrid"
                                       : "Helios" )
              << " SHM_NAME=\"" << SHM_NAME << "\""
              << std::endl;

    // ======== 共享内存相关 ========
    if ( mode_ == Mode::Base || mode_ == Mode::Hybrid )
    {
        if ( SHM_NAME.empty() )
        {
            std::cerr << "[SamplePlayer::initImpl][ERROR] Mode=Base/Hybrid 但 SHM_NAME 为空！"
                      << " 这是 Python 那边没传 --shm-name 的典型情况。" << std::endl;
            return false;
        }

        std::cerr << "[SamplePlayer] 使用共享内存名: " << SHM_NAME << std::endl;

        if ( ! initSharedMemory() )
        {
            std::cerr << "***ERROR*** 无法初始化共享内存" << std::endl;
            return false;
        }

        std::cerr << "[SamplePlayer::initImpl] 初始化共享内存成功" << std::endl;
    }
    else
    {
        std::cerr << "[SamplePlayer::initImpl] Helios mode, 不使用共享内存 (忽略 SHM_NAME)" << std::endl;
    }

    std::cerr << "[SamplePlayer::initImpl] DONE" << std::endl;
    return true;
}


// ------- 小工具：算“球相对身体”的角度（度） -------
static inline double relDirToBallDeg(const rcsc::WorldModel & wm){
    rcsc::AngleDeg to_ball = (wm.ball().pos() - wm.self().pos()).th();
    return (to_ball - wm.self().body()).degree();  // 范围约在 [-180, 180]
}

// SamplePlayer 成员函数：0~3 对应 turn/dash/tackle/catch
// 返回 true 表示本帧已发送了1条身体指令；false 表示没发（例如 catch 条件不满足）

// 0:TURN, 1:DASH, 2:KICK, 3:CATCH
bool SamplePlayer::takeHybridAction(int a, double u0, double u1)
{
    const rcsc::WorldModel & wm = this->world();

    // ---- 仅在本函数内可见的归一化 → 物理量 映射 ----
    auto clamp01 = [](double x)->double {
        return std::max(0.0, std::min(1.0, x));
    };
    auto u_to_deg180 = [&](double u)->double {               // [-180, 180]°
        u = clamp01(u);
        return (u * 2.0 - 1.0) * 180.0;
    };
    auto u_to_dash = [&](double u)->double {                 // [-maxDash, +maxDash]
        const double maxDash = rcsc::ServerParam::i().maxDashPower(); // 通常 100
        u = clamp01(u);
        return (u * 2.0 - 1.0) * maxDash;
    };
    auto u_to_kick_power = [&](double u)->double {           // [-maxPower, +maxPower]
        const double maxP = rcsc::ServerParam::i().maxPower();         // 通常 100
        u = clamp01(u);
        return (u * 2.0 - 1.0) * maxP;
    };

    // ---- 只发“一条”身体指令（turn/dash/kick/catch/move 之一）----
    switch(a){
        case 0: { // TURN(moment°) ；u0∈[0,1] → [-180,180]°
            const double moment_deg = u_to_deg180(u0);
            this->doTurn(moment_deg);
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            return true;
        }
        case 1: { // DASH
            const double power = 1.0 + 99.0 * std::clamp(u0, 0.0, 1.0); // [1,100]
            this->doDash(power);                                        // ✅ 只给正功率
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            return true;
        }
        case 2: { // KICK(power,dir)；u0→力度, u1→方向（相对身体，度）
            if (!wm.self().isKickable()) return false;
            const double power   = u_to_kick_power(u0);
            const double dir_deg = u_to_deg180(u1);
            this->doKick(power, dir_deg);
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            return true;
        }
        case 3: { // CATCH(dir)（你当前工程为“无参 doCatch()”，先忽略方向）
            if (!wm.self().goalie()) return false;
            // 如果以后改成带参版本：this->doCatch( rcsc::AngleDeg(u_to_deg180(u0)) );
            this->doCatch();
            this->setNeckAction(new Neck_TurnToBallOrScan(0));
            return true;
        }
        default:
            return false;
    }
}

void SamplePlayer::writeHybridMaskToSharedMemory()
{
    if (!shm_ptr) {
            std::cerr << "[Hybrid][SHM][WARN] writeHybridMaskToSharedMemory(): shm_ptr null"
                  << " cycle=" << world().time().cycle()
                  << std::endl;
    return;
    }
    auto* base         = static_cast<uint8_t*>(shm_ptr);
    uint8_t* hy_mask   = base + OFFSET_HYBRID_MASK;

    const auto m = getHybridActionMask();
    for (int i = 0; i < 4; ++i) hy_mask[i] = static_cast<uint8_t>(m[i]);
    std::cerr << "[Hybrid][SHM][WRITE_MASK] cycle=" << world().time().cycle()
              << " mask={" << int(hy_mask[0]) << "," << int(hy_mask[1]) << ","
              << int(hy_mask[2]) << "," << int(hy_mask[3]) << "}"
              << " OFFSET_HYBRID_MASK=" << OFFSET_HYBRID_MASK
              << std::endl;
}

bool SamplePlayer::readHybridActionFromSharedMemory(int &a, float &u0, float &u1, int timeout_ms)
{
    if (!shm_ptr) {
        std::cerr << "[Hybrid][SHM][FATAL] readHybridActionFromSharedMemory(): shm_ptr null"
                  << " cycle=" << world().time().cycle()
                  << std::endl;
        throw std::runtime_error("shm_ptr null (hybrid)");
    }

    auto* base                 = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A   = base + OFFSET_FLAG_A;
    volatile uint8_t* flag_B   = base + OFFSET_FLAG_B;
    int32_t* shm_cycle         = reinterpret_cast<int32_t*>(base + OFFSET_CYCLE);
    int32_t* shm_hybrid_action = reinterpret_cast<int32_t*>(base + OFFSET_HYBRID_ACT);
    float*   shm_u0            = reinterpret_cast<float*>(base + OFFSET_HYBRID_U0);
    float*   shm_u1            = reinterpret_cast<float*>(base + OFFSET_HYBRID_U1);

    const int my_cycle = world().time().cycle();

    std::cerr << "[Hybrid][SHM][READ] enter. cycle=" << my_cycle
              << " flag_A=" << int(*flag_A)
              << " flag_B=" << int(*flag_B)
              << " shm_cycle=" << *shm_cycle
              << " OFFSET_FLAG_A=" << OFFSET_FLAG_A
              << " OFFSET_FLAG_B=" << OFFSET_FLAG_B
              << " OFFSET_HYBRID_ACT=" << OFFSET_HYBRID_ACT
              << " OFFSET_HYBRID_U0=" << OFFSET_HYBRID_U0
              << " OFFSET_HYBRID_U1=" << OFFSET_HYBRID_U1
              << std::endl;

    // 和 Base 的握手保持一致：等待 Python 写完 (A==1,B==0)
    const auto start = std::chrono::steady_clock::now();
    const auto TO    = std::chrono::milliseconds(timeout_ms);
    int loop_cnt = 0;

    while (!(*flag_A == 1 && *flag_B == 0)) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ++loop_cnt;
        if (loop_cnt % 10000 == 0) {
            std::cerr << "[Hybrid][SHM][WAIT] cycle=" << my_cycle
                      << " loop=" << loop_cnt
                      << " flag_A=" << int(*flag_A)
                      << " flag_B=" << int(*flag_B)
                      << " shm_cycle=" << *shm_cycle
                      << std::endl;
        }
        if (std::chrono::steady_clock::now() - start > TO) {
            std::ostringstream oss;
            oss << "[Hybrid] wait action timeout at cycle=" << my_cycle
                << " flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << " shm_cycle=" << *shm_cycle;
            std::cerr << oss.str() << std::endl;  
            std::cerr << "[Hybrid][SHM][TIMEOUT] " << oss.str() << std::endl;      
            dlog.addText(Logger::TEAM,"[Hybrid][ERROR] ... wait action timeout");
            return false; // 超时：让上层去兜底（随机/本地策略）
        }
    }

    std::atomic_thread_fence(std::memory_order_acquire);

    a  = *shm_hybrid_action;
    u0 = *shm_u0;
    u1 = *shm_u1;
    std::cerr << "[Hybrid][SHM][READ] done. cycle=" << my_cycle
              << " a=" << a << " u0=" << u0 << " u1=" << u1
              << " raw_flag_A=" << int(*flag_A)
              << " raw_flag_B=" << int(*flag_B)
              << " shm_cycle=" << *shm_cycle
              << std::endl;
    // 保护：a 合法、u0/u1 合法
    if (a < 0 || a > 3) {
        std::cerr << "[Hybrid][SHM][ERROR] invalid action a=" << a << std::endl;
        dlog.addText(Logger::TEAM,
             "[Hybrid][ERROR] ... invalid action");
        return false;
    }
    // 你当前实现假设 u0/u1 ∈ [0,1]，这里做截断
    auto clamp01 = [](float x){ return std::max(0.0f, std::min(1.0f, x)); };
    u0 = clamp01(u0);
    u1 = clamp01(u1);
    std::cerr << "[Hybrid][SHM][READ] clamped. a=" << a
              << " u0=" << u0 << " u1=" << u1 << std::endl;
    return true;
}

void SamplePlayer::writeSharedMemory()
{
    if (!shm_ptr) {
        std::cerr << "[FATAL] writeSharedMemory(): shm_ptr == nullptr! "
                  << "Shared memory not initialized or unmapped."
                  << std::endl;
        dlog.addText(Logger::TEAM,
            "[Hybrid][ERROR] ... invalid action");      
        throw std::runtime_error("shm_ptr is null — shared memory write aborted");
    }

    const int my_cycle = world().time().cycle(); 

    // 指针区
    auto* base = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A = base + OFFSET_FLAG_A;
    volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
    uint8_t*  shm_mask  = base + OFFSET_MASK;
    int32_t*  shm_cycle = reinterpret_cast<int32_t*>(base + OFFSET_CYCLE);
    float*    shm_state = reinterpret_cast<float*>(base + OFFSET_STATE);
    int32_t*  shm_action= reinterpret_cast<int32_t*>(base + OFFSET_ACTION);

    std::cerr << "[SHM][WRITE] cycle=" << my_cycle
              << " base=" << static_cast<void*>(base)
              << " flag_A=" << int(*flag_A)
              << " flag_B=" << int(*flag_B)
              << " OFFSET_MASK=" << OFFSET_MASK
              << " OFFSET_STATE=" << OFFSET_STATE
              << " OFFSET_ACTION=" << OFFSET_ACTION
              << std::endl;
    // 1) 写 mask
    const auto& mask = getActionMask();
    for (int i = 0; i < BASE_ACTION_NUM; ++i) {
        shm_mask[i] = static_cast<uint8_t>(mask[i] ? 1 : 0);
    }
    std::cerr << "[SHM][WRITE] mask[0..min(16,BASE_ACTION_NUM-1)] =";
    for (int i = 0; i < std::min(static_cast<int>(BASE_ACTION_NUM), 17); ++i) {
        std::cerr << " " << int(shm_mask[i]);
    }

    std::cerr << std::endl;
    // 2) 写 cycle
    const int32_t cycle = static_cast<int32_t>(world().time().cycle());
    *shm_cycle = cycle;
    std::cerr << "[SHM][WRITE] shm_cycle=" << *shm_cycle << std::endl;
    // 3) 写 obs
    const auto state = getAllState();
    for (size_t i = 0; i < state.size(); ++i) {
        shm_state[i] = state[i];
    }
    std::cerr << "[SHM][WRITE] state.size=" << state.size()
              << " first5=";
    for (int i = 0; i < 5 && i < (int)state.size(); ++i) {
        std::cerr << state[i] << " ";
    }
    std::cerr << std::endl;

}

// --- 放在合适的位置，例如与其它 mask/utility 函数放一起 ---

// 仅用于 Hybrid 四个底层动作：0:TURN, 1:DASH, 2:KICK, 3:CATCH
std::array<bool,4> SamplePlayer::getHybridActionMask() const
{
    const rcsc::WorldModel & wm = this->world();

    const bool frozen      = wm.self().isFrozen();
    const bool kickable    = wm.self().isKickable();
    const bool is_goalie   = wm.self().goalie();

    std::array<bool,4> m{};
    m[0] = true;                    // TURN: 总是允许
    m[1] = !frozen;                 // DASH: 冻结时不允许
    m[2] = (!frozen && kickable);   // KICK: 要能踢球且未冻结
    m[3] = (!frozen && is_goalie && kickable);  // CATCH: 门将 + 未冻结 + 球够近(用kickable近似)
    return m;
}

// 如果你仍然用全局的 action_mask(std::array<bool, BASE_ACTION_NUM>) 往共享内存写：
// 把前4位写为 Hybrid 掩码，其它清 False（避免 Python 侧误读）。
void SamplePlayer::setHybridActionMask()
{
    const auto m = getHybridActionMask();
    action_mask.fill(false);
    action_mask[0] = m[0]; // TURN
    action_mask[1] = m[1]; // DASH
    action_mask[2] = m[2]; // KICK
    action_mask[3] = m[3]; // CATCH
}



/*-------------------------------------------------------------------*/
/*!
  main decision
  virtual method in super class
// */
void
SamplePlayer::actionImpl()
{

    // ===========================================================
    // === 1️⃣ PlayOn 模式：Python 同步控制版本（你的逻辑） ===
    // ===========================================================

    if (world().gameMode().type() == GameMode::PlayOn && mode_ == Mode::Base)
    {
        const int my_cycle = world().time().cycle();
        std::cerr << "[Base][ACTION] PlayOn + Mode::Base, cycle=" << my_cycle << std::endl;

        if (!shm_ptr) {
            std::cerr << "[Base][ACTION][FATAL] shm_ptr null at cycle=" << my_cycle << std::endl;
            throw std::runtime_error("shm_ptr null in Base mode");
        }
        // === 写入 mask/state 到共享内存 ===
        setActionMask();
        writeSharedMemory();

        // === 同步信号交互 ===
        auto* base = static_cast<uint8_t*>(shm_ptr);
        volatile uint8_t* flag_A = base + OFFSET_FLAG_A;
        volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
        std::cerr << "[Base][ACTION] before signal C++->Py, "
                << "flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << std::endl;
        //  标志位01
        // 通知 Python: 观测已准备好
        std::atomic_thread_fence(std::memory_order_release);
        *flag_A = 0;
        *flag_B = 1;
        std::cerr << "[Base][ACTION] after set (A,B)=(0,1), "
                << "flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << std::endl;
        // 等待 Python 写入cerr动作并清零 flag
        const int act = getActionFromSharedMemory();
        std::cerr << "[Base][ACTION] got act=" << act << " from shm at cycle=" << my_cycle << std::endl;
        // 执行动作
        takeAction(act);
        //  标志位11
        // 通知 Python: 观测已准备好
        *flag_A = 1;
        *flag_B = 1; 
        // 确保动作执行完毕后同步
        std::atomic_thread_fence(std::memory_order_release);
        std::cerr << "[Base][ACTION] after takeAction, set (A,B)=(1,1), "
                << "flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << std::endl;
        return; // ✅ PlayOn 模式逻辑结束
    }

    // === 1.5️⃣ Hybrid 模式：PlayOn 时由 Python 选择 a,u0,u1；失败则随机兜底 ===
    // === 1.5️⃣ Hybrid 模式：严格执行传入动作；任何异常直接报错 ===
    if (world().gameMode().type() == GameMode::PlayOn && mode_ == Mode::Hybrid)
    {
        const int my_cycle = world().time().cycle();
        std::cerr << "[Hybrid][ACTION] PlayOn + Mode::Hybrid, cycle=" << my_cycle << std::endl;

        if (!shm_ptr) {
            std::cerr << "[Hybrid][ACTION][FATAL] shm_ptr null at cycle=" << my_cycle << std::endl;
            throw std::runtime_error("shm_ptr null in Hybrid mode");
        }
        // 1) 写 mask/state 到共享内存（包含 17 位全量 mask 与 4 位 hybrid mask）
        setHybridActionMask();
        writeSharedMemory();
        writeHybridMaskToSharedMemory();

        auto* base = static_cast<uint8_t*>(shm_ptr);
        volatile uint8_t* flag_A = base + OFFSET_FLAG_A;
        volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
        std::cerr << "[Hybrid][ACTION] before signal C++->Py, "
                << "flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << std::endl;
        // C++->Py: 观测就绪
        std::atomic_thread_fence(std::memory_order_release);
        *flag_A = 0;
        *flag_B = 1;
        std::cerr << "[Hybrid][ACTION] after set (A,B)=(0,1), "
                << "flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << std::endl;
        // 2) 读取 Python 写回的 Hybrid 动作 & 参数（严格模式）
        int   a   = -1;
        float u0  = 0.5f;
        float u1  = 0.5f;

        // 没读到动作 → 直接抛错（不做任何兜底）
        if (!readHybridActionFromSharedMemory(a, u0, u1)) {
            std::ostringstream oss;
            oss << "[Hybrid][ERROR] read action timeout/invalid at cycle="
                << world().time().cycle();
            std::cerr << "[Hybrid][ACTION][ERROR] " << oss.str() << std::endl;
            throw std::runtime_error(oss.str());
        }
        std::cerr << "[Hybrid][ACTION] got a=" << a << " u0=" << u0 << " u1=" << u1
                << " at cycle=" << my_cycle << std::endl;
        // 3) 掩码检查：被屏蔽的动作一律报错
        const auto m = getHybridActionMask();
        if (a < 0 || a > 3 || !m[a]) {
            std::ostringstream oss;
            oss << "[Hybrid][ERROR] action blocked or invalid. a=" << a
                << " mask={" << int(m[0]) << "," << int(m[1]) << ","
                << int(m[2]) << "," << int(m[3]) << "}";
            std::cerr << "[Hybrid][ACTION][ERROR] " << oss.str() << std::endl;
            throw std::runtime_error(oss.str());
        }

        // 4) 真正执行；返回 false = 当前时刻条件不满足（如不可踢球/非门将等）→ 直接报错
        if (!takeHybridAction(a, double(u0), double(u1))) {
            std::ostringstream oss;
            oss << "[Hybrid][ERROR] takeHybridAction failed. a=" << a
                << " u0=" << u0 << " u1=" << u1
                << " kickable=" << world().self().isKickable()
                << " goalie=" << world().self().goalie();
            std::cerr << "[Hybrid][ACTION][ERROR] " << oss.str() << std::endl;
            throw std::runtime_error(oss.str());
        }

        this->setNeckAction(new Neck_TurnToBallOrScan(0));

        // 5) 只有成功执行时，才置 done：(A,B)=(1,1)
        *flag_A = 1;
        *flag_B = 1;
        std::atomic_thread_fence(std::memory_order_release);
        std::cerr << "[Hybrid][ACTION] success, set (A,B)=(1,1), "
                << "flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << " cycle=" << my_cycle
                << std::endl;
        return;
    }

    // ===========================================================
    // === 2️⃣ 非 PlayOn 模式：完全复原原版 HELIOS 行为系统 ===
    // ===========================================================

    if (shm_ptr) {
    auto* base = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A = base + OFFSET_FLAG_A;
    volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
    std::cerr << "[ACTION][NonPlayOn] reset flags to (0,0). "
              << "old flag_A=" << int(*flag_A)
              << " old_flag_B=" << int(*flag_B)
              << " cycle=" << world().time().cycle()
              << std::endl;

    *flag_A = 0;
    *flag_B = 0;
    std::atomic_thread_fence(std::memory_order_release);

    // Trainer message（原样保留，可无视）
    if (this->audioSensor().trainerMessageTime() == world().time())
    {
        std::cerr << world().ourTeamName() << ' ' << world().self().unum()
                  << ' ' << world().time()
                  << " receive trainer message["
                  << this->audioSensor().trainerMessage() << ']'
                  << std::endl;
    }

    // 更新全局策略与场地分析
    Strategy::instance().update(world());
    FieldAnalyzer::instance().update(world());

    // 准备行动链
    M_field_evaluator = createFieldEvaluator();
    M_action_generator = createActionGenerator();
    ActionChainHolder::instance().setFieldEvaluator(M_field_evaluator);
    ActionChainHolder::instance().setActionGenerator(M_action_generator);

    // 特殊优先行为（铲球、意图等）
    if (doPreprocess())
    {
        dlog.addText(Logger::TEAM, __FILE__": preprocess done");
        return;
    }

    // 更新行动链
    ActionChainHolder::instance().update(world());

    // 创建当前角色
    SoccerRole::Ptr role_ptr =
        Strategy::i().createRole(world().self().unum(), world());
    // std::cerr << "createRole)" << std::endl;
    if (!role_ptr)
    {
        std::cerr << config().teamName() << ": "
                  << world().self().unum()
                  << " Error. Role is not registered.\nExit ..." << std::endl;
        M_client->setServerAlive(false);
        return;
    }

    // 若角色接受执行，则直接执行（某些模式下会 override）
    if (role_ptr->acceptExecution(world()))
    {
        role_ptr->execute(this);
        // std::cerr << "role_ptr->acceptExecution(world()" << std::endl;
        return;
    }

    if ( world().gameMode().type() == GameMode::PlayOn && mode_ == Mode::Helios)
    {
        role_ptr->execute( this );
        // std::cerr << "play on :role_ptr->execute( this )" << std::endl;
        return;
    }

    // PlayOn 之外模式分支
    if (world().gameMode().isPenaltyKickMode())
    {
        dlog.addText(Logger::TEAM, __FILE__": penalty kick");
        Bhv_PenaltyKick().execute(this);
        return;
    }

    // 其他 SetPlay（KickOff, Corner, FreeKick 等）
    Bhv_SetPlay().execute(this);
    //check
    }
}

// sample_player.cpp
int SamplePlayer::getActionFromSharedMemory()
{
    const int my_cycle = world().time().cycle(); //获取当前cycle
    if (!shm_ptr) {
        std::cerr << "[Base][SHM][FATAL] getActionFromSharedMemory(): shm_ptr null"
            << " cycle=" << my_cycle << std::endl;
        throw std::runtime_error("shm_ptr null");
    }

    auto* base        = static_cast<uint8_t*>(shm_ptr);
    volatile uint8_t* flag_A = base + OFFSET_FLAG_A;    //动作储存标志
    volatile uint8_t* flag_B = base + OFFSET_FLAG_B;
    auto* shm_cycle   = reinterpret_cast<int32_t*>(base + OFFSET_CYCLE); //当前帧号
    auto* shm_action  = reinterpret_cast<int32_t*>(base + OFFSET_ACTION); //动作
    

    std::cerr << "[Base][SHM][READ] enter. cycle=" << my_cycle
              << " flag_A=" << int(*flag_A)
              << " flag_B=" << int(*flag_B)
              << " shm_cycle=" << *shm_cycle
              << " OFFSET_FLAG_A=" << OFFSET_FLAG_A
              << " OFFSET_FLAG_B=" << OFFSET_FLAG_B
              << " OFFSET_ACTION=" << OFFSET_ACTION
              << std::endl;
    // C++ 端等待 Python 写完动作（flag 清零），并且最多等 5000 毫秒（5 秒）。
    const auto start   = std::chrono::steady_clock::now();
    const auto TIMEOUT = std::chrono::milliseconds(50000);
    int loop_cnt = 0;
    while (!(*flag_A == 1 && *flag_B == 0)) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ++loop_cnt;

        if (loop_cnt % 10000 == 0) {
            std::cerr << "[Base][SHM][WAIT] cycle=" << my_cycle
                      << " loop=" << loop_cnt
                      << " flag_A=" << int(*flag_A)
                      << " flag_B=" << int(*flag_B)
                      << " shm_cycle=" << *shm_cycle
                      << std::endl;
        }
        if (std::chrono::steady_clock::now() - start > TIMEOUT) {
            std::ostringstream oss;
            oss << "wait action timeout at cycle=" << my_cycle
                << " flag_A=" << int(*flag_A)
                << " flag_B=" << int(*flag_B)
                << " shm_cycle=" << *shm_cycle;
            std::cerr << "[Base][SHM][TIMEOUT] " << oss.str() << std::endl;
            throw std::runtime_error(oss.str());  // ✅ 允许报错
        }
    }

    std::atomic_thread_fence(std::memory_order_acquire);

    const int act = *shm_action; //拿出动作并检查
    std::cerr << "[Base][SHM][READ] done. cycle=" << my_cycle
              << " act=" << act
              << " flag_A=" << int(*flag_A)
              << " flag_B=" << int(*flag_B)
              << " shm_cycle=" << *shm_cycle
              << std::endl;
    if (!(0 <= act && act < BASE_ACTION_NUM)) {
        std::ostringstream oss;
        oss << "invalid action value: " << act << " at cycle=" << my_cycle;
        std::cerr << "[Base][SHM][ERROR] " << oss.str() << std::endl;
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

}

std::vector<float> SamplePlayer::GetState()
{
    const WorldModel& wm = this->world();
    std::vector<float> state;

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
        // std::cerr << "set Keepaway mode communication." << std::endl;
        M_communication = Communication::Ptr( new KeepawayCommunication() );
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
        // std::cerr << world().teamName() << ' '
        //           << world().self().unum() << ": "
        //           << " KickTable created."
        //           << std::endl;
        // std::cerr << << std::endl;
    }
    else
    {
        // std::cerr << world().teamName() << ' '
        //           << world().self().unum() << ": "
        //           << " KickTable failed..."
        //           << std::endl;
        M_client->setServerAlive( false );
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerType()
{

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
}

/*-------------------------------------------------------------------*/
/*!
*/
bool
SamplePlayer::doPreprocess()
{
    // check tackle expires
    // check self position accuracy
    // ball search
    // check queued intention
    // check simultaneous kick

    const WorldModel & wm = this->world();

    dlog.addText( Logger::TEAM,
                  __FILE__": (doPreProcess)" );

    //
    // freezed by tackle effect
    //
    if ( wm.self().isFrozen() )
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
    if ( wm.gameMode().type() == GameMode::BeforeKickOff
         || wm.gameMode().type() == GameMode::AfterGoal_ )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": before_kick_off" );
        Vector2D move_point =  Strategy::i().getPosition( wm.self().unum() );
        Bhv_CustomBeforeKickOff( move_point ).execute( this );
        this->setViewAction( new View_Tactical() );
        return true;
    }

    //
    // self localization error
    //
    if ( ! wm.self().posValid() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": invalid my pos" );
        Bhv_Emergency().execute( this ); // includes change view
        return true;
    }

    //
    // ball localization error
    //
    const int count_thr = ( wm.self().goalie()
                            ? 10
                            : 5 );
    if ( wm.ball().posCount() > count_thr
         || ( wm.gameMode().type() != GameMode::PlayOn
              && wm.ball().seenPosCount() > count_thr + 10 ) )
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

    if ( wm.gameMode().type() != GameMode::IndFreeKick_
         && wm.time().stopped() == 0
         && wm.self().isKickable()
         && Bhv_StrictCheckShoot().execute( this ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": shooted" );

        // reset intention
        this->setIntention( static_cast< SoccerIntention * >( 0 ) );
        // std::cerr << "shot!" << std::endl;  // 新增：简单标记一次射门
        return true;
    }

    return false;
}

bool
SamplePlayer::isDoShootExecutable()
{
    const WorldModel & wm = this->world(); //获取当前世界状态

    if ( wm.gameMode().type() != GameMode::IndFreeKick_ // 不是间接任意球
         && wm.time().stopped() == 0 // 比赛没有暂停
         && wm.self().isKickable()// 球员能踢球
         && Bhv_StrictCheckShoot().isExecutable( this ) )// 射门行为判断为可行->射门
    {
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!


*/

static rcsc::Vector2D computeTargetByDirection(const rcsc::PlayerAgent* agent, int dir_id)
{
    const rcsc::Vector2D current_pos = agent->world().self().pos();
    const double step_length = 3.0;

    static const rcsc::Vector2D direction_offsets[4] = {
        rcsc::Vector2D(0.0, 1.0),   // 上
        rcsc::Vector2D(0.0, -1.0),  // 下
        rcsc::Vector2D(-1.0, 0.0),  // 左
        rcsc::Vector2D(1.0, 0.0)    // 右
    };

    if (dir_id >= 0 && dir_id <= 3)
    {
        return current_pos + direction_offsets[dir_id] * step_length;
    }
    else if (dir_id == 4)
    {
        return current_pos;
    }
    else
    {
        return current_pos;
    }
}

bool
SamplePlayer::doMoveTo(int n) // n: 0=上, 1=下, 2=左, 3=右, 4=原地
{
    const WorldModel & wm = this->world();

    if ( wm.gameMode().type() == GameMode::PlayOn
         && !wm.self().isKickable() )
    {
        const rcsc::Vector2D target_point = computeTargetByDirection(this, n);

        double dist_thr = 1;
        double dash_power = 50.0;

        if ( !Body_GoToPoint( target_point, dist_thr, dash_power ).execute( this ) )
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

    if ( wm.gameMode().type() == GameMode::PlayOn
         && ! wm.self().goalie()
         && wm.self().isKickable()
         && wm.kickableOpponent() )
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
SamplePlayer::isDoForceKickExecutable()//当你在比赛中 能踢球，且敌人也能踢球，且比赛是正常状态（PlayOn），就立即执行 Body_KickOneStep() 把球解围或射门，防止丢球。
{
    const WorldModel & wm = this->world();
    if ( wm.gameMode().type() == GameMode::PlayOn// 比赛进行中
         && ! wm.self().goalie()// 自己不是守门员
         && wm.self().isKickable())
    {
        return true;
    }
    return false;
}
/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doHeardPassReceive()
{
    const WorldModel & wm = this->world();

    if ( wm.audioMemory().passTime() != wm.time()
         || wm.audioMemory().pass().empty()
         || wm.audioMemory().pass().front().receiver_ != wm.self().unum() )
    {

        return false;
    }

    int self_min = wm.interceptTable().selfStep();
    Vector2D intercept_pos = wm.ball().inertiaPoint( self_min );
    Vector2D heard_pos = wm.audioMemory().pass().front().receive_pos_;

    dlog.addText( Logger::TEAM,
                  __FILE__":  (doHeardPassReceive) heard_pos(%.2f %.2f) intercept_pos(%.2f %.2f)",
                  heard_pos.x, heard_pos.y,
                  intercept_pos.x, intercept_pos.y );

    if ( ! wm.kickableTeammate()
         && wm.ball().posCount() <= 1
         && wm.ball().velCount() <= 1
         && self_min < 20
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
    return M_field_evaluator;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::createFieldEvaluator() const
{
    return FieldEvaluator::ConstPtr( new SampleFieldEvaluator );
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
    CompositeActionGenerator * g = new CompositeActionGenerator();

    //
    // shoot
    //
    g->addGenerator( new ActGen_RangeActionChainLengthFilter
                     ( new ActGen_Shoot(),
                       2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // strict check pass
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_StrictCheckPass(), 1 ) );

    //
    // cross
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_Cross(), 1 ) );

    //
    // direct pass
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_DirectPass(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // short dribble
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_ShortDribble(), 1 ) );

    //
    // self pass (long dribble)
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_SelfPass(), 1 ) );

    //
    // simple dribble
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_SimpleDribble(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    return ActionGenerator::ConstPtr( g );
}

const std::array<bool, BASE_ACTION_NUM>& SamplePlayer::getActionMask() const {
    return action_mask;
}
std::vector<float> SamplePlayer::getAllState() const
{
    const WorldModel & wm = this->world();
    std::vector<float> state;
    state.reserve(97);                              

    //---------------- 1. 自身 ----------------
    state.push_back(static_cast<float>(wm.self().pos().x));
    state.push_back(static_cast<float>(wm.self().pos().y));
    state.push_back(static_cast<float>(wm.self().vel().x));
    state.push_back(static_cast<float>(wm.self().vel().y));
    state.push_back(static_cast<float>(wm.self().stamina()));
    state.push_back(wm.self().isKickable() ? 1.f : 0.f);

    //---------------- 2. 球 ------------------
    state.push_back(static_cast<float>(wm.ball().pos().x));
    state.push_back(static_cast<float>(wm.ball().pos().y));
    state.push_back(static_cast<float>(wm.ball().vel().x));
    state.push_back(static_cast<float>(wm.ball().vel().y));

    //---------------- 3. 对手 11 人 ----------
    int opp_cnt = 0;
    for (const auto *opp_base : wm.theirPlayers()) {
        if (opp_cnt >= 11) break;
        const rcsc::PlayerObject *opp =
            dynamic_cast<const rcsc::PlayerObject *>(opp_base);

        if (opp && opp->posValid()) {
            state.push_back(static_cast<float>(opp->pos().x));
            state.push_back(static_cast<float>(opp->pos().y));
            state.push_back(static_cast<float>(opp->vel().x));
            state.push_back(static_cast<float>(opp->vel().y));
        } else {
            state.insert(state.end(), 4, 0.f);
        }
        ++opp_cnt;
    }
    for (; opp_cnt < 11; ++opp_cnt)   // 补零
        state.insert(state.end(), 4, 0.f);

    //---------------- 4. 队友（除自己）10 人 --
    int mate_cnt = 0;
    for (const auto *mate_base : wm.ourPlayers()) {
        const rcsc::PlayerObject *mate =
            dynamic_cast<const rcsc::PlayerObject *>(mate_base);

        if (!mate || mate->unum() == wm.self().unum()) continue;
        if (mate_cnt >= 10) break;

        if (mate->posValid()) {
            state.push_back(static_cast<float>(mate->pos().x));
            state.push_back(static_cast<float>(mate->pos().y));
            state.push_back(static_cast<float>(mate->vel().x));
            state.push_back(static_cast<float>(mate->vel().y));
        } else {
            state.insert(state.end(), 4, 0.f);
        }
        ++mate_cnt;
    }
    for (; mate_cnt < 10; ++mate_cnt)  // 补零
        state.insert(state.end(), 4, 0.f);

    //---------------- 5. 当前 GameMode -------
    state.push_back(static_cast<float>(wm.gameMode().type()));
    //left,right
    state.push_back( wm.ourSide() == rcsc::LEFT ? 0.0f : 1.0f );
    //goal keeper
    state.push_back( wm.self().goalie() ? 1.0f : 0.0f );
    // -------- 完整性检查 ----------
    assert(state.size() == STATE_NUM && "getAllState(): length mismatch");
    return state;
}


void SamplePlayer::takeAction(int n) {
    const WorldModel & wm = this->world();
    Bhv_BasicMove move_behavior;
    Body_HoldBall2008 hold_ball;
    Body_Pass pass_action;
    Body_AdvanceBall2009 advance_ball_action;
    // std::cerr << "[DEBUG] takeAction(" << n << ") "
    //       << "mode=" << (int)world().gameMode().type()
    //       << " kickable=" << world().self().isKickable()
    //       << " pos=(" << world().self().pos().x << "," << world().self().pos().y << ")"
    //       << std::endl;

    switch (n) {
        case 0:
            // 铲球
            move_behavior.doTackle(this);
            break;
        case 1:
            // 射门
            if (isDoShootExecutable()) {
                doShoot();   // 严格射门，可行时优先
            } else {
                doForceKick();  // 如果严格射门条件不满足，就执行强制射门
            }
            break;
        case 2:
            // 拦截
            move_behavior.doIntercept(this);  // 内部会判断是否需要拦截
            break;
        case 3:
            // run to ball
            advance_ball_action.execute(this);
            break;
        // ------- 三类传球 -------
        case 4: { // Direct Pass
            Body_Pass pass_action;
            pass_action.DirectPass(this);   // TODO: 你实现的 direct 路线执行接口
        } break;

        case 5: { // Lead Pass
            Body_Pass pass_action;
            pass_action.LeadPass(this);     // TODO
        } break;

        case 6: { // Through Pass
            Body_Pass pass_action;
            pass_action.ThroughPass(this);  // TODO
        } break;
        case 7:
            hold_ball.execute(this);
            break;
        case 8:
            doCatch();
            break;
        case 9:
            {   
                int dribble_dir = 0;  // 默认向上，之后你可以改成由 Python 传来
                rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
                Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
                dribble_action.execute(this);
                break;
            }
        case 10:
            {   
                int dribble_dir = 1;  // 默认向上，之后你可以改成由 Python 传来
                rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
                Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
                dribble_action.execute(this);
                break;
            }
        case 11:
            {   
                int dribble_dir = 2;  // 默认向上，之后你可以改成由 Python 传来
                rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
                Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
                dribble_action.execute(this);
                break;
            }
        case 12:
            {   
                int dribble_dir = 3;  // 默认向上，之后你可以改成由 Python 传来
                rcsc::Vector2D target = computeTargetByDirection(this, dribble_dir);
                Body_Dribble2008 dribble_action(target, 0.5, 50.0, 2, true);
                dribble_action.execute(this);
                break;
            }
        case 13:
            // 向左移动
            doMoveTo(0);
            break;
        case 14:
            // 向右移动
            doMoveTo(1);
            break;
        case 15:
            // 向右移动
            doMoveTo(2);
            break;
        case 16:
            // 向右移动
            doMoveTo(3);
            break;
        default:
            std::cerr << "[takeAction] Unknown action index: " << n << std::endl;
            break;
    }
}

/*-------------------------------------------------------------------*/
/*!
  Set the current action mask (for RL usage)
*/

void SamplePlayer::setActionMask() {
    const WorldModel & wm = this->world();
    Bhv_BasicMove move_behavior;
    Body_HoldBall2008 hold_ball;
    Body_Pass pass_action;
    Body_AdvanceBall2009 advance_ball_action;
    // 0 铲球
    action_mask[0] = move_behavior.isTackleExecutable(this);
    // 1 射门
    action_mask[1] = isDoShootExecutable();
    // 2 追球
    action_mask[2] = !wm.self().isKickable();
    // 3 解围
    action_mask[3] = advance_ball_action.isExecutable(this);
    // 4 传球
    action_mask[4] = pass_action.isExecutable(this);          // Direct Pass   // TODO: isDirectPassExecutable()
    action_mask[5] = pass_action.isExecutable(this);          // Lead Pass     // TODO: isLeadPassExecutable()
    action_mask[6] = pass_action.isExecutable(this);          // Through Pass  // TODO: isThroughPassExecutable()

    // 5 控球
    action_mask[7] = hold_ball.isExecutable(this);
    // 6.Catch
    action_mask[8] = isDoCatchExecutable();
    // 6~9: 带球推进 // 默认向上，之后你可以改成由 Python 传来
    action_mask[9] = wm.self().isKickable();
    action_mask[10] = wm.self().isKickable();
    action_mask[11] = wm.self().isKickable();
    action_mask[12] = wm.self().isKickable();
    // 10–13: 四方向移动（由 doMoveTo(n) 设定）
    // --- 无球移动动作 ---
    const rcsc::ServerParam & SP = rcsc::ServerParam::i();
    const rcsc::Vector2D cur = wm.self().pos();
    const double step = 0.3;
    const double x_min = -SP.pitchHalfLength();
    const double x_max =  SP.pitchHalfLength();
    const double y_min = -SP.pitchHalfWidth();
    const double y_max =  SP.pitchHalfWidth();

    bool can_move_up    = (cur.y + step <= y_max);
    bool can_move_down  = (cur.y - step >= y_min);
    bool can_move_left  = (cur.x - step >= x_min);
    bool can_move_right = (cur.x + step <= x_max);

    // ✅ 如果当前能踢球（kickable），则禁止无球移动
    if (wm.self().isKickable()) {
        can_move_up = can_move_down = can_move_left = can_move_right = false;
    }

    action_mask[13] = can_move_up;
    action_mask[14] = can_move_down;
    action_mask[15] = can_move_left;
    action_mask[16] = can_move_right;
}


bool SamplePlayer::initSharedMemory() {
    std::cerr << "[SHM] initSharedMemory() called. "
              << "SHM_NAME=" << SHM_NAME
              << " SHM_SIZE=" << SHM_SIZE
              << std::endl;
    int shm_fd = shm_open(SHM_NAME.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "[SHM][ERROR] shm_open failed. name=" << SHM_NAME
                  << " errno=" << errno << " (" << strerror(errno) << ")"
                  << std::endl;
        perror("[SamplePlayer] shm_open 失败");
        return false;
    }
    std::cerr << "[SHM] shm_open success. fd=" << shm_fd << std::endl;
    // 设置共享内存大小
    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        std::cerr << "[SHM][ERROR] ftruncate failed. size=" << SHM_SIZE
                  << " errno=" << errno << " (" << strerror(errno) << ")"
                  << std::endl;        
        perror("[SamplePlayer] ftruncate 失败");
        close(shm_fd);
        return false;
    }
    std::cerr << "[SHM] ftruncate success." << std::endl;
    shm_ptr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        std::cerr << "[SHM][ERROR] mmap failed. errno=" << errno
                  << " (" << strerror(errno) << ")"
                  << std::endl;
        perror("[SamplePlayer] mmap 失败");
        shm_ptr = nullptr;
        return false;
    }
    std::cerr << "[SHM] mmap success. shm_ptr=" << shm_ptr << std::endl;
    // ✅ 初始化内存（清零）
    std::memset(shm_ptr, 0, SHM_SIZE);

    std::cerr << "[SamplePlayer] 共享内存连接成功, flag 初始化为 0" << std::endl;
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
    // if (wm.ourSide() == rcsc::LEFT) {
    //     // 左队禁区: [-L, -L+len]
    //     x_min = -x_goal_line - eps;
    //     x_max = -x_goal_line + pa_len + eps;
    // } else {
    //     // 右队禁区: [ +L-len, +L ]
    //     x_min =  x_goal_line - pa_len - eps;
    //     x_max =  x_goal_line + eps;
    // }

    x_min = -x_goal_line - eps;
    x_max = -x_goal_line + pa_len + eps;
    return (p.x >= x_min && p.x <= x_max
            && std::fabs(p.y) <= pa_half_width + eps);
}
