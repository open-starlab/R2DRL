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
#include "config.h"
#endif

#include "sample_coach.h"

#include "sample_freeform_message.h"

#include <rcsc/coach/coach_command.h>
#include <rcsc/coach/coach_config.h>
#include <rcsc/coach/coach_debug_client.h>
#include <rcsc/common/abstract_client.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/player_type.h>
#include <rcsc/common/audio_memory.h>
#include <rcsc/common/say_message_parser.h>
#include <rcsc/param/param_map.h>
#include <rcsc/param/cmd_line_parser.h>

#include <rcsc/coach/coach_world_model.h>

#include <cstdio>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <functional>

#include "team_logo.xpm"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#include <iostream>
#include <iomanip>
#include <sys/stat.h>   // 
#include <sys/types.h>  // 
#include <cstdint>
#include <cmath>
using namespace rcsc;


struct RealSpeedMaxCmp
{
    bool operator()( const PlayerType* lhs,
                     const PlayerType* rhs ) const
    {

        if (!lhs || !rhs)
        {
            return false;
        }

        double lhs_speed = lhs->realSpeedMax();
        double rhs_speed = rhs->realSpeedMax();
        int lhs_cycle = lhs->cyclesToReachMaxSpeed();
        int rhs_cycle = rhs->cyclesToReachMaxSpeed();


        double diff = std::fabs(lhs_speed - rhs_speed);

        if (diff < 0.005)
        {
            bool res = lhs_cycle < rhs_cycle;
            return res;
        }

        bool res = lhs_speed > rhs_speed;

        return res;
    }
};


/*-------------------------------------------------------------------*/
/*!

*/
/*-------------------------------------------------------------------*/
/*!

*/
SampleCoach::SampleCoach()
    : CoachAgent()
{

    std::shared_ptr< AudioMemory > audio_memory( new AudioMemory );

    M_worldmodel.setAudioMemory( audio_memory );


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


    for ( int i = 0; i < 11; ++i )
    {
        M_opponent_player_types[i] = Hetero_Default;
    }

    // 初始化 shm 字段（如果你有这些成员）
    shm_fd_  = -1;
    shm_ptr_ = nullptr;

}

/*-------------------------------------------------------------------*/
/*!

*/
SampleCoach::~SampleCoach()
{

    // 如果你想在这里解除映射和关闭 shm
    if ( shm_ptr_ )
    {
        munmap(shm_ptr_, SHM_SIZE);
        shm_ptr_ = nullptr;
    }
    else
    {
    }

    if ( shm_fd_ != -1 )
    {
        close(shm_fd_);
        shm_fd_ = -1;
    }
    else
    {
    }

}


/*-------------------------------------------------------------------*/
/*!

 */

bool SampleCoach::initSharedMemory()
{

    // attach-only: Python side must create + ftruncate + zero_fill
    // coach side must NOT O_CREAT, must NOT ftruncate, must NOT memset
    shm_fd_ = -1;
    shm_ptr_ = nullptr;

    if (shm_name_.empty())
    {
    }

    // wait for shm to appear (coach may start before python creates it)
    constexpr int kRetry = 400;        // 400 * 10ms = 4s
    constexpr int kSleepUs = 10 * 1000;


    for (int i = 0; i < kRetry; ++i)
    {
        if (!shm_name_.empty() && shm_name_[0] != '/') {
            shm_name_ = "/" + shm_name_;
        }
        shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);

        if (shm_fd_ != -1)
        {
            break;
        }

        usleep(kSleepUs);
    }

    if (shm_fd_ == -1)
    {
        perror("shm_open(attach-only)");
        return false;
    }

    // (optional but recommended) verify size matches SHM_SIZE
    struct stat st;
    if (fstat(shm_fd_, &st) == -1)
    {
        perror("fstat(shm_fd)");
        close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }


    if (static_cast<size_t>(st.st_size) != static_cast<size_t>(SHM_SIZE))
    {
        close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_ptr_ = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED)
    {
        perror("mmap");
        shm_ptr_ = nullptr;
        close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    return true;
}

bool SampleCoach::initImpl( rcsc::CmdLineParser & cmd_parser )
{

    bool result = CoachAgent::initImpl( cmd_parser );

    // 共享内存选项解析
    rcsc::ParamMap my_params( "SampleCoach Options" );
    my_params.add()
        ( "shm-name", "", &shm_name_, "shared memory name (default: /coach_global_state)" );

    cmd_parser.parse( my_params );

    if ( cmd_parser.count( "help" ) > 0 )
    {
        my_params.printHelp( std::cout );
        return false;
    }

    if ( cmd_parser.failed() )
    {
        // 原注释保留
    }
    else
    {
    }

    if ( ! result )
    {
        return false;
    }

    //////////////////////////////////////////////////////////////////
    // 共享内存初始化
    //////////////////////////////////////////////////////////////////

    if (!initSharedMemory())
    {
        return false;
    }



    //////////////////////////////////////////////////////////////////
    // 其他初始化
    //////////////////////////////////////////////////////////////////

    if ( config().useTeamGraphic() )
    {
        if ( config().teamGraphicFile().empty() )
        {
            // M_team_graphic.readXpmFile( team_logo_xpm );
        }
        else
        {
            M_team_graphic.readXpmFile( config().teamGraphicFile().c_str() );
        }
    }

    // 让 C++ iostream 更快且不缓冲
    std::ios::sync_with_stdio(false);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);


    return true;
}


#include <iomanip>  // 👈 加上这一行
#include <iomanip>  // 确保有这一行

void SampleCoach::writeSharedMemory()
{
    if (!shm_ptr_) {
        return;
    }

    auto* base = static_cast<std::uint8_t*>(shm_ptr_);

    // 布局保持不变：
    // flag   : +0 (uint8)
    // cycle  : +1 (int32)   -> 注意这里是“非对齐地址”，必须 memcpy
    // floats : +5 (136 * float)
    // mode   : + (1 + 4 + 136*4) = +549 (int32) -> 同样可能非对齐，必须 memcpy

    constexpr std::size_t OFF_FLAG  = 0;
    constexpr std::size_t OFF_CYCLE = 1;
    constexpr std::size_t OFF_FLOAT = 5;
    constexpr std::size_t NFLOATS   = 136;
    constexpr std::size_t OFF_MODE  = 1 + 4 + NFLOATS * 4;

    auto wr_u8 = [&](std::size_t off, std::uint8_t v) {
        std::memcpy(base + off, &v, sizeof(v));
    };
    auto wr_i32 = [&](std::size_t off, std::int32_t v) {
        std::memcpy(base + off, &v, sizeof(v));
    };
    auto wr_f32 = [&](std::size_t off, float v) {
        std::memcpy(base + off, &v, sizeof(v));
    };

    // 1) cycle
    const std::int32_t cur_cycle = static_cast<std::int32_t>(world().time().cycle());
    wr_i32(OFF_CYCLE, cur_cycle);

    // 2) 写 float 区：从 +5 开始，顺序保持和你原来完全一致
    std::size_t k = 0; // float index [0, NFLOATS)

    auto push_f = [&](float v) {
        if (k < NFLOATS) {
            wr_f32(OFF_FLOAT + k * sizeof(float), v);
            ++k;
        }
        // 若 k 超了，说明 NFLOATS/布局定义不匹配；这里选择静默忽略，避免越界写
    };

    // ===== 球 =====
    push_f(static_cast<float>(world().ball().pos().x));
    push_f(static_cast<float>(world().ball().pos().y));
    push_f(static_cast<float>(world().ball().vel().x));
    push_f(static_cast<float>(world().ball().vel().y));

    // ===== 球员 =====
    int n_team = static_cast<int>(world().teammates().size());
    int n_opp  = static_cast<int>(world().opponents().size());

    auto dump_player = [&](const rcsc::CoachPlayerObject* p, int team) {
        const float x   = p ? static_cast<float>(p->pos().x)         : 0.f;
        const float y   = p ? static_cast<float>(p->pos().y)         : 0.f;
        const float vx  = p ? static_cast<float>(p->vel().x)         : 0.f;
        const float vy  = p ? static_cast<float>(p->vel().y)         : 0.f;
        const float dir = p ? static_cast<float>(p->body().degree()) : 0.f;

        push_f(x);
        push_f(y);
        push_f(vx);
        push_f(vy);
        push_f(dir);
        push_f(static_cast<float>(team));
    };

    // 我方：最多 11
    for (const auto& t : world().teammates()) {
        dump_player(t, 0);
        if (--n_team <= 0) break;
    }
    // 不足补 0 到 11 人
    for (int i = static_cast<int>(world().teammates().size()); i < 11; ++i) {
        dump_player(nullptr, 0);
    }

    // 对方：最多 11
    for (const auto& o : world().opponents()) {
        dump_player(o, 1);
        if (--n_opp <= 0) break;
    }
    // 不足补 0 到 11 人
    for (int i = static_cast<int>(world().opponents().size()); i < 11; ++i) {
        dump_player(nullptr, 1);
    }

    // 如果还没写满 136 floats（例如未来字段调整），剩余填 0，避免 Python 读到旧数据
    while (k < NFLOATS) {
        push_f(0.f);
    }

    // 3) mode (GameMode type) @ +549
    const std::int32_t gm_type = static_cast<std::int32_t>(world().gameMode().type());
    wr_i32(OFF_MODE, gm_type);

    // 4) flag = 1 通知 Python
    wr_u8(OFF_FLAG, static_cast<std::uint8_t>(1));
}

/*-------------------------------------------------------------------*/
/*!

*/

void
SampleCoach::actionImpl()
{

    long cyc_from_world = world().time().cycle();

    debugClient().addMessage( "Cycle=%ld", world().time().cycle() );

    const long cyc = world().time().cycle();

    // 原来的 teamGraphic/substitute/sayPlayerTypes 都先关掉了
    // 如果之后要开，也可以继续在里面打 log
    // if ( world().time().cycle() == 0
    //      && config().useTeamGraphic()
    //      && M_team_graphic.tiles().size() != teamGraphicOKSet().size() )
    // {
    //     sendTeamGraphic();
    // }

    // doSubstitute();
    // sayPlayerTypes();

    writeSharedMemory();

}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::handleInitMessage()
{
    // nothing yet
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::handleServerParam()
{
    // nothing yet
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::handlePlayerParam()
{
    // nothing yet
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::handlePlayerType()
{
    // nothing yet
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::doSubstitute()
{

    static bool S_first_substituted = false;

    int cyc = world().time().cycle();
    double stopped = world().time().stopped();

    if ( ! S_first_substituted
         && cyc == 0
         && stopped > 10 )
    {

        doFirstSubstitute();

        S_first_substituted = true;

        return;
    }

    GameMode::Type gm_type = world().gameMode().type();
    bool is_penalty = world().gameMode().isPenaltyKickMode();


    if ( cyc > 0
         && gm_type != GameMode::PlayOn
         && ! is_penalty )
    {

        doSubstituteTiredPlayers();

        return;
    }

}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::doFirstSubstitute()
{

    PlayerTypePtrCont candidates;

    std::fprintf( stderr,
                  "[COACH][doFirstSubstitute] id speed step inc  power  stam  karea\n" );

    int total_ptypes = PlayerParam::i().playerTypes();

    for ( int id = 0; id < total_ptypes; ++id )
    {

        const PlayerType * param = PlayerTypeSet::i().get( id );

        if ( ! param )
        {
            continue;
        }

        bool is_default = ( id == Hetero_Default );
        bool allow_multi_default = PlayerParam::i().allowMultDefaultType();


        if ( is_default
             && allow_multi_default )
        {
            int pt_num = MAX_PLAYER + 1;

            for ( int i = 0; i <= MAX_PLAYER; ++i )
            {
                candidates.push_back( param );
            }

        }

        int pt_max = PlayerParam::i().ptMax();

        for ( int i = 0; i < pt_max; ++i )
        {
            candidates.push_back( param );
        }


        std::fprintf( stderr,
                      "[COACH][doFirstSubstitute] %d %.3f  %2d  %.1f %5.1f %5.1f  %.3f\n",
                      id,
                      param->realSpeedMax(),
                      param->cyclesToReachMaxSpeed(),
                      param->staminaIncMax(),
                      param->getDashPowerToKeepMaxSpeed(),
                      param->getOneStepStaminaComsumption(),
                      param->kickableArea()
                      );

    }


    std::vector< int > ordered_unum;
    ordered_unum.reserve( 11 );

    // wing player has priority
    ordered_unum.push_back( 11 ); // center forward
    ordered_unum.push_back( 2 );  // center back
    ordered_unum.push_back( 3 );  // center back
    ordered_unum.push_back( 10 ); // side half
    ordered_unum.push_back( 9 );  // side half
    ordered_unum.push_back( 6 );  // center half
    ordered_unum.push_back( 4 );  // side back
    ordered_unum.push_back( 5 );  // side back
    ordered_unum.push_back( 7 );  // defensive half
    ordered_unum.push_back( 8 );  // defensive half

    for ( size_t i = 0; i < ordered_unum.size(); ++i )
    {
    }

    //
    // goalie:
    //
    double ver = config().version();

    if ( ver >= 14.0 )
    {
        substituteTo( 1, Hetero_Default ); // goalie
    }
    else
    {
    }

    {
        PlayerTypePtrCont::iterator it = candidates.begin();
        for ( ; it != candidates.end(); ++it )
        {
            if ( (*it) && (*it)->id() == Hetero_Default )
            {
                break;
            }
        }

        if ( it != candidates.end() )
        {
            candidates.erase( it );
        }
        else
        {
        }
    }

    //
    // change field players
    //

    for ( std::vector< int >::iterator unum = ordered_unum.begin();
          unum != ordered_unum.end();
          ++unum )
    {

        const CoachPlayerObject * p = world().teammate( *unum );
        if ( ! p )
        {

            dlog.addText( Logger::TEAM,
                          __FILE__": teammate %d does not exist. skip first substitution.",
                          *unum );
            continue;
        }

        int type = getFastestType( candidates );

        if ( type != Hetero_Unknown )
        {
            substituteTo( *unum, type );
        }
        else
        {
        }

    }

}


/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::doSubstituteTiredPlayers()
{

    int substitute_count = world().ourSubstituteCount();

    int subs_max = PlayerParam::i().subsMax();

    if ( substitute_count >= subs_max )
    {
        return;
    }

    const ServerParam & SP = ServerParam::i();

    //
    // check game time
    //
    const int half_time = SP.actualHalfTime();

    const int normal_time = half_time * SP.nrNormalHalfs();

    const int cur_cycle = world().time().cycle();

    if ( cur_cycle < normal_time - 500
         //|| world().time().cycle() <= half_time + 1
         //|| world().gameMode().type() == GameMode::KickOff_
         )
    {
        return;
    }


    //
    // create candidate teammate
    //
    std::vector< int > tired_teammate_unum;

    for ( CoachPlayerObject::Cont::const_iterator t = world().teammates().begin(),
              end = world().teammates().end();
          t != end;
          ++t )
    {

        if ( ! (*t) )
        {
            continue;
        }

        int unum = (*t)->unum();
        double rec = (*t)->recovery();
        double threshold = ServerParam::i().recoverInit() - 0.002;


        if ( rec < threshold )
        {
            tired_teammate_unum.push_back( unum );
        }
        else
        {
        }

    }

    if ( ! tired_teammate_unum.empty() )
    {
        for ( size_t i = 0; i < tired_teammate_unum.size(); ++i )
        {
        }
    }

    if ( tired_teammate_unum.empty() )
    {
        return;
    }

    //
    // create candidate player type
    //
    PlayerTypePtrCont candidates;

    for ( std::vector< int >::const_iterator
              id = world().availablePlayerTypeId().begin(),
              end = world().availablePlayerTypeId().end();
          id != end;
          ++id )
    {

        const PlayerType * param = PlayerTypeSet::i().get( *id );
        if ( ! param )
        {
            continue;
        }

        candidates.push_back( param );

    }


    //
    // try substitution
    //

    for ( std::vector< int >::iterator unum = tired_teammate_unum.begin();
          unum != tired_teammate_unum.end();
          ++unum )
    {

        int type = getFastestType( candidates );

        if ( type != Hetero_Unknown )
        {
            substituteTo( *unum, type );

            substitute_count++;

            if ( substitute_count >= subs_max )
            {
                break;
            }
        }
        else
        {
        }

    }

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SampleCoach::substituteTo( const int unum,
                           const int type )
{

    int cur_cycle = world().time().cycle();
    int subs_max = PlayerParam::i().subsMax();
    int cur_sub_count = world().ourSubstituteCount();


    if ( cur_cycle > 0
         && cur_sub_count >= subs_max )
    {
        return;
    }

    std::vector< int >::const_iterator
        it = std::find( world().availablePlayerTypeId().begin(),
                        world().availablePlayerTypeId().end(),
                        type );
    bool found = ( it != world().availablePlayerTypeId().end() );

    if ( ! found )
    {
        return;
    }

    doChangePlayerType( unum, type );

}

/*-------------------------------------------------------------------*/
/*!

 */
int
SampleCoach::getFastestType( PlayerTypePtrCont & candidates )
{

    if ( candidates.empty() )
    {
        return Hetero_Unknown;
    }

    std::sort( candidates.begin(),
               candidates.end(),
               RealSpeedMaxCmp() );

    PlayerTypePtrCont::iterator best_type = candidates.end();
    double max_speed = 0.0;
    int min_cycle = 100;


    for ( PlayerTypePtrCont::iterator it = candidates.begin();
          it != candidates.end();
          ++it )
    {
        if ( ! (*it) )
        {
            continue;
        }

        double speed = (*it)->realSpeedMax();
        int cycle_to_max = (*it)->cyclesToReachMaxSpeed();
        double stamina_one_step = (*it)->getOneStepStaminaComsumption();
        int id = (*it)->id();


        if ( speed < max_speed - 0.01 )
        {
            break;
        }

        if ( cycle_to_max < min_cycle )
        {

            best_type = it;
            max_speed = speed;
            min_cycle = cycle_to_max;


            continue;
        }

        if ( cycle_to_max == min_cycle )
        {

            if ( best_type == candidates.end() )
            {
                best_type = it;
                max_speed = speed;
                continue;
            }

            double best_stamina_one_step = (*best_type)->getOneStepStaminaComsumption();
            int best_id = (*best_type)->id();


            if ( stamina_one_step < best_stamina_one_step )
            {
                best_type = it;
                max_speed = speed;

            }
            else
            {
            }
        }
    }


    if ( best_type != candidates.end() && (*best_type) )
    {
        int id = (*best_type)->id();

        candidates.erase( best_type );

        return id;
    }

    return Hetero_Unknown;
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::sayPlayerTypes()
{

    static GameTime s_last_send_time( 0, 0 );

    if ( ! config().useFreeform() )
    {
        return;
    }

    if ( ! world().canSendFreeform() )
    {
        return;
    }

    int analyzed_count = 0;

    for ( int unum = 1; unum <= 11; ++unum )
    {

        const int id = world().theirPlayerTypeId( unum );


        if ( id != M_opponent_player_types[unum - 1] )
        {

            M_opponent_player_types[unum - 1] = id;


            if ( id != Hetero_Unknown )
            {
                ++analyzed_count;
            }
            else
            {
            }
        }
        else
        {
        }

    }

    if ( analyzed_count == 0 )
    {
        return;
    }


    std::shared_ptr< FreeformMessage > ptr( new OpponentPlayerTypeMessage( M_opponent_player_types[0],
                                                                           M_opponent_player_types[1],
                                                                           M_opponent_player_types[2],
                                                                           M_opponent_player_types[3],
                                                                           M_opponent_player_types[4],
                                                                           M_opponent_player_types[5],
                                                                           M_opponent_player_types[6],
                                                                           M_opponent_player_types[7],
                                                                           M_opponent_player_types[8],
                                                                           M_opponent_player_types[9],
                                                                           M_opponent_player_types[10] ) );

    this->addFreeformMessage( ptr );

    s_last_send_time = world().time();

    std::string msg;
    msg.reserve( 128 );

    msg = "(player_types ";

    for ( int unum = 1; unum <= 11; ++unum )
    {
        char buf[8];

        snprintf( buf, 8, "(%d %d)",
                  unum, M_opponent_player_types[unum - 1] );

        msg += buf;

    }

    msg += ')';

    // 原来这里是 cout 的注释，改成 cerr 输出

}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleCoach::sendTeamGraphic()
{

    int count = 0;


    for ( TeamGraphic::Map::const_reverse_iterator tile = M_team_graphic.tiles().rbegin();
          tile != M_team_graphic.tiles().rend();
          ++tile )
    {

        bool already_ok = ( teamGraphicOKSet().find( tile->first ) != teamGraphicOKSet().end() );

        if ( teamGraphicOKSet().find( tile->first ) == teamGraphicOKSet().end() )
        {

            bool ok = doTeamGraphic( tile->first.first,
                                     tile->first.second,
                                     M_team_graphic );

            if ( ! ok )
            {
                break;
            }

            ++count;
        }
        else
        {
        }

    }

}