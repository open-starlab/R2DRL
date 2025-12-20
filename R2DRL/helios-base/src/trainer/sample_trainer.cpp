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

#include "sample_trainer.h"

#include <rcsc/trainer/trainer_command.h>
#include <rcsc/trainer/trainer_config.h>
#include <rcsc/coach/coach_world_model.h>
#include <rcsc/common/abstract_client.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/player_type.h>
#include <rcsc/common/server_param.h>
#include <rcsc/param/param_map.h>
#include <rcsc/param/cmd_line_parser.h>
#include <rcsc/random.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdlib>   // getenv
#include <cstring>   // memset
#include <iostream>  // cerr
#include <cstdint>
#include <utility>
#include <atomic>
#include <chrono>
#include <thread>

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
SampleTrainer::SampleTrainer()
    : TrainerAgent()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
SampleTrainer::~SampleTrainer()
{
    close_shm_();
}

/*-------------------------------------------------------------------*/
/*!

 */
bool
SampleTrainer::initImpl( CmdLineParser & cmd_parser )
{
    bool result = TrainerAgent::initImpl( cmd_parser );

#if 0
    ParamMap my_params;

    std::string formation_conf;
    my_map.add()
        ( &conf_path, "fconf" )
        ;

    cmd_parser.parse( my_params );
#endif

    if ( cmd_parser.failed() )
    {
        std::cerr << "coach: ***WARNING*** detected unsupported options: ";
        cmd_parser.print( std::cerr );
        std::cerr << std::endl;
    }

    if ( ! result )
    {
        return false;
    }

    //////////////////////////////////////////////////////////////////
    // Add your code here.
    //////////////////////////////////////////////////////////////////

    // === attach / create trainer shm ===
    shm_ready_ = init_shm_();
    if (!shm_ready_) {
        std::cerr << "[trainer] WARN: SHM init failed; IPC disabled." << std::endl;
    }

    return true;
}
bool SampleTrainer::init_shm_()
{
    // 名字从 Python 传进来：在 Python 里设置环境变量
    // os.environ["RCSC_TRAINER_SHM"] = shm_name
    const char* name = std::getenv(SHM_ENV_NAME);
    if (!name || !*name) {
        std::cerr << "[trainer] RCSC_TRAINER_SHM not set." << std::endl;
        return false;
    }
    shm_name_ = name;

    // ✅ 只 attach，不创建、不删除
    //    Python 负责 shm_open(..., O_CREAT|O_EXCL|O_RDWR) + ftruncate + 初始化
    shm_fd_ = ::shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[trainer] shm_open attach");
        return false;
    }

    // 不再 ftruncate，避免改坏 Python 那边的大小
    void* p = ::mmap(nullptr,
                     TRAINER_SHM_SIZE,          // 要和 Python 约定好相同的大小
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     shm_fd_,
                     0);
    if (p == MAP_FAILED) {
        std::perror("[trainer] mmap");
        ::close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    shm_      = static_cast<std::uint8_t*>(p);
    shm_size_ = TRAINER_SHM_SIZE;

    std::cerr << "[trainer] shm attached: " << shm_name_
              << " size=" << shm_size_ << std::endl;

    const std::uint8_t A = rd8_(T_FLAG_A);
    const std::uint8_t B = rd8_(T_FLAG_B);

    
    return (shm_ready_ = true);
}


void SampleTrainer::close_shm_()
{
    if (shm_) {
        ::munmap(shm_, shm_size_);
        shm_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    // 是否 unlink:
    // 这里可以不 unlink，让 Python 侧去 unlink；就算 unlink 了，已 attach 的映射也仍可用。
    // 如果你想主动清理，可加：
    // if (!shm_name_.empty()) ::shm_unlink(shm_name_.c_str());
}

bool SampleTrainer::try_handle_trainer_ipc_()
{
    if (!shm_ready_ || !shm_) return false;

    const std::uint8_t A = rd8_(T_FLAG_A);
    const std::uint8_t B = rd8_(T_FLAG_B);
    std::cerr << "[trainer][IPC] check flags: A=" << int(A)
              << " B=" << int(B) << std::endl;

    // ✅ 只在 10 时处理：Python 提交请求
    if (A == 1 && B == 0) {
        const std::int32_t opcode = rd32_(T_OPCODE);
        std::cerr << "[trainer][IPC] got request: opcode=" << opcode << std::endl;

        // ✅ ACK：立刻置 B=1，进入 11（C++ 已接单/处理中）
        wr8_(T_FLAG_B, 1);

        exec_opcode_(opcode);

        // ✅ 完成：回到 01（ready/done）
        wr8_(T_FLAG_A, 0);   // 清 A（表示处理结束）
        // B 保持 1
        return true;
    }
    return false;
}



void SampleTrainer::exec_opcode_(std::int32_t opcode)
{
    switch (opcode) {
    case 1: // KickOff_Left
        doChangeMode( PM_KickOff_Left );
        break;
    case 2: // KickOff_Right
        doChangeMode( PM_KickOff_Right );
        break;
    case 4: // NEW: PlayOn
        doChangeMode( PM_PlayOn );
        break;
    case 5: // ⭐ 球 + 所有球员 全场随机站位
    {
        const double half_len = ServerParam::i().pitchHalfLength();
        const double half_wid = ServerParam::i().pitchHalfWidth();

        // 全场范围 [-L, L] x [-W, W]
        UniformReal uni_x(-half_len, half_len);
        UniformReal uni_y(-half_wid,  half_wid);

        // 先拿到左右两队队名（注意有可能还没 set）
        const std::string & left_name  = world().teamNameLeft();
        const std::string & right_name = world().teamNameRight();

        if (left_name.empty() || right_name.empty()) {
            std::cerr << "[trainer] team names not ready, skip random reset.\n";
            break;
        }

        // 1) 球：全场随机
        const Vector2D ball_pos(uni_x(), uni_y());
        const Vector2D ball_vel(0.0, 0.0);
        doMoveBall(ball_pos, ball_vel);

        // 2) 所有球员：左右队全部在全场随机
        for (int unum = 1; unum <= 11; ++unum)
        {
            // 左队
            {
                const Vector2D pos_L(uni_x(), uni_y());
                // 使用 (teamname, unum, pos) 这个 3 参数版本
                doMovePlayer(left_name, unum, pos_L);
                // 如果你想顺便设朝向，也可以用 4 参数版本：
                // doMovePlayer(left_name, unum, pos_L, AngleDeg(0.0));
            }

            // 右队
            {
                const Vector2D pos_R(uni_x(), uni_y());
                doMovePlayer(right_name, unum, pos_R);
                // 或带朝向：
                // doMovePlayer(right_name, unum, pos_R, AngleDeg(180.0));
            }
        }

        break;
    }
    case OP_RESET_FROM_PY:
        resetFromPython_();
        break;
    default:
        break;
    }
}


void SampleTrainer::resetFromPython_()
{
    if (!shm_ready_ || !shm_) {
        std::cerr << "[trainer][resetFromPy] shm not ready.\n";
        return;
    }

    const std::string & left_name  = world().teamNameLeft();
    const std::string & right_name = world().teamNameRight();
    if (left_name.empty() || right_name.empty()) {
        std::cerr << "[trainer][resetFromPy] team names not ready.\n";
        return;
    }

    // 读球
    const float bx  = rdF_(T_BALL_X);
    const float by  = rdF_(T_BALL_Y);
    const float bvx = rdF_(T_BALL_VX);
    const float bvy = rdF_(T_BALL_VY);

    // recover
    doRecover();

    // 摆球（含速度）
    doMoveBall(Vector2D(bx, by), Vector2D(bvx, bvy));

    // 左队
    for (int i = 0; i < N_LEFT; ++i) {
        const float px  = rdF_(T_LPX(i));
        const float py  = rdF_(T_LPY(i));
        const float dir = rdF_(T_LPD(i));
        const int unum = i + 1;

        doMovePlayer(left_name, unum,
                     Vector2D(px, py),
                     AngleDeg(dir));
    }

    // 右队
    for (int i = 0; i < N_RIGHT; ++i) {
        const float px  = rdF_(T_RPX(i));
        const float py  = rdF_(T_RPY(i));
        const float dir = rdF_(T_RPD(i));
        const int unum = i + 1;

        doMovePlayer(right_name, unum,
                     Vector2D(px, py),
                     AngleDeg(dir));
    }

    // ✅ 切 PlayOn（摆完马上开球）
    doChangeMode(PM_PlayOn);

    // ✅ 清 opcode，防止重复执行
    wr32_(T_OPCODE, OP_NOP);

    std::cerr << "[trainer][resetFromPy] done. "
              << "ball=(" << bx << "," << by << "," << bvx << "," << bvy << ")\n";
}



/*-------------------------------------------------------------------*/
/*!

 */
void
SampleTrainer::actionImpl()
{
    // 1) 队名
    if ( world().teamNameLeft().empty() )
    {
        doTeamNames();
        std::cerr << "[trainer][ERR] teamNameLeft empty -> doTeamNames()\n";
        if ( world().teamNameLeft().empty() ) {
            std::cerr << "[trainer][ERR] doTeamNames() but still empty, return.\n";
            return;
        }
    }

    // 2) shm
    if (!shm_ready_ || !shm_) {
        std::cerr << "[trainer][ERR] shm not ready, return.\n";
        return;
    }

    // 3) 只在 PlayOn 处理（否则直接 return）
    //    这里的接口名按 rcsc 常见写法；如果你编译报错，把这一行换成你工程里正确的取 playmode 的函数即可
    if ( world().gameMode().type() != GameMode::PlayOn ) {
        std::cerr << "[trainer][ERR] not PlayOn (pm=" << world().gameMode().type()
                  << "), skip IPC.\n";
        return;
    }
    else{
        std::cerr << "[trainer]PlayOn mode, ready for IPC.\n";
        // 4) flags：不要覆盖 11；只在 (0,0) 时初始化为 (0,1)
        auto [a0, b0] = trainer_flags(shm_);
        std::cerr << "[trainer][IPC] current flags A=" << int(a0)
                  << " B=" << int(b0) << "\n";
        if (((a0 == 0 && b0 == 0) or (a0 == 1 && b0 == 1))){
            trainer_set_ready(shm_);  // -> 01
        }
        else{
            std::cerr << "[trainer][IPC] keep existing flags A=" << int(a0)
                      << " B=" << int(b0) << "\n";
            return;
        }
        // 5) 等到 Python 写入 10（超时返回并报错）
        if (!(a0 == 1 && b0 == 0)) {
            if (!wait_trainer_request(shm_)) {
                std::cerr << "[trainer][ERR] wait_trainer_request() timeout, return.\n";
                return;
            }
        } else {
            // 已经是 10，也做一次 acquire，保证后续读 payload 可见
            trainer_acquire_fence();
        }

        // 6) ===== 只有这里是“执行动作的分支” =====
        const std::int32_t opcode = rd32_(T_OPCODE);
        std::cerr << "[trainer][IPC] got request opcode=" << opcode << "\n";
        exec_opcode_(opcode);

        // 可选：清 opcode，防止重复执行
        wr32_(T_OPCODE, OP_NOP);

        // 7) 按你要求：结束写 11
        trainer_set_ack11(shm_);
        return;
    }
    
}


/*-------------------------------------------------------------------*/
/*!

*/
void
SampleTrainer::handleInitMessage()
{

}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleTrainer::handleServerParam()
{

}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleTrainer::handlePlayerParam()
{

}

/*-------------------------------------------------------------------*/
/*!

*/
void
SampleTrainer::handlePlayerType()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SampleTrainer::sampleAction()
{
    // sample training to test a ball interception.

    static int s_state = 0;
    static int s_wait_counter = 0;

    static Vector2D s_last_player_move_pos;

    if ( world().existKickablePlayer() )
    {
        s_state = 1;
    }

    switch ( s_state ) {
    case 0:
        // nothing to do
        break;
    case 1:
        // exist kickable left player

        // recover stamina
        doRecover();
        // move ball to center
        doMoveBall( Vector2D( 0.0, 0.0 ),
                    Vector2D( 0.0, 0.0 ) );
        // change playmode to play_on
        doChangeMode( PM_PlayOn );
        {
            // move player to random point
            UniformReal uni01( 0.0, 1.0 );
            Vector2D move_pos
                = Vector2D::polar2vector( 15.0, //20.0,
                                          AngleDeg( 360.0 * uni01() ) );
            s_last_player_move_pos = move_pos;

            doMovePlayer( config().teamName(),
                          1, // uniform number
                          move_pos,
                          move_pos.th() - 180.0 );
        }
        // change player type
        {
            static int type = 0;
            doChangePlayerType( world().teamNameLeft(), 1, type );
            type = ( type + 1 ) % PlayerParam::i().playerTypes();
        }

        doSay( "move player" );
        s_state = 2;
        std::cout << "trainer: actionImpl init episode." << std::endl;
        break;
    case 2:
        ++s_wait_counter;
        if ( s_wait_counter > 3
             && ! world().playersLeft().empty() )
        {
            // add velocity to the ball
            //UniformReal uni_spd( 2.7, 3.0 );
            //UniformReal uni_spd( 2.5, 3.0 );
            UniformReal uni_spd( 2.3, 2.7 );
            //UniformReal uni_ang( -50.0, 50.0 );
            UniformReal uni_ang( -10.0, 10.0 );
            Vector2D velocity
                = Vector2D::polar2vector( uni_spd(),
                                          s_last_player_move_pos.th()
                                          + uni_ang() );
            doMoveBall( Vector2D( 0.0, 0.0 ),
                        velocity );
            s_state = 0;
            s_wait_counter = 0;
            std::cout << "trainer: actionImpl start ball" << std::endl;
        }
        break;

    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SampleTrainer::recoverForever()
{
    if ( world().playersLeft().empty() )
    {
        return;
    }

    if ( world().time().stopped() == 0
         && world().time().cycle() % 50 == 0 )
    {
        // recover stamina
        doRecover();
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SampleTrainer::doSubstitute()
{
    static bool s_substitute = false;
    if ( ! s_substitute
         && world().time().cycle() == 0
         && world().time().stopped() >= 10 )
    {
        std::cerr << "trainer " << world().time() << " team name = "
                  << world().teamNameLeft()
                  << std::endl;

        if ( ! world().teamNameLeft().empty() )
        {
            UniformInt uni( 0, PlayerParam::i().ptMax() );
            doChangePlayerType( world().teamNameLeft(),
                                1,
                                uni() );

            s_substitute = true;
        }
    }

    if ( world().time().stopped() == 0
         && world().time().cycle() % 100 == 1
         && ! world().teamNameLeft().empty() )
    {
        static int type = 0;
        doChangePlayerType( world().teamNameLeft(), 1, type );
        type = ( type + 1 ) % PlayerParam::i().playerTypes();
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SampleTrainer::doKeepaway()
{
    if ( world().trainingTime() == world().time() )
    {
        std::cerr << "trainer: "
                  << world().time()
                  << " keepaway training time." << std::endl;
    }

}
