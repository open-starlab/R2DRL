// -*-c++-*-
/*
 * Copyright:
 *
 * Copyright (C) Hidehisa AKIYAMA
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifndef SAMPLE_TRAINER_H
#define SAMPLE_TRAINER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>  // std::pair

// === helper 依赖 ===
#include <atomic>   // std::atomic_thread_fence
#include <chrono>   // std::chrono::steady_clock
#include <thread>   // std::this_thread::sleep_for

#include <rcsc/trainer/trainer_agent.h>

// ===================== align helper =====================
constexpr std::size_t align4_sz(std::size_t x) {
    return (x + 3) & ~std::size_t(3);
}

// ============================================================================
// SampleTrainer
// ============================================================================
class SampleTrainer : public rcsc::TrainerAgent {
private:
    // ===== Trainer ⟷ Python sequence-based SHM protocol =====
    static constexpr std::size_t TRAINER_SHM_SIZE = 4096;

    static constexpr std::uint8_t TRAINER_PHASE_IDLE = 0;
    static constexpr std::uint8_t TRAINER_PHASE_BUSY = 1;

    static constexpr std::size_t T_PHASE    = 0;  // uint8
    static constexpr std::size_t T_REQ_SEQ  = 4;  // int32
    static constexpr std::size_t T_DONE_SEQ = 8;  // int32
    static constexpr std::size_t T_OPCODE   = 12; // int32
    static constexpr const char* SHM_ENV_NAME = "RCSC_TRAINER_SHM";

    enum Opcode : std::int32_t {
        OP_NOOP            = 0,
        OP_RESET_RANDOMLY  = 1,
        OP_RESET_FROM_PY   = 2,
        OP_PLAY_ON         = 3,
    };

    // 资源句柄
    int           shm_fd_    = -1;
    std::string   shm_name_;
    std::size_t   shm_size_  = 0;
    std::uint8_t* shm_       = nullptr;
    bool          shm_ready_ = false;

    // 生命周期/轮询
    bool init_shm_();
    void close_shm_();
    void exec_opcode_(std::int32_t opcode);

    // ====== payload offsets ======
    // ball: (x,y,vx,vy) 4 floats
    static constexpr std::size_t T_BALL_X  = align4_sz(T_OPCODE + sizeof(std::int32_t)); // 16
    static constexpr std::size_t T_BALL_Y  = T_BALL_X + sizeof(float);                   // 20
    static constexpr std::size_t T_BALL_VX = T_BALL_Y + sizeof(float);                   // 24
    static constexpr std::size_t T_BALL_VY = T_BALL_VX + sizeof(float);                  // 28

    // players: 22 * (x,y,body_deg,vx,vy)
    static constexpr int N_LEFT    = 11;
    static constexpr int N_RIGHT   = 11;
    static constexpr int N_PLAYERS = N_LEFT + N_RIGHT; // 22
    static constexpr std::size_t PLAYER_STRIDE = 5 * sizeof(float); // 20 bytes

    static constexpr std::size_t T_PLAYERS_BASE = align4_sz(T_BALL_VY + sizeof(float)); // 32

    // 左队 i=0..10
    static inline std::size_t T_LPX(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 0*sizeof(float); }
    static inline std::size_t T_LPY(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 1*sizeof(float); }
    static inline std::size_t T_LPD(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 2*sizeof(float); }
    static inline std::size_t T_LVX(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 3*sizeof(float); }
    static inline std::size_t T_LVY(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 4*sizeof(float); }

    // 右队 i=0..10
    static constexpr std::size_t T_R_BASE = T_PLAYERS_BASE + N_LEFT*PLAYER_STRIDE;
    static inline std::size_t T_RPX(int i){ return T_R_BASE + i*PLAYER_STRIDE + 0*sizeof(float); }
    static inline std::size_t T_RPY(int i){ return T_R_BASE + i*PLAYER_STRIDE + 1*sizeof(float); }
    static inline std::size_t T_RPD(int i){ return T_R_BASE + i*PLAYER_STRIDE + 2*sizeof(float); }
    static inline std::size_t T_RVX(int i){ return T_R_BASE + i*PLAYER_STRIDE + 3*sizeof(float); }
    static inline std::size_t T_RVY(int i){ return T_R_BASE + i*PLAYER_STRIDE + 4*sizeof(float); }

    // 轻量读写工具（inline 放头文件）
    inline float rdF_(std::size_t off) const {
        return shm_ ? *(volatile float*)(shm_ + off) : 0.f;
    }
    inline void wrF_(std::size_t off, float v) {
        if (shm_) *(volatile float*)(shm_ + off) = v;
    }

    inline std::uint8_t rd8_(std::size_t off) const {
        return shm_ ? *(volatile std::uint8_t*)(shm_ + off) : 0;
    }
    inline void wr8_(std::size_t off, std::uint8_t v) {
        if (shm_) *(volatile std::uint8_t*)(shm_ + off) = v;
    }
    inline std::int32_t rd32_(std::size_t off) const {
        return shm_ ? *(volatile std::int32_t*)(shm_ + off) : 0;
    }
    inline void wr32_(std::size_t off, std::int32_t v) {
        if (shm_) *(volatile std::int32_t*)(shm_ + off) = v;
    }
    inline std::uint8_t phase_() const {
        return rd8_(T_PHASE);
    }
    inline std::int32_t req_seq_() const {
        return rd32_(T_REQ_SEQ);
    }
    inline std::int32_t done_seq_() const {
        return rd32_(T_DONE_SEQ);
    }

public:
    SampleTrainer();
    virtual ~SampleTrainer();

protected:
    virtual bool initImpl( rcsc::CmdLineParser & cmd_parser );
    virtual void actionImpl();

    virtual void handleInitMessage();
    virtual void handleServerParam();
    virtual void handlePlayerParam();
    virtual void handlePlayerType();

private:
    void resetFromPython_();
    void sampleAction();
    void recoverForever();
    void doSubstitute();
    void doKeepaway();
};

#endif
