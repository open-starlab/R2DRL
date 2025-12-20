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
// Trainer SHM flags helpers（头文件内 static inline，避免多重定义）
//
// 约定：A=flag_a, B=flag_b（uint8）
// - READY:   (0,1)  C++ 初始/空闲
// - REQUEST: (1,0)  Python 提交
// - ACK:     (1,1)  C++ 处理结束（按你要求写 11）
//
// 注意：poll_us 单位是微秒：100us = 0.1ms（不是 0.1s）
// ============================================================================
static constexpr int TRAINER_WAIT_REQ_TIMEOUT_MS = 30000;  // 30s
static constexpr int TRAINER_WAIT_REQ_POLL_US    = 100;    // 100us = 0.1ms

static inline std::pair<std::uint8_t, std::uint8_t>
read_flags_trainer(const std::uint8_t* shm, std::size_t off_a, std::size_t off_b)
{
    if (!shm) return {0, 0};
    const volatile std::uint8_t* A = shm + off_a;
    const volatile std::uint8_t* B = shm + off_b;
    return { static_cast<std::uint8_t>(*A), static_cast<std::uint8_t>(*B) };
}

static inline void
write_flags_trainer(std::uint8_t* shm,
                    std::size_t off_a, std::size_t off_b,
                    std::uint8_t a, std::uint8_t b)
{
    if (!shm) return;
    volatile std::uint8_t* A = shm + off_a;
    volatile std::uint8_t* B = shm + off_b;

    // 安全顺序：先写 B，再 release fence，再写 A
    // 目的：当对端看到 A 改变时，能看到之前写入的 opcode/payload
    *B = b;
    std::atomic_thread_fence(std::memory_order_release);
    *A = a;
}

// 固定偏移 wrappers：A=0, B=1（与你的协议一致）
static inline std::pair<std::uint8_t, std::uint8_t>
trainer_flags(const std::uint8_t* shm)
{
    return read_flags_trainer(shm, /*off_a=*/0, /*off_b=*/1);
}

static inline void trainer_set_ready(std::uint8_t* shm) { // (0,1)
    write_flags_trainer(shm, 0, 1, /*a=*/0, /*b=*/1);
}

static inline void trainer_set_ack11(std::uint8_t* shm) { // (1,1)
    write_flags_trainer(shm, 0, 1, /*a=*/1, /*b=*/1);
}

static inline void trainer_acquire_fence() {
    std::atomic_thread_fence(std::memory_order_acquire);
}

// 等待 Python 写 REQUEST(1,0)，超时返回 false
static inline bool
wait_trainer_request(const std::uint8_t* shm,
                     int timeout_ms = TRAINER_WAIT_REQ_TIMEOUT_MS,
                     int poll_us    = TRAINER_WAIT_REQ_POLL_US)
{
    if (!shm) return false;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        auto [a, b] = trainer_flags(shm);
        if (a == 1 && b == 0) {
            // 观测到 REQUEST 后做 acquire，保证后续读 opcode/payload 可见
            trainer_acquire_fence();
            return true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(poll_us));
    }
    return false;
}

// ============================================================================
// SampleTrainer
// ============================================================================
class SampleTrainer : public rcsc::TrainerAgent {
private:
    // ===== Trainer ⟷ Python 极简 SHM 协议：2个flag + 1个int opcode =====
    static constexpr std::size_t TRAINER_SHM_SIZE = 4096;

    static constexpr std::size_t T_FLAG_A  = 0;  // uint8
    static constexpr std::size_t T_FLAG_B  = 1;  // uint8
    static constexpr std::size_t T_OPCODE  = 4;  // int32 (对齐到 4)
    static constexpr const char* SHM_ENV_NAME = "RCSC_TRAINER_SHM";

    enum Opcode : std::int32_t {
        OP_NOP           = 0,
        OP_RESET_TO_BKO  = 1,
        OP_SET_BALL_XY   = 2,
        OP_SET_PLAYER_XY = 3,
        OP_RESET_FROM_PY = 10,
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
    bool try_handle_trainer_ipc_();
    void exec_opcode_(std::int32_t opcode);

    // ====== payload offsets ======
    // ball: (x,y,vx,vy) 4 floats
    static constexpr std::size_t T_BALL_X  = align4_sz(T_OPCODE + sizeof(std::int32_t)); // 8
    static constexpr std::size_t T_BALL_Y  = T_BALL_X + sizeof(float);                   // 12
    static constexpr std::size_t T_BALL_VX = T_BALL_Y + sizeof(float);                   // 16
    static constexpr std::size_t T_BALL_VY = T_BALL_VX + sizeof(float);                  // 20

    // players: 22 * (x,y,dir_deg)
    static constexpr int N_LEFT    = 11;
    static constexpr int N_RIGHT   = 11;
    static constexpr int N_PLAYERS = N_LEFT + N_RIGHT; // 22
    static constexpr std::size_t PLAYER_STRIDE = 3 * sizeof(float); // 12 bytes

    static constexpr std::size_t T_PLAYERS_BASE = align4_sz(T_BALL_VY + sizeof(float)); // 24

    // 左队 i=0..10
    static inline std::size_t T_LPX(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 0*sizeof(float); }
    static inline std::size_t T_LPY(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 1*sizeof(float); }
    static inline std::size_t T_LPD(int i){ return T_PLAYERS_BASE + i*PLAYER_STRIDE + 2*sizeof(float); }

    // 右队 i=0..10
    static constexpr std::size_t T_R_BASE = T_PLAYERS_BASE + N_LEFT*PLAYER_STRIDE;
    static inline std::size_t T_RPX(int i){ return T_R_BASE + i*PLAYER_STRIDE + 0*sizeof(float); }
    static inline std::size_t T_RPY(int i){ return T_R_BASE + i*PLAYER_STRIDE + 1*sizeof(float); }
    static inline std::size_t T_RPD(int i){ return T_R_BASE + i*PLAYER_STRIDE + 2*sizeof(float); }

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
    inline std::pair<std::uint8_t,std::uint8_t> flags_() const {
        return { rd8_(T_FLAG_A), rd8_(T_FLAG_B) };
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
