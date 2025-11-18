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

#ifndef SAMPLE_TRAINER_H
#define SAMPLE_TRAINER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>  // for std::pair


#include <rcsc/trainer/trainer_agent.h>

class SampleTrainer
    : public rcsc::TrainerAgent {
private:
    // ===== Trainer ⟷ Python 极简 SHM 协议：2个flag + 1个int opcode =====
    // 只用到前8字节，其它空间作为预留
    static constexpr std::size_t TRAINER_SHM_SIZE = 4096;
    static constexpr std::size_t T_FLAG_A = 0;  // uint8  (C++ 写)
    static constexpr std::size_t T_FLAG_B = 1;  // uint8  (Python 写)
    static constexpr std::size_t T_OPCODE = 4;  // int32  对齐到4
    static constexpr const char* SHM_ENV_NAME = "RCSC_TRAINER_SHM";

    // 可选：给 opcode 起名，方便读代码（Python 只要写数字即可）
    enum Opcode : std::int32_t {
        OP_NOP           = 0,
        OP_RESET_TO_BKO  = 1,
        OP_SET_BALL_XY   = 2,   // 需要时在 .cpp 扩展额外参数布局
        OP_SET_PLAYER_XY = 3,
    };

    // 资源句柄
    int             shm_fd_    = -1;      // shm_open 的 fd
    std::string     shm_name_;            // getenv("RCSC_TRAINER_SHM")
    std::size_t     shm_size_  = 0;       // 实际映射大小（期望 4096）
    std::uint8_t*   shm_       = nullptr; // 映射指针
    bool            shm_ready_ = false;   // 是否可用

    // 生命周期/轮询
    bool init_shm_();                 // 读取 env → shm_open + ftruncate + mmap
    void close_shm_();                // munmap + close
    bool try_handle_trainer_ipc_();   // 检测 (A,B) 状态并处理一条指令（在 actionImpl() 里调用）
    void exec_opcode_(std::int32_t opcode); // 最小 switch 分发

    // 轻量读写工具（inline 放头文件，减少 .cpp 负担）
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

    virtual
    ~SampleTrainer();

protected:

    /*!
      You can override this method.
      But you must call TrainerAgent::doInit() in this method.
    */
    virtual
    bool initImpl( rcsc::CmdLineParser & cmd_parser );

    //! main decision
    virtual
    void actionImpl();

    virtual
    void handleInitMessage();
    virtual
    void handleServerParam();
    virtual
    void handlePlayerParam();
    virtual
    void handlePlayerType();

private:

    void sampleAction();
    void recoverForever();
    void doSubstitute();
    void doKeepaway();

};

#endif