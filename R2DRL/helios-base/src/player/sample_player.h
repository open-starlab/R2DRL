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

#ifndef SAMPLE_PLAYER_H
#define SAMPLE_PLAYER_H

#include "action_generator.h"
#include "field_evaluator.h"
#include "communication.h"

#include <rcsc/player/player_agent.h>
#include <vector>

#include <array>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>  // uint8_t
#include <string>
#include <atomic>

enum ActionType {
    ACTION_TACKLE = 0,        // 铲球
    ACTION_SHOOT,             // 射门（严格可行，否则降级解围）
    ACTION_INTERCEPT,         // 追球/拦截
    ACTION_ADVANCE,           // 推进/解围（AdvanceBall）
    ACTION_PASS_DIRECT,       // 传球：Direct
    ACTION_PASS_LEAD,         // 传球：Lead
    ACTION_PASS_THROUGH,      // 传球：Through
    ACTION_HOLD,              // 控球
    ACTION_CATCH,             // CATCH
    ACTION_DRIBBLE_UP,        // 带球 向上
    ACTION_DRIBBLE_DOWN,      // 带球 向下
    ACTION_DRIBBLE_LEFT,      // 带球 向左
    ACTION_DRIBBLE_RIGHT,     // 带球 向右
    ACTION_MOVE_UP,           // 无球位移 向上（doMoveTo(0)）
    ACTION_MOVE_DOWN,         // 无球位移 向下（doMoveTo(1)）
    ACTION_MOVE_LEFT,         // 无球位移 向左（doMoveTo(2)）
    ACTION_MOVE_RIGHT,        // 无球位移 向右（doMoveTo(3)）
    BASE_ACTION_NUM                 // = 17
};

constexpr size_t align4(size_t x) {
     return (x + 3) & ~size_t(3);
}


class SamplePlayer
    : public rcsc::PlayerAgent {
private:

    Communication::Ptr M_communication;

    FieldEvaluator::ConstPtr M_field_evaluator;
    ActionGenerator::ConstPtr M_action_generator;

    static constexpr int STATE_NUM = 97;
    bool handshake_done_ = false;
    // C++ & Python 必须一致
    static constexpr size_t OFFSET_FLAG_A   = 0;  // 1 byte
    static constexpr size_t OFFSET_FLAG_B   = 1;  // 1 byte
    static constexpr size_t OFFSET_MASK     = align4(OFFSET_FLAG_B + 1);  // => 4
    static constexpr size_t OFFSET_CYCLE    = align4(OFFSET_MASK + BASE_ACTION_NUM /* or MASK_NUM */);
    static constexpr size_t OFFSET_STATE    = align4(OFFSET_CYCLE + sizeof(int32_t));
    static constexpr size_t OFFSET_ACTION   = align4(OFFSET_STATE + STATE_NUM * sizeof(float));
    
    static constexpr size_t OFFSET_HYBRID_MASK = align4(OFFSET_ACTION + sizeof(int32_t));          // 4 * uint8_t
    static constexpr size_t OFFSET_HYBRID_ACT  = align4(OFFSET_HYBRID_MASK + 4 * sizeof(uint8_t)); // int32_t
    static constexpr size_t OFFSET_HYBRID_U0   = align4(OFFSET_HYBRID_ACT + sizeof(int32_t));      // float
    static constexpr size_t OFFSET_HYBRID_U1   = align4(OFFSET_HYBRID_U0 + sizeof(float));         // float

    static constexpr size_t SHM_SIZE           = align4(OFFSET_HYBRID_U1 + sizeof(float));

    std::string RUN_MODE_ = "Helios";   // 只保存原始参数（可留作日志）
    enum class Mode { Base, Helios, Hybrid };
    Mode mode_ = Mode::Helios;          // ✅ 运行期使用的枚举

public:

    SamplePlayer();

    virtual
    ~SamplePlayer();

protected:

    /*!
      You can override this method.
      But you must call PlayerAgent::initImpl() in this method.
    */
    virtual
    bool initImpl( rcsc::CmdLineParser & cmd_parser );

    //! main decision
    virtual
    void actionImpl();
    void writeSharedMemory();
    
    //! communication decision
    virtual
    void communicationImpl();

    virtual
    void handleActionStart();

    std::vector<float> GetState();

    virtual
    void handleActionEnd();

    virtual
    void handleInitMessage();
    virtual
    void handleServerParam();
    virtual
    void handlePlayerParam();
    virtual
    void handlePlayerType();

    virtual
    FieldEvaluator::ConstPtr createFieldEvaluator() const;

    virtual
    ActionGenerator::ConstPtr createActionGenerator() const;

private:
    void* shm_ptr = nullptr;  // 共享内存映射地址
    std::string SHM_NAME = "";  // 支持命令行传入

    bool doPreprocess();
    bool doShoot();
    bool doForceKick();
    bool doHeardPassReceive();
    bool doMoveTo(int n);
    bool isDoShootExecutable();
    bool isDoForceKickExecutable();
    bool initSharedMemory(); 
    void runHeliosFrame_();
    bool isDoCatchExecutable() const;
    int getActionFromSharedMemory();  // 在 actionImpl 中调用
    bool inOurPenaltyArea() const;
public:
    void writeHybridMaskToSharedMemory();
    bool readHybridActionFromSharedMemory(int &a, float &u0, float &u1, int timeout_ms = 5000);
    bool takeHybridAction(int a, double u0, double u1);
    std::array<bool,4> getHybridActionMask() const;
    void setHybridActionMask();
    
    virtual
    FieldEvaluator::ConstPtr getFieldEvaluator() const;
    const std::array<bool, BASE_ACTION_NUM>& getActionMask() const;
    void setActionMask();
    void takeAction(int n);
    std::vector<float> getAllState() const; 
protected:
    // FeatureExtractor* feature_extractor;
    // long lastTrainerMessageTime;
    int num_teammates, num_opponents;
    bool playing_offense;
    std::array<bool, BASE_ACTION_NUM> action_mask;

};

#endif
