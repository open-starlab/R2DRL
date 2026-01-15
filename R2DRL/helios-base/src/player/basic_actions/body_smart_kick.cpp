// -*-c++-*-

/*!
  \file body_smart_kick.cpp
  \brief smart kick action class source file.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 3 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "body_smart_kick.h"

#include "kick_table.h"

#include "body_stop_ball.h"
#include "body_hold_ball2008.h"

#include <rcsc/player/player_agent.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <algorithm>

using namespace rcsc;

// #define DEBUG_PRINT

/*-------------------------------------------------------------------*/
/*!

*/
bool
Body_SmartKick::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::KICK,
                  __FILE__": Body_SmartKick" );

    const WorldModel & wm = agent->world();

    if ( ! wm.self().isKickable() )
    {
        std::cerr << __FILE__ << ": " << __LINE__
                  << " not ball kickable!"
                  << std::endl;
        dlog.addText( Logger::ACTION,
                      __FILE__":  not kickable" );
        return false;
    }

    if ( ! wm.ball().velValid() )
    {
        dlog.addText( Logger::KICK,
                      __FILE__". unknown ball vel" );
        return Body_StopBall().execute( agent );
    }

    double first_speed = bound( 0.001, M_first_speed, ServerParam::i().ballSpeedMax() );
    double first_speed_thr = std::max( 0.0, M_first_speed_thr );
    int max_step = std::max( 1, M_max_step );

    if ( KickTable::instance().simulate( wm,
                                         M_target_point,
                                         first_speed,
                                         first_speed_thr,
                                         max_step,
                                         M_sequence )
         || M_sequence.speed_ >= first_speed_thr )
    {
        agent->debugClient().addMessage( "SmartKick%d", (int)M_sequence.pos_list_.size() );
#ifdef DEBUG_PRINT
        for ( const Vector2D & p : M_sequence.pos_list_ )
        {
            agent->debugClient().addCircle( p, 0.05 );
        }
#endif
        dlog.addText( Logger::KICK,
                      "(Body_SmartKick) Success! target=(%.2f %.2f) speed=%.3f speed_thr=%.3f max_step=%d",
                      M_target_point.x, M_target_point.y,
                      first_speed, first_speed_thr, max_step );

        dlog.addText( Logger::KICK,
                      "(Body_SmartKick) -> achieved_speed=%.3f power=%.2f actual_step=%d",
                      M_sequence.speed_,
                      M_sequence.power_,
                      (int)M_sequence.pos_list_.size() );

        Vector2D vel = M_sequence.pos_list_.front() - wm.ball().pos();
        Vector2D kick_accel = vel - wm.ball().vel();
        
        const double kick_rate = wm.self().kickRate();
        const double kick_power = kick_accel.r() / kick_rate;          // doKick 的 power
        const double kick_dir   = (kick_accel.th() - wm.self().body()).degree(); // doKick 的 dir (deg)

        // 关键调试：看是不是 power 太小/方向异常/球速无效
        std::cerr << "[SmartKick]"
                  << " kickable=" << wm.self().isKickable()
                  << " dist=" << wm.self().distFromBall()
                  << " kickRate=" << kick_rate
                  << " body=" << wm.self().body().degree()
                  << " ball_pos=(" << wm.ball().pos().x << "," << wm.ball().pos().y << ")"
                  << " ball_vel=(" << wm.ball().vel().x << "," << wm.ball().vel().y << ")"
                  << " target=(" << M_target_point.x << "," << M_target_point.y << ")"
                  << " first_speed=" << first_speed
                  << " thr=" << first_speed_thr
                  << " max_step=" << max_step
                  << " achieved_speed=" << M_sequence.speed_
                  << " seq_power=" << M_sequence.power_
                  << " steps=" << M_sequence.pos_list_.size()
                  << " kick_accel=(" << kick_accel.x << "," << kick_accel.y << ")"
                  << " doKick(power=" << kick_power << ", dir=" << kick_dir << "deg)"
                  << std::endl;

        // 如果你也想进 dlog（rcg logger），加这一句
        dlog.addText(Logger::KICK,
                     "(SmartKick DEBUG) dist=%.3f kickRate=%.5f body=%.2f "
                     "ball_vel=(%.3f %.3f) achieved_speed=%.3f seq_power=%.2f "
                     "kick_accel=(%.3f %.3f) doKick(power=%.2f dir=%.2fdeg)",
                     wm.self().distFromBall(),
                     kick_rate,
                     wm.self().body().degree(),
                     wm.ball().vel().x, wm.ball().vel().y,
                     M_sequence.speed_, M_sequence.power_,
                     kick_accel.x, kick_accel.y,
                     kick_power, kick_dir);
        
        agent->doKick( kick_accel.r() / wm.self().kickRate(),
                       kick_accel.th() - wm.self().body() );
        std::cerr <<  " smart kick!" << std::endl;
        return true;
    }


    //
    // TODO: force mode
    //


    // failed to search the kick sequence

    agent->debugClient().addMessage( "SmartKick.Hold" );
    dlog.addText( Logger::KICK,
                  "(Body_SmartKick) Failure! target=(%.2f %.2f) speed=%.3f speed_thr=%.3f max_step=%d",
                  M_target_point.x, M_target_point.y,
                  first_speed,
                  first_speed_thr, max_step );
    dlog.addText( Logger::KICK,
                  "(Body_SmartKick) -> speed=%.3f power=%.2f step=%d",
                  M_sequence.speed_,
                  M_sequence.power_,
                  (int)M_sequence.pos_list_.size() );

    Body_HoldBall2008( false, M_target_point, M_target_point ).execute( agent );
    return false;
}



bool
Body_SmartKick::isExecutable( PlayerAgent * agent )
{

    const WorldModel & wm = agent->world();

    if ( ! wm.self().isKickable() )
    {
        return false;
    }

    if ( ! wm.ball().velValid() )
    {
        return Body_StopBall().isExecutable( agent );
    }

    double first_speed = bound( 0.001, M_first_speed, ServerParam::i().ballSpeedMax() );
    double first_speed_thr = std::max( 0.0, M_first_speed_thr );
    int max_step = std::max( 1, M_max_step );

    if ( KickTable::instance().simulate( wm,
                                         M_target_point,
                                         first_speed,
                                         first_speed_thr,
                                         max_step,
                                         M_sequence )
         || M_sequence.speed_ >= first_speed_thr )
    {
        return true;
    }

    return false;
}
