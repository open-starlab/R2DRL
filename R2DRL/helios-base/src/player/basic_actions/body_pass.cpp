// -*-c++-*-

/*!
  \file body_pass.cpp
  \brief advanced pass planning & behavior.
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

#include "body_pass.h"

//#include "intention_kick.h"
#include "body_kick_one_step.h"
//#include "body_kick_two_step.h"
//#include "body_kick_multi_step.h"
#include "body_smart_kick.h"
#include "body_stop_ball.h"
#include "body_hold_ball2008.h"

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/say_message_builder.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/rect_2d.h>
#include <rcsc/geom/sector_2d.h>
#include <rcsc/soccer_math.h>
#include <rcsc/math_util.h>
#include <iostream>
#include <cstdlib>
using namespace rcsc;

//#define DEBUG

std::vector< Body_Pass::PassRoute > Body_Pass::S_cached_pass_route;
// 新增的三个定义：
std::vector<Body_Pass::PassRoute> Body_Pass::S_cached_direct_routes;
std::vector<Body_Pass::PassRoute> Body_Pass::S_cached_lead_routes;
std::vector<Body_Pass::PassRoute> Body_Pass::S_cached_through_routes;

namespace {
inline bool pick_best_from(const std::vector<Body_Pass::PassRoute>& routes,
                           rcsc::Vector2D* target_point,
                           double* first_speed,
                           int* receiver,
                           const Body_Pass::PassRoute** best_out = nullptr)
{
    if (routes.empty()) return false;
    auto it = std::max_element(routes.begin(), routes.end(),
                               [](const Body_Pass::PassRoute& a, const Body_Pass::PassRoute& b){
                                   return a.score_ < b.score_;
                               });
    if (target_point) *target_point = it->receive_point_;
    if (first_speed)  *first_speed  = it->first_speed_;
    if (receiver)     *receiver     = it->receiver_ ? it->receiver_->unum() : Unum_Unknown;
    if (best_out)     *best_out     = &(*it);
    return true;
}
} // namespace

/*-------------------------------------------------------------------*/
/*!
  execute action
// */
// bool
// Body_Pass::execute( PlayerAgent * agent )
// {
//     dlog.addText( Logger::ACTION,
//                   "%s:%d: Body_Pass. execute()"
//                   ,__FILE__, __LINE__ );

//     if ( ! agent->world().self().isKickable() )
//     {
//         std::cerr << __FILE__ << ": " << __LINE__
//                   << " not ball kickable!"
//                   << std::endl;
//         dlog.addText( Logger::ACTION,
//                       "%s:%d:  not kickable"
//                       ,__FILE__, __LINE__ );
//         return false;
//     }

//     Vector2D target_point(50.0, 0.0);
//     double first_speed = 0.0;
//     int receiver = 0;

//     if ( ! get_best_pass( agent->world(), &target_point, &first_speed, &receiver ) )
//     {
//         return false;
//     }


//     // evaluation
//     //   judge situation
//     //   decide max kick step
//     //

//     agent->debugClient().addMessage( "pass" );
//     agent->debugClient().setTarget( target_point );

//     int kick_step = ( agent->world().gameMode().type() != GameMode::PlayOn
//                       && agent->world().gameMode().type() != GameMode::GoalKick_
//                       ? 1
//                       : 3 );
//     if ( ! Body_SmartKick( target_point,
//                            first_speed,
//                            first_speed * 0.96,
//                            kick_step ).execute( agent ) )
//     {
//         if ( agent->world().gameMode().type() != GameMode::PlayOn
//              && agent->world().gameMode().type() != GameMode::GoalKick_ )
//         {
//             first_speed = std::min( agent->world().self().kickRate() * ServerParam::i().maxPower(),
//                                     first_speed );
//             Body_KickOneStep( target_point,
//                               first_speed
//                               ).execute( agent );
//             dlog.addText( Logger::ACTION,
//                           __FILE__": execute() one step kick" );
//         }
//         else
//         {
//             dlog.addText( Logger::ACTION,
//                           __FILE__": execute() failed to pass kick." );
//             return false;
//         }
//     }

//     if ( agent->config().useCommunication()
//          && receiver != Unum_Unknown )
//     {
//         dlog.addText( Logger::ACTION,
//                       __FILE__": execute() set pass communication." );
//         Vector2D target_buf = target_point - agent->world().self().pos();
//         target_buf.setLength( 1.0 );

//         agent->addSayMessage( new PassMessage( receiver,
//                                                target_point + target_buf,
//                                                agent->effector().queuedNextBallPos(),
//                                                agent->effector().queuedNextBallVel() ) );
//     }

//     return true;
// }


bool
Body_Pass::execute( PlayerAgent * agent )
{
    //1024
    std::cerr << "[HIT] Body_Pass::execute()" << std::endl;
    std::abort();  // 一旦调用就崩，堆栈一目了然
    return false;

    if ( ! agent->world().self().isKickable() ) //kickable
    {
        return false;
    }

    Vector2D target_point(50.0, 0.0);
    double first_speed = 0.0;
    int receiver = 0;

    if ( ! get_best_pass( agent->world(), &target_point, &first_speed, &receiver ) )
    {
        return false;
    }


    // evaluation
    //   judge situation
    //   decide max kick step
    //

    agent->debugClient().addMessage( "pass" );
    agent->debugClient().setTarget( target_point );

    int kick_step = ( agent->world().gameMode().type() != GameMode::PlayOn
                      && agent->world().gameMode().type() != GameMode::GoalKick_
                      ? 1
                      : 3 );
    if ( ! Body_SmartKick( target_point,
                           first_speed,
                           first_speed * 0.96,
                           kick_step ).execute( agent ) )
    {
        if ( agent->world().gameMode().type() != GameMode::PlayOn
             && agent->world().gameMode().type() != GameMode::GoalKick_ )
        {
            first_speed = std::min( agent->world().self().kickRate() * ServerParam::i().maxPower(),
                                    first_speed );
            Body_KickOneStep( target_point,
                              first_speed
                              ).execute( agent );
            dlog.addText( Logger::ACTION,
                          __FILE__": execute() one step kick" );
        }
        else
        {
            dlog.addText( Logger::ACTION,
                          __FILE__": execute() failed to pass kick." );
            return false;
        }
    }

    if ( agent->config().useCommunication()
         && receiver != Unum_Unknown )
    {
        dlog.addText( Logger::ACTION,
                      __FILE__": execute() set pass communication." );
        Vector2D target_buf = target_point - agent->world().self().pos();
        target_buf.setLength( 1.0 );

        agent->addSayMessage( new PassMessage( receiver,
                                               target_point + target_buf,
                                               agent->effector().queuedNextBallPos(),
                                               agent->effector().queuedNextBallVel() ) );
    }

    return true;
}

bool
Body_Pass::isExecutable( PlayerAgent * agent )
{

    if ( ! agent->world().self().isKickable() )
    {
        return false;
    }

    Vector2D target_point(50.0, 0.0);
    double first_speed = 0.0;
    int receiver = 0;

    if ( ! get_best_pass( agent->world(), &target_point, &first_speed, &receiver ) )
    {
        return false;
    }

    int kick_step = ( agent->world().gameMode().type() != GameMode::PlayOn
                      && agent->world().gameMode().type() != GameMode::GoalKick_
                      ? 1
                      : 3 );
    if ( ! Body_SmartKick( target_point,
                           first_speed,
                           first_speed * 0.96,
                           kick_step ).isExecutable( agent ) )
    {
        if ( agent->world().gameMode().type() != GameMode::PlayOn
             && agent->world().gameMode().type() != GameMode::GoalKick_ )
        {

        }
        else
        {
            return false;
        }
    }

    return true;
}
/*-------------------------------------------------------------------*/
/*!
  static method
*/
bool Body_Pass::get_best_direct(const WorldModel& world,
                                Vector2D* tp, double* fs, int* recv)
{
    create_routes_direct_only(world);                 // 填充 S_cached_direct_routes 并打分
    return pick_best_from(S_cached_direct_routes, tp, fs, recv);
}

bool Body_Pass::get_best_lead(const WorldModel& world,
                              Vector2D* tp, double* fs, int* recv)
{
    create_routes_lead_only(world);
    return pick_best_from(S_cached_lead_routes, tp, fs, recv);
}

bool Body_Pass::get_best_through(const WorldModel& world,
                                 Vector2D* tp, double* fs, int* recv)
{
    create_routes_through_only(world);
    return pick_best_from(S_cached_through_routes, tp, fs, recv);
}


bool
Body_Pass::get_best_pass( const WorldModel & world,
                          Vector2D * target_point,
                          double * first_speed,
                          int * receiver )
{
    static GameTime S_last_calc_time( 0, 0 );
    static bool S_last_calc_valid = false;
    static Vector2D S_last_calc_target;
    static double S_last_calc_speed = 0.0;
    static int S_last_calc_receiver = Unum_Unknown;

    if ( S_last_calc_time == world.time() )
    {
        if ( S_last_calc_valid )
        {
            if ( target_point )
            {
                *target_point = S_last_calc_target;
            }
            if ( first_speed )
            {
                *first_speed = S_last_calc_speed;
            }
            if ( receiver )
            {
                *receiver = S_last_calc_receiver;
            }
            return true;
        }
        return false;
    }

    S_last_calc_time = world.time();
    S_last_calc_valid = false;

    // create route
    create_routes( world );

    if ( ! S_cached_pass_route.empty() )
    {
        std::vector< PassRoute >::iterator max_it
            = std::max_element( S_cached_pass_route.begin(),
                                S_cached_pass_route.end(),
                                []( const PassRoute & lhs, const PassRoute & rhs )
                                  {
                                      return lhs.score_ < rhs.score_;
                                  } );
        S_last_calc_target = max_it->receive_point_;
        S_last_calc_speed = max_it->first_speed_;
        S_last_calc_receiver = max_it->receiver_->unum();
        S_last_calc_valid = true;
        dlog.addText( Logger::ACTION,
                      "%s:%d: get_best_pass() size=%d. target=(%.1f %.1f)"
                      " speed=%.3f  receiver=%d"
                      ,__FILE__, __LINE__,
                      S_cached_pass_route.size(),
                      S_last_calc_target.x, S_last_calc_target.y,
                      S_last_calc_speed,
                      S_last_calc_receiver );
    }

    if ( S_last_calc_valid )
    {
        if ( target_point )
        {
            *target_point = S_last_calc_target;
        }
        if ( first_speed )
        {
            *first_speed = S_last_calc_speed;
        }
        if ( receiver )
        {
            *receiver = S_last_calc_receiver;
        }

        dlog.addText( Logger::ACTION,
                      "%s:%d: best pass (%.2f, %.2f). speed=%.2f. receiver=%d"
                      ,__FILE__, __LINE__,
                      S_last_calc_target.x, S_last_calc_target.y,
                      S_last_calc_speed, S_last_calc_receiver );
    }

    return S_last_calc_valid;
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
void
Body_Pass::create_routes_direct_only( const WorldModel & world )
{
    S_cached_direct_routes.clear();

    for ( const PlayerObject * t : world.teammatesFromSelf() )
    {
        if ( !t ) continue;
        if ( t->goalie() && t->pos().x < -22.0 ) continue;
        if ( t->posCount() > 3 ) continue;
        if ( t->pos().x > world.offsideLineX() + 1.0 ) continue;
        if ( t->pos().x < world.ball().pos().x - 25.0 ) continue;

        // 只生成 direct 候选到 direct 缓存
        create_direct_pass( world, t, S_cached_direct_routes );
    }

    evaluate_routes_on( world, S_cached_direct_routes );
}

void
Body_Pass::create_routes_lead_only( const WorldModel & world )
{
    S_cached_lead_routes.clear();

    for ( const PlayerObject * t : world.teammatesFromSelf() )
    {
        if ( !t ) continue;
        if ( t->goalie() && t->pos().x < -22.0 ) continue;
        if ( t->posCount() > 3 ) continue;
        if ( t->pos().x > world.offsideLineX() + 1.0 ) continue;
        if ( t->pos().x < world.ball().pos().x - 25.0 ) continue;

        create_lead_pass( world, t, S_cached_lead_routes );
    }

    evaluate_routes_on( world, S_cached_lead_routes );
}

void
Body_Pass::create_routes_through_only( const WorldModel & world )
{
    S_cached_through_routes.clear();

    for ( const PlayerObject * t : world.teammatesFromSelf() )
    {
        if ( !t ) continue;
        if ( t->goalie() && t->pos().x < -22.0 ) continue;
        if ( t->posCount() > 3 ) continue;
        if ( t->pos().x > world.offsideLineX() + 1.0 ) continue;
        if ( t->pos().x < world.ball().pos().x - 25.0 ) continue;

        // 与原 create_routes 一致：接近越位线才考虑 through
        if ( world.self().pos().x > world.offsideLineX() - 20.0 )
        {
            create_through_pass( world, t, S_cached_through_routes );
        }
    }

    evaluate_routes_on( world, S_cached_through_routes );
}


void
Body_Pass::create_routes( const WorldModel & world )
{
    // reset old info
    S_cached_pass_route.clear();

    // loop candidate teammates
    for ( const PlayerObject * t : world.teammatesFromSelf() )
    {
        if ( t->goalie() && t->pos().x < -22.0 )
        {
            // goalie is rejected.
            continue;
        }
        if ( t->posCount() > 3 )
        {
            // low confidence players are rejected.
            continue;
        }
        if ( t->pos().x > world.offsideLineX() + 1.0 )
        {
            // offside players are rejected.
            continue;
        }
        if ( t->pos().x < world.ball().pos().x - 25.0 )
        {
            // too back
            continue;
        }

        // create & verify each route
        create_direct_pass( world, t , S_cached_pass_route);
        create_lead_pass( world, t , S_cached_pass_route);
        if ( world.self().pos().x > world.offsideLineX() - 20.0 )
        {
            create_through_pass( world, t , S_cached_pass_route);
        }
    }

    ////////////////////////////////////////////////////////////////
    // evaluation
    evaluate_routes( world );
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
void
Body_Pass::create_direct_pass( const WorldModel & world,
                               const PlayerObject * receiver,
                               std::vector<PassRoute> & out )
{
    static const double MAX_DIRECT_PASS_DIST
        = 0.8 * inertia_final_distance( ServerParam::i().ballSpeedMax(),
                                              ServerParam::i().ballDecay() );
#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "Create_direct_pass() to %d(%.1f %.1f)",
                  receiver->unum(),
                  receiver->pos().x, receiver->pos().y );
#endif

    // out of pitch?
    if ( receiver->pos().absX() > ServerParam::i().pitchHalfLength() - 3.0
         || receiver->pos().absY() > ServerParam::i().pitchHalfWidth() - 3.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ out of pitch" );
#endif
        return;
    }

    /////////////////////////////////////////////////////////////////
    // too far
    if ( receiver->distFromSelf() > MAX_DIRECT_PASS_DIST )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ over max distance %.2f > %.2f",
                      receiver->distFromSelf(),
                      MAX_DIRECT_PASS_DIST );
#endif
        return;
    }
    // too close
    if ( receiver->distFromSelf() < 6.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ too close. dist = %.2f  canceled",
                      receiver->distFromSelf() );
#endif
        return;
    }
    /////////////////
    if ( receiver->pos().x < world.self().pos().x + 5.0 )
    {
        if ( receiver->angleFromSelf().abs() < 40.0
             || ( receiver->pos().x > world.ourDefenseLineX() + 10.0
                  && receiver->pos().x > 0.0 )
             )
        {
            // "DIRECT: not defender." <<;
        }
        else
        {
            // "DIRECT: back.;
#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "__ looks back pass. DF Line=%.1f. canceled",
                          world.defenseLineX() );
#endif
            return;
        }
    }

    // not safety area
    if ( receiver->pos().x < -20.0 )
    {
        if ( receiver->pos().x > world.self().pos().x + 13.0 )
        {
            // safety clear??
        }
        else if ( receiver->pos().x > world.self().pos().x + 5.0
                  && receiver->pos().absY() > 20.0
                  && fabs(receiver->pos().y - world.self().pos().y) < 20.0
                  )
        {
            // safety area
        }

        else
        {
            // dangerous
#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "__ receiver is in dangerous area. canceled" );
#endif
            return;
        }
    }

    /////////////////////////////////////////////////////////////////

    // set base target
    Vector2D base_player_pos = receiver->pos();
    if ( receiver->velCount() < 3 )
    {
        Vector2D fvel = receiver->vel();
        fvel /= ServerParam::i().defaultPlayerDecay();
        fvel *= std::min( receiver->velCount() + 1, 2 );
        base_player_pos += fvel;
    }

    const Vector2D receiver_rel = base_player_pos - world.ball().pos();
    const double receiver_dist = receiver_rel.r();
    const AngleDeg receiver_angle = receiver_rel.th();

#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "__ receiver. predict pos(%.2f %.2f) rel(%.2f %.2f)"
                  "  dist=%.2f  angle=%.1f",
                  base_player_pos.x, base_player_pos.y,
                  receiver_rel.x, receiver_rel.y,
                  receiver_dist,
                  receiver_angle.degree() );

#endif

    double end_speed = 1.5;
    double first_speed = 100.0;
    do
    {
        first_speed
            = calc_first_term_geom_series_last
            ( end_speed,
              receiver_dist,
              ServerParam::i().ballDecay() );
        //= required_first_speed( receiver_dist,
        //ServerParam::i().ballDecay(),
        //end_speed );
        if ( first_speed < ServerParam::i().ballSpeedMax() )
        {
            break;
        }
        end_speed -= 0.1;
    }
    while ( end_speed > 0.8 );


    if ( first_speed > ServerParam::i().ballSpeedMax() )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ ball first speed= %.3f.  too high. canceled",
                      first_speed );
#endif
        return;
    }


    // add strictly direct pass
    if ( verify_direct_pass( world,
                             receiver,
                             base_player_pos,
                             receiver_dist,
                             receiver_angle,
                             first_speed ) )
    {
        out.emplace_back( DIRECT,
                                          receiver,
                                          base_player_pos,
                                          first_speed,
                                          can_kick_by_one_step( world,
                                                                first_speed,
                                                                receiver_angle ) );
    }

    // add kickable edge points
    double kickable_angle_buf = 360.0 * ( ServerParam::i().defaultKickableArea()
                                          / (2.0 * M_PI * receiver_dist) );
    first_speed *= ServerParam::i().ballDecay();

    // right side
    Vector2D target_new = world.ball().pos();
    AngleDeg angle_new = receiver_angle;
    angle_new += kickable_angle_buf;
    target_new += Vector2D::polar2vector(receiver_dist, angle_new);

    if ( verify_direct_pass( world,
                             receiver,
                             target_new,
                             receiver_dist,
                             angle_new,
                             first_speed ) )
    {
        out.emplace_back( DIRECT,
                                          receiver,
                                          target_new,
                                          first_speed,
                                          can_kick_by_one_step( world,
                                                                first_speed,
                                                                angle_new ) );
    }
    // left side
    target_new = world.ball().pos();
    angle_new = receiver_angle;
    angle_new -= kickable_angle_buf;
    target_new += Vector2D::polar2vector( receiver_dist, angle_new );

    if ( verify_direct_pass( world,
                             receiver,
                             target_new,
                             receiver_dist,
                             angle_new,
                             first_speed ) )
    {
        out.emplace_back( DIRECT,
                                          receiver,
                                          target_new,
                                          first_speed,
                                          can_kick_by_one_step( world,
                                                                first_speed,
                                                                angle_new ) );
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "Pass Success direct unum=%d pos=(%.1f %.1f). first_speed= %.1f",
                      receiver->unum(),
                      target_new.x, target_new.y,
                      first_speed );
#endif
    }
    else
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "Pass Failed direct unum=%d pos=(%.1f %.1f). first_speed= %.1f",
                      receiver->unum(),
                      target_new.x, target_new.y,
                      first_speed );
#endif
    }
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
void
Body_Pass::create_lead_pass( const WorldModel & world,
                             const PlayerObject * receiver,
                             std::vector<PassRoute> & out )
{
    static const double MAX_LEAD_PASS_DIST
        = 0.7 * inertia_final_distance( ServerParam::i().ballSpeedMax(),
                                              ServerParam::i().ballDecay() );
    static const
        Rect2D shrinked_pitch( Vector2D( -ServerParam::i().pitchHalfLength() + 3.0,
                                         -ServerParam::i().pitchHalfWidth() + 3.0 ),
                               Size2D( ServerParam::i().pitchLength() - 6.0,
                                       ServerParam::i().pitchWidth() - 6.0 ) );

    //static const double receiver_dash_speed = 0.9;
    static const double receiver_dash_speed = 0.8;

#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "Create_lead_pass() to %d(%.1f %.1f)",
                  receiver->unum(),
                  receiver->pos().x, receiver->pos().y );
#endif

    /////////////////////////////////////////////////////////////////
    // too far
    if ( receiver->distFromSelf() > MAX_LEAD_PASS_DIST )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ over max distance %.2f > %.2f",
                      receiver->distFromSelf(), MAX_LEAD_PASS_DIST );
#endif
        return;
    }
    // too close
    if ( receiver->distFromSelf() < 2.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ too close %.2f",
                      receiver->distFromSelf() );
#endif
        return;
    }
    if ( receiver->pos().x < world.self().pos().x - 15.0
         && receiver->pos().x < 15.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver is back cancel" );
#endif
        return;
    }
    //
    if ( receiver->pos().x < -10.0
         && std::fabs( receiver->pos().y - world.self().pos().y ) > 20.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver is in our field. or Y diff is big" );
#endif
        return;
    }

    const Vector2D receiver_rel = receiver->pos() - world.ball().pos();
    const double receiver_dist = receiver_rel.r();
    const AngleDeg receiver_angle = receiver_rel.th();

    const double circum = 2.0 * receiver_dist * M_PI;
    const double angle_step_abs = std::max( 5.0, 360.0 * ( 2.0 / circum ));
    /// angle loop
    double total_add_angle_abs;
    int count;
    for ( total_add_angle_abs = angle_step_abs, count = 0;
          total_add_angle_abs < 31.0 && count < 5;
          total_add_angle_abs += angle_step_abs, ++count )
    {
        /////////////////////////////////////////////////////////////
        // estimate required first ball speed
        const double dash_step
            = ( (angle_step_abs / 360.0) * circum ) / receiver_dash_speed + 2.0;

        double end_speed = 1.2 - 0.11 * count; // Magic Number
        double first_speed = 100.0;
        double ball_steps_to_target = 100.0;
        do
        {
            first_speed
                = calc_first_term_geom_series_last
                ( end_speed,
                  receiver_dist,
                  ServerParam::i().ballDecay() );
            //= required_first_speed( receiver_dist,
            //ServerParam::i().ballDecay(),
            //end_speed );
            if ( first_speed < ServerParam::i().ballSpeedMax() )
            {
                break;
            }
            ball_steps_to_target
                = calc_length_geom_series( first_speed,
                                                 receiver_dist,
                                                 ServerParam::i().ballDecay() );
            //= move_step_for_first( receiver_dist,
            //first_speed,
            //ServerParam::i().ballDecay() );
            if ( dash_step < ball_steps_to_target )
            {
                break;
            }
            end_speed -= 0.1;
        }
        while ( end_speed > 0.5 );


        if ( first_speed > ServerParam::i().ballSpeedMax() )
        {
            continue;
        }

        /////////////////////////////////////////////////////////////
        // angle plus minus loop
        for ( int i = 0; i < 2; i++ )
        {
            AngleDeg target_angle = receiver_angle;
            if ( i == 0 )
            {
                target_angle -= total_add_angle_abs;
            }
            else
            {
                target_angle += total_add_angle_abs;
            }

            // check dir confidence
            int max_count = 100, ave_count = 100;
            world.dirRangeCount( target_angle, 20.0,
                                 &max_count, NULL, &ave_count );
            if ( max_count > 9 || ave_count > 3 )
            {
                continue;
            }

            const Vector2D target_point
                = world.ball().pos()
                + Vector2D::polar2vector(receiver_dist, target_angle);

            /////////////////////////////////////////////////////////////////
            // ignore back pass
            if ( target_point.x < 0.0
                 && target_point.x <  world.self().pos().x )
            {
                continue;
            }

            if ( target_point.x < 0.0
                 && target_point.x < receiver->pos().x - 3.0 )
            {
                continue;
            }
            if ( target_point.x < receiver->pos().x - 6.0 )
            {
                continue;
            }
            // out of pitch
            if ( ! shrinked_pitch.contains( target_point ) )
            {
                continue;
            }
            // not safety area
            if ( target_point.x < -10.0 )
            {
                if ( target_point.x < world.ourDefenseLineX() + 10.0 )
                {
                    continue;
                }
                else if ( target_point.x > world.self().pos().x + 20.0
                          && fabs(target_point.y - world.self().pos().y) < 20.0 )
                {
                    // safety clear ??
                }
                else if ( target_point.x > world.self().pos().x + 5.0 // forward than me
                          && std::fabs( target_point.y - world.self().pos().y ) < 20.0
                          ) // out side of me
                {
                    // safety area
                }
                else if ( target_point.x > world.ourDefenseLineX() + 20.0 )
                {
                    // safety area
                }
                else
                {
                    // dangerous
                    continue;
                }
            }
            /////////////////////////////////////////////////////////////////

#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "__ lead pass to (%.1f %.1f). first_speed= %.3f. angle = %.1f",
                          target_point.x, target_point.y,
                          first_speed, target_angle.degree() );
#endif
            // add lead pass route
            // this methid is same as through pass verification method.
            if ( verify_through_pass( world,
                                      receiver,
                                      receiver->pos(),
                                      target_point,
                                      receiver_dist,
                                      target_angle,
                                      first_speed,
                                      ball_steps_to_target ) )
            {
                out.emplace_back( LEAD,
                                                  receiver,
                                                  target_point,
                                                  first_speed,
                                                  can_kick_by_one_step( world,
                                                                        first_speed,
                                                                        target_angle ) );
#ifdef DEBUG
                dlog.addText( Logger::PASS,
                              "Pass Success lead unum=%d pos=(%.1f %.1f) angle=%.1f first_speed=%.1f",
                              receiver->unum(),
                              target_point.x, target_point.y,
                              target_angle.degree(),
                              first_speed );
#endif
            }
#ifdef DEBUG
            else
            {
                dlog.addText( Logger::PASS,
                              "Pass Failed lead unum=%d pos=(%.1f %.1f) angle=%.1f first_speed=%.1f",
                              receiver->unum(),
                              target_point.x, target_point.y,
                              target_angle.degree(),
                              first_speed );

            }
#endif
        }
    }
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
void
Body_Pass::create_through_pass(const WorldModel & world,
                               const PlayerObject * receiver,
                               std::vector<PassRoute> & out)
{
    static const double MAX_THROUGH_PASS_DIST
        = 0.9 * inertia_final_distance( ServerParam::i().ballSpeedMax(),
                                              ServerParam::i().ballDecay() );

    ////////////////////////////////////////////////////////////////
    static const
        Rect2D shrinked_pitch( Vector2D( -ServerParam::i().pitchHalfLength() + 3.0,
                                         -ServerParam::i().pitchHalfWidth() + 3.0 ),
                               Size2D( ServerParam::i().pitchLength() - 6.0,
                                       ServerParam::i().pitchWidth() - 6.0 ) );

    static const double receiver_dash_speed = 1.0;
    //static const double receiver_dash_speed = 0.85;

    static const double S_min_dash = 5.0;
    static const double S_max_dash = 25.0;
    static const double S_dash_range = S_max_dash - S_min_dash;
    static const double S_dash_inc = 4.0;
    static const int S_dash_loop = static_cast< int >( std::ceil( S_dash_range / S_dash_inc ) ) + 1;

    static const AngleDeg S_min_angle = -20.0;
    static const AngleDeg S_max_angle = 20.0;
    static const double S_angle_range = ( S_min_angle - S_max_angle ).abs();
    static const double S_angle_inc = 10.0;
    static const int S_angle_loop = static_cast< int >( std::ceil( S_angle_range / S_angle_inc ) ) + 1;

#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "Create_through_pass() to %d(%.1f %.1f)",
                  receiver->unum(),
                  receiver->pos().x, receiver->pos().y );
#endif

    /////////////////////////////////////////////////////////////////
    // check receiver position
    if ( receiver->pos().x > world.offsideLineX() - 0.5 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver is offside" );
#endif
        return;
    }
    if ( receiver->pos().x < world.self().pos().x - 10.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver is back" );
#endif
        return;
    }
    if ( std::fabs( receiver->pos().y - world.self().pos().y ) > 35.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver Y diff is big" );
#endif
        return;
    }
    if ( world.ourDefenseLineX() < 0.0
         && receiver->pos().x < world.ourDefenseLineX() - 15.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver is near to defense line" );
#endif
        return;
    }
    if ( world.offsideLineX() < 30.0
         && receiver->pos().x < world.offsideLineX() - 15.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver is far from offside line" );
#endif
        return;
    }
    if ( receiver->angleFromSelf().abs() > 135.0 )
    {
#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "__ receiver angle is too back" );
#endif
        return;
    }

    // angle loop
    AngleDeg dash_angle = S_min_angle;
    for ( int i = 0; i < S_angle_loop; ++i, dash_angle += S_angle_inc )
    {
        const Vector2D base_dash = Vector2D::polar2vector( 1.0, dash_angle );

        // dash dist loop
        double dash_dist = S_min_dash;
        for ( int j = 0; j < S_dash_loop; ++j, dash_dist += S_dash_inc )
        {
            Vector2D target_point = base_dash;
            target_point *= dash_dist;
            target_point += receiver->pos();

            if ( ! shrinked_pitch.contains( target_point ) )
            {
                // out of pitch
                continue;
            }

            if ( target_point.x < world.self().pos().x + 3.0 )
            {
                continue;
            }

            const Vector2D target_rel = target_point - world.ball().pos();
            const double target_dist = target_rel.r();
            const AngleDeg target_angle = target_rel.th();

            // check dir confidence
            int max_count = 100, ave_count = 100;
            world.dirRangeCount( target_angle, 20.0,
                                 &max_count, NULL, &ave_count );
            if ( max_count > 9 || ave_count > 3 )
            {
                continue;
            }

            if ( target_dist > MAX_THROUGH_PASS_DIST ) // dist range over
            {
                continue;
            }

            if ( target_dist < dash_dist ) // I am closer than receiver
            {
                continue;
            }

            const double dash_step = dash_dist / receiver_dash_speed;// + 5.0;//+2.0

            double end_speed = 0.81;//0.65
            double first_speed = 100.0;
            double ball_steps_to_target = 100.0;
            do
            {
                first_speed
                    = calc_first_term_geom_series_last
                    ( end_speed,
                      target_dist,
                      ServerParam::i().ballDecay() );
                //= required_first_speed(target_dist,
                //ServerParam::i().ballDecay(),
                //end_speed);
                if ( first_speed > ServerParam::i().ballSpeedMax() )
                {
                    end_speed -= 0.1;
                    continue;
                }

                ball_steps_to_target
                    = calc_length_geom_series( first_speed,
                                               target_dist,
                                               ServerParam::i().ballDecay() );
                //= move_step_for_first( target_dist,
                //first_speed,
                //ServerParam::i().ballDecay() );
                //if ( ball_steps_to_target < dash_step )
                if ( dash_step < ball_steps_to_target )
                {
                    break;
                }

                end_speed -= 0.1;
            }
            while ( end_speed > 0.5 );

            if ( first_speed > ServerParam::i().ballSpeedMax()
                 || dash_step > ball_steps_to_target )
            {
                continue;
            }

#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "__ throug pass to (%.1f %.1f). first_speed= %.3f",
                          target_point.x, target_point.y,
                          first_speed );
#endif
            if ( verify_through_pass( world,
                                      receiver,
                                      receiver->pos(),
                                      target_point, target_dist, target_angle,
                                      first_speed,
                                      ball_steps_to_target ) )
            {
                out.emplace_back( THROUGH,
                                                  receiver,
                                                  target_point,
                                                  first_speed,
                                                  can_kick_by_one_step( world,
                                                                        first_speed,
                                                                        target_angle ) );
#ifdef DEBUG

                dlog.addText( Logger::PASS,
                              "Pass Success through pass unum=%d pos=(%.1f %.1f) angle=%.1f first_speed=%.1f dash_step=%.1f ball_step=%.1f",
                              receiver->unum(),
                              target_point.x, target_point.y,
                              target_angle.degree(),
                              first_speed,
                              dash_step,
                              ball_steps_to_target );
#endif
            }
#ifdef DEBUG
            else
            {
                dlog.addText( Logger::PASS,
                              "Pass Failed through unum=%d pos=(%.1f %.1f) angle=%.1f first_speed=%.1f dash_step-%.1f ball_step=%.1f",
                              receiver->unum(),
                              target_point.x, target_point.y,
                              target_angle.degree(),
                              first_speed,
                              dash_step,
                              ball_steps_to_target );
            }
#endif
        }
    }
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
bool
Body_Pass::verify_direct_pass( const WorldModel & world,
                               const PlayerObject * /*receiver*/,
                               const Vector2D & target_point,
                               const double & target_dist,
                               const AngleDeg & target_angle,
                               const double & first_speed )
{
    static const double player_dash_speed = 1.0;

    const Vector2D first_vel = Vector2D::polar2vector( first_speed, target_angle );
    const AngleDeg minus_target_angle = -target_angle;
    const double next_speed = first_speed * ServerParam::i().ballDecay();

#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "____ verify direct pass to(%.1f %.1f). first_speed=%.3f. angle=%.1f",
                  target_point.x, target_point.y,
                  first_speed, target_angle.degree() );
#endif

    for ( const PlayerObject * o : world.opponentsFromSelf() )
    {
        if ( o->posCount() > 10 ) continue;
        if ( o->isGhost() && o->posCount() >= 4 ) continue;

        const double virtual_dash
            = player_dash_speed * 0.8 * std::min( 5, o->posCount() );

//         if ( o->pos().dist( target_point ) - virtual_dash > target_dist + 2.0 )
//         {
// #ifdef DEBUG
//             dlog.addText( Logger::PASS,
//                           "______ opp%d(%.1f %.1f) is too far. not calculated.",
//                           o->unum(),
//                           o->pos().x, o->pos().y );
// #endif
//             continue;
//         }


        if ( ( o->angleFromSelf() - target_angle ).abs() > 100.0 )
        {
// #ifdef DEBUG
//             dlog.addText( Logger::PASS,
//                           "______ opp%d(%.1f %.1f) is back of target dir. not calculated.",
//                           o->unum(),
//                           o->pos().x, o->pos().y );
//#endif
            continue;
        }

        if ( o->pos().dist2( target_point ) < 3.0 * 3.0 )
        {
#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "______ opp%d(%.1f %.1f) is already on target point(%.1f %.1f).",
                          o->unum(),
                          o->pos().x, o->pos().y,
                          target_point.x, target_point.y );
#endif
            return false;
        }

        Vector2D ball_to_opp = o->pos();
        ball_to_opp -= world.ball().pos();
        ball_to_opp -= first_vel;
        ball_to_opp.rotate( minus_target_angle );

        if ( 0.0 < ball_to_opp.x && ball_to_opp.x < target_dist )
        {
            double opp2line_dist = ball_to_opp.absY();
            opp2line_dist -= virtual_dash;
            opp2line_dist -= ServerParam::i().defaultKickableArea();
            opp2line_dist -= 0.1;

            if ( opp2line_dist < 0.0 )
            {
#ifdef DEBUG
                dlog.addText( Logger::PASS,
                              "______ opp%d(%.1f %.1f) can reach pass line. rejected. vdash=%.1f",
                              o->unum(),
                              o->pos().x, o->pos().y,
                              virtual_dash );
#endif
                return false;
            }

            const double ball_steps_to_project
                = calc_length_geom_series( next_speed,
                                           ball_to_opp.x,
                                           ServerParam::i().ballDecay() );
            //= move_step_for_first(ball_to_opp.x,
            //next_speed,
            //ServerParam::i().ballDecay());
            if ( ball_steps_to_project < 0.0
                 || opp2line_dist / player_dash_speed < ball_steps_to_project )
            {
#ifdef DEBUG
                dlog.addText( Logger::PASS,
                              "______ opp%d(%.1f %.1f) can reach pass line."
                              " ball reach step to project= %.1f",
                              o->unum(),
                              o->pos().x, o->pos().y,
                              ball_steps_to_project );
#endif
                return false;
            }
#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "______ opp%d(%.1f %.1f) cannot intercept.",
                          o->unum(),
                          o->pos().x, o->pos().y );
#endif
        }
    }

#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "__ Success!" );
#endif
    return true;
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
bool
Body_Pass::verify_through_pass( const WorldModel & world,
                                const PlayerObject * /*receiver*/,
                                const Vector2D & receiver_pos,
                                const Vector2D & target_point,
                                const double & target_dist,
                                const AngleDeg & target_angle,
                                const double & first_speed,
                                const double & /*reach_step*/ )
{
    static const double player_dash_speed = 1.0;

    const Vector2D first_vel = Vector2D::polar2vector( first_speed, target_angle );
    const AngleDeg minus_target_angle = -target_angle;
    const double next_speed = first_speed * ServerParam::i().ballDecay();

    const double receiver_to_target = receiver_pos.dist( target_point );

    bool very_aggressive = false;
    if ( target_point.x > 28.0
         && target_point.x > world.ball().pos().x + 20.0 )
    {
        very_aggressive = true;
    }
    else if ( target_point.x > world.offsideLineX() + 15.0
              && target_point.x > world.ball().pos().x + 15.0 )
    {
        very_aggressive = true;
    }
    else if ( target_point.x > 38.0
              && target_point.x > world.offsideLineX()
              && target_point.absY() < 14.0 )
    {
        very_aggressive = true;
    }

    for ( const PlayerObject * o : world.opponentsFromSelf() )
    {
        if ( o->posCount() > 10 ) continue;

        if ( o->goalie() )
        {
            if ( target_point.absY() > ServerParam::i().penaltyAreaWidth() - 3.0
                 || target_point.x < ServerParam::i().theirPenaltyAreaLineX() + 2.0 )
            {
                continue;
            }
        }

        const double virtual_dash
            = player_dash_speed * std::min( 2, o->posCount() );
        const double opp_to_target = o->pos().dist( target_point );
        double dist_rate = ( very_aggressive ? 0.8 : 1.0 );
        double dist_buf = ( very_aggressive ? 0.5 : 2.0 );

        if ( opp_to_target - virtual_dash < receiver_to_target * dist_rate + dist_buf )
        {
#ifdef DEBUG
            dlog.addText( Logger::PASS,
                          "______ opp%d(%.1f %.1f) is closer than receiver.",
                          o->unum(),
                          o->pos().x, o->pos().y );
#endif
            return false;
        }

        Vector2D ball_to_opp = o->pos();
        ball_to_opp -= world.ball().pos();
        ball_to_opp -= first_vel;
        ball_to_opp.rotate(minus_target_angle);

        if ( 0.0 < ball_to_opp.x && ball_to_opp.x < target_dist )
        {
            double opp2line_dist = ball_to_opp.absY();
            opp2line_dist -= virtual_dash;
            opp2line_dist -= ServerParam::i().defaultKickableArea();
            opp2line_dist -= 0.1;
            if ( opp2line_dist < 0.0 )
            {
#ifdef DEBUG
                dlog.addText( Logger::PASS,
                              "______ opp%d(%.1f %.1f) is already on pass line.",
                              o->unum(),
                              o->pos().x, o->pos().y );
#endif
                return false;
            }

            const double ball_steps_to_project
                = calc_length_geom_series( next_speed,
                                                 ball_to_opp.x,
                                                 ServerParam::i().ballDecay() );
            //= move_step_for_first( ball_to_opp.x,
            //next_speed,
            //ServerParam::i().ballDecay() );
            if ( ball_steps_to_project < 0.0
                 || opp2line_dist / player_dash_speed < ball_steps_to_project )
            {
#ifdef DEBUG
                dlog.addText( Logger::PASS,
                              "______ opp%d(%.1f %.1f) can reach pass line."
                              " ball reach step to project= %.1f",
                              o->unum(),
                              o->pos().x, o->pos().y,
                              ball_steps_to_project );
#endif
                return false;
            }

        }
    }

#ifdef DEBUG
    dlog.addText( Logger::PASS,
                  "__ Success!" );
#endif
    return true;
}

/*-------------------------------------------------------------------*/
/*!
  static method
*/
void
Body_Pass::evaluate_routes( const WorldModel & world )
{
    const AngleDeg min_angle = -45.0;
    const AngleDeg max_angle = 45.0;

    for ( std::vector< PassRoute >::iterator it = S_cached_pass_route.begin(), end = S_cached_pass_route.end();
          it != end;
          ++it )
    {
        //-----------------------------------------------------------
        double opp_dist_rate = 1.0;
        {
            double opp_dist = 100.0;
            world.getOpponentNearestTo( it->receive_point_, 20, &opp_dist );
            opp_dist_rate = std::pow( 0.99, std::max( 0.0, 30.0 - opp_dist ) );
        }
        //-----------------------------------------------------------
        double x_diff_rate = 1.0;
        {
            double x_diff = it->receive_point_.x - world.self().pos().x;
            x_diff_rate = std::pow( 0.98, std::max( 0.0, 30.0 - x_diff ) );
        }
        //-----------------------------------------------------------
        double receiver_move_rate = 1.0;
        //= std::pow( 0.995,
        //it->receiver_->pos().dist( it->receive_point_ ) );
        //-----------------------------------------------------------
        double pos_conf_rate = std::pow( 0.98, it->receiver_->posCount() );
        //-----------------------------------------------------------
        double dir_conf_rate = 1.0;
        {
            AngleDeg pass_angle = ( it->receive_point_ - world.self().pos() ).th();
            int max_count = 0;
            world.dirRangeCount( pass_angle, 20.0, &max_count, NULL, NULL );

            dir_conf_rate = std::pow( 0.95, max_count );
        }
        //-----------------------------------------------------------
        double offense_rate
            = std::pow( 0.98,
                        std::max( 5.0, std::fabs( it->receive_point_.y
                                                  - world.ball().pos().y ) ) );
        //-----------------------------------------------------------
        const Sector2D sector( it->receive_point_,
                               0.0, 10.0,
                               min_angle, max_angle );

        // opponent check with goalie
        double front_space_rate = 1.0;
        if ( world.existOpponentIn( sector, 10, true ) )
        {
            front_space_rate = 0.95;
        }

        //-----------------------------------------------------------
        it->score_ = 1000.0;
        it->score_ *= opp_dist_rate;
        it->score_ *= x_diff_rate;
        it->score_ *= receiver_move_rate;
        it->score_ *= pos_conf_rate;
        it->score_ *= dir_conf_rate;
        it->score_ *= offense_rate;
        it->score_ *= front_space_rate;

        if ( it->one_step_kick_ )
        {
            it->score_ *= 1.2;
        }

#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "PASS Score %6.2f -- to%d(%.1f %.1f) recv_pos(%.1f %.1f) type %d "
                      " speed=%.2f",
                      it->score_,
                      it->receiver_->unum(),
                      it->receiver_->pos().x, it->receiver_->pos().y,
                      it->receive_point_.x, it->receive_point_.y,
                      it->type_,
                      it->first_speed_ );
        dlog.addText( Logger::PASS,
                      "____ opp_dist=%.2f x_diff=%.2f pos_conf=%.2f"
                      " dir_conf=%.2f space=%.1f %s",
                      opp_dist_rate, x_diff_rate, pos_conf_rate,
                      dir_conf_rate, front_space_rate,
                      ( it->one_step_kick_ ? "one_step" : "" ) );
#endif
    }

}


void
Body_Pass::evaluate_routes_on( const WorldModel & world, std::vector< PassRoute > & routes)
{
    const AngleDeg min_angle = -45.0;
    const AngleDeg max_angle = 45.0;

    for ( auto it = routes.begin(), end = routes.end();
          it != end;
          ++it )
    {
        //-----------------------------------------------------------
        double opp_dist_rate = 1.0;
        {
            double opp_dist = 100.0;
            world.getOpponentNearestTo( it->receive_point_, 20, &opp_dist );
            opp_dist_rate = std::pow( 0.99, std::max( 0.0, 30.0 - opp_dist ) );
        }
        //-----------------------------------------------------------
        double x_diff_rate = 1.0;
        {
            double x_diff = it->receive_point_.x - world.self().pos().x;
            x_diff_rate = std::pow( 0.98, std::max( 0.0, 30.0 - x_diff ) );
        }
        //-----------------------------------------------------------
        double receiver_move_rate = 1.0;
        //= std::pow( 0.995,
        //it->receiver_->pos().dist( it->receive_point_ ) );
        //-----------------------------------------------------------
        double pos_conf_rate = std::pow( 0.98, it->receiver_->posCount() );
        //-----------------------------------------------------------
        double dir_conf_rate = 1.0;
        {
            AngleDeg pass_angle = ( it->receive_point_ - world.self().pos() ).th();
            int max_count = 0;
            world.dirRangeCount( pass_angle, 20.0, &max_count, NULL, NULL );

            dir_conf_rate = std::pow( 0.95, max_count );
        }
        //-----------------------------------------------------------
        double offense_rate
            = std::pow( 0.98,
                        std::max( 5.0, std::fabs( it->receive_point_.y
                                                  - world.ball().pos().y ) ) );
        //-----------------------------------------------------------
        const Sector2D sector( it->receive_point_,
                               0.0, 10.0,
                               min_angle, max_angle );

        // opponent check with goalie
        double front_space_rate = 1.0;
        if ( world.existOpponentIn( sector, 10, true ) )
        {
            front_space_rate = 0.95;
        }

        //-----------------------------------------------------------
        it->score_ = 1000.0;
        it->score_ *= opp_dist_rate;
        it->score_ *= x_diff_rate;
        it->score_ *= receiver_move_rate;
        it->score_ *= pos_conf_rate;
        it->score_ *= dir_conf_rate;
        it->score_ *= offense_rate;
        it->score_ *= front_space_rate;

        if (! it->one_step_kick_ )
        {
            it->score_ *= 0.0;
        }

#ifdef DEBUG
        dlog.addText( Logger::PASS,
                      "PASS Score %6.2f -- to%d(%.1f %.1f) recv_pos(%.1f %.1f) type %d "
                      " speed=%.2f",
                      it->score_,
                      it->receiver_->unum(),
                      it->receiver_->pos().x, it->receiver_->pos().y,
                      it->receive_point_.x, it->receive_point_.y,
                      it->type_,
                      it->first_speed_ );
        dlog.addText( Logger::PASS,
                      "____ opp_dist=%.2f x_diff=%.2f pos_conf=%.2f"
                      " dir_conf=%.2f space=%.1f %s",
                      opp_dist_rate, x_diff_rate, pos_conf_rate,
                      dir_conf_rate, front_space_rate,
                      ( it->one_step_kick_ ? "one_step" : "" ) );
#endif
    }

}
/*-------------------------------------------------------------------*/
/*!
  static method
*/
bool
Body_Pass::can_kick_by_one_step( const WorldModel & world,
                                 const double & first_speed,
                                 const AngleDeg & target_angle )
{
    Vector2D required_accel = Vector2D::polar2vector( first_speed, target_angle );
    required_accel -= world.ball().vel();
    return ( world.self().kickRate() * ServerParam::i().maxPower()
             > required_accel.r() );
}

bool Body_Pass::DirectPass(PlayerAgent* agent)
{
    if (!agent->world().self().isKickable()) return false;

    Vector2D tp; double fs = 0.0; int recv = Unum_Unknown;
    if (!get_best_direct(agent->world(), &tp, &fs, &recv)) return false;

    agent->debugClient().addMessage("pass:direct-1step");
    agent->debugClient().setTarget(tp);

    // 一脚能打到的最大“有效初速”上限（与你原 fallback 一致）
    fs = std::min(fs, agent->world().self().kickRate() * ServerParam::i().maxPower());

    // 直接一脚踢。如果一脚达不到目标初速，execute 会返回 false
    if (!Body_KickOneStep(tp, fs).execute(agent)) {
        return false;
    }

    if (agent->config().useCommunication() && recv != Unum_Unknown) {
        Vector2D buf = tp - agent->world().self().pos(); buf.setLength(1.0);
        agent->addSayMessage(new PassMessage(recv,
                                             tp + buf,
                                             agent->effector().queuedNextBallPos(),
                                             agent->effector().queuedNextBallVel()));
    }
    return true;
}


bool Body_Pass::LeadPass(PlayerAgent* agent)
{
    if (!agent->world().self().isKickable()) return false;

    Vector2D tp; double fs = 0.0; int recv = Unum_Unknown;
    if (!get_best_lead(agent->world(), &tp, &fs, &recv)) return false;

    agent->debugClient().addMessage("pass:lead-1step");
    agent->debugClient().setTarget(tp);
    // 一脚能打到的最大“有效初速”上限（与你原 fallback 一致）
    fs = std::min(fs, agent->world().self().kickRate() * ServerParam::i().maxPower());

    // 直接一脚踢。如果一脚达不到目标初速，execute 会返回 false
    if (!Body_KickOneStep(tp, fs).execute(agent)) {
        return false;
    }

    if (agent->config().useCommunication() && recv != Unum_Unknown) {
        Vector2D buf = tp - agent->world().self().pos(); buf.setLength(1.0);
        agent->addSayMessage(new PassMessage(recv,
                                             tp + buf,
                                             agent->effector().queuedNextBallPos(),
                                             agent->effector().queuedNextBallVel()));
    }
    return true;
}

bool Body_Pass::ThroughPass(PlayerAgent* agent)
{
    if (!agent->world().self().isKickable()) return false;

    Vector2D tp; double fs = 0.0; int recv = Unum_Unknown;
    if (!get_best_through(agent->world(), &tp, &fs, &recv)) return false;

    agent->debugClient().addMessage("pass:through-1step");
    agent->debugClient().setTarget(tp);
    // 一脚能打到的最大“有效初速”上限（与你原 fallback 一致）
    fs = std::min(fs, agent->world().self().kickRate() * ServerParam::i().maxPower());

    // 直接一脚踢。如果一脚达不到目标初速，execute 会返回 false
    if (!Body_KickOneStep(tp, fs).execute(agent)) {
        return false;
    }

    if (agent->config().useCommunication() && recv != Unum_Unknown) {
        Vector2D buf = tp - agent->world().self().pos(); buf.setLength(1.0);
        agent->addSayMessage(new PassMessage(recv,
                                             tp + buf,
                                             agent->effector().queuedNextBallPos(),
                                             agent->effector().queuedNextBallVel()));
    }
    return true;
}
bool Body_Pass::isDirectPassExecutable(PlayerAgent* agent)
{
    if (!agent->world().self().isKickable()) return false;

    Vector2D tp; double fs = 0.0; int recv = Unum_Unknown;
    if (!get_best_direct(agent->world(), &tp, &fs, &recv)) return false;

    // 一脚最大可实现初速上限，避免给出不可达速度
    const double kick_cap = agent->world().self().kickRate() * ServerParam::i().maxPower();
    fs = std::max(0.01, std::min(fs, kick_cap));

    return Body_KickOneStep(tp, fs).isExecutable(agent);
}

bool Body_Pass::isLeadPassExecutable(PlayerAgent* agent)
{
    if (!agent->world().self().isKickable()) return false;

    Vector2D tp; double fs = 0.0; int recv = Unum_Unknown;
    if (!get_best_lead(agent->world(), &tp, &fs, &recv)) return false;

    const double kick_cap = agent->world().self().kickRate() * ServerParam::i().maxPower();
    fs = std::max(0.01, std::min(fs, kick_cap));

    return Body_KickOneStep(tp, fs).isExecutable(agent);
}

bool Body_Pass::isThroughPassExecutable(PlayerAgent* agent)
{
    if (!agent->world().self().isKickable()) return false;

    Vector2D tp; double fs = 0.0; int recv = Unum_Unknown;
    if (!get_best_through(agent->world(), &tp, &fs, &recv)) return false;

    const double kick_cap = agent->world().self().kickRate() * ServerParam::i().maxPower();
    fs = std::max(0.01, std::min(fs, kick_cap));

    return Body_KickOneStep(tp, fs).isExecutable(agent);
}
