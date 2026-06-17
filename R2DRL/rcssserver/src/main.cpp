// -*-c++-*-

/***************************************************************************
                                  main.cpp
                           Main for rcssserver
                             -------------------
    begin                : 1996
    copyright            : (C) 1996-2000 Electrotechnical Laboratory.
                           Itsuki Noda, Yasuo Kuniyoshi and Hitoshi Matsubara.
                           (C) 2001- by The RoboCup Soccer Server
                           Maintenance Group.
    email                : sserver-admin@lists.sourceforge.net
***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU LGPL as published by the Free Software  *
 *   Foundation; either version 3 of the License, or (at your option) any  *
 *   later version.                                                        *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "stadium.h"
#include "serverparam.h"
#include "version.h"

#include "stdtimer.h"
#include "synctimer.h"

#include <memory>
#include <iostream>
#include <locale>
#include <cmath>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <unistd.h> // getpid()

namespace {

Stadium Std;

// SA_SIGINFO handler: can print sender pid/uid
void
sigHandleInfo( int sig, siginfo_t * info, void * )
{
    std::cerr << "[rcssserver] sigHandle called. sig=" << sig
              << " self_pid=" << ::getpid();

    if ( info )
    {
        std::cerr << " sender_pid=" << info->si_pid
                  << " sender_uid=" << info->si_uid
                  << " si_code=" << info->si_code;
    }

    std::cerr << std::endl;

    Std.finalize( "Server Killed. Exiting..." );

    std::cerr << "[rcssserver] sigHandle finished finalize(). sig=" << sig
              << " self_pid=" << ::getpid() << std::endl;
}

} // anonymous namespace

int
main( int argc, char *argv[] )
{
    std::locale::global( std::locale::classic() );

    std::cout << PACKAGE << "-" << VERSION << "\n\n"
              << Copyright << std::endl;

    if ( ! ServerParam::init( argc, argv ) )
    {
        return 1;
    }

    struct sigaction sig_action;
    std::memset( &sig_action, 0, sizeof( sig_action ) );
    sig_action.sa_sigaction = &sigHandleInfo;
    sig_action.sa_flags = SA_SIGINFO;

    if ( sigaction( SIGINT, &sig_action, nullptr ) != 0
         || sigaction( SIGTERM, &sig_action, nullptr ) != 0
         || sigaction( SIGHUP, &sig_action, nullptr ) != 0 )
    {
        std::cerr << __FILE__ << ": " << __LINE__
                  << ": could not set signal handler: "
                  << strerror( errno ) << std::endl;
        ServerParam::instance().clear();
        return 1;
    }

    if ( ! Std.init() )
    {
        ServerParam::instance().clear();
        return 1;
    }

    std::shared_ptr< Timer > timer;
    if ( ServerParam::instance().synchMode() )
    {
        timer = std::shared_ptr< Timer >( new SyncTimer( Std ) );
    }
    else
    {
        timer = std::shared_ptr< Timer >( new StandardTimer( Std ) );
    }

    std::cout << "\nHit CTRL-C to exit\n";

    timer->run();

    ServerParam::instance().clear();

    return 0;
}
