/*
 *  librcsscontroller
 *  A library for controlling rcssserver3d simulations.
 *  Copyright (C) 2017 Jeremy Collette.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBRCSSCONTROLLER_SIMULATORUPDATE_H_
#define LIBRCSSCONTROLLER_SIMULATORUPDATE_H_

#include "comms/MessageParser.h"
#include "Player.h"
#include "PlayMode.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace librcsscontroller 
{
    /**
     *  The SimulatorUpdate class stores update information received by 
     *  SimulatorConnection instances that interface with rcssserver3d. It 
     *  contains information about the state of the simulator.
     */
    struct SimulatorUpdate
    {
        /**
         *  Constructor
         */
        SimulatorUpdate();

        /** 
         *  Populates a SimulatorUpdate instance with the information contained
         *  in a parsable string sent over the network.
         */
        bool FromMessage(MessageParser msg);

        /**< Holds informaiton regarding the current players on the server */
        std::vector<Player> players;    
        
        /**< Indicates the current PlayMode */
        PlayMode play_mode;
    };
}

#endif // LIBRCSSCONTROLLER_SIMULATORUPDATE_H_