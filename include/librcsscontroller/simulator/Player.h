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

#ifndef LIBRCSSCONTROLLER_PLAYER_H_
#define LIBRCSSCONTROLLER_PLAYER_H_

namespace librcsscontroller
{
    /**
     *  The Player struct is used by SimulatorConnection to represent a virtual
     *  NAO that is present on the simulator, being controlled by an agent.
     */
    struct Player
    {
        /**
         *  Constructor
         */ 
        Player()
        { }

        /**
         *  Constructor
         *
         *  @param num The player number of the player.
         *  @param t The team of the player. 0 indicates left team, 1 indicates
         *  right team.
         */
        Player(int num, bool t)
            : number{num}, team{t}
        { }

        /**< Player number */
        int number;         
        
        /**< Player team. 0 indicates left team, 1 indicates right team. */
        std::string team;   
    };
}


#endif // LIBRCSSCONTROLLER_PLAYER_H_