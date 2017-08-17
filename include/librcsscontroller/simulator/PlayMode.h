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

#ifndef LIBRCSSCONTROLLER_PLAYMODE_H_
#define LIBRCSSCONTROLLER_PLAYMODE_H_

#include <string>

namespace librcsscontroller
{
    /**
     *  The PlayMode class represents simulator play modes, which describe the
     *  current state of the soccer game. 
     *  
     *  For example, BEFORE_KICK_OFF refers to the state before the soccer game 
     *  has started, when agents are connecting to the simulator.
     */
    class PlayMode
    {
    public:
        /**
         *  This enum describes the available PlayModes
         */
        enum {
            BEFORE_KICK_OFF = 0,
            KICK_OFF_LEFT,
            KICK_OFF_RIGHT,
            PLAY_ON, 
            KICK_IN_LEFT, 
            KICK_IN_RIGHT, 
            CORNER_KICK_LEFT,
            CORNER_KICK_RIGHT, 
            GOAL_KICK_LEFT, 
            GOAL_KICK_RIGHT,
            OFFSIDE_LEFT, 
            OFFSIDE_RIGHT, 
            GAME_OVER, 
            GOAL_LEFT,
            GOAL_RIGHT, 
            FREE_KICK_LEFT, 
            FREE_KICK_RIGHT,
            DIRECT_FREE_KICK_LEFT, 
            DIRECT_FREE_KICK_RIGHT,
            NUM_MODES 
        };

        /**
         *  Constructor
         *
         *  @param mode The specified PlayMode, selected from the enum declared
         *  above.
         */
        PlayMode(int mode);

        /**
         *  Constructor
         *
         *  @param mode The name of the specified PlayMode, which maps to a
         *  selection from the enum declared above.
         */
        PlayMode(const std::string& name);

        /**
         *  Int Conversion Operator
         *  
         *  Converts the PlayMode to an integer, equivalent to the integer value
         *  of one of the enum selections defined above.
         *
         *  @return int The integer representation of the PlayMode.
         */
        operator int() const;

        /**
         *  String Conversion Operator
         *  
         *  Converts the PlayMode to a string.
         *
         *  @return std::string The string representation of the PlayMode.
         */        
        operator std::string() const;

        /**
         *  Converts the PlayMode to a string.
         *
         *  @return std::string The string representation of the PlayMode.
         */         
        std::string ToString() const;

        /**
         *  Converts the PlayMode to an integer, equivalent to the integer value
         *  of one of the enum selections defined above.
         *
         *  @return int The integer representation of the PlayMode.
         */
        int ToInt() const;        

    private:
        /**< Defines a mapping from PlayMode integer to string values */
        static std::string PLAY_MODE_STRINGS[NUM_MODES];
        
        /**< The underlying integer value of the PlayMode */
        int mode_;
    };

}

#endif // LIBRCSSCONTROLLER_PLAYMODE_H_