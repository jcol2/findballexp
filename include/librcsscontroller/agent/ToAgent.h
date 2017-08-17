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

#ifndef LIBRCSSCONTROLLER_TOAGENT_H_
#define LIBRCSSCONTROLLER_TOAGENT_H_

#include "comms/MessageParser.h"

namespace librcsscontroller 
{
    /**
     *  The ToAgent struct is an abstract base struct that must be inherited
     *  by types that are to be sent from AgentServer instances to external
     *  agents.
     */
    struct ToAgent        
    {
        /**
         *  Converts the inherited type to a string to be transmitted via TCP
         *  to an agent. Essentially the reverse of the FromMessage() 
         *  function.
         *
         *  @return std::string The inherited type represented as a consumable
         *  string.
         */
        virtual std::string ToMessage() const = 0;

        /**
         *  Populates the inherited type using a string representation, most
         *  likely transmitted over TCP from an agent. The reverse of the 
         *  ToMessage() function.
         *
         *  @param msg The received message to populate the struct with.
         *  @return bool True if the struct was successfully populated,
         *  false otherwise.
         */
        virtual bool FromMessage(const std::string& received) = 0;
    };

}

#endif // LIBRCSSCONTROLLER_TOAGENT_H_