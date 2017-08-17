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

#ifndef LIBRCSSCONTROLLER_AGENTCONNECTION_H_
#define LIBRCSSCONTROLLER_AGENTCONNECTION_H_

#include "comms/EndpointConnection.h"
#include "comms/MessageParser.h"
#include "FromAgent.h"
#include "ToAgent.h"

#include <time.h>
#include <unistd.h>

namespace librcsscontroller 
{
    /**
     *  The AgentConnection class facilitates a TCP connection with agents.
     *  Agents are expected to send TFromAgent instances, and will receieve
     *  TToAgent instances.
     *
     *  @tparam TFromAgent The type that is expected to be received from
     *  agents. 
     *  @tparam TToAgent The type that will be sent to agents.
     */
    template <typename TFromAgent, typename TToAgent>
    class AgentConnection 
    {
    public:
        /**
         *  Constructor
         */        
        AgentConnection();

        /**
         *  Initialises the connection with the agent.
         *
         *  @param ep A reference to the underlying 
         *  EndpointConnection with the agent.
         */
        bool Init(const EndpointConnection& ep);

        /**
         *  Attempts to receive a TFromAgent from the agent.
         *
         *  @param[out] out A pointer to a TFromAgent to store the 
         *  received TFromAgent in.
         *  @return bool True indicates success. False indicates failure.
         */
        bool Receive(TFromAgent* out);

        /**
         *  Attempts to send a TToAgent to the agent.
         *
         *  @param update The TToAgent instance to send.
         *  @return bool True indicates success. False indicates failure.
         */
        bool Send(const TToAgent& update);

        /**
         *  Attempts to close the connection with the agent.
         *
         *  @return bool True if the connection was successfully closed. False
         *  if the connection could not be closed.
         */
        bool Close();

        /** 
         *  Returns the unique agent connection id.
         *
         *  @return int The unique agent connection id.
         */
        int GetId() const;

    private:
        /**
         *  The time in seconds until an agent is considered to have timed out.
         */
        constexpr static double RECEIVE_TIMEOUT_SEC = 2.0;     

        /**
         *  The underlying EndpointConnection with the agent.
         */
        EndpointConnection ep_;
    };
}

#include "AgentConnection.tcc"

#endif // LIBRCSSCONTROLLER_AGENTCONNECTION_H_