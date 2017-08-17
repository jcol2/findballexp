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

#ifndef LIBRCSSCONTROLLER_AGENTSERVER_H_
#define LIBRCSSCONTROLLER_AGENTSERVER_H_

#include "Agent.h"
#include "AgentConnection.h"
#include "comms/SocketStream.h"
#include "FromAgent.h"
#include "utils/Logger.h"

#include <iostream>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace librcsscontroller 
{
    /**
     *  The AgentServer class provides an interface for agent communication over
     *  TCP. This is useful for designing a simulator controller that sits 
     *  between agents and rcssserver3d.
     *
     *  @tparam TFromAgent The class that is expected to be sent by agents to 
     *  the AgentServer. It should inherit from the FromAgent class.
     *  @tparam TToAgent The class that will be sent from the AgentServer to 
     *  agents. It should inherit from the ToAgent class.
     */
    template <typename TFromAgent, typename TToAgent>
    class AgentServer 
    {
    public:
        /*
         *  A typedef for client connections of type <TFromAgent, TToAgent>.
         */
        typedef AgentConnection<TFromAgent, TToAgent> Client; 

        /**
         *  Constructor
         */
        AgentServer();
        /**
         *  Initialises the AgentServer.
         *
         *  @param port The port ot listen for agents on.
         *  @return bool Returns 'true' for success. 'False' for
         *  failure.
         */
        bool Init(const int port);

        /**
         *  Ticks the AgentServer. Receives packets from current clients
         *  and attempts to accept new clients. Also drops inactive or
         *  disconnected clients.
         */
        void Tick();

        /**
         *  Returns a vector of currently connected agents.
         *
         *  @return std::vector<> Vector of agents.
         */
        std::vector<Agent> GetAgents() const;

        /**
         *  Sends a TToAgent update to the specified agent.
         *
         *  @param to_agent The Agent to send to.
         *  @param to_send The TToAgent message to send.         
         *  @return bool True indicates success. False indicates failure.
         */
         bool Send(Agent& to_agent, const TToAgent& to_send);
        
        /**
         *  Sends a TToAgent update to all agents.
         *
         *  @param to_send The TToAgent message to send.
         *  @return bool True indicates success. False indicates failure.
         */
         bool Send(const TToAgent& to_send);

        /**
         *  Returns the last update received from a specific agent.
         *
         *  @param from_agent The agent to return the last update from.
         *  @param out[out] The TFromAgent instance to write the update to.
         *  @return bool Returns true if successful. False otherwise.
         */
         bool GetLastUpdate(const Agent& from_agent, TFromAgent* out);

        /**
         *  Returns the last update received from all agents.
         *
         *  @return std::vector<TFromAgent> A vector of the last received
         *  updates. If no updates are available, the vector is empty.
         */
         std::vector<TFromAgent> GetLastUpdates();

        /**
         *  Disconnects the specified agent from the AgentServer.
         *
         *  @param to_disconnect The agent to disconnect.
         *  @return bool Returns true if successful. False otherwise.
         */
         bool Disconnect(Agent& to_disconnect);

    private:
        /**
         * Handles new clients connecting.
         * 
         * @return int On success, returns the socket file descriptor of the 
         * connected client. On failure, returns 0.
         */
        int HandleIncomingClient();     

        /**
         *  Gets the client with the corresponding id.
         *
         *  @param id The clients ID.
         *  @param out[out] The found client. nullptr if not found.
         *  @return bool Returns true if a client was found. False otherwise.
         */
        bool GetClient(const int id, Client* out);

        /**
         *  Gets the latest update from the specified client.
         *
         *  @param from The client to receive an update from.
         *  @return bool Returns true if the updates were received successfully.
         *  False otherwise.
         */
        bool ReceiveClientUpdate(Client& from);


        int sockfd_;                    /**< The socket descriptor for the listening socket */
        Logger& log_;                   /**< The logging instance in use */
        std::vector<Client> clients_;   /**< The connected clients */
        std::unordered_map<int, TFromAgent> received_; /**< Holds received updates */

    };
}

#include "AgentServer.tcc"

#endif // LIBRCSSCONTROLLER_AGENTSERVER_H_