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

namespace librcsscontroller 
{

    template <typename TFromAgent, typename TToAgent>
    AgentServer<TFromAgent, TToAgent>::AgentServer()
        : sockfd_{0}, log_(Logger::GetInstance())
    { }

    template <typename TFromAgent, typename TToAgent>
    bool AgentServer<TFromAgent, TToAgent>::Init(const int port)
    {
        struct sockaddr_in serv_addr;

        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0)
        { 
            log_(LogLevel::ERROR) << "Error opening server socket!\n";
            return false;
        }

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1;

        if (setsockopt (sockfd_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                         sizeof(timeout)) < 0)
        {
            log_(LogLevel::ERROR) << "setsockopt failed!\n";
            return false;
        }
     
        if (setsockopt (sockfd_, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                    sizeof(timeout)) < 0)
        {
            log_(LogLevel::ERROR) << "setsockopt failed!\n";
            return false;
        }

        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port);

        if (bind(sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        {
            log_(LogLevel::ERROR) << "Error binding server socket!\n";
            return false;
        }

        listen(sockfd_, 22);
        return true;
    }

    template <typename TFromAgent, typename TToAgent>
    void AgentServer<TFromAgent, TToAgent>::Tick()
    {
        int cli = HandleIncomingClient();
        if (cli > 0)
        {        
            SocketStream ss;
            ss.Init(cli);

            EndpointConnection ec;
            if (!ec.Init(ss))
            {
                log_(LogLevel::ERROR) << "Error accepting client " << cli << "!\n";
            }

            AgentConnection<TFromAgent, TToAgent> ac;
            if (!ac.Init(ec))
            {
                log_(LogLevel::ERROR) << "Error accepting client " << cli << "!\n";
            }

            log_(LogLevel::INFO) << "Accepted client: " << cli << "\n";
            clients_.push_back(ac);
        }

        for (auto itr = clients_.begin(); itr != clients_.end(); )
        {
            if (!ReceiveClientUpdate(*itr))
            {
                itr = clients_.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }

    template <typename TFromAgent, typename TToAgent>
    std::vector<Agent> AgentServer<TFromAgent, TToAgent>::GetAgents() const
    {
        std::vector<Agent> agents;

        for (const auto& c : clients_)
        {
            agents.push_back(Agent{c.GetId()});
        }
        return agents;
    }

    template <typename TFromAgent, typename TToAgent>
    bool AgentServer<TFromAgent, TToAgent>::Send(Agent& to_agent, const TToAgent& to_send)
    {
        Client c;
        if (!GetClient(to_agent.id, &c))
        {
            return false;
        }
        if (!c.Send(to_send))
        {
            Disconnect(to_agent);
            return false;
        }
        return true;
    }
        
    template <typename TFromAgent, typename TToAgent>        
    bool AgentServer<TFromAgent, TToAgent>::Send(const TToAgent& to_send)
    {
        bool result = true;
        for (auto& a : GetAgents())
        {
            if (!Send(a, to_send))
            {
                result = false;
            }
        }
        return result;
    }

    template <typename TFromAgent, typename TToAgent>
    bool AgentServer<TFromAgent, TToAgent>::GetLastUpdate(const Agent& from_agent, TFromAgent* out)
    {
        if (received_.find(from_agent.id) == received_.end())
        {
            return false;
        }
        *out = received_[from_agent.id];
        return true;
    }

    template <typename TFromAgent, typename TToAgent>
    std::vector<TFromAgent> AgentServer<TFromAgent, TToAgent>::GetLastUpdates()
    {
        std::vector<TFromAgent> to_return;
        for (auto& a : GetAgents())
        {
            TFromAgent update;
            if (GetLastUpdate(a, &update))
            {
                to_return.push_back(update);
            }
        }
        return to_return;
    }


    template <typename TFromAgent, typename TToAgent>
    bool AgentServer<TFromAgent, TToAgent>::Disconnect(Agent& to_disconnect)
    {
        Client c;
        if (!GetClient(to_disconnect.id, &c))
        {
            return false;
        }
        if (!c.Close())
        {
            return false;
        }
        clients_.erase(std::find_if(clients_.begin(), clients_.end(),
            [c](const Client& a){ return a.GetId() == c.GetId(); }));
        return true;        
    }

    template <typename TFromAgent, typename TToAgent>
    int AgentServer<TFromAgent, TToAgent>::HandleIncomingClient()
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sockfd_, &rfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1;
        if (!select(sockfd_+1, &rfds, NULL, NULL, &timeout))
        {
            return 0;
        }

        socklen_t clilen;
        struct sockaddr_in serv_addr, cli_addr;

        clilen = sizeof(cli_addr);
        int newsockfd = accept(sockfd_, 
            (struct sockaddr *) &cli_addr, 
            &clilen);
     
        timeout.tv_sec = 0;
        timeout.tv_usec = 1;
        if (setsockopt (newsockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                         sizeof(timeout)) < 0)
        {
            log_(LogLevel::ERROR) << "client setsockopt failed!\n";
            return 0;
        }
     
        if (setsockopt (newsockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                    sizeof(timeout)) < 0)
        {
            log_(LogLevel::ERROR) << "client setsockopt failed!\n";
            return 0;
        }

        if (newsockfd < 0) 
        {
            log_(LogLevel::ERROR) << "Error accepting client!\n";
            return 0;
        }
        return newsockfd;      
    }

    template <typename TFromAgent, typename TToAgent>
    bool AgentServer<TFromAgent, TToAgent>::GetClient(const int id, Client* out)
    {
        auto itr = std::find_if(clients_.begin(), clients_.end(), 
            [id](const Client& c){return c.GetId() == id;});

        if (itr == clients_.end())
        {
            return false;
        }
        *out = *itr;        
        return true;
    }    

    template <typename TFromAgent, typename TToAgent>
    bool AgentServer<TFromAgent, TToAgent>::ReceiveClientUpdate(Client& from)
    {
        TFromAgent update;
        if (!from.Receive(&update))
        {
            return false;
        }
        received_[from.GetId()] = update;
        return true;
    }

}