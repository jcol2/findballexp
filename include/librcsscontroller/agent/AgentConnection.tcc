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
    AgentConnection<TFromAgent, TToAgent>::AgentConnection()
    {  } 

    template <typename TFromAgent, typename TToAgent>
    bool AgentConnection<TFromAgent, TToAgent>::Init(const EndpointConnection& ep)
    {
        ep_ = ep;
    }       

    template <typename TFromAgent, typename TToAgent>
    bool AgentConnection<TFromAgent, TToAgent>::Receive(TFromAgent* out)
    {
        time_t start, now;
        time(&start);
        time(&now);

        std::string recv = ep_.Receive();
        while (!recv.size() && (difftime(now, start) < RECEIVE_TIMEOUT_SEC))
        {
            recv = ep_.Receive();
            usleep(10);
            time(&now);
        }
        
        if (!recv.size())
        {
            std::cerr << "Client timed out!\n";
            return false;
        }
        return out->FromMessage(recv);
    }

    template <typename TFromAgent, typename TToAgent>
    bool AgentConnection<TFromAgent, TToAgent>::Send(const TToAgent& update)
    {
        return ep_.Send(update.ToMessage());
    }

    template <typename TFromAgent, typename TToAgent>
    bool AgentConnection<TFromAgent, TToAgent>::Close()
    {
        return ep_.Close();
    }

    template <typename TFromAgent, typename TToAgent>
    int AgentConnection<TFromAgent, TToAgent>::GetId() const
    {
        return ep_.GetId();
    }
}