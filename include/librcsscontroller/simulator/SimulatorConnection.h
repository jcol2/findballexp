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

#ifndef LIBRCSSCONTROLLER_SIMULATORCONNECTION_H_
#define LIBRCSSCONTROLLER_SIMULATORCONNECTION_H_

#include "comms/EndpointConnection.h"
#include "PlayMode.h"
#include "SimulatorUpdate.h"


namespace librcsscontroller 
{
    /**
     *  The SimulatorConnection class provides an interface to a running
     *  rcssserver3d instance. 
     *
     *  This allows the client to read information about the state of the
     *  simulator and execute simulator commands.
     */
    class SimulatorConnection 
    {
    public:
        /**
         *  Constructor
         */
        SimulatorConnection();        

        /**
         *  Initialises the SimulatorConnection.
         *
         *  @param ep The underlying EndpointConnection to connect to the
         *  simulator.
         *  @return bool True indicates success.
         */
        bool Init(const EndpointConnection& ep);

        /**
         *  Receives the latest information from the simulator.
         *
         *  @return bool True indicates success.
         */
        bool Tick();

        /**
         *  Returns the latest information from the simulator.
         *
         *  @return SimulatorUpdate The latest information from the simulator.
         */
        SimulatorUpdate GetLastUpdate();

        /**
         *  Sends the init command to the simulator.
         *
         *  @return bool True indicates sending succeeded.
         */
        bool SendInit();

        /**
         *  Sends the kick off command to the simulator, which initiates a kick
         *  off for a specific team.
         *
         *  @param team 0 the 'left' team, 1 indicates the 'right' team.
         *  @return bool True indicates sending succeeded.
         */
        bool SendKickOffCommand(bool team);

        /**
         *  Sends the drop ball command to the simulator.
         *
         *  @return bool True indicates sending succeeded.
         */
        bool SendDropBallCommand();

        /**
         *  Sends the ball command to the simulator, which places the ball and
         *  sets velocity.
         * 
         *  @param x_pos The desired X position of the ball.
         *  @param y_pos The desired Y position of the ball.
         *  @param z_pos The desired Z position of the ball.
         *  @param x_pos The desired X velocity of the ball.
         *  @param y_pos The desired Y velocity of the ball.
         *  @param z_pos The desired Z velocity of the ball.
         *  @return bool True indicates sending succeeded.
         */
        bool SendMoveBallCommand(float x_pos, float y_pos, float z_pos, 
                                 float x_vel, float y_vel, float z_vel);

        /**
         *  Sends the ball command to the simulator, which places the ball and
         *  sets velocity.
         * 
         *  @param x_pos The desired X position of the ball.
         *  @param y_pos The desired Y position of the ball.
         *  @param z_pos The desired Z position of the ball.
         *  @return bool True indicates sending succeeded.
         */
        bool SendMoveBallCommand(float x_pos, float y_pos, float z_pos);
        
        /**
         *  Sends the play mode command to the simulator, which changes the
         *  current play mode.
         *
         *  @param play_mode The new PlayMode to set.
         *  @return bool True indicates sending succeeded.
         */        
        bool SendPlayModeCommand(PlayMode play_mode);        

        /**
         *  Sends the move player command, which changes the field position of 
         *  an agent.
         *
         *  @param to_move The agent to move.
         *  @param x_pos The X position to move to.
         *  @param y_pos The Y position to move to.
         *  @param z_pos The Z position to move to.
         *  @param orientation The orientation to face.
         *  @return bool True indicates sending succeeded.
         */
        bool SendMovePlayerCommand(const Player& to_move, float x_pos,
                                  float y_pos, float z_pos,
                                  float orientation);

        /**
         *  Sends a free kick command to the server, which awards a team a free
         *  kick.  
         *
         *  @param team 0 the 'left' team, 1 indicates the 'right' team.
         *  @return bool True indicates sending succeeded.
         */
        bool SendFreeKickCommand(bool team);

        /**
         *  Sends a direct free kick command to the server, which awards a team
         *  a direct free kick.  
         *
         *  @param team 0 the 'left' team, 1 indicates the 'right' team.
         *  @return bool True indicates sending succeeded.
         */
        bool SendDirectFreeKickCommand(bool team);

        /**
         *  Sends the kill server command, which kills the simulator.
         *
         *  @return bool True indicates sending succeeded.
         */
        bool SendKillServerCommand();

        /**
         *  Sends the time command to the simulator, which changes the time of
         *  the game.
         *
         *  @param time The new game time.
         *  @return bool True indicates sending succeeded.
         */
        bool SendSetTimeCommand(int time);

        /**
         *  Resets the game time to 0.
         *
         *  @return bool True indicates sending succeeded.
         */
        bool SendResetTimeCommand();

        /**
         *  Requests a full state from the server, which includes more
         *  information about the agents and environment being simulated.
         *
         *  @return bool True indicates sending succeeded.
         */
        bool SendFullStateRequest();

        /**
         *  Requests an acknowledgement from the server
         *
         *  @return bool True indicates sending succeeded.
         */
        bool SendGetAckCommand();

        /**
         *  Sends a set score command to the server
         *
         *  @param left_score The new score for the left team
         *  @param right_score The new score for the right team
         *  @return bool True indicates sending succeeded.
         */
        bool SendSetScoreCommand(int left_score, int right_score);

        /**
         *  Disconnects the specified player from the simulation.
         *
         *  @param to_kill The player to remove from the ismulation
         *  @return bool True indicates sending succeeded.
         */
        bool SendKillPlayerCommand(const Player& to_kill);

        /**
         *  Repositions the player to be on the side of the field. This is the
         *  command that the server uses when a robot commits a foul.
         *
         *  @param to_repos The player to reposition.
         *  @return bool True indicates sending succeeded.
         */        
        bool SendReposPlayerCommand(const Player& to_repos);

        /**
         *  Selects a player.
         *
         *  @param to_select The player to select.
         *  @return bool True indicates sending succeeded.
         */        
        bool SendSelectPlayerCommand(const Player& to_select);

    private:        
        EndpointConnection ep_;
        SimulatorUpdate last_;
    };
}

#endif // LIBRCSSCONTROLLER_SIMULATORCONNECTION_H_