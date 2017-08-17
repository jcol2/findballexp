#ifndef FINDBALLEXP_FINDBALLEXPERIMENT_H_
#define FINDBALLEXP_FINDBALLEXPERIMENT_H_

#include "agent/AgentServer.h"
#include "FromRunswiftAgent.h"
#include "ToRunswiftAgent.h"
#include "simulator/SimulatorConnection.h"
#include "utils/Logger.h"

#include <time.h>
#include <fstream>
#include <random>
#include <utility>

using namespace librcsscontroller;

namespace findballexp
{

    /**
     *  FindBallExperiment was used to test ball finding behaviours (developed 
     *  in simulation) for Collette's (2017) thesis.
     *
     *  The ball is placed in locations around the field, and the time it takes
     *  for agents to find the ball is recorded. Data is output in .cvs files
     *  which can be used for data mining and visualisation.
     */
    class FindBallExperiment
    {
    public:
        /**< The AgentServer used by rUNSWift agents */

        typedef AgentServer<FromRunswiftAgent, ToRunswiftAgent> RunswiftAgentServer;
        /**< Represents a point on the field */
        typedef std::pair<float, float> Point;

        /**
         *  Constructor
         * 
         *  @param simulator Connection to the simulator to use
         *  @param agent_server Agent server to communicate with agents
         *  @param start_from The test number to start from
         */
        FindBallExperiment(SimulatorConnection& simulator
                , RunswiftAgentServer& agent_server
                , const int start_from);
        
        /**
         *  Deconstructor
         */
        ~FindBallExperiment();

        /**
         *  Initialises experiment
         *
         *  @return bool True indicates success. False indicates error.
         */
        bool Init();

        /**
         *  Updates experiment
         *
         *  @return bool True indicates success. False indicates error.
         */
        bool Tick();

        /**
         *  Finishes experiment
         *
         *  @return bool True indicates success. False indicates error.
         */        
        bool Finish();

    private:
        /**< How long in seconds the robots have to find the ball before timeout */
        const static int FIND_BALL_TIMEOUT      = 300;

        /**< How many frames in a row the ball must be seen before it is considered to be found */
        const static int FIND_BALL_SEEN_FRAMES  = 5;

        /**< The maximum distance (in mm) from the ball before it is considered to be found */
        const static int FIND_BALL_MAX_DIST     = 300;

        /**< Number of unique ball positions to use */
        const static int UNIQUE_POINTS          = 10;

        /**< Number of pre-determined ball positions to use */
        const static int NUM_PREDEFINED_POINTS  = 10;

        /**< Holds pre-determined ball positions */
        const static Point PREDEFINED_POINTS[NUM_PREDEFINED_POINTS];

        /**
         * States represent the "state" of the experiment. These are used to 
         * break down tick processing in to a series of methods that handle
         * certain states.
         *
         * Definitions:
         *      NOT_STARTED         No tests have been run yet
         *      TEST_STARTING       A test is about to start
         *      TEST_STARTED        A test is running
         *      TEST_FINISHED       A test just finished      
         */
        enum State {NOT_STARTED, TEST_STARTING, TEST_STARTED, TEST_FINISHED};

        /**
         *  Called each tick during NOT_STARTED states  
         *
         *  @return bool True indicates success. False indicates error.
         */
        bool HandleNotStarted();
        
        /**
         *  Called each tick during TEST_STARTING states  
         *
         *  @return bool True indicates success. False indicates error.
         */
        bool HandleStarting();

        /**
         *  Called each tick during TEST_STARTED states  
         *
         *  @return bool True indicates success. False indicates error.
         */
        bool HandleStarted();

        /**
         *  Called each tick during TEST_FINISHED states  
         *
         *  @return bool True indicates success. False indicates error.
         */        
        bool HandleFinished();

        /**
         *  Sets the experiment state
         *
         *  @param s The new state.
         */
        void SetExperimentState(int s);

        /**
         *  Prepares a new experiment.
         *
         *  @return bool True indicates success. False indicates error.
         */
        bool PrepareExperiment();

        /**
         *  Starts a new experiment.
         *
         *  @return bool True indicates success. False indicates error.
         */  
        bool StartExperiment();    

        /**
         *  Finish an experiment.
         *
         *  @param time The time the experiment finished
         *  @param found_by A list of agents who found the ball
         *  @return bool True indicates success. False indicates error.
         */        
        bool FinishExperiment(int time, const std::vector<Agent>& found_by);

        /**
         *  Cancels the current experiment.
         *
         *  @return bool True indicates success. False indicates error.
         */  
        bool CancelExperiment();

        /**
         *  Resets the timer.
         */
        void ResetTimer();

        /**
         *  Checks the game state of rcssserver3d, and changes it if 
         *  necessary.
         *
         *  @return bool True indicates success. False indicates error.
         */  
        bool CheckSimulatorGameState();

        /**
         *  Logs the estimated positions of robots for that second.
         *
         *  @return bool True indicates success. False indicates error.
         */  
        bool LogAgentPositions();

        /**
         *  Gets the experiment state
         *
         *  @return int The current state.
         */
        int GetExperimentState();

        /**
         *  Indicates if a new agent has joined since the last time this
         *  function was called.
         *
         *  @return bool True indicates new agent.
         */          
        bool HasNewAgent();

        /**
         *  Gets the current timer seconds.
         *
         *  @return int The current timer seconds.
         */
        int  GetTimerSeconds();

        /**
         *  Indicates if an agent has found the ball.
         *
         *  @param found_by[out] If not NULL, is used to store the ball 
         *  finder(s).
         *  @return bool True indicates ball found.
         */              
        bool IsBallFound(std::vector<Agent>* found_by);   

        /**
         *  Gets the starting position for a robot.
         *
         *  @param player_num The player number of the robot to position.
         *  @param x_out[out] The recommended X position.
         *  @param y_out[out] The recommended Y position.
         *  @param o_out[out] The recommended orientation.
         *  @return bool True indicates success.
         *
         */     
        bool GetStartingPosition(int player_num, float* x_out, float* y_out, float* o_out);

        /**
         *  Converts a state to a string representation.
         *
         *  @param s The state to convert.
         *  @return std::string String representation of the state s.
         */
        std::string StateToString(int s);

        Logger& log_;               /**< Used for logging */
        SimulatorConnection& simulator_;    /**< Interfaces with rcssserver3d */
        RunswiftAgentServer& agent_server_; /**< Interfaces with rUNSWift agents */
        const int start_index_;      /**< The test index to start from */

        int state_;                 /**< Current state */
        std::ofstream tests_file_;  /**< File for test result logging */
        std::ofstream pos_file_;    /**< File for agent position logging */
        std::ofstream log_file_;    /**< File for general logging */
        time_t timer_;              /**< Timer used for timing */
        int counter_;               /**< Current test number */
        int num_agents_;            /**< Current number of agents */
        bool started_;              /**< Indicates test is running */
        int start_in_;              /**< Indicates how long until test is started */
        int last_log_;              /**< Indicates how long since agent pos was logged */
        std::mt19937 mt_;           /**< Random number generator */
        std::uniform_real_distribution<> dist_; /**< Random number distribution */

    };
}

#endif // FINDBALLEXP_FINDBALLEXPERIMENT_H_