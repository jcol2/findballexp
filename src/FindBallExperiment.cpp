#include "FindBallExperiment.h"
#include "RoboCupGameControlData.hpp"

#include "agent/AgentServer.h"
#include "simulator/SimulatorConnection.h"

#include <csignal>
#include <iostream>

namespace findballexp
{
    // Predefined ball locations
    // 1. Right middle
    // 2. Bottom right corner
    // 3. Top right corner
    // 4. Top right corner (on line)
    // 5. Right middle bottom (on line)
    // (New points)
    // 6. Top left corner (on line)
    // 7. Right goal box (in box)
    // 8. Left goal box (on box line)
    // 9. Right bottom corner (on line)
    // 10. Left boundary next to goal (on line)
    const FindBallExperiment::Point FindBallExperiment::PREDEFINED_POINTS
          [NUM_PREDEFINED_POINTS] =
            { {2.25, 0}, {4, -2.5}, {4, 2.5}, {4.5, 3},  {2.25, -3},
              {-4.5, 3}, {4.5, 0},  {-3.5, 0},{4.5, -3}, {-4.5, -1} };

    FindBallExperiment::FindBallExperiment(SimulatorConnection& simulator, 
                                        RunswiftAgentServer& agent_server,
                                        const int start_from)
        : simulator_(simulator), agent_server_(agent_server), 
        start_index_{start_from-1}, log_(Logger::GetInstance()), 
        state_{NOT_STARTED}, timer_{0}, counter_{start_from-1}, started_{false}, 
        num_agents_{0}, start_in_{0}, last_log_{0}, mt_{0}, dist_{-1, 1}
    {
        time(&timer_);
    }

    FindBallExperiment::~FindBallExperiment()
    {

    }

    bool FindBallExperiment::Init()
    {    
        // For general logging
        std::stringstream name;
        name << "logs/" << timer_ << ".log";
        log_file_.open(name.str(), std::ofstream::out);
        if (log_file_.is_open())
        {
            log_.AddStream(&log_file_, LogLevel::INFO);
            log_(LogLevel::INFO) << "Opened general log file '" 
                << name.str() << "'\n";
        }
        else
        {
            log_(LogLevel::WARNING) << "Could not open log file '" 
                << name.str() << "'\n";
        }

        // For experiment results
        name.str("");
        name << timer_ << "_test.csv";
        tests_file_.open(name.str(), std::ofstream::out);
        if (tests_file_.is_open())
        {
            log_(LogLevel::INFO) << "Opened tests output file '" << name.str() << "'\n";
            log_.AddStream(&tests_file_, LogLevel::DEBUG_5, LogLevel::DEBUG_5, nullptr);      
        }
        else
        {
            log_(LogLevel::WARNING) << "Could not open tests output file '" 
                << name.str() << "'\n";
        }
        log_(LogLevel::DEBUG_5) << "Test,BallX,BallY,Robots,Seconds,FoundBy\n";

        // For robot positions
        name.str("");
        name << timer_ << "_pos.csv";
        pos_file_.open(name.str(), std::ofstream::out);
        if (pos_file_.is_open())
        {
            log_(LogLevel::INFO) << "Opened positions output file '" 
                << name.str() << "'\n";
            log_.AddStream(&pos_file_, LogLevel::DEBUG_4, LogLevel::DEBUG_4, nullptr);        
        }
        else
        {
            log_(LogLevel::WARNING) << "Could not open positions output file '" << name.str() << "'\n";
        }
        log_(LogLevel::DEBUG_4) 
            << "Test,Seconds,Robot1Pos,Robot2Pos,Robot3Pos,Robot4Pos,Robot5Pos\n";

        log_(LogLevel::INFO) << "Waiting for agents to connect...\n";
        return true;
    }

    bool FindBallExperiment::Tick()
    {
        CheckSimulatorGameState();

        int s = GetExperimentState();
        switch(s)
        {
            case NOT_STARTED:
                HandleNotStarted();
                break;
            case TEST_STARTING:
                HandleStarting();
                break;
            case TEST_STARTED:
                HandleStarted();
                break;
            case TEST_FINISHED:
                HandleFinished();
                break;
        }

        ToRunswiftAgent response;        
        if (GetExperimentState() == TEST_STARTED 
            || GetExperimentState() == TEST_FINISHED)
        {
            response.penalty = PENALTY_NONE;
            response.game_state = STATE_PLAYING;
            LogAgentPositions();
        }
        else
        {
            response.penalty = PENALTY_SPL_ILLEGAL_BALL_CONTACT;
            response.game_state = STATE_PENALISED;            
        }
        agent_server_.Send(response); 

        return true;
    }

    bool FindBallExperiment::Finish()
    {
        CancelExperiment();

        log_(LogLevel::INFO) << "Shutting down experiment...\n";
        tests_file_.close();
        pos_file_.close();
        log_file_.close();
        return true;
    }


    bool FindBallExperiment::HandleNotStarted()
    {
        if (HasNewAgent())
        {
            log_(LogLevel::INFO) << "Starting experiment...\n";
            SetExperimentState(TEST_STARTING);
        }
        return true;
    }

    bool FindBallExperiment::HandleStarting()
    {
        if (start_in_ == 0)
        {
            simulator_.SendSelectPlayerCommand(simulator_.GetLastUpdate().players[0]);  
            SetExperimentState(TEST_STARTED);
        }
        
        if (start_in_ == 50)
        {
            PrepareExperiment();
        }
        --start_in_;
        
        return true;
    }

    bool FindBallExperiment::HandleStarted()
    {
        if (HasNewAgent())
        {
            log_(LogLevel::INFO) << "New agent detected. Restarting experiment...\n";
            CancelExperiment();
            counter_ = start_index_;
            SetExperimentState(TEST_STARTING);
            return true;
        }        
    
        std::vector<Agent> found_by;
        int time = GetTimerSeconds();
        bool finish = time > FIND_BALL_TIMEOUT || (IsBallFound(&found_by) && time > 1);
        if (finish)
        {
            FinishExperiment(time, found_by);
            SetExperimentState(TEST_FINISHED);
        }
        return true;
    }

    bool FindBallExperiment::HandleFinished()
    {
        if (start_in_ == 300)
        {
            // Move ball out of bounds so we don't detect it
            simulator_.SendMoveBallCommand(10000.0f, 10000.0f, 0.0f);
        }
        else if (start_in_ == 150)
        {
            SetExperimentState(TEST_STARTING);
        }
        --start_in_;
        
        return true;
    }

    void FindBallExperiment::SetExperimentState(int s)
    {
        log_(LogLevel::INFO) << "Changing experiment state from "
                << StateToString(state_) << " to " << StateToString(s) << "\n";
        switch (s)
        {
            case NOT_STARTED:
                start_in_ = 0;
                started_ = false;
                break;
            case TEST_STARTING:
                start_in_ = 150;
                started_ = false;
                break;
            case TEST_STARTED:                
                start_in_ = 0;
                started_ = true;
                StartExperiment();
                break;
            case TEST_FINISHED:
                start_in_ = 300;
                started_ = false;
                break;
        }
        state_ = s;

    }

    bool FindBallExperiment::PrepareExperiment()
    {
        log_(LogLevel::INFO) << "Preparing test no. " << counter_+1 << "...\n";

        SimulatorUpdate su = simulator_.GetLastUpdate();

        // Move players to starting positions
        for (int i=0; i < su.players.size(); ++i)
        {
            Player p = su.players[i];   
            float x, y, z, o;
            z = 0.4;
            if (GetStartingPosition(p.number, &x, &y, &o))
            {
                if (!simulator_.SendMovePlayerCommand(p, x, y, z, o))
                {
                    log_(LogLevel::ERROR) << "Error sending move player command!\n";
                }
            }
        }

        float ball_x, ball_y;
        int ball_index = counter_ % UNIQUE_POINTS;
        if (ball_index < NUM_PREDEFINED_POINTS)
        {
            // Predetermined ball
            ball_x = PREDEFINED_POINTS[ball_index].first;
            ball_y = PREDEFINED_POINTS[ball_index].second;            
        }
        else
        {
            // Random ball (seeded by ball index)
            mt_.seed(ball_index);
            ball_x = dist_(mt_) * 4.5;
            ball_y = dist_(mt_) * 3;
        }

        if (!simulator_.SendMoveBallCommand(ball_x, ball_y, 0, 0, 0, 0))
        {
            log_(LogLevel::ERROR) << "Error sending move ball command!\n";
        }

        // Save data
        // Fields: Test, BallX, BallY, Robots, (Seconds, FoundBy)\n
        log_(LogLevel::DEBUG_5) << counter_+1 << "," << ball_x*1000 << "," 
                                << ball_y*1000 << "," << su.players.size() 
                                << ",";

    }

    bool FindBallExperiment::StartExperiment()
    {
        ResetTimer();
        log_(LogLevel::INFO) << "New test started!\n";
        return true;
    }

    bool FindBallExperiment::FinishExperiment(int time, 
           const std::vector<Agent>& found_by)
    {   
        std::string found_str = "";
        for (auto itr = found_by.begin(); itr != found_by.end(); ++itr)
        {
            FromRunswiftAgent update;
            if (agent_server_.GetLastUpdate(*itr, &update))
            {
                found_str += '0' + update.player_number;
                if (itr != found_by.end()-1) found_str += ";";
            }
        }
        if (found_str == "")
        {
            found_str = "-1";
        }

        log_(LogLevel::INFO) << "Test " << counter_+1 << " completed. Ball found"
            << " by " << (found_str  == "-1" ? "nobody" : found_str)
            << " in " << time << " seconds.\n";

        // Save data
        // Fields: (Test, BallX, BallY, Robots), Seconds, FoundBy\n
        log_(LogLevel::DEBUG_5) << time << "," << found_str << "\n";

        ++counter_;
        return true;
    }

    bool FindBallExperiment::CancelExperiment()
    {
        if (GetExperimentState() == TEST_STARTED
            || GetExperimentState() == NOT_STARTED)
        {
            std::vector<Agent> agents;
            return FinishExperiment(GetTimerSeconds(), agents);
        }
        return true;
    }

    void FindBallExperiment::ResetTimer()
    {
        time(&timer_);
    }

    bool FindBallExperiment::CheckSimulatorGameState()
    {
        auto su = simulator_.GetLastUpdate();
        if (su.play_mode == PlayMode::BEFORE_KICK_OFF)
        {
            return simulator_.SendPlayModeCommand(PlayMode::GAME_OVER);
        }
        return true;
    }

    bool FindBallExperiment::LogAgentPositions()
    {
        auto time = GetTimerSeconds();
        if (!started_ || last_log_ == time)
        {
            return false;
        }
        last_log_ = time;

        FromRunswiftAgent updates[5];

        auto agents = agent_server_.GetAgents();
        for (int i=0; i < 5 && i < agents.size(); ++i)
        {
            FromRunswiftAgent u;
            agent_server_.GetLastUpdate(agents[i], &u);
            updates[u.player_number-1] = u;
        }

        // CSV format: "Test,Seconds,Robot1,Robot2,Robot3,Robot4,Robot5\n";
        log_(LogLevel::DEBUG_4) << counter_+1 << "," << GetTimerSeconds();

        for (int i=0; i < 5; ++i)
        {
            log_(LogLevel::DEBUG_4) << "," << updates[i].estimated_x_pos << ";"
                                    << updates[i].estimated_y_pos << ";"
                                    << updates[i].estimated_orientation;
        }
        log_(LogLevel::DEBUG_4) << "\n";

        return true;
    }

    int FindBallExperiment::GetExperimentState()
    {
        return state_;
    }

    bool FindBallExperiment::HasNewAgent()
    {
        int new_num_agents = agent_server_.GetAgents().size();
        bool new_agent = new_num_agents > num_agents_;
        num_agents_ = new_num_agents;
        return new_agent;
    }

    int FindBallExperiment::GetTimerSeconds()
    {
        time_t new_time;
        time(&new_time);
        return difftime(new_time, timer_);
    }
    
    bool FindBallExperiment::IsBallFound(std::vector<Agent>* found_by)
    {
        bool found = false;
        auto agents = agent_server_.GetAgents();
        for (auto& a : agents)
        {
            FromRunswiftAgent u;
            if (agent_server_.GetLastUpdate(a, &u))
            {
                // Hack dist for ball point 10
                if (u.ball_seen_count >= FIND_BALL_SEEN_FRAMES 
                    && u.can_see_ball 
                    && u.dist_from_ball <= FIND_BALL_MAX_DIST)
                {
                    found = true;
                    if (found_by)
                    {
                        found_by->push_back(a);
                    }
                }
            }
        }
        return found;
    }

    bool FindBallExperiment::GetStartingPosition(int player_num, float* x_out, float* y_out, float* o_out)
    {
        switch (player_num)
        {
            case 2:
                *x_out = -3.25;
                *y_out = 3;
                *o_out = 180;
                break;

            case 4:
                *x_out = -1.75;
                *y_out = 3;
                *o_out = 180;
                break;

            case 1:
                *x_out = -3;
                *y_out = -3;
                *o_out = 0;
                break;

            case 3:
                *x_out = -2;
                *y_out = -3;
                *o_out = 0;
                break;

            case 5:
                *x_out = -1;
                *y_out = -3;
                *o_out = 0;
                break;

            default:
                return false;
        }
        return true;
    }

    std::string FindBallExperiment::StateToString(int s)
    {
        switch(s)
        {
        case NOT_STARTED:
            return "NOT_STARTED";
        case TEST_STARTING:
            return "TEST_STARTING";
        case TEST_STARTED:
            return "TEST_STARTED";
        case TEST_FINISHED:
            return "TEST_FINISHED";
        }
        return "N/A";
    }

}

using namespace findballexp;
FindBallExperiment* experiment = nullptr;

void signal_handler(int signal)
{
    std::cout << "\nSIGNAL DETECTED. Shutting down...\n";
    if (experiment)
    {
        experiment->Finish();
    }
    delete experiment;
    experiment = nullptr;
    abort();
}


int run_experiment(int start_from)
{
    signal(SIGPIPE, SIG_IGN);

    Logger& log = Logger::GetInstance();
    log.AddStream(&std::cout, LogLevel::DEBUG, LogLevel::INFO);
    log.AddStream(&std::cerr, LogLevel::WARNING);

    log(LogLevel::INFO) << "Running econtroller experiment...\n";      
    EndpointConnection sim_ec;
    if (!sim_ec.Init("localhost", 3200))
    {
        log(LogLevel::ERROR) << "Error initialising connection to simulator.\n";
        return 1;
    }
    log(LogLevel::INFO) << "Connected to simulator on port 3200!\n";
    
    SimulatorConnection simulator;
    if (!simulator.Init(sim_ec))
    {
        log(LogLevel::ERROR) << "Error initialising connection to simulator.\n";
        return 2;
    }

    AgentServer<FromRunswiftAgent, ToRunswiftAgent> agent_server;
    if (!agent_server.Init(3232))
    {
        log(LogLevel::ERROR) << "Error initialising agent server.\n";
        return 3;
    }
    log(LogLevel::INFO) << "Listening for agents on port 3232...\n";

    experiment = new FindBallExperiment(simulator, agent_server, start_from);
    experiment->Init();
    while(true)
    {
        simulator.Tick();
        agent_server.Tick();
        if (!experiment->Tick())
        {
            break;
        }
        usleep(10);
    }
    experiment->Finish();
    return 0;
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    
    int start_from = 1;
    if (argc > 1)
    {
        start_from = std::stoi(argv[1]);
    }    

    run_experiment(start_from);
    if (experiment)
    {
        delete experiment;
        experiment = nullptr;
    }
    return 0;
}