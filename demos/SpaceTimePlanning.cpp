/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the TU Berlin nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francesco Grothe */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "common.h"
#include "SharedEnv.h"
#include "ConstraintTable.h"
#include "constraint/ConstrainedPlanningCommon.h"


#include <boost/xpressive/detail/core/access.hpp>

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * Demonstration of planning through space time using the Animation state space and the SpaceTimeRRT* planner.
 */
SharedEnv* env_ptr;
ConstraintTable* constraint_table_ptr;
string benchmarkPath;
string solutionPath;
string dataPath;
Solution solution;
double width = 40.0;
double height = 40.0;

bool isStateValid(const ob::State *state)
{
    // extract the space component of the state and cast it to what we expect
    const auto x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    const auto y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];

    // extract the time component of the state and cast it to what we expect
    const auto t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    // check validity of state defined by pos & t (e.g. check if constraints are satisfied)...
    Point point = make_tuple(x, y);
    // check area constraints
    if (x < env_ptr->radii[0] || x > width - env_ptr->radii[0] || y < env_ptr->radii[0] || y > height - env_ptr->radii[0]) {
        return false;
    }
    // check static obstacles constraints
    for (const auto &obstacle : env_ptr->obstacles) {
        if (obstacle->constrained(point, env_ptr->radii[0])) {
            return false;
        }
    }
    // check dynamic obstacles constraints
    for (auto occupied_agent_id = 0; occupied_agent_id < constraint_table_ptr->path_table.size(); ++occupied_agent_id) {
        if (constraint_table_ptr->path_table[occupied_agent_id].empty()) continue;
        // path conflict
        for (int i = 0; i < constraint_table_ptr->path_table[occupied_agent_id].size() - 1; ++i) {
            auto [prev_point, prev_time] = constraint_table_ptr->path_table[occupied_agent_id][i];
            auto [next_point, next_time] = constraint_table_ptr->path_table[occupied_agent_id][i + 1];

            // check if temporal constraint is satisfied
            if (prev_time > t || t > next_time)
                continue;

            const double expand_distance = calculateDistance(prev_point, next_point);
            const double expand_time = t - prev_time;
            const double theta = atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
            const double velocity = expand_distance / (next_time - prev_time);
            assert(expand_time >= 0.0);
            auto agent_point = prev_point;
            if (theta != 0.0) {
                agent_point = make_tuple(get<0>(prev_point) + velocity * cos(theta) * expand_time,
                                         get<1>(prev_point) + velocity * sin(theta) * expand_time);
            }
            if (calculateDistance(point, agent_point) < env_ptr->radii[0] + env_ptr->radii[occupied_agent_id]) {
                return false;
            }
        }
        // target conflict
        auto [last_point, last_time] = constraint_table_ptr->path_table[occupied_agent_id].back();
        // check if temporal constraint is satisfied
        if (last_time > t)
            continue;
        if (calculateDistance(point, last_point) < env_ptr->radii[0] + env_ptr->radii[occupied_agent_id]) {
            return false;
        }
    }
    return true;
}

class SpaceTimeMotionValidator : public ob::MotionValidator {

public:
    explicit SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si) : MotionValidator(si),
      vMax_(si_->getStateSpace().get()->as<ob::SpaceTimeStateSpace>()->getVMax()),
      stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        // assume motion starts in a valid configuration, so s1 is valid
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        // check if motion is forward in time and is not exceeding the speed limit
        auto *space = stateSpace_->as<ob::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                      s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_)) {
            invalid_++;
            return false;
        }

        // check if the path between the states is unconstrained (perform interpolation)...
        // interpolate states
        auto *state1 = s1->as<ob::CompoundState>();
        auto *state2 = s2->as<ob::CompoundState>();
        auto *state1Pos = state1->as<ob::RealVectorStateSpace::StateType>(0);
        auto *state2Pos = state2->as<ob::RealVectorStateSpace::StateType>(0);
        auto *state1Time = state1->as<ob::TimeStateSpace::StateType>(1);
        auto *state2Time = state2->as<ob::TimeStateSpace::StateType>(1);

        Point point1 = make_tuple(state1Pos->values[0], state1Pos->values[1]);
        Point point2 = make_tuple(state2Pos->values[0], state2Pos->values[1]);
        const double expand_distance = calculateDistance(point1, point2);
        const double theta = atan2(get<1>(point2) - get<1>(point1), get<0>(point2) - get<0>(point1));
        const double velocity = expand_distance / (state2Time->position - state1Time->position);
        const auto timesteps = static_cast<int>(floor(expand_distance / velocity));
        for (double timestep = 0.0; timestep < timesteps; timestep += 0.5) {
            Point interpolated_point = point1;
            if (theta != 0.0) {
                interpolated_point = make_tuple(get<0>(point1) + velocity * cos(theta) * timestep,
                                                get<1>(point1) + velocity * sin(theta) * timestep);
            }
            // new ob state
            ob::State *interpolated_state = stateSpace_->as<ob::SpaceTimeStateSpace>()->allocState();
            interpolated_state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = get<0>(interpolated_point);
            interpolated_state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = get<1>(interpolated_point);
            interpolated_state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = state1Time->position + timestep;
            if (!isStateValid(interpolated_state)) {
                invalid_++;
                return false;
            }
        }
        const double remain_time = fmod(expand_distance, velocity);
        if (remain_time > 0.001) {
            const Point interpolated_point = point2;
            // new ob state
            ob::State *interpolated_state = stateSpace_->as<ob::SpaceTimeStateSpace>()->allocState();
            interpolated_state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = get<0>(interpolated_point);
            interpolated_state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = get<1>(interpolated_point);
            interpolated_state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = state2Time->position;
            if (!isStateValid(interpolated_state)) {
                invalid_++;
                return false;
            }
        }

        return true;
    }

    bool checkMotion(const ompl::base::State *, const ompl::base::State *,
                     std::pair<ob::State *, double> &) const override
    {
        throw ompl::Exception("SpaceTimeMotionValidator::checkMotion", "not implemented");
    }

private:
    double vMax_; // maximum velocity
    ob::StateSpace *stateSpace_; // the animation state space for distance calculation
};

double getEarliestGoalArrivalTime(Point goal_point) {
    double earliest_goal_arrival_time = 0.0;
    for (auto occupied_agent_id = 0; occupied_agent_id < constraint_table_ptr->path_table.size(); ++occupied_agent_id) {
        if (constraint_table_ptr->path_table[occupied_agent_id].empty()) continue;
        for (int i = 0; i < constraint_table_ptr->path_table[occupied_agent_id].size() - 1; ++i) {
            auto [prev_point, prev_time] = constraint_table_ptr->path_table[occupied_agent_id][i];
            auto [next_point, next_time] = constraint_table_ptr->path_table[occupied_agent_id][i + 1];

            const double expand_distance = calculateDistance(prev_point, next_point);
            const double theta = atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
            const double velocity = expand_distance / (next_time - prev_time);
            const auto timesteps = static_cast<int>(floor(expand_distance / velocity));
            for (double timestep = 0.0; timestep < timesteps; timestep += 0.5) {
                Point interpolated_point = prev_point;
                if (theta != 0.0) {
                    interpolated_point = make_tuple(get<0>(prev_point) + velocity * cos(theta) * timestep,
                                                    get<1>(prev_point) + velocity * sin(theta) * timestep);
                }
                if (calculateDistance(interpolated_point, goal_point) < env_ptr->radii[occupied_agent_id] + env_ptr->radii[0]) {
                    earliest_goal_arrival_time = max(earliest_goal_arrival_time, prev_time + timestep + 1.0);
                }
            }
            const double remain_time = fmod(expand_distance, velocity);
            if (remain_time > 0.001) {
                const Point interpolated_point = next_point;
                if (calculateDistance(interpolated_point, goal_point) < env_ptr->radii[occupied_agent_id] + env_ptr->radii[0]) {
                    earliest_goal_arrival_time = max(earliest_goal_arrival_time, next_time + 1.0);
                }
            }
        }
    }
    return earliest_goal_arrival_time;
}

Path plan(int agent_id)
{
    // construct the state space we are planning in
    auto vectorSpace(std::make_shared<ob::RealVectorStateSpace>(2));
    auto space = std::make_shared<ob::SpaceTimeStateSpace>(vectorSpace, env_ptr->velocities[agent_id]);

    // set the bounds for R1
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(40.0);
    vectorSpace->setBounds(bounds);

    // set time bounds. Planning with unbounded time is also possible when using ST-RRT*.
    // space->setTimeBounds(0.0, 10.0);

    // create the space information class for the space
    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

    // set state validity checking for this space
    si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

    // define a simple setup class
    og::SimpleSetup ss(si);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = get<0>(env_ptr->start_points[agent_id]);
    start[1] = get<1>(env_ptr->start_points[agent_id]);

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = get<0>(env_ptr->goal_points[agent_id]);
    goal[1] = get<1>(env_ptr->goal_points[agent_id]);

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // construct the planner
    auto *strrtStar = new og::STRRTstar(si);

    // set planner parameters
    strrtStar->setRange(env_ptr->max_expand_distances[agent_id]);

    // set the used planner
    ss.setPlanner(ob::PlannerPtr(strrtStar));

    double earliest_goal_arrival_time = getEarliestGoalArrivalTime(make_tuple(goal[0], goal[1]));
    strrtStar->setGoalTime(earliest_goal_arrival_time);
    strrtStar->setAgentIndex(agent_id);
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(300.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        // ss.getSolutionPath().print(std::cout);
        auto ompl_path = ss.getSolutionPath();
        Path path = {};

        for (auto &state : ompl_path.getStates()) {
            const auto x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
            const auto y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
            const auto t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
            Point point = make_tuple(x, y);
            path.emplace_back(make_tuple(point, t));
        }
        Point start = make_tuple(get<0>(env_ptr->start_points[agent_id]), get<1>(env_ptr->start_points[agent_id]));
        Point goal = make_tuple(get<0>(env_ptr->goal_points[agent_id]), get<1>(env_ptr->goal_points[agent_id]));
        if (calculateDistance(start, get<0>(path.front())) >= 0.001) {
            cout << "Start point is not on the path" << endl;
            return {};
        }
        if (calculateDistance(goal, get<0>(path.back())) >= 0.001) {
            cout << "Goal point is not on the path" << endl;
            return {};
        }
        return path;
    }
    else {
        std::cout << "No solution found" << std::endl;
        return {};
    }
}

// Function to split a string by a delimiter
std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Function to parse a single point
Point parsePoint(const std::string &s) {
    std::vector<std::string> pointData = split(s, ',');
    double x = std::stod(pointData[0]);
    double y = std::stod(pointData[1]);
    return std::make_tuple(x, y);
}

// Updated function to parse a path
Path parsePath(const std::string &s) {
    std::vector<std::string> pathData = split(s, '-'); // Splitting using "->" instead of ">"
    Path path;
    for (const std::string &pointString : pathData) {
        size_t start = pointString.find('(') + 1;
        size_t end = pointString.find(')', start);
        if (start == std::string::npos || end == std::string::npos) {
            continue; // Invalid format, skip this entry
        }
        std::string pointData = pointString.substr(start, end - start);
        Point point = parsePoint(pointData);

        size_t zStart = pointString.rfind(',', end) + 1;
        if (zStart == std::string::npos || zStart >= end) {
            continue; // Invalid format, skip this entry
        }
        std::string zData = pointString.substr(zStart, end - zStart);

        double z;
        try {
            z = std::stod(zData);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument for stod: " << zData << '\n';
            continue; // Invalid z value, skip this entry
        }

        path.push_back(std::make_tuple(point, z));
    }
    return path;
}

// Updated function to read and parse the file
Solution parseFile(const std::string &fileName) {
    std::ifstream file(fileName);
    Solution solution;
    std::string line;
    while (std::getline(file, line)) {
        size_t labelEnd = line.find(':'); // Find the end of the agent label
        if (labelEnd != std::string::npos) {
            std::string pathString = line.substr(labelEnd + 1); // Extract the path string after the label
            Path path = parsePath(pathString);
            solution.push_back(path);
        }
    }
    return solution;
}

int main(int argc, char ** argv)
{
    string mapname;
    string obs;
    string robotnum;
    string testnum;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            mapname = argv[i + 1];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            obs = argv[i + 1];
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            robotnum = argv[i + 1];
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            testnum = argv[i + 1];
        }
    }

    benchmarkPath = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                       robotnum + "_" + testnum + ".yaml";
    solutionPath = "solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                          robotnum + "_" + testnum + "_solution.txt";
    dataPath = "data/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum +
                      "_" + testnum + "_data.txt";
    YAML::Node config = YAML::LoadFile(benchmarkPath);
    vector<shared_ptr<Obstacle>> obstacles;
    for (size_t i = 0; i < config["obstacles"].size(); ++i) {
        if (mapname == "CircleEnv") {
            auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
            auto radius = config["obstacles"][i]["radius"].as<double>();
            obstacles.emplace_back(make_shared<CircularObstacle>(center[0], center[1], radius));
        } else {
            auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
            auto height = config["obstacles"][i]["height"].as<double>();
            auto width = config["obstacles"][i]["width"].as<double>();
            obstacles.emplace_back(make_shared<RectangularObstacle>(center[0], center[1], width, height));
        }
    }
    vector<Point> start_points;
    vector<Point> goal_points;
    for (size_t i = 0; i < config["startPoints"].size(); ++i) {
        auto start = config["startPoints"][i].as<std::vector<double>>();
        auto goal = config["goalPoints"][i].as<std::vector<double>>();
        start_points.emplace_back(start[0], start[1]);
        goal_points.emplace_back(goal[0], goal[1]);
    }
    int num_of_agents = config["agentNum"].as<int>();
    int width = 40;
    int height = 40;
    vector<double> radii;
    vector<double> max_expand_distances;
    vector<double> velocities;
    vector<double> thresholds;
    vector<int> iterations;
    vector<double> goal_sample_rates;
    // srand(time(0));
    for (int i = 0; i < num_of_agents; ++i) {
        // double randomValue = 0.25 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (1 - 0.25)));
        radii.emplace_back(0.5);
        max_expand_distances.emplace_back(5.0);
        velocities.emplace_back(0.5);
        thresholds.emplace_back(0.01);
        iterations.emplace_back(1500);
        goal_sample_rates.emplace_back(10.0);
    }
    auto env = SharedEnv(num_of_agents, width, height, start_points, goal_points, radii, max_expand_distances, velocities, iterations, goal_sample_rates, obstacles);
    env_ptr = &env;
    ConstraintTable constraint_table(env);
    constraint_table_ptr = &constraint_table;

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    double sum_of_costs = 0.0;
    double makespan = 0.0;
    // for (int agent_id = 0; agent_id < 30; ++agent_id) {
    //     cout << "Agent " << agent_id << endl;
    //     auto path = plan(agent_id);
    //     solution.emplace_back(path);
    //     sum_of_costs += get<1>(path.back());
    //     makespan = max(makespan, get<1>(path.back()));
    //     constraint_table.path_table[agent_id] = path;
    // }
    std::string fileName = "dynamic_obstacle.txt";
    Solution solution = parseFile(fileName);
    for (int agent_id = 0; agent_id < solution.size(); ++agent_id) {
        constraint_table.path_table[agent_id] = solution[agent_id];
    }


    start_points[30] = make_tuple(1.0, 1.0);
    goal_points[30] = make_tuple(39.0, 39.0);
    env.start_points[30] = start_points[30];
    env.goal_points[30] = goal_points[30];
    cout << "Lower bound: " << calculateDistance(start_points[30], goal_points[30]) * 2 << endl;
    auto path = plan(30);
    solution.emplace_back(path);


    std::chrono::duration<double, std::ratio<1>> duration = std::chrono::high_resolution_clock::now() - start_time;
    saveSolution(solution, solutionPath);
    saveData(sum_of_costs, makespan, duration.count(), dataPath);

    return 0;
}