/**
 * @file ddm.cpp
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief the DDM algorithm (https://ieeexplore.ieee.org/document/8962218)
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
    BSD 2-Clause License

    Copyright (c) 2021, Rutgers Algorithmic Robotics and Control Lab
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "ddm.h"

std::vector<std::unordered_map<std::string, std::string>> ddm_database;

void load_database() {
    if (!ddm_database.empty()) return;  // Resolve multi-calls
    // The database is organized as 1 empty entry, 6 entries for 2x3 graph, and
    // 8 entries for 3x3 graph with an obstacle
    ddm_database = std::vector<std::unordered_map<std::string, std::string>>(
        15, std::unordered_map<std::string, std::string>());
    // Load 2x3 database
    for (size_t i = 1; i < 7; i++) {
        std::string file_name = "database/2x3-" + std::to_string(i) + ".db";
        std::ifstream file_reader(file_name);
        std::string temp_line;
        while (std::getline(file_reader, temp_line)) {
            std::string initial_goal, path;
            std::stringstream lineStream(temp_line);
            lineStream >> initial_goal;
            lineStream >> path;
            ddm_database[i][initial_goal] = path;
        }
    }
    // Load 3x3 database
    for (size_t i = 7; i < 15; i++) {
        std::string file_name =
            "database/3x3-obs-" + std::to_string(i - 6) + ".db";
        std::ifstream file_reader(file_name);
        std::string temp_line;
        while (std::getline(file_reader, temp_line)) {
            std::string initial_goal, path;
            std::stringstream lineStream(temp_line);
            lineStream >> initial_goal;
            lineStream >> path;
            ddm_database[i][initial_goal] = path;
        }
    }
    assert(ddm_database[1].size() == 36);
}

std::vector<std::vector<Node>> DdmSolver::solve(OneShotTask& task, Graph& g,
                                                int horizon) {
    assert(!ddm_database.empty());
    // Make sure the output is deterministic
    srand(0);
    // Generate all necessary data structures
    auto paths =
        std::vector<std::vector<Node>>();  // Valid solution paths. Arranged as
                                           // config at time. Will be populated
                                           // as the algorithm runs
    paths.push_back(std::vector<Node>());
    for (size_t i = 0; i < task.num_robots; i++)
        paths[0].push_back(task.starts[i]);
    auto future_paths = std::vector<std::vector<Node>>(
        task.num_robots,
        std::vector<Node>());  // Envisioned paths without
                               // considering collisions. Arranged
                               // as a path per robot. Note that
                               // the paths are arranged in
                               // reverse for quick pop operation
    auto occupied_subgraphs =
        std::vector<Protected_Subgraph>();  // Subgraphs used for collision
                                            // resolution
    // Sort robots, with shorter path length in the front
    auto robot_order = std::vector<size_t>(task.num_robots);
    auto path_lengths = std::vector<size_t>(task.num_robots);
    for (size_t i = 0; i < task.num_robots; i++) {
        robot_order[i] = i;
        path_lengths[i] = get_manhattan_distance(task.starts[i], task.goals[i]);
    }
    std::sort(robot_order.begin(), robot_order.end(),
              [&path_lengths](size_t a, size_t b) {
                  return path_lengths[a] < path_lengths[b];
              });
    // First, get an independent path for each robot
    auto path_planner =
        SingleRobotPathPlanner<OmniDirectionalRobot::State,
                               OmniDirectionalRobot::Action,
                               OmniDirectionalRobot::Environment>(g);
    for (size_t i = 0; i < task.num_robots; i++) {
        auto robot_index = robot_order[i];
        future_paths[robot_order[i]] = path_planner.reversed_search(
            task.starts[robot_order[i]], task.goals[robot_order[i]]);
        future_paths[robot_order[i]].pop_back();
    }

    // Entering main solving iteration
    while (true) {
        // Check if robots are already at the goals. If goals reached,
        // return the solution; if max horizon reached, return solution.
        bool future_paths_all_empty = true;
        for (size_t i = 0; i < task.num_robots; i++)
            if (!future_paths[i].empty()) {
                future_paths_all_empty = false;
                break;
            }
        if (future_paths_all_empty || paths.size() > horizon) {
            return paths;
        }

        // If a robot has finished its paths, add its current Node
        // (TODO should be the goal) as the next waypoint
        for (size_t i = 0; i < task.num_robots; i++)
            if (future_paths[i].empty())
                future_paths[i].push_back(paths.back()[i]);

        // Find the next step according to future paths
        auto next_step = std::vector<Node>(task.num_robots);
        for (size_t i = 0; i < task.num_robots; i++)
            next_step[i] = future_paths[i].back();

        // Find all collisions
        auto collided_robots =
            std::vector<std::pair<size_t, size_t>>();  // Robots with
                                                       // conflicts
        for (size_t i = 0; i < task.num_robots; i++)
            for (size_t j = i + 1; j < task.num_robots; j++)
                if (next_step[i] == next_step[j] ||
                    (paths.back()[i] == next_step[j] &&
                     paths.back()[j] == next_step[i]))
                    collided_robots.push_back(std::make_pair(i, j));
        // Random shuffle collisions for more diversified sollision
        // resolution
        std::shuffle(collided_robots.begin(), collided_robots.end(),
                     std::default_random_engine(rand()));

        // Resolve collisions one by one
        if (!collided_robots.empty()) {
            for (auto collided_robot_pair : collided_robots) {
                // Find the colliding robots, with the one with a longer
                // future path in the front
                size_t r1 = collided_robot_pair.first;
                size_t r2 = collided_robot_pair.second;

                if (future_paths[r1].size() < future_paths[r2].size())
                    std::swap(r1, r2);

                // Find a 2x3 graph
                std::vector<SubGraph> subgraph_candidates =
                    get_all_possible_2x3(paths.back()[r1], paths.back()[r2],
                                         g);  // All possible 2x3 graphs
                                              // w.r.task. environment obstacles
                // Choose the 2x3 graph that does not intersect with current
                // 2x3 graphs in use
                size_t chosen_subgraph_candidate_index = 0;
                for (; chosen_subgraph_candidate_index <
                       subgraph_candidates.size();
                     chosen_subgraph_candidate_index += 1) {
                    bool no_intersection =
                        true;  // Flag to tell if the candidate has
                               // intersection with any existing subgraphs
                    for (auto occupied_subgraph : occupied_subgraphs)
                        if (subgraph_candidates[chosen_subgraph_candidate_index]
                                .intersect_with(occupied_subgraph.graph)) {
                            no_intersection = false;
                            break;
                        }
                    if (no_intersection) break;
                }

                // If a valid 2x3 graph is not found, move on to the next
                // collision; otherwise start collision resolution
                if (chosen_subgraph_candidate_index ==
                    subgraph_candidates.size())
                    continue;
                SubGraph& subg =
                    subgraph_candidates[chosen_subgraph_candidate_index];

                // Find all robots in the subgraph. The movement of these
                // robots will be affected by the collision resolution
                // process. Sort them to give priority to robots with a
                // longer residual path
                std::vector<size_t> affected_robots({r1, r2});
                for (size_t i = 0; i < task.num_robots; i++)
                    if (i != r1 && i != r2 && subg.contains(paths.back()[i]))
                        affected_robots.push_back(i);
                std::vector<std::vector<Node>>* pp = &future_paths;
                std::sort(affected_robots.begin(), affected_robots.end(),
                          [&pp](size_t a, size_t b) {
                              return (*pp)[a].size() > (*pp)[b].size();
                          });

                // Find temporary starts and goals for each robot.
                std::vector<Node> potential_sub_goals =
                    subg.get_nodes();  // Nodes in the subgraph not yet
                                       // assigned to the robots as goals
                std::vector<Node> sub_starts_global(
                    affected_robots.size());  // Starts of the robots in the
                                              // subproblem in global scope
                std::vector<Node> sub_goals_global(
                    affected_robots.size());  // Goal of the robots in the
                                              // subproblem in global scope
                for (size_t i = 0; i < affected_robots.size(); i++) {
                    sub_starts_global[i] = paths.back()[affected_robots[i]];
                    // Find the robot's the last waypoint in the subgraph.
                    // Here we use a lookahead of at most 5 steps to speed
                    // up.
                    size_t last_waypoint_index = std::max(
                        0, int(future_paths[affected_robots[i]].size()) - 5);
                    bool found_subgoal = false;
                    for (; last_waypoint_index <
                           future_paths[affected_robots[i]].size();
                         last_waypoint_index++)
                        if (subg.contains(future_paths[affected_robots[i]]
                                                      [last_waypoint_index])) {
                            sub_goals_global[i] =
                                future_paths[affected_robots[i]]
                                            [last_waypoint_index];
                            found_subgoal = true;
                            break;
                        }
                    auto it = std::find(potential_sub_goals.begin(),
                                        potential_sub_goals.end(),
                                        sub_goals_global[i]);
                    // Find if the desired goal is occupied. If goal not
                    // occupied, assign.
                    if (found_subgoal && it != potential_sub_goals.end()) {
                        potential_sub_goals.erase(it);
                        // Update future path
                        // TODO STO: update path
                        future_paths[affected_robots[i]].erase(
                            future_paths[affected_robots[i]].begin() +
                                last_waypoint_index,
                            future_paths[affected_robots[i]].end());
                        if (future_paths[affected_robots[i]].empty()) {
                            // TODO check if this is necessary
                            future_paths[affected_robots[i]].push_back(
                                task.goals[affected_robots[i]]);
                            // TODO STO: update path
                        }
                        continue;
                    }
                    // If desired goal occupied, try to move the robot to a
                    // random vertex
                    // TODO replace with some near vertex?
                    std::shuffle(potential_sub_goals.begin(),
                                 potential_sub_goals.end(),
                                 std::default_random_engine(rand()));
                    if (g.is_blocked(potential_sub_goals.back()))
                        potential_sub_goals.pop_back();
                    sub_goals_global[i] = potential_sub_goals.back();
                    potential_sub_goals.pop_back();
                    if (sub_goals_global[i] != sub_starts_global[i]) {
                        future_paths[affected_robots[i]].clear();
                        if (sub_goals_global[i] !=
                            task.goals[affected_robots[i]]) {
                            future_paths[affected_robots[i]] =
                                path_planner.reversed_search(
                                    sub_goals_global[i],
                                    task.goals[affected_robots[i]]);
                            future_paths[affected_robots[i]].pop_back();
                        }
                    }
                }
                // Get local starts and goals
                size_t num_affected_robots = affected_robots.size();
                auto sub_starts_local = std::vector<size_t>();
                auto sub_goals_local = std::vector<size_t>();
                if (subg.small) {
                    if (subg.hi.x - subg.lo.x ==
                        2)  // Subgraph landscape placement
                        for (size_t i = 0; i < affected_robots.size(); i++) {
                            sub_starts_local.push_back(
                                (sub_starts_global[i].x - subg.lo.x) +
                                (sub_starts_global[i].y - subg.lo.y) * 3);
                            sub_goals_local.push_back(
                                (sub_goals_global[i].x - subg.lo.x) +
                                (sub_goals_global[i].y - subg.lo.y) * 3);
                        }
                    else {  // Subgraph portrait placement
                        size_t converter[] = {0, 3, 1, 4, 2, 5};
                        for (size_t i = 0; i < affected_robots.size(); i++) {
                            sub_starts_local.push_back(
                                converter[(sub_starts_global[i].x - subg.lo.x) +
                                          (sub_starts_global[i].y - subg.lo.y) *
                                              2]);
                            sub_goals_local.push_back(
                                converter[(sub_goals_global[i].x - subg.lo.x) +
                                          (sub_goals_global[i].y - subg.lo.y) *
                                              2]);
                        }
                    }
                } else {
                    // 3x3 with an obstacle at corner.
                    std::vector<int> converter;
                    if (g.is_blocked(subg.hi.x, subg.hi.y))
                        converter =
                            std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8});
                    if (g.is_blocked(subg.lo.x, subg.lo.y))
                        converter =
                            std::vector<int>({8, 7, 6, 5, 4, 3, 2, 1, 0});
                    if (g.is_blocked(subg.hi.x, subg.lo.y))
                        converter =
                            std::vector<int>({2, 5, 8, 1, 4, 7, 0, 3, 6});
                    if (g.is_blocked(subg.lo.x, subg.hi.y))
                        converter =
                            std::vector<int>({6, 3, 0, 7, 4, 1, 8, 5, 2});
                    for (size_t i = 0; i < affected_robots.size(); i++) {
                        sub_starts_local.push_back(
                            converter[(sub_starts_global[i].x - subg.lo.x) +
                                      (sub_starts_global[i].y - subg.lo.y) *
                                          3]);
                        sub_goals_local.push_back(
                            converter[(sub_goals_global[i].x - subg.lo.x) +
                                      (sub_goals_global[i].y - subg.lo.y) * 3]);
                    }
                }

                // Sort starts and goals for database entires
                std::vector<size_t> sub_starts_local_sorted =
                    sub_starts_local;  // Sorted local starts
                std::sort(sub_starts_local_sorted.begin(),
                          sub_starts_local_sorted.end());
                auto sort_index = std::vector<size_t>(num_affected_robots, 0);
                auto sub_goals_local_sorted = std::vector<size_t>(
                    num_affected_robots, 0);  // Sorted local goals
                for (size_t i = 0; i < num_affected_robots; i++) {
                    sort_index[i] = std::distance(
                        sub_starts_local.begin(),
                        find(sub_starts_local.begin(), sub_starts_local.end(),
                             sub_starts_local_sorted[i]));
                    sub_goals_local_sorted[i] = sub_goals_local[sort_index[i]];
                }

                // Database query
                std::string database_key;
                for (auto var : sub_starts_local_sorted)
                    database_key += std::to_string(var);
                for (auto var : sub_goals_local_sorted)
                    database_key += std::to_string(var);
                std::string solution_str;
                if (subg.small)
                    solution_str =
                        ddm_database[num_affected_robots][database_key];
                else
                    solution_str =
                        ddm_database[num_affected_robots + 6][database_key];

                // Extract solution
                size_t path_length_subg =
                    solution_str.size() / num_affected_robots;
                auto temp_paths = std::vector<std::vector<size_t>>(
                    num_affected_robots,
                    std::vector<size_t>(path_length_subg + 2, 0));
                for (size_t t_ = 0; t_ < path_length_subg; t_++)
                    for (size_t i = 0; i < num_affected_robots; i++) {
                        temp_paths[sort_index[i]][t_ + 1] =
                            solution_str[t_ * num_affected_robots + i] - 48;
                    }
                for (size_t i = 0; i < num_affected_robots; i++) {
                    temp_paths[i][0] = sub_starts_local[i];
                    temp_paths[i][path_length_subg + 1] = sub_goals_local[i];
                }

                // Retrive solution and modify planned paths
                path_length_subg += 1;
                if (subg.small) {
                    if (subg.hi.x - subg.lo.x ==
                        2)  // Subgraph landscape placement
                        for (size_t t = path_length_subg; t > 0; t--) {
                            for (size_t i = 0; i < num_affected_robots; i++) {
                                future_paths[affected_robots[i]].push_back(
                                    Node(subg.lo.x + temp_paths[i][t] % 3,
                                         subg.lo.y + temp_paths[i][t] / 3));
                            }
                        }
                    else {  // Subgraph portrait placement
                        static int converter[] = {0, 2, 4, 1, 3, 5};
                        for (size_t t = path_length_subg; t > 0; t--) {
                            for (size_t i = 0; i < num_affected_robots; i++) {
                                temp_paths[i][t] = converter[temp_paths[i][t]];
                                future_paths[affected_robots[i]].push_back(
                                    Node(subg.lo.x + temp_paths[i][t] % 2,
                                         subg.lo.y + temp_paths[i][t] / 2));
                            }
                        }
                    }
                } else {
                    // 3x3 with an obstacle at corner.
                    std::vector<int> converter;
                    if (g.is_blocked(subg.hi.x, subg.hi.y))
                        converter =
                            std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8});
                    if (g.is_blocked(subg.lo.x, subg.lo.y))
                        converter =
                            std::vector<int>({8, 7, 6, 5, 4, 3, 2, 1, 0});
                    if (g.is_blocked(subg.lo.x, subg.hi.y))
                        converter =
                            std::vector<int>({2, 5, 8, 1, 4, 7, 0, 3, 6});
                    if (g.is_blocked(subg.hi.x, subg.lo.y))
                        converter =
                            std::vector<int>({6, 3, 0, 7, 4, 1, 8, 5, 2});
                    for (size_t t = path_length_subg; t > 0; t--) {
                        for (size_t i = 0; i < num_affected_robots; i++) {
                            temp_paths[i][t] = converter[temp_paths[i][t]];
                            future_paths[affected_robots[i]].push_back(
                                Node(subg.lo.x + temp_paths[i][t] % 3,
                                     subg.lo.y + temp_paths[i][t] / 3));
                        }
                    }
                }
                // Mark subg as occupied
                occupied_subgraphs.push_back(Protected_Subgraph(
                    subg, affected_robots, path_length_subg));
            }
        }

        // Execute paths: preparation
        paths.push_back(std::vector<Node>(
            task.num_robots, Node(std::numeric_limits<int>::max(),
                                  std::numeric_limits<int>::max())));
        for (size_t i = 0; i < task.num_robots; i++)
            next_step[i] = future_paths[i].back();

        // Execute paths: execute protected paths (robots in subgraphs), and
        // deal with all non-protected paths (robots not in subgraphs)
        auto non_protected_robots = std::vector<size_t>(task.num_robots);
        for (size_t i = 0; i < task.num_robots; i++)
            non_protected_robots[i] = i;
        for (size_t i = 0; i < occupied_subgraphs.size(); i++)
            for (size_t j = 0; j < occupied_subgraphs[i].robots.size(); j++) {
                // Execute protected robots
                paths.back()[occupied_subgraphs[i].robots[j]] =
                    next_step[occupied_subgraphs[i].robots[j]];
                future_paths[occupied_subgraphs[i].robots[j]].pop_back();
                // Non-protected
                non_protected_robots.erase(find(
                    non_protected_robots.begin(), non_protected_robots.end(),
                    occupied_subgraphs[i].robots[j]));
            }

        // Execute paths: non protected robots
        // Iterate through subgraphs to see if a robot needs to be stopped
        for (size_t r : non_protected_robots)
            for (size_t i = 0; i < occupied_subgraphs.size(); i++)
                if (occupied_subgraphs[i].graph.contains(next_step[r])) {
                    // Outlier: if the robot does not affect the collision
                    // resolution, it can move into the subgraph
                    if (occupied_subgraphs[i].delay == 1)
                        if (std::find(paths.back().begin(), paths.back().end(),
                                      next_step[r]) == paths.back().end())
                            break;
                    // Otherwise, the robot stays still
                    if (!(next_step[r] == paths.end()[-2][r])) {
                        future_paths[r].push_back(paths.end()[-2][r]);
                    }
                    next_step[r] = paths.end()[-2][r];
                    break;
                }

        // Execute paths: non protected robots
        // Recursively stop the other robots
        auto flow =
            std::unordered_map<Node, std::vector<std::pair<size_t, Node>>,
                               std::hash<Node>>();  // Key is a node, value is
                                                    // the indices and current
                                                    // Nodeations of the robots
                                                    // going into the node
        for (auto r : non_protected_robots) {
            auto it = flow.find(next_step[r]);
            if (it == flow.end())
                flow.insert(std::make_pair(
                    next_step[r],
                    std::vector<std::pair<size_t, Node>>(
                        {std::make_pair(r, paths.end()[-2][r])})));
            else
                it->second.push_back(std::make_pair(r, paths.end()[-2][r]));
        }
        auto stopped_robots = std::vector<size_t>();
        for (auto it = flow.begin(); it != flow.end(); it++) {
            if (it->second.size() ==
                1) {  // The node has only one robot intended to move in
                auto it2 = flow.find(it->second[0].second);
                if (it2 == flow.end()) continue;
                // Here, if jump is false, there will be a head to head
                // collision. Thus robots must be stopped.
                bool jump = true;
                for (auto var : it2->second)
                    if (var.second == it->first) {
                        jump = false;
                        break;
                    }
                if (jump) continue;
            }
            // Recursively stop robots
            recursive_adder(flow, stopped_robots, it);
        }

        // Execute paths: non protected robots
        for (auto r : stopped_robots) {
            paths.back()[r] = paths.end()[-2][r];
            if (future_paths[r].back() == paths.end()[-2][r]) {
                future_paths[r].pop_back();
            }
        }
        for (auto r : non_protected_robots)
            if (paths.back()[r].x == std::numeric_limits<int>::max()) {
                paths.back()[r] = next_step[r];
                future_paths[r].pop_back();
            }

        // Remove duplicate step (if any)
        if (paths.back() == paths.end()[-2]) paths.pop_back();

        // Remove the subgraphs that finished execution
        auto remove_list = std::vector<size_t>();
        for (size_t i = 0; i < occupied_subgraphs.size(); i++) {
            occupied_subgraphs[i].delay -= 1;
            if (occupied_subgraphs[i].delay == 0) remove_list.push_back(i);
        }
        std::sort(remove_list.begin(), remove_list.end());
        std::reverse(remove_list.begin(), remove_list.end());
        for (size_t i : remove_list) {
            occupied_subgraphs.erase(occupied_subgraphs.begin() + i);
        }
    }
}

std::vector<std::vector<Node>> DdmSolver::solve(MultiGoalTask& t, Graph& g,
                                                int horizon) {
    // TODO
    throw "Not implemented";
    return std::vector<std::vector<Node>>();
}

inline std::vector<SubGraph> DdmSolver::get_all_possible_2x3(Node& n1, Node& n2,
                                                             const Graph& g) {
    std::vector<std::pair<Node, Node>> node_pairs;
    node_pairs.reserve(8);
    std::vector<SubGraph> result;
    result.reserve(8);
    int x_low = n1.x > n2.x ? n2.x : n1.x;
    int x_high = n1.x > n2.x ? n1.x : n2.x;
    int y_low = n1.y > n2.y ? n2.y : n1.y;
    int y_high = n1.y > n2.y ? n1.y : n2.y;
    // 3x3 with obs
    if (std::abs(n1.x - n2.x) == 1 && std::abs(n1.y - n2.y) == 1 &&
        (g.is_blocked(n1.x, n2.y) || g.is_blocked(n2.x, n1.y))) {
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low - 1), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 1), Node(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low), Node(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high + 1, y_high + 1)));
    }
    if (!node_pairs.empty()) {
        for (auto np : node_pairs)
            if (g.has_node(np.first) && g.has_node(np.second)) {
                auto subg = SubGraph(np.first, np.second);
                auto subg_nodes = subg.get_nodes();
                int num_blocked = 0;
                for (auto n : subg_nodes) {
                    if (g.is_blocked(n)) num_blocked++;
                    if (num_blocked > 1) break;
                }
                if (num_blocked == 1) {
                    result.push_back(subg);
                    break;
                }
            }
        assert(result.size() == 1);
        return result;
    }
    if (n1.x == n2.x && std::abs(n1.y - n2.y) == 1) {
        node_pairs.push_back(
            std::make_pair(Node(x_low - 2, y_low), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low), Node(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high + 2, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low - 1), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 1), Node(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low), Node(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high + 1, y_high + 1)));
    } else if (n1.x == n2.x && std::abs(n1.y - n2.y) == 2) {
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high + 1, y_high)));
    } else if (n1.y == n2.y && std::abs(n1.x - n2.x) == 1) {
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 2), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 1), Node(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high, y_high + 2)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low - 1), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low), Node(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 1), Node(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high + 1, y_high + 1)));
    } else if (n1.y == n2.y && std::abs(n1.x - n2.x) == 2) {
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 1), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high, y_high + 1)));
    } else {
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low - 1), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Node(x_low - 1, y_low), Node(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Node(x_low, y_low), Node(x_high + 1, y_high)));
    }
    for (auto np : node_pairs) {
        auto subg = SubGraph(np.first, np.second);
        if (test_2x3_valid(g, subg)) result.push_back(subg);
    }
    std::shuffle(result.begin(), result.end(),
                 std::default_random_engine(rand()));
    assert(result.size() != 0);
    return result;
}

inline bool DdmSolver::test_2x3_valid(const Graph& g, const SubGraph& subg) {
    if (subg.lo.x < 0 || subg.lo.y < 0 || subg.hi.x >= g.x_size ||
        subg.hi.y >= g.y_size)
        return false;
    for (auto var : subg.get_nodes())
        if (g.is_blocked(var)) return false;
    return true;
}
