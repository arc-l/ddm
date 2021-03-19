/**
 * @file custom_astar.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief A* searching function, borrowed and customized from the
 * libMultiRobotPlanning library
 * @version 0.1
 * @date 2021-01-09
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
#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include <libMultiRobotPlanning/neighbor.hpp>

#include "task.h"

namespace libMultiRobotPlanning {

template <typename State, typename Action, typename Cost>

/**
 * @brief Path planning result.
 * TODO not used much right now since we only have a single robot model. But
 * will be used extensively later.
 *
 */
struct SinglePlanResult {
    std::vector<State> states;    // states and their gScore
    std::vector<Action> actions;  // actions and their cost
    Cost cost;                    // actual cost of the result TODO not used
    Cost
        fmin;  // lower bound of the cost (for suboptimal solvers) TODO not used
};

/**
 * @brief Customized A* path planner
 *
 * @tparam State: Custom state for the search. Needs to be copy-able
 * @tparam Action: Custom action for the search. Needs to be copy-able
 * @tparam Cost: Custom Cost type (we normally just use double (why not?))
 * @tparam Environment: This class needs to provide the custom A* logic. In
 * particular, it needs to support the following functions:
 *  - Cost admissibleHeuristic(const State& s)
 *      This function can return 0 if no suitable heuristic is available.
 *  - bool isSolution(const State& s)
 *      Return true if the given state is a goal state.
 *  - void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
 *   int> >& neighbors)
 *      Fill the list of neighboring state for the given state s.
 *  - void onExpandNode(const State& s, int fScore, int gScore)
 *      This function is called on every expansion and can be used for
 *      statistical purposes.
 *  - void onDiscover(const State& s, int fScore, int gScore)
 *      This function is called on every node discovery and can be used for
 *      statistical purposes.
 * @tparam StateHasher: A class to convert a state to a hash value.
 */
template <typename State, typename Action, typename Cost, typename Environment,
          typename StateHasher = std::hash<State> >
class AStar {
   public:
    AStar(Environment& environment) : m_env(environment) {}

    /**
     * @brief Main search function
     *
     * @param startState
     * @param solution: place to put the solution
     * @param initialCost: TODO
     * @return true: Search successful
     * @return false: Unable to find a path
     */
    bool search(const State& startState,
                SinglePlanResult<State, Action, Cost>& solution,
                Cost initialCost = 0) {
        solution.states.clear();
        solution.states.push_back(startState);
        solution.actions.clear();
        solution.cost = 0;

        openSet_t openSet;
        std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
        std::unordered_set<State, StateHasher> closedSet;
        std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                           StateHasher>
            cameFrom;

        auto handle = openSet.push(
            Node(startState, m_env.admissibleHeuristic(startState, startState),
                 initialCost));
        stateToHeap.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        std::vector<Neighbor<State, Action, Cost> > neighbors;
        neighbors.reserve(5);

        while (!openSet.empty()) {
            Node current = openSet.top();
            m_env.onExpandNode(current.state, current.fScore, current.gScore);

            if (m_env.isSolution(current.state)) {
                solution.states.clear();
                solution.actions.clear();
                auto iter = cameFrom.find(current.state);
                while (iter != cameFrom.end()) {
                    solution.states.push_back(iter->first);
                    solution.actions.push_back(std::get<1>(iter->second));
                    iter = cameFrom.find(std::get<0>(iter->second));
                }
                solution.states.push_back(startState);
                std::reverse(solution.states.begin(), solution.states.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.gScore;
                return true;
            }

            openSet.pop();
            stateToHeap.erase(current.state);
            closedSet.insert(current.state);

            // traverse neighbors
            neighbors.clear();
            m_env.getNeighbors(current.state, neighbors);
            for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
                if (closedSet.find(neighbor.state) == closedSet.end()) {
                    Cost tentative_gScore = current.gScore + neighbor.cost;
                    auto iter = stateToHeap.find(neighbor.state);
                    if (iter == stateToHeap.end()) {  // Discover a new node
                        Cost fScore = tentative_gScore +
                                      m_env.admissibleHeuristic(current.state,
                                                                neighbor.state);
                        auto handle = openSet.push(
                            Node(neighbor.state, fScore, tentative_gScore));
                        (*handle).handle = handle;
                        stateToHeap.insert(
                            std::make_pair<>(neighbor.state, handle));
                        m_env.onDiscover(neighbor.state, fScore,
                                         tentative_gScore);
                    } else {
                        auto handle = iter->second;
                        // We found this node before with a better path
                        if (tentative_gScore >= (*handle).gScore) {
                            continue;
                        }
                        // update f and gScore
                        Cost delta = (*handle).gScore - tentative_gScore;
                        (*handle).gScore = tentative_gScore;
                        (*handle).fScore -= delta;
                        openSet.increase(handle);
                        m_env.onDiscover(neighbor.state, (*handle).fScore,
                                         (*handle).gScore);
                    }
                    // Best path for this node so far
                    cameFrom.erase(neighbor.state);
                    cameFrom.insert(std::make_pair<>(
                        neighbor.state,
                        std::make_tuple<>(current.state, neighbor.action,
                                          neighbor.cost, tentative_gScore)));
                }
            }
        }
        return false;
    }

   private:
    /**
     * @brief A search node in A*. Note that this is different from a graph node
     *
     */
    struct Node {
        Node(const State& state, Cost fScore, Cost gScore)
            : state(state), fScore(fScore), gScore(gScore) {}

        bool operator<(const Node& other) const {
            // Sort order
            // 1. lowest fScore
            // 2. highest gScore

            // Our heap is a maximum heap, so we invert the comperator function
            // here
            if (fScore != other.fScore) {
                return fScore > other.fScore;
            } else {
                return gScore < other.gScore;
            }
        }

        friend std::ostream& operator<<(std::ostream& os, const Node& node) {
            os << "state: " << node.state << " fScore: " << node.fScore
               << " gScore: " << node.gScore;
            return os;
        }

        State state;
        Cost fScore;
        Cost gScore;

#ifdef USE_FIBONACCI_HEAP
        typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
        typename boost::heap::d_ary_heap<
            Node, boost::heap::arity<2>,
            boost::heap::mutable_<true> >::handle_type handle;
#endif
    };

#ifdef USE_FIBONACCI_HEAP
    typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
#else
    typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                             boost::heap::mutable_<true> >
        openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
#endif

   private:
    Environment& m_env;
};
}  // namespace libMultiRobotPlanning

/**
 * @brief Check whether a plan is feasible
 *
 * @tparam State: same as AStar
 * @tparam Action: same as AStar
 * @tparam Cost: same as AStar
 * @tparam Environment: same as AStar
 * @param solution: A planning solution
 * @param env: Environment
 * @param start: start State
 * @param goal: goal State
 * @return true: plan is feasible
 * @return false: plan is infeasible
 */
template <typename State, typename Action, typename Cost, typename Environment>
inline bool check_single_path_feasibility(
    const libMultiRobotPlanning::SinglePlanResult<State, Action, Cost>&
        solution,
    const Environment& env, const State& start, const State& goal) {
    const Graph& g = env.g;
    const std::vector<State>& states = solution.states;
    const std::vector<Action>& actions = solution.actions;
    // Check if all nodes are in the given graph
    for (auto s : states)
        if (std::find(g.nodes.begin(), g.nodes.end(), s.n) == g.nodes.end())
            return false;
    // Check if all nodes are not obstacles
    for (auto s : states)
        if (g.is_blocked(s.n)) return false;
    // Check if start and goal are correctly located
    if (states[0] != start || states.back() != goal) return false;
    // Check if all transitions are valid
    for (size_t i = 0; i < states.size() - 1; i++)
        if (env.takeAction(states[i], actions[i]) != states[i + 1])
            return false;
    // Feasible
    return true;
}
