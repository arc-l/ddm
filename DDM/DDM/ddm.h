/**
 * @file ddm.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief the DDM algorithm (https://ieeexplore.ieee.org/document/8962218)
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "search_utils.h"
#include "task.h"
#include "utils.h"

/**
 * @brief Describes a 2x3 graph. Note that it may occasionally be 3x3
 *
 */
struct SubGraph {
    Node lo;            // Lower corner
    Node hi;            // Upper corner
    bool small = true;  // Special case: 3x3 graph without a corner
    SubGraph(Node& lo_in, Node& hi_in) : lo(lo_in), hi(hi_in) {
        if (hi.x - lo.x + hi.y - lo.y == 4) small = false;
    };
    SubGraph(const SubGraph& that)
        : lo(that.lo), hi(that.hi), small(that.small){};
    /**
     * @brief Check whether the subgraph intersects with another one
     *
     * @param that:
     * @return true
     * @return false
     */
    bool intersect_with(const SubGraph& that) {
        return !(hi.x < that.lo.x || that.hi.x < lo.x || hi.y < that.lo.y ||
                 that.hi.y < lo.y);
    }
    /**
     * @brief Check whether the subgraph contains a node
     *
     * @param that:
     * @return true
     * @return false
     */
    bool contains(const Node& that) {
        return (that.x >= lo.x && that.x <= hi.x && that.y >= lo.y &&
                that.y <= hi.y);
    }
    /**
     * @brief Get the nodes inside the subgraph. Note that the list of nodes may
     * include obstacles
     *
     * @return std::vector<Node>
     */
    std::vector<Node> get_nodes() const {
        auto nodes = std::vector<Node>();
        nodes.reserve((hi.x - lo.x + 1) * (hi.y - lo.y + 1));
        for (int i = lo.x; i <= hi.x; i++)
            for (int j = lo.y; j <= hi.y; j++) nodes.push_back(Node(i, j));
        return nodes;
    }
};

/**
 * @brief Describes a collision resolution stage in a subgraph
 *
 */
struct Protected_Subgraph {
    SubGraph graph;              // The 2x3 graph
    std::vector<size_t> robots;  // Robots evolved in the graph
    int delay;  // Number of time steps before finishing collision resolution
    Protected_Subgraph(SubGraph& g_in, std::vector<size_t>& r_in, int d_in)
        : graph(g_in), robots(r_in), delay(d_in){};
};

extern std::vector<std::unordered_map<std::string, std::string>>
    ddm_database;  // Solution database
/**
 * @brief Load the path planning solution database. Must be called before
 * solving a problem.
 *
 */
void load_database();

/**
 * @brief DDM solver (https://ieeexplore.ieee.org/document/8962218)
 *
 */
class DdmSolver {
   public:
    DdmSolver(){};
    ~DdmSolver(){};

    /**
     * @brief Solve a one shot problem
     *
     * @param t:
     * @param g:
     * @param horizon: max number of time steps to solve
     * @return std::vector<std::vector<Node>>
     */
    std::vector<std::vector<Node>> solve(
        OneShotTask& t, Graph& g,
        int horizon = std::numeric_limits<int>::max());
    /**
     * @brief Solve a multi step problem
     * TODO not implemented
     *
     * @param t:
     * @param g:
     * @param horizon: max number of time steps to solve
     * @return std::vector<std::vector<Node>>
     */
    std::vector<std::vector<Node>> solve(
        MultiGoalTask& t, Graph& g,
        int horizon = std::numeric_limits<int>::max());

   private:
    /**
     * @brief Helper function to get subgraphs
     *
     * @param n1: a node to be included
     * @param n2: a node to be included
     * @param g: graph
     * @return std::vector<SubGraph>
     */
    inline std::vector<SubGraph> get_all_possible_2x3(Node& n1, Node& n2,
                                                      const Graph& g);
    /**
     * @brief Helper function to test whether a subgraph is valid (inside of a
     * graph, and contains no obstacles)
     *
     * @param g: graph
     * @param subg: subgraph
     * @return true
     * @return false
     */
    inline bool test_2x3_valid(const Graph& g, const SubGraph& subg);
};

/**
 * @brief Helper function to direct robot movement when they are prone to be
 * stopped
 *
 * @param flow:
 * @param stopped:
 * @param it:
 */
inline void recursive_adder(
    std::unordered_map<Node, std::vector<std::pair<size_t, Node>>,
                       std::hash<Node>>& flow,
    std::vector<size_t>& stopped,
    std::unordered_map<Node, std::vector<std::pair<size_t, Node>>,
                       std::hash<Node>>::iterator it) {
    if (it == flow.end()) return;
    for (auto var : it->second) {
        if (find(stopped.begin(), stopped.end(), var.first) == stopped.end()) {
            stopped.push_back(var.first);
            recursive_adder(flow, stopped, flow.find(var.second));
        }
    }
}
