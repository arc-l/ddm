/**
 * @file task.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief Defines the problem
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

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/functional/hash.hpp>

/**
 * @brief A grid point
 *
 */
struct Node {
    int x;
    int y;

    Node() : x(0), y(0){};
    Node(int x_in, int y_in) : x(x_in), y(y_in){};
    Node(const Node& node_in) : x(node_in.x), y(node_in.y){};

    inline bool operator==(const Node& that) {
        return (x == that.x && y == that.y);
    }
    inline bool operator!=(const Node& that) { return !(*this == that); }
    inline bool operator<(const Node& that) {
        return x < that.x || (!(that.x < x) && y < that.y);
    }
    inline Node operator=(const Node& that) {
        x = that.x;
        y = that.y;
        return *this;
    }
};
inline bool operator<(const Node& lhs, const Node& rhs) {
    return lhs.x < rhs.x || (!(rhs.x < lhs.x) && lhs.y < rhs.y);
}
inline bool operator==(const Node& lhs, const Node& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}
inline bool operator!=(const Node& lhs, const Node& rhs) {
    return !(lhs == rhs);
}
inline std::ostream& operator<<(std::ostream& out, Node const& loc) {
    out << "(" << loc.x << ", " << loc.y << ")";
    return out;
}

/**
 * @brief Hash function for a Node
 *
 */
namespace std {
template <>
struct hash<Node> {
    size_t operator()(const Node& n) const {
        size_t seed = 0;
        boost::hash_combine(seed, n.x);
        boost::hash_combine(seed, n.y);
        return seed;
    }
};
}  // namespace std

/**
 * @brief Get the manhattan distance between two Nodes
 *
 * @param n1: a Node
 * @param n2: a Node
 * @return int
 */
inline int get_manhattan_distance(const Node& n1, const Node& n2) {
    return std::abs(n1.x - n2.x) + std::abs(n1.y - n2.y);
};

/**
 * @brief Discrete square graph
 *
 */
struct Graph {
    int x_size = 0;
    int y_size = 0;
    std::vector<Node> nodes;      // Non-blocking locations in the graph
    std::vector<Node> obstacles;  // Blocked locations in the graph
    std::unordered_map<Node, std::vector<Node>, std::hash<Node>>
        adj_list;  // Adjacency list, each node is mapped to its neighbors
    std::vector<std::vector<bool>>
        blocked;  // Datastructure for quick checking whether a node exists

    /**
     * @brief Construct a new Graph object, which is an obstacle free graph.
     * This function is the entry point of constructing the graph. Then,
     * obstacles may be removed using the "remove_node" function.
     *
     * @param x_size_in: graph x dimension
     * @param y_size_in: graph y dimension
     */
    Graph(int x_size_in, int y_size_in) : x_size(x_size_in), y_size(y_size_in) {
        assert(x_size > 0 && y_size > 0);
        // Initialize data structures
        nodes = std::vector<Node>();
        nodes.reserve(x_size * y_size);
        adj_list =
            std::unordered_map<Node, std::vector<Node>, std::hash<Node>>();
        blocked = std::vector<std::vector<bool>>(
            x_size_in, std::vector<bool>(y_size_in, false));
        obstacles = std::vector<Node>();
        // Generate nodes
        for (size_t i = 0; i < x_size; i++)
            for (size_t j = 0; j < y_size; j++) {
                auto n = Node(i, j);
                nodes.push_back(n);
                // Find all neighbors
                auto neighbors = std::vector<Node>();
                if (i > 0) neighbors.push_back(Node(i - 1, j));
                if (i < x_size - 1) neighbors.push_back(Node(i + 1, j));
                if (j > 0) neighbors.push_back(Node(i, j - 1));
                if (j < y_size - 1) neighbors.push_back(Node(i, j + 1));
                adj_list.insert(std::make_pair(n, neighbors));
            }
    };

    /**
     * @brief Check if a node is marked as an obstacle
     *
     * @param x: node x value
     * @param y: node y value
     * @return true
     * @return false
     */
    inline bool is_blocked(int x, int y) const { return blocked[x][y]; }

    /**
     * @brief Check if a node is marked as an obstacle
     *
     * @param n: node
     * @return true
     * @return false
     */
    inline bool is_blocked(const Node& n) const { return blocked[n.x][n.y]; }

    /**
     * @brief Check if a node is in the x y range
     *
     * @param n: node
     * @return true
     * @return false
     */
    inline bool has_node(const Node& n) const {
        return (n.x >= 0 && n.x < x_size && n.y >= 0 && n.y < y_size);
    }

    /**
     * @brief Mark a node as obstacle, and update all data structures
     *
     * @param n: the obstacle node
     */
    inline void remove_node(Node& n) {
        blocked[n.x][n.y] = true;
        nodes.erase(std::find(nodes.begin(), nodes.end(), n));
        auto neighbors = adj_list.find(n)->second;
        for (auto neighbor : neighbors) {
            assert(neighbor != n);
            auto neighbor_it = adj_list.find(neighbor);
            neighbor_it->second.erase(std::find(neighbor_it->second.begin(),
                                                neighbor_it->second.end(), n));
        }
        adj_list.erase(n);
        obstacles.push_back(n);
    }
};
inline std::ostream& operator<<(std::ostream& out, Graph const& g) {
    out << "Information for current graph\n";
    out << "Dimension: " << g.x_size << " x " << g.y_size << "\n";
    for (int i = 0; i < g.y_size; i++) {
        for (int j = 0; j < g.x_size; j++) {
            if (g.is_blocked(j, i))
                out << "X";
            else
                out << "O";
        }
        out << std::endl;
    }
    return out;
}

/**
 * @brief Return a connected component of a graph
 *
 * @param g: input graph
 * @param avoid: nodes to exclude from searching
 * @return std::vector<Node>
 */
inline std::vector<Node> graph_single_bfs(
    Graph& g, std::set<Node> avoid = std::set<Node>()) {
    // Find a valid start vertex
    Node start = Node(-1, -1);
    for (auto n : g.nodes)
        if (avoid.find(n) == avoid.end()) {
            start = n;
            break;
        }
    if (start == Node(-1, -1)) return std::vector<Node>();
    // Initialize structures
    auto closed = std::set<Node>();
    auto open = std::set<Node>({start});
    // Perform BFS search
    while (!open.empty()) {
        auto temp = *open.begin();
        open.erase(temp);
        closed.insert(temp);
        for (auto n : g.adj_list[temp])
            if (closed.find(n) == closed.end() && avoid.find(n) == avoid.end())
                open.insert(n);
    }
    return std::vector<Node>(closed.begin(), closed.end());
}

/**

 * @brief Generate a graph with random obstacles
 *
 * @param x_size: graph x size
 * @param y_size: graph y size
 * @param obstacle_pct: obstacle percentage between 0 and 1
 * @param seed: random seed
 * @return Graph
 */
inline Graph get_graph(int x_size, int y_size, double obstacle_pct = 0,
                       int seed = 0) {
    // First, find an obstacle free graph
    Graph g = Graph(x_size, y_size);
    // Look for obstacles to remove
    assert(obstacle_pct >= 0 && obstacle_pct <= 1);
    srand(seed);
    while (g.nodes.size() > x_size * y_size * (1 - obstacle_pct)) {
        // Select a random node which is currently not an obstacle
        auto node_to_delete = g.nodes[rand() % g.nodes.size()];
        // If the graph is still connected after removal, remove the node
        if (graph_single_bfs(g, std::set<Node>({node_to_delete})).size() ==
            g.nodes.size() - 1)
            g.remove_node(node_to_delete);
    }
    return g;
}

/**
 * @brief Generate a low resolution graph
 *
 * @param x_size: graph x size
 * @param y_size: graph y size
 * @param obstacle_pct: obstacle percentage between 0 and 1
 * @param ratio: low resolution ration, int >= 1
 * @param seed: random seed
 * @return Graph
 */
inline Graph get_low_res_graph(int x_size, int y_size, double obstacle_pct = 0,
                               int ratio = 2, int seed = 0) {
    assert(ratio >= 1);
    assert(x_size % ratio == 0 && y_size % ratio == 0);
    // Generate a smaller graph
    Graph g_small =
        get_graph(x_size / ratio, y_size / ratio, obstacle_pct, seed);
    // Look for low resolution nodes
    auto low_res_nodes = std::vector<Node>();
    for (auto n : g_small.nodes) {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                low_res_nodes.push_back(Node(ratio * n.x + i, ratio * n.y + j));
            }
    }
    // Find an obstacle free graph
    Graph g = Graph(x_size, y_size);
    // Get the nodes to remove by vector difference
    auto obstacles = std::vector<Node>();
    for (auto n : g.nodes)
        if (std::find(low_res_nodes.begin(), low_res_nodes.end(), n) ==
            low_res_nodes.end())
            obstacles.push_back(n);
    // Remove nodes
    for (auto n : obstacles) g.remove_node(n);
    return g;
}

/**
 * @brief Generate a graph of warehouse structure
 *
 * @param num_shelf_x: number of shelves in x direction
 * @param num_shelf_y: number of shelves in y direction
 * @param shelf_x: shelf x size
 * @param shelf_y: shelf y size
 * @param space_x: space between two adjacent shelves in x direction
 * @param space_y: space between two adjacent shelves in y direction
 * @param border_x: graph boundary padding x
 * @param border_y: graph boundary padding y
 * @return Graph
 */
inline Graph get_warehouse_graph(int num_shelf_x, int num_shelf_y,
                                 int shelf_x = 5, int shelf_y = 2,
                                 int space_x = 3, int space_y = 2,
                                 int border_x = 4, int border_y = 3) {
    // Calculate x size and y size based on the input parameters
    int x_size = num_shelf_x * (shelf_x + space_x) - space_x + 2 * border_x;
    int y_size = num_shelf_y * (shelf_y + space_y) - space_y + 2 * border_y;
    // Construct graph: preparation.
    Graph g = Graph(x_size, y_size);
    // Remove obstacles from the shelves
    auto obstacles = std::vector<Node>();
    for (int x = 0; x < num_shelf_x; x++)
        for (int y = 0; y < num_shelf_y; y++) {
            int start_x = border_x + (shelf_x + space_x) * x;
            int start_y = border_y + (shelf_y + space_y) * y;
            for (int i = 0; i < shelf_x; i++)
                for (int j = 0; j < shelf_y; j++)
                    obstacles.push_back(Node(start_x + i, start_y + j));
        }
    for (auto n : obstacles) g.remove_node(n);
    return g;
}

inline Graph get_dao_graph(std::string dao_file_name) {
    // Readfile
    std::vector<std::string> lines;
    std::string temp;
    std::ifstream map_file("dao-map/" + dao_file_name + ".map");
    if (map_file) {
        while (std::getline(map_file, temp)) lines.push_back(temp);
        map_file.close();
    } else
        throw "DAO file name error\n";
    // Get dimension
    temp = lines[1].substr(7, lines[1].length() - 7);
    int x_size = atoi(temp.c_str());
    temp = lines[2].substr(6, lines[2].length() - 6);
    int y_size = atoi(temp.c_str());
    // Get obstacles
    auto obstacles = std::vector<Node>();
    lines = std::vector<std::string>(lines.begin() + 4, lines.end());
    for (size_t i = 0; i < x_size; i++)
        for (size_t j = 0; j < y_size; j++)
            if (lines[i][j] != '.') obstacles.push_back(Node(i, j));
    // Construct graph
    Graph g = Graph(x_size, y_size);
    for (auto n : obstacles) g.remove_node(n);
    return g;
}

/**
 * @brief Return a list of random graph vertices
 *
 * @param g: graph
 * @param num_nodes: number of vertices to return
 * @param allow_repeat: whether multiple same nodes are allowed
 * @param seed: random seed
 * @return std::vector<Node>
 */
inline std::vector<Node> get_random_nodes(Graph& g, int num_nodes,
                                          bool allow_repeat, int seed = 0) {
    if (allow_repeat) {
        auto ret_val = std::vector<Node>();
        srand(seed);
        for (size_t i = 0; i < num_nodes; i++)
            ret_val.push_back(g.nodes[rand() % g.nodes.size()]);
        return ret_val;
    } else {
        assert(g.nodes.size() >= num_nodes);
        auto nodes_copy = g.nodes;
        std::shuffle(nodes_copy.begin(), nodes_copy.end(),
                     std::default_random_engine(seed));
        return std::vector<Node>(nodes_copy.begin(),
                                 nodes_copy.begin() + num_nodes);
    }
}

// TODO fusion with different robot models
struct PathPlanningTask {
    Graph* g;
    std::vector<Node> starts;
    int num_robots;
    int seed;
};

/**
 * @brief A one shot path planning problem
 *
 */
struct OneShotTask : public PathPlanningTask {
    std::vector<Node> goals;
};

/**
 * @brief Get a one shot task
 *
 * @param num_robots:
 * @param g:
 * @param seed:
 * @return OneShotTask
 */
inline OneShotTask get_one_shot_task(int num_robots, Graph& g, int seed) {
    assert(num_robots > 0);
    OneShotTask t;
    t.g = &g;
    t.num_robots = num_robots;
    t.seed = seed;
    srand(seed);
    t.starts = get_random_nodes(g, num_robots, false, rand());
    t.goals = get_random_nodes(g, num_robots, false, rand());
    return t;
}

inline bool check_one_shot_path_feasibility(
    const std::vector<std::vector<Node>>& paths, const OneShotTask& t) {
    const Graph& g = *t.g;
    // Check if number of robots is consistent
    for (auto config : paths)
        if (config.size() != t.num_robots) return false;
    // Check if all nodes are in the given graph
    for (auto config : paths)
        for (auto n : config)
            if (std::find(g.nodes.begin(), g.nodes.end(), n) == g.nodes.end())
                return false;
    // Check if all nodes are not obstacles
    for (auto config : paths)
        for (auto n : config)
            if (g.is_blocked(n)) return false;
    // Check if start and goal are correctly located
    if (paths[0] != t.starts || paths.back() != t.goals) return false;
    // Check if all transitions are valid
    for (size_t i = 0; i < paths.size() - 1; i++)
        for (size_t j = 0; j < t.num_robots; j++) {
            auto it = g.adj_list.find(paths[i][j]);
            if (paths[i][j] != paths[i + 1][j] &&
                std::find(it->second.begin(), it->second.end(),
                          paths[i + 1][j]) == it->second.end())
                return false;
        }
    // Check for collision on a vertex
    for (size_t i = 0; i < paths.size(); i++)
        for (size_t j = 0; j < t.num_robots; j++)
            for (size_t k = j + 1; k < t.num_robots; k++)
                if (paths[i][j] == paths[i][k]) return false;
    // Check for collision on an edge
    for (size_t i = 0; i < paths.size() - 1; i++)
        for (size_t j = 0; j < t.num_robots; j++)
            for (size_t k = j + 1; k < t.num_robots; k++)
                if (paths[i][j] == paths[i + 1][k] &&
                    paths[i][k] == paths[i + 1][j])
                    return false;
    // Passed!
    return true;
}

/**
 * @brief A life long (dynamic) path planning problem, with goals
 * deterministically implemented
 *
 */
struct MultiGoalTask : public PathPlanningTask {
    std::vector<std::vector<Node>> goals;
    int target_goal_reaching_num;
};

/**
 * @brief Get multi goal task
 *
 * @param num_robots:
 * @param g:
 * @param seed:
 * @return MultiGoalTask
 */
inline MultiGoalTask get_multi_goal_task(int num_robots,
                                         int num_goals_per_robot,
                                         int target_goal_reaching_num, Graph& g,
                                         int seed) {
    assert(num_robots > 0 && num_goals_per_robot > 0);
    MultiGoalTask t;
    t.g = &g;
    t.num_robots = num_robots;
    t.seed = seed;
    srand(seed);
    t.starts = get_random_nodes(g, num_robots, false, rand());
    t.goals = std::vector<std::vector<Node>>();
    t.target_goal_reaching_num = target_goal_reaching_num;
    for (size_t i = 0; i < num_robots; i++)
        t.goals.push_back(
            get_random_nodes(g, num_goals_per_robot, true, rand()));
    return t;
}

inline bool check_multi_goal_path_feasibility(
    const std::vector<std::vector<Node>>& paths, const MultiGoalTask& task,
    const Graph& graph) {
    // Check if number of robots is consistent
    for (auto config : paths)
        if (config.size() != task.num_robots) return false;
    // Check if all nodes are in the given graph
    for (auto config : paths)
        for (auto n : config)
            if (std::find(graph.nodes.begin(), graph.nodes.end(), n) ==
                graph.nodes.end())
                return false;
    // Check if all nodes are not obstacles
    for (auto config : paths)
        for (auto n : config)
            if (graph.is_blocked(n)) return false;
    // Check if start is correctly located
    if (paths[0] != task.starts) return false;
    // Check if reached enough number of goals
    int num_finished = 0;
    auto labels = std::vector<size_t>(task.num_robots, 0);
    auto robot_finished = std::vector<bool>(task.num_robots, false);
    for (auto& config : paths) {
        for (size_t i = 0; i < task.num_robots; i++) {
            while (config[i] == task.goals[i][labels[i]] &&
                   labels[i] < task.goals[i].size() - 1) {
                labels[i]++;
                num_finished++;
            }
            if (labels[i] == task.goals[i].size() - 1) {
                if (!robot_finished[i] && config[i] == task.goals[i].back()) {
                    robot_finished[i] = true;
                    num_finished++;
                }
                if (robot_finished[i] && config[i] != task.goals[i].back()) {
                    robot_finished[i] = false;
                    num_finished--;
                }
            }
        }
    }
    int max_finished = 0;
    for (size_t i = 0; i < task.num_robots; i++)
        max_finished += task.goals[i].size();
    if (num_finished < std::min(max_finished, task.target_goal_reaching_num))
        return false;
    // Check if all transitions are valid
    for (size_t i = 0; i < paths.size() - 1; i++)
        for (size_t j = 0; j < task.num_robots; j++) {
            auto it = graph.adj_list.find(paths[i][j]);
            if (paths[i][j] != paths[i + 1][j] &&
                std::find(it->second.begin(), it->second.end(),
                          paths[i + 1][j]) == it->second.end())
                return false;
        }
    // Check for collision on a vertex
    for (size_t i = 0; i < paths.size(); i++)
        for (size_t j = 0; j < task.num_robots; j++)
            for (size_t k = j + 1; k < task.num_robots; k++)
                if (paths[i][j] == paths[i][k]) return false;
    // Check for collision on an edge
    for (size_t i = 0; i < paths.size() - 1; i++)
        for (size_t j = 0; j < task.num_robots; j++)
            for (size_t k = j + 1; k < task.num_robots; k++)
                if (paths[i][j] == paths[i + 1][k] &&
                    paths[i][k] == paths[i + 1][j])
                    return false;
    // Passed!
    return true;
}