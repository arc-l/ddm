/**
 * @file main.cpp
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief the DDM algorithm implementation
 (https://ieeexplore.ieee.org/document/8962218)
 * @version 0.1
 * @date 2021-03-19
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

#include <iostream>
#include "ddm.h"

int main() {
    load_database();  // Load database file. Takes around 5 seconds.
    auto g = get_low_res_graph(
        6, 6, 0.1);  // Construct a graph: 6x6 with ~10% obstacles
    auto t = get_one_shot_task(
        6, g,
        0);  // Randomly generate a one shot task with 6 robots and graph g.
    auto solver = DdmSolver();        // Initialize solver
    auto paths = solver.solve(t, g);  // Solve the problem

    // Output solution
    std::cout << g << std::endl;
    std::cout << "Start configuration:" << std::endl;
    print_vector(t.starts);
    std::cout << "Goal configuration:" << std::endl;
    print_vector(t.goals);
    std::cout << "Solution:" << std::endl;
    for (auto& path : paths) print_vector(path);
}
