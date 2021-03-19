/**
 * @file utils.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief Useful functions not associated with the problem
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

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief Given a time point, get the time elapsed since the time point
 *
 * @param start_time
 * @return double
 */
inline double time_elapsed(
    std::chrono::time_point<std::chrono::high_resolution_clock>& start_time) {
    return double(static_cast<long long int>(
               std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - start_time)
                   .count())) /
           1000000;
}

/**
 * @brief Print a 1d vector
 *
 * @tparam T: vector type with printable elements
 * @param t
 */
template <typename T>
inline void print_vector(const T& t) {
    for (auto i = t.begin(); i != t.end(); ++i)
        std::cout << std::setw(16) << *i;
    std::cout << std::endl;
}

/**
 * @brief Show or update a progress bar on the screen
 *
 * @param progress: 0 to 1
 * @param bar_width: width of the bar, default to 70
 */
inline void progress_bar(double progress, int bar_width = 70) {
    std::cout.flush();
    std::cout << "[";
    int pos = bar_width * progress;
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " \%\r";
}

/**
 * @brief All platform safe helper function for getting date and time, intended
 * to work with
 *      std::string current_date_time()
 */
inline std::tm localtime_xp(std::time_t timer) {
    std::tm bt{};
#if defined(__unix__)
    localtime_r(&timer, &bt);
#elif defined(_MSC_VER)
    localtime_s(&bt, &timer);
#else
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);
    bt = *std::localtime(&timer);
#endif
    return bt;
}

/**
 * @brief Return current date and time, which can be used to generate an output
 * folder
 *
 * @return std::string
 */
inline std::string current_date_time() {
    std::time_t now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto tm = localtime_xp(now);
    auto time_obj = std::put_time(&tm, "%F %T");
    std::stringstream ss;
    ss << time_obj;
    auto str = ss.str();
    str[10] = '-';
    str[13] = '-';
    str[16] = '-';
    return str;
}

/**
 * @brief Generate a CSV file with one or more columns.
 *
 * @param filename: csv file path and name (without extension)
 * @param dataset: data to write. Each column of data is represented by the pair
 * <column name, column data> as std::pair<std::string, std::vector<double>> The
 * dataset is represented as a vector of these columns. Note that all columns
 * should be the same sizeEach column of data is represented by the pair <column
 * name, column data> as std::pair<std::string, std::vector<double>> The dataset
 * is represented as a vector of these columns. Note that all columns should be
 * the same size
 */
inline void write_csv(
    std::string file_dir,
    std::vector<std::pair<std::string, std::vector<double>>>& dataset) {
    // Create an output filestream object
    std::ofstream my_file(file_dir + ".csv");
    // Write headers
    for (auto iter = dataset.begin(); iter != dataset.end();) {
        my_file << iter->first;
        if (iter++ != dataset.end()) my_file << ",";
    }
    my_file << "\n";
    // Write data
    for (int i = 0; i < dataset.begin()->second.size(); i++) {
        for (auto iter = dataset.begin(); iter != dataset.end(); iter) {
            auto value = iter->second[i];
            my_file << value;
            if (iter++ != dataset.end()) my_file << ",";
        }
        my_file << "\n";
    }
    my_file.close();
}