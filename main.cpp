/*
 * Copyright (c) 2020, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file incorporates work covered by the following copyright and permission notice:  
 *
 * Copyright 2020 Jee W. Choi, Marat Dukhan, and Xing Liu
 * Permission is hereby granted, free of charge, to any person obtaining a copy of 
 * this software and associated documentation files (the "Software"), to deal in 
 * the Software without restriction, including without limitation the rights to use, 
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
 * Software, and to permit persons to whom the Software is furnished to do so, subject 
 * to the following conditions:
 * The above copyright notice and this permission notice shall be included in all 
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <cstddef>
#include "CLI11.hpp"
#include <mpi.h>
#include "bench.hpp"

int main(int argc, char **argv)
{
    CLI::App app{"Run a benchmark with configurable arithmetic intensity and MPI imbalance"};

    size_t base_internal_iterations = 1;
    app.add_option("--base-internal-iterations", base_internal_iterations,
                   "How many iterations to perform in the inner loop, for the "
                   "fast set of ranks (all ranks if there is no imbalance).",
                   true);

    double imbalance_multiplier = 1;
    app.add_option("--slowdown", imbalance_multiplier,
                   "When imbalance is present, this specifies the amount of work for slow "
                   "ranks to perform, as a factor of the amount of work the fast ranks "
                   "perform.",
                   true);

    size_t slow_rank_count = 0;
    app.add_option("--slow-ranks", slow_rank_count,
                   "The number of ranks to run with extra work for an imbalanced load.",
                   true);

    size_t floats = 1024 * 1024 * 64;
    app.add_option("--floats", floats,
                   "The number of floating-point numbers per rank in the problem array.",
                   true);

    auto verbosity = app.add_flag("-v,--verbose", "Run in verbose mode");

    auto is_single_precision = app.add_flag("-s,--single-precision", "Run in single-precision mode");

    auto do_list_intensities = app.add_flag("-l,--list",
                                            "List the available arithmetic intensity levels "
                                            "for use with the --benchmarks option");

    size_t iteration_count = 5;
    app.add_option("-i,--iterations", iteration_count,
                   "The number of times to run each phase of the benchmark",
                   true);

    std::vector<double> requested_intensities;
    app.add_option("-b,--benchmarks", requested_intensities,
                   "List of benchmark intensity variants to run (all are run if this option is not specified)");

    intmax_t start_time = 0;
    app.add_option("--start-time", start_time,
                   "Time at which the benchmark will start, in seconds since "
                   "the system clock's epoch. Start immediately by default, "
                   "or if the provided time is in the past");

    MPI_Init(&argc, &argv);
    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    try {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError &e) {
        MPI_Finalize();
        if (rank == 0) {
            return (app).exit(e);
        }
        else {
            return 0;
        }
    }

    auto all_benchmarks = get_benchmarks(*is_single_precision);
    if (*do_list_intensities) {
        if (rank == 0) {
            for (auto bench : all_benchmarks) {
                std::cout << bench.first << std::endl;
            }
        }
        MPI_Finalize();
        return 0;
    }

    std::vector<size_t> enabled_benchmarks;
    if (requested_intensities.empty()) {
        for (auto bench : all_benchmarks) {
            enabled_benchmarks.push_back(bench.second);
        }
    }
    else {
        for (auto intensity : requested_intensities) {
            auto found_bench = std::find_if(
                all_benchmarks.begin(), all_benchmarks.end(),
                [&intensity](const decltype(all_benchmarks)::value_type &bench) {
                    return intensity == bench.first;
                });

            if (found_bench == all_benchmarks.end()) {
                if (rank == 0) {
                    std::cerr << "Error: Benchmark with requested intensity "
                              << intensity << " does not exist." << std::endl;
                }
                MPI_Finalize();
                return 0;
            }
            enabled_benchmarks.push_back(found_bench->second);
        }
    }

    run_benchmarks({ slow_rank_count, base_internal_iterations, imbalance_multiplier, floats,
                     verbosity->count(), *is_single_precision, iteration_count,
                     enabled_benchmarks, start_time });

    MPI_Finalize();

    return 0;
}
