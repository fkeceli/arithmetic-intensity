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
#include "bench.hpp"
#include <mpi.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <thread>
#include <vector>
#include "geopm.h"
#include "geopm_error.h"

extern "C" void sumsq_0(const double *data, size_t length);
extern "C" void sumsq_1(const double *data, size_t length);
extern "C" void sumsq_2(const double *data, size_t length);
extern "C" void sumsq_4(const double *data, size_t length);
extern "C" void sumsq_8(const double *data, size_t length);
extern "C" void sumsq_16(const double *data, size_t length);
extern "C" void sumsq_32(const double *data, size_t length);
extern "C" void sumsq_64(const double *data, size_t length);
extern "C" void sumsq_128(const double *data, size_t length);

extern "C" void sumsqf_0(const float *data, size_t length);
extern "C" void sumsqf_1(const float *data, size_t length);
extern "C" void sumsqf_2(const float *data, size_t length);
extern "C" void sumsqf_4(const float *data, size_t length);
extern "C" void sumsqf_8(const float *data, size_t length);
extern "C" void sumsqf_16(const float *data, size_t length);
extern "C" void sumsqf_32(const float *data, size_t length);
extern "C" void sumsqf_64(const float *data, size_t length);
extern "C" void sumsqf_128(const float *data, size_t length);

extern "C" uint32_t SUMSQ_BASE_STRIDE;

template<typename Precision>
using Benchmark = void (*)(const Precision *, size_t);

namespace {
const std::map<size_t, Benchmark<float>> fp_benchmarks {
    {0, sumsqf_0},
    {1, sumsqf_1},
    {2, sumsqf_2},
    {4, sumsqf_4},
    {8, sumsqf_8},
    {16, sumsqf_16},
    {32, sumsqf_32},
    {64, sumsqf_64},
    {128, sumsqf_128}
};

const std::map<size_t, Benchmark<double>> dp_benchmarks {
    {0, sumsq_0},
    {1, sumsq_1},
    {2, sumsq_2},
    {4, sumsq_4},
    {8, sumsq_8},
    {16, sumsq_16},
    {32, sumsq_32},
    {64, sumsq_64},
    {128, sumsq_128}
};

struct NullStream : public std::ostream
{
    template<typename T> NullStream &operator<<(T const &)
    {
        return *this;
    }
} null_stream;

class Logger
{
    public:
        Logger(int rank, int verbosity)
            : m_rank(rank)
            , m_verbosity(verbosity)
        {}

        std::ostream& stream(int message_verbosity, bool all_ranks = false)
        {
            if ((m_rank == 0 || all_ranks) && m_verbosity >= message_verbosity) {
                return std::cout;
            }
            else {
                return null_stream;
            }
        }

    private:
        int m_rank;
        int m_verbosity;
};

using AlignedBuffer = std::unique_ptr<void, void (*)(void *)>;
AlignedBuffer make_aligned_buffer(size_t alignment, size_t size)
{
    return AlignedBuffer(aligned_alloc(alignment, size),
                         [](void *buf) { free(buf); });
}

[[noreturn]] static void throw_geopm_error(const std::string& context, int geopm_error)
{
    char error_buf[1024];
    geopm_error_message(geopm_error, error_buf, sizeof error_buf);
    std::ostringstream message;
    message << context << ": geopm error: " << error_buf;
    throw std::runtime_error(message.str());
}

double get_compute_intensity(size_t fmas_per_load, size_t bytes_per_value)
{
    return fmas_per_load * 2.0 / bytes_per_value;
}

// Run the benchmark from function 'bench'
template<typename Precision>
static void time_bench(const std::string &bench_name, const BenchmarkConfig &config,
                       Benchmark<Precision> bench, int rank, int rank_count,
                       const AlignedBuffer &data, size_t array_length_per_rank,
                       size_t fmas_per_load, Logger &logger)
{
    double aggregated_wall_time = 0;
    double aggregated_exec_time = 0;
    const size_t internal_iterations = (rank < config.slow_rank_count)
                                     ? config.imbalance_multiplier
                                     : 1;
    uint64_t region_id = 0;
    const double ops_per_byte = get_compute_intensity(fmas_per_load, sizeof(Precision));
    std::ostringstream region_name_oss;
    region_name_oss << "intensity_" << ops_per_byte;

    int geopm_error = geopm_prof_region(region_name_oss.str().c_str(),
                                        GEOPM_REGION_HINT_UNKNOWN, &region_id);
    if (geopm_error) {
        throw_geopm_error("Creating profile region " + bench_name, geopm_error);
    }

    logger.stream(0) << bench_name << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < config.iteration_count; ++i) {
        geopm_error = geopm_prof_epoch();
        if (geopm_error) {
            throw_geopm_error("Starting new epoch", geopm_error);
        }

        geopm_error = geopm_prof_enter(region_id);
        if (geopm_error) {
            throw_geopm_error("Entering region " + bench_name, geopm_error);
        }
        for (size_t j = 0; j < internal_iterations; ++j) {
            bench((Precision *)data.get(), array_length_per_rank);
        }

        // Ensure that we wait for any imbalance.
        MPI_Barrier(MPI_COMM_WORLD);

        geopm_error = geopm_prof_exit(region_id);
        if (geopm_error) {
            throw_geopm_error("Exiting region " + bench_name, geopm_error);
        }
    }

    std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start;
    double wall_time = diff.count();

    const double total_iterations =
        config.iteration_count *               // external loops
        (rank_count - config.slow_rank_count + // regular rank internal loops
         config.slow_rank_count * config.imbalance_multiplier); // slow rank internal loops
    const double bytes_loaded = array_length_per_rank * sizeof(Precision) * total_iterations;
    const double total_ops = ops_per_byte * bytes_loaded;
    logger.stream(0)
        << "  Elapsed time: " << wall_time << " s\n"
        << "   Data loaded: " << bytes_loaded << " B\n"
        << "     Bandwidth: " << bytes_loaded / wall_time << " B/s\n"
        << "     Intensity: " << ops_per_byte << " FLOP/B\n"
        << "    Operations: " << total_ops << " FLOP\n"
        << "   Performance: " << total_ops / wall_time << " FLOP/s\n";
}
} // Namespace (anonymous)

std::vector<std::pair<double, size_t> > get_benchmarks(bool is_single_precision)
{
    std::vector<std::pair<double, size_t> > benchmarks;
    for (const auto& bench : fp_benchmarks)
    {
        benchmarks.emplace_back(
            get_compute_intensity(
                bench.first, is_single_precision ? sizeof(float) : sizeof(double)),
            bench.first);
    }
    return benchmarks;
}

// Run all benchmarks
void run_benchmarks(const BenchmarkConfig& config)
{
    int i;
    int rank_count;
    int rank;

    MPI_Comm_size(MPI_COMM_WORLD, &rank_count);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    // Array stride is the step size of each iteration internal to the benchmarks.
    // Contributing factors are:
    //     precision: Half as many double-precision floats fit in a vectored register
    //                as single-precision floats fit in the same register type
    //   base stride: This depends on which type of vectored operations are being
    //                used. E.g. The AVX512 benchmark implementation walks over the
    //                data in a stride twice as long as the AVX2 implementation.
    const size_t array_stride = SUMSQ_BASE_STRIDE * (config.is_single_precision ? 2 : 1);

    // Round up the requested array length so that the data fits the stride of
    // the benchmark loops, and we don't need to do special handling at
    // boundaries in the benchmark assembly implementations.
    size_t floats_per_rank = (config.floats_per_rank / array_stride +
                              (config.floats_per_rank % array_stride != 0)) *
                             array_stride;
    Logger logger(rank, config.verbosity);

    std::mt19937 rand_engine;
    rand_engine.seed(1);

    AlignedBuffer data(make_aligned_buffer(64, 0));
    if (config.is_single_precision) {
        auto buffer = make_aligned_buffer(64, floats_per_rank * sizeof(float));
        if (buffer) {
            std::uniform_real_distribution<float> distribution(0, 1);
            std::generate((float *)buffer.get(), (float *)buffer.get() + floats_per_rank, std::bind(distribution, rand_engine));
        }
        data = std::move(buffer);
    }
    else {
        auto buffer = make_aligned_buffer(64, floats_per_rank * sizeof(double));
        if (buffer) {
            std::uniform_real_distribution<double> distribution(0, 1);
            std::generate((double *)buffer.get(), (double *)buffer.get() + floats_per_rank, std::bind(distribution, rand_engine));
        }
        data = std::move(buffer);
    }

    if (!data) {
        std::cerr << "Unable to allocate all needed memory" << std::endl;
        return;
    }


    using Clock = std::chrono::system_clock;
    std::chrono::time_point<Clock> start_time(std::chrono::seconds{ config.start_time });
    if (start_time >= Clock::now()) {
        logger.stream(0) << "Wait "
                         << std::chrono::duration_cast<std::chrono::seconds>(
                                start_time - Clock::now())
                                .count()
                         << " seconds to start" << std::endl;
    }

    while (start_time >= Clock::now()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //////// Run the benchmarks ////////
    logger.stream(0)
        << "Array of " << floats_per_rank << " "
        << (config.is_single_precision ? "single" : "double") << "-precision floats per rank\n"
        << "ranks: " << rank_count
        << "\nslow ranks: " << config.slow_rank_count
        << ", slowdown factor: " << config.imbalance_multiplier << std::endl;

    if (config.is_single_precision) {
        for (size_t bench_size : config.enabled_fmas_per_load) {
            auto benchmark = fp_benchmarks.find(bench_size);
            if (benchmark != fp_benchmarks.end()) {
                std::ostringstream bench_name;
                bench_name << "sumsqf_" << bench_size;
                time_bench(bench_name.str(), config, benchmark->second, rank, rank_count, data,
                           floats_per_rank, bench_size, logger);
            }
        }
    } else {
        for (size_t bench_size : config.enabled_fmas_per_load) {
            auto benchmark = dp_benchmarks.find(bench_size);
            if (benchmark != dp_benchmarks.end()) {
                std::ostringstream bench_name;
                bench_name << "sumsq_" << bench_size;
                time_bench(bench_name.str(),config, benchmark->second, rank, rank_count, data,
                           floats_per_rank, bench_size, logger);
            }
        }
    }
}
