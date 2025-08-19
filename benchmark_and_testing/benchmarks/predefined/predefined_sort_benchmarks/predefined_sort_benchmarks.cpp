// Unit test utils
#include <predefined_utils/benchmark/fixtures/sort_bench_fixtures.h>

// Day 2 sort library
#include <lib3611/w1d1_2_sort/counting_sort.h>
#include <lib3611/w1d1_2_sort/binary_sort.h>
#include <lib3611/w1d1_2_sort/radix_sort.h>
#include <lib3611/w1d1_2_sort/custom_aa_sort.h>

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <memory>
#include <algorithm>


// Qualify predefined fixtures
using namespace dte3611::predef::benchmarking::sort::fixtures;

namespace alg = dte3611::sort::algorithms;

// Define benchmark fixtures for sorting of a sorted collection
BENCHMARK_DEFINE_F(SortedIntColF, stlSort)
(benchmark::State& st)
{
  for (auto const& _ : st) std::sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(SortedIntColF, countingSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::counting_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(SortedIntColF, binarySort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::binary_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(SortedIntColF, radixSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::radix_sort(m_data.begin(), m_data.end());
}


BENCHMARK_DEFINE_F(SortedIntColF, AndAlxSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::custom_aa_sort(m_data.begin(), m_data.end());
}

// Define benchmark fixtures for sorting of a reverse sorted collection
BENCHMARK_DEFINE_F(ReverseIntColF, stlSort)
(benchmark::State& st)
{
  for (auto const& _ : st) std::sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(ReverseIntColF, countingSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::counting_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(ReverseIntColF, binarySort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::binary_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(ReverseIntColF, radixSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::radix_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(ReverseIntColF, AndAlxSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::custom_aa_sort(m_data.begin(), m_data.end());
}


// Define benchmark fixtures for sorting of a organpipe sorted collection
BENCHMARK_DEFINE_F(OrganpipeIntColF, stlSort)
(benchmark::State& st)
{
  for (auto const& _ : st) std::sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(OrganpipeIntColF, countingSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::counting_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(OrganpipeIntColF, binarySort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::binary_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(OrganpipeIntColF, radixSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::radix_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(OrganpipeIntColF, AndAlxSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::custom_aa_sort(m_data.begin(), m_data.end());
}

// Define benchmark fixtures for sorting of a sorted and rotated (by one index)
// collection
BENCHMARK_DEFINE_F(RotatedIntColF, stlSort)
(benchmark::State& st)
{
  for (auto const& _ : st) std::sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(RotatedIntColF, countingSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::counting_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(RotatedIntColF, binarySort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::binary_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(RotatedIntColF, radixSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::radix_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(RotatedIntColF, AndAlxSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::custom_aa_sort(m_data.begin(), m_data.end());
}

// Define benchmark fixtures for sorting of a sorted and random collection of 0s
// and 1s
BENCHMARK_DEFINE_F(Random01IntColF, stlSort)
(benchmark::State& st)
{
  for (auto const& _ : st) std::sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(Random01IntColF, countingSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::counting_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(Random01IntColF, binarySort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::binary_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(Random01IntColF, radixSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::radix_sort(m_data.begin(), m_data.end());
}

BENCHMARK_DEFINE_F(Random01IntColF, AndAlxSort)
(benchmark::State& st)
{
  for (auto const& _ : st) alg::custom_aa_sort(m_data.begin(), m_data.end());
}



// Register Benchmark : benchmark sorting of a sorted collection using different
// algorithms
BENCHMARK_REGISTER_F(SortedIntColF, stlSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(SortedIntColF, countingSort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(SortedIntColF, binarySort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(SortedIntColF, radixSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(SortedIntColF, AndAlxSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);



// Register Benchmark : benchmark sorting of a reversed collection using
// different algorithms
BENCHMARK_REGISTER_F(ReverseIntColF, stlSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(ReverseIntColF, countingSort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(ReverseIntColF, binarySort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(ReverseIntColF, radixSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(ReverseIntColF, AndAlxSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

// Register Benchmark : benchmark sorting of a organpipe-ordered collection
// using different algorithms
BENCHMARK_REGISTER_F(OrganpipeIntColF, stlSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(OrganpipeIntColF, countingSort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(OrganpipeIntColF, binarySort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(OrganpipeIntColF, radixSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(OrganpipeIntColF, AndAlxSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

// Register Benchmark : benchmark sorting of a rotated collection using
// different algorithms
BENCHMARK_REGISTER_F(RotatedIntColF, stlSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(RotatedIntColF, countingSort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(RotatedIntColF, binarySort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(RotatedIntColF, radixSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(RotatedIntColF, AndAlxSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

// Register Benchmark : benchmark sorting of a random collection using different
// algorithms
BENCHMARK_REGISTER_F(Random01IntColF, stlSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_REGISTER_F(Random01IntColF, countingSort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(Random01IntColF, binarySort)
  ->RangeMultiplier(10)
  ->Range(1e1, 1e6);

BENCHMARK_REGISTER_F(Random01IntColF, radixSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);


BENCHMARK_REGISTER_F(Random01IntColF, AndAlxSort)
  ->RangeMultiplier(10)
  ->Range(1e4, 1e6);

BENCHMARK_MAIN();
