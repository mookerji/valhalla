#include <benchmark/benchmark.h>
#include <iostream>
#include <string>
#include <unordered_map>

#include "midgard/small_flat_map.h"

#include <absl/container/flat_hash_map.h>

namespace {

// Inserts

constexpr float kMaxRange = 1024;

static void BM_VFlatMapInsert(benchmark::State& state) {
  const int size = state.range(0);
  valhalla::midgard::SmallFlatMap<std::string, std::string> map;
  for (auto _ : state) {
    int map_size = size;
    while (map_size--) {
      map[std::to_string(map_size)] = std::to_string(map_size);
    }
  }
}

BENCHMARK(BM_VFlatMapInsert)->RangeMultiplier(2)->Range(1, kMaxRange);

static void BM_AbslFlatMapInsert(benchmark::State& state) {
  const int size = state.range(0);
  absl::flat_hash_map<std::string, std::string> map;
  for (auto _ : state) {
    int map_size = size;
    while (map_size--) {
      map[std::to_string(map_size)] = std::to_string(map_size);
    }
  }
}

BENCHMARK(BM_AbslFlatMapInsert)->RangeMultiplier(2)->Range(1, kMaxRange);

static void BM_STLUnorderedMapInsert(benchmark::State& state) {
  const int size = state.range(0);
  std::unordered_map<std::string, std::string> map;
  for (auto _ : state) {
    int map_size = size;
    while (map_size--) {
      map[std::to_string(map_size)] = std::to_string(map_size);
    }
  }
}

BENCHMARK(BM_STLUnorderedMapInsert)->RangeMultiplier(2)->Range(1, kMaxRange);

// Accesses

static void BM_VFlatMapAccess(benchmark::State& state) {
  const int size = state.range(0);
  valhalla::midgard::SmallFlatMap<std::string, std::string> map;
  int map_size = size;
  while (map_size--) {
    map[std::to_string(map_size)] = std::to_string(map_size);
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(map[std::to_string(size / 2)]);
  }
}

BENCHMARK(BM_VFlatMapAccess)->RangeMultiplier(2)->Range(1, kMaxRange);

static void BM_AbslFlatMapAccess(benchmark::State& state) {
  const int size = state.range(0);
  absl::flat_hash_map<std::string, std::string> map;
  int map_size = size;
  while (map_size--) {
    map[std::to_string(map_size)] = std::to_string(map_size);
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(map[std::to_string(size / 2)]);
  }
}

BENCHMARK(BM_AbslFlatMapAccess)->RangeMultiplier(2)->Range(1, kMaxRange);

static void BM_STLUnorderedMapAccess(benchmark::State& state) {
  const int size = state.range(0);
  std::unordered_map<std::string, std::string> map;
  int map_size = size;
  while (map_size--) {
    map[std::to_string(map_size)] = std::to_string(map_size);
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(map[std::to_string(size / 2)]);
  }
}

BENCHMARK(BM_STLUnorderedMapAccess)->RangeMultiplier(2)->Range(1, kMaxRange);

} // namespace

BENCHMARK_MAIN();
