#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/common/yaml/yaml_read_archive.h"

/// Per-scenario environment definition for the sampling-C3 demos: named
/// scenario, static obstacles, and the obstacle-proximity cost shaping.
/// Obstacles are planar discs [x, y, radius] used analytically by the
/// planner's exponential proximity penalty; obstacle_model optionally names
/// a static SDF the simulator adds so the physics matches the cost.
struct SamplingC3ScenarioParams {
  std::string scenario_name;
  std::vector<std::vector<double>> obstacles;  // each [x, y, radius]
  double obstacle_cost_weight{0.0};
  double obstacle_cost_decay{0.02};
  std::optional<std::string> obstacle_model;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(scenario_name));
    a->Visit(DRAKE_NVP(obstacles));
    a->Visit(DRAKE_NVP(obstacle_cost_weight));
    a->Visit(DRAKE_NVP(obstacle_cost_decay));
    a->Visit(DRAKE_NVP(obstacle_model));
  }
};
