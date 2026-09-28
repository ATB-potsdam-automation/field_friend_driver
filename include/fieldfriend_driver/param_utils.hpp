// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <array>
#include <vector>

namespace fieldfriend_driver
{

/// Builds a flattened row-major NxN diagonal covariance matrix (36 = 6x6)
/// from a vector of N diagonal values, mirroring `np.diag(values).reshape(-1)`.
inline std::array<double, 36> diag_covariance_from_vector(const std::vector<double> & values)
{
  std::array<double, 36> result{};
  result.fill(0.0);
  const std::size_t n = values.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t index = i * n + i;
    if (index < result.size()) {
      result[index] = values[i];
    }
  }
  return result;
}

}  // namespace fieldfriend_driver
