// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once

#include <stdexcept>
#include <string>

namespace hydra {

enum class ReconstructionBackend { Cpu, NvbloxCpu, NvbloxGpu, Both };

inline ReconstructionBackend parseReconstructionBackend(const std::string& value) {
  if (value == "cpu") return ReconstructionBackend::Cpu;
  if (value == "nvblox-cpu" || value == "nvblox") return ReconstructionBackend::NvbloxCpu;
  if (value == "nvblox-gpu") return ReconstructionBackend::NvbloxGpu;
  if (value == "both") return ReconstructionBackend::Both;
  throw std::invalid_argument("reconstruction.backend must be cpu, nvblox-cpu, both, or nvblox-gpu: " + value);
}

// Do not silently run CPU reconstruction when GPU reconstruction was requested.
inline void requireAvailableReconstructionBackend(const std::string& value,
                                                  bool gpu_stage_available = false) {
  const auto backend = parseReconstructionBackend(value);
  if (backend != ReconstructionBackend::Cpu && !gpu_stage_available) {
    throw std::runtime_error("GPU reconstruction requires HYDRA_ENABLE_NVBLOX_STAGE=ON at build time.");
  }
}

}  // namespace hydra
