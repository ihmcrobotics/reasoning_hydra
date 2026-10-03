// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once
#include <memory>
#include <array>
#include <vector>
#include <cstdint>
#include <cstddef>
#include "hydra/reconstruction/tsdf_geometry_delta.h"
namespace hydra {
struct InputData;
struct SemanticIntegrator;
class VolumetricMap;
// Geometry-only staging boundary. Not yet a scene-graph reconstruction adapter.
class NvbloxReconstructionStage {
 public:
  explicit NvbloxReconstructionStage(float voxel_size,
      std::shared_ptr<SemanticIntegrator> semantic_policy = nullptr,
      float integration_range_m = 3.5f);
  ~NvbloxReconstructionStage();
  void integrate(const InputData& input);
  // Release distant voxel blocks to the nvblox pool; archived meshes live in Hydra.
  void clearOutsideRadius(const Eigen::Vector3f& center, float radius);
  void integrateColor(const InputData& input);
  void updateMesh();
  void exportMesh(VolumetricMap& destination, const Eigen::Vector3f& center, float active_radius) const;
  std::size_t validateGvd(uint64_t timestamp_ns, const VolumetricMap::Config& config,
                          const Eigen::Vector3f& center = Eigen::Vector3f::Zero(),
                          float active_radius = -1.0f);
  // Full voxel export, optionally including independently fused RGB.
  // Missing RGB is neutral gray. Does not transfer semantics or deletions.
  std::size_t exportTsdfGeometry(VolumetricMap& destination, bool include_color = false) const;
  TsdfGeometryDelta exportTsdfDelta(VolumetricMap& destination);
  std::size_t numTriangles() const;
  std::size_t numValidationPlaces() const;
  std::size_t numActiveValidationPlaces() const;
  std::size_t numFrontendValidationPlaces() const;
  // Diagnostic: use existing backend merge criteria on a clone, never the live graph.
  std::size_t countValidationMergeProposals(double position_tolerance, double distance_tolerance) const;
  std::size_t numBlocks() const;
  // Diagnostic readback; not called in the live integration loop.
  std::vector<std::array<uint8_t, 3>> meshColors() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
