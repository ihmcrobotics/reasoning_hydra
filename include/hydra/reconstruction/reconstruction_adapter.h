#pragma once

#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/reconstruction/mesh_integrator.h"

namespace hydra {

// Adapters update Hydra-compatible layers. The module retains responsibility for
// output queues, semantic feature lifetime, footprint carving and archival.
// A GPU adapter must export semantic and distance data, not only RGB triangles.
class ReconstructionAdapter {
 public:
  virtual ~ReconstructionAdapter() = default;
  virtual BlockIndices integrate(const InputData& input, VolumetricMap& map) = 0;
  virtual void mesh(VolumetricMap& map) = 0;
};

class CpuReconstructionAdapter final : public ReconstructionAdapter {
 public:
  CpuReconstructionAdapter(const ProjectiveIntegratorConfig& tsdf,
                           const MeshIntegratorConfig& mesh)
      : tsdf_(tsdf), mesh_(mesh) {}

  BlockIndices integrate(const InputData& input, VolumetricMap& map) override {
    return tsdf_.updateMap(input, map);
  }

  void mesh(VolumetricMap& map) override {
    mesh_.generateMesh(map, true, true);
  }

 private:
  ProjectiveIntegrator tsdf_;
  MeshIntegrator mesh_;
};

}  // namespace hydra
