#pragma once
#include "hydra/reconstruction/reconstruction_adapter.h"
#include "hydra/reconstruction/nvblox_reconstruction_stage.h"
#include "hydra/input/input_data.h"
#include <set>
#include <array>

namespace hydra {
// First runtime adapter: GPU depth/color fusion, CPU semantic association and meshing.
// Local GPU voxel window; Hydra retains archived mesh through its output pipeline.
class NvbloxReconstructionAdapter final : public ReconstructionAdapter {
 public:
  NvbloxReconstructionAdapter(const VolumetricMap::Config& map,
                             const ProjectiveIntegratorConfig& tsdf,
                             const MeshIntegratorConfig& mesh,
                             float active_radius, float integration_range_m = 3.5f, bool gpu_mesh = false)
      : gpu_(map.voxel_size, tsdf.semantic_integrator.create(), integration_range_m),
        appearance_(tsdf), mesh_(mesh), active_radius_(active_radius), gpu_mesh_(gpu_mesh) {}

  BlockIndices integrate(const InputData& input, VolumetricMap& map) override {
    center_ = input.world_T_body.translation().cast<float>();
    gpu_.integrate(input);
    gpu_.integrateColor(input);
    if (active_radius_ > 0.0f) {
      gpu_.clearOutsideRadius(input.world_T_body.translation().cast<float>(), active_radius_);
    }
    // Keep CPU blocks until ReconstructionModule emits their archival events.
    // Missing GPU blocks must not erase geometry before that output is consumed.
    VolumetricMap snapshot(map.config, false);
    gpu_.exportTsdfGeometry(snapshot, true);
    BlockIndices touched;
    for (const auto& index : snapshot.getTsdfLayer().allocatedBlockIndices()) {
      const Eigen::Vector3f center =
          (index.cast<float>() + Eigen::Vector3f::Constant(.5f)) * map.blockSize();
      if (active_radius_ > 0 && (center - input.world_T_body.translation().cast<float>()).norm() > active_radius_)
        continue;
      const auto& source = snapshot.getTsdfLayer().getBlock(index);
      map.allocateBlock(index);
      auto destination = map.getBlock(index).tsdf;
      bool changed=false;
      for(size_t i=0;i<source.numVoxels();++i) {
        const auto& from=source.getVoxel(i);
        if(from.weight<=0)continue;
        auto& to=destination->getVoxel(i);
        if(to.distance!=from.distance || to.weight!=from.weight || to.color!=from.color) {
          to.distance=from.distance;to.weight=from.weight;to.color=from.color;changed=true;
        }
      }
      if(changed){destination->setUpdated();touched.push_back(index);}
    }
    // GPU color uses independent observation confidence. Only associate semantics
    // here: CPU TSDF-weighted RGB blending would corrupt the transferred color.
    const auto appearance_blocks=appearance_.updateAppearanceBlocks(
        map.getTsdfLayer().allocatedBlockIndices(),input,map,false);
    std::set<std::array<int,3>> seen;
    BlockIndices all;
    for(const auto& list : {touched, appearance_blocks}) for(const auto& i:list)
      if(seen.insert({i.x(),i.y(),i.z()}).second) all.push_back(i);
    return all;
  }
  void mesh(VolumetricMap& map) override {
    if (gpu_mesh_) {
      gpu_.updateMesh();
      gpu_.exportMesh(map, center_, active_radius_);
    } else {
      mesh_.generateMesh(map,true,true);
    }
  }
 private:
  NvbloxReconstructionStage gpu_;
  ProjectiveIntegrator appearance_;
  MeshIntegrator mesh_;
  float active_radius_;
  bool gpu_mesh_;
  Eigen::Vector3f center_ = Eigen::Vector3f::Zero();
};
}
