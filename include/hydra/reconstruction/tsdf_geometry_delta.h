#pragma once
#include "hydra/reconstruction/volumetric_map.h"
#include <set>
#include <array>
#include <stdexcept>

namespace hydra {
struct TsdfGeometryDelta {
  BlockIndices changed;
  // Retained in the destination until the caller explicitly archives them.
  BlockIndices removed;
};

// A geometry-only mirror; each instance belongs to one destination map.
// Full snapshot comparison for correctness first, not reduced GPU transfer yet.
class TsdfGeometryMirror {
 public:
  TsdfGeometryDelta update(const VolumetricMap& snapshot, VolumetricMap& destination) {
    if (&snapshot == &destination || snapshot.hasSemantics() || destination.hasSemantics() ||
        snapshot.getTrackingLayer() || destination.getTrackingLayer() ||
        snapshot.config.voxel_size != destination.config.voxel_size ||
        snapshot.config.voxels_per_side != destination.config.voxels_per_side)
      throw std::invalid_argument("Incompatible geometry mirror maps");
    if (destination_ && destination_ != &destination)
      throw std::invalid_argument("A TSDF mirror cannot change destinations");
    destination_ = &destination;
    TsdfGeometryDelta delta;
    std::set<std::array<int,3>> current;
    for (const auto& idx : snapshot.getTsdfLayer().allocatedBlockIndices()) {
      current.insert({idx.x(),idx.y(),idx.z()});
      const auto& source = snapshot.getTsdfLayer().getBlock(idx);
      const bool allocated = destination.allocateBlock(idx);
      auto target = destination.getBlock(idx).tsdf;
      bool changed = allocated;
      for (size_t i=0;i<source.numVoxels();++i) {
        const auto& from = source.getVoxel(i);
        auto& to = target->getVoxel(i);
        // Missing/unobserved source voxels are not evidence that a surface vanished.
        if (from.weight <= 0) continue;
        if (to.distance == from.distance && to.weight == from.weight) continue;
        to.distance=from.distance;
        to.weight=from.weight;
        changed=true;
      }
      if (changed) { target->setUpdated(); delta.changed.push_back(idx); }
    }
    for (const auto& idx : previous_) {
      if (!current.count(idx)) delta.removed.emplace_back(idx[0],idx[1],idx[2]);
    }
    previous_ = std::move(current);
    return delta;
  }
 private:
  const VolumetricMap* destination_ = nullptr;
  std::set<std::array<int,3>> previous_;
};
}
