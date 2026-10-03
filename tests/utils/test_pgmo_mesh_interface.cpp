// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include <gtest/gtest.h>

#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/pgmo_mesh_interface.h"
#include "kimera_pgmo/compression/delta_compression.h"

namespace hydra {
TEST(PgmoMeshLayerInterface, SelectedViewMatchesCopiedLayer) {
  MeshLayer layer(1.0f);
  // Enough blocks to exercise hash-table rehashing and mixed updated/archive flags.
  for (int i = 0; i < 200; ++i) {
    auto& block = layer.allocateBlock(BlockIndex(i, i % 7, 0));
    block.updated = i % 3 != 0;
    block.points = {{float(i), 0, 0}, {float(i), 0.5f, 0}, {float(i), 0, 0.5f}};
    block.colors.resize(3);
    block.labels = {1, 2, 3};
    block.semantic_features.assign(3, Eigen::VectorXf::Ones(4));
    block.panoptic_ids = {10, 20, 30};
  }
  BlockIndices archived;
  for (int i = 0; i < 200; i += 5) {
    archived.emplace_back(i, i % 7, 0);
  }
  auto copied = getActiveMesh(layer, archived);
  PgmoMeshLayerInterface reference(*copied);
  PgmoMeshLayerInterface view(layer, archived);
  ASSERT_EQ(reference.blockIndices(), view.blockIndices());
  EXPECT_EQ(reference.hasSemantics(), view.hasSemantics());
  EXPECT_EQ(reference.hasSemanticFeatures(), view.hasSemanticFeatures());
  EXPECT_EQ(reference.hasPanopticIDs(), view.hasPanopticIDs());
  for (const auto& index : reference.blockIndices()) {
    reference.markBlockActive(index);
    view.markBlockActive(index);
    ASSERT_EQ(reference.activeBlockSize(), view.activeBlockSize());
    for (size_t i = 0; i < view.activeBlockSize(); ++i) {
      const auto a = reference.getActiveVertex(i);
      const auto b = view.getActiveVertex(i);
      EXPECT_EQ(a.x, b.x);
      EXPECT_EQ(a.y, b.y);
      EXPECT_EQ(a.z, b.z);
      EXPECT_EQ(a.rgba, b.rgba);
      EXPECT_EQ(reference.getActiveSemantics(i), view.getActiveSemantics(i));
      EXPECT_EQ(reference.getActivePanopticID(i), view.getActivePanopticID(i));
      EXPECT_TRUE(reference.getActiveSemanticFeatures(i)->isApprox(*view.getActiveSemanticFeatures(i)));
    }
  }
  kimera_pgmo::DeltaCompression a(0.01), b(0.01);
  kimera_pgmo::HashedIndexMapping map_a, map_b;
  auto delta_a = a.update(reference, 1, &map_a);
  auto delta_b = b.update(view, 1, &map_b, true);
  EXPECT_EQ(map_a, map_b);
  EXPECT_EQ(delta_a->vertex_updates->size(), delta_b->vertex_updates->size());
  EXPECT_EQ(delta_a->face_updates.size(), delta_b->face_updates.size());
}

TEST(PgmoMeshLayerInterface, EmptySelectionHasNoAttributes) {
  MeshLayer layer(1.0f);
  auto& block = layer.allocateBlock(BlockIndex(0, 0, 0));
  block.updated = true;
  PgmoMeshLayerInterface view(layer, {BlockIndex(0, 0, 0)});
  EXPECT_TRUE(view.blockIndices().empty());
  EXPECT_FALSE(view.hasSemantics());
  EXPECT_FALSE(view.hasSemanticFeatures());
  EXPECT_FALSE(view.hasPanopticIDs());
}
}  // namespace hydra
