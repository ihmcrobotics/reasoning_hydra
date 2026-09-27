/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <gtest/gtest.h>
#include <hydra/backend/update_rooms_buildings_functor.h>

#include "hydra_test/shared_dsg_fixture.h"
#include "hydra_test/config_guard.h"
#include <spark_dsg/serialization/graph_binary_serialization.h>

namespace hydra {

TEST(UpdateRoomsBuildingsFunctor, BuildingUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  graph.emplaceNode(DsgLayers::BUILDINGS,
                    "B0"_id,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));

  graph.emplaceNode(DsgLayers::ROOMS,
                    3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(DsgLayers::ROOMS,
                    4,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(DsgLayers::ROOMS,
                    5,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));

  graph.insertEdge("B0"_id, 3);
  graph.insertEdge("B0"_id, 4);
  graph.insertEdge("B0"_id, 5);

  UpdateInfo::ConstPtr info(new UpdateInfo{nullptr, nullptr, false, 0, false, {}});
  UpdateBuildingsFunctor functor(Color(), 0);
  const auto unmerged = dsg->graph->clone();
  functor.call(*unmerged, *dsg, info);

  Eigen::Vector3d first_expected(-1.0, 0.0, 1.0);
  Eigen::Vector3d first_result = graph.getPosition("B0"_id);
  EXPECT_NEAR(0.0, (first_expected - first_result).norm(), 1.0e-7);
}

}  // namespace hydra

TEST(BackendOptimization, DirectPlacesMatchesFilteredCopy) {
  hydra::test::ConfigGuard guard;
  auto direct = hydra::test::makeSharedDsg();
  for (size_t i = 0; i < 40; ++i) {
    auto attrs = std::make_unique<hydra::PlaceNodeAttributes>();
    attrs->distance = 1.0 + (i % 4) * 0.2;
    attrs->position = Eigen::Vector3d(i * 0.1, 0, 0);
    direct->graph->emplaceNode(hydra::DsgLayers::PLACES,
                               hydra::NodeSymbol('p', i), std::move(attrs));
    if (i) {
      auto edge = std::make_unique<hydra::EdgeAttributes>();
      edge->weight = 0.1 + (i % 10) * 0.1;
      direct->graph->insertEdge(hydra::NodeSymbol('p', i-1),
                                hydra::NodeSymbol('p', i), std::move(edge));
    }
  }
  auto filtered = hydra::test::makeSharedDsg();
  filtered->graph = direct->graph->clone();
  // Force the mixed-layer copy path without changing the selected p subgraph.
  filtered->graph->emplaceNode(hydra::DsgLayers::PLACES,
      hydra::NodeSymbol('x', 0), std::make_unique<hydra::PlaceNodeAttributes>());
  hydra::RoomsFunctorConfig config;
  config.room_finder_config.min_component_size = 2;
  config.room_finder_config.min_room_size = 2;
  hydra::UpdateRoomsFunctor a(config), b(config);
  hydra::UpdateInfo::ConstPtr info(new hydra::UpdateInfo{nullptr,nullptr,false,123,false,{}});
  const auto unmerged = direct->graph->clone();
  a.call(*unmerged, *direct, info);
  b.call(*unmerged, *filtered, info);
  ASSERT_GT(direct->graph->getLayer(hydra::DsgLayers::ROOMS).numNodes(), 0u);
  filtered->graph->removeNode(hydra::NodeSymbol('x', 0));
  std::vector<uint8_t> bytes_a, bytes_b;
  spark_dsg::io::binary::writeGraph(*direct->graph, bytes_a, true);
  spark_dsg::io::binary::writeGraph(*filtered->graph, bytes_b, true);
  EXPECT_EQ(direct->graph->numNodes(), filtered->graph->numNodes());
  EXPECT_EQ(direct->graph->numEdges(), filtered->graph->numEdges());
  for (const auto& layer : direct->graph->layers()) {
    for (const auto& entry : layer.second->nodes()) {
      ASSERT_TRUE(filtered->graph->hasNode(entry.first));
      const auto& other = filtered->graph->getNode(entry.first);
      EXPECT_TRUE(entry.second->attributes() == other.attributes()) << entry.first;
      EXPECT_EQ(entry.second->getParent(), other.getParent()) << entry.first;
      EXPECT_EQ(entry.second->siblings(), other.siblings()) << entry.first;
    }
  }
  std::vector<uint8_t> reserved;
  reserved.reserve(bytes_a.size());
  spark_dsg::io::binary::writeGraph(*direct->graph, reserved, true);
  EXPECT_EQ(bytes_a, reserved);
}
