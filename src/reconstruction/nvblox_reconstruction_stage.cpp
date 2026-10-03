// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include "hydra/reconstruction/nvblox_reconstruction_stage.h"
#include "hydra/input/input_data.h"
#include "hydra/input/camera.h"
#include "hydra/reconstruction/volumetric_map.h"
#include <nvblox/mapper/mapper.h>
#include <nvblox/sensors/camera.h>
#include "hydra/reconstruction/semantic_integrator.h"
#include "hydra/places/gvd_integrator.h"
#include "hydra/places/compression_graph_extractor.h"
#include "hydra/frontend/gvd_place_extractor.h"
#include "hydra/backend/update_places_functor.h"
#include "hydra/reconstruction/reconstruction_output.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hydra {
struct NvbloxReconstructionStage::Impl {
  explicit Impl(float voxel_size)
      : stream(std::make_shared<nvblox::CudaStreamOwning>()),
        mapper(voxel_size, nvblox::BlockMemoryPoolParams(), nvblox::ProjectiveLayerType::kTsdf, stream),
        depth(nvblox::MemoryType::kDevice), color(nvblox::MemoryType::kDevice), color_mask(nvblox::MemoryType::kDevice) {}
  std::shared_ptr<nvblox::CudaStream> stream;
  nvblox::Mapper mapper;
  TsdfGeometryMirror geometry_mirror;
  TsdfGeometryMirror validation_mirror;
  std::unique_ptr<VolumetricMap> validation_map;
  places::GvdLayer::Ptr validation_gvd;
  std::shared_ptr<places::CompressionGraphExtractor> validation_extractor;
  std::unique_ptr<places::GvdIntegrator> validation_integrator;
  std::unique_ptr<GvdPlaceExtractor> validation_frontend;
  std::unique_ptr<DynamicSceneGraph> validation_graph;
  std::shared_ptr<SemanticIntegrator> semantic_policy;
  nvblox::DepthImage depth;
  nvblox::ColorImage color;
  nvblox::MonoImage color_mask;
  std::vector<uint8_t> mask_staging;
  std::vector<nvblox::Color> color_staging;
  std::vector<float> staging;
};
NvbloxReconstructionStage::NvbloxReconstructionStage(float voxel_size,
    std::shared_ptr<SemanticIntegrator> semantic_policy, float integration_range_m) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0) throw std::invalid_argument("Invalid nvblox voxel size");
  impl_ = std::make_unique<Impl>(voxel_size);
  impl_->semantic_policy = std::move(semantic_policy);
  // Internal GPU reconstruction range; leave Hydra's CPU sensor range unchanged.
  if (!std::isfinite(integration_range_m) || integration_range_m <= 0.0f)
    throw std::invalid_argument("Invalid GPU integration range");
  impl_->mapper.tsdf_integrator().max_integration_distance_m(integration_range_m);
  impl_->mapper.color_integrator().max_integration_distance_m(integration_range_m);
  // Masks/depth can change while the camera remains stationary.
  impl_->mapper.tsdf_integrator().view_calculator().cache_last_viewpoint(false);
}
NvbloxReconstructionStage::~NvbloxReconstructionStage() = default;
void NvbloxReconstructionStage::clearOutsideRadius(const Eigen::Vector3f& center, float radius) {
  if (!center.allFinite() || !std::isfinite(radius) || radius <= 0.0f) {
    throw std::invalid_argument("Local nvblox clearing requires a finite center and positive radius");
  }
  impl_->mapper.clearOutsideRadius(center, radius);
  impl_->stream->synchronize();
}
void NvbloxReconstructionStage::integrate(const InputData& input) {
  const auto* camera = dynamic_cast<const Camera*>(&input.getSensor());
  if (!camera) throw std::invalid_argument("nvblox stage requires a pinhole camera");
  const auto& c = camera->getConfig();
  const auto& depth = input.depth_image;
  if (depth.type() != CV_32FC1 || depth.rows != c.height || depth.cols != c.width)
    throw std::invalid_argument("nvblox stage requires metric float depth matching calibration");
  if (!(c.fx > 0 && c.fy > 0) || !input.getSensorPose().matrix().allFinite())
    throw std::invalid_argument("Invalid nvblox camera calibration or pose");
  const bool filter = impl_->semantic_policy && !input.label_image.empty();
  if (filter && (input.label_image.type() != CV_32SC1 || input.label_image.size() != depth.size()))
    throw std::invalid_argument("Semantic labels must be aligned int32 images");
  impl_->staging.resize(depth.total());
  // Copy row-wise: OpenCV observations can be non-contiguous. Invalid depth is zero.
  for (int y = 0; y < depth.rows; ++y) {
    const auto* row = depth.ptr<float>(y);
    for (int x = 0; x < depth.cols; ++x) {
      const float d = row[x];
      impl_->staging[y * depth.cols + x] = std::isfinite(d) && d > 0 &&
          (!filter || impl_->semantic_policy->canIntegrate(input.label_image.at<int>(y,x))) ? d : 0;
    }
  }
  impl_->depth.copyFrom(depth.rows, depth.cols, impl_->staging.data());
  const nvblox::Camera intrinsics(c.fx, c.fy, c.cx, c.cy, c.width, c.height);
  const nvblox::Transform pose(input.getSensorPose().cast<float>().matrix());
  impl_->mapper.integrateDepth(impl_->depth, pose, intrinsics);
  // Include completed GPU work in the comparison timer, not only kernel submission.
  impl_->stream->synchronize();
}
void NvbloxReconstructionStage::integrateColor(const InputData& input) {
  if (input.color_image.empty()) return;
  const auto* camera = dynamic_cast<const Camera*>(&input.getSensor());
  if (!camera) throw std::invalid_argument("nvblox color requires a pinhole camera");
  const auto& c = camera->getConfig();
  const auto& rgb = input.color_image;
  // Hydra InputData stores RGB, not OpenCV's usual BGR order.
  if (rgb.type() != CV_8UC3 || rgb.rows != c.height || rgb.cols != c.width)
    throw std::invalid_argument("nvblox color requires aligned RGB8 matching calibration");
  const bool filter = impl_->semantic_policy && !input.label_image.empty();
  if (filter && (input.label_image.type() != CV_32SC1 || input.label_image.size() != rgb.size()))
    throw std::invalid_argument("Color semantic labels must be aligned int32 images");
  impl_->mask_staging.assign(rgb.total(), 1);
  if (filter) {
    for (int y=0;y<rgb.rows;++y) for(int x=0;x<rgb.cols;++x)
      impl_->mask_staging[y*rgb.cols+x] = impl_->semantic_policy->canIntegrate(input.label_image.at<int>(y,x)) ? 1 : 0;
  }
  impl_->color_staging.resize(rgb.total());
  for (int y = 0; y < rgb.rows; ++y) {
    const auto* row = rgb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < rgb.cols; ++x)
      impl_->color_staging[y * rgb.cols + x] = nvblox::Color(row[x][0], row[x][1], row[x][2]);
  }
  impl_->color.copyFrom(rgb.rows, rgb.cols, impl_->color_staging.data());
  const nvblox::Camera intrinsics(c.fx, c.fy, c.cx, c.cy, c.width, c.height);
  const nvblox::Transform pose(input.getSensorPose().cast<float>().matrix());
  impl_->color_mask.copyFrom(rgb.rows, rgb.cols, impl_->mask_staging.data());
  const nvblox::MaskedColorImageConstView masked(
      impl_->color, nvblox::MonoImageConstView(impl_->color_mask));
  impl_->mapper.integrateColor(masked, pose, intrinsics);
  impl_->stream->synchronize();
}
void NvbloxReconstructionStage::updateMesh() {
  impl_->mapper.updateColorMesh();
  impl_->stream->synchronize();
}
std::size_t NvbloxReconstructionStage::exportTsdfGeometry(VolumetricMap& destination, bool include_color) const {
  const auto& source = impl_->mapper.tsdf_layer();
  if (std::abs(destination.config.voxel_size - source.voxel_size()) > 1e-7f)
    throw std::invalid_argument("TSDF export requires matching voxel sizes");
  if (destination.hasSemantics() || destination.getTrackingLayer())
    throw std::invalid_argument("Geometry-only export cannot populate semantic/tracking maps");
  const int side = destination.config.voxels_per_side;
  if (side <= 0) throw std::invalid_argument("Invalid destination block size");
  std::size_t copied = 0;
  // Full export for conversion validation. Incremental block tracking is pending.
  for (const auto& index : source.getAllBlockIndices()) {
    nvblox::VoxelBlock<nvblox::TsdfVoxel> host;
    const auto block = source.getBlockAtIndex(index);
    const auto error = cudaMemcpy(&host, block.get(), sizeof(host), cudaMemcpyDeviceToHost);
    if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
    nvblox::VoxelBlock<nvblox::ColorVoxel> colors;
    bool has_colors = false;
    if (include_color) {
      const auto color_block = impl_->mapper.color_layer().getBlockAtIndex(index);
      if (color_block) {
        const auto status = cudaMemcpy(&colors, color_block.get(), sizeof(colors), cudaMemcpyDeviceToHost);
        if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
        has_colors = true;
      }
    }
    for (int x = 0; x < 8; ++x) for (int y = 0; y < 8; ++y) for (int z = 0; z < 8; ++z) {
      const auto& voxel = host.voxels[x][y][z];
      if (!(voxel.weight > 0) || !std::isfinite(voxel.weight) || !std::isfinite(voxel.distance)) continue;
      const Eigen::Vector3i global = index * 8 + Eigen::Vector3i(x,y,z);
      Eigen::Vector3i target, local;
      for (int axis = 0; axis < 3; ++axis) {
        // C++ integer division truncates toward zero; block coordinates require floor.
        target[axis] = global[axis] / side;
        if (global[axis] % side < 0) --target[axis];
        local[axis] = global[axis] - target[axis] * side;
      }
      destination.allocateBlock(target);
      auto out = destination.getBlock(target).tsdf;
      auto& result = out->getVoxel(local);
      result.distance = voxel.distance;
      result.weight = voxel.weight;
      if (include_color) {
        // Unobserved RGB is unknown, not black. Confidence comes from the
        // independent nvblox color layer, never from the TSDF weight.
        result.color = Color(128, 128, 128);
        if (has_colors) {
          const auto& rgb = colors.voxels[x][y][z];
          if (std::isfinite(rgb.weight) && rgb.weight > 0.0f)
            result.color = Color(rgb.color.r(), rgb.color.g(), rgb.color.b());
        }
      }
      out->setUpdated();
      ++copied;
    }
  }
  return copied;
}
TsdfGeometryDelta NvbloxReconstructionStage::exportTsdfDelta(VolumetricMap& destination) {
  VolumetricMap snapshot(destination.config, false);
  exportTsdfGeometry(snapshot);
  return impl_->geometry_mirror.update(snapshot, destination);
}
std::size_t NvbloxReconstructionStage::validateGvd(
    uint64_t timestamp_ns, const VolumetricMap::Config& config,
    const Eigen::Vector3f& center, float active_radius) {
  if (!center.allFinite() || !std::isfinite(active_radius))
    throw std::invalid_argument("Invalid validation active region");
  if (!impl_->validation_map) {
    impl_->validation_map = std::make_unique<VolumetricMap>(config, false);
    GvdPlaceExtractor::Config frontend_config;
    frontend_config.graph = places::CompressionExtractorConfig();
    impl_->validation_frontend = std::make_unique<GvdPlaceExtractor>(frontend_config);
    impl_->validation_graph = std::make_unique<DynamicSceneGraph>();
    impl_->validation_gvd = std::make_shared<places::GvdLayer>(config.voxel_size, config.voxels_per_side);
    impl_->validation_extractor = std::make_shared<places::CompressionGraphExtractor>(places::CompressionExtractorConfig());
    impl_->validation_integrator = std::make_unique<places::GvdIntegrator>(
        places::GvdIntegratorConfig(), impl_->validation_gvd, impl_->validation_extractor);
  }
  VolumetricMap snapshot(config, false);
  exportTsdfGeometry(snapshot);
  // Preserve GPU voxel evidence; archive only the CPU validation window.
  // Block-center selection is approximate at the boundary.
  if (active_radius > 0) {
    for (const auto& index : snapshot.getTsdfLayer().allocatedBlockIndices()) {
      const Eigen::Vector3f block_center =
          (index.cast<float>() + Eigen::Vector3f::Constant(0.5f)) * snapshot.blockSize();
      if ((block_center - center).norm() > active_radius) snapshot.removeBlock(index);
    }
  }
  const auto delta = impl_->validation_mirror.update(snapshot, *impl_->validation_map);
  ReconstructionOutput output;
  output.timestamp_ns = timestamp_ns;
  output.world_t_body = center.cast<double>();
  output.world_R_body = Eigen::Quaterniond::Identity();
  output.archived_blocks = delta.removed;
  output.setMap(*impl_->validation_map);
  impl_->validation_frontend->detect(output);
  impl_->validation_frontend->updateGraph(timestamp_ns, *impl_->validation_graph);
  if (!delta.removed.empty()) {
    impl_->validation_integrator->archiveBlocks(delta.removed);
    impl_->validation_map->removeBlocks(delta.removed);
  }
  impl_->validation_integrator->updateFromTsdf(timestamp_ns, impl_->validation_map->getTsdfLayer(), true);
  impl_->validation_integrator->updateGvd(timestamp_ns);
  return impl_->validation_gvd->numBlocks();
}
std::size_t NvbloxReconstructionStage::numValidationPlaces() const {
  return impl_->validation_extractor ? impl_->validation_extractor->getGraph().numNodes() : 0;
}
std::size_t NvbloxReconstructionStage::numActiveValidationPlaces() const {
  return impl_->validation_extractor ? impl_->validation_extractor->getActiveNodes().size() : 0;
}
std::size_t NvbloxReconstructionStage::numFrontendValidationPlaces() const {
  if (!impl_->validation_graph || !impl_->validation_graph->hasLayer(DsgLayers::PLACES)) return 0;
  return impl_->validation_graph->getLayer(DsgLayers::PLACES).numNodes();
}
std::size_t NvbloxReconstructionStage::countValidationMergeProposals(
    double position_tolerance, double distance_tolerance) const {
  if (!std::isfinite(position_tolerance) || position_tolerance < 0 ||
      !std::isfinite(distance_tolerance) || distance_tolerance < 0)
    throw std::invalid_argument("Invalid place merge tolerance");
  if (!impl_->validation_graph || !impl_->validation_graph->hasLayer(DsgLayers::PLACES)) return 0;
  auto graph = impl_->validation_graph->clone();
  const auto active = impl_->validation_frontend->getActiveNodes();
  const auto& places = graph->getLayer(DsgLayers::PLACES);
  for (const auto& entry : places.nodes())
    entry.second->attributes().is_active = active.count(entry.first) != 0;
  UpdatePlacesFunctor functor(position_tolerance, distance_tolerance);
  functor.node_finder = NearestNodeFinder::fromLayer(places, [](const SceneGraphNode& node) {
    return !node.attributes().is_active && node.attributes<PlaceNodeAttributes>().real_place;
  });
  if (!functor.node_finder) return 0;
  std::size_t proposals = 0;
  for (const auto id : active) {
    if (!places.hasNode(id)) continue;
    if (functor.proposeMerge(places, places.getNode(id))) ++proposals;
  }
  return proposals;
}
// Rebuild active Hydra mesh blocks from the GPU mesh. Group by source block
// origin (not triangle centroid) so seam triangles keep deterministic ownership.
// Distant CPU blocks remain intact until Hydra emits its normal archival events.
void NvbloxReconstructionStage::exportMesh(VolumetricMap& destination,
    const Eigen::Vector3f& center, float active_radius) const {
  if (destination.config.voxels_per_side < 8 || destination.config.voxels_per_side % 8)
    throw std::invalid_argument("GPU mesh export requires Hydra block side divisible by 8");
  auto& output = destination.getMeshLayer();
  const auto active = [&](const BlockIndex& index) {
    const Eigen::Vector3f block_center = (index.cast<float>() + Eigen::Vector3f::Constant(0.5f)) * destination.blockSize();
    return active_radius <= 0 || (block_center - center).norm() <= active_radius;
  };
  for (const auto& index : destination.getTsdfLayer().allocatedBlockIndices()) {
    if (!active(index)) continue;
    auto& mesh = output.allocateBlock(index, destination.hasSemantics(), destination.hasSemantics(), destination.hasSemantics());
    mesh.clear();
    mesh.updated = true;
    destination.getTsdfLayer().getBlock(index).mesh_updated = false;
  }
  const auto& source = impl_->mapper.color_mesh_layer();
  auto indices = source.getAllBlockIndices();
  std::sort(indices.begin(), indices.end(), [](const auto& a, const auto& b) {
    for (int i=0;i<3;++i) { if(a[i]!=b[i]) return a[i]<b[i]; }
    return false;
  });
  for (const auto& index : indices) {
    BlockIndex target;
    for (int axis=0;axis<3;++axis) {
      const int global = index[axis]*8;
      target[axis] = global / destination.config.voxels_per_side;
      if (global % destination.config.voxels_per_side < 0) --target[axis];
    }
    if (!active(target) || !destination.getTsdfLayer().hasBlock(target)) continue;
    const auto block = source.getBlockAtIndex(index);
    auto vertices = block->vertices.toVectorAsync(*impl_->stream);
    auto colors = block->vertex_appearances.toVectorAsync(*impl_->stream);
    auto triangles = block->triangles.toVectorAsync(*impl_->stream);
    impl_->stream->synchronize();
    if (colors.size()!=vertices.size() || triangles.size()%3)
      throw std::runtime_error("Invalid nvblox mesh arrays");
    auto& mesh = output.getBlock(target);
    const size_t offset = mesh.points.size();
    // PGMO interprets consecutive triples as faces; export triangle soup.
    mesh.resizeVertices(offset+triangles.size());
    for (size_t i=0;i<triangles.size();++i) {
      if(triangles[i]<0 || static_cast<size_t>(triangles[i])>=vertices.size())
        throw std::runtime_error("Invalid nvblox triangle index");
      const size_t source_index = triangles[i];
      mesh.points[offset+i] = vertices[source_index];
      mesh.colors[offset+i] = Color(colors[source_index].r(), colors[source_index].g(), colors[source_index].b());
      if (destination.hasSemantics()) {
        // Choose the nearest observed semantic voxel among the eight corners
        // surrounding this surface vertex. CPU semantics are kept independent.
        const Eigen::Vector3f grid = vertices[source_index]/destination.config.voxel_size - Eigen::Vector3f::Constant(0.5f);
        const GlobalIndex base = grid.array().floor().cast<int>();
        const SemanticVoxel* best = nullptr;
        float best_distance = std::numeric_limits<float>::infinity();
        for(int x=0;x<2;++x) for(int y=0;y<2;++y) for(int z=0;z<2;++z) {
          const GlobalIndex key = base+GlobalIndex(x,y,z);
          const auto* voxel = destination.getSemanticLayer()->getVoxelPtr(key);
          if (!voxel || voxel->empty) continue;
          const float distance = (grid-key.cast<float>()).squaredNorm();
          if(distance<best_distance) { best=voxel;best_distance=distance; }
        }
        if(best) {
          mesh.labels[offset+i]=best->semantic_label;
          mesh.semantic_features[offset+i]=best->feature_vector;
          mesh.panoptic_ids[offset+i]=best->panoptic_id;
        }
      }
    }
    for(size_t i=0;i<triangles.size();i+=3)
      mesh.faces.push_back({offset+i,offset+i+1,offset+i+2});
  }
}
std::size_t NvbloxReconstructionStage::numTriangles() const {
  std::size_t count = 0;
  const auto& mesh = impl_->mapper.color_mesh_layer();
  for (const auto& index : mesh.getAllBlockIndices())
    count += mesh.getBlockAtIndex(index)->triangles.size() / 3;
  return count;
}
std::vector<std::array<uint8_t, 3>> NvbloxReconstructionStage::meshColors() const {
  std::vector<std::array<uint8_t, 3>> output;
  const auto& mesh = impl_->mapper.color_mesh_layer();
  auto indices = mesh.getAllBlockIndices();
  std::sort(indices.begin(), indices.end(), [](const auto& a, const auto& b) {
    for(int axis=0;axis<3;++axis) { if(a[axis]!=b[axis]) return a[axis]<b[axis]; }
    return false;
  });
  for(const auto& index : indices) {
    auto colors = mesh.getBlockAtIndex(index)->vertex_appearances.toVectorAsync(*impl_->stream);
    impl_->stream->synchronize();
    for(const auto& c : colors) output.push_back({c.r(),c.g(),c.b()});
  }
  return output;
}
std::size_t NvbloxReconstructionStage::numBlocks() const {
  return impl_->mapper.tsdf_layer().numAllocatedBlocks();
}
}
