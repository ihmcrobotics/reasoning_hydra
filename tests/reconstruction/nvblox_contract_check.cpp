// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#include "hydra/utils/pgmo_mesh_interface.h"
#include <kimera_pgmo/compression/delta_compression.h>
#include "hydra/reconstruction/nvblox_reconstruction_stage.h"
#include "hydra/input/camera.h"
#include "hydra/input/input_data.h"
#include <iostream>
#include "hydra/reconstruction/nvblox_reconstruction_adapter.h"
#include "hydra/reconstruction/reconstruction_output.h"
#include "hydra/places/gvd_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/reconstruction/volumetric_map.h"
#include <stdexcept>
int main() {
 hydra::PipelineConfig pipeline;pipeline.label_space.total_labels=4;pipeline.label_space.dynamic_labels={3};pipeline.label_space.invalid_labels={0};
 hydra::GlobalInfo::init(pipeline);
 hydra::Camera::Config c; c.width=64;c.height=48;c.fx=60;c.fy=60;c.cx=32;c.cy=24;c.min_range=.1;c.max_range=5;
 c.extrinsics=hydra::IdentitySensorExtrinsics::Config();
 auto camera=std::make_shared<hydra::Camera>(c);
 hydra::InputData input(camera);input.timestamp_ns=1;input.world_T_body=Eigen::Isometry3d::Identity();
 cv::Mat storage(48,66,CV_32FC1,cv::Scalar(2.0));input.depth_image=storage(cv::Rect(1,0,64,48));
 input.color_image=cv::Mat(48,64,CV_8UC3,cv::Scalar(200,30,10));
 hydra::NvbloxReconstructionStage stage(.1f);
 for(int i=0;i<3;++i){stage.integrate(input);stage.integrateColor(input);}
 stage.updateMesh();
 if(stage.numTriangles()==0)throw std::runtime_error("No mesh triangles");
 std::cout << "PASS RGB-D GPU meshing; triangles=" << stage.numTriangles() << std::endl;
 if(stage.numBlocks()==0)throw std::runtime_error("No GPU TSDF blocks");
 input.world_T_body.translation().x()=1;input.timestamp_ns=2;stage.integrate(input);
 std::cout << "PASS metric depth with strided input and changed pose; blocks=" << stage.numBlocks() << std::endl;
 hydra::VolumetricMap::Config mc;mc.voxel_size=.1f;mc.voxels_per_side=16;
 hydra::VolumetricMap converted(mc,false);
 auto copied=stage.exportTsdfGeometry(converted);
 if(!copied || converted.getTsdfLayer().numBlocks()==0) throw std::runtime_error("Empty TSDF conversion");
 bool negative=false,positive=false;std::size_t observed=0;
 for(const auto& idx:converted.getTsdfLayer().allocatedBlockIndices()) {
  const auto& block=converted.getTsdfLayer().getBlock(idx);
  if(!block.esdf_updated || !block.mesh_updated)throw std::runtime_error("Missing update flags");
  for(size_t i=0;i<block.numVoxels();++i) {
   const auto& v=block.getVoxel(i);if(v.weight<=0)continue;++observed;
   const auto pos=block.getVoxelPosition(i);
   negative |= pos.x()<0;positive |= pos.x()>0;
   if(std::abs(pos.z()-2.f)<.2f && std::abs(v.distance-(2.f-pos.z()))>.06f)
    throw std::runtime_error("Plane signed distance mismatch");
  }
 }
 if(!negative || !positive || observed!=copied)throw std::runtime_error("Coordinate/count mismatch");
 auto again=stage.exportTsdfGeometry(converted);if(again!=copied)throw std::runtime_error("Repeat export mismatch");
 hydra::VolumetricMap semantic(mc,true);bool refused=false;
 try{stage.exportTsdfGeometry(semantic);}catch(const std::invalid_argument&){refused=true;}
 if(!refused)throw std::runtime_error("Silently exported incomplete semantics");
 std::cout << "PASS GPU-to-Hydra export: distances, negative coordinates, flags, repeat export; voxels=" << copied << std::endl;
 hydra::VolumetricMap mirrorMap(mc,false);
 auto first=stage.exportTsdfDelta(mirrorMap);
 if(first.changed.empty())throw std::runtime_error("Initial delta empty");
 auto repeat=stage.exportTsdfDelta(mirrorMap);
 if(!repeat.changed.empty() || !repeat.removed.empty())throw std::runtime_error("Unchanged export produced deltas");
 hydra::VolumetricMap synthetic(mc,false), retained(mc,false);
 hydra::TsdfGeometryMirror mirror;
 hydra::BlockIndex key(-1,0,0);synthetic.allocateBlock(key);
 auto v=synthetic.getBlock(key).tsdf;v->getVoxel(0).weight=1;v->getVoxel(0).distance=.2;
 mirror.update(synthetic,retained);
 v->getVoxel(1).weight=1;v->getVoxel(1).distance=-.1;
 if(mirror.update(synthetic,retained).changed.size()!=1)throw std::runtime_error("Missing changed block");
 v->getVoxel(0).weight=0;
 mirror.update(synthetic,retained);
 if(retained.getBlock(key).tsdf->getVoxel(0).weight!=1)throw std::runtime_error("Partial update erased evidence");
 synthetic.removeBlock(key);
 auto removed=mirror.update(synthetic,retained);
 if(removed.removed.size()!=1 || retained.getTsdfLayer().numBlocks()!=1)throw std::runtime_error("Archive event or retention incorrect");
 if(!mirror.update(synthetic,retained).removed.empty())throw std::runtime_error("Repeated removal event");
 std::cout << "PASS incremental export: unchanged suppression, partial-block retention, explicit removal event" << std::endl;
 hydra::ProjectiveIntegratorConfig config;config.num_threads=1;config.interp_method="nearest";
 hydra::ProjectiveIntegrator appearance(config);
 input.range_image=cv::Mat(48,64,CV_32FC1);
 for(int y=0;y<48;++y)for(int x=0;x<64;++x)input.range_image.at<float>(y,x)=2.f*std::sqrt(1.f+std::pow((x-32)/60.f,2)+std::pow((y-24)/60.f,2));
 std::vector<std::pair<float,float>> before;
 auto indices=converted.getTsdfLayer().allocatedBlockIndices();
 for(const auto& idx:indices){const auto& block=converted.getTsdfLayer().getBlock(idx);for(size_t i=0;i<block.numVoxels();++i)before.emplace_back(block.getVoxel(i).distance,block.getVoxel(i).weight);}
 auto changedAppearance=appearance.updateAppearanceBlocks(indices,input,converted);
 if(changedAppearance.empty())throw std::runtime_error("No appearance association");
 size_t j=0;
 for(const auto& idx:indices){const auto& block=converted.getTsdfLayer().getBlock(idx);for(size_t i=0;i<block.numVoxels();++i){const auto& v=block.getVoxel(i);if(before[j++]!=std::make_pair(v.distance,v.weight))throw std::runtime_error("Appearance changed TSDF");}}
 std::cout << "PASS appearance association preserves all TSDF distances and weights" << std::endl;
 hydra::VolumetricMap semanticMap(mc,true);
 for(const auto& idx:indices) {
  semanticMap.allocateBlock(idx);
  const auto& source=converted.getTsdfLayer().getBlock(idx);
  auto target=semanticMap.getBlock(idx).tsdf;
  for(size_t i=0;i<source.numVoxels();++i)target->getVoxel(i)=source.getVoxel(i);
 }
 config.semantic_integrator=hydra::MLESemanticIntegrator::Config();
 hydra::ProjectiveIntegrator semantics(config);
 input.label_image=cv::Mat(48,64,CV_32SC1,cv::Scalar(2));
 input.features_mask=cv::Mat(48,64,CV_16UC1,cv::Scalar(7));
 Eigen::VectorXf feature(3);feature<<.2f,.4f,.6f;
 input.semantic_features=std::unordered_map<uint16_t,Eigen::VectorXf>{{7,feature}};
 semantics.updateAppearanceBlocks(indices,input,semanticMap);
 size_t labeled=0;
 for(const auto& idx:indices) {
  auto block=semanticMap.getBlock(idx);
  for(size_t i=0;i<block.tsdf->numVoxels();++i) {
   const auto& v=block.semantic->getVoxel(i);if(v.empty)continue;++labeled;
   if(v.semantic_label!=2 || v.panoptic_id!=7 || !v.feature_vector || !v.feature_vector->isApprox(feature))throw std::runtime_error("Semantic transfer mismatch");
   const auto& original=converted.getTsdfLayer().getBlock(idx).getVoxel(i);
   if(original.distance!=block.tsdf->getVoxel(i).distance || original.weight!=block.tsdf->getVoxel(i).weight)throw std::runtime_error("Semantic fusion changed geometry");
  }
 }
 if(!labeled)throw std::runtime_error("No semantic voxels");
 input.label_image.setTo(cv::Scalar(3));
 semantics.updateAppearanceBlocks(indices,input,semanticMap);
 for(const auto& idx:indices) {
  auto block=semanticMap.getBlock(idx);
  for(size_t i=0;i<block.tsdf->numVoxels();++i) {
   const auto& v=block.semantic->getVoxel(i);if(!v.empty && (v.semantic_label!=2 || v.num_observations!=1))throw std::runtime_error("Dynamic observation integrated");
  }
 }
 std::cout << "PASS semantic labels, features, panoptic IDs, dynamic rejection; labeled=" << labeled << std::endl;
 auto policy=std::make_shared<hydra::MLESemanticIntegrator>(hydra::MLESemanticIntegrator::Config());
 hydra::NvbloxReconstructionStage filtered(.1f,policy);
 for(int label:{3,0}) {
  input.label_image.setTo(cv::Scalar(label));filtered.integrate(input);
  hydra::VolumetricMap empty(mc,false);
  if(filtered.exportTsdfGeometry(empty)!=0)throw std::runtime_error("Excluded depth entered TSDF");
 }
 input.label_image.setTo(cv::Scalar(2));filtered.integrate(input);
 hydra::VolumetricMap accepted(mc,false);
 if(!filtered.exportTsdfGeometry(accepted))throw std::runtime_error("Static depth rejected");
 std::cout << "PASS dynamic/invalid depth masking and static observation acceptance" << std::endl;
 filtered.integrateColor(input);filtered.updateMesh();
 const auto originalColors=filtered.meshColors();
 if(originalColors.empty())throw std::runtime_error("No mesh colors to validate");
 input.label_image.setTo(cv::Scalar(3));input.color_image.setTo(cv::Scalar(0,255,0));
 filtered.integrateColor(input);filtered.updateMesh();
 if(filtered.meshColors()!=originalColors)throw std::runtime_error("Excluded frame repainted mesh");
 input.label_image.colRange(0,32).setTo(cv::Scalar(2));
 filtered.integrateColor(input);filtered.updateMesh();
 const auto mixedColors=filtered.meshColors();size_t changed=0,unchanged=0;
 if(mixedColors.size()!=originalColors.size())throw std::runtime_error("Color update changed geometry");
 for(size_t i=0;i<mixedColors.size();++i){if(mixedColors[i]!=originalColors[i])++changed;else ++unchanged;}
 if(!changed || !unchanged)throw std::runtime_error("Mixed-label color mask failed");
 std::cout << "PASS color mask: excluded repaint blocked; mixed frame changed=" << changed << " preserved=" << unchanged << std::endl;
 auto gvd=std::make_shared<hydra::places::GvdLayer>(mc.voxel_size,mc.voxels_per_side);
 hydra::places::GvdIntegratorConfig gvdConfig;
 hydra::places::GvdIntegrator integrator(gvdConfig,gvd,nullptr);
 integrator.updateFromTsdf(input.timestamp_ns,semanticMap.getTsdfLayer(),true);
 integrator.updateGvd(input.timestamp_ns);
 size_t observedGvd=0;
 for(const auto& idx:gvd->allocatedBlockIndices()) {
  const auto& block=gvd->getBlock(idx);
  for(size_t i=0;i<block.numVoxels();++i)if(block.getVoxel(i).observed){
   ++observedGvd;if(!std::isfinite(block.getVoxel(i).distance))throw std::runtime_error("Invalid GVD distance");
  }
 }
 if(!observedGvd)throw std::runtime_error("No GVD observations from GPU map");
 std::cout << "PASS Hydra GVD update from converted GPU TSDF; observed=" << observedGvd << std::endl;
 if(!stage.validateGvd(input.timestamp_ns,mc) || !stage.validateGvd(input.timestamp_ns+1,mc))throw std::runtime_error("Shadow GVD validation failed");
 hydra::NvbloxReconstructionStage room(.1f);
 auto roomConfig=c;roomConfig.fx=24;roomConfig.fy=24;
 hydra::InputData roomInput(std::make_shared<hydra::Camera>(roomConfig));roomInput.timestamp_ns=10;
 input.label_image.setTo(cv::Scalar(2));
 for(int view=0;view<6;++view) {
  roomInput.world_T_body=Eigen::Isometry3d::Identity();
  if(view<4)roomInput.world_T_body.linear()=Eigen::AngleAxisd(view*1.5707963267948966,Eigen::Vector3d::UnitY()).toRotationMatrix();
  else roomInput.world_T_body.linear()=Eigen::AngleAxisd((view==4?1:-1)*1.5707963267948966,Eigen::Vector3d::UnitX()).toRotationMatrix();
  roomInput.depth_image=cv::Mat(48,64,CV_32FC1,cv::Scalar(2.f));
  for(int y=0;y<48;++y)for(int x=0;x<64;++x) {
   const float rx=(x-32)/24.f,ry=(y-24)/24.f;
   roomInput.depth_image.at<float>(y,x)=2.f/std::max(1.f,std::max(std::abs(rx),std::abs(ry)));
  }
  room.integrate(roomInput);
 }
 room.validateGvd(10,mc);
 std::cout << "GPU room validation places=" << room.numValidationPlaces() << std::endl;
 if(!room.numValidationPlaces())throw std::runtime_error("No GPU-derived room places");
 std::cout << "PASS GPU TSDF to Hydra GVD to place graph" << std::endl;
 const auto gpuBlocksBefore=room.numBlocks();
 const auto placesBefore=room.numValidationPlaces();
 if(room.validateGvd(11,mc,Eigen::Vector3f(100,100,100),8.f)!=0)throw std::runtime_error("Far window failed to archive GVD");
 if(room.numBlocks()!=gpuBlocksBefore)throw std::runtime_error("Archival discarded GPU evidence");
 if(!room.validateGvd(12,mc,Eigen::Vector3f::Zero(),8.f))throw std::runtime_error("Revisit did not restore GVD");
 if(!room.numValidationPlaces())throw std::runtime_error("Revisit did not restore places");
 std::cout << "PASS voxel archival/revisit only (graph reconciliation pending): places before=" << placesBefore << " after=" << room.numValidationPlaces() << std::endl;
 for(int cycle=0;cycle<3;++cycle) {
  room.validateGvd(20+cycle*2,mc,Eigen::Vector3f(100,100,100),8.f);
  if(room.numActiveValidationPlaces()!=0)throw std::runtime_error("Archived places still active");
  room.validateGvd(21+cycle*2,mc,Eigen::Vector3f::Zero(),8.f);
  std::cout << "REVISIT cycle=" << cycle << " total=" << room.numValidationPlaces() << " active=" << room.numActiveValidationPlaces() << std::endl;
  if(!room.numActiveValidationPlaces())throw std::runtime_error("No active places on revisit");
 }
 if(!room.numFrontendValidationPlaces())throw std::runtime_error("No frontend graph places");
 std::cout << "PASS GPU map through Hydra place frontend; graph nodes=" << room.numFrontendValidationPlaces() << std::endl;
 const auto graphCount=room.numFrontendValidationPlaces();
 const auto proposals=room.countValidationMergeProposals(.4,.3);
 if(!proposals)throw std::runtime_error("No backend merge proposals for revisited room");
 if(room.numFrontendValidationPlaces()!=graphCount)throw std::runtime_error("Proposal diagnostic mutated graph");
 std::cout << "PASS Hydra backend merge proposal diagnostic; candidates=" << proposals << std::endl;
 input.label_image.setTo(cv::Scalar(2));
 hydra::MeshIntegratorConfig meshConfig;meshConfig.integrator_threads=1;
 hydra::NvbloxReconstructionAdapter adapter(mc,config,meshConfig,8.f);
 hydra::VolumetricMap runtimeMap(mc,true);
 if(adapter.integrate(input,runtimeMap).empty())throw std::runtime_error("Runtime adapter has no updates");
 adapter.mesh(runtimeMap);
 if(!runtimeMap.getMeshLayer().numBlocks())throw std::runtime_error("Runtime adapter has no mesh blocks");
 hydra::ReconstructionOutput runtimeOutput;runtimeOutput.timestamp_ns=100;
 runtimeOutput.world_t_body=input.world_T_body.translation();runtimeOutput.world_R_body=Eigen::Quaterniond(input.world_T_body.linear());runtimeOutput.setMap(runtimeMap);
 std::cout << "PASS runtime GPU-depth adapter produces Hydra reconstruction output" << std::endl;
 input.color_image.setTo(cv::Scalar(200,30,10));
 hydra::NvbloxReconstructionAdapter rgbAdapter(mc,config,meshConfig,8.f);
 hydra::VolumetricMap rgbMap(mc,true);
 for(int repeat=0;repeat<3;++repeat)rgbAdapter.integrate(input,rgbMap);
 size_t colored=0,unknown=0;
 for(const auto& idx:rgbMap.getTsdfLayer().allocatedBlockIndices()) {
  const auto& block=rgbMap.getTsdfLayer().getBlock(idx);
  for(size_t i=0;i<block.numVoxels();++i) {
   const auto& v=block.getVoxel(i);if(v.weight<=0)continue;
   if(v.color.r==128 && v.color.g==128 && v.color.b==128){++unknown;continue;}
   if(v.color.r<190 || v.color.g<20 || v.color.b>20)throw std::runtime_error("GPU RGB export darkened or changed channel order");
   ++colored;
  }
 }
 if(!colored)throw std::runtime_error("Runtime adapter exported no valid RGB");
 std::cout << "PASS runtime GPU RGB export: colored=" << colored << " unknown=" << unknown << std::endl;

 hydra::NvbloxReconstructionStage local(mc.voxel_size);
 local.integrate(input);
 hydra::VolumetricMap beforeClear(mc,false);
 if(!local.exportTsdfGeometry(beforeClear))throw std::runtime_error("Local map empty before clearing");
 local.clearOutsideRadius(Eigen::Vector3f(100,0,0),8.f);
 hydra::VolumetricMap afterClear(mc,false);
 if(local.exportTsdfGeometry(afterClear))throw std::runtime_error("Distant GPU voxels retained");
 local.integrate(input);
 hydra::VolumetricMap revisit(mc,false);
 if(!local.exportTsdfGeometry(revisit))throw std::runtime_error("Local map failed to reconstruct on revisit");
 std::cout << "PASS local GPU clearing and fresh reconstruction on revisit" << std::endl;

 hydra::NvbloxReconstructionAdapter gpuMeshAdapter(mc,config,meshConfig,8.f,4.f,true);
 hydra::VolumetricMap gpuMeshMap(mc,true);
 gpuMeshAdapter.integrate(input,gpuMeshMap);
 gpuMeshAdapter.mesh(gpuMeshMap);
 size_t gpuFaces=0,gpuLabeled=0;
 for(const auto& idx:gpuMeshMap.getMeshLayer().allocatedBlockIndices()) {
   const auto& mesh=gpuMeshMap.getMeshLayer().getBlock(idx);
   gpuFaces+=mesh.faces.size();
   if(mesh.colors.size()!=mesh.points.size() || mesh.labels.size()!=mesh.points.size())
     throw std::runtime_error("GPU mesh attribute mismatch");
   for(const auto label:mesh.labels)if(label==2)++gpuLabeled;
   for(const auto& face:mesh.faces)for(const auto index:face)
     if(index>=mesh.points.size())throw std::runtime_error("GPU mesh invalid face");
 }
 if(!gpuFaces || !gpuLabeled)throw std::runtime_error("GPU mesh missing geometry or semantics");
 gpuMeshAdapter.mesh(gpuMeshMap);
 size_t repeatFaces=0;
 for(const auto& idx:gpuMeshMap.getMeshLayer().allocatedBlockIndices())repeatFaces+=gpuMeshMap.getMeshLayer().getBlock(idx).faces.size();
 if(repeatFaces!=gpuFaces)throw std::runtime_error("Repeated GPU export duplicated geometry");
 kimera_pgmo::DeltaCompression compressor(0.01);
 kimera_pgmo::HashedIndexMapping remapping;
 hydra::PgmoMeshLayerInterface meshInterface(gpuMeshMap.getMeshLayer());
 auto firstDelta=compressor.update(meshInterface,1000,&remapping);
 auto secondDelta=compressor.update(meshInterface,2000,&remapping);
 if(!firstDelta || !secondDelta)throw std::runtime_error("GPU mesh compression failed");
 for(const auto& idx:gpuMeshMap.getMeshLayer().allocatedBlockIndices()) {
  const auto& mesh=gpuMeshMap.getMeshLayer().getBlock(idx);
  if(mesh.points.size()!=3*mesh.faces.size())throw std::runtime_error("Not triangle soup");
 }
 std::cout << "PASS actual PGMO compressor accepts GPU mesh twice" << std::endl;
 std::cout << "PASS GPU mesh adapter: triangles=" << gpuFaces << " gpuLabeled vertices=" << gpuLabeled << std::endl;
 input.depth_image=cv::Mat(48,64,CV_16UC1);
 try {stage.integrate(input);} catch(const std::invalid_argument&) {return 0;}
 throw std::runtime_error("Invalid input was accepted");
}
