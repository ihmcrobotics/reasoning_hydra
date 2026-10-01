# Experimental internal nvblox contract test

Build with `HYDRA_ENABLE_NVBLOX_STAGE=ON` and `HYDRA_ENABLE_TESTS=ON`; run `hydra_nvblox_contract_check` with an NVIDIA GPU available. The ordinary CPU test build does not include this executable.

Checks cover RGB-D meshing, strided image input, changed pose, TSDF coordinate conversion including negative coordinates, signed distances, weights, update flags, unchanged delta suppression, partial-map retention, explicit removal events, and appearance-only geometry invariance.

Semantic checks use the real Hydra MLE integrator: label 2, panoptic ID 7, a three-component feature vector, and dynamic label 3. The tested synthetic observation labeled 2,010 GPU-derived voxels without modifying their TSDF distances or weights.

This validates semantic association, not end-to-end scene-graph parity. Dynamic and invalid labels are now masked from GPU depth fusion using Hydra's configured semantic policy. Tests verify zero observed voxels for fully excluded frames and successful static fusion afterward at the same pose. Viewpoint caching is disabled because depth/masks can change at a fixed pose. Color masking is tested: a fully excluded repaint leaves mesh colors unchanged, and a mixed-label frame changes 183 vertex colors while preserving 191. Exact mixed-label boundary equivalence with the CPU interpolator remains unverified. GPU-only operation remains blocked pending masking, archival, and GVD/output wiring.

The GVD test feeds the converted TSDF to Hydra's GVD integrator and checks finite distances for 4,032 observed voxels. `gpu_validate_gvd: true` enables a separate persistent validation layer in `both` mode using default GVD parameters. It now instantiates Hydra's compression place-graph extractor, but does not publish its output into the live scene graph. Full GPU TSDF readback still occurs. The validation CPU window now archives distant GVD blocks and reloads geometry from retained GPU evidence on revisit. This is a correctness diagnostic, not a performance mode.

A closed-room fixture with six overlapping camera views generated 14 place nodes from GPU TSDF through Hydra GVD and the compression extractor. The earlier isolated-plane fixture validates distances but is not expected to establish place-graph topology. Room semantics, graph associations, archival and live-output parity are not yet tested.

## Archival/revisit validation

- The optional validation path uses Hydra's configured active radius and robot position for block-center window selection.
- GVD/extractor archival is invoked before removing distant CPU TSDF blocks. GPU TSDF evidence is deliberately retained; this does not bound VRAM.
- A synthetic leave/revisit test restored distance-field blocks without discarding GPU evidence.
- Graph identity/reconciliation is unresolved: the test had 14 nodes initially and 31 after revisit. The test asserts voxel restoration and graph existence only, not topology equivalence. This output must remain isolated from the live scene graph until archived/revisited node reconciliation is validated.

## Place lifecycle clarification

The increasing `numValidationPlaces()` count includes retained archived nodes, not just active places. Repeated leave/revisit testing produced total/active counts 51/20, 68/17 and 84/16. Active counts went to zero outside the window. This does not establish topology equivalence or stable identities on revisit.

Hydra's normal `GvdPlaceExtractor` transfers active-node updates and deleted-node/edge events to the scene graph; `UpdatePlacesFunctor` proposes backend merges using position and distance compatibility. The isolated validator does not execute this full lifecycle. Do not add a nearest-neighbor deduplication shortcut to the GVD extractor or claim the retained count alone is a regression. Production routing and merge tests remain required.

## Frontend output connection

The optional validation path now also constructs `ReconstructionOutput` from the GPU-derived CPU mirror, including explicit archived blocks, and calls Hydra's existing `GvdPlaceExtractor::detect` and `updateGraph` on a separate `DynamicSceneGraph`. The repeated-revisit test completes with 43 frontend place nodes. This verifies routing and graph existence, not correct reconciliation or deletion-event parity. It retains the lower-level GVD validator for comparison, so this diagnostic computes GVD twice. Backend merge processing, object/mesh associations and production publication remain pending; CPU still owns the live graph.

## Experimental runtime adapter

The `nvblox` selector now routes GPU-fused TSDF through `NvbloxReconstructionAdapter` into the normal reconstruction output. CPU appearance/semantic association and meshing precede the existing frontend/backend. The synthetic runtime adapter test passed; CPU-only and GPU-enabled reconstruction-module syntax checks passed. The diagnostic validator remains isolated in `both` mode.

Full image compilation, live frontend/backend parity, archived semantic retention and bounded GPU memory are not verified. Full geometry readback currently happens per observation; no speedup is claimed. This experimental selection must not be treated as a production replacement yet.

## GPU RGB transfer regression

Internal nvblox mode integrates RGB after depth using the same input pose and semantic mask. Export uses ColorVoxel observation weight to accept valid RGB, with neutral gray for unobserved color. CPU semantic association disables RGB blending so GPU geometry weight cannot darken transferred colors. Color-only changes also mark blocks for meshing. CPU-only integration retains its existing behavior.

The synthetic contract checks repeated constant-color observations through the runtime adapter for channel order and darkening. Live visual confirmation and full image rebuild are still required; existing archived colors are not repaired retroactively.

## PGMO triangle layout regression

The GPU export must expand indexed triangles into consecutive triples, including corresponding color and semantic attributes. PgmoMeshLayerInterface/DeltaCompression consume vertices as triangle soup rather than reading Mesh::faces. Indexed GPU vertices can therefore cause out-of-range access or incorrect faces even when all explicit face indices are valid. The contract test now sends GPU output through the real DeltaCompression twice and checks the triangle-soup vertex count.
