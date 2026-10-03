<!--
Copyright (c) 2026, IHMC Robotics Lab.
All rights reserved.
-->

# Compression profiling baseline

This instrumentation retains the existing algorithms, resolution, callback threading and update frequency. It splits the existing frontend timers; nested durations must not be added to their parent totals.

- `frontend/mesh_input_copy`: create the temporary layer of changed, non-archived blocks (including their semantic feature data).
- `frontend/mesh_compressor_core`: entire DeltaCompression update.
- `frontend/compressor_archive`: archive block faces.
- `frontend/compressor_remap`: process changed blocks, hash vertices, update observations and indices.
- `frontend/compressor_active_vertices`: emit all active compressed vertices.
- `frontend/compressor_active_faces`: update active faces and per-block input-to-output mappings.
- `frontend/compressor_archived_faces`: update partially archived faces.
- `frontend/mesh_delta_apply`: apply the delta to the persistent graph mesh.
- `frontend/mesh_invalidate_edges`: invalidate affected mesh associations.
- `frontend/dgraph_prune`, `dgraph_integrate`, `dgraph_copy_vertices`, `dgraph_build_graph`: deformation graph sub-stages.
- `frontend/object_graph_lock_wait`: time waiting to acquire the object-update graph mutex.
- `frontend/feature_cleanup`: clear delta, stored mesh, and compressor features/IDs while holding the graph lock.

The compressor core also includes small setup/finalization work outside its five internal stages. The internal timers use steady_clock and are exported to Hydra's normal timing recorder. Aggregate callback timing includes work plus waiting, not just thread creation overhead. These timers do not measure allocation counts, bytes copied, or all lock contention.

Use the same SVO, start/end frames, configuration and viewers for baseline and optimized runs. Save timing_stats.csv and raw stage CSVs after graceful shutdown. Compare per-update samples and first/last quarters; report sample counts and map size. Do not infer speedup from unrelated runs.

Initial source inspection: changed blocks are copied before compression, but active vertex/face output scans the active compressed map; feature cleanup scans the persistent mesh. Profile before changing remapping, archival semantics, or object/place associations. A persistent callback pool is a separate experiment and must support nested dispatch without deadlock.

## First allocation optimization

The follow-up build moves temporary voxel sets and semantic feature values instead of copying them, merges duplicate set lookup/insertion, and replaces deformation compression's consecutive-index hash map with a vector. It does not alter voxel size, vertex traversal, deduplication rules, archival policy, or update frequency. Active-face processing remains unchanged pending further evidence.

Additional timers isolate `mesh_remapping_replace`, `mesh_interface_setup`, `mesh_temporary_destroy`, and `mesh_timing_bookkeeping`. The baseline's 57.44 ms remainder must not be called destruction time until measured. Core setup/finalization remains outside its five internal stages.

Correctness validation uses delta-compression, mesh-delta, and block-compression tests. The test container needs a PCL fixture compatibility adjustment (`std::vector<uint>{...}` assignments become `{...}`); this does not change runtime code. Baseline: 16/17 tests pass; `TestMeshDelta.archiveVerticesCorrect` already fails (archived count 1 vs expected 3). Compare the candidate against this known baseline. Live speedup requires the same recording and route duration; no live gain is claimed from code inspection.

## Selected-block view and persistent remapping

The frontend now constructs a read-only interface over updated, unarchived input
blocks instead of deep-copying their geometry and semantic feature vectors.
A metadata-only hash table reproduces the old temporary layer's traversal order.
The input map stays alive throughout the synchronous compressor update.

The frontend opts into persistent output remapping. The compressor removes blocks
not updated in this pass, removes indices beyond a shrinking block, and overwrites
retained entries. Other callers retain the original update API and behavior.
Detailed compressor timings are recorded under one recorder lock per batch.
`mesh_input_copy` disappears; block selection is included in
`mesh_interface_setup`. `mesh_remapping_replace` measures first allocation only;
pruning/overwrite work is included in `compressor_active_faces`.

Validation: six existing multi-update/archival scenarios also pass with reused
remapping, and selected-view tests match copied-layer ordering, data and vertex
mappings across 200 blocks (including archived/unmodified blocks and an empty
selection). The known baseline `TestMeshDelta.archiveVerticesCorrect` failure is
excluded from this focused run. No live speedup has yet been measured.
