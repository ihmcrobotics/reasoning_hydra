# <div align="center">Relationship-Aware Hierarchical 3D Scene Graph</div>

This package implements an **enhanced hierarchical 3D scene graph** based on
[Hydra](https://github.com/MIT-SPARK/Hydra/tree/main). It adds
open-vocabulary features for rooms and objects, object relationships, semantic
object search, and graph-based navigation.

> **Running the IHMC ROS 2 system:** Users who want to build, launch, or deploy
> Reasoning Hydra can go directly to the
> [IHMC Reasoning Hydra README](../ihmc_reasoning_hydra/README.md). It contains
> the recommended Docker workflow, local installation, model provisioning,
> launch files, sensor topics, and runtime configuration. The remainder of
> this document describes the core `hydra` package and its architecture.

A **Vision-Language Model (VLM)** infers semantic relationships. A separate
**task reasoning module** combines language and vision-language models to
interpret semantic and relational scene-graph information for task planning.

For the IHMC custom YOLOv8 model, Hydra uses
`config/label_spaces/ihmc_custom_yolov8_label_space.yaml`. This preserves the
original Alex IDs, adds the IHMC-specific object classes, and excludes the
robot's own `robot_hand` detection from scene-graph objects. `person_operator`
is retained as a separate dynamic class for designated personnel wearing an
identifying vest, while `trash_can` and `bottle` reuse the existing Alex
concepts.

Door components remain distinct: `door` is the complete semantic assembly,
`door_panel` is movable, and `door_frame` is fixed and should provide the
navigation position. The current custom model does not detect `door_frame`, so
stable door assembly and panel-to-frame association require a future model (or
another frame detector); the movable panel is not silently treated as the
permanent door pose.

<div align="center">
    <img src="assets/demo.png" alt="Demo Scene Graph">
</div>

## Core package

The ROS package name is `hydra`. It is an `ament_cmake` C++ library that owns
the scene-graph algorithms and configuration. ROS 2 nodes, message bridges,
semantic inference, model downloads, launch orchestration, and Docker wrappers
live in the surrounding workspace packages.

To build the core package and its in-workspace dependencies from the
`reasoning-hydra-sg` repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to hydra
```

For a complete deployment, use the integration instructions linked at the top
instead of building or launching this package in isolation.

## Scene-graph reasoning

Reasoning Hydra augments the hierarchical scene graph with object
relationships and task-aware navigation. It separates candidate retrieval,
relationship evaluation, and graph path generation:

```text
Structured task objects and embeddings
        ↓
ObjectSearchModule
Retrieves candidate object nodes from the scene graph
        ↓
Candidate objects and their active relationships
        ↓
External semantic/VLM reasoning bridge
Selects relationships relevant to the task
        ↓
NavigationModule
Computes graph paths to the selected objects
        ↓
Navigation result
```

The core package owns the graph-side operations:

- maintaining object and relationship information in the scene graph;
- searching object nodes using semantic query embeddings;
- publishing candidate objects and active object relationships;
- accepting selected task-relevant objects from a reasoning bridge;
- computing paths over the place graph, including Dijkstra search; and
- publishing the resulting navigation candidates and paths.

Natural-language parsing, image segmentation, language embeddings, and VLM
inference are integration-layer responsibilities. They may be implemented with
different models as long as they publish the message and feature formats
expected by Reasoning Hydra. The IHMC ROS 2 workspace currently connects local
Qwen instruction parsing, OpenCLIP retrieval, Qwen3VL relationship features,
and Cosmos-Reason2 reasoning.

## Package boundaries

| Responsibility | Package or documentation |
| --- | --- |
| Scene-graph construction, object search, relationships, and navigation algorithms | This `hydra` package |
| ROS 2 subscriptions, publishers, and reasoning/navigation bridges | `hydra_ros` |
| Segmentation, embeddings, task parsing, and VLM inference | `semantic_inference_ros` |
| IHMC launch orchestration, model provisioning, Docker, and deployment | [`ihmc_reasoning_hydra`](../ihmc_reasoning_hydra/README.md) |
| Workspace overview and external visualization contract | [reasoning-hydra-sg README](../../README.md) |

This separation keeps the core algorithms independent of a particular camera,
robot, model provider, or deployment environment.
