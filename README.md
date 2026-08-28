# <div align="center">Relationship-Aware Hierarchical 3D Scene Graph</div>

This package implements an **enhanced hierarchical 3D scene graph** based on [Hydra](https://github.com/MIT-SPARK/Hydra/tree/main), integrating open-vocabulary features for rooms and objects, and supporting object-relational reasoning.

A **Vision-Language Model (VLM)** infers semantic relationships. A separate
**task reasoning module** combines language and vision-language models to
interpret semantic and relational scene-graph information for task planning.

<div align="center">
    <img src="assets/demo.png" alt="Demo Scene Graph">
</div>

## Setup

### General Requirements

These instructions assume that `python 3.12` and `ros-jazzy-desktop-full` is installed on **Ubuntu 24.04**.

Install general dependencies:

```bash
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

### Building

Build the repository in **Release mode**:

```bash
cd src
git clone git@github.com:ntnu-arl/reasoning_hydra.git
vcs import . < reasoning_hydra/install/packages.repos
rosdep install --from-paths . --ignore-src -r -y

cd ..
colcon build
```

### Python Environment for Semantics and Reasoning

Follow the instructions in [semantic_inference_ros](https://github.com/ntnu-arl/semantic_inference_ros) to set up the Python environment required to run the semantic and reasoning models.

---

## Usage

### Scene Graph Construction

The system supports multiple datasets and online deployment on robots with GPU capabilities (e.g., **Nvidia Jetson Orin AGX**).

#### Uhumans2

Download rosbags from [Uhumans2 dataset](https://web.mit.edu/sparklab/datasets/uHumans2/).

Start the scene graph:

```bash
roslaunch hydra_ros uhumans2.launch
```

In a separate terminal, play the rosbag:

```bash
rosbag play path/to/rosbag
```

#### Replica

Follow [NICE-SLAM instructions](https://github.com/cvg/nice-slam#replica-1) to download posed RGB-D data from Replica scenes.

Run the scene graph:

```bash
roslaunch hydra_ros replica.launch
```

Publish the data:

```bash
roslaunch hydra_ros publish_replica.launch dataset_path:=<replica-dataset-path> scene_name:=<scene-name>
```

#### Habitat-Matterport 3D Semantics Dataset

Follow [HOV-SG instructions](https://github.com/hovsg/HOV-SG?tab=readme-ov-file#habitat-matterport-3d-semantics) (Step 2 can be skipped) to download posed RGB-D data from several scenes.

Run the scene graph:

```bash
roslaunch hydra_ros hm3dsem.launch
roslaunch hydra_ros publish_hm3dsem.launch dataset_path:=<Path to hm3d_trajectories> scene_name:=<Scene name>
```

#### Robot Deployment

Robot deployment requires:

- Robot must provide posed RGB-D data as `sensor_msgs/Image`
- Pose must be provided via **TFs**

Update [robot.launch](https://github.com/ntnu-arl/reasoning_hydra_ros/blob/master/hydra_ros/launch/robot.launch) with the correct TFs and camera topic names, then run:

```bash
roslaunch hydra_ros robot.launch
```

Recorded data for the Alex robot configuration is available from the
[Reasoning Graph Dataset](https://huggingface.co/datasets/ntnu-arl/reasoning-graph-dataset).

To use this data:

```bash
roslaunch hydra_ros robot.launch playback_mode:=True
```

Then play one of the downloaded rosbags:

```bash
rosbag play <bag_to_play> --topics /tf /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/camera_info /camera/color/image_raw/compressed --clock
```

---

### Task Reasoning

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

For ROS 2 launch order, model configuration, topics, services, and the complete
runtime flow, see the
[reasoning-hydra-sg integration README](https://github.com/ihmcrobotics/reasoning-hydra-sg).
