# autoware_lidar_centerpoint

<p align="center">
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/thinking-cars/autoware_lidar_centerpoint/releases/latest"><img src="https://img.shields.io/github/v/release/thinking-cars/autoware_lidar_centerpoint"/></a>
  <a href="https://github.com/thinking-cars/autoware_lidar_centerpoint/blob/main/LICENSE"><img src="https://img.shields.io/github/license/thinking-cars/autoware_lidar_centerpoint"/></a>
  <br>
  <a href="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/docker-ros.yml"><img src="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/compose-oci.yml"><img src="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://thinking-cars.github.io/autoware_lidar_centerpoint"><img src="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/consistency.yml"><img src="https://github.com/thinking-cars/autoware_lidar_centerpoint/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

This repository integrates the [`autoware_lidar_centerpoint`](https://github.com/autowarefoundation/autoware_universe/tree/f15e433ab4da38792d3b464a5ce4856678ef79f1/perception/autoware_lidar_centerpoint) package for 3D lidar object detection from [Autoware Universe](https://github.com/autowarefoundation/autoware_universe) into the [OpenADS](https://github.com/openads-project) ecosystem, which emphasizes a modular microservice architecture. Hence, this repository is self-contained and includes only the necessary dependencies.

> [!IMPORTANT]
> This repository is a prototypical integration of `autoware_lidar_centerpoint` into [OpenADS](https://github.com/openads-project) for testing and benchmarking purposes. Thus, only necessary changes were made for integration without adopting the Autoware module to the OpenADS consistency guidelines. These adoptions will only be made in case of a full integration into OpenADS after testing and benchmarking. 

The following teaser shows **detected objects (turquoise)** using the Waymo Open Dataset provided by [autonomy_datasets](https://github.com/thinking-cars/autonomy_datasets).

![Teaser](./assets/teaser-waymo.gif)

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>


## 🚀 Quick Start

Run the `autoware_lidar_centerpoint` model on lidar point clouds from the NVIDIA PhyiscalAI AV dataset published by [autonomy_datasets](https://github.com/thinking-cars/autonomy_datasets):

```bash
export HF_TOKEN=<your-huggingface-access-token>
docker compose up
```

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/thinking-cars/autoware_lidar_centerpoint.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd autoware_lidar_centerpoint
    git submodule update --init --recursive
    ```
1. Run [autonomy_datasets](https://github.com/thinking-cars/autonomy_datasets) along with a development container for `autoware_lidar_centerpoint`:
    ```bash
    docker compose -f docker-compose.dev.yml up
    ```
1. Open [Visual Studio Code](https://code.visualstudio.com) and attach to the running development container.
    > *F1 / Dev Containers: Attach to Running Container... / autoware_lidar_centerpoint-autoware_lidar_centerpoint-1*
1. Navigate to the workspace and compile the source code.
    > *Ctrl+O / `/docker-ros/ws/src/target/` / Ctrl+Shift+B*
1. Run the compiled ROS node.
    ```bash
    ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint input/pointcloud:=/lidar_01/point_cloud output/objects:=/detected_objects use_sim_time:=true
    ```

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://thinking-cars.github.io/autoware_lidar_centerpoint).

| Package | Description |
| --- | --- |
| [autoware_cuda_dependency_meta](autoware_cuda_dependency_meta/README.md) | Virtual package that provides an empty cuda stub package |
| [autoware_cuda_utils](autoware_cuda_utils/README.md) | cuda utility library |
| [autoware_interpolation](autoware_interpolation/README.md) | The spline interpolation package |
| [autoware_lidar_centerpoint](autoware_lidar_centerpoint/README.md) | The autoware_lidar_centerpoint package |
| [autoware_object_recognition_utils](autoware_object_recognition_utils/README.md) | The autoware_object_recognition_utils package |
| [autoware_tensorrt_common](autoware_tensorrt_common/README.md) | tensorrt utility wrapper |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

This project is maintained by [Thinking Cars](https://www.thinking-cars.de). We acknowledge the work of the [Autoware](https://autoware.org/) contributors and are happy to discuss potential collaborations.
