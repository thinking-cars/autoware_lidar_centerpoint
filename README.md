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

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/thinking-cars/autoware_lidar_centerpoint:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml
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
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

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
