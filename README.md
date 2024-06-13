<div align="center">
  <p>
    <a href="https://github.com/ultralytics/assets/releases/tag/v8.2.0" target="_blank">
      <img width="100%" src="https://raw.githubusercontent.com/ultralytics/assets/main/yolov8/banner-yolov8.png" alt="YOLO Vision banner"></a>
  </p>
</div>


### Repository Overview

Welcome to the ROS [Ultralytics](https://github.com/ultralytics/ultralytics) testing playground! This repository is designed as a comprehensive environment for experimenting with various models. The examples provided in this repository are crafted to function seamlessly across different settings and with diverse sensor inputs, whether operating in the real world or within simulation environments such as Gazebo, MuJoCo, or other simulators.

<div align="center">
  <p>
      <img width="100%" src="https://github.com/ambitious-octopus/rosbot_ros/assets/3855193/764fce17-ed8d-40e0-b599-89c7eff7a879" alt="YOLO Vision banner"></a>
  </p>
</div>

### ROSbot_ROS 🤖🚀

This is a fork of the ROS packages for ROSbot 2.0 and ROSbot 2.0 Pro. Additional features and improvements have been added to the original repository for testing with the [Ultralytics](https://github.com/ultralytics/ultralytics) repository. 
These changes include:
- 🐋 A Dockerfile for building the ROSbot packages in a Docker container
- 🏃‍♂️ A run.sh script for running the ROSbot packages in a Docker container
- 🌍 A couple of new worlds for the Gazebo simulation
- 📂 A new examples folder with many examples for experimenting with the Ultralytics package

For implementing with the real robot, please use the [original repository](https://github.com/husarion/rosbot).

### Key Features

- **Versatile Examples:** The scripts located in the `examples` directory are highly adaptable, allowing you to test them in multiple environments and with a variety of sensors.
- **Simulation Support:** Compatible with popular simulation platforms including Gazebo and MuJoCo.
- **Docker Integration:** The repository is optimized for use with Docker, ensuring a consistent and reproducible setup.



### Installation

#### Prerequisites

Ensure you have Docker installed on your system. If Docker is not already installed, follow the [official Docker installation guide](https://docs.docker.com/get-docker/) for your operating system.

#### Setting Up the Docker Environment

To build and launch the Docker container, execute the following command:

```bash
./run.sh
```

This script will handle the creation and initialization of the Docker container, providing a contained and controlled environment for your testing needs.

#### Running Simulations

Once you have successfully accessed the Docker container, you can start the simulation by running:

```bash
./simulate.sh
```

This script performs the following actions:

1. **Launch Gazebo:** Launches the Gazebo simulation environment.
2. **Calls the Bringup Robot:** Initializes the robot within the simulation.
3. **Opens RViz:** Opens RViz for visualization, enabling you to observe and interact with the simulated environment.

At this stage, you are ready to test the example scripts provided in the `examples` directory.

### Example Scripts

<div align="center">
  <p>
      <img width="100%" src="https://github.com/ambitious-octopus/rosbot_ros/assets/3855193/55e8b00a-2df6-432a-b382-13d4a82c76f5" alt="YOLO Vision banner"></a>
  </p>
</div>


The `examples` directory contains a variety of scripts designed to showcase different functionalities and use cases. These scripts are written to be modular and can be adapted for different sensors and environments.

```bash
cd examples
```

#### Navigation

Sitting still is boring, right? Spice things up by opening a terminal and launching:

```bash
roslaunch rosbot_navigation rosbot_teleop.launch
```

This script lets you control your ROSbot using your keyboard, turning your ideas into action in no time!


### Additional Information

- **Extensibility:** The repository is designed to be easily extendable. You can add your own models and scripts to further customize your testing environment.
- **Community Support:** If you encounter any issues or have questions, feel free to open an issue on the repository or reach out to the community for support.

