**NOTE: this software is part of the BenchBot software stack, and not intended to be run in isolation. For a working BenchBot system, please install the BenchBot software stack by following the instructions [here](https://github.com/roboticvisionorg/benchbot).**

# BenchBot Simulator

![benchbot_simulator](./docs/benchbot_simulator.gif)

The BenchBot Simulator is an extension of the [NVIDIA Isaac SDK](https://developer.nvidia.com/isaac-sdk) that establishes ROS communications to a running instance of an Unreal Engine-based [NVIDIA Isaac SIM](https://developer.nvidia.com/isaac-sim). BenchBot simulator is explicitly linked to version 2019.2 of Isaac, the last version with direct support for the Unreal Engine-based simulator. In the future we intend to move to [Omniverse](https://developer.nvidia.com/nvidia-omniverse), NVIDIA's new 3D production pipeline with RTX support.

BenchBot simulator provides direct access to the following data on the simulated robot. Access is via ROS on the topic provided in brackets:

- RGB images from the top camera (`/camera/color/image_raw`)
- Camera information for RGB images from the top camera (`/camera/color/camera_info`)
- Depth images from the top camera (`/camera/depth/image_raw`)
- Camera information for depth images from the top camera (`/camera/depth/camera_info`)
- Laserscan data from the LiDAR (`/scan_laser`)
- Raw odometry data (`/odom`)
- A full transform tree (`/tf`)

Direct control of the robot is also facilitated via:

- 3D twist velocities sent to topic `/cmd_vel`

## Installing BenchBot Simulator

**Please see the note at the top of the page; installation of BenchBot Simulator in isolation is generally not what you want!**

If you are sure you need to install the simulator in isolation, the following steps should be sufficient. Note there are a significant number of driver & software requirements for your system:

1. Download version 2019.2 of the Isaac SDK from the [NVIDIA site](https://developer.nvidia.com/isaac/downloads). If creating your own environments, also download version 2019.2 of Isaac SIM (_not_ NavSim). You will have to create / sign in to an NVIDIA developer account, and look in the "Archive" drop down for version 2019.2.

2. Either setup your system with Isaac SIM, or download our environments:

   a) Follow the [install instructions](https://docs.nvidia.com/isaac/isaac_sim/setup.html) for Isaac SIM to get Unreal Engine (through IsaacSimProject) running on your system. You will have to link Epic Games to your Github account to get access.

   b) Download our latest environments: [Isaac Development Environments](https://github.com/benchbot-addons/envs_isaac_develop/blob/master/.remote), and [Isaac Challenge Environments](https://github.com/benchbot-addons/envs_isaac_challenge/blob/master/.remote)

3. Install the Isaac SDK by following the instructions [here](https://docs.nvidia.com/isaac/archive/2019.2/doc/setup.html).

4. Clone the BenchBot simulator, apply our patches to the installed Isaac SDK, & build the simulator using the Bazel wrapper script (ensure the environment variable `ISAAC_SDK_PATH` is set to where you installed Isaac SDK):
   ```
   u@pc:~$ git clone https://github.com/roboticvisionorg/benchbot_simulator && cd benchbot_simulator
   u@pc:~$ .isaac_patches/apply_patches
   u@pc:~$ ./bazelros build //apps/benchbot_simulator
   ```

## Running BenchBot Simulator

The BenchBot simulator interface is run alongside a running Isaac Unreal Engine Simulator. To get both components running:

1. Start the Unreal Engine Simulator, either via our precompiled environments or the IsaacSimProject Unreal Editor:

   ```
   u@pc:~$ ./IsaacSimProject <map_name> \
       -isaac_sim_config_json='<path_to_isaac>/apps/carter/carter_sim/bridge_config/carter_full.json' \
       -windowed -ResX=960 -ResY=540 -vulkan -game
   ```

2. Launch BenchBot simulator Isaac application (you first need to hardcode the pose unfortunately...):
   ```
   u@pc:~$ START_POSE=<robot_start_pose> \
       sed -i "0,/\"pose\":/{s/\(\"pose\": \)\(.*\)/\1$START_POSE}" \
       <path_to_isaac>/apps/carter/carter_sim/bridge_config/carter_full_config.json
   u@pc:~$ ./bazelros run //apps/benchbot_simulator
   ```

At this point you will have a running Isaac Unreal Engine Simulator, with sensorimotor data available from the robot in ROS!

## Using BenchBot Simulator with the BenchBot Robot Controller

The [BenchBot Robot Controller](https://github.com/RoboticVisionOrg/benchbot_robot_controller) is a wrapping ROS / HTTP hybrid script that manages running robots and their required subprocesses. See the `carter_sim.yaml` configuration in the [BenchBot Supervisor](https://github.com/roboticvisionorg/benchbot_supervisor) for an example configuration of how to run BenchBot Simulator through the Robot Controller.
