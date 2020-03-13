**NOTE: this software is part of the BenchBot Software Stack, and not intended to be run in isolation. For a working BenchBot system, please install the BenchBot Software Stack by following the instructions [here](https://github.com/RoboticVisionOrg/benchbot).**

# BenchBot Simulator

![benchbot_simulator](./docs/benchbot_simulator.gif)

The BenchBot Simulator is an extension of the [Nvidia Isaac SDK](https://developer.nvidia.com/isaac-sdk) that establishes ROS communications to a running instance of an Unreal Engine-based [Nvidia Isaac SIM](https://developer.nvidia.com/isaac-sim). BenchBot simulator is explicitly linked to version 2019.2 of Isaac, the last version with direct support for the Unreal Engine-based simulator. In the future we intend to move to [Omniverse](https://developer.nvidia.com/nvidia-omniverse), Nvidia's new 3D production pipeline with RTX support.

BenchBot simulator provides direct access to the following data on the simulated robot. Access is via ROS on the topic provided in brackets:

- RGB images from the top camera (`/camera/color/image_raw`)
- Camera information for RGB images from the top camera (`/camera/color/camera_info`)
- Depth images from the top camera (`/camera/depth/image_raw`)
- Camera information for depth images from the top camera (`/camera/color/camera_info`)
- Laserscan data from the LiDAR  (`/scan_laser`)
- Raw odometry data (`/odom`)
- A full transform tree (`/tf`)

Direct control of the robot is also facilitated via:

- 3D twist velocities sent to topic `/cmd_vel`

## Installing BenchBot Simulator

**Please see the note at the top of the page; installation of BenchBot Simulator in isolation is generally not what you want!**

If you are sure you need to install the simulator in isolation, the following steps should be required. Note there are significant number of driver & software requirements for your system:

1. Download version 2019.2 of the Isaac SDK & Isaac SIM (*not* NavSim) if creating your own environments from the [Nvidia site](https://developer.nvidia.com/isaac/downloads). You will have to create / sign in to an Nvidia developer account, and look in the "Archive" drop down for version 2019.2.

2. Either setup your system with Isaac SIM, or download our environments: 
    
    a) Follow the [install instructions](https://docs.nvidia.com/isaac/isaac_sim/setup.html) for Isaac SIM to get Unreal Engine (through IsaacSimProject) running on your system. You will have to link Epic Games to your Github account to get access.

    b) Download our latest environments ([this file](https://cloudstor.aarnet.edu.au/plus/s/egb4u65MVZEVkPB/download) will always provide a URL for the latest environments).

3. Install the Isaac SDK by following the instructions [here](https://docs.nvidia.com/isaac/archive/2019.2/doc/setup.html).

4. Clone the BenchBot simulator, apply our patches to installed Isaac SDK, & build the simulator using the Bazel wrapper script (ensure the environment variable `ISAAC_SDK_PATH` is set to where you installed Isaac SDK):
    ```
    u@pc:~$ git clone https://github.com/RoboticVisionOrg/benchbot_simulator && cd benchbot_simulator
    u@pc:~$ .isaac_patches/apply_patches
    u@pc:~$ ./bazelros build //apps/benchbot_simulator
    ```

## Running BenchBot Simulator

Running the BenchBot simulator requires two parts:

1. The Unreal Engine Simulator, either via our precompiled environments or the IsaacSimProject Unreal Editor:
    ```
    u@pc:~$ ./IsaacSimProject <map_name> \
        -isaac_sim_config_json='<path_to_isaac>/apps/carter/carter_sim/bridge_config/carter_full.json' \
        -windowed -ResX=960 -ResY=540 -vulkan -game
    ``` 

2. The BenchBot simulator Isaac application (you first need to hardcode the pose unfortunately...):
    ```
    u@pc:~$ START_POSE=<robot_start_pose> \
        sed -i "0,/\"pose\":/{s/\(\"pose\": \)\(.*\)/\1$START_POSE}" \
        <path_to_isaac>/apps/carter/carter_sim/bridge_config/carter_full_config.json
    u@pc:~$ ./bazelros run //apps/benchbot_simulator
    ```

Alternatively, you can run everything through the wrapper script which controls the lifecycle of the entire simulator stack via a RESTful API (only or precompiled environments are supported currently):
```
u@pc:~$ scripts/simulator_controller \
    --path-envs <precompiled_envs_location> \
    --path-isaac <path_to_isaac> \
    --path-simulator <local_path_to_this_repo> \
    --port <#> \
    .benchbot_data_files <colon_separated_maps> <colon_separated_start_poses>
```

Look inside `<precompiled_envs_location>/.benchbot_data_files/` for map metadata including start poses & map names.

## Interacting with the BenchBot Simulator Controller

The BenchBot Simulator Controller wrapper script `scripts/simulator_controller` exposes a RESTful API for getting data about simulator state, & managing the lifecycle of running simulator instances. They are currently all implemented as `GET` requests, which should probably be tweaked in the future.

The API includes the following commands:

| Request Route | Response JSON Format | Description |
| --------------|:---------------:|-------------|
| `/`           | `{Hello, I am the BenchBot simulator}` | Arbitrary response to confirm connection |
| `/is_collided` | `{'is_collided': True|False}` | Goes to `True` once the robot has collided with an obstacle (never returns to false after that point) |
| `/is_dirty` | `{'is_dirty': True|False}` | Goes to `True` once the robot has moved for the first time |
| `/is_running` | `{'is_running': True|False}` | Returns `True` if all simulator ROS topics are alive |
| `/map_selection_number` | `{'map_selection_number': int}` | Returns the currently running map number out of the list of maps provided on startup |
| `/next` | `{'next_success': True|False}` | Kills any currently running simulator & starts the *next* simulated environment from the list provided on startup |
| `/reset` | `{reset_success: True|False}` | Kills the currently running simulator & restarts the in the *same* simulated environment |
| `/restart` | `{restart_success: True|False}` | Kills the currently running simulator & restarts the in the *first* simulated environment |

