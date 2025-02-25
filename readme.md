# NoodleBot
For hardware assembly instructions and a description of the differences between the simulated and hardware swimmer, see the swimmer [readme](https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/readme.md).

<img src="https://github.com/MurpheyLab/noodlebot/blob/main/swimmer/NoodleBot.png" height="500">

Experiment code is designed to run with ROS Noetic and Python 3 on Ubuntu 20.04

## One-Time Setup
1. Make a ros workspace
    - `$ mkdir -p $HOME/swimmer_ws/src`
2. Clone this repository into the src directory
    - `$ git clone git@github.com:MurpheyLab/noodlebot.git $HOME/swimmer_ws/src/noodlebot`
3. Set up robot tracking (using april tags, marvelmind indoor gps, or other system as desired)
4. Compile ros workspace
    - `$ cd $HOME/swimmer_ws`
    - `$ catkin_make`

The workspace name above is just a suggestion. Throught the rest of this document, we refer to the base directory of this package as `noodlebot` and the ros workspace as `swimmer_ws`.

# Test Setup
### Sourcing the ROS workspace
Any time you open a new terminal, the workspace needs to be sourced
`$ source $HOME/swimmer_ws/devel/setup.bash`

### MaxDiff Parameter Selection
- If you want to change the parameters for maxdiff, modify update `noodlebot/config/swimmer_config.yaml`

### Test Configuration
- Additional arguments can be set in the launch file. To see the configuration options, run `$ roslaunch noodlebot train.launch --ros-args`
- These arguemnts can be changed within the launch file in `noodlebot/launch/train.launch` or can be passed from the command line. An example of command line specification is `$ roslaunch noodlebot train.launch method:=maxdiff cpu:=True`

## Running a test
1. Set all desired parameters in  `noodlebot/config/swimmer_config.yaml`
2. Run launch file(s) in separate terminals
    - `$ roslaunch noodlebot init_cam_n_track.launch` (only if using april tags)
    - `$ roslaunch noodlebot train.launch`
3. When you're ready, the episode can be started with another terminal by running
    - `$ rostopic pub /start_episode std_msgs/Empty "{}" `
    - you'll be prompted to reset robot after each episode


Note: By default, any saved checkpoints are loaded at the beginning of training. Therefore, the `seed` and/or `name_mod` should be changed whenever learning from scratch is desired.

### April Tag Configuration:
Both `tf` and `ffmpeg` dump a lot of warning message into the console. You can filter out filter out excessive console messages by running the following commands instead of those specified in step 2 above
- `$ roslaunch noodlebot init_cam_n_track.launch 2> >(grep -Ev 'mjpeg |buffer_core.cpp' | grep -v '^$')`
- `$ roslaunch noodlebot train.launch april_tags:=True 2> >(grep -Ev 'TF_REPEATED_DATA|buffer_core.cpp' | grep -v '^$')`

### Other commands:
- To pause the test: `$ rostopic pub /pause std_msgs/Empty "{}" `
- To resume the test: `$ rostopic pub /resume std_msgs/Empty "{}" `
and save a checkpoint with  `$ rostopic pub /save_checkpoint std_msgs/Empty "{}" ` 
- To monitor any Arduino errors: `$ rostopic echo /error` 
- To run a gui with the above: `python scripts/gui` or `rosrun noodlebot gui`
- To monitor commands to swimmer joints: `$ rostopic echo /servo_cb` 
- To monitor states from swimmer: `$ rostopic echo /joint_swimmer_info` 


## Code
```
.
├── Arduino
│   ├── MultiServoWithIntegrator    // main test folder (assumes Marvelmind Indoor GPS is used for robot tracking)
│   ├── Sweep                       // alternate program to test joint ranges (useful for swimmer assembly)
│   └── VideoDemo                   // manual program to sweep through joint angles (largely for debugging)
├── CMakeLists.txt                  // ROS configuration file
├── config
│   ├── april_tags.rviz             // provides april tag visualization (april tag tracking only)
│   ├── swimmer_config.yaml         // specifies test configuration
│   ├── swimmer.urdf                // setup file for `joint_test_gui.launch`
│   ├── tags.yaml                   // specifies tag numbers for world and swimmmer (april tag tracking only)
│   ├── tag_settings.yaml           // specifies april tag family (april tag tracking only)
│   └── usb_cam1.yaml               // provides camera calibration info (april tag tracking only)
├── launch
│   ├── enjoy.launch                // executes learned policies
│   ├── init_cam_n_track.launch     // [optional] sets up april tag tracking 
│   ├── joint_test_gui.launch       // [debug] allows user to send joint angles using gui
│   └── train.launch                // main training program
├── msg                             // custom ROS messages to pass information between swimmer and offline compute
│   ├── swimmer_command.msg         // control commands
│   ├── swimmer_info.msg            // state info
│   └── swimmer_reset.msg           // setup/reset info
├── package.xml                     // ROS configuration file
├── readme.md
├── requirements.txt                // list of python packages
├── scripts                         // ROS nodes
│   ├── april_tag_repeater          // [optional] repeats april tags to prevent dropped messages (april tag tracking only)
│   ├── dummy_swimmer               // [debug] mimics publishing by swimmer arduino to debug train and enjoy nodes
│   ├── enjoy                       // ROS node to execute learned policies
│   ├── gui                         // ROS node to allow user to send commands and display error messages
│   ├── joint_repeater              // [debug] reformats default joint_state_gui format to swimmer expects 
│   └── train                       // ROS node to train RL policies
├── setup.py                        // sets up python packages for import by ROS nodes
├── src
│   ├── hlt_planner.py              // planner for Hybrid Learning RL algorithm
│   ├── mpc_lib                     // model and planners for Model Predictive Path Integral Control (MPPI) and MaxDiff RL
│   ├── replay_buffer.py            // memory buffer for RL training
│   ├── sac_lib                     // policy and optimizer for Soft Actor Critic (SAC) learning algorithm 
│   ├── test_env.py                 // interface between swimmer hardware and learning algorithm loop (inspired by MuJoCo Swimmer)
│   └── utils.py                    // helper functions
└── swimmer                         // folder contains instructions for assembling and solidworks files for NoodleBot
```

## Sources 

Hybrid Learning Code is from "Pinosky, A., Abraham, I., Broad, A., Argall, B. and Murphey, T.D., 2023. Hybrid control for combining model-based and model-free reinforcement learning. The International Journal of Robotics Research, 42(6), pp.337-355." [https://github.com/MurpheyLab/HybridLearning.git](https://github.com/MurpheyLab/HybridLearning.git)

MaxDiff Code is from "Berrueta, T.A., Pinosky, A. and Murphey, T.D., 2024. Maximum diffusion reinforcement learning. Nature Machine Intelligence, pp.1-11." [https://github.com/MurpheyLab/MaxDiffRL.git](https://github.com/MurpheyLab/MaxDiffRL.git)


## Copyright and License
The implementations of MaxDiff contained herein are copyright (C) 2024 - 2025 by Allison Pinosky and Todd Murphey and are distributed under the terms of the GNU General Public License (GPL) version 3 (or later). Please see the LICENSE for more information.

Contact: MurpheyLab@u.northwestern.edu

Lab Info:
Todd D. Murphey
https://murpheylab.github.io/
Northwestern University