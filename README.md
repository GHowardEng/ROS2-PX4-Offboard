# ROS2-PX4-Offboard
ROS2 node examples for bi-directional interfacing with PX4 and implementing custom offboard control algorithms.

Based on source code from [PX4](https://github.com/PX4) and [Jaeyoung Lim](https://github.com/Jaeyoung-Lim/px4-offboard).

## Setup
<b>This is intended to be used on a companion computer or workstation running Ubuntu 22.04.</b>

### External Dependencies
Refer to the [official PX4 setup guide for ROS2](https://docs.px4.io/main/en/ros2/user_guide.html):

* Installation and setup of the [PX4 developement environment](https://docs.px4.io/main/en/ros2/user_guide.html#install-px4) is not strictly required but is recommended for simualtion purposes and source code.
  * Note that this project is developed for and tested with the upcomming [version 1.15 of PX4](https://github.com/PX4/PX4-Autopilot/tree/v1.15.0-rc1) for full support of the uXRCE-DDS interface.
* Install [ROS2 Humble](https://docs.px4.io/main/en/ros2/user_guide.html#install-ros-2).
* Install the [Micro XRCE-DDS Agent](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client).

### ROS Node Setup
1. Clone this example repository: `git clone https://github.com/GHowardEng/ROS2-PX4-Offboard.git`
* The `dds_topics.yaml` file provided is a modified copy of the vanilla version from [PX4-Autopilot/src/modules/uxrce_dds_client](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) with a number of additional topics added.
  * You may use the vanilla version of this file already included in the PX4-Autopilot reposity - most telemetry and commands are already present.
  * If you wish to use the custom version for addition topics, you must replace the file in your local copy of the PX4 repo, and generate a custom build of the firmware (upload to your target hardware if required). 
  * You may add or remove topics as desired for your application.

2. <b>IMPORTANT:</b> Verify px4_msgs is synchronized with the specific version of PX4-Autopilot being used!
* A version of the [px4_msgs repo](https://github.com/PX4/px4_msgs) is included as a submodule in the `ws_offboard_example/src` folder. This should be compatible with PX4 version `v1.15.0-rc1`
* If using another major release, you may checkout the branch assoicated with this release version in the `px4_msgs` submodule (e.g. [1.14](https://github.com/PX4/px4_msgs/tree/release/1.14))
* If using any other version, such as running PX4 from the `main` branch, or encounter message compatibility issues, you should manually synchronize the message definitions:

  * Checkout the correct branch/tag of the PX4-Autopilot repo you wish to interface with.
  * Delete all `*.msg` files in workspace `src/px4_msgs/msg/` folder and copy all `*.msg` files from `PX4-Autopilot/msg/` in it. Assuming that this repository and the PX4-Autopilot repository are placed in your home folder, you can run: 
  ```
  rm -f ~/ROS2-PX4-Offboard/ws_offboard_example/src/px4_msgs/msg/*.msg
  cp ~/PX4-Autopilot/msg/*.msg ~/ROS2-PX4-Offboard/ws_offboard_example/src/px4_msgs/msg/
  ```
  
3. Build the ROS node:
* Navigate to the `ws_offboard_example` folder
* Run a build with `colcon build`. This will build all packages under the `/src` folder.

## Running
1. PX4 and uXRCE-DDS Client:
  * If running software-in-the-loop (SITL), start a PX4 SITL instance on your machine with `make px4_sitl gz_rc_cessna`. The uXRCE-DDS client is started automatically on a SITL instances, with network configuraiton `localhost:8888'.
  * If running with real PX4 hardware, boot up the autopilot and access the system shell (MAVLink Console in QGroundControl). Start the uXRCE-DDS client with `uxrce_dds_client start -h <Companion IP> -p <Port>` (note this assumes UDP protocol).

2. uXRCE-DDS Agent (on companion device):
  * From a new terminal instance, boot the agent with `MicroXRCEAgent udp4 -p <Port>` (matching port that selected with PX4's client).
  * At this point, notifications should be printed on both the PX4 shell (client-side) and terminal instance (agent-side) showing that topics are being published and recieved.
  * Note that uXRCE-DDS agent initialization may be automated on Ubuntu by adding the command to 'Startup Applications Preferences'.

3. ROS Node:
  * From the `ws_offboard_example` folder, the node may be started with the provided start-up script: `./startNode.sh`.
  * You should see notifications from the subscriber callback functions indicating that the various topics are being recieved.

4. Activate Offboard Mode:
  * Arm the vehicle and takeoff (if in simulation). The ROS node should detect the arming event.
  * Switch PX4 to Offboard mode via QGroundControl or using `commander` in the PX4 system shell.
  * The ROS node should detect the mode change, and begin sending attitude commands to the autopilot.
    * The example behavior is to command a pitch attitude of 0 degrees, and a roll sinusoid of +/- 15 degrees with a throttle value of 50%. 
