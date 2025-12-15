# RC Truck ROS 2 Workspace

This repository contains the ROS 2 workspace for an RC truck built on a Raspberry Pi. It will eventually host nodes for throttle (ESC) control, steering servo actuation, and sensor acquisition.

## Repository Layout

```
.
├── pwmtest.py                # Standalone GPIO PWM smoke test for the ESC
├── src/
│   └── rc_truck_control/
│       ├── launch/
│       ├── rc_truck_control/
│       ├── resource/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── .gitignore
```

## Getting Started

1. **Choose and install a supported ROS 2 distribution.** Debian 13 (trixie) currently requires building ROS 2 (e.g., Jazzy) from source. Alternatively, reimage the Pi with a supported OS such as Ubuntu 24.04 (ROS 2 Jazzy binaries) or Debian 12.
2. **Set up the build tools:** `colcon`, `rosdep`, `vcstool`, and the required Python dependencies.
3. **Install hardware libraries:** for example `rpi-lgpio` (Python replacement for `RPi.GPIO` on 64-bit Raspberry Pi OS).
4. **Bootstrap the workspace:**
   ```bash
   cd /home/raspberrypi/Documents/rc_truck
   rosdep install --from-paths src --ignore-src --rosdistro <ROS_DISTRO> -y
   colcon build
   source install/setup.bash
   ```
5. **Run the launch file (after implementing the nodes):**
   ```bash
   ros2 launch rc_truck_control bringup.launch.py
   ```

## Next Steps

- Replace the placeholder maintainer information in `package.xml` and `setup.py`.
- Implement the ROS 2 nodes (`throttle_node.py`, `servo_node.py`, `sensor_node.py`) once the ROS environment and hardware interfaces are finalized.
- Add parameter files, message definitions, and tests as the control stack evolves.
- Mirror this repository to GitHub and set up CI (e.g., GitHub Actions) after the initial commit.
# rc_car_raspberry
