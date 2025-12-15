# RC Truck ROS 2 Workspace

This repository hosts the ROS 2 workspace for an autonomous RC truck built on a Raspberry Pi. The workspace already contains scaffolding for hardware interfaces, perception, navigation, bringup launch files, and a standalone PWM smoke test.

## Repository Layout

```
.
├── pwmtest.py                # Standalone GPIO PWM smoke test for the ESC
├── src/
│   ├── rc_truck_bringup/     # Launch and configuration entry points
│   ├── rc_truck_control/     # Supervisory control placeholders
│   ├── rc_truck_hw/          # GPIO and sensor interface stubs
│   ├── rc_truck_interfaces/  # Custom messages for the stack
│   ├── rc_truck_navigation/  # Behavior management and planning
│   └── rc_truck_perception/  # Camera and range processing pipelines
└── .gitignore
```

Each Python package is wired with sample nodes and launch files that show expected topics and parameters. The `rc_truck_interfaces` package defines baseline messages for throttle, steering, vehicle state, and behavior hints.

## Getting Started

1. **Install a supported ROS 2 distribution.** Debian 13 (trixie) currently requires building ROS 2 (e.g., Jazzy) from source or switching to a supported OS (Ubuntu 24.04 for Jazzy binaries, Debian 12, etc.).
2. **Add toolchain dependencies:** `colcon`, `rosdep`, `vcstool`, and any ROS 2 build essentials.
3. **Install hardware Python libraries:** e.g., `rpi-lgpio` or a fork of `RPi.GPIO` for 64-bit Raspberry Pi OS.
4. **Bootstrap the workspace:**
   ```bash
   cd /home/raspberrypi/Documents/rc_truck
   rosdep install --from-paths src --ignore-src --rosdistro <ROS_DISTRO> -y
   colcon build
   source install/setup.bash
   ```
5. **Launch the full stack (after customizing nodes):**
   ```bash
   ros2 launch rc_truck_bringup bringup.launch.py
   ```
6. **Launch perception/navigation only (simulation stubs):**
   ```bash
   ros2 launch rc_truck_bringup simulation.launch.py
   ```

## Next Steps

- Replace placeholder maintainer info in every `package.xml` and `setup.py`.
- Swap the stub logic in each node for real hardware drivers, perception, and autonomy behaviors.
- Extend `rc_truck_interfaces` with any additional messages/services/actions needed by your stack.
- Add automated tests (`colcon test`, hardware smoke tests) and CI once ROS 2 builds locally.
- Integrate simulation assets or Gazebo to validate autonomy before driving the real truck.
