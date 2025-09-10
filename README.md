
This ROS2 package provides a node, `draw_n`, that draws the 'N' from the NCSU logo in the `turtlesim` simulator.

## Build & Run

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select p1d2_sai_marthala
````

### 2. Run the Simulation

You will need **two terminals**.

**In Terminal 1**, start the turtlesim simulator:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
```

**In Terminal 2**, run the drawing node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run p1d2_sai_marthala draw_n
```

---

## Outcome

The turtle will trace the 'N' shape in the simulator window.
When finished, a `matplotlib` plot will appear, overlaying the turtle's path on the NCSU logo.

