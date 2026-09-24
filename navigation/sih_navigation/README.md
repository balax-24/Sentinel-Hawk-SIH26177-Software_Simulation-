# SIH Navigation Package (`sih_navigation`)

Provides path planning and reactive local obstacle avoidance for the SIH26177 UAV:
1. **Lawnmower Search Planner (`planner_node.py`)**: Computes optimal coverage grids over the disaster site perimeter.
2. **Artificial Potential Field Avoidance (`obstacle_avoidance_node.py`)**: Real-time reactive steering around detected rubble and walls.

## Launch Commands

### 1. Launch Path Planner
```bash
ros2 launch sih_navigation planner.launch.py
```

### 2. Launch Obstacle Avoidance
```bash
ros2 launch sih_navigation obstacle_avoidance.launch.py
```

### 3. Standalone Demonstrations
```bash
python -m navigation.sih_navigation.planner_node
python -m navigation.sih_navigation.obstacle_avoidance_node
```
