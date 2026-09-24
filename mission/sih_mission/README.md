# SIH Mission Package (`sih_mission`)

Manages the high-level mission lifecycle (pre-flight checks, autonomous survey, survivor confirmation, payload drops, and return-to-launch safety triggers).

## Launch Commands

### 1. Launch Mission Manager
```bash
ros2 launch sih_mission mission_manager.launch.py
```

### 2. Launch Payload Delivery Dispenser
```bash
ros2 launch sih_mission payload_delivery.launch.py
```

### 3. Standalone Demonstrations
```bash
python -m mission.sih_mission.mission_manager_node
python -m mission.sih_mission.payload_delivery_node
```
