# Follow The Gap - Reactive Navigation

Simple reactive obstacle avoidance for F1TENTH using LiDAR-only navigation. Implements the classic "Follow the Gap" algorithm for autonomous driving without localization or mapping.

Part of the F1TENTH autonomous racing system - provides basic autonomous navigation using only laser scan data.

## Quick Start

```bash
# Build (from workspace root)
colcon build --packages-select gap_follow --symlink-install
source install/setup.bash

# For Real Car
ros2 run gap_follow reactive_node

# For Simulation (launch gym first)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run gap_follow reactive_node
```

## Algorithm Overview

The **Follow the Gap** algorithm provides reactive obstacle avoidance:

1. **LiDAR Preprocessing**: Filter and clean laser scan data
2. **Closest Point Detection**: Find nearest obstacle
3. **Safety Bubble**: Create danger zone around closest point
4. **Gap Finding**: Identify largest free space area
5. **Target Selection**: Choose best driving direction in gap
6. **Drive Command**: Steer toward target point

### Key Features
- **No Localization Required**: Uses only LiDAR data
- **Real-time Reactive**: Fast obstacle avoidance
- **Simple Implementation**: ~120 lines of code
- **Robust Navigation**: Handles dynamic obstacles

## Algorithm Steps

```
LiDAR Scan → Preprocessing → Find Closest Point → Safety Bubble
     ↓                                                    ↓
Drive Command ← Target Selection ← Find Max Gap ← Free Space
```

### 1. Preprocessing
- Smooth noisy LiDAR readings
- Reject invalid/extreme values (>3m)
- Apply median filtering

### 2. Safety Bubble
- Create circular safety zone around closest obstacle
- Set bubble points to zero (blocked space)
- Prevents collision during turning

### 3. Gap Detection
- Find consecutive non-zero ranges (free space)
- Identify largest continuous gap
- Consider gap width and distance

### 4. Target Selection
- **Naive**: Furthest point in largest gap
- **Better**: Weighted selection considering gap width and safety

## Usage Modes

### Standalone Navigation
```bash
# Simple obstacle avoidance - no other systems needed
ros2 run gap_follow reactive_node
```

### Integration with Full System
```bash
# Can run alongside localization for comparison
ros2 launch particle_filter_cpp localize_sim_launch.py
ros2 run gap_follow reactive_node  # Alternative to full planning stack
```

## Key Topics

**Subscribes:**
- `/scan` - LiDAR laser scan data

**Publishes:**
- `/drive` - Ackermann drive commands

## Configuration

Edit the reactive node parameters:
- **Bubble radius**: Safety distance around obstacles
- **Speed limits**: Maximum forward/turning speeds  
- **Gap thresholds**: Minimum gap size to consider
- **Smoothing window**: LiDAR preprocessing filter size

## Advantages

- **No mapping**: Works in unknown environments
- **Low computational cost**: Real-time performance
- **Robust**: Handles sensor noise and dynamic obstacles
- **Simple**: Easy to understand and modify

## Limitations

- **Local minima**: Can get stuck in dead-ends
- **No global planning**: Takes longer, suboptimal paths
- **Reactive only**: No prediction or future planning
- **Speed limited**: Must drive conservatively for safety

## Testing Maps

Included test environments:
- **levine_blocked.png**: Simple environment for basic testing
- **levine_obs.png**: Complex obstacles for advanced testing

Change map in `f1tenth_gym_ros/config/sim.yaml`:
```yaml
map_path: 'levine_obs'  # or 'levine_blocked'
```

## Comparison with Full System

| Feature | Follow the Gap | Full Planning System |
|---------|---------------|---------------------|
| **Sensors** | LiDAR only | LiDAR + Localization |
| **Speed** | Fast reaction | Optimal planning |
| **Complexity** | Simple (~120 lines) | Complex (1000+ lines) |
| **Performance** | Good obstacle avoidance | Optimal racing lines |
| **Use Case** | Quick testing, simple navigation | Competition racing |

## Integration with Workspace

This package provides an alternative to the full planning stack:

```
Option 1: Full System
Hardware → Localization → Planning → Control → Drive

Option 2: Reactive Only  
Hardware → Follow the Gap → Drive
```

Perfect for testing, learning, and scenarios where simple navigation is sufficient.

Built for educational use and rapid prototyping of F1TENTH navigation systems.
