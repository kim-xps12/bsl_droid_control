# assets/

This directory contains robot model assets for RL training.

## Files

- `export_urdf.py` - Script to export URDF from ROS 2 xacro files
- `biped_digitigrade.urdf` - Exported URDF (generated)

## Usage

```bash
# From rl_ws directory
uv run python assets/export_urdf.py
```

This will generate `biped_digitigrade.urdf` from the source xacro file in
`ros2_ws/src/biped_description/urdf/biped_digitigrade.urdf.xacro`.

## Notes

- The exported URDF uses primitive shapes (cylinders, boxes) only
- No mesh files are required
- The URDF is compatible with Genesis and MuJoCo

## Genesis Usage

```python
import genesis as gs

gs.init(backend=gs.backend.metal)  # or gs.backend.cpu

scene = gs.Scene()
robot = scene.add_entity(
    gs.morphs.URDF(
        file="assets/biped_digitigrade.urdf",
        pos=(0, 0, 0.6),
    )
)
```
