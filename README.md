# ros2_cartographer_mapping
<!-- Required -->
<!-- Package description -->
ros2_cartographer_mapping launch files.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to ros2_cartographer_mapping
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
Run launch file:
* Mapping
```bash
ros2 launch ros2_cartographer_mapping cartographer_launch.launch.py mapping:=True
```

 * Localization
```bash
ros2 launch ros2_cartographer_mapping cartographer_launch.launch.py mapping:=False map_path:=/path/to/map.pbstream
```

