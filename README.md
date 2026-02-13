# SLVROV ROS 2 Stack

## Packages
- `slvrov_nodes_python` – ROS 2 nodes
- `slvrov_interfaces` – Messages and interfaces
- `slvrov_tools` – Shared tools (vendored PyPI package)

## Build
```bash
mkdir -p ~/slvrov_ws/src
cd ~/slvrov_ws/src
git clone --recurse-submodules https://github.com/LegionaryOfLogic/slvrov_ros.git
cd ..
colcon build --symlink-install
source install/setup.bash
