# Software Control Directory Structure

This directory contains all code packages, third-party dependencies, and test documentation related to software control.

## Directory Structure

### Codes_packages/
**Self-built Packages Directory**

This directory contains ROS2 packages that are developed and built by the project team. These packages are customized for project-specific requirements, including:
- Custom launch files
- Project-specific configuration parameters
- Self-developed nodes and functional modules

### Source_packages/
**Third-party Source Packages Directory**

This directory contains third-party ROS2 packages cloned from Git repositories. These packages **cannot be installed via apt** and must be built from source. Main packages include:
- `m-explore-ros2/` - Exploration algorithm related packages
- `mycobot_ws/` - Robotic arm control related packages
- `rplidar_ros/` - RPLIDAR LiDAR driver package

**Note**: These packages need to be obtained via `git clone` and then compiled using `colcon build`.

### documents/
**Test Data and Documentation Directory**

This directory contains various data and documents generated during project testing, including:
- **Frame Graph** - ROS2 frame graph files (.gv format)
- **ROS Graph Screenshots** - Visual screenshots of ROS node relationship graphs
- **Configuration Screenshots** - Screenshots documenting configuration issues
- **Video Files** - Video recordings of test processes
- **Other Test Data** - Other relevant files generated during testing

These documents and data are used to record and verify system operation status, facilitating problem diagnosis and system analysis.
