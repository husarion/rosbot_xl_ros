# Developer guide

Description of useful tools and good practices to maintain code readability and reliability.

## Tools

### pre-commit

[pre-commit configuration](.pre-commit-config.yaml) prepares plenty of tests helping for developing and contributing. Usage:

```bash
# install pre-commit
pip install pre-commit

# initialize pre-commit workspace
pre-commit install

# manually run pre-commit hooks
pre-commit run -a

# update revision
pre-commit autoupdate
```

After initialization [pre-commit configuration](.pre-commit-config.yaml) will applied on every commit.

### Run tests

```bash
# Run tests
colcon test

# Show results
colcon test-result --verbose
```

> [!NOTE]
> Command `colcon test` does not build the code. Remember to build your code after changes.

### Industrial CI

Download industrial package from [github](https://github.com/ros-industrial/industrial_ci/) build it and run following command to run industrial_ci tests locally.

```bash
ros2 run industrial_ci rerun_ci src/ ROS_DISTRO=humble
```
