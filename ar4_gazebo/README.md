# Description
A Gazebo simulation of the AR4 Package.

To build the package, run
`colcon build`

To run the simulation, source and run
`ros2 launch ar4_gazebo ar4_sim.launch.py`

#### Launch file arguments
- 'jsp_gui':
    - Run [`joint state publisher gui`](https://github.com/ros/joint_state_publisher/tree/ros2) node. (default: 'false')
- 'rsp':
    - Run [`robot state publisher`](https://github.com/ros/robot_state_publisher) node. (default: 'false')
- 'rviz':
    - Start RViz. (default: 'false')

---
# Using pre-commit

Pre-commit is a tool that allows git's pre-commit hook integrate with various code linters and formatters.

To install `pre-commit`, run
```sh
pip install pre-commit
```

To automatically run it on each commit, from repository's root:
```sh
pre-commit install
```

And that's it! Every time you commit, `pre-commit` will trigger and let you know if everything goes well.
If the checks fail, the commit won't be created, and you'll have to fix the issue (some of them are automatically fixed by `pre-commit`), STAGE the changes, and try again.

To manually run `pre-commit` on the staged changes, one can run:
```sh
pre-commit run
```

Or to change the whole codebase
```sh
pre-commit run --all-files
```

**Note**: `pre-commit` only runs on staged changes by default.
**Note2**: To bypass `pre-commit`, use `git commit --no-verify`.
