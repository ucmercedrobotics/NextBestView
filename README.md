# Kinova NextBestView
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)
![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)

[![github](https://img.shields.io/badge/GitHub-ucmercedrobotics-181717.svg?style=flat&logo=github)](https://github.com/ucmercedrobotics)
[![website](https://img.shields.io/badge/Website-UCMRobotics-5087B2.svg?style=flat&logo=telegram)](https://robotics.ucmerced.edu/)
[![python](https://img.shields.io/badge/Python-3.10.12-3776AB.svg?style=flat&logo=python&logoColor=white)](https://www.python.org)
[![pre-commits](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)

<!-- [![Checked with mypy](http://www.mypy-lang.org/static/mypy_badge.svg)](http://mypy-lang.org/) -->
<!-- TODO: work to enable pydocstyle -->
<!-- [![pydocstyle](https://img.shields.io/badge/pydocstyle-enabled-AD4CD3)](http://www.pydocstyle.org/en/stable/) -->

<!-- [![arXiv](https://img.shields.io/badge/arXiv-2409.04653-b31b1b.svg)](https://arxiv.org/abs/2409.04653) -->

## How to Start
Build your container:
```bash
make build-image
```
After, make sure you initialize the repo with the repo pre-commits:
```bash
make repo-init
```
You'll be forwarding graphic display to noVNC using your browser.
First start the Docker network that will manage these packets:
To start, make your local Docker network to connect your VNC client, local machine, and Kinova together. You'll use this later when remote controlling the Kinova.
```bash
make network
```

Next, standup the VNC container to forward X11 to your web browser. You can see this at `localhost:8080`.
```bash
make vnc
```

### Simulation
To start the Docker environment:
```bash
make bash
```

Finally, to launch the ROS2 simulated drivers for MoveIt Kortex control:
```bash
make moveit
```

### Target
Make sure you're on the same subnet as the Kinova and plugged in via ethernet. Then run the NIC setup command:
```bash
make config-target-network
```
By default, the name of the NIC is `en7`. Change this to whatever your NIC is that connects to the Kinova with the `Make` argument `KINOVA_NIC`.
Example:
```bash
make config-target-network KINOVA_NIC=<your_nic_name>
```

To start the Docker environment:
```bash
make bash
```

If you want to run planning system with the provided actions (identify object, go to position and next best view) run the following command.
This will launch Rviz2, Moveit2, Vision, action nodes and mission interface. If you intent to use this we are providing example xml file created with Chat GPT.
After running this command go to section Using with Mission Planning (MP) to understand how to send the generated example plan.
```bash
make one4all
```

Finally, to launch the ROS2 drivers for MoveIt Kortex control:
```bash
make moveit-target
```

### Example
If you want to see the robot move in sim or on target, you can launch the custom example node we prebuilt.
This will just move the arm to an arbitrary position:
```bash
make moveit-example
```

### Vision
If you intend on using the vision module, run an additional vision module by opening up another shell in Docker using `docker exec`.
From there, run the following command:
```bash
make vision
```
This will bring up the vision ROS2 node that exposes the RGBD camera on ROS2 topics and can be visualized in RViz.

NOTE: this works only with hardware connected.

## Using with Mission Planning (MP)
Currently, control with an XML generated mission plan is under implementation.
XML mission plans are sent via any compliant MP generation tool such as our own [GPT planner](https://github.com/ucmercedrobotics/gpt-mission-planner).
Connect the planner to TCP port `12345` after initializing all relevant nodes.
```bash
nc localhost 12345
```
Paste given example plan and Ctrl+c

### Expected behavior:

    1- Object (pot plant) will be detected.
    2- Arm will center the object according to camera frame
    3- Arm will get point cloud information of the plant from different angles and these point clouds will be merged.
