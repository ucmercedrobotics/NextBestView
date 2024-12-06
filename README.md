# Kinova NextBestView
[![github](https://img.shields.io/badge/GitHub-ucmercedrobotics-181717.svg?style=flat&logo=github)](https://github.com/ucmercedrobotics)
[![website](https://img.shields.io/badge/Website-UCMRobotics-5087B2.svg?style=flat&logo=telegram)](https://robotics.ucmerced.edu/)
[![python](https://img.shields.io/badge/Python-3.10.12-3776AB.svg?style=flat&logo=python&logoColor=white)](https://www.python.org)
[![pre-commits](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
<!-- [![Checked with mypy](http://www.mypy-lang.org/static/mypy_badge.svg)](http://mypy-lang.org/) -->
<!-- TODO: work to enable pydocstyle -->
<!-- [![pydocstyle](https://img.shields.io/badge/pydocstyle-enabled-AD4CD3)](http://www.pydocstyle.org/en/stable/) -->

<!-- [![arXiv](https://img.shields.io/badge/arXiv-2409.04653-b31b1b.svg)](https://arxiv.org/abs/2409.04653) -->

## How to Start
Make sure you initialize the repo with the repo pre-commits:
```bash
$ make repo-init
```

After, build your container:
```bash
$ make build-image
```

### Simulation
If you're running in simulation, you'll be forwarding graphic display to noVNC using your browser.
First start the Docker network that will manage these packets:
To start, make your local Docker network to connect your VNC client, local machine, and Kinova together. You'll use this later when remote controlling the Kinova.
```bash
$ make network
```

Next, standup the VNC container to forward X11 to your web browser. You can see this at `localhost:8080`.
```bash
$ make vnc
```

Then run the sim container:
```bash
$ make sim
```

Finally, from within the container, build your ROS2 project run the sim command:
```bash
$ make nbv
$ source install/setup.bash
$ make sim-run
```
NOTE: currently this runs without the bracelet. To add this, more config must be done to the launch file to find the right model.

### Target
For some reason, I can't figure out how to connect the host network into a Docker network to have connection from Kinova working with X11 forwarding all in one.
So to get around it, we have one run command for connecting directly (this will run without Rviz) to the target as if you're running only with ROS2 and the one above running only in simulation with Rviz.
Until we get around it, this is how it will be.

Make sure you're on the same subnet as the Kinova and plugged in via ethernet. Then run the NIC setup command:
```bash
$ make config-target-network
```

To start the Docker environment for target only:
```bash
$ make target
```

Finally, to launch the ROS2 drivers for Kortex control:
```bash
$ make nbv
$ source install/setup.bash
$ make target-run
```
