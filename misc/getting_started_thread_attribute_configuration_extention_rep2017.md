# Getting started to use the REP-2017's infrastructure.

This memo explains how to use the REP-2017's infrastructure proposed in the pull-request below.  
https://github.com/ros-infrastructure/rep/pull/385

Throughout the procedures below, you can prepare the environment where the ROS 2 and benchmark application use the thread attribute configuration feature.

## [Host PC] Create an SD card for Raspberry Pi 4

````bash
$ wget https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/download/22.04.1_v5.15.39-rt42-raspi_ros2_humble/ubuntu-22.04.1-rt-ros2-arm64+raspi.img.xz
$ sudo rpi-imager --cli ubuntu-22.04.1-rt-ros2-arm64+raspi.img.xz [SD card's device node, e.g. /dev/sdc]
````
After booting with the SD card above, you have to configure the Raspberry Pi 4 to connect to the internet.

## Prepare the build/execution environment for ROS 2 and sample benchmark.

````bash
$ sudo apt purge -y needrestart
$ sudo apt update && sudo apt install -y locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
$ sudo apt update && sudo apt install -y software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install -y curl
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
$ sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures
$ sudo apt update && sudo apt install -y libacl1-dev
## needed to execute reference system's benchmark below
## https://github.com/ros-realtime/reference-system
$ sudo pip install bokeh==2.4.3
$ sudo pip install pandas
$ sudo pip install psrecord
## Disable preinstalled ROS 2 humble.
$ sudo rm /etc/profile.d/99-source-ros.sh
$ sudo reboot
````

## Build ROS 2 Rolling, adopting the patches related to the REP-2017.

````bash
$ mkdir -p ros2_rolling/src
$ cd ros2_rolling
$ vcs import -w 1 src < ~/rolling-with-thread-attribute-configuration-extention-rep2017.repos
# Copy the repos file above from the repository below.
# https://github.com/esol-community/reference-system/tree/rep2017_demo/misc
$ touch src/gazebo-release/COLCON_IGNORE
$ touch src/ros2/rviz/COLCON_IGNORE
$ sudo rosdep init
$ rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
$ colcon build --symlink-install --executor sequential
# It takes at least 10 hours to self-build on the Raspberry Pi 4.
````

## Build reference-system benchmark modified to use the REP-2017 infrastructure.

````bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone -b rep2017_demo https://github.com/esol-community/reference-system.git
$ cd ~/ros2_ws
$ source  ~/ros2_rolling/install/setup.bash
$ colcon build --symlink-install --executor sequential
````

## Execute the benchmark(prioritized).

To get the result without thread attribute settings, execute the benchmark
without the dedicated environment variable (ROS_THREAD_ATTRS_FILE).
````bash
$ export ROS_THREAD_ATTRS_FILE=
$ python3 $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py 120 autoware_default_prioritized_using_rep2017
````
To get the result with the thread attribute settings, execute the benchmark after setting the environment variable to point to the YAML configuration file.
````bash
$ export ROS_THREAD_ATTRS_FILE=~/ros2_ws/install/autoware_reference_system/share/autoware_reference_system/cfg/thread_attr_for_prioritized.yaml
$ python3 $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py 120 autoware_default_prioritized_using_rep2017
````
You can check if the thread attribute settings are set expectedly with the command below.
````
$ ps -eLo pid,tid,class,rtprio,ni,pri,psr,comm | grep autoware
   1137    1137 TS       -   0  19   2 autoware_defaul
   1137    1140 TS       -   0  19   0 autoware_de-ust
   1137    1141 TS       -   0  19   2 autoware_de-ust
   1137    1142 TS       -   0  19   3 autoware_defaul
   1137    1143 TS       -   0  19   0 autoware_defaul
   1137    1151 TS       -   0  19   3 autoware_defaul
   1137    1152 TS       -   0  19   0 autoware_defaul
   1137    1153 TS       -   0  19   3 autoware_defaul
   1137    1154 TS       -   0  19   3 autoware_defaul
   1137    1155 TS       -   0  19   0 autoware_defaul
   1137    1156 TS       -   0  19   2 autoware_defaul
   1137    1157 RR       1   -  41   0 autoware_defaul
   1137    1158 RR       1   -  41   1 autoware_defaul
   1137    1159 RR       1   -  41   2 autoware_defaul
   1137    1160 RR      30   -  70   2 autoware_defaul
````

Then, You can find scheduling policies and priorities in the third and fourth columns (e.g., "RR," 30).

## Execute the benchmark(singlethreaded/multithreaded).

To get the result without thread attribute settings, execute the benchmark
without the dedicated environment variable (ROS_THREAD_ATTRS_FILE).
````bash
$ export ROS_THREAD_ATTRS_FILE=
$ python3 $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py 120 autoware_default_singlethreaded
or
$ python3 $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py 120 autoware_default_multithreaded

````
To get the result with the thread attribute settings, execute the benchmark after setting the environment variable to point to the YAML configuration file.
````bash
$ export ROS_THREAD_ATTRS_FILE=~/ros2_ws/install/autoware_reference_system/share/autoware_reference_system/cfg/thread_attr_for_existing_executors.yaml
$ python3 $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py 120 autoware_default_singlethreaded
or
$ python3 $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py 120 autoware_default_multithreaded
````

