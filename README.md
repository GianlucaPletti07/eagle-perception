# LiDAR odometry — E-Agle TRT perception task (path B)

A ROS 2 Humble workspace that estimates how the car moves over one lap by aligning each LiDAR
scan against a local map of recent scans. It publishes the pose on `/odometry` and as the
`odom → base_link` transform, draws the trajectory in RViz and builds a point cloud map of the
whole track.

How it works, the problems met along the way and the measured results are in
[REPORT.md](REPORT.md). This file only explains how to run it.

## Repository layout

```
.
├── compose.yaml               # how the container is run
├── docker/Dockerfile          # environment provided with the task, unmodified
├── scripts/evaluate_run.py    # timing statistics and lap closure from a recorded run
├── src/
│   ├── lidar_odometry_msgs/   # interface package: ProcessingTime message
│   └── lidar_odometry/        # the odometry node, its parameters, launch file and RViz setup
├── docs/images/               # figures used in the report
├── REPORT.md                  # approach, problems, results, improvements
├── AI_USAGE.md                # how AI tools were used
└── bags/                      # put the recording here (ignored by git)
```

## 1. Requirements

One of the following:

- **Linux** with an X11 desktop, Docker Engine and the Docker Compose plugin.
- **Windows 10 (21H2, build 19044 or later) or Windows 11** with WSL 2, an Ubuntu distribution and Docker Desktop with *Settings → Resources → WSL integration* enabled for Ubuntu. RViz is a Linux GUI application. On Windows its window is shown by WSLg, the part of WSL that displays Linux windows on the Windows desktop. Make sure it is available: run `wsl --update` once in PowerShell, then in the Ubuntu terminal check that echo `$DISPLAY prints :0`.
**Run every command below in the Ubuntu terminal, not in PowerShell**, and keep the repository
inside the Linux file system (e.g. `~/`), not under `/mnt/c`: builds there are much slower.

About 5 GB of free disk space are needed for the image.

## 2. Get the code and the recording

```bash
cd ~
git clone https://github.com/GianlucaPletti07/eagle-perception.git
cd eagle-perception
```

Download the bag (zip, 185 MB) from the link in the task description and unzip it into `bags/`,
so that this file exists:

```
bags/perception_bag/metadata.yaml
```

On Windows the downloaded zip is reachable from Ubuntu under `/mnt/c/Users/<your user>/Downloads/`:

```bash
sudo apt install -y unzip   
unzip /mnt/c/Users/<your user>/Downloads/<bag zip> -d bags/
```

## 3. Build and start the container

```bash
docker compose build        # about 4 GB, takes a while the first time
```

On **Linux** only, allow the container to open windows on your display (once per login):

```bash
xhost +local:docker
```

Then start the container and open a shell in it:

```bash
docker compose up -d
docker compose exec ros bash
```

The prompt becomes `eagle@...:~/ws$`. The repository is mounted at `/home/eagle/ws`, so files
edited on the host are seen by the container immediately. Every further terminal is opened the
same way: `cd ~/eagle-perception && docker compose exec ros bash`.

The image creates the user `eagle` with UID 1000. If `id -u` on the host prints a different
number, files created by the container will belong to UID 1000.

## 4. Build the workspace

Inside the container, from a fresh shell:

```bash
colcon build
source install/setup.bash
```

The build must finish with `Summary: 2 packages finished` and no warnings.
`source install/setup.bash` tells the shell where the freshly built packages are: run it in
every new terminal before using them.


## 5. Run

Two terminals inside the container, each after `source install/setup.bash`.

**Terminal 1** — the node and RViz:

```bash
ros2 launch lidar_odometry lidar_odometry.launch.py
```

**Terminal 2** — the recording, started after the node:

```bash
ros2 bag play bags/perception_bag
```

RViz opens already configured in the `odom` frame. While the bag plays you should see the current
scan, the trajectory (blue line), the vehicle (orange arrow) and the track map growing every
couple of metres.

Things to know:

- **Start the node before the bag.** The node needs `/tf_static`, which the bag publishes once
  at the start, to know where the sensor is mounted.
- **Restart the node every time the bag is played again** (Ctrl+C in terminal 1, then launch
  again). The node keeps its pose and map, so a second playback would continue from where the
  first one ended. For the same reason, do not use `--loop`.
- To look at the scene calmly, play at half speed with `--rate 0.5`, or pause with the space bar
  in terminal 2.

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `rviz` | `true` | open RViz; use `rviz:=false` when measuring timing |
| `params_file` | installed `config/lidar_odometry.yaml` | YAML with the node parameters |

## 6. Outputs

| Topic | Type | Content |
|---|---|---|
| `/odometry` | `nav_msgs/Odometry` | pose of `base_link` in `odom`, one message per scan |
| `/tf` | `odom → base_link` | the same pose as a transform |
| `/lidar_odometry/markers` | `visualization_msgs/MarkerArray` | trajectory line and vehicle arrow |
| `/lidar_odometry/track_map` | `sensor_msgs/PointCloud2` | map of the whole track (transient local) |
| `/lidar_odometry/processing_time` | `lidar_odometry_msgs/ProcessingTime` | time spent on each scan, per stage |
| `/lidar_odometry/filtered_points` | `sensor_msgs/PointCloud2` | preprocessed scan, only if `publish_filtered_cloud` is true |


## 7. Parameters

Every parameter is in
[`src/lidar_odometry/config/lidar_odometry.yaml`](src/lidar_odometry/config/lidar_odometry.yaml),
with a comment each. After editing that file, run `colcon build` again, or point the launch file
at it directly without rebuilding:

```bash
ros2 launch lidar_odometry lidar_odometry.launch.py \
  params_file:=/home/eagle/ws/src/lidar_odometry/config/lidar_odometry.yaml
```

Parameters fall into two groups:

- **Startup** (topic, frames, queue depths, track map switch): read-only, ROS rejects changes.
- **Tunable** (preprocessing, registration, local map, visualization): can be changed while the
  node runs and apply from the next scan. Each has a description and a valid range enforced by
  ROS.

```bash
ros2 param set /lidar_odometry visualization.trajectory_width 0.5
ros2 param describe /lidar_odometry local_map.keyframe_distance
ros2 run rqt_reconfigure rqt_reconfigure      # sliders for every tunable parameter
```

## 8. Measure timing and lap closure

Four terminals inside the container, after `source install/setup.bash`, in this order:

```bash
# 1: the node alone, so RViz does not compete for the CPU
ros2 launch lidar_odometry lidar_odometry.launch.py rviz:=false
# 2
mkdir -p bags/runs && ros2 topic echo /odometry --csv > bags/runs/odometry.csv
# 3
ros2 topic echo /lidar_odometry/processing_time --csv > bags/runs/timing.csv
# 4: last
ros2 bag play bags/perception_bag
```

When the bag has finished, stop terminals 1–3 with Ctrl+C and run:

```bash
python3 scripts/evaluate_run.py bags/runs/odometry.csv bags/runs/timing.csv
```

It prints mean, median, 95th percentile and maximum time per stage, how many scans exceeded the
100 ms scan period, and how far the end of the lap lies from the first pass over the same
stretch of track. Add `--plot bags/runs/run.png` to also save the trajectory and timing as an
image. Both CSV files should have 344 lines, one per scan.

## 9. Stop

```bash
exit                    # leave the container shell
docker compose down     # stop the container
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `TF_OLD_DATA ignoring data from the past` in the node output, or a trajectory that does not start at the origin | the bag was played again without restarting the node: restart it |
| `Package 'lidar_odometry' not found` or `message type ... is invalid` | run `source install/setup.bash` in that terminal |
| No RViz window on Linux | run `xhost +local:docker` on the host, then launch again |
| No RViz window on Windows | run `wsl --update` and `wsl --shutdown` in PowerShell, reopen Ubuntu and check `echo $DISPLAY` prints `:0` |

## AI usage

How I used AI tools during the project is described in [AI_USAGE.md](AI_USAGE.md).
