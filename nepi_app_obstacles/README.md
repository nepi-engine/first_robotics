# nepi_app_obstacles

Obstacle localization app for NEPI Engine.

The app consumes a depth map published by any NEPI depth map source, splits the
returns into ground and obstacles, groups the obstacle returns into discrete
obstacles, and publishes each one with its pixel box, range, azimuth and
elevation.

## Layout

```
nepi_app_obstacles/
├── scripts/obstacles_app_node.py          # app node (ROS node app_obstacles)
├── scripts/obstacles_app_img_pub_node.py  # overlay image publisher, launched by the app node
├── api/obstacles_if.py                    # ObstaclesIF, installs into nepi_api
├── api/connect_obstacles_if.py            # ConnectObstaclesIF, for other nodes
├── sdk/nepi_obstacles.py                  # the algorithm, installs into nepi_sdk
├── msg/                                   # Obstacle, Obstacles, ObstaclesDepthMap, ObstaclesStatus
├── params/obstacles_app_params.yaml       # app registration + RUI registration
└── rui/NepiAppObstacles.js                # RUI page
```

## ROS interface

All names are rooted at the device namespace, `<base>/app_obstacles`.

Published:
- `<base>/app_obstacles/obstacles` — `nepi_app_obstacles/Obstacles`, the obstacle
  list only; it carries no images, so subscribing to it is cheap
- `<base>/app_obstacles/obstacles_depth_map` — `nepi_app_obstacles/ObstaclesDepthMap`,
  the same per-cycle header fields plus the two 32FC1 segmentation range images
  (ground and obstacles), published back to back with the obstacle list. Pair the
  two on `source_topic` + `source_timestamp`. Local namespace only, no `all`
  fan-out, so the images are never broadcast to consumers that did not ask.
- `<base>/app_obstacles/obstacles/status` — `nepi_app_obstacles/ObstaclesStatus`, latched
- `<base>/all/obstacles` — the same `Obstacles` message, collective fan-out
- `<image namespace>/obstacles_image` — overlay image, one per active source
- `<base>/app_obstacles/controls/status` — `nepi_interfaces/ControlsStatus`

Subscribed (each also available under `<base>/all/obstacles/…`):
- `enable`, `set_auto_select_enable`
- `set_source_topic`, `set_source_topics`, `add_source_topic(s)`, `remove_source_topic(s)`
- `set_max_process_rate`, `set_max_image_pub_rate`, `set_image_pub`, `set_use_last_image`
- `set_full_screen`, `set_show_sources`, `set_show_ground`, `set_show_obstacles`

Sources are discovered by their `nepi_interfaces/DepthMapStatus` publisher, so
any NEPI depth map is a valid input with no configuration.

## Deploy

```bash
./deploy_app.sh
```

Requires `NEPI_REMOTE_SETUP` to be set (`0` local, `1` remote).
