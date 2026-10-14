# Package containers

Each ROS package owns its Dockerfile, dependency lists, and Compose services
under `docker/`. The repository root `compose.yaml` includes those definitions.
The project `curtmini_piper_containers` repository includes this root file and
adds CycloneDDS/Zenoh settings, GUI configuration, and the development workspace.
Use Docker Compose 2.24.4 or newer for the override files.

| Owner | Services / image targets |
| --- | --- |
| `docker/common` | Shared `piper-ros-base` image and infrastructure entrypoint |
| `curtmini_piper_description` | `piper-description`: installed descriptions and upstream meshes |
| `curtmini_piper_moveit_config` | `piper-moveit-config`, `moveit-rviz-sim`, `moveit-rviz-hardware` |
| `curtmini_piper_bringup` | `piper-bringup`, `base-bringup`, `keyboard-teleop-sim`, `keyboard-teleop-hardware`, `ros-cli` |
| `curtmini_piper_navigation` | `mapping-sim`, `nav2-sim`, `nav2-rviz-sim` |
| `curtmini_piper_sensors` | `realsense-base`, `realsense-tool` |

The shared ROS, description, and MoveIt configuration services have `scale: 0`.
They remain visible to Compose's build dependency resolver, but normal launches
create no containers for them. `additional_contexts` links the builds explicitly:

```text
ROS base -> description -> MoveIt configuration -> arm hardware
                       -> MoveIt RViz
         -> navigation (copies installed description assets)
         -> sensors
         -> base hardware / teleop
```

Runtime images contain installed artifacts rather than build tools and source
trees. The MoveIt RViz image tag remains compatible with the MoveItPy and C++
client images in `curtmini_piper_tools`. Build that image before those clients;
the infrastructure `scripts/build-all.sh` already orders them this way.

Shared source manifests remain in `docker/dependencies/{arm,base}.repos`.
The pinned Python SDK belongs to
`curtmini_piper_bringup/docker/dependencies/jazzy-hardware-python.env`.
The `base-bringup` definition stays with bringup while its ROS packages remain
owned by the upstream Curt Mini repository.

## Build sources

Regular robot builds default to `https://github.com/ipa-may/curtmini_piper.git#main`.
Build contexts cover the whole repository so a package can access shared
manifests. The Dockerfile path selects the owning package. `CURTMINI_PIPER_SOURCE`
in the infrastructure selects the local Compose checkout; it does not change
the image source.

To test local changes before publishing, run from this repository:

```bash
export CURTMINI_PIPER_IMAGE_SOURCE="$PWD"
docker compose -f compose.yaml build piper-bringup moveit-rviz-sim
```

Use an absolute path for the image-source override because included Compose
files resolve relative paths from their own directories. The common entrypoint
is read from the sibling project infrastructure. For another checkout layout,
set `CONTAINER_INFRA_SOURCE` to that repository's absolute path.

The same override works when building from `curtmini_piper_containers`. It
changes all robot image layers, including hardware, but does not change the
simulation or tools image sources. The infrastructure workspace overlay is a
separate workflow that compiles mounted simulation sources into named volumes.

## Checks

From this repository, build the test targets against the local source:

```bash
export CURTMINI_PIPER_IMAGE_SOURCE="$PWD"
docker compose -f compose.yaml -f compose.test.yaml --profile image-test build \
  piper-description-test piper-moveit-test piper-hardware-test piper-teleop-test
```

These checks run during image construction and do not start hardware services.
Test image tags are distinct from runtime tags. From the infrastructure repo:

```bash
python3 scripts/test-robot-compose.py
python3 scripts/test-compose-settings.py
bash scripts/validate-locks.sh
```

The Compose tests cover package ownership, build dependencies, zero-replica
helpers, source overrides, config mounts, and middleware/GUI/workspace overlays.

## Migration

The project infrastructure now includes `curtmini_piper/compose.yaml` and
`curtmini_piper/compose.gui.yaml`. Existing public service names, runtime image
tags, profiles, commands, devices, and mounts are preserved. GUI overrides live
with the MoveIt and navigation packages and are included by the root GUI file.

Deploy the robot and infrastructure changes together. Until the package
Dockerfiles are published upstream, select the local image-source override.
The old root `docker/Dockerfile` and `docker/compose*.yaml` are retired.
