# iso21423_ros

A ROS 2 connector that makes a robot an ISO 21423 IMR (Industrial Mobile Robot). The robot can then
join any compliant MQTT broker and fleet manager, such as OpenRobOps.

It is one node, `iso21423_connector`, configured with a ROS parameter YAML. A standard Nav2 robot
needs YAML only. Anything else (other topics, vendor actions, extra status states) is a small Python
class named in the YAML. It is built on the ROS-free [`iso21423`](https://github.com/OpenRobOps/iso21423)
Python library (`py-sdk/`).

Supported: ROS 2 **Jazzy** (primary) and **Humble** (see [Humble notes](#humble-notes)).

## Quick start (Nav2 robot)

1. Copy [`config/nav2_default.yaml`](config/nav2_default.yaml) and set at least:
   - `entity_uuid`: this robot's UUID
   - `ccs.id`: the fleet's CCS (coordinate system) UUID, plus `ccs.reference_points` when your map
     frame is not the CCS
   - `broker.url` (and `username` / `password_file` if the broker needs them)
   - `identity.*`: model, serial number, height, footprint
2. Launch:

   ```bash
   ros2 launch iso21423_ros connector.launch.py params_file:=/path/to/robot.yaml
   ```

The robot appears on the broker under `/ISO_21423/v1/IMR/<entity_uuid>/...` with `identity`,
`status`, `odometry`, `batteryStatus`, `globalPlan` and `localTrajectory`. Its identity advertises
`move`, `dock`, `undock`, `pauseImr` and `resumeImr`. `cancelRequest` also works, but it is not in
the advertised list.

## Docker image

```bash
# from this repo, with the iso21423 repo checked out next to it
docker build --build-context iso21423=../iso21423 --target runtime -t iso21423-ros:jazzy -f docker/Dockerfile .
docker run --rm --net host -v $PWD/robot.yaml:/robot.yaml:ro iso21423-ros:jazzy \
  ros2 launch iso21423_ros connector.launch.py params_file:=/robot.yaml
```

Pass `--build-arg ROS_DISTRO=humble` for a Humble image. The image's default command launches the
default config, which stops with `entity_uuid: required` until you pass your own params file.

## Parameter reference

All parameters live under `iso21423_connector: ros__parameters:`. Lists are flat because ROS
parameters can't nest lists.

| Key | Default | Meaning |
|---|---|---|
| `entity_uuid` | (required) | This robot's UUID. |
| `manufacturer_name` | `""` | Published in the identity. |
| `identity.model` | `""` | `imrModel`. |
| `identity.serial_number` | `""` | `imrSerialNumber`. |
| `identity.height` | `0.0` | `imrHeight`, in m. |
| `identity.footprint` | `[]` | Flat `[x1, y1, x2, y2, ...]` in the base frame, in m. Published as `imrFootprint` and `imrWorkingArea`. |
| `identity.footprint_from_nav2` | `false` | Read `footprint` from `/global_costmap/global_costmap` (5 s timeout) and fall back to `identity.footprint` on failure. |
| `broker.url` | `""` | `mqtt://host:1883`, or `mqtts://host:8883` for TLS. |
| `broker.username` | `""` | |
| `broker.password_file` | `""` | A file holding the password. |
| `broker.tls` | `false` | TLS with the system CAs (also implied by `mqtts://`). |
| `credentials_provider` | `""` | `""` uses `broker.*`. `openrobops` fetches them; see [OpenRobOps extras](#openrobops-extras-opt-in). |
| `oro.url` | `""` | OpenRobOps base URL (`openrobops` provider). |
| `oro.api_key_file` | `""` | A file holding the OpenRobOps API key. |
| `ccs.id` | `""` | The CCS UUID used in every published location. |
| `ccs.map_frame` | `map` | The ROS frame the CCS is fitted to. |
| `ccs.reference_points` | `[]` | Flat `[map_x, map_y, ccs_x, ccs_y, ...]`. With fewer than 3 points, map coordinates are CCS coordinates. |
| `base_frame` | `base_link` | Robot frame for odometry (TF `map_frame` -> `base_frame`). |
| `mode` | `MODE_AUTO` | The operating mode, always the first `status` state. |
| `execution_policy` | `preempt` | What a new request does while one runs: `abort_new`, `queue_after`, `queue_replace`, `parallel`, `priority`, `preempt`. |
| `sources.<name>.*` | | Telemetry sources; see below. |
| `handlers.<name>.*` | | Request handlers; see below. |
| `extensions.<name>.*` | | Extension resources (sources publishing non-standard resources). |

### Sources, handlers, extensions

An entry is enabled by any key under it (`enabled: true` is the usual way) and disabled by
`enabled: false`. The entry name is the ISO resource or action type, so `provides` is `status` plus
the enabled sources and extensions, and `accepts` is the enabled handlers. Nothing is configured
twice. `class: "package.module:ClassName"` (or `"/path/to/file.py:ClassName"`) replaces the default
class; it is required for names without a default. Every other key is passed to the class as `params`.

| Entry | Default class params (default) | Notes |
|---|---|---|
| `sources.odometry` | `rate_hz` (2.0), `twist_topic` (`/odom`), `lost_after_s` (2.0) | TF pose in the CCS plus `/odom` twist. Adds `LOST`, `FORWARD`, `REVERSE`. |
| `sources.batteryStatus` | `topic` (`/battery_state`), `low_threshold` (0.2) | `sensor_msgs/BatteryState`. Adds `CHARGING`, `LOW_BATTERY`. |
| `sources.globalPlan` | `topic` (`/plan`) | `nav_msgs/Path`, republished when it changes. |
| `sources.localTrajectory` | `topic` (`/local_plan`), `rate_hz` (2.0) | `nav_msgs/Path`, throttled. |
| `handlers.move` | `action` (`/navigate_to_pose`) | Nav2 `NavigateToPose`; honours `toleranceRadius`. |
| `handlers.dock` | `mode` (`auto`: `docking_server` or `navigate`), `action` (`/dock_robot`), `dock_type` (`""`), `server_timeout_s` (2.0) | `DockRobot` by `dockId`, or by pose when `dock_type` is set; otherwise navigates to `dockLocation`. Adds `DOCKING`. |
| `handlers.undock` | `action` (`/undock_robot`), `dock_type` (`""`), `server_timeout_s` (2.0) | `UndockRobot`. Not accepted on Humble. |
| `handlers.pauseImr`, `handlers.resumeImr` | none | Pause cancels the Nav2 goal and keeps the objective; resume resends it. Adds `PAUSED`. |
| `handlers.customCommand` | `topic` (`/inorbit/custom_command`) | Opt-in, needs `class: iso21423_ros.oro:CustomCommandHandler`. |
| `extensions.customData` | `topic` (`/inorbit/custom_data`), `batch_ms` (500) | Opt-in. |

`cancelRequest` is handled by the library and is not listed in `accepts`. It sets `ctx.cancelled` on
the running handler.

## Hooks: your own sources and handlers

```python
class Source:                                 # publishes resources, may contribute status states
    def __init__(self, node, publisher, params): ...  # subscribe to what you need
    def states(self) -> list[str]: ...        # optional; fast and lock-free, called under the status lock
    def close(self): ...                      # optional; called at shutdown

class Handler:                                # serves one action type
    def __init__(self, node, params): ...
    def execute(self, action, ctx) -> ActionResult: ...  # blocking; return ctx.succeeded() or ctx.aborted(reason, msg)
```

- `node` is the connector node. Hooks may use `node.config`, `node.ccs` (map <-> CCS transforms),
  `node.map_frame`, `node.base_frame`, `node.tf_buffer`, `node.status` (call `refresh()` when your
  `states()` change, never while holding a lock that `states()` takes), `node.nav` (the shared
  `NavigateToPose` runner) and `node.entity`.
- `publisher` is the library's `EntityHandle`: `publish_odometry`, `publish_battery_status`,
  `publish_extension`, and so on.
- In `execute`, watch `ctx.cancelled` (a `threading.Event`) and report progress with
  `ctx.progress({...})`. Each request runs on its own thread.
- Source subscription and timer callbacks run serialised, one at a time, in the node's default
  callback group. A source doesn't need locks between its own callbacks. A callback that blocks
  delays every other source.
- A handler may define `close()`. The node calls it at shutdown, before it waits for in-flight
  requests to end. Stop your work there, for example by cancelling your own goals.
- A class that fails to load stops the node at startup with an error naming the config key.

[`config/example_vendor_hook.py`](config/example_vendor_hook.py) is a complete custom action `beep`
in 30 lines. It is installed to `share/iso21423_ros/config/`, which is
`/ws/install/iso21423_ros/share/iso21423_ros/config/example_vendor_hook.py` in the Docker image.
Hooks load from any module path or any `.py` file, so you can mount your own file into the container:

```yaml
    handlers:
      beep:
        class: /ws/install/iso21423_ros/share/iso21423_ros/config/example_vendor_hook.py:BeepHandler
        topic: /beep
```

## OpenRobOps extras (opt-in)

All of these are off by default, so the connector behaves the same on any broker.

```yaml
    credentials_provider: openrobops   # POST <oro.url>/iso_mqtt_config, retried every 10 s until admitted
    oro:
      url: https://your-oro-instance
      api_key_file: /run/secrets/oro_api_key
    extensions:
      customData: {enabled: true}      # std_msgs/String "key=value" lines, batched every 500 ms
    handlers:
      customCommand: {class: "iso21423_ros.oro:CustomCommandHandler"}   # republished as std_msgs/String
```

## Behaviour notes

- **Startup:** the broker connection (and the OpenRobOps credential fetch) retries every 10 s, so the
  node can start before the broker is up.
- **Shutdown** (SIGINT/SIGTERM): the node runs these steps in order:
  1. Each running `move`/`dock`/`undock` cancels its own Nav2 goal and ends ABORTED ("connector
     shutting down"). In-flight requests get up to 3 s to end.
  2. Sources are flushed.
  3. The node publishes the `OFFLINE` status, clears its retained topics and disconnects.
- **Preempt** (the default policy): a new request cancels the running one and starts at once. The
  library doesn't wait for the old handler to return, so a handler must cancel only its own Nav2
  goal, never "the current goal".
- **Status:** one `MODE_*` first, then the contributed states in priority order, published on change.
  The connector never emits `READY` or `NOT_READY`, which are fleet manager states, not IMR states.

## Humble notes

Humble's `nav2_msgs` has no `DockRobot`/`UndockRobot`, so:
- `undock` is left out of `accepts`.
- `dock` navigates to `dockLocation`. With `mode: docking_server`, a dock by `dockId` or with
  `dock_type` set ends ABORTED (docking server unavailable).

## Development

Tests run in the dev image, against a Mosquitto started inside it:

```bash
scripts/test.sh                      # Jazzy
ROS_DISTRO=humble scripts/test.sh    # Humble
scripts/test.sh -k test_node         # extra args go to pytest
```

## License

Apache-2.0
