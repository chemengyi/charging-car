# Industrial Mobile Charging Scheduling

ROS 2 research software for scheduling mobile charging vehicles (MCVs) that replenish working vehicles at designated locations in an industrial road network.

The scheduler links road travel, arrival-dependent charging demand, charger energy availability, and repeated station-to-station trips. An event-based simulator evaluates how a fleet recovers from delayed services through fixed-assignment or joint rescheduling. A separate task-planning component connects dispatched missions to road-following and local-motion planners.

## Features

- **Arrival-dependent demand:** recipient vehicles consume energy while waiting. Predicted arrival time determines charging demand and service duration.
- **Multiple chargers and trips:** a decoder converts a target priority order into MCV assignments, service sequences, and station returns.
- **Alternative travel representations:** compare estimated connection distance, directed shortest paths, abstract endpoint access, and Euclidean distance.
- **Two service priorities:** minimize either the number of late recipients or their accumulated overtime first.
- **Completion-triggered rescheduling:** preserve dispatched actions and repair the remaining work after a delayed completion.
- **Controlled experiments:** compare travel models, recovery policies, fleet capacity, station turnaround, and dynamic versus frozen demand.

Adaptive large neighborhood search (ALNS) is the main service-order search method. The research focus is the scheduling model and its operating dependencies, rather than a claim that ALNS itself is a new algorithm.

## Scope

The numerical experiments use mapped roads, prescribed requests, and simulated time and energy states. Map 1 corresponds to the mapped Jigang industrial site; Map 2 is an additional road-network case. Requests and vehicle parameters are simulation inputs, not records of an observed factory shift.

Each request has a nominated service location that remains fixed. Recipients may continue working while waiting, but the model assumes they are ready at that location when the MCV arrives. Recipient rendezvous, station queues, physical energy transfer, and production throughput are not evaluated.

Travel costs include local access and maneuver estimates. They do not certify obstacle clearance, turning feasibility, or permission to cross a boundary. The task-planning interfaces should not be interpreted as evidence of validated multi-vehicle physical charging.

## Code and supporting files

The ROS package is named `task_planner`. In the original Autoware workspace, its location is `src/universe/planning/task_planner/`.

| Component | Purpose |
|---|---|
| `src/charge_scheduler.cpp` | Distance construction, service decoding, search, demand evaluation, and rescheduling |
| `include/task_planner/charge_scheduler.hpp` | Scheduler declarations |
| `src/task_planner.cpp` and associated node/utilities | Mission-level task planning and execution interfaces |
| `launch/charge_scheduler.launch.xml` | Scheduler launch file; accepts a `param_file` argument |
| `config/charge_scheduler.param.yaml` | Package-level default configuration |
| `cases/` in the experiment workspace | Case-specific parameters, including the dynamic/frozen comparison |
| `run_4c_full.sh`, `run_4c_all.sh` | Original single-case and batch wrappers for the demand comparison |
| `aggregate_4c.py` | Demand-comparison log aggregation |

Package-relative paths in the first five rows are relative to `task_planner/`. Experiment files are separate from the ROS package. The commands below assume they have been placed at the root of the experiment workspace.

The complete package is required for building: the CMake configuration also compiles the task-planning executable. A diagnostic snapshot containing only the scheduler source and header is not a standalone buildable release.

## Requirements and build

The supplied experiment wrappers target **ROS 2 Humble** within an existing **Autoware-based workspace**. This is not a plain C++ project or a standalone ROS installation.

Dependencies declared by the package include:

- `ament_cmake_auto`, `autoware_cmake`, and `eigen3_cmake_module`;
- Eigen3 and `yaml-cpp`;
- `rclcpp`, `tf2_geometry_msgs`, and standard ROS message packages;
- `lanelet2_routing`, `lanelet2_traffic_rules`, and `lanelet2_extension`;
- Autoware vehicle, perception, mapping, and planning message packages;
- the workspace-specific `promote_planning_msgs` package.

Use a compatible Autoware dependency set. In particular, `promote_planning_msgs` and the project's motion-planning components must be supplied by the host workspace. No portable Docker image is specified by this README.

From the **actual ROS workspace root**, after adding the complete package and its dependencies under `src/`:

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --packages-up-to task_planner --symlink-install
source install/setup.bash
ros2 pkg executables task_planner
```

If dependencies are provided by another already-built workspace, source that workspace's `install/setup.bash` before building. `--packages-up-to` builds dependencies that are present in the workspace; it does not download missing packages.

The executable list should include `charge_scheduler`. The full package also builds the `task_planner` executable.

## Run one scheduling case

### 1. Prepare the map

Start the host workspace's Lanelet2 map publisher with the map corresponding to the selected case. The scheduler subscribes to `/map/vector_map` using transient-local durability and an Autoware `HADMapBin` message. Map coordinates and request coordinates must use the same frame.

Check the publisher:

```bash
ros2 topic info /map/vector_map --verbose
```

A publisher count alone does not prove that the scheduler has received a valid map. Check the scheduler's map-ready message after launch. For an isolated numerical experiment, run only the required map and scheduler components, without connecting to a live vehicle execution stack. RViz is optional for the numerical experiments.

### 2. Load an explicit experiment YAML

From the experiment workspace root, with the built ROS environment sourced:

```bash
CHARGING_WS="$(pwd)"
CASE_YAML="$CHARGING_WS/cases/4c_map1_M20_objN_dynamic_seed1.yaml"

if [ ! -f "$CASE_YAML" ]; then
  echo "Case YAML not found: $CASE_YAML" >&2
else
  mkdir -p "$CHARGING_WS/audit_runs"
  set -o pipefail
  ros2 launch task_planner charge_scheduler.launch.xml \
    param_file:="$CASE_YAML" 2>&1 | \
    tee "$CHARGING_WS/audit_runs/4c_map1_M20_objN_dynamic_seed1.log"
fi
```

The case above uses Map 1, 20 recipients, two MCVs, the late-count-first objective, dynamic demand, and seed 1. Stop the launch after the final result has been emitted; a running ROS process is not itself evidence that a numerical experiment is unfinished.

**Do not use constructor defaults to reproduce the paper.** The supplied experiment YAML sets MCV capacity to **800 kWh**, while the source retains a **31.3 kWh** fallback default. Charging power and search budgets also differ between defaults and experiment configurations. Always pass the intended YAML and inspect the effective settings in the startup log.

## Configuration and model labels

### Travel models

| Paper label | `distance_mode` | Definition |
|---|---|---|
| ECD — estimated connection distance | `maneuver` | Directed lane connections with designated maneuver costs and local endpoint access estimates |
| DSP — directed shortest-path model | `dsp` | Directed following-only routing with fixed endpoint matches; uses ECD when that route is unreachable |
| ABS — abstract road model | `abstract` | Relaxes endpoint lane access and takes the least-cost directed following route over eligible endpoint combinations |
| EUC — Euclidean distance model | `euclid` | Straight-line endpoint separation |

Historical scripts and logs may call ECD `MAN` or `maneuver`. These labels refer to the corresponding implementation mode; they are not additional independent baselines.

ABS does not make the entire internal graph undirected. The static dashed-segment extraction and ABS's opposite-lane endpoint matching are different operations. In the scheduler source, static extraction requires exactly two lanelets to reference the same dashed left boundary. ECD also has a local connection estimate for a destination behind the origin on the same lanelet, so an ECD–DSP comparison does not isolate dashed-boundary maneuvers alone.

For the travel-model comparison, distinguish the cost used during order search from the **common final ECD evaluation**. Use the unified evaluation records when comparing reported service performance.

### Objectives and demand

| Parameter | Values and meaning |
|---|---|
| `objective_order` | `nlate_first`: late count, then overtime, then distance; `tover_first`: overtime, then late count, then distance |
| `demand_update_mode` | `dynamic`: use predicted arrival time during search; `frozen`: use request-time demand during search |
| `heuristic_method` | `alns` for the main experiments |
| `num_chargers` | Number of MCVs |
| `alns_seed`, `alns_iters` | Random seed and initial search budget |
| `battery_swap`, `swap_time_min` | Enable station battery swapping and specify its duration |
| `execution_simulation_mode` | Enable the event-based execution experiment |
| `enable_reschedule` | Enable rescheduling, subject to the selected execution mode and policy |
| `reschedule_threshold_min` | Completion-delay trigger threshold; 10 minutes in the reported experiments |
| `reschedule_alns_iters` | Search budget per repair invocation |

The objectives are lexicographic: lower-priority metrics resolve ties in higher-priority metrics. Makespan provides a further tie-breaker. Scalar guide costs used within search should not be confused with the reported objective tuple.

### Energy parameters and units

Representative demand-comparison configurations use the following values; the selected case YAML remains authoritative.

| Parameter | Value | Unit or interpretation |
|---|---:|---|
| `battery_capacity_kwh` | 800.0 | MCV battery capacity, kWh |
| `charge_power_kw` | 360.0 | Power delivered to the recipient, kW |
| `charge_efficiency` | 0.97 | Transfer efficiency |
| `drive_consume_kwh_per_km` | 0.6 | MCV propulsion consumption, kWh/km |
| `avg_speed` | 66.7 | Scheduling travel speed, m/min |
| `soc_low_threshold` | 0.20 | MCV residual-energy fraction after return |
| `target_charge_target` | 0.80 | Desired recipient SOC fraction |
| `heavy_soc_threshold` | 20.0 | Heavy-to-light operating threshold, percent SOC |
| `park_soc_threshold` | 5.0 | Recipient stopping threshold, percent SOC |
| `swap_time_min` | 10.0 | Station battery-swap duration, min |

SOC parameters use **both fractions and percentages**, as indicated above. Recipient capacities, initial SOCs, operating modes, and positions are specified by the case.

A feasible service must leave enough MCV energy for the modeled return journey **and** the residual floor. With the settings above, the floor is 160 kWh after return. Swapping restores full MCV capacity. Station-charging settings are inactive in the swap experiments.

## Reproduce the experiments

### Dynamic versus frozen demand

The demand comparison, labeled **4C** in the scripts, contains:

- two maps;
- M20/K2 and M30/K3 cases;
- OBJ-N and OBJ-T objectives;
- dynamic and frozen search modes;
- ten matched seeds per configuration.

This gives **160 runs and 80 matched pairs**. Initial search uses 15,000 iterations; event-based execution and rescheduling are disabled for this comparison.

Both final priority orders are evaluated with the dynamic-demand ECD decoder. That evaluation reconstructs MCV assignments and trips. It is not a physical rollout that preserves the original frozen-demand assignments unchanged.

The original wrapper interface is:

```text
bash run_4c_full.sh MAP CASE OBJECTIVE MODE SEED
MAP:       map1 | map2
CASE:      M20 | M30
OBJECTIVE: N | T
MODE:      dynamic | frozen
SEED:      1 ... 10
```

Before using the wrappers on another machine, update their workspace and map paths. `run_4c_all.sh` also contains a machine-local Docker image identifier that must be replaced with a compatible local image. Both scripts assign these settings internally; exporting a variable alone does not override them. The single-case wrapper forcibly cleans up map-loader processes by name, so use it only in a dedicated experiment environment without unrelated map loaders.

For a manual single run, use the launch command above instead. Place `aggregate_4c.py` next to `audit_runs/`, then run:

```bash
python3 aggregate_4c.py
```

It writes `audit_runs/4c_results_detail.csv` and `audit_runs/4c_results_summary.csv`. Aggregation alone does not establish completeness: verify the expected 160 unique configurations and their final evaluation records.

### Travel models and rescheduling

The travel-model experiment (**E2**) covers M10/K1, M10R/K1, M20/K2, and M30/K3. M10R changes target placement at a fixed target count. Optimize OBJ-N and OBJ-T separately, and keep search and common-evaluation metrics distinct.

The disturbance experiment (**E7**) compares:

| Policy | Allowed response |
|---|---|
| Open | Retain the initial plan |
| Fixed-assignment | Repair unstarted work within its assigned MCV |
| Global-joint | Reassign and reorder unstarted work across MCVs |

Dispatched actions remain committed. Target-service and station-turnaround delays use 15, 30, or 60 minutes, together with a combined 60-minute case and a no-disturbance reference. M20/K2 and M30/K3 each use ten seeds per scenario.

Keep the original batch's YAML, code revision, objective, map, seed, and iteration budgets together. The development scripts include historical runners with different purposes and dependencies; a similarly named script is not sufficient evidence that it reproduces a particular final table. Some runners require companion files such as `common.sh` and validation scripts. Do not run an isolated historical shell script without those files.

## Outputs and checks

| Metric | Meaning | Unit |
|---|---|---|
| `N_late` / `n_late` | Recipients reached after their energy-related deadline | vehicles |
| `T_over` / `t_over` | Sum of positive arrival-time overruns | min |
| `C_total` / `c_total` | Total modeled MCV travel distance, including station returns | m |
| `makespan` | Latest final-return time | min |
| `energy_viol` | Recorded violations of the modeled energy constraints | count |
| `served` | Number of recipients served | vehicles |
| `trips` | Number of station-to-station trips | count |
| `reschedules` | Number of repair events | count |
| `reschedule_ms` | Cumulative measured repair computation time | ms |
| `changed_assign` | Recorded target-assignment changes during repair | count |

`C_total` is a distance, not a weighted objective or a measured odometry trace. Simulated service times and measured computation times are different quantities. Summary standard deviations in the demand aggregation use the population convention.

For each batch, check that all expected runs are present, final evaluation is complete, all required recipients are served, and feasibility is reported alongside service performance. Match seeds and inputs between methods. Fixed-assignment should report no cross-vehicle reassignment, and the no-disturbance reference should be unchanged across recovery policies. Open can become energy-infeasible after a delay; its shorter distance or delay is not automatically an operating advantage.

## Troubleshooting

| Symptom | What to check |
|---|---|
| `ignoring unknown package 'task_planner'` | Run `colcon` from the ROS workspace containing the package; confirm it appears in `colcon list` |
| `lanelet2_coreConfig.cmake` not found | Check that Lanelet2 is installed or built and that the correct dependency workspace is sourced |
| Missing `promote_planning_msgs` | Obtain the compatible custom message package from the host project |
| No scheduling result | Check map receipt, coordinate consistency, target configuration, and startup errors |
| Wrong battery capacity or iteration count | Pass the intended YAML through `param_file` and inspect effective startup parameters |
| Old runs unexpectedly skipped | Use a fresh log directory or verify the wrapper's resume logic and completed-run markers |
| YAML parameter type error | Preserve declared types; for example, use `30.0` for the double-valued `disturbance_delay_min` |

## Citation and licensing

When using this implementation, identify the repository URL and the exact commit or release tag used. Cite the associated paper when its final bibliographic record is available. No publication DOI is assigned in this README.

The `task_planner/package.xml` manifest declares **Apache License 2.0**. This README does not assign a new license or extend that declaration to third-party dependencies, maps, point clouds, or other datasets. Consult the license and attribution files distributed with each component; a site map's use in an experiment does not by itself establish redistribution permission.
