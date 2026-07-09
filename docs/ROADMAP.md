# Roadmap / Remaining Work

Last updated: 2026-07-09

Everything originally planned (point cloud map generation, Nav2
occupancy grids, full geometry coverage, Fuel auto-download, NDT
validation, lint + CI) is **implemented and verified**. This document
tracks the one pending decision and the optional items to pick up when
their trigger arrives.

## Pending decision

### 1. Whether to implement "method 2" (simulated-scan mapping)

An alternative approach that accumulates scans from a virtual lidar
driven through the world in Gazebo.

- Status: with method 1 (offline geometric sampling), PCL NDT converges
  from a 0.5 m / 8.6° initial offset to 9.5 mm / 0.03° error on a real
  world (kachaka warehouse).
- Recommendation: **do not implement**. Method 2's advantage
  (occlusion-consistent maps) does not help NDT in practice, while its
  drawbacks (range-dependent point density, scan-pose planning, runtime
  cost) remain.
- Revisit only if NDT underperforms in on-robot deployments.

## Optional items (pick up when needed)

| # | Item | When to start | Notes |
|---|---|---|---|
| 2 | Tag a `v0.1.0` release | anytime | Create a GitHub Release matching the package.xml version |
| 3 | DEM / GeoTIFF heightmap support | when outdoor / real-terrain worlds appear | Use gz-common's `Dem` class (GDAL). Only image heightmaps are supported today; unsupported formats emit a warning |
| 4 | Change the copyright holder in license headers | when the publishing identity changes | All sources currently say `Copyright 2026 atinfinity`; a bulk replace suffices |
| 5 | Unknown cells in the occupancy grid | when Nav2 should treat out-of-building areas as unknown | Output is currently binary occupied/free within bounds; requires an xy-projection test against floor geometry |
| 6 | `--world-name` option | when SDF files with multiple worlds appear | Currently the first world is processed and a NOTE is printed |
| 7 | CI extensions | when build times grow or more distros are targeted | apt caching, a ROS Rolling build matrix, etc. |
| 8 | Reinstate xmllint | when splitting CI-only lints | Removed because fetching the package.xml schema hangs offline; could run only on networked CI |

## History of implemented work (reference)

- 2026-07-09: initial release. SDF → PCD / occupancy grid core, all
  primitives + mesh + heightmap + polyline, `relative_to` frame
  resolution, Fuel auto-download, `--exclude` /
  `--transparency-threshold` / `--grid-close` options, NDT regression
  test, GitHub Actions CI.
- Design decision: `<actor>` entities are **never mapped by design** —
  they are dynamic objects.
- Lesson learned: gz-math7's `Pose3d::operator*` uses the matrix
  convention (parent × child), the opposite of Gazebo Classic. The bug
  only manifests with a rotated child under a translated parent; a
  regression test guards it (`test/test_world_sampler.cpp`).
