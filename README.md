
# my-lidar-graph-slam-v2

This repository contains source files for the grid-based 2D LiDAR SLAM,
which were borrowed from the old repository (my-lidar-graph-slam).

- https://github.com/sterngerlach/my-lidar-graph-slam
- https://gitlab.com/arc.ics.keio.ac.jp/dev/2020/my-lidar-graph-slam

## TODO lists

The author's plans are listed as follows.
- Compare the performance of the grid-based scan matching methods
(i.e. correlative scan matching and branch-and-bound based matching)
against point-to-point or point-to-plane ICP algorithms using Open3D package
(or PCL).
- Reduce the large part of the data-transfer overhead between the
processing system (PS) and the programmable logic (PL) by storing the grid map
occupancy probability values on the CMA memory right from the beginning.
- Store the discretized angles and their corresponding cosine and sine values
as the look-up table on the on-chip block RAM (BRAM) to eliminate the
costly computations and reduce the complexity of the logic, improving the
throughput.
- Reduce the overhead for grid map accesses by directly storing the grid map
values in the `Patch` class rather than creating lots of `GridCell` instances
each of which contains a single grid map value.
