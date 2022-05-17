
# my-lidar-graph-slam-v2

This repository contains C/C++ implementation of the grid-based 2D LiDAR SLAM,
which were used in the following papers:

- A Unified Accelerator Design for LiDAR SLAM Algorithms for Low-end FPGAs (FPT, 2021)
  - https://ieeexplore.ieee.org/abstract/document/9609886
- A Universal LiDAR SLAM Accelerator System on Low-Cost FPGA (IEEE Access, 2022)
  - https://ieeexplore.ieee.org/document/9730869

The correlative scan matching (CSM) algorithm, which lies at the heart of the
2D LiDAR SLAM, is implemented on the TUL Pynq-Z2. The implementation of the
custom CSM IP core is found in the following GitHub repository.

- https://github.com/sterngerlach/hls_scan_matcher_correlative

We used Xilinx Vivado HLS 2019.2 for synthesis and Vivado 2019.2 for the
board-level implementation.

Our implementation is tested under the following environment:

- TUL Pynq-Z2
- Pynq Linux version 2.5 (based on Ubuntu 18.04)
- Python 3.6.5
- GCC 7.3.0
- Boost 1.65.1
- Eigen 3.3
- [g2o](https://github.com/RainerKuemmerle/g2o/releases/tag/20201223_git)

Please refer the above papers if you find it interesting. Thank you!
