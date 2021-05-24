
/* ap_ctrl.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_AP_CTRL_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_AP_CTRL_HPP

#include <cstdint>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * AxiLiteApCtrl enum represents the block-level control signals for
 * the AXI4-Lite slave interface
 */
enum class AxiLiteSApCtrl : std::uint32_t
{
    Start       = 0x01,
    Done        = 0x02,
    Idle        = 0x04,
    Ready       = 0x08,
    AutoRestart = 0x80,
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_AP_CTRL_HPP */
