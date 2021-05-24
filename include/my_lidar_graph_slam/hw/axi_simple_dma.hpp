
/* axi_simple_dma.hpp */

#include "my_lidar_graph_slam/hw/axi_dma.hpp"

#ifndef MY_LIDAR_GRAPH_SLAM_HW_AXI_SIMPLE_DMA_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_AXI_SIMPLE_DMA_HPP

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * Class for interacting with the AXI DMA IP core
 * Only Simple DMA (Direct Register Mode) is supported
 * Interrupt, Scatter/Gather mode and multichannel DMA are not supported
 */
class AxiSimpleDMA
{
public:
    /* Constructor */
    explicit AxiSimpleDMA(const SharedMemoryMappedIO& mmapRegisters) :
        mSendChannel(mmapRegisters, AxiDmaMM2SOffset),
        mRecvChannel(mmapRegisters, AxiDmaS2MMOffset) { }

    /* Destructor */
    ~AxiSimpleDMA() = default;

    /* Get the memory map to stream (MM2S / Tx) channel */
    inline AxiDMAChannel& SendChannel() noexcept
    { return this->mSendChannel; }
    /* Get the memory map to stream (MM2S / Tx) channel */
    inline const AxiDMAChannel& SendChannel() const noexcept
    { return this->mSendChannel; }

    /* Get the stream to memory map (S2MM / Rx) channel */
    inline AxiDMAChannel& RecvChannel() noexcept
    { return this->mRecvChannel; }
    /* Get the stream to memory map (S2MM / Rx) channel */
    inline const AxiDMAChannel& RecvChannel() const noexcept
    { return this->mRecvChannel; }

private:
    /* Offset to the MM2S registers */
    static constexpr const std::uint32_t AxiDmaMM2SOffset = 0x00;
    /* Offset to the S2MM registers */
    static constexpr const std::uint32_t AxiDmaS2MMOffset = 0x30;

private:
    /* Memory map to stream (MM2S) channel */
    AxiDMAChannel mSendChannel;
    /* Stream to memory map (S2MM) channel */
    AxiDMAChannel mRecvChannel;
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_AXI_SIMPLE_DMA_HPP */
