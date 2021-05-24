
/* axi_dma.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_AXI_DMA_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_AXI_DMA_HPP

#include "my_lidar_graph_slam/hw/mmio_shared.hpp"

#include <cstdint>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * Class for DMA channel (MM2S / S2MM) management
 */
class AxiDMAChannel
{
public:
    /* Constructor */
    AxiDMAChannel(const SharedMemoryMappedIO& mmapRegisters,
                  std::uint32_t addressOffset) :
        mRegisters(mmapRegisters),
        mAddressOffset(addressOffset),
        mIsFirstTransfer(false) { }

    /* Destructor */
    ~AxiDMAChannel() = default;

    /* Reset the DMA channel */
    void Reset();

    /* Start the DMA channel */
    void Start();

    /* Stop the DMA channel */
    void Stop();

    /* Transfer the memory data */
    void Transfer(std::uint32_t dataLengthInBytes,
                  std::uint32_t physicalAddress);

    /* Wait for the transfer to complete */
    void Wait();

    /* Read the control register */
    std::uint32_t ReadControlReg() const;

    /* Read the status register */
    std::uint32_t ReadStatusReg() const;

    /* Check if AXI DMA is halted */
    inline bool IsHalted() const
    { return this->ReadStatusReg() & AxiDmaStatusHalted; }

    /* Check if AXI DMA is running */
    inline bool IsRunning() const
    { return !this->IsHalted(); }

    /* Check if AXI DMA is idle */
    inline bool IsIdle() const
    { return this->ReadStatusReg() & AxiDmaStatusIdle; }

    /* Check if AXI DMA has slave error */
    inline bool HasDMASlaveError() const
    { return this->ReadStatusReg() & AxiDmaStatusDMASlvError; }

    /* Check if AXI DMA has decode error */
    inline bool HasDMADecodeError() const
    { return this->ReadStatusReg() & AxiDmaStatusDMADecError; }

private:
    /* Write to the control register */
    void WriteControlReg(std::uint32_t regValue) const;

    /* Write to the address register */
    void WriteAddrReg(std::uint32_t regValue) const;

    /* Write to the address MSB register (upper 32 bits) */
    void WriteAddrMSBReg(std::uint32_t regValue) const;

    /* Write to the length register */
    void WriteLengthReg(std::uint32_t regValue) const;

private:
    /* Constants for AXI DMA registers */
    /* Offset to the control register */
    static constexpr const std::uint32_t AxiDmaCtrlRegOffset   = 0x00;
    /* Offset to the status register */
    static constexpr const std::uint32_t AxiDmaStatusRegOffset = 0x04;

    /* Offset to the MM2S source / S2MM destination address */
    static constexpr const std::uint32_t AxiDmaAddrOffset    = 0x18;
    /* Offset to the MM2S source / S2MM destination address (upper 32 bits) */
    static constexpr const std::uint32_t AxiDmaAddrMSBOffset = 0x1C;
    /* Offset to the MM2S transfer / S2MM buffer length in bytes */
    static constexpr const std::uint32_t AxiDmaLengthOffset  = 0x28;

    /* Bitmasks for control registers */
    static constexpr const std::uint32_t AxiDmaCtrlRunStop = 0x01;
    static constexpr const std::uint32_t AxiDmaCtrlReset   = 0x04;
    static constexpr const std::uint32_t AxiDmaCtrlKeyhole = 0x08;
    static constexpr const std::uint32_t AxiDmaCtrlCyclic  = 0x10;

    /* Bitmasks for status registers */
    static constexpr const std::uint32_t AxiDmaStatusHalted      = 0x01;
    static constexpr const std::uint32_t AxiDmaStatusIdle        = 0x02;
    static constexpr const std::uint32_t AxiDmaStatusDmaIntError = 0x10;
    static constexpr const std::uint32_t AxiDmaStatusDMASlvError = 0x20;
    static constexpr const std::uint32_t AxiDmaStatusDMADecError = 0x40;

private:
    /* Memory mapped AXI DMA registers */
    SharedMemoryMappedIO mRegisters;
    /* Address offset */
    std::uint32_t        mAddressOffset;
    /* Flag to determine whether being the first transfer */
    bool                 mIsFirstTransfer;
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_AXI_DMA_HPP */
