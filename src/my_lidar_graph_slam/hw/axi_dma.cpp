
/* axi_dma.cpp */

#include "my_lidar_graph_slam/hw/axi_dma.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Hardware {

/* Reset the DMA channel */
void AxiDMAChannel::Reset()
{
    /* Set the DMA control register */
    this->WriteControlReg(this->ReadControlReg() | AxiDmaCtrlReset);
    /* Ensure that the DMA channel is reset */
    while (this->ReadControlReg() & AxiDmaCtrlReset);
}

/* Start the DMA channel */
void AxiDMAChannel::Start()
{
    /* Set the DMA control register */
    this->WriteControlReg(this->ReadControlReg() | AxiDmaCtrlRunStop);
    /* Ensure that the DMA channel is started */
    while (this->ReadStatusReg() & AxiDmaStatusHalted);
    /* Set the first transfer flag */
    this->mIsFirstTransfer = true;
}

/* Stop the DMA channel */
void AxiDMAChannel::Stop()
{
    /* Set the DMA control register */
    this->WriteControlReg(this->ReadControlReg() & ~AxiDmaCtrlRunStop);
    /* Ensure that the DMA channel is stopped */
    while (!(this->ReadStatusReg() & AxiDmaStatusHalted));
}

/* Transfer the memory data */
void AxiDMAChannel::Transfer(
    std::uint32_t dataLengthInBytes,
    std::uint32_t physicalAddress)
{
    XAssert(this->IsRunning(),
            "DMA channel is not yet started");
    XAssert(this->IsIdle() || this->mIsFirstTransfer,
            "DMA channel is not idle");

    /* Write to the address register */
    this->WriteAddrReg(physicalAddress);

    /* Write to the length register */
    this->WriteLengthReg(dataLengthInBytes);

    /* Set the first transfer flag */
    this->mIsFirstTransfer = false;
}

/* Wait for the transfer to complete */
void AxiDMAChannel::Wait()
{
    XAssert(this->IsRunning(), "DMA channel is not yet started");

    /* Wait until the DMA channel becomes idle state */
    while (!(this->ReadStatusReg() & AxiDmaStatusIdle));
}

/* Read the control register */
std::uint32_t AxiDMAChannel::ReadControlReg() const
{
    return this->mRegisters.Read(
        this->mAddressOffset + AxiDmaCtrlRegOffset);
}

/* Read the status register */
std::uint32_t AxiDMAChannel::ReadStatusReg() const
{
    return this->mRegisters.Read(
        this->mAddressOffset + AxiDmaStatusRegOffset);
}

/* Write to the control register */
void AxiDMAChannel::WriteControlReg(std::uint32_t regValue) const
{
    this->mRegisters.Write(
        this->mAddressOffset + AxiDmaCtrlRegOffset, regValue);
}

/* Write to the address register */
void AxiDMAChannel::WriteAddrReg(std::uint32_t regValue) const
{
    this->mRegisters.Write(
        this->mAddressOffset + AxiDmaAddrOffset, regValue);
}

/* Write to the address MSB register (upper 32 bits) */
void AxiDMAChannel::WriteAddrMSBReg(std::uint32_t regValue) const
{
    this->mRegisters.Write(
        this->mAddressOffset + AxiDmaAddrMSBOffset, regValue);
}

/* Write to the length register */
void AxiDMAChannel::WriteLengthReg(std::uint32_t regValue) const
{
    this->mRegisters.Write(
        this->mAddressOffset + AxiDmaLengthOffset, regValue);
}

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */
