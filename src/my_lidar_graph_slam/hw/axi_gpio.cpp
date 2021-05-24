
/* axi_gpio.cpp */

#include "my_lidar_graph_slam/hw/axi_gpio.hpp"

namespace MyLidarGraphSlam {
namespace Hardware {

/* Read the data register */
std::uint32_t AxiGPIO::Read(
    std::uint32_t startBit,
    std::uint32_t stopBit) const
{
    const std::uint32_t mask = (1 << (stopBit - startBit)) - 1;
    const std::uint32_t directionMask = mask << startBit;

    /* Set the data direction as input */
    this->WriteDataDirectionReg(
        0U, this->ReadDataDirectionReg(0U) | directionMask);
    /* Read the data value */
    return (this->ReadDataReg(0U) >> startBit) & mask;
}

/* Read the data register */
std::uint32_t AxiGPIO::Read() const
{
    /* Set the data direction as input */
    this->WriteDataDirectionReg(0U, ~0U);
    /* Read the data value */
    return this->ReadDataReg(0U);
}

/* Write to the data register */
void AxiGPIO::Write(
    std::uint32_t startBit,
    std::uint32_t stopBit,
    std::uint32_t regValue) const
{
    const std::uint32_t mask = (1 << (stopBit - startBit)) - 1;
    const std::uint32_t directionMask = mask << startBit;

    /* Set the data direction as output */
    this->WriteDataDirectionReg(
        0U, this->ReadDataDirectionReg(0U) & ~directionMask);
    /* Write to the data value */
    std::uint32_t newValue = (this->ReadDataReg(0U) & ~directionMask) |
                             ((regValue << startBit) & directionMask);
    this->WriteDataReg(0U, newValue);
}

/* Write to the data register with mask */
void AxiGPIO::Write(
    std::uint32_t regValue,
    std::uint32_t mask) const
{
    /* Set the data direction as output */
    this->WriteDataDirectionReg(
        0U, this->ReadDataDirectionReg(0U) & ~mask);
    /* Write to the data value */
    std::uint32_t newValue = (this->ReadDataReg(0U) & ~mask) |
                             (regValue & mask);
    this->WriteDataReg(0U, newValue);
}

/* Write to the data register */
void AxiGPIO::Write(std::uint32_t regValue) const
{
    /* Set the data direction as output */
    this->WriteDataDirectionReg(0U, 0U);
    /* Write to the data value */
    this->WriteDataReg(0U, regValue);
}

/* Toggle the specified bits */
void AxiGPIO::Toggle(
    std::uint32_t startBit,
    std::uint32_t stopBit) const
{
    const std::uint32_t mask = (1 << (stopBit - startBit)) - 1;
    const std::uint32_t directionMask = mask << startBit;

    /* Set the data direction as output */
    this->WriteDataDirectionReg(
        0U, this->ReadDataDirectionReg(0U) & ~directionMask);
    /* Write to the data value */
    std::uint32_t regValue = this->ReadDataReg(0U) ^ directionMask;
    this->WriteDataReg(0U, regValue);
}

/* Read the I/O direction (3-state control) register */
std::uint32_t AxiGPIO::ReadDataDirectionReg(std::uint32_t channel) const
{
    return this->mRegisters.Read(
        (channel - 1) * AxiGpioChannelOffset + AxiGpioTriOffset);
}

/* Write to the I/O direction (3-state control) register */
void AxiGPIO::WriteDataDirectionReg(
    std::uint32_t channel, std::uint32_t regValue) const
{
    this->mRegisters.Write(
        (channel - 1) * AxiGpioChannelOffset + AxiGpioTriOffset, regValue);
}

/* Read the data register */
std::uint32_t AxiGPIO::ReadDataReg(std::uint32_t channel) const
{
    return this->mRegisters.Read(
        (channel - 1) * AxiGpioChannelOffset + AxiGpioDataOffset);
}

/* Write to the data register */
void AxiGPIO::WriteDataReg(
    std::uint32_t channel, std::uint32_t regValue) const
{
    this->mRegisters.Write(
        (channel - 1) * AxiGpioChannelOffset + AxiGpioDataOffset, regValue);
}

/* Set the specified bits in data register */
void AxiGPIO::SetDataRegBits(
    std::uint32_t channel, std::uint32_t mask) const
{
    const std::uint32_t offsetInBytes =
        (channel - 1) * AxiGpioChannelOffset + AxiGpioDataOffset;
    this->mRegisters.Write(
        offsetInBytes, this->mRegisters.Read(offsetInBytes) | mask);
}

/* Clear the specified bits in data register */
void AxiGPIO::ClearDataRegBits(
    std::uint32_t channel, std::uint32_t mask) const
{
    const std::uint32_t offsetInBytes =
        (channel - 1) * AxiGpioChannelOffset + AxiGpioDataOffset;
    this->mRegisters.Write(
        offsetInBytes, this->mRegisters.Read(offsetInBytes) & ~mask);
}

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */
