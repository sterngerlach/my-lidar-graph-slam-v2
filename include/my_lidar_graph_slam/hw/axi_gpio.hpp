
/* axi_gpio.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_HW_AXI_GPIO_HPP
#define MY_LIDAR_GRAPH_SLAM_HW_AXI_GPIO_HPP

#include "my_lidar_graph_slam/hw/mmio_shared.hpp"

#include <cstdint>

namespace MyLidarGraphSlam {
namespace Hardware {

/*
 * Class for interacting with the AXI GPIO
 * Interrupt and dual channel are not supported
 */
class AxiGPIO
{
public:
    /* Constructor */
    AxiGPIO(const SharedMemoryMappedIO& mmapRegisters) :
        mRegisters(mmapRegisters) { }

    /* Destructor */
    ~AxiGPIO() = default;

    /* Read the data register */
    std::uint32_t Read(std::uint32_t startBit,
                       std::uint32_t stopBit) const;

    /* Read the data register */
    std::uint32_t Read() const;

    /* Write to the data register */
    void Write(std::uint32_t startBit,
               std::uint32_t stopBit,
               std::uint32_t regValue) const;

    /* Write to the data register with mask */
    void Write(std::uint32_t regValue,
               std::uint32_t mask) const;

    /* Write to the data register */
    void Write(std::uint32_t regValue) const;

    /* Toggle the specified bits */
    void Toggle(std::uint32_t startBit, std::uint32_t stopBit) const;

    /* Turn off the specified bits */
    inline void Off(std::uint32_t startBit, std::uint32_t stopBit) const
    { this->Write(startBit, stopBit, 0U); }

    /* Turn on the specified bits */
    inline void On(std::uint32_t startBit, std::uint32_t stopBit) const
    { this->Write(startBit, stopBit, ~0U); }

private:
    /* Read the I/O direction (3-state control) register */
    std::uint32_t ReadDataDirectionReg(std::uint32_t channel) const;

    /* Write to the I/O direction (3-state control) register */
    void WriteDataDirectionReg(std::uint32_t channel,
                               std::uint32_t regValue) const;

    /* Read the data register */
    std::uint32_t ReadDataReg(std::uint32_t channel) const;

    /* Write to the data register */
    void WriteDataReg(std::uint32_t channel,
                      std::uint32_t regValue) const;

    /* Set the specified bits in data register */
    void SetDataRegBits(std::uint32_t channel,
                        std::uint32_t mask) const;

    /* Clear the specified bits in data register */
    void ClearDataRegBits(std::uint32_t channel,
                          std::uint32_t mask) const;

private:
    /* Constants for AXI GPIO registers */
    /* Offset to the data register */
    static constexpr const std::uint32_t AxiGpioDataOffset    = 0x00;
    /* Offset to the I/O (3-state) register */
    static constexpr const std::uint32_t AxiGpioTriOffset     = 0x04;
    /* Offset for the channel */
    static constexpr const std::uint32_t AxiGpioChannelOffset = 0x08;

private:
    /* Memory mapped AXI GPIO registers */
    SharedMemoryMappedIO mRegisters;
};

} /* namespace Hardware */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_HW_AXI_GPIO_HPP */
