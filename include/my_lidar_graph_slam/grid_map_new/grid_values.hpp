
/* grid_values.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_VALUES_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_VALUES_HPP

#include <cstdint>
#include <vector>

namespace MyLidarGraphSlam {

/* Convert the probability value to the discretized value */
inline std::uint16_t ProbabilityToValue(
    const double prob,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax)
{
    return valueMin + (prob - probMin) *
           static_cast<double>(valueMax - valueMin) /
           (probMax - probMin);
}

/* Convert the discretized value to the probability value */
inline double ValueToProbability(
    const std::uint16_t value,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax)
{
    return probMin + (probMax - probMin) *
           static_cast<double>(value - valueMin) /
           static_cast<double>(valueMax - valueMin);
}

/* Convert the probability value to the odds */
inline double ProbabilityToOdds(const double prob)
{
    return prob / (1.0 - prob);
}

/* Convert the odds to the probability */
inline double OddsToProbability(const double odds)
{
    return odds / (1.0 + odds);
}

/* Convert the discretized value to the odds */
inline double ValueToOdds(
    const std::uint16_t value,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax)
{
    const double prob = ValueToProbability(
        value, valueMin, valueMax, probMin, probMax);
    return ProbabilityToOdds(prob);
}

/* Compute the lookup table for converting from discretized values to
 * corresponding probability values */
std::vector<double> ComputeValueToProbabilityLookup(
    const std::uint16_t unknownValue,
    const double unknownProb,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax);

/* Compute the lookup table for converting from discretized values to
 * corresponding odds */
std::vector<double> ComputeValueToOddsLookup(
    const std::uint16_t unknownValue,
    const double unknownProb,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax);

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_NEW_GRID_VALUES_HPP */
