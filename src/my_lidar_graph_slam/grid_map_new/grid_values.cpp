
/* grid_values.cpp */

#include "my_lidar_graph_slam/grid_map_new/grid_values.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {

/* Compute the lookup table for converting from discretized values to
 * corresponding probability values */
std::vector<double> ComputeValueToProbabilityLookup(
    const std::uint16_t unknownValue,
    const double unknownProb,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax)
{
    /* Check the input values */
    Assert(valueMin == 1U);
    Assert(valueMax > valueMin);
    Assert(probMax > probMin);
    Assert(probMax < 1.0);
    Assert(probMin > 0.0);

    /* Make sure that the unknown grid value is 0 and the minimum value is 1
     * Otherwise, the following lookup computations will not work */
    Assert(unknownValue == 0U);
    Assert(unknownProb == 0.0);

    /* Initialize the lookup table */
    const std::size_t numOfValues =
        static_cast<std::size_t>(valueMax - valueMin) + 1U;
    std::vector<double> lookupTable;
    lookupTable.resize(numOfValues);

    /* Set the unknown probability value */
    lookupTable[unknownValue] = unknownProb;

    /* Set the other probability values */
    for (std::size_t i = 1; i < numOfValues; ++i)
        lookupTable[i] = ValueToProbability(
            i, valueMin, valueMax, probMin, probMax);

    return lookupTable;
}

/* Compute the lookup table for converting from discretized values to
 * corresponding odds */
std::vector<double> ComputeValueToOddsLookup(
    const std::uint16_t unknownValue,
    const double unknownProb,
    const std::uint16_t valueMin,
    const std::uint16_t valueMax,
    const double probMin,
    const double probMax)
{
    /* Check the input values */
    Assert(valueMin == 1U);
    Assert(valueMax > valueMin);
    Assert(probMax > probMin);
    Assert(probMax < 1.0);
    Assert(probMin > 0.0);

    /* Make sure that the unknown grid value is 0 and the minimum value is 1
     * Otherwise, the following lookup computations will not work */
    Assert(unknownValue == 0U);
    Assert(unknownProb == 0.0);

    /* Initialize the lookup table */
    const std::size_t numOfValues =
        static_cast<std::size_t>(valueMax - valueMin) + 1U;
    std::vector<double> lookupTable;
    lookupTable.resize(numOfValues);

    /* Compute the unknown odds */
    lookupTable[unknownValue] = 1.0;

    /* Compute the other odds */
    for (std::size_t i = 1; i < numOfValues; ++i)
        lookupTable[i] = ValueToOdds(
            i, valueMin, valueMax, probMin, probMax);

    return lookupTable;
}

} /* namespace MyLidarGraphSlam */
