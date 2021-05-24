
/* discrete_grid_cell.cpp */

#include "my_lidar_graph_slam/grid_map/discrete_grid_cell.hpp"

namespace MyLidarGraphSlam {

/* Update the value of the grid cell */
void DiscreteGridCell::Update(const double probValue)
{
    /* Ensure that the probability value is valid */
    assert(probValue != Unknown);
    assert(probValue >= 0.0 && probValue <= 1.0);

    /* Initialize the occupancy probability value of this grid cell
     * when observed for the first time */
    if (this->mValue == UnknownRaw) {
        /* Clamp the occupancy probability value for safety */
        this->mValue = ValueToRaw(probValue);
        return;
    }

    /* Update the occupancy probability value using Binary Bayes Filter */
    const double oldOdds = ValueToOdds(RawToValue(this->mValue));
    const double valueOdds = ValueToOdds(probValue);
    const double newValue = OddsToValue(oldOdds * valueOdds);

    /* Clamp the occupancy probability value for safety */
    this->mValue = ValueToRaw(newValue);
}

/* Convert the internal representation to the probability value */
double DiscreteGridCell::RawToValue(const std::uint16_t discretizedValue)
{
    if (discretizedValue == UnknownRaw)
        return Unknown;

    return static_cast<double>(discretizedValue - ValueMin) /
           static_cast<double>(ValueRange);
}

/* Convert the probability value to the internal representation */
std::uint16_t DiscreteGridCell::ValueToRaw(const double probValue)
{
    /* Ensure that the probability value is valid */
    assert(probValue >= 0.0 && probValue <= 1.0);

    if (probValue == Unknown)
        return UnknownRaw;
    if (probValue < 0.0)
        return ValueMin;
    if (probValue > 1.0)
        return ValueMax;

    return static_cast<std::uint16_t>(probValue * ValueRange) + ValueMin;
}

/* Compute an odds from the occupancy probability value */
double DiscreteGridCell::ValueToOdds(const double probValue)
{
    /* Ensure that the probability value is valid */
    assert(probValue != Unknown);
    assert(probValue >= 0.0 && probValue <= 1.0);

    if (probValue == Unknown)
        return 1.0;
    if (probValue < 0.0)
        return ProbabilityMin / (1.0 - ProbabilityMin);
    if (probValue > 1.0)
        return ProbabilityMax / (1.0 - ProbabilityMax);

    /* Clamp the occupancy probability value to avoid zero division */
    const double clampedValue =
        std::clamp(probValue, ProbabilityMin, ProbabilityMax);
    /* Compute an odds from the occupancy probability value */
    return clampedValue / (1.0 - clampedValue);
}

/* Compute an occupancy probability value from the odds */
double DiscreteGridCell::OddsToValue(const double oddsValue)
{
    /* Ensure that the odds is valid */
    assert(oddsValue >= 0.0);

    if (oddsValue < 0.0)
        return Unknown;

    /* Compute an occupancy probability value from the odds */
    const double probValue = oddsValue / (1.0 + oddsValue);
    /* Clamp the occupancy probability value */
    const double clampedValue =
        std::clamp(probValue, ProbabilityMin, ProbabilityMax);
    return clampedValue;
}

} /* namespace MyLidarGraphSlam */
