
/* scan_outlier_filter.cpp */

#include "my_lidar_graph_slam/mapping/scan_outlier_filter.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanOutlierFilter::ScanOutlierFilter(const double validRangeMin,
                                     const double validRangeMax) :
    mValidRangeMin(validRangeMin),
    mValidRangeMax(validRangeMax)
{
    /* Check the scan range limits */
    Assert(this->mValidRangeMin < this->mValidRangeMax);
}

/* Remove outliers from the given scan data */
void ScanOutlierFilter::RemoveOutliers(
    Sensor::ScanDataPtr<double>& scanData) const
{
    const std::size_t numOfScans = scanData->NumOfScans();

    /* Compute the maximum and minimum scan ranges */
    const double minRange = std::max(
        scanData->MinRange(), this->mValidRangeMin);
    const double maxRange = std::min(
        scanData->MaxRange(), this->mValidRangeMax);

    /* Zip the scan ranges and scan angles */
    std::vector<std::pair<double, double>> scanRangesAndAngles;
    scanRangesAndAngles.reserve(numOfScans);

    for (std::size_t i = 0; i < numOfScans; ++i)
        scanRangesAndAngles.emplace_back(
            scanData->RangeAt(i), scanData->AngleAt(i));

    /* Remove the outliers */
    const auto removeIt = std::remove_if(
        scanRangesAndAngles.begin(), scanRangesAndAngles.end(),
        [this](const std::pair<double, double>& rangeAnglePair) {
            return rangeAnglePair.first <= this->mValidRangeMin ||
                   rangeAnglePair.first >= this->mValidRangeMax; });
    scanRangesAndAngles.erase(removeIt, scanRangesAndAngles.end());

    /* Compute the minimum and maximum angles */
    const auto [minAngleIt, maxAngleIt] = std::minmax_element(
        scanRangesAndAngles.cbegin(), scanRangesAndAngles.cend(),
        [](const std::pair<double, double>& lhs,
           const std::pair<double, double>& rhs) {
            return lhs.second < rhs.second; });
    const double minAngle = minAngleIt->second;
    const double maxAngle = maxAngleIt->second;

    /* Overwrite the scan ranges and angles */
    const std::size_t numOfFilteredScans = scanRangesAndAngles.size();
    scanData->Ranges().resize(numOfFilteredScans);
    scanData->Angles().resize(numOfFilteredScans);

    for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
        scanData->Ranges().at(i) = scanRangesAndAngles.at(i).first;
        scanData->Angles().at(i) = scanRangesAndAngles.at(i).second;
    }

    /* Overwrite the minimum and maximum scan ranges */
    scanData->SetMinRange(minRange);
    scanData->SetMaxRange(maxRange);
    /* Overwrite the minimum and maximum scan angles */
    scanData->SetMinAngle(minAngle);
    scanData->SetMaxAngle(maxAngle);
}

} /* namespace MyLidarGraphSlam */
} /* namespace Mapping */
