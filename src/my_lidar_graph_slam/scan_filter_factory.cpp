
/* scan_filter_factory.cpp */

#include "my_lidar_graph_slam/scan_filter_factory.hpp"

namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {

/* Create a scan outlier filter */
std::shared_ptr<Mapping::ScanOutlierFilter> CreateScanOutlierFilter(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a scan outlier filter */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double validRangeMin = config.get<double>("ValidRangeMin");
    const double validRangeMax = config.get<double>("ValidRangeMax");

    /* Create a scan outlier filter */
    auto pScanOutlierFilter = std::make_shared<Mapping::ScanOutlierFilter>(
        validRangeMin, validRangeMax);

    return pScanOutlierFilter;
}

/* Create a scan accumulator */
std::shared_ptr<Mapping::ScanAccumulator> CreateScanAccumulator(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for scan accumulator */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const std::size_t numOfAccumulatedScans =
        config.get<std::size_t>("NumOfAccumulatedScans");

    /* Construct scan accumulator object */
    auto pScanAccumulator = std::make_shared<Mapping::ScanAccumulator>(
        numOfAccumulatedScans);

    return pScanAccumulator;
}

/* Create a scan interpolator */
std::shared_ptr<Mapping::ScanInterpolator> CreateScanInterpolator(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for scan interpolator */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double distScans = config.get<double>("DistScans");
    const double distThresholdEmpty = config.get<double>("DistThresholdEmpty");

    /* Construct scan interpolator object */
    auto pScanInterpolator = std::make_shared<Mapping::ScanInterpolator>(
        distScans, distThresholdEmpty);

    return pScanInterpolator;
}

} /* namespace MyLidarGraphSlam */
