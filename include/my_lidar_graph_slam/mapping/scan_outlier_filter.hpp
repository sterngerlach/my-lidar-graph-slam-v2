
/* scan_outlier_filter.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SCAN_OUTLIER_FILTER_HPP
#define MY_LIDAR_GRAPH_SLAM_SCAN_OUTLIER_FILTER_HPP

#include <algorithm>
#include <memory>
#include <vector>

#include "my_lidar_graph_slam/sensor/sensor_data.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class ScanOutlierFilter;
using ScanOutlierFilterPtr = std::shared_ptr<ScanOutlierFilter>;

class ScanOutlierFilter final
{
public:
    /* Constructor */
    ScanOutlierFilter(const double validRangeMin,
                      const double validRangeMax);
    /* Destructor */
    ~ScanOutlierFilter() = default;

    /* Remove outliers from the given scan data */
    void RemoveOutliers(Sensor::ScanDataPtr<double>& scanData) const;

private:
    /* Minimum scan range considered valid */
    const double mValidRangeMin;
    /* Maximum scan range considered valid */
    const double mValidRangeMax;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SCAN_OUTLIER_FILTER_HPP */
