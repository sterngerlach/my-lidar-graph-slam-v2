
/* scan_interpolator.cpp */

#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Interpolate scan data */
void ScanInterpolator::Interpolate(
    Sensor::ScanDataPtr<double>& scanData) const
{
    /* Check the given scan data */
    Assert(scanData->NumOfScans() > 0);
    Assert(scanData->NumOfScans() == scanData->Ranges().size());
    Assert(scanData->Ranges().size() == scanData->Angles().size());

    /* Interpolate scan data so that the distance between two scan points
     * are equalized */
    const std::vector<double>& scanRanges = scanData->Ranges();
    const std::vector<double>& scanAngles = scanData->Angles();

    /* Convert polar coordinate to cartesian coordinate */
    std::vector<Point2D<double>> scanPoints;
    scanPoints.reserve(scanData->NumOfScans());

    for (std::size_t i = 0; i < scanData->NumOfScans(); ++i)
        scanPoints.emplace_back(ToCartesianCoordinate(
            scanRanges.at(i), scanAngles.at(i)));

    /* Insert the first scan point */
    std::vector<Point2D<double>> interpolatedScanPoints;
    interpolatedScanPoints.push_back(scanPoints.at(0));

    Point2D<double> prevPoint = scanPoints.at(0);
    double accumDist = 0.0;

    /* Interpolate scan data */
    for (std::size_t i = 1; i < scanRanges.size(); ++i) {
        const Point2D<double>& point = scanPoints.at(i);
        const double dist = Distance(prevPoint, point);

        if (accumDist + dist < this->mDistScans) {
            /* Do not interpolate the scan point
             * adjacent scan points are too close */
            accumDist += dist;
            prevPoint = point;
        } else if (accumDist + dist >= this->mDistThresholdEmpty) {
            /* The space between two adjacent scan points are considered empty
             * thus do not perform interpolation */
            interpolatedScanPoints.push_back(point);
            prevPoint = point;
            accumDist = 0.0;
        } else {
            /* Interpolate scan points */
            const double ratio = (this->mDistScans - accumDist) / dist;
            const double scanPointX =
                (point.mX - prevPoint.mX) * ratio + prevPoint.mX;
            const double scanPointY =
                (point.mY - prevPoint.mY) * ratio + prevPoint.mY;
            const Point2D<double> scanPoint { scanPointX, scanPointY };
            interpolatedScanPoints.push_back(scanPoint);
            prevPoint = scanPoint;
            accumDist = 0.0;
            /* Process the current scan point again */
            --i;
        }
    }

    /* Overwrite the scan ranges and angles */
    const std::size_t numOfInterpolatedScans = interpolatedScanPoints.size();
    scanData->Ranges().resize(numOfInterpolatedScans);
    scanData->Angles().resize(numOfInterpolatedScans);

    for (std::size_t i = 0; i < numOfInterpolatedScans; ++i) {
        /* Convert cartesian coordinate to polar coordinate */
        const auto [scanRange, scanAngle] =
            ToPolarCoordinate(interpolatedScanPoints.at(i));
        scanData->Ranges().at(i) = scanRange;
        scanData->Angles().at(i) = scanAngle;
    }

    /* Overwrite the minimum and maximum scan ranges */
    const auto [minRangeIt, maxRangeIt] = std::minmax_element(
        scanData->Ranges().cbegin(), scanData->Ranges().cend());
    scanData->SetMinRange(*minRangeIt);
    scanData->SetMaxRange(*maxRangeIt);

    /* Overwrite the minimum and maximum scan ranges */
    const auto [minAngleIt, maxAngleIt] = std::minmax_element(
        scanData->Angles().cbegin(), scanData->Angles().cend());
    scanData->SetMinAngle(*minAngleIt);
    scanData->SetMaxAngle(*maxAngleIt);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
