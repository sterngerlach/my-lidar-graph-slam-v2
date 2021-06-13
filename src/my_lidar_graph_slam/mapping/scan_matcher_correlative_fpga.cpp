
/* scan_matcher_correlative_fpga.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_correlative_fpga.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

using namespace Hardware;

/* Static assertions for safety */
static_assert(std::is_same<GridMap::GridType::ValueType, std::uint16_t>::value,
              "Grid map should store occupancy probability values as "
              "discretized 16-bit unsigned integers");
static_assert(GridMap::GridType::UnknownValue == 0,
              "Unknown occupancy probability should be 0");

/* Constructor */
ScanMatcherFPGAMetrics::ScanMatcherFPGAMetrics(
    const std::string& scanMatcherName) :
    mInputSetupTime(nullptr),
    mSetupIPTime(nullptr),
    mScanSendTime(nullptr),
    mMapSendTime(nullptr),
    mOptimizationTime(nullptr),
    mWaitIPTime(nullptr),
    mScanMatchingTime(nullptr),
    mDiffTranslation(nullptr),
    mDiffRotation(nullptr),
    mWinSizeX(nullptr),
    mWinSizeY(nullptr),
    mWinSizeTheta(nullptr),
    mStepSizeX(nullptr),
    mStepSizeY(nullptr),
    mStepSizeTheta(nullptr),
    mMapSizeX(nullptr),
    mMapSizeY(nullptr),
    mMapChunks(nullptr),
    mScanTransferSkip(nullptr),
    mMapTransferSkip(nullptr),
    mScoreValue(nullptr),
    mCostValue(nullptr),
    mNumOfTransferredScans(nullptr)
{
    /* Retrieve the metrics manager instance */
    auto* const pMetricManager = Metric::MetricManager::Instance();

    /* Register the counter metrics */
    this->mMapChunks = pMetricManager->AddCounter(
        scanMatcherName + ".MapChunks");
    this->mScanTransferSkip = pMetricManager->AddCounter(
        scanMatcherName + ".ScanTransferSkip");
    this->mMapTransferSkip = pMetricManager->AddCounter(
        scanMatcherName + ".MapTransferSkip");

    /* Register the distribution metrics */
    this->mInputSetupTime = pMetricManager->AddDistribution(
        scanMatcherName + ".InputSetupTime");
    this->mSetupIPTime = pMetricManager->AddDistribution(
        scanMatcherName + ".SetupIPTime");
    this->mScanSendTime = pMetricManager->AddDistribution(
        scanMatcherName + ".ScanSendTime");
    this->mMapSendTime = pMetricManager->AddDistribution(
        scanMatcherName + ".MapSendTime");
    this->mOptimizationTime = pMetricManager->AddDistribution(
        scanMatcherName + ".OptimizationTime");
    this->mWaitIPTime = pMetricManager->AddDistribution(
        scanMatcherName + ".WaitIPTime");
    this->mScanMatchingTime = pMetricManager->AddDistribution(
        scanMatcherName + ".ScanMatchingTime");
    this->mDiffTranslation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffTranslation");
    this->mDiffRotation = pMetricManager->AddDistribution(
        scanMatcherName + ".DiffRotation");
    this->mWinSizeX = pMetricManager->AddDistribution(
        scanMatcherName + ".WinSizeX");
    this->mWinSizeY = pMetricManager->AddDistribution(
        scanMatcherName + ".WinSizeY");
    this->mWinSizeTheta = pMetricManager->AddDistribution(
        scanMatcherName + ".WinSizeTheta");
    this->mStepSizeX = pMetricManager->AddDistribution(
        scanMatcherName + ".StepSizeX");
    this->mStepSizeY = pMetricManager->AddDistribution(
        scanMatcherName + ".StepSizeY");
    this->mStepSizeTheta = pMetricManager->AddDistribution(
        scanMatcherName + ".StepSizeTheta");

    /* Register the histogram metrics */
    const Metric::BucketBoundaries mapSizeBuckets =
        Metric::Histogram::CreateFixedWidthBoundaries(0.0, 400.0, 20.0);
    this->mMapSizeX = pMetricManager->AddHistogram(
        scanMatcherName + ".MapSizeX", mapSizeBuckets);
    this->mMapSizeY = pMetricManager->AddHistogram(
        scanMatcherName + ".MapSizeY", mapSizeBuckets);

    /* Register the value sequence metrics */
    this->mScoreValue = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".ScoreValue");
    this->mCostValue = pMetricManager->AddValueSequenceFloat(
        scanMatcherName + ".CostValue");
    this->mNumOfTransferredScans = pMetricManager->AddValueSequenceInt(
        scanMatcherName + ".NumOfTransferredScans");
}

/* Constructor */
ScanMatcherCorrelativeFPGA::ScanMatcherCorrelativeFPGA(
    const std::string& scanMatcherName,
    const CostFuncPtr& costFunc,
    const double rangeX,
    const double rangeY,
    const double rangeTheta,
    ScanMatcherHardwareConfig&& scanMatcherConfig,
    AxiDmaConfig&& axiDmaConfig) :
    ScanMatcher(scanMatcherName),
    mCostFunc(costFunc),
    mRangeX(rangeX),
    mRangeY(rangeY),
    mRangeTheta(rangeTheta),
    mLastMapId(LocalMapId::Invalid),
    mLastMapSize(0, 0),
    mLastMapMinPos(0.0, 0.0),
    mConfig(std::move(scanMatcherConfig)),
    mAxiDmaConfig(std::move(axiDmaConfig)),
    mMetrics(scanMatcherName)
{
    /* Check the information of the scan matcher IP core */
    XAssert(this->mConfig.mMaxNumOfScans > 0,
            "Maximum number of the scan points should be greater than 0");
    XAssert(this->mConfig.mMapResolution > 0.0,
            "Grid map resolution should be greater than 0");
    XAssert(this->mConfig.mMaxMapSizeX > 0,
            "Maximum width of the grid map (in the number of grid cells) "
            "should be greater than 0");
    XAssert(this->mConfig.mMaxMapSizeY > 0,
            "Maximum height of the grid map (in the number of grid cells) "
            "should be greater than 0");
    XAssert(this->mConfig.mLowResolution > 1,
            "Resolution of the coarse grid map (in the number of grid cells) "
            "should be greater than 0");
    XAssert(this->mConfig.mMapBitWidth > 0 &&
            this->mConfig.mMapBitWidth <= 8,
            "Bit width of the discretized occupancy probability "
            "should be in the range between 1 and 8");
    XAssert(this->mConfig.mMapChunkWidth == 8,
            "Width of the grid map chunk (consecutive grid map cells) "
            "should be 8");

    /* Initialize the scan matcher IP core */
    this->mControlRegisters = std::make_unique<MemoryMappedIO>(
        this->mConfig.mAxiLiteSBaseAddress,
        this->mConfig.mAxiLiteSAddressRange);

    /* Initialize the AXI DMA IP core */
    SharedMemoryMappedIO axiDmaRegisters {
        this->mAxiDmaConfig.mBaseAddress,
        this->mAxiDmaConfig.mAddressRange };
    this->mAxiDma = std::make_unique<AxiSimpleDMA>(axiDmaRegisters);

    /* Initialize the CMA memory to store the input data */
    this->InitializeCMAMemoryInput();
    /* Initialize the CMA memory to store the output data */
    this->InitializeCMAMemoryOutput();

    /* Reset and halt the AXI DMA IP core */
    this->mAxiDma->SendChannel().Reset();
    this->mAxiDma->RecvChannel().Reset();
    this->mAxiDma->SendChannel().Stop();
    this->mAxiDma->RecvChannel().Stop();

    /* Start the AXI DMA transfer */
    this->mAxiDma->SendChannel().Start();
    this->mAxiDma->RecvChannel().Start();
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherCorrelativeFPGA::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Create an invalid grid map Id */
    const LocalMapId invalidMapId { LocalMapId::Invalid };

    /* Optimize the robot pose by scan matching
     * Pass the minimum possible value as a score threshold to
     * search the entire window */
    return this->OptimizePose(
        queryInfo.mGridMap, invalidMapId, queryInfo.mGridMapCenterPos,
        queryInfo.mScanData, queryInfo.mMapLocalInitialPose, 0.0, 0.0);
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherCorrelativeFPGA::OptimizePose(
    const GridMap& gridMap,
    const LocalMapId gridMapId,
    const Point2D<double>& gridMapCenterPos,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalInitialPose,
    const double normalizedScoreThreshold,
    const double /* knownRateThreshold */)
{
    /* Create the timer */
    Metric::Timer outerTimer;
    Metric::Timer timer;

    /* Find the best pose from the search window */
    const RobotPose2D<double> mapLocalSensorPose =
        Compound(mapLocalInitialPose, scanData->RelativeSensorPose());

    /* Determine the number of the scan points to be transferred */
    const int maxNumOfScans = this->mConfig.mMaxNumOfScans;
    const int numOfScans = static_cast<int>(scanData->NumOfScans());
    const int numOfScansTransferred = std::min(maxNumOfScans, numOfScans);

    /* Determine the maximum scan range considered valid */
    const float scanRangeMax = static_cast<float>(scanData->MaxRange());
    /* Compute the score threshold that is not normalized */
    const double scoreThreshold =
        normalizedScoreThreshold * numOfScansTransferred;
    const int quantizedScoreThreshold = static_cast<int>(
        ((1 << this->mConfig.mMapBitWidth) - 1) * scoreThreshold);

    /* Determine the search step */
    double stepX;
    double stepY;
    double stepTheta;
    this->ComputeSearchStep(gridMap, scanData, stepX, stepY, stepTheta);

    /* Determine the radius of the search window */
    const int originalWinX = static_cast<int>(
        std::ceil(0.5 * this->mRangeX / stepX));
    const int originalWinY = static_cast<int>(
        std::ceil(0.5 * this->mRangeY / stepY));
    const int originalWinTheta = static_cast<int>(
        std::ceil(0.5 * this->mRangeTheta / stepTheta));

    const int winX = std::clamp(originalWinX, 1,
                                this->mConfig.mMaxMapSizeX / 2);
    const int winY = std::clamp(originalWinY, 1,
                                this->mConfig.mMaxMapSizeY / 2);
    const int winTheta = std::max(1, originalWinTheta);

    /* Compute the minimum possible sensor pose */
    const RobotPose2D<double> minSensorPose {
        mapLocalSensorPose.mX - stepX * winX,
        mapLocalSensorPose.mY - stepY * winY,
        mapLocalSensorPose.mTheta - stepTheta * winTheta };

    /* Compute the bounding box of the grid map */
    const BoundingBox<int> boundingBox =
        this->ComputeBoundingBox(gridMap, gridMapCenterPos);
    /* Compute the size of the cropped grid map */
    const Point2D<int> gridMapSize {
        boundingBox.Width(), boundingBox.Height() };
    /* Compute the minimum position of the cropped grid map */
    const Point2D<double> gridMapMinPos =
        gridMap.IndexToPosition(boundingBox.mMin.mY, boundingBox.mMin.mX);

    /* Check if the grid map should be transferred */
    /* If the given grid map `gridMap` with an Id `gridMapId` is
     * already stored on the scan matcher IP core, `this->mLastMapId` is
     * set as `gridMapId` */
    /* If the given grid map `gridMap` does not have an Id, that is,
     * `gridMapId` is the predefined invalid value `LocalMapId::Invalid`,
     * then the grid map should be transferred */
    const bool mapTransferred = !(gridMapId == this->mLastMapId &&
                                  gridMapId.mId != LocalMapId::Invalid);

    /* Set the scan matching parameters through AXI4-Lite slave interface */
    /* Do not use the above grid map parameters `gridMapSize` and
     * `gridMapMinPos` when the grid map is already stored on the device */
    const Point2D<int>& actualGridMapSize =
        mapTransferred ? gridMapSize : this->mLastMapSize;
    const Point2D<double>& actualGridMapMinPos =
        mapTransferred ? gridMapMinPos : this->mLastMapMinPos;

    /* Update the metrics and restart the timer */
    this->mMetrics.mInputSetupTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Set the parameter registers */
    this->SetParameterRegisters(
        numOfScansTransferred, scanRangeMax, quantizedScoreThreshold,
        minSensorPose, actualGridMapSize, actualGridMapMinPos,
        winX * 2, winY * 2, winTheta * 2, stepX, stepY, stepTheta);
    /* Start the scan matcher IP core */
    this->StartIPCore();

    /* Update the metrics and restart the timer */
    this->mMetrics.mSetupIPTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Send the scan data */
    this->SendScanData(scanData);
    /* Update the metrics and restart the timer */
    this->mMetrics.mScanSendTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Send the grid map */
    this->SendGridMap(gridMap, mapTransferred, boundingBox.mMin, gridMapSize);
    /* Update the metrics and restart the timer */
    this->mMetrics.mMapSendTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Update the Id of the last grid map that is transferred */
    this->mLastMapId = gridMapId;
    /* Update the parameters of the last grid map */
    this->mLastMapSize = actualGridMapSize;
    this->mLastMapMinPos = actualGridMapMinPos;

    /* Receive the result */
    int scoreMax;
    int bestX;
    int bestY;
    int bestTheta;
    this->ReceiveResult(scoreMax, bestX, bestY, bestTheta);
    /* Update the metrics and restart the timer */
    this->mMetrics.mOptimizationTime->Observe(timer.ElapsedMicro());
    timer.Start();

    /* Wait for the scan matcher IP core */
    this->WaitIPCore();
    /* Update the metrics and stop the timer */
    this->mMetrics.mWaitIPTime->Observe(timer.ElapsedMicro());
    timer.Stop();

    /* The appropriate solution is found if the maximum score is
     * larger than (not larger than or equal to) the score threshold */
    const bool poseFound = scoreMax > quantizedScoreThreshold;
    const double normalizedScore =
        static_cast<double>(scoreMax) /
        static_cast<double>(numOfScansTransferred) /
        static_cast<double>((1 << this->mConfig.mMapBitWidth) - 1);
    /* Compute the best sensor pose */
    const RobotPose2D<double> bestSensorPose {
        minSensorPose.mX + bestX * stepX,
        minSensorPose.mY + bestY * stepY,
        minSensorPose.mTheta + bestTheta * stepTheta };

    /* Evaluate the cost value */
    const double costValue = this->mCostFunc->Cost(
        gridMap, scanData, bestSensorPose);
    /* Compute the normalized cost value */
    const double normalizedCost = costValue / scanData->NumOfScans();

    /* Compute the estimated robot pose in a map-local coordinate frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());
    /* Compute the pose covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Update the lots of metrics */
    this->mMetrics.mScanMatchingTime->Observe(outerTimer.ElapsedMicro());
    this->mMetrics.mDiffTranslation->Observe(
        Distance(mapLocalInitialPose, estimatedPose));
    this->mMetrics.mDiffRotation->Observe(
        std::abs(mapLocalInitialPose.mTheta - estimatedPose.mTheta));
    this->mMetrics.mWinSizeX->Observe(winX);
    this->mMetrics.mWinSizeY->Observe(winY);
    this->mMetrics.mWinSizeTheta->Observe(winTheta);
    this->mMetrics.mStepSizeX->Observe(stepX);
    this->mMetrics.mStepSizeY->Observe(stepY);
    this->mMetrics.mStepSizeTheta->Observe(stepTheta);
    this->mMetrics.mMapSizeX->Observe(actualGridMapSize.mX);
    this->mMetrics.mMapSizeY->Observe(actualGridMapSize.mY);
    this->mMetrics.mScoreValue->Observe(normalizedScore);
    this->mMetrics.mCostValue->Observe(normalizedCost);
    this->mMetrics.mNumOfTransferredScans->Observe(numOfScansTransferred);

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a map-local frame */
    return ScanMatchingSummary {
        poseFound, normalizedCost, mapLocalInitialPose,
        estimatedPose, estimatedCovariance };
}

/* Compute the search step */
void ScanMatcherCorrelativeFPGA::ComputeSearchStep(
    const GridMapInterface& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    double& stepX,
    double& stepY,
    double& stepTheta) const
{
    /* Determine the search step */
    const double mapResolution = gridMap.Resolution();
    const auto maxRangeIt = std::max_element(
        scanData->Ranges().cbegin(), scanData->Ranges().cend());
    const double maxRange = *maxRangeIt;
    const double theta = mapResolution / maxRange;

    stepX = mapResolution;
    stepY = mapResolution;
    stepTheta = std::acos(1.0 - 0.5 * theta * theta);

    return;
}

/* Initialize the CMA memory for the input data */
void ScanMatcherCorrelativeFPGA::InitializeCMAMemoryInput()
{
    /* Compute the number of bytes required for transferring scan data
     * Each 64-bit element packs the scan range (32-bit floating point) and
     * the scan angle (32-bit floating point) */
    /* Add 1 to transfer the flag indicating that scan data is transferred */
    const std::size_t scanDataLength = this->mConfig.mMaxNumOfScans + 1;

    /* Compute the number of bytes required for transferring grid map
     * Each 64-bit element (grid map chunk) packs 8 occupancy probability
     * values for 8 consecutive grid cells, which are quantized to
     * 8-bit unsigned integers when transferred */
    const std::size_t numOfMapChunksPerRow =
        (this->mConfig.mMaxMapSizeX + this->mConfig.mMapChunkWidth - 1) /
        this->mConfig.mMapChunkWidth;
    /* Add 1 to transfer the flag that indicates whether the grid map
     * should be transferred (grid map is cached on the BRAM to reduce
     * the expensive data transfer cost) */
    const std::size_t gridMapLength =
        this->mConfig.mMaxMapSizeY * numOfMapChunksPerRow + 1;

    /* The number of bytes required to store the input data */
    const std::size_t lengthInBytes =
        std::max(scanDataLength, gridMapLength) * sizeof(std::uint64_t);
    /* Allocate the CMA memory for the input data */
    this->mInputData.Initialize(static_cast<std::uint32_t>(lengthInBytes));
}

/* Initialize the CMA memory for the output data */
void ScanMatcherCorrelativeFPGA::InitializeCMAMemoryOutput()
{
    /* The number of bytes required to store the output data
     * The first 64-bit element contains the maximum score and the
     * discretized X-coordinate of the final sensor pose, and the
     * second 64-bit element contains the discretized Y-coordinate and
     * the discretized rotation angle of the final sensor pose */
    const std::size_t lengthInBytes = 2 * sizeof(std::uint64_t);
    /* Allocate the CMA memory for the output data */
    this->mOutputData.Initialize(static_cast<std::uint32_t>(lengthInBytes));
}

/* Set the scan matching parameters through AXI4-Lite slave interface */
void ScanMatcherCorrelativeFPGA::SetParameterRegisters(
    const int numOfScans,
    const double scanRangeMax,
    const int scoreThreshold,
    const RobotPose2D<double>& minSensorPose,
    const Point2D<int>& gridMapSize,
    const Point2D<double>& gridMapMinPos,
    const int winX, const int winY, const int winTheta,
    const double stepX, const double stepY, const double stepTheta)
{
    const auto doubleToU32 = [](const double x) {
        return FloatToUInt32(static_cast<double>(x)); };

    /* Set the actual number of the scan points */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSNumOfScans, Int32ToUInt32(numOfScans));
    /* Set the maximum scan range considered valid */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSScanRangeMax, doubleToU32(scanRangeMax));
    /* Set the score threshold (for loop detection) */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSScoreThreshold, Int32ToUInt32(scoreThreshold));

    /* Set the minimum possible sensor pose */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSPoseX, doubleToU32(minSensorPose.mX));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSPoseY, doubleToU32(minSensorPose.mY));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSPoseTheta, doubleToU32(minSensorPose.mTheta));

    /* Set the actual size of the cropped grid map */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSMapSizeX, Int32ToUInt32(gridMapSize.mX));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSMapSizeY, Int32ToUInt32(gridMapSize.mY));
    /* Set the minimum coordinate of the cropped grid map */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSMapMinX, doubleToU32(gridMapMinPos.mX));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSMapMinY, doubleToU32(gridMapMinPos.mY));

    /* Set the size of the search window */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSWinX, Int32ToUInt32(winX));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSWinY, Int32ToUInt32(winY));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSWinTheta, Int32ToUInt32(winTheta));
    /* Set the search step */
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSStepX, doubleToU32(stepX));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSStepY, doubleToU32(stepY));
    this->mControlRegisters->Write(
        this->mConfig.mAxiLiteSStepTheta, doubleToU32(stepTheta));
}

/* Compute the bounding box of the grid map */
BoundingBox<int> ScanMatcherCorrelativeFPGA::ComputeBoundingBox(
    const GridMap& gridMap,
    const Point2D<double>& centerPos) const
{
    /* Make sure that the block size is the multiple of 4 so that the
     * size of the grid map `gridMap.Cols()` and `gridMap.Rows()` are
     * also the multiples of 4 */
    Assert(gridMap.BlockSize() % 4 == 0);

    const int mapColsMax = this->mConfig.mMaxMapSizeX;
    const int mapRowsMax = this->mConfig.mMaxMapSizeY;

    /* Compute the center position of the cropped grid map */
    /* Use std::clamp() here since the below `centerIdx` could be
     * out-of-bounds (the robot pose is outside of the grid map) */
    const Point2D<int> desiredCenterIdx =
        gridMap.PositionToIndex(centerPos.mX, centerPos.mY);
    const Point2D<int> possibleIdxMin {
        std::clamp(desiredCenterIdx.mX - mapColsMax, 0, gridMap.Cols() - 1),
        std::clamp(desiredCenterIdx.mY - mapRowsMax, 0, gridMap.Rows() - 1) };
    const Point2D<int> possibleIdxMax {
        std::clamp(desiredCenterIdx.mX + mapColsMax, 1, gridMap.Cols()),
        std::clamp(desiredCenterIdx.mY + mapRowsMax, 1, gridMap.Rows()) };
    const Point2D<int> centerIdx {
        (possibleIdxMax.mX + possibleIdxMin.mX) / 2,
        (possibleIdxMax.mY + possibleIdxMin.mY) / 2 };

    /* Crop the grid map around the center position */
    /* Align by 4 (i.e. 4-bytes) to prevent the unaligned accesses */
    const auto alignBy4 = [](const int x) { return (x >> 2) << 2; };
    const int colMin = alignBy4(centerIdx.mX - mapColsMax / 2);
    const int rowMin = alignBy4(centerIdx.mY - mapRowsMax / 2);
    const int colMax = alignBy4(centerIdx.mX + mapColsMax / 2);
    const int rowMax = alignBy4(centerIdx.mY + mapRowsMax / 2);

    const Point2D<int> idxMin { std::max(colMin, 0),
                                std::max(rowMin, 0) };
    const Point2D<int> idxMax { std::min(colMax, gridMap.Cols()),
                                std::min(rowMax, gridMap.Rows()) };

    /* Return the bounding box */
    return BoundingBox<int> { idxMin, idxMax };
}

/* Send the scan data through AXI DMA */
void ScanMatcherCorrelativeFPGA::SendScanData(
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Retrieve the pointer to the CMA memory */
    auto* pInput = this->mInputData.Ptr<volatile std::uint64_t>();

    /* Write the flag to indicate that the scan data is transferred */
    *pInput++ = static_cast<std::uint64_t>(1);

    const std::size_t maxNumOfScans =
        static_cast<std::size_t>(this->mConfig.mMaxNumOfScans);
    const std::size_t numOfScans = scanData->NumOfScans();

    /* Subsample the scan points if the number of the scan points
     * exceeds the maximum that the scan matcher IP core can handle */
    if (maxNumOfScans < numOfScans) {
        /* Interval is greater than 1 and thus the scan points are sampled */
        const double idxInterval =
            static_cast<double>(numOfScans - 1) /
            static_cast<double>(maxNumOfScans - 1);
        double subsampledIdx = 0.0;

        for (std::size_t i = 0; i < maxNumOfScans; ++i) {
            /* Write the subsampled scan point */
            const std::size_t scanIdx =
                static_cast<std::size_t>(subsampledIdx);
            const float scanRange = static_cast<float>(
                scanData->RangeAt(scanIdx));
            const float scanAngle = static_cast<float>(
                scanData->AngleAt(scanIdx));
            *pInput++ = PackFloat(scanRange, scanAngle);
            /* Advance the index of the subsampled scan point */
            subsampledIdx += idxInterval;
        }
    } else {
        /* Write all the scan points */
        for (std::size_t i = 0; i < numOfScans; ++i) {
            /* Write the scan range and angle */
            const float scanRange = static_cast<float>(scanData->RangeAt(i));
            const float scanAngle = static_cast<float>(scanData->AngleAt(i));
            *pInput++ = PackFloat(scanRange, scanAngle);
        }
    }

    /* Transfer the scan points using the AXI DMA IP core */
    const std::size_t numOfScansTransferred =
        std::min(maxNumOfScans, numOfScans);
    const std::size_t transferLengthInBytes =
        (numOfScansTransferred + 1) * sizeof(std::uint64_t);
    this->mAxiDma->SendChannel().Transfer(
        transferLengthInBytes, this->mInputData.PhysicalAddress());

    /* Wait for the data transfer to complete by polling the status register
     * of the AXI DMA IP core */
    if (this->mConfig.mWaitForDmaTransfer)
        this->mAxiDma->SendChannel().Wait();
}

/* Send the grid map through AXI DMA */
void ScanMatcherCorrelativeFPGA::SendGridMap(
    const GridMap& gridMap,
    const bool gridMapTransferred,
    const Point2D<int>& gridMapMinIdx,
    const Point2D<int>& gridMapSize)
{
    /* Retrieve the pointer to the CMA memory */
    auto* pInput = this->mInputData.Ptr<volatile std::uint64_t>();

    /* Do not transfer the grid map if it is redundant */
    if (!gridMapTransferred) {
        /* Write the flag to indicate that the grid map is not transferred */
        *pInput++ = static_cast<std::uint64_t>(0);

        /* Transfer the flag only using the AXI DMA IP core */
        this->mAxiDma->SendChannel().Transfer(
            sizeof(std::uint64_t), this->mInputData.PhysicalAddress());

        /* Wait for the data transfer to complete if necessary */
        if (this->mConfig.mWaitForDmaTransfer)
            this->mAxiDma->SendChannel().Wait();

        /* Update the metrics */
        this->mMetrics.mMapChunks->Increment(0);
        this->mMetrics.mMapTransferSkip->Increment();

        return;
    }

    /* Write the flag to indicate that the grid map is transferred */
    *pInput++ = static_cast<std::uint64_t>(1);

    /* Write the cropped grid map */
    /* The below `chunkWidth` denotes the number of grid values in the
     * 64-bit data (DMA controller transfers the 64-bit data per clock) */
    const int chunkWidth = this->mConfig.mMapChunkWidth;
    const int chunkCols = (gridMapSize.mX + chunkWidth - 1) / chunkWidth;
    const BoundingBox<int> boundingBox {
        gridMapMinIdx.mX, gridMapMinIdx.mY,
        gridMapMinIdx.mX + chunkCols * chunkWidth,
        gridMapMinIdx.mY + gridMapSize.mY };

    auto* pBuffer = this->mInputData.Ptr<std::uint32_t>() +
                    sizeof(std::uint64_t) / sizeof(std::uint32_t);
    gridMap.CopyValuesU8x4(pBuffer, boundingBox);

    /* Transfer the grid map using the AXI DMA IP core */
    const std::size_t transferLengthInBytes =
        (gridMapSize.mY * chunkCols + 1) * sizeof(std::uint64_t);
    this->mAxiDma->SendChannel().Transfer(
        transferLengthInBytes, this->mInputData.PhysicalAddress());

    /* Wait for the data transfer to complete by polling the status register
     * of the AXI DMA IP core */
    if (this->mConfig.mWaitForDmaTransfer)
        this->mAxiDma->SendChannel().Wait();

    /* Update the metrics */
    const int numOfChunks = gridMapSize.mY * chunkCols;
    this->mMetrics.mMapChunks->Increment(numOfChunks);
    this->mMetrics.mMapTransferSkip->Increment(0);
}

/* Receive the result through AXI DMA */
void ScanMatcherCorrelativeFPGA::ReceiveResult(
    int& scoreMax, int& bestX, int& bestY, int& bestTheta)
{
    /* Receive the result using the AXI DMA IP core */
    const std::size_t receiveLengthInBytes = 2 * sizeof(std::uint64_t);
    this->mAxiDma->RecvChannel().Transfer(
        receiveLengthInBytes, this->mOutputData.PhysicalAddress());

    /* Wait for the data transfer to complete by polling the status register
     * of the AXI DMA IP core */
    if (this->mConfig.mWaitForDmaTransfer)
        this->mAxiDma->RecvChannel().Wait();

    /* Retrieve the pointer to the CMA memory */
    auto* pOutput = this->mOutputData.Ptr<volatile std::uint64_t>();

    /* Read the maximum score and the best X-coordinate */
    UnpackInt32(*pOutput++, scoreMax, bestX);
    UnpackInt32(*pOutput++, bestY, bestTheta);
}

/* Start the scan matcher IP core */
void ScanMatcherCorrelativeFPGA::StartIPCore()
{
    /* Set the control register of the scan matcher IP core */
    this->WriteCtrlReg(ToUnderlying(AxiLiteSApCtrl::Start));

    /* Wait until the scan matcher IP core starts by polling the
     * control registers of the AXI4-Lite slave interface */
    if (this->mConfig.mWaitForCtrlReg)
        while (!(this->ReadCtrlReg() & ToUnderlying(AxiLiteSApCtrl::Start)));
}

/* Wait until the scan matcher IP core is in idle state */
void ScanMatcherCorrelativeFPGA::WaitIPCore()
{
    /* Wait until the scan matcher IP core is in idle state by polling the
     * control registers of the AXI4-Lite slave interface */
    if (this->mConfig.mWaitForCtrlReg)
        while (!(this->ReadCtrlReg() & ToUnderlying(AxiLiteSApCtrl::Idle)));
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
