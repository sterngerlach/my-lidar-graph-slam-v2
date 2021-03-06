
/* scan_matcher_correlative_fpga.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_CORRELATIVE_FPGA_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_CORRELATIVE_FPGA_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/hw/ap_ctrl.hpp"
#include "my_lidar_graph_slam/hw/axi_simple_dma.hpp"
#include "my_lidar_graph_slam/hw/cma_memory.hpp"
#include "my_lidar_graph_slam/hw/mmio.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

struct ScanMatcherFPGAMetrics
{
    /* Constructor */
    ScanMatcherFPGAMetrics(const std::string& scanMatcherName);
    /* Destructor */
    ~ScanMatcherFPGAMetrics() = default;

    /* Total processing time for setting up the input */
    Metric::ValueSequenceBase<int>*   mInputSetupTime;
    /* Total processing time for setting up the scan matcher IP core */
    Metric::ValueSequenceBase<int>*   mSetupIPTime;
    /* Total processing time for sending the scan data */
    Metric::ValueSequenceBase<int>*   mScanSendTime;
    /* Total processing time for sending the grid map data */
    Metric::ValueSequenceBase<int>*   mMapSendTime;
    /* Total processing time for the calculation on the FPGA device */
    Metric::ValueSequenceBase<int>*   mOptimizationTime;
    /* Total processing time for waiting for the scan matcher IP core */
    Metric::ValueSequenceBase<int>*   mWaitIPTime;
    /* Total processing time for the scan matching */
    Metric::ValueSequenceBase<int>*   mScanMatchingTime;
    /* Distance between the initial pose and the final pose */
    Metric::ValueSequenceBase<float>* mDiffTranslation;
    /* Absolute difference between the initial angle and the final angle */
    Metric::ValueSequenceBase<float>* mDiffRotation;
    /* Size of the search window along the X-axis */
    Metric::ValueSequenceBase<int>*   mWinSizeX;
    /* Size of the search window along the Y-axis */
    Metric::ValueSequenceBase<int>*   mWinSizeY;
    /* Size of the search window along the Theta-axis */
    Metric::ValueSequenceBase<int>*   mWinSizeTheta;
    /* Step size along the X-axis */
    Metric::ValueSequenceBase<float>* mStepSizeX;
    /* Step size along the Y-axis */
    Metric::ValueSequenceBase<float>* mStepSizeY;
    /* Step size along the Theta-axis */
    Metric::ValueSequenceBase<float>* mStepSizeTheta;
    /* Width of the transferred grid map (in the number of the grid cells) */
    Metric::ValueSequenceBase<int>*   mMapSizeX;
    /* Height of the transferred grid map (in the number of the grid cells) */
    Metric::ValueSequenceBase<int>*   mMapSizeY;
    /* Number of the transferred grid cell chunks in the grid map */
    Metric::ValueSequenceBase<int>*   mMapChunks;
    /* Number of the scan data transfer skips */
    Metric::ValueSequenceBase<char>*  mScanTransferSkip;
    /* Number of the grid map transfer skips */
    Metric::ValueSequenceBase<char>*  mMapTransferSkip;
    /* Normalized score value of the best solution */
    Metric::ValueSequenceBase<float>* mScoreValue;
    /* Normalized cost value of the best solution */
    Metric::ValueSequenceBase<float>* mCostValue;
    /* Number of the transferred scan points */
    Metric::ValueSequenceBase<int>*   mNumOfTransferredScans;
};

/*
 * ScanMatcherHardwareConfig struct holds the necessary information for
 * the real-time correlative scan matcher IP core, including the
 * scan matching parameters, the base address and address range of the
 * AXI4-Lite interface, the address offsets of the registers
 */
struct ScanMatcherHardwareConfig
{
    /* Maximum number of the scan points */
    int    mMaxNumOfScans;
    /* Grid map resolution (in meters) */
    double mMapResolution;
    /* Maximum width of the grid map (in the number of grid cells) */
    int    mMaxMapSizeX;
    /* Maximum height of the grid map (in the number of grid cells) */
    int    mMaxMapSizeY;
    /* Resolution of the coarse grid map (in the number of grid cells) */
    int    mLowResolution;
    /* Bit width of the occupancy probability value */
    int    mMapBitWidth;
    /* Width of the grid map chunk (consecutive grid map cells) */
    int    mMapChunkWidth;

    /* Register offsets for the AXI4-Lite slave interface */

    /* AXI4-Lite slave interface base address */
    std::uint32_t mAxiLiteSBaseAddress;
    /* AXI4-Lite slave interface address range */
    std::uint32_t mAxiLiteSAddressRange;
    /* AXI4-Lite slave interface control signal */
    std::uint32_t mAxiLiteSApCtrl;
    /* AXI4-Lite slave interface global interrupt enable register */
    std::uint32_t mAxiLiteSGIE;
    /* AXI4-Lite slave interface IP interrupt enable register */
    std::uint32_t mAxiLiteSIER;
    /* AXI4-Lite slave interface IP interrupt status register */
    std::uint32_t mAxiLiteSISR;

    /* Register offsets for the scan matcher */

    /* Register offset for the actual number of the scan points */
    std::uint32_t mAxiLiteSNumOfScans;
    /* Register offset for the maximum scan range considered valid */
    std::uint32_t mAxiLiteSScanRangeMax;
    /* Register offset for the score threshold (for loop detection) */
    std::uint32_t mAxiLiteSScoreThreshold;
    /* Register offset for the sensor pose */
    std::uint32_t mAxiLiteSPoseX;
    std::uint32_t mAxiLiteSPoseY;
    std::uint32_t mAxiLiteSPoseTheta;
    /* Register offset for the actual size of the grid map */
    std::uint32_t mAxiLiteSMapSizeX;
    std::uint32_t mAxiLiteSMapSizeY;
    /* Register offset for the minimum coordinate of the grid map */
    std::uint32_t mAxiLiteSMapMinX;
    std::uint32_t mAxiLiteSMapMinY;
    /* Register offset for the size of the search window */
    std::uint32_t mAxiLiteSWinX;
    std::uint32_t mAxiLiteSWinY;
    std::uint32_t mAxiLiteSWinTheta;
    /* Register offset for the search step */
    std::uint32_t mAxiLiteSStepX;
    std::uint32_t mAxiLiteSStepY;
    std::uint32_t mAxiLiteSStepTheta;
};

/*
 * AxiDmaConfig struct holds the necessary information for the AXI DMA IP core,
 * especially the base address and the address range
 */
struct AxiDmaConfig
{
    /* AXI DMA base address */
    std::uint32_t mBaseAddress;
    /* AXI DMA address range */
    std::uint32_t mAddressRange;
};

/*
 * ScanMatcherCorrelativeFPGA class manipulates the real-time
 * correlative-based scan matcher IP core implemented on the Zynq device
 */
class ScanMatcherCorrelativeFPGA final : public ScanMatcher
{
public:
    /* Type definitions */
    using MemoryMappedIOPtr = std::unique_ptr<Hardware::MemoryMappedIO>;
    using AxiSimpleDMAPtr = std::unique_ptr<Hardware::AxiSimpleDMA>;
    using CMAMemory = Hardware::CMAMemory;

    /* Constructor */
    ScanMatcherCorrelativeFPGA(
        const std::string& scanMatcherName,
        const CostFuncPtr& costFunc,
        const double rangeX,
        const double rangeY,
        const double rangeTheta,
        ScanMatcherHardwareConfig&& scanMatcherConfig,
        AxiDmaConfig&& axiDmaConfig);

    /* Destructor */
    ~ScanMatcherCorrelativeFPGA() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const GridMap& gridMap,
        const LocalMapId gridMapId,
        const Point2D<double>& gridMapCenterPos,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalInitialPose,
        const double normalizedScoreThreshold,
        const double knownRateThreshold);

private:
    /* Compute the search step */
    void ComputeSearchStep(const GridMapInterface& gridMap,
                           const Sensor::ScanDataPtr<double>& scanData,
                           double& stepX,
                           double& stepY,
                           double& stepTheta) const;

    /* Initialize the CMA memory for the input data */
    void InitializeCMAMemoryInput();
    /* Initialize the CMA memory for the output data */
    void InitializeCMAMemoryOutput();

    /* Compute the bounding box of the grid map */
    BoundingBox<int> ComputeBoundingBox(
        const GridMap& gridMap,
        const Point2D<double>& centerPos) const;

    /* Set the scan matching parameters through AXI4-Lite slave interface */
    void SetParameterRegisters(
        const int numOfScans,
        const double scanRangeMax,
        const int scoreThreshold,
        const RobotPose2D<double>& minSensorPose,
        const Point2D<int>& gridMapSize,
        const Point2D<double>& gridMapMinPos,
        const int winX, const int winY, const int winTheta,
        const double stepX, const double stepY, const double stepTheta);

    /* Send the scan data through AXI DMA */
    void SendScanData(const Sensor::ScanDataPtr<double>& scanData);
    /* Send the grid map through AXI DMA */
    void SendGridMap(const GridMap& gridMap,
                     const bool gridMapTransferred,
                     const BoundingBox<int>& desiredBox);
    /* Receive the result through AXI DMA */
    void ReceiveResult(int& scoreMax, int& bestX, int& bestY, int& bestTheta);

    /* Start the scan matcher IP core */
    void StartIPCore();
    /* Wait until the scan matcher IP core is in idle state */
    void WaitIPCore();

    /* Read the control register of the scan matcher IP core */
    inline std::uint32_t ReadCtrlReg()
    { return this->mControlRegisters->Read(this->mConfig.mAxiLiteSApCtrl); }
    /* Write to the control register of the scan matcher IP core */
    inline void WriteCtrlReg(std::uint32_t value)
    { this->mControlRegisters->Write(this->mConfig.mAxiLiteSApCtrl, value); }

private:
    /* Cost function just for calculating the pose covariance matrix */
    const CostFuncPtr mCostFunc;
    /* Linear (horizontal) size of the searching window */
    const double      mRangeX;
    /* Linear (vertical) size of the searching window */
    const double      mRangeY;
    /* Angular range of the searching window */
    const double      mRangeTheta;

    /* Id of the last grid map that is transferred to the IP core */
    LocalMapId                      mLastMapId;
    /* Size of the last grid map (in the number of the grid cells) */
    Point2D<int>                    mLastMapSize;
    /* Minimum position of the last grid map
     * (in the map-local coordinate frame) */
    Point2D<double>                 mLastMapMinPos;
    /* Configuration of the scan matcher IP core */
    const ScanMatcherHardwareConfig mConfig;
    /* Configuration of the AXI DMA IP core */
    const AxiDmaConfig              mAxiDmaConfig;
    /* Interface to the scan matcher IP core control registers */
    MemoryMappedIOPtr               mControlRegisters;
    /* Interface to the AXI DMA IP core */
    AxiSimpleDMAPtr                 mAxiDma;
    /* CMA memory to store the scan matching input */
    CMAMemory                       mInputData;
    /* CMA memory to store the scan matching result */
    CMAMemory                       mOutputData;
    /* Metrics information */
    ScanMatcherFPGAMetrics          mMetrics;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_CORRELATIVE_FPGA_HPP */
