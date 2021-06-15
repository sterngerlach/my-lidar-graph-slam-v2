
/* metric.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_METRIC_METRIC_HPP
#define MY_LIDAR_GRAPH_SLAM_METRIC_METRIC_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <shared_mutex>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/timer/timer.hpp>

namespace MyLidarGraphSlam {
namespace Metric {

enum class MetricType
{
    None,
    Counter,
    Gauge,
    Distribution,
    Histogram,
    ValueSequence,
    MetricFamily,
};

class MetricBase
{
public:
    /* Constructor */
    MetricBase(MetricType type,
               const std::string& metricId) :
        mType(type),
        mId(metricId) { }
    /* Destructor */
    virtual ~MetricBase() = default;

    /* Retrieve the metric type */
    virtual MetricType Type() const final { return this->mType; }
    /* Retrieve the metric Id string */
    virtual const std::string& Id() const final { return this->mId; }

protected:
    /* Metric type */
    MetricType  mType;
    /* Metric Id string */
    std::string mId;
};

class CounterBase : public MetricBase
{
public:
    /* Constructor */
    CounterBase(const std::string& metricId) :
        MetricBase(MetricType::Counter, metricId) { }
    /* Destructor */
    virtual ~CounterBase() = default;

    /* Reset the counter value */
    virtual void Reset() = 0;

    /* Retrieve the counter value */
    virtual double Value() const = 0;

    /* Increment the counter by specified value */
    virtual void Increment(double val = 1.0) = 0;

    /* Dump the counter object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullCounter final : public CounterBase
{
public:
    /* Constructor */
    NullCounter() : CounterBase("") { }
    /* Destructor */
    ~NullCounter() = default;

    /* Reset the counter value */
    void Reset() override { }

    /* Retrieve the counter value */
    double Value() const override { return 0.0; }

    /* Increment the counter by specified value */
    void Increment(double) override { }

    /* Dump the counter object */
    void Dump(std::ostream&) const override { }
};

class Counter final : public CounterBase
{
public:
    /* Constructor */
    Counter(const std::string& metricId) :
        CounterBase(metricId), mValue(0.0), mMutex() { }
    /* Constructor with default counter */
    Counter(const std::string& metricId, double initVal) :
        CounterBase(metricId), mValue(initVal) { }
    /* Destructor */
    ~Counter() = default;

    /* Reset the counter value */
    void Reset() override;
    /* Retrieve the counter value */
    double Value() const override;
    /* Increment the counter by the specified value */
    void Increment(double val = 1.0) override;
    /* Dump the counter object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Counter value */
    double                    mValue;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

class GaugeBase : public MetricBase
{
public:
    /* Constructor */
    GaugeBase(const std::string& metricId) :
        MetricBase(MetricType::Gauge, metricId) { }
    /* Destructor */
    virtual ~GaugeBase() = default;

    /* Reset the gauge value */
    virtual void Reset() = 0;

    /* Retrieve the gauge value */
    virtual double Value() const = 0;
    /* Set the gauge value */
    virtual void SetValue(double val) = 0;

    /* Increment the gauge by specified value */
    virtual void Increment(double val = 1.0) = 0;
    /* Decrement the gauge by specified value */
    virtual void Decrement(double val = 1.0) = 0;

    /* Dump the gauge object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullGauge final : public GaugeBase
{
public:
    /* Constructor */
    NullGauge() : GaugeBase("") { }
    /* Destructor */
    ~NullGauge() = default;

    /* Reset the gauge value */
    void Reset() override { }

    /* Retrieve the gauge value */
    double Value() const override { return 0.0; }
    /* Set the gauge value */
    void SetValue(double) override { }

    /* Increment the gauge by specified value */
    void Increment(double) override { }
    /* Decrement the gauge by specified value */
    void Decrement(double) override { }

    /* Dump the gauge object */
    void Dump(std::ostream&) const override { }
};

class Gauge final : public GaugeBase
{
public:
    /* Constructor */
    Gauge(const std::string& metricId) :
        GaugeBase(metricId), mValue(0.0), mMutex() { }
    /* Constructor with default value */
    Gauge(const std::string& metricId, double initVal) :
        GaugeBase(metricId), mValue(initVal), mMutex() { }
    /* Destructor */
    ~Gauge() = default;

    /* Reset the gauge value */
    void Reset() override;

    /* Retrieve the gauge value */
    double Value() const override;
    /* Set the gauge value */
    void SetValue(double val) override;

    /* Increment the gauge by the specified value */
    void Increment(double val = 1.0) override;
    /* Decrement the gauge by the specified value */
    void Decrement(double val = 1.0) override;

    /* Dump the gauge object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Gauge value */
    double                    mValue;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

class DistributionBase : public MetricBase
{
public:
    /* Constructor */
    DistributionBase(const std::string& metricId) :
        MetricBase(MetricType::Distribution, metricId) { }
    /* Destructor */
    virtual ~DistributionBase() = default;

    /* Reset the distribution */
    virtual void Reset() = 0;

    /* Observe the value and update mean and variance */
    virtual void Observe(double val) = 0;

    /* Retrieve the number of the observed values */
    virtual int NumOfSamples() const = 0;
    /* Retrieve the sum of the observed values */
    virtual double Sum() const = 0;
    /* Retrieve the mean of the observed values */
    virtual double Mean() const = 0;
    /* Retrieve the unbiased variance of the observed values */
    virtual double Variance() const = 0;
    /* Retrieve the standard deviation of the observed values */
    virtual double StandardDeviation() const = 0;
    /* Retrieve the maximum of the observed values */
    virtual double Maximum() const = 0;
    /* Retrieve the minimum of the observed values */
    virtual double Minimum() const = 0;

    /* Dump the distribution object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullDistribution final : public DistributionBase
{
public:
    /* Constructor */
    NullDistribution() : DistributionBase("") { }
    /* Destructor */
    ~NullDistribution() = default;

    /* Reset the distribution */
    void Reset() override { }

    /* Observe the value and update mean and variance */
    void Observe(double) override { }

    /* Retrieve the number of the observed values */
    int NumOfSamples() const override { return 0; }
    /* Retrieve the sum of the observed values */
    double Sum() const override { return 0.0; }
    /* Retrieve the mean of the observed values */
    double Mean() const override { return 0.0; }
    /* Retrieve the unbiased variance of the observed values */
    double Variance() const override { return 0.0; }
    /* Retrieve the standard deviation of the observed values */
    double StandardDeviation() const override { return 0.0; }
    /* Retrieve the maximum of the observed values */
    double Maximum() const override { return 0.0; }
    /* Retrieve the minimum of the observed values */
    double Minimum() const override { return 0.0; }

    /* Dump the distribution object */
    void Dump(std::ostream&) const override { }
};

class Distribution final : public DistributionBase
{
public:
    /* Constructor */
    Distribution(const std::string& metricId) :
        DistributionBase(metricId),
        mNumOfSamples(0),
        mSum(0.0),
        mMean(0.0),
        mScaledVariance(0.0),
        mMaximum(0.0),
        mMinimum(0.0),
        mMutex() { }
    /* Destructor */
    ~Distribution() = default;

    /* Reset the distribution */
    void Reset() override;

    /* Observe the value and update mean and variance */
    void Observe(double val) override;

    /* Retrieve the number of the observed values */
    int NumOfSamples() const override;
    /* Retrieve the sum of the observed values */
    double Sum() const override;
    /* Retrieve the mean of the observed values */
    double Mean() const override;
    /* Retrieve the unbiased variance of the observed values */
    double Variance() const override;
    /* Retrieve the standard deviation of the observed values */
    double StandardDeviation() const override;
    /* Retrieve the maximum of the observed values */
    double Maximum() const override;
    /* Retrieve the minimum of the observed values */
    double Minimum() const override;

    /* Dump the distribution object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Compute the unbiased variance of the observed values */
    double UnlockedVariance() const;
    /* Compute the standard deviation of the observed values */
    double UnlockedStandardDeviation() const;

private:
    /* Number of the observed values */
    int                       mNumOfSamples;
    /* Sum of the observed values */
    double                    mSum;
    /* Mean of the observed values */
    double                    mMean;
    /* Scaled variance of the observed values */
    double                    mScaledVariance;
    /* Maximum of the observed values */
    double                    mMaximum;
    /* Minimum of the observed values */
    double                    mMinimum;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

/* Type declarations */
using BucketBoundaries = std::vector<double>;

class HistogramBase : public MetricBase
{
public:
    /* Constructor */
    HistogramBase(const std::string& metricId) :
        MetricBase(MetricType::Histogram, metricId) { }
    /* Destructor */
    virtual ~HistogramBase() = default;

    /* Reset the histogram */
    virtual void Reset() = 0;

    /* Observe the value */
    virtual void Observe(double val) = 0;

    /* Retrieve the boundary values */
    virtual BucketBoundaries Boundaries() const = 0;
    /* Retrieve the bucket counts */
    virtual std::vector<double> Counts() const = 0;
    /* Retrieve the summation counter */
    virtual double SumValues() const = 0;

    /* Retrieve the number of observed values */
    virtual double NumOfSamples() const = 0;
    /* Retrieve the mean of the observed values */
    virtual double Mean() const = 0;

    /* Retrieve the value range of the specified bucket */
    virtual void ValueRange(std::size_t bucketIdx,
                            double& rangeMin,
                            double& rangeMax) const = 0;

    /* Dump the histogram object */
    virtual void Dump(std::ostream& outStream, bool isVerbose) const = 0;
};

class NullHistogram final : public HistogramBase
{
public:
    /* Constructor */
    NullHistogram() : HistogramBase("") { }
    /* Destructor */
    ~NullHistogram() = default;

    /* Reset the histogram */
    void Reset() override { }

    /* Observe the value */
    void Observe(double) override { }

    /* Retrieve the boundary values */
    BucketBoundaries Boundaries() const override
    { return BucketBoundaries(); }
    /* Retrieve the bucket counts */
    std::vector<double> Counts() const override
    { return std::vector<double>(); }
    /* Retrieve the summation counter */
    double SumValues() const override { return 0.0; }

    /* Retrieve the number of observed values */
    double NumOfSamples() const override { return 0.0; }
    /* Retrieve the mean of the observed values */
    double Mean() const override { return 0.0; }

    /* Retrieve the value range of the specified bucket */
    void ValueRange(std::size_t bucketIdx,
                    double& rangeMin,
                    double& rangeMax) const override;

    /* Dump the histogram object */
    void Dump(std::ostream&, bool) const override { }
};

class Histogram final : public HistogramBase
{
public:
    /* Create the bucket boundaries with fixed width */
    static BucketBoundaries CreateFixedWidthBoundaries(
        double startVal, double bucketWidth, int numOfFiniteBuckets);

    /* Create the bucket boundaries with fixed width */
    static BucketBoundaries CreateFixedWidthBoundaries(
        double startVal, double endVal, double bucketWidth);

    /* Create the bucket boundaries with exponential width */
    static BucketBoundaries CreateExponentialWidthBoundaries(
        double startVal, double endVal, double baseVal);

    /* Constructor */
    Histogram(const std::string& metricId,
              const BucketBoundaries& bucketBoundaries);
    /* Destructor */
    ~Histogram() = default;

    /* Reset the histogram */
    void Reset() override;

    /* Observe the value */
    void Observe(double val) override;

    /* Retrieve the boundary values */
    BucketBoundaries Boundaries() const override;
    /* Retrieve the bucket counts */
    std::vector<double> Counts() const override;
    /* Retrieve the summation counter */
    double SumValues() const override;

    /* Retrieve the number of observed values */
    double NumOfSamples() const override;
    /* Retrieve the mean of the observed values */
    double Mean() const override;

    /* Retrieve the value range of the specified bucket */
    void ValueRange(std::size_t bucketIdx,
                    double& rangeMin,
                    double& rangeMax) const override;

    /* Dump the histogram object */
    void Dump(std::ostream& outStream, bool isVerbose) const override;

private:
    /* Compute the number of the observed values */
    double UnlockedNumOfSamples() const;
    /* Compute the mean of the observed values */
    double UnlockedMean() const;

private:
    /* Bucket boundaries */
    BucketBoundaries          mBucketBoundaries;
    /* Bucket counters */
    std::vector<double>       mBucketCounts;
    /* Sum of the observed values */
    double                    mSumValues;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

class ValueSequenceBase : public MetricBase
{
public:
    /* Constructor */
    ValueSequenceBase(const std::string& metricId) :
        MetricBase(MetricType::ValueSequence, metricId) { }
    /* Destructor */
    virtual ~ValueSequenceBase() = default;

    /* Reset the value sequence */
    virtual void Reset() = 0;

    /* Observe the value and append to the sequence */
    virtual void Observe(double val) = 0;

    /* Retrieve the number of data points */
    virtual std::size_t NumOfValues() const = 0;
    /* Retrieve the value in the container */
    virtual double ValueAt(std::size_t valueIdx) const = 0;

    /* Dump the value sequence object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullValueSequence final : public ValueSequenceBase
{
public:
    /* Constructor */
    NullValueSequence() : ValueSequenceBase("") { }
    /* Destructor */
    ~NullValueSequence() = default;

    /* Reset the value sequence */
    void Reset() override { }

    /* Observe the value and append to the sequence */
    void Observe(double) override { }

    /* Retrieve the number of data points */
    std::size_t NumOfValues() const override { return 0; }
    /* Retrieve the value in the container */
    double ValueAt(std::size_t) const override { return 0.0; }

    /* Dump the value sequence object */
    void Dump(std::ostream&) const override { }
};

template <typename T>
class ValueSequence final : public ValueSequenceBase
{
public:
    /* Constructor */
    ValueSequence(const std::string& metricId) :
        ValueSequenceBase(metricId), mValues(), mMutex() { }
    /* Destructor */
    ~ValueSequence() = default;

    /* Reset the value sequence */
    void Reset() override;

    /* Observe the value and append to the sequence */
    void Observe(double val) override;

    /* Retrieve the number of data points */
    std::size_t NumOfValues() const override;
    /* Retrieve the value in the container */
    double ValueAt(std::size_t valueIdx) const override;

    /* Dump the value sequence object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Value sequence */
    std::vector<T>            mValues;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

template <typename T>
class MetricFamily final : public MetricBase
{
public:
    /* Constructor */
    MetricFamily(const std::string& metricId,
                 T* pNullMetric) :
        MetricBase(MetricType::MetricFamily, metricId),
        mNullMetric(pNullMetric) { }
    /* Destructor */
    ~MetricFamily() = default;

    /* Append the new metric */
    void Append(T* pMetric);
    /* Remove the metric */
    void Remove(const std::string& metricId);

    /* Retrieve the number of the metrics */
    inline std::size_t NumOfMetrics() const
    { return this->mMetrics.size(); }
    /* Retrieve the metric at a specified index */
    inline const T* MetricAt(std::size_t metricIdx) const
    { return this->mMetrics.at(metricIdx).get(); }
    /* Retrieve the metric at a specified index */
    T* MetricAt(std::size_t metricIdx)
    { return this->mMetrics.at(metricIdx).get(); }

    /* Retrieve the specified metric object */
    const T* Metric(const std::string& metricId) const;
    /* Retrieve the specified metric object */
    T* Metric(const std::string& metricId);

    /* Retrieve the specified metric object */
    const T* operator()(const std::string& metricId) const
    { return this->Metric(metricId); }
    /* Retrieve the specified metric object */
    T* operator()(const std::string& metricId)
    { return this->Metric(metricId); }

private:
    /* List of the metric Ids and metric objects */
    std::vector<std::unique_ptr<T>> mMetrics;
    /* Null metric */
    std::unique_ptr<T>              mNullMetric;
};

class MetricManager final
{
private:
    /* Constructor */
    MetricManager() :
        mCounterMetrics("Counters", new NullCounter()),
        mGaugeMetrics("Gauges", new NullGauge()),
        mDistributionMetrics("Distributions", new NullDistribution()),
        mHistogramMetrics("Histograms", new NullHistogram()),
        mValueSequenceMetrics("ValueSequences", new NullValueSequence()) { }
    /* Destructor */
    ~MetricManager() = default;

public:
    /* Type definitions */
    using ptree = boost::property_tree::ptree;

    /* Copy constructor (disabled) */
    MetricManager(const MetricManager&) = delete;
    /* Copy assignment operator (disabled) */
    MetricManager& operator=(const MetricManager&) = delete;
    /* Move constructor (disabled) */
    MetricManager(MetricManager&&) = delete;
    /* Move assignment operator (disabled) */
    MetricManager& operator=(MetricManager&&) = delete;

    /* Get the MetricManager singleton instance */
    static MetricManager* Instance();

    /* Convert all metrics to the Boost property tree */
    ptree ToPropertyTree() const;

    /* Retrieve the counter metrics */
    const MetricFamily<CounterBase>& CounterMetrics() const
    { return this->mCounterMetrics; }
    /* Retrieve the counter metrics */
    MetricFamily<CounterBase>& CounterMetrics()
    { return this->mCounterMetrics; }

    /* Retrieve the gauge metrics */
    const MetricFamily<GaugeBase>& GaugeMetrics() const
    { return this->mGaugeMetrics; }
    /* Retrieve the gauge metrics */
    MetricFamily<GaugeBase>& GaugeMetrics()
    { return this->mGaugeMetrics; }

    /* Retrieve the distribution metrics */
    const MetricFamily<DistributionBase>& DistributionMetrics() const
    { return this->mDistributionMetrics; }
    /* Retrieve the distribution metrics */
    MetricFamily<DistributionBase>& DistributionMetrics()
    { return this->mDistributionMetrics; }

    /* Retrieve the histogram metrics */
    const MetricFamily<HistogramBase>& HistogramMetrics() const
    { return this->mHistogramMetrics; }
    /* Retrieve the histogram metrics */
    MetricFamily<HistogramBase>& HistogramMetrics()
    { return this->mHistogramMetrics; }

    /* Retrieve the value sequence metrics */
    const MetricFamily<ValueSequenceBase>& ValueSequenceMetrics() const
    { return this->mValueSequenceMetrics; }
    /* Retrieve the value sequence metrics */
    MetricFamily<ValueSequenceBase>& ValueSequenceMetrics()
    { return this->mValueSequenceMetrics; }

    /* Append the counter metric */
    CounterBase* AddCounter(const std::string& metricName);
    /* Append the gauge metric */
    GaugeBase* AddGauge(const std::string& metricName);
    /* Append the distribution metric */
    DistributionBase* AddDistribution(const std::string& metricName);
    /* Append the histogram metric */
    HistogramBase* AddHistogram(const std::string& metricName,
                                const BucketBoundaries& bucketBoundaries);
    /* Append the value sequence metric */
    ValueSequenceBase* AddValueSequenceDouble(const std::string& metricName);
    /* Append the value sequence metric */
    ValueSequenceBase* AddValueSequenceFloat(const std::string& metricName);
    /* Append the value sequence metric */
    ValueSequenceBase* AddValueSequenceInt(const std::string& metricName);
    /* Append the value sequence metric */
    ValueSequenceBase* AddValueSequenceBool(const std::string& metricName);

private:
    /* List of the counter metrics */
    MetricFamily<CounterBase>       mCounterMetrics;
    /* List of the gauge metrics */
    MetricFamily<GaugeBase>         mGaugeMetrics;
    /* List of the distribution metrics */
    MetricFamily<DistributionBase>  mDistributionMetrics;
    /* List of the histogram metrics */
    MetricFamily<HistogramBase>     mHistogramMetrics;
    /* List of the value sequence metrics */
    MetricFamily<ValueSequenceBase> mValueSequenceMetrics;
};

struct Timer
{
    /* Constructor (internal timer is automatically started) */
    Timer() = default;
    /* Destructor */
    ~Timer() = default;

    /* Convert nanoseconds to milliseconds */
    inline std::int64_t ToMilli(const std::int_least64_t nanoSec) const
    { return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::nanoseconds(nanoSec)).count(); }

    /* Convert nanoseconds to microseconds */
    inline std::int64_t ToMicro(const std::int_least64_t nanoSec) const
    { return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::nanoseconds(nanoSec)).count(); }

    /* Get the elapsed time in microseconds */
    inline std::int64_t ElapsedMicro() const
    { return this->ToMicro(this->mTimer.elapsed().wall); }
    /* Get the elapsed time in milliseconds */
    inline std::int64_t ElapsedMilli() const
    { return this->ToMilli(this->mTimer.elapsed().wall); }

    /* Check if the timer is stopped */
    inline bool IsStopped() const { return this->mTimer.is_stopped(); }
    /* Stop the timer */
    inline void Stop() { this->mTimer.stop(); }
    /* Start the timer (timer restarts from zero) */
    inline void Start() { this->mTimer.start(); }
    /* Resume the timer */
    inline void Resume() { this->mTimer.resume(); }

    /* Internal timer object */
    boost::timer::cpu_timer mTimer;
};

} /* namespace Metric */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_METRIC_METRIC_HPP */
