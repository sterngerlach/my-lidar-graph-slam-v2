
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
#include <type_traits>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/timer/timer.hpp>

#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Metric {

/* Convert the double to the std::string */
inline std::string DoubleToString(const double value)
{
    std::stringstream strStream;
    strStream << std::fixed << std::setprecision(6);
    strStream << value;
    return strStream.str();
}

/* Convert the vector to the std::string */
template <typename T>
inline std::string VecToString(const std::vector<T>& values)
{
    std::stringstream strStream;

    if (std::is_floating_point<T>::value)
        strStream << std::fixed << std::setprecision(6);

    const std::size_t numOfValues = values.size();

    for (std::size_t i = 0; i < numOfValues; ++i)
        if (i == numOfValues - 1)
            strStream << values[i];
        else
            strStream << values[i] << ' ';

    return strStream.str();
}

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

    /* Convert the metric to the Boost property tree */
    virtual boost::property_tree::ptree ToPropertyTree() const { return {}; }

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

    /* Convert the metric to the Boost property tree */
    boost::property_tree::ptree ToPropertyTree() const override;

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

    /* Convert the metric to the Boost property tree */
    boost::property_tree::ptree ToPropertyTree() const override;

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

    /* Convert the metric to the Boost property tree */
    boost::property_tree::ptree ToPropertyTree() const override;

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
    virtual const BucketBoundaries* Boundaries() const = 0;
    /* Retrieve the bucket counts */
    virtual const std::vector<double>* Counts() const = 0;
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
    const BucketBoundaries* Boundaries() const override { return nullptr; }
    /* Retrieve the bucket counts */
    const std::vector<double>* Counts() const override { return nullptr; }
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

    /* Convert the metric to the Boost property tree */
    boost::property_tree::ptree ToPropertyTree() const override;

    /* Reset the histogram */
    void Reset() override;

    /* Observe the value */
    void Observe(double val) override;

    /* Retrieve the boundary values */
    const BucketBoundaries* Boundaries() const override
    { return &this->mBucketBoundaries; }
    /* Retrieve the bucket counts */
    const std::vector<double>* Counts() const override
    { return &this->mBucketCounts; }
    /* Retrieve the summation counter */
    double SumValues() const override { return this->mSumValues; }

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
    /* Bucket boundaries */
    BucketBoundaries          mBucketBoundaries;
    /* Bucket counters */
    std::vector<double>       mBucketCounts;
    /* Sum of the observed values */
    double                    mSumValues;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

template <typename T>
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
    virtual void Observe(T val) = 0;

    /* Retrieve the number of data points */
    virtual std::size_t NumOfValues() const = 0;
    /* Retrieve the values */
    virtual const std::vector<T>* Values() const = 0;
    /* Retrieve the value in the container */
    virtual T ValueAt(std::size_t valueIdx) const = 0;

    /* Dump the value sequence object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

template <typename T>
class NullValueSequence final : public ValueSequenceBase<T>
{
public:
    /* Constructor */
    NullValueSequence() : ValueSequenceBase<T>("") { }
    /* Destructor */
    ~NullValueSequence() = default;

    /* Reset the value sequence */
    void Reset() override { }

    /* Observe the value and append to the sequence */
    void Observe(T) override { }

    /* Retrieve the number of data points */
    std::size_t NumOfValues() const override { return 0; }
    /* Retrieve the values */
    const std::vector<T>* Values() const override { return nullptr; }
    /* Retrieve the value in the container */
    T ValueAt(std::size_t) const override { return {}; }

    /* Dump the value sequence object */
    void Dump(std::ostream&) const override { }
};

template <typename T>
class ValueSequence final : public ValueSequenceBase<T>
{
public:
    /* Constructor */
    ValueSequence(const std::string& metricId) :
        ValueSequenceBase<T>(metricId), mValues(), mMutex() { }
    /* Destructor */
    ~ValueSequence() = default;

    /* Convert the metric to the Boost property tree */
    boost::property_tree::ptree ToPropertyTree() const override;

    /* Reset the value sequence */
    void Reset() override;

    /* Observe the value and append to the sequence */
    void Observe(T val) override;

    /* Retrieve the number of data points */
    std::size_t NumOfValues() const override
    { return this->mValues.size(); }
    /* Retrieve the values */
    const std::vector<T>* Values() const override
    { return &this->mValues; }
    /* Retrieve the value in the container */
    T ValueAt(std::size_t valueIdx) const override
    { return this->mValues.at(valueIdx); }

    /* Dump the value sequence object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Value sequence */
    std::vector<T>            mValues;
    /* Shared mutex */
    mutable std::shared_mutex mMutex;
};

/*
 * ValueSequence class implementations
 */

/* Convert the metric to the Boost property tree */
template <typename T>
boost::property_tree::ptree ValueSequence<T>::ToPropertyTree() const
{
    const auto valuesStr = this->Values() != nullptr ?
        VecToString(*this->Values()) : std::string();

    boost::property_tree::ptree metric;
    metric.put("NumOfSamples", this->NumOfValues());
    metric.put("Values", valuesStr);

    return metric;
}

/* Reset the value sequence */
template <typename T>
void ValueSequence<T>::Reset()
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValues.clear();
}

/* Observe the value and append to the sequence */
template <typename T>
void ValueSequence<T>::Observe(T val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValues.push_back(static_cast<T>(val));
}

/* Dump the value sequence object */
template <typename T>
void ValueSequence<T>::Dump(std::ostream& outStream) const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };

    outStream << "ValueSequence Id: " << this->mId << ", "
              << "Number of samples: " << this->mValues.size() << '\n';
    outStream << Join(this->mValues, ", ") << '\n';
}

class MetricManager final
{
private:
    /* Constructor */
    MetricManager() = default;
    /* Destructor */
    ~MetricManager() = default;

public:
    /* Type definitions */
    using ptree = boost::property_tree::ptree;
    using MetricPtr = std::unique_ptr<MetricBase>;

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
    inline const CounterBase* CounterMetric(const std::string& metricId) const;
    /* Retrieve the counter metrics */
    inline CounterBase* CounterMetric(const std::string& metricId);

    /* Retrieve the gauge metrics */
    inline const GaugeBase* GaugeMetric(const std::string& metricId) const;
    /* Retrieve the gauge metrics */
    inline GaugeBase* GaugeMetric(const std::string& metricId);

    /* Retrieve the distribution metrics */
    inline const DistributionBase* DistributionMetric(
        const std::string& metricId) const;
    /* Retrieve the distribution metrics */
    inline DistributionBase* DistributionMetric(const std::string& metricId);

    /* Retrieve the histogram metrics */
    inline const HistogramBase* HistogramMetric(
        const std::string& metricId) const;
    /* Retrieve the histogram metrics */
    inline HistogramBase* HistogramMetric(const std::string& metricId);

    /* Retrieve the value sequence metrics */
    template <typename T>
    inline const ValueSequenceBase<T>* ValueSequenceMetric(
        const std::string& metricId) const;
    /* Retrieve the value sequence metrics */
    template <typename T>
    inline ValueSequenceBase<T>* ValueSequenceMetric(
        const std::string& metricId);

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
    template <typename T>
    ValueSequenceBase<T>* AddValueSequence(const std::string& metricName);

private:
    /* Append the metric to the list */
    void Append(std::vector<MetricPtr>& metrics,
                MetricBase* metric);
    /* Remove the metric from the list */
    void Remove(std::vector<MetricPtr>& metrics,
                const std::string& metricId);
    /* Find the metric from the list */
    MetricBase* Find(std::vector<MetricPtr>& metrics,
                     const std::string& metricId);
    /* Find the metric from the list */
    const MetricBase* Find(const std::vector<MetricPtr>& metrics,
                           const std::string& metricId) const;

private:
    /* List of the counter metrics */
    std::vector<MetricPtr> mCounters;
    /* List of the gauge metrics */
    std::vector<MetricPtr> mGauges;
    /* List of the distribution metrics */
    std::vector<MetricPtr> mDists;
    /* List of the histogram metrics */
    std::vector<MetricPtr> mHists;
    /* List of the value sequence metrics */
    std::vector<MetricPtr> mValueSeqs;
};

/* Retrieve the counter metrics */
const CounterBase* MetricManager::CounterMetric(
    const std::string& metricId) const
{
    static const auto nullCounter = std::make_unique<NullCounter>();
    const MetricBase* metric = this->Find(this->mCounters, metricId);

    if (metric == nullptr)
        return nullCounter.get();

    const auto* counter = dynamic_cast<const CounterBase*>(metric);
    Assert(counter != nullptr);
    return counter;
}

/* Retrieve the counter metrics */
CounterBase* MetricManager::CounterMetric(const std::string& metricId)
{
    const auto* constThis = this;
    return const_cast<CounterBase*>(constThis->CounterMetric(metricId));
}

/* Retrieve the gauge metrics */
const GaugeBase* MetricManager::GaugeMetric(const std::string& metricId) const
{
    static const auto nullGauge = std::make_unique<NullGauge>();
    const MetricBase* metric = this->Find(this->mGauges, metricId);

    if (metric == nullptr)
        return nullGauge.get();

    const auto* gauge = dynamic_cast<const GaugeBase*>(metric);
    Assert(gauge != nullptr);
    return gauge;
}

/* Retrieve the gauge metrics */
GaugeBase* MetricManager::GaugeMetric(const std::string& metricId)
{
    const auto* constThis = this;
    return const_cast<GaugeBase*>(constThis->GaugeMetric(metricId));
}

/* Retrieve the distribution metrics */
const DistributionBase* MetricManager::DistributionMetric(
    const std::string& metricId) const
{
    static const auto nullDist = std::make_unique<NullDistribution>();
    const MetricBase* metric = this->Find(this->mDists, metricId);

    if (metric == nullptr)
        return nullDist.get();

    const auto* dist = dynamic_cast<const DistributionBase*>(metric);
    Assert(dist != nullptr);
    return dist;
}

/* Retrieve the distribution metrics */
DistributionBase* MetricManager::DistributionMetric(
    const std::string& metricId)
{
    const auto* constThis = this;
    return const_cast<DistributionBase*>(
        constThis->DistributionMetric(metricId));
}

/* Retrieve the histogram metrics */
const HistogramBase* MetricManager::HistogramMetric(
    const std::string& metricId) const
{
    static const auto nullHist = std::make_unique<NullHistogram>();
    const MetricBase* metric = this->Find(this->mHists, metricId);

    if (metric == nullptr)
        return nullHist.get();

    const auto* hist = dynamic_cast<const HistogramBase*>(metric);
    Assert(hist != nullptr);
    return hist;
}

/* Retrieve the histogram metrics */
HistogramBase* MetricManager::HistogramMetric(
    const std::string& metricId)
{
    const auto* constThis = this;
    return const_cast<HistogramBase*>(
        constThis->HistogramMetric(metricId));
}

/* Retrieve the value sequence metrics */
template <typename T>
const ValueSequenceBase<T>* MetricManager::ValueSequenceMetric(
    const std::string& metricId) const
{
    static const auto nullValueSeq = std::make_unique<NullValueSequence<T>>();
    const MetricBase* metric = this->Find(this->mValueSeqs, metricId);

    if (metric == nullptr)
        return nullValueSeq.get();

    const auto* valueSeq = dynamic_cast<const ValueSequenceBase<T>*>(metric);
    Assert(valueSeq != nullptr);
    return valueSeq;
}

/* Retrieve the value sequence metrics */
template <typename T>
ValueSequenceBase<T>* MetricManager::ValueSequenceMetric(
    const std::string& metricId)
{
    const auto* constThis = this;
    return const_cast<ValueSequenceBase<T>*>(
        constThis->ValueSequenceMetric<T>(metricId));
}

/* Append the value sequence metric */
template <typename T>
ValueSequenceBase<T>* MetricManager::AddValueSequence(
    const std::string& metricName)
{
    auto* valueSeq = new ValueSequence<T>(metricName);
    this->Append(this->mValueSeqs, valueSeq);
    return valueSeq;
}

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
