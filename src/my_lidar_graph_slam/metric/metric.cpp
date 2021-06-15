
/* metric.cpp */

#include "my_lidar_graph_slam/metric/metric.hpp"

#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Metric {

/*
 * Counter class implementations
 */

/* Reset the counter value */
void Counter::Reset()
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValue = 0.0;
}

/* Retrieve the counter value */
double Counter::Value() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mValue;
}

/* Increment the counter by the specified value */
void Counter::Increment(double val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValue += std::max(0.0, val);
}

/* Dump the counter object */
void Counter::Dump(std::ostream& outStream) const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };

    outStream << "Counter Id: " << this->mId << ", "
              << "Value: " << this->mValue << '\n';
}

/*
 * Gauge class implementations
 */

/* Reset the gauge value */
void Gauge::Reset()
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValue = 0.0;
}

/* Retrieve the gauge value */
double Gauge::Value() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mValue;
}

/* Set the gauge value */
void Gauge::SetValue(double val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValue = val;
}

/* Increment the gauge by the specified value */
void Gauge::Increment(double val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValue += val;
}

/* Decrement the gauge by the specified value */
void Gauge::Decrement(double val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };
    this->mValue -= val;
}

/* Dump the gauge object */
void Gauge::Dump(std::ostream& outStream) const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };

    outStream << "Gauge Id: " << this->mId << ", "
              << "Value: " << this->mValue << '\n';
}

/*
 * Distribution class implementations
 */

/* Reset the distribution */
void Distribution::Reset()
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };

    this->mNumOfSamples = 0;
    this->mSum = 0.0;
    this->mMean = 0.0;
    this->mScaledVariance = 0.0;
    this->mMaximum = 0.0;
    this->mMinimum = 0.0;
}

/* Observe the value and update mean and variance */
void Distribution::Observe(double val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };

    ++this->mNumOfSamples;
    this->mSum += val;

    if (this->mNumOfSamples == 1) {
        this->mMean = val;
        this->mScaledVariance = 0.0;
        this->mMaximum = val;
        this->mMinimum = val;
    } else {
        const double prevMean = this->mMean;
        this->mMean += (val - prevMean) / this->mNumOfSamples;
        this->mScaledVariance += (val - prevMean) * (val - this->mMean);
        this->mMaximum = std::max(this->mMaximum, val);
        this->mMinimum = std::min(this->mMinimum, val);
    }
}

/* Retrieve the number of the observed values */
int Distribution::NumOfSamples() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mNumOfSamples;
}

/* Retrieve the sum of the observed values */
double Distribution::Sum() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mSum;
}

/* Retrieve the mean of the observed values */
double Distribution::Mean() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mMean;
}

/* Retrieve the unbiased variance of the observed values */
double Distribution::Variance() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->UnlockedVariance();
}

/* Retrieve the standard deviation of the observed values */
double Distribution::StandardDeviation() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->UnlockedStandardDeviation();
}

/* Retrieve the maximum of the observed values */
double Distribution::Maximum() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mMaximum;
}

/* Retrieve the minimum of the observed values */
double Distribution::Minimum() const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };
    return this->mMinimum;
}

/* Compute the unbiased variance of the observed values */
double Distribution::UnlockedVariance() const
{
    return this->mNumOfSamples > 1 ?
        this->mScaledVariance / (this->mNumOfSamples - 1) : 0.0;
}

/* Compute the standard deviation of the observed values */
double Distribution::UnlockedStandardDeviation() const
{
    return std::sqrt(this->UnlockedVariance());
}

/* Dump the distribution object */
void Distribution::Dump(std::ostream& outStream) const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };

    const double variance = this->UnlockedVariance();
    const double standardDeviation = this->UnlockedStandardDeviation();

    outStream << "Distribution Id: " << this->mId << ", "
              << "Number of samples: " << this->mNumOfSamples << ", "
              << "Sum: " << this->mSum << ", "
              << "Mean: " << this->mMean << ", "
              << "Variance: " << variance << ", "
              << "Standard deviation: " << standardDeviation << ", "
              << "Max: " << this->mMaximum << ", "
              << "Min: " << this->mMinimum << '\n';
}

/*
 * NullHistogram class implementations
 */

/* Retrieve the value range of the specified bucket */
void NullHistogram::ValueRange(std::size_t,
                               double& rangeMin,
                               double& rangeMax) const
{
    rangeMin = 0.0;
    rangeMax = 0.0;
}

/*
 * Histogram class implementations
 */

/* Create the bucket boundaries with fixed width */
BucketBoundaries Histogram::CreateFixedWidthBoundaries(
    double startVal, double bucketWidth, int numOfFiniteBuckets)
{
    /* Input checks */
    assert(bucketWidth > 0.0);
    assert(numOfFiniteBuckets >= 0);

    /* Compute bucket boundaries */
    BucketBoundaries bucketBoundaries;
    bucketBoundaries.reserve(numOfFiniteBuckets + 1);

    double boundary = startVal;
    bucketBoundaries.emplace_back(boundary);

    for (int i = 0; i < numOfFiniteBuckets; ++i) {
        boundary += bucketWidth;
        bucketBoundaries.emplace_back(boundary);
    }

    return bucketBoundaries;
}

/* Create the bucket boundaries with fixed width */
BucketBoundaries Histogram::CreateFixedWidthBoundaries(
    double startVal, double endVal, double bucketWidth)
{
    /* Input checks */
    assert(endVal >= startVal);
    assert(bucketWidth > 0.0);

    /* Compute bucket boundaries */
    const int numOfFiniteBuckets = static_cast<int>(
        std::ceil((endVal - startVal) / bucketWidth));
    return Histogram::CreateFixedWidthBoundaries(
        startVal, bucketWidth, numOfFiniteBuckets);
}

/* Create the bucket boundaries with exponential width */
BucketBoundaries Histogram::CreateExponentialWidthBoundaries(
    double startVal, double endVal, double baseVal)
{
    /* Input checks */
    assert(startVal > 0.0);
    assert(endVal >= startVal);
    assert(baseVal > 1.0);

    /* Compute bucket boundaries */
    BucketBoundaries bucketBoundaries;
    double boundary = startVal;

    while (boundary < endVal) {
        bucketBoundaries.emplace_back(boundary);
        boundary *= baseVal;
    }

    return bucketBoundaries;
}

/* Constructor */
Histogram::Histogram(const std::string& metricId,
                     const BucketBoundaries& bucketBoundaries) :
    HistogramBase(metricId),
    mBucketBoundaries(bucketBoundaries),
    mBucketCounts(bucketBoundaries.size() + 1),
    mSumValues(0.0),
    mMutex()
{
    assert(!this->mBucketBoundaries.empty());
    assert(std::is_sorted(this->mBucketBoundaries.cbegin(),
                          this->mBucketBoundaries.cend()));
}

/* Reset the histogram */
void Histogram::Reset()
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };

    std::fill(this->mBucketCounts.begin(),
              this->mBucketCounts.end(), 0.0);
    this->mSumValues = 0.0;
}

/* Observe the value */
void Histogram::Observe(double val)
{
    std::lock_guard<std::shared_mutex> lock { this->mMutex };

    /* Determine the bucket index */
    const auto bucketIt = std::find_if(
        this->mBucketBoundaries.cbegin(),
        this->mBucketBoundaries.cend(),
        [val](const double boundary) { return val < boundary; });
    const auto bucketIdx = static_cast<std::size_t>(
        std::distance(this->mBucketBoundaries.cbegin(), bucketIt));

    /* Update the bucket counters */
    this->mBucketCounts[bucketIdx] += 1.0;

    /* Update the value sum */
    this->mSumValues += val;
}

/* Retrieve the number of observed values */
double Histogram::NumOfSamples() const
{
    return std::accumulate(this->mBucketCounts.cbegin(),
                           this->mBucketCounts.cend(), 0.0);
}

/* Retrieve the mean of the observed values */
double Histogram::Mean() const
{
    return this->mSumValues / this->NumOfSamples();
}

/* Retrieve the value range of the specified bucket */
void Histogram::ValueRange(std::size_t bucketIdx,
                           double& rangeMin,
                           double& rangeMax) const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };

    /* Input checks */
    assert(bucketIdx < this->mBucketCounts.size());

    /* Set the value range of the specified bucket */
    rangeMin = (bucketIdx == 0) ?
        std::numeric_limits<double>::min() :
        this->mBucketBoundaries[bucketIdx - 1];
    rangeMax = (bucketIdx == this->mBucketCounts.size() - 1) ?
        std::numeric_limits<double>::max() :
        this->mBucketBoundaries[bucketIdx];

    return;
}

/* Dump the histogram object */
void Histogram::Dump(std::ostream& outStream, bool isVerbose) const
{
    std::shared_lock<std::shared_mutex> lock { this->mMutex };

    outStream << "Histogram Id: " << this->mId << ", "
              << "Number of samples: " << this->NumOfSamples() << ", "
              << "Sum: " << this->mSumValues << ", "
              << "Mean: " << this->Mean() << '\n';

    /* Print the number of data points in each bins in verbose mode */
    if (!isVerbose)
        return;

    /* Dump the histogram (number of the values in each bucket) */
    outStream << " - " << this->mBucketBoundaries.front() << ": "
              << this->mBucketCounts.front() << '\n';

    for (std::size_t i = 0; i < this->mBucketBoundaries.size() - 1; ++i)
        outStream << this->mBucketBoundaries[i] << " - "
                  << this->mBucketBoundaries[i + 1] << ": "
                  << this->mBucketCounts[i + 1] << '\n';

    outStream << this->mBucketBoundaries.back() << " - : "
              << this->mBucketCounts.back() << '\n';
}

/*
 * MetricManager class implementations
 */

/* Get the MetricManager singleton instance */
MetricManager* MetricManager::Instance()
{
    static MetricManager theInstance;
    return &theInstance;
}

/* Append the metric to the list */
void MetricManager::Append(std::vector<MetricPtr>& metrics,
                           MetricBase* metric)
{
    const auto metricIt = std::find_if(
        metrics.begin(), metrics.end(),
        [metric](const MetricPtr& m) { return m->Id() == metric->Id(); });
    Assert(metricIt == metrics.end());
    metrics.push_back(std::unique_ptr<MetricBase>(metric));
}

/* Remove the metric from the list */
void MetricManager::Remove(std::vector<MetricPtr>& metrics,
                           const std::string& metricId)
{
    const auto metricIt = std::find_if(
        metrics.begin(), metrics.end(),
        [&metricId](const MetricPtr& m) { return m->Id() == metricId; });
    Assert(metricIt != metrics.end());
    metrics.erase(metricIt);
}

/* Find the metric from the list */
MetricBase* MetricManager::Find(std::vector<MetricPtr>& metrics,
                                const std::string& metricId)
{
    return const_cast<MetricBase*>(
        static_cast<const MetricManager*>(this)->Find(metrics, metricId));
}

/* Find the metric from the list */
const MetricBase* MetricManager::Find(const std::vector<MetricPtr>& metrics,
                                      const std::string& metricId) const
{
    const auto metricIt = std::find_if(
        metrics.begin(), metrics.end(),
        [metricId](const MetricPtr& m) { return m->Id() == metricId; });
    return (metricIt == metrics.end()) ? nullptr : metricIt->get();
}

/* Convert all metrics to the Boost property tree */
MetricManager::ptree MetricManager::ToPropertyTree() const
{
    const auto doubleToString = [](const double value) {
        std::stringstream strStream;
        strStream << std::fixed << std::setprecision(6);
        strStream << value;
        return strStream.str();
    };

    const auto doubleVecToString = [](const std::vector<double> values) {
        std::stringstream strStream;
        strStream << std::fixed << std::setprecision(6);

        const std::size_t numOfValues = values.size();

        for (std::size_t i = 0; i < numOfValues; ++i)
            if (i == numOfValues - 1)
                strStream << values[i];
            else
                strStream << values[i] << ' ';

        return strStream.str();
    };

    const auto valueSeqToString = [](const ValueSequenceBase* pValueSeq) {
        std::stringstream strStream;
        strStream << std::fixed << std::setprecision(6);

        const std::size_t numOfValues = pValueSeq->NumOfValues();

        for (std::size_t i = 0; i < numOfValues; ++i)
            if (i == numOfValues - 1)
                strStream << pValueSeq->ValueAt(i);
            else
                strStream << pValueSeq->ValueAt(i) << ' ';

        return strStream.str();
    };

    ptree rootTree;

    /* Process counter metrics */
    const std::size_t numOfCounters = this->mCounterMetrics.NumOfMetrics();
    ptree counterTree;

    for (std::size_t i = 0; i < numOfCounters; ++i) {
        const CounterBase* pCounter = this->mCounterMetrics.MetricAt(i);
        const auto valueStr = doubleToString(pCounter->Value());
        ptree metricTree;
        metricTree.put("Value", valueStr);
        counterTree.push_back(std::make_pair(pCounter->Id(), metricTree));
    }

    /* Process gauge metrics */
    const std::size_t numOfGauges = this->mGaugeMetrics.NumOfMetrics();
    ptree gaugeTree;

    for (std::size_t i = 0; i < numOfGauges; ++i) {
        const GaugeBase* pGauge = this->mGaugeMetrics.MetricAt(i);
        const auto valueStr = doubleToString(pGauge->Value());
        ptree metricTree;
        metricTree.put("Value", valueStr);
        gaugeTree.push_back(std::make_pair(pGauge->Id(), metricTree));
    }

    /* Process distribution metrics */
    const std::size_t numOfDists = this->mDistributionMetrics.NumOfMetrics();
    ptree distributionTree;

    for (std::size_t i = 0; i < numOfDists; ++i) {
        const DistributionBase* pDist = this->mDistributionMetrics.MetricAt(i);

        const auto sumStr = doubleToString(pDist->Sum());
        const auto meanStr = doubleToString(pDist->Mean());
        const auto stdDevStr = doubleToString(pDist->StandardDeviation());
        const auto maxStr = doubleToString(pDist->Maximum());
        const auto minStr = doubleToString(pDist->Minimum());

        ptree distTree;
        distTree.put("NumOfSamples", pDist->NumOfSamples());
        distTree.put("Sum", sumStr);
        distTree.put("Mean", meanStr);
        distTree.put("StandardDeviation", stdDevStr);
        distTree.put("Maximum", maxStr);
        distTree.put("Minimum", minStr);

        distributionTree.push_back(std::make_pair(pDist->Id(), distTree));
    }

    /* Process histogram metrics */
    const std::size_t numOfHists = this->mHistogramMetrics.NumOfMetrics();
    ptree histogramTree;

    for (std::size_t i = 0; i < numOfHists; ++i) {
        const HistogramBase* pHist = this->mHistogramMetrics.MetricAt(i);

        const auto sumStr = doubleToString(pHist->SumValues());
        const double mean = pHist->NumOfSamples() > 0 ? pHist->Mean() : 0.0;
        const auto meanStr = doubleToString(mean);
        const auto boundariesStr = doubleVecToString(pHist->Boundaries());
        const auto bucketCountsStr = doubleVecToString(pHist->Counts());

        ptree metricTree;
        metricTree.put("NumOfSamples", pHist->NumOfSamples());
        metricTree.put("SumValues", sumStr);
        metricTree.put("Mean", meanStr);
        metricTree.put("BucketBoundaries", boundariesStr);
        metricTree.put("BucketCounts", bucketCountsStr);

        histogramTree.push_back(std::make_pair(pHist->Id(), metricTree));
    }

    /* Process value sequence metrics */
    const std::size_t numOfSeqs = this->mValueSequenceMetrics.NumOfMetrics();
    ptree valueSeqTree;

    for (std::size_t i = 0; i < numOfSeqs; ++i) {
        const ValueSequenceBase* pValueSeq =
            this->mValueSequenceMetrics.MetricAt(i);

        const auto valuesStr = valueSeqToString(pValueSeq);

        ptree metricTree;
        metricTree.put("NumOfValues", pValueSeq->NumOfValues());
        metricTree.put("Values", valuesStr);

        valueSeqTree.push_back(std::make_pair(pValueSeq->Id(), metricTree));
    }

    rootTree.add_child("Counters", counterTree);
    rootTree.add_child("Gauges", gaugeTree);
    rootTree.add_child("Distributions", distributionTree);
    rootTree.add_child("Histograms", histogramTree);
    rootTree.add_child("ValueSequences", valueSeqTree);

    return rootTree;
}

/* Append the counter metric */
CounterBase* MetricManager::AddCounter(const std::string& metricName)
{
    auto* counter = new Counter(metricName);
    this->Append(this->mCounters, counter);
    return counter;
}

/* Append the gauge metric */
GaugeBase* MetricManager::AddGauge(const std::string& metricName)
{
    auto* gauge = new Gauge(metricName);
    this->Append(this->mGauges, gauge);
    return gauge;
}

/* Append the distribution metric */
DistributionBase* MetricManager::AddDistribution(const std::string& metricName)
{
    auto* dist = new Distribution(metricName);
    this->Append(this->mDists, dist);
    return dist;
}

/* Append the histogram metric */
HistogramBase* MetricManager::AddHistogram(
    const std::string& metricName,
    const BucketBoundaries& bucketBoundaries)
{
    auto* hist = new Histogram(metricName, bucketBoundaries);
    this->Append(this->mHists, hist);
    return hist;
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

} /* namespace Metric */
} /* namespace MyLidarGraphSlam */
