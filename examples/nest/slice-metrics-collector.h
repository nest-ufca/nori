#ifndef NEST_SLICE_METRICS_COLLECTOR_H
#define NEST_SLICE_METRICS_COLLECTOR_H

#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <vector>
#include <functional>

namespace ns3
{

/**
 * Cumulative FlowMonitor counters associated with one UE.
 *
 * Two consecutive snapshots are subtracted to obtain metrics for one
 * observation window.
 */
struct UeFlowCounters
{
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    double delaySumSeconds{0.0};
};

/**
 * Metrics aggregated for one slice during one observation window.
 */
struct SliceWindowMetrics
{
    uint32_t sliceIndex{0};
    uint8_t sst{0};
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    double delaySumSeconds{0.0};
};

/**
 * State retained between consecutive FlowMonitor observations.
 */
struct SliceMetricsCollectorState
{
    bool initialized{false};
    double previousSampleTime{0.0};
    std::vector<UeFlowCounters> previousUeCounters;
};

using SliceWindowMetricsCallback =
    std::function<void(
        double windowStart,
        double windowEnd,
        const std::vector<SliceWindowMetrics>& metrics)>;

void WriteSliceRbgAllocation(std::ofstream* output,
                             uint32_t gNbIdx,
                             uint8_t bwpId,
                             uint32_t sliceIdx,
                             uint8_t sst,
                             uint32_t allocatedRbg,
                             uint32_t availableRbg);

std::vector<UeFlowCounters>
CollectCurrentDlUeCounters(
    Ptr<FlowMonitor> monitor,
    FlowMonitorHelper* flowmonHelper,
    const std::map<Ipv4Address, uint32_t>& ueIpToIndex,
    const std::map<uint16_t, uint32_t>& downlinkPortToUe,
    uint32_t ueCount);

std::vector<SliceWindowMetrics>
BuildSliceWindowMetrics(
    const std::vector<UeFlowCounters>& currentUeCounters,
    const std::vector<int>& ueSliceId,
    const std::vector<uint8_t>& sstPerSlice,
    double sampleTime,
    SliceMetricsCollectorState* state);

void SampleSliceWindowMetrics(
    Ptr<FlowMonitor> monitor,
    FlowMonitorHelper* flowmonHelper,
    const std::map<Ipv4Address, uint32_t>& ueIpToIndex,
    const std::map<uint16_t, uint32_t>& downlinkPortToUe,
    const std::vector<int>& ueSliceId,
    const std::vector<uint8_t>& sstPerSlice,
    double simTime,
    double interval,
    SliceMetricsCollectorState* state,
    std::ofstream* output,
    SliceWindowMetricsCallback metricsCallback);

} // namespace ns3

#endif // NEST_SLICE_METRICS_COLLECTOR_H
