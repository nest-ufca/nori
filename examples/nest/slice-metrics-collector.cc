#include "slice-metrics-collector.h"

#include "ns3/core-module.h"

#include <iomanip>

namespace ns3
{
void
WriteSliceRbgAllocation(std::ofstream* output,
                        uint32_t gNbIdx,
                        uint8_t bwpId,
                        uint32_t sliceIdx,
                        uint8_t sst,
                        uint32_t allocatedRbg,
                        uint32_t availableRbg)
{
    NS_ASSERT(output);
    NS_ASSERT(output->is_open());

    *output << Simulator::Now().GetNanoSeconds() << ","
            << gNbIdx << ","
            << static_cast<uint32_t>(bwpId) << ","
            << sliceIdx << ","
            << static_cast<uint32_t>(sst) << ","
            << allocatedRbg << ","
            << availableRbg << "\n";
}

/**
 * Read the current cumulative downlink FlowMonitor counters per UE.
 *
 * Only flows whose destination belongs to the UE network are considered.
 * Infrastructure and uplink flows are therefore excluded.
 */
std::vector<UeFlowCounters>
CollectCurrentDlUeCounters(
    Ptr<FlowMonitor> monitor,
    FlowMonitorHelper* flowmonHelper,
    const std::map<Ipv4Address, uint32_t>& ueIpToIndex,
    Ipv4Address ueNetworkAddress,
    Ipv4Mask ueNetworkMask,
    uint16_t echoPort,
    uint32_t ueCount)
{
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper->GetClassifier());

    NS_ABORT_MSG_UNLESS(classifier,
                        "Could not obtain the IPv4 FlowMonitor classifier");

    const std::map<FlowId, FlowMonitor::FlowStats> statsMap =
        monitor->GetFlowStats();

    std::vector<UeFlowCounters> counters(ueCount);

    for (const auto& [flowId, stats] : statsMap)
    {
        const Ipv4FlowClassifier::FiveTuple tuple =
            classifier->FindFlow(flowId);

        const bool isDownlinkToUe =
            ueNetworkMask.IsMatch(tuple.destinationAddress,
                                ueNetworkAddress);

        if (!isDownlinkToUe)
        {
            continue;
        }

        if (tuple.sourcePort == echoPort ||
            tuple.destinationPort == echoPort)
        {
            continue;
        }

        const auto ueIndexIt =
            ueIpToIndex.find(tuple.destinationAddress);

        if (ueIndexIt == ueIpToIndex.end())
        {
            continue;
        }

        const uint32_t ueIndex = ueIndexIt->second;

        NS_ABORT_MSG_IF(
            ueIndex >= counters.size(),
            "UE index obtained from IP mapping is outside the counter vector");

        UeFlowCounters& ueCounters = counters[ueIndex];

        ueCounters.txPackets += stats.txPackets;
        ueCounters.rxPackets += stats.rxPackets;
        ueCounters.txBytes += stats.txBytes;
        ueCounters.rxBytes += stats.rxBytes;
        ueCounters.delaySumSeconds += stats.delaySum.GetSeconds();
    }

    return counters;
}

/**
 * Calculate per-slice counter deltas between two consecutive samples.
 *
 * An empty vector is returned for the first sample because it is used
 * only as the baseline for the next observation window.
 */
std::vector<SliceWindowMetrics>
BuildSliceWindowMetrics(
    const std::vector<UeFlowCounters>& currentUeCounters,
    const std::vector<int>& ueSliceId,
    const std::vector<uint8_t>& sstPerSlice,
    double sampleTime,
    SliceMetricsCollectorState* state)
{
    NS_ABORT_MSG_IF(state == nullptr,
                    "Slice metrics collector state is null");

    NS_ABORT_MSG_IF(
        currentUeCounters.size() != ueSliceId.size(),
        "UE counter and UE-to-slice mapping sizes are different");

    NS_ABORT_MSG_IF(sstPerSlice.empty(),
                    "No SST is configured for slice metric aggregation");

    if (!state->initialized)
    {
        state->initialized = true;
        state->previousSampleTime = sampleTime;
        state->previousUeCounters = currentUeCounters;
        return {};
    }

    NS_ABORT_MSG_IF(
        sampleTime <= state->previousSampleTime,
        "Slice metric sample time must increase");

    NS_ABORT_MSG_IF(
        state->previousUeCounters.size() != currentUeCounters.size(),
        "The number of UE counters changed between samples");

    std::vector<SliceWindowMetrics> sliceMetrics(sstPerSlice.size());

    for (uint32_t sliceIndex = 0;
        sliceIndex < sliceMetrics.size();
        ++sliceIndex)
    {
        sliceMetrics[sliceIndex].sliceIndex = sliceIndex;
        sliceMetrics[sliceIndex].sst = sstPerSlice[sliceIndex];
    }

    for (uint32_t ueIndex = 0;
        ueIndex < currentUeCounters.size();
        ++ueIndex)
    {
        const int sliceIndex = ueSliceId[ueIndex];

        NS_ABORT_MSG_IF(
            sliceIndex < 0 ||
                sliceIndex >= static_cast<int>(sliceMetrics.size()),
            "UE does not have a valid internal slice index");

        const UeFlowCounters& current =
            currentUeCounters[ueIndex];

        const UeFlowCounters& previous =
            state->previousUeCounters[ueIndex];

        NS_ABORT_MSG_IF(
            current.txPackets < previous.txPackets ||
                current.rxPackets < previous.rxPackets ||
                current.txBytes < previous.txBytes ||
                current.rxBytes < previous.rxBytes ||
                current.delaySumSeconds < previous.delaySumSeconds,
            "FlowMonitor counters decreased between samples");

        SliceWindowMetrics& metrics =
            sliceMetrics[static_cast<uint32_t>(sliceIndex)];

        metrics.txPackets +=
            current.txPackets - previous.txPackets;

        metrics.rxPackets +=
            current.rxPackets - previous.rxPackets;

        metrics.txBytes +=
            current.txBytes - previous.txBytes;

        metrics.rxBytes +=
            current.rxBytes - previous.rxBytes;

        metrics.delaySumSeconds +=
            current.delaySumSeconds - previous.delaySumSeconds;
    }

    state->previousSampleTime = sampleTime;
    state->previousUeCounters = currentUeCounters;

    return sliceMetrics;
}

/**
 * Periodically sample FlowMonitor and write per-slice window metrics.
 *
 * The first invocation establishes the cumulative counter baseline.
 * Subsequent invocations produce one observation window each.
 */
void
SampleSliceWindowMetrics(
    Ptr<FlowMonitor> monitor,
    FlowMonitorHelper* flowmonHelper,
    const std::map<Ipv4Address, uint32_t>& ueIpToIndex,
    const std::vector<int>& ueSliceId,
    const std::vector<uint8_t>& sstPerSlice,
    Ipv4Address ueNetworkAddress,
    Ipv4Mask ueNetworkMask,
    uint16_t echoPort,
    double simTime,
    double interval,
    SliceMetricsCollectorState* state,
    std::ofstream* output,
    SliceWindowMetricsCallback metricsCallback)
{
    NS_ABORT_MSG_IF(interval <= 0.0,
                    "Slice metric interval must be greater than zero");

    NS_ABORT_MSG_IF(state == nullptr,
                    "Slice metrics collector state is null");

    NS_ABORT_MSG_IF(output != nullptr && !output->is_open(),
                    "Slice metric output stream is not open");

    const double sampleTime =
        Simulator::Now().GetSeconds();

    if (sampleTime > simTime + 1e-9)
    {
        return;
    }

    const std::vector<UeFlowCounters> currentUeCounters =
        CollectCurrentDlUeCounters(
            monitor,
            flowmonHelper,
            ueIpToIndex,
            ueNetworkAddress,
            ueNetworkMask,
            echoPort,
            static_cast<uint32_t>(ueSliceId.size()));

    const bool baselineAvailable = state->initialized;
    const double windowStart = state->previousSampleTime;

    const std::vector<SliceWindowMetrics> sliceMetrics =
        BuildSliceWindowMetrics(
            currentUeCounters,
            ueSliceId,
            sstPerSlice,
            sampleTime,
            state);

    if (baselineAvailable)
    {
        const double windowDuration =
            sampleTime - windowStart;

        NS_ABORT_MSG_IF(
            windowDuration <= 0.0,
            "Slice metric observation window has invalid duration");

        if (output != nullptr)
        {
            for (const SliceWindowMetrics& metrics : sliceMetrics)
            {
                const double offeredMbps =
                    metrics.txBytes * 8.0 /
                    windowDuration / 1e6;

                const double throughputMbps =
                    metrics.rxBytes * 8.0 /
                    windowDuration / 1e6;

                const double meanDelayMs =
                    metrics.rxPackets > 0
                        ? metrics.delaySumSeconds /
                            metrics.rxPackets * 1e3
                        : 0.0;

                *output << std::fixed << std::setprecision(6)
                        << windowStart << ","
                        << sampleTime << ","
                        << windowDuration << ","
                        << metrics.sliceIndex << ","
                        << static_cast<uint32_t>(metrics.sst) << ","
                        << metrics.txPackets << ","
                        << metrics.rxPackets << ","
                        << metrics.txBytes << ","
                        << metrics.rxBytes << ","
                        << offeredMbps << ","
                        << throughputMbps << ","
                        << meanDelayMs << "\n";
            }

            output->flush();
        }

        if (metricsCallback)
        {
            metricsCallback(
                windowStart,
                sampleTime,
                sliceMetrics);
        }
    }

    if (sampleTime + interval <= simTime + 1e-9)
    {
        Simulator::Schedule(
            Seconds(interval),
            &SampleSliceWindowMetrics,
            monitor,
            flowmonHelper,
            ueIpToIndex,
            ueSliceId,
            sstPerSlice,
            ueNetworkAddress,
            ueNetworkMask,
            echoPort,
            simTime,
            interval,
            state,
            output,
            metricsCallback);
    }
}
} // namespace ns3
