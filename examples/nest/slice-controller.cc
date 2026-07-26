#include "slice-controller.h"

#include "ns3/core-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-rl-mac-scheduler-ofdma.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>
#include <cmath>
#include <utility>
#include <iomanip>
#include <sstream>
#include <exception>

namespace ns3
{

/**
 * Apply locally generated slice quotas to every configured gNB.
 *
 * This bypasses E2SM-RC and is intended to validate the scheduler
 * independently from the RIC/xApp control path.
 */
void
ApplyLocalSliceQuotas(
    NetDeviceContainer gNbDevs,
    std::vector<RicControlMessage::SlicePRBQuota> quotas)
{
    NS_ABORT_MSG_IF(quotas.empty(), "Local slice quota list is empty");

    std::cout << "[LOCAL QUOTA] Applying " << quotas.size()
              << " slice quotas at t=" << Simulator::Now().GetSeconds()
              << "s" << std::endl;

    for (uint32_t gNbIdx = 0; gNbIdx < gNbDevs.GetN(); ++gNbIdx)
    {
        auto gNbDevice = DynamicCast<NrGnbNetDevice>(gNbDevs.Get(gNbIdx));

        NS_ABORT_MSG_UNLESS(gNbDevice,
                            "Could not cast device to NrGnbNetDevice");

        auto scheduler =
            DynamicCast<NrRLMacSchedulerOfdma>(gNbDevice->GetScheduler(0));

        NS_ABORT_MSG_UNLESS(
            scheduler,
            "Local slice quotas require NrRLMacSchedulerOfdma");

        scheduler->SetSlicingParameters(quotas);
    }
}

/**
 * Parse and validate one local PRB quota action.
 */
LocalPrbQuotaAction
ParseLocalPrbQuotaAction(const nlohmann::json& actionJson,
                         const std::vector<uint8_t>& sstPerSlice,
                         double simTime,
                         const std::string& context)
{
    NS_ABORT_MSG_UNLESS(actionJson.is_object(),
                        context + " must be a JSON object");

    LocalPrbQuotaAction action;
    action.applyTime = actionJson.value("applyTime", -1.0);

    NS_ABORT_MSG_UNLESS(
        action.applyTime > 1.0 && action.applyTime < simTime,
        context + ".applyTime must be after slice mapping at 1.0 s "
                  "and before the end of the simulation");

    NS_ABORT_MSG_UNLESS(
        actionJson.contains("quotas") &&
            actionJson["quotas"].is_array(),
        context + ".quotas must be a JSON array");

    std::set<uint32_t> configuredSsts;
    uint32_t totalDedicated = 0;
    uint32_t totalMinimum = 0;

    action.quotas.reserve(sstPerSlice.size());

    for (const auto& quotaJson : actionJson["quotas"])
    {
        NS_ABORT_MSG_UNLESS(
            quotaJson.is_object(),
            context + " contains a quota that is not a JSON object");

        uint32_t sliceId = quotaJson.value("sliceId", 0u);
        long dedicated = quotaJson.value("dedicated", -1L);
        long minimum = quotaJson.value("min", -1L);
        long maximum = quotaJson.value("max", -1L);

        NS_ABORT_MSG_UNLESS(
            sliceId > 0 && sliceId <= 255,
            context + " contains a sliceId outside the SST range 1..255");

        NS_ABORT_MSG_UNLESS(
            std::find(sstPerSlice.begin(),
                      sstPerSlice.end(),
                      static_cast<uint8_t>(sliceId)) != sstPerSlice.end(),
            context + " refers to an SST that is not configured in SstPerSlice");

        NS_ABORT_MSG_UNLESS(
            configuredSsts.insert(sliceId).second,
            context + " contains a duplicate SST");

        NS_ABORT_MSG_UNLESS(
            dedicated >= 0 && dedicated <= minimum &&
                minimum <= maximum && maximum <= 100,
            context + " must satisfy 0 <= dedicated <= min <= max <= 100");

        RicControlMessage::SlicePRBQuota quota;
        quota.sliceId = sliceId;
        quota.dedicatePRBRatio = dedicated;
        quota.minPRBRatio = minimum;
        quota.maxPRBRatio = maximum;

        action.quotas.push_back(quota);

        totalDedicated += static_cast<uint32_t>(dedicated);
        totalMinimum += static_cast<uint32_t>(minimum);
    }

    NS_ABORT_MSG_UNLESS(
        action.quotas.size() == sstPerSlice.size(),
        context + " must provide one quota for every configured SST");

    NS_ABORT_MSG_UNLESS(
        totalDedicated <= 100,
        context + " has a dedicated quota sum greater than 100");

    NS_ABORT_MSG_UNLESS(
        totalMinimum <= 100,
        context + " has a minimum quota sum greater than 100");

    return action;
}

/**
 * Create and validate a local periodic slice controller.
 *
 * The constructor verifies the controller timing, quota limits, slice
 * identifiers, throughput targets and initial quota distribution. It also
 * allocates the internal accumulators used between control decisions.
 */
LocalPeriodicSliceController::LocalPeriodicSliceController(
    LocalSliceControllerConfig config,
    NetDeviceContainer gNbDevs)
    : m_config(std::move(config)),
      m_gNbDevs(gNbDevs)
{
    NS_ABORT_MSG_IF(
        m_gNbDevs.GetN() == 0,
        "Local periodic slice controller requires at least one gNB");

    NS_ABORT_MSG_IF(
        m_config.slices.size() < 2,
        "Local periodic slice controller requires at least two slices");

    NS_ABORT_MSG_IF(
        !std::isfinite(m_config.initialApplyTime) ||
            m_config.initialApplyTime <= 1.0,
        "Controller initialApplyTime must be after "
        "slice mapping at 1.0 seconds");

    NS_ABORT_MSG_IF(
        !std::isfinite(m_config.decisionInterval) ||
            m_config.decisionInterval <= 0.0,
        "Controller decisionInterval must be finite and greater than zero");

    NS_ABORT_MSG_IF(
        m_config.quotaStep == 0 ||
            m_config.quotaStep > 100,
        "Controller quotaStep must be in the range 1..100");

    NS_ABORT_MSG_IF(
        !std::isfinite(m_config.satisfactionHysteresis) ||
            m_config.satisfactionHysteresis < 0.0 ||
            m_config.satisfactionHysteresis >= 1.0,
        "Controller satisfactionHysteresis must be in the range [0, 1)");

    NS_ABORT_MSG_IF(
        m_config.minimumQuota > m_config.maximumQuota ||
            m_config.maximumQuota > 100,
        "Controller quota limits must satisfy "
        "0 <= minimumQuota <= maximumQuota <= 100");

    std::set<uint8_t> configuredSsts;
    uint64_t initialQuotaSum = 0;

    m_currentQuotas.reserve(m_config.slices.size());

    for (const LocalSliceControllerSliceConfig& slice :
         m_config.slices)
    {
        NS_ABORT_MSG_IF(
            slice.sst == 0,
            "Controller slice SST must be greater than zero");

        NS_ABORT_MSG_IF(
            !configuredSsts.insert(slice.sst).second,
            "Controller contains a duplicate SST");

        NS_ABORT_MSG_IF(
            slice.initialQuota < m_config.minimumQuota ||
                slice.initialQuota > m_config.maximumQuota,
            "Controller initial quota is outside the configured limits");

        NS_ABORT_MSG_IF(
            !std::isfinite(slice.throughputTargetMbps) ||
                slice.throughputTargetMbps <= 0.0,
            "Controller throughput target must be finite and "
            "greater than zero");

        m_currentQuotas.push_back(slice.initialQuota);
        initialQuotaSum += slice.initialQuota;
    }

    NS_ABORT_MSG_IF(
        initialQuotaSum != 100,
        "Controller initial slice quotas must sum exactly 100");

    const std::size_t sliceCount =
        m_config.slices.size();

    m_accumulatedTxBytes.assign(sliceCount, 0);
    m_accumulatedRxBytes.assign(sliceCount, 0);
    m_accumulatedRxPackets.assign(sliceCount, 0);
    m_accumulatedDelaySumSeconds.assign(sliceCount, 0.0);
}

/**
 * Convert the controller's current quota state into scheduler quota messages.
 *
 * The locally controlled quotas are fixed allocations: minimum and maximum
 * are assigned the same percentage, while the dedicated percentage is zero.
 */
std::vector<RicControlMessage::SlicePRBQuota>
LocalPeriodicSliceController::BuildCurrentQuotas() const
{
    NS_ABORT_MSG_IF(
        m_currentQuotas.size() != m_config.slices.size(),
        "Controller quota state and slice configuration sizes differ");

    std::vector<RicControlMessage::SlicePRBQuota> quotas;
    quotas.reserve(m_config.slices.size());

    for (std::size_t sliceIndex = 0;
         sliceIndex < m_config.slices.size();
         ++sliceIndex)
    {
        RicControlMessage::SlicePRBQuota quota;

        quota.sliceId =
            m_config.slices[sliceIndex].sst;

        quota.dedicatePRBRatio = 0;

        quota.minPRBRatio =
            m_currentQuotas[sliceIndex];

        quota.maxPRBRatio =
            m_currentQuotas[sliceIndex];

        quotas.push_back(quota);
    }

    return quotas;
}

/**
 * Apply the initial quota distribution to every configured gNB.
 *
 * This method must be called only once, after the UE-to-slice mapping has
 * been installed in the scheduler.
 */
void
LocalPeriodicSliceController::ApplyInitialQuotas()
{
    NS_ABORT_MSG_IF(
        m_initialQuotasApplied,
        "Controller initial quotas were already applied");

    ApplyLocalSliceQuotas(
        m_gNbDevs,
        BuildCurrentQuotas());

    m_initialQuotasApplied = true;
}

/**
 * Return a read-only reference to the controller's current quota distribution.
 */
const std::vector<uint32_t>&
LocalPeriodicSliceController::GetCurrentQuotas() const
{
    return m_currentQuotas;
}

/**
 * Clear all metrics accumulated during the current decision period.
 *
 * The current quota distribution and decision counter are intentionally
 * preserved because only the observation window state is being reset.
 */
void
LocalPeriodicSliceController::ResetObservationPeriod()
{
    std::fill(
        m_accumulatedTxBytes.begin(),
        m_accumulatedTxBytes.end(),
        0);

    std::fill(
        m_accumulatedRxBytes.begin(),
        m_accumulatedRxBytes.end(),
        0);

    std::fill(
        m_accumulatedRxPackets.begin(),
        m_accumulatedRxPackets.end(),
        0);

    std::fill(
        m_accumulatedDelaySumSeconds.begin(),
        m_accumulatedDelaySumSeconds.end(),
        0.0);

    m_observationStart = 0.0;
    m_accumulatedDuration = 0.0;
}

/**
 * Consume one metrics window and execute a control decision when due.
 *
 * Consecutive FlowMonitor windows are accumulated until decisionInterval is
 * reached. The controller then evaluates per-slice satisfaction and may
 * transfer quota between slices.
 */
void
LocalPeriodicSliceController::ObserveWindow(
    double windowStart,
    double windowEnd,
    const std::vector<SliceWindowMetrics>& metrics)
{
    NS_ABORT_MSG_UNLESS(
        m_initialQuotasApplied,
        "Controller received metrics before its initial quotas were applied");

    NS_ABORT_MSG_IF(
        !std::isfinite(windowStart) ||
            !std::isfinite(windowEnd) ||
            windowEnd <= windowStart,
        "Controller received an invalid metrics window");

    NS_ABORT_MSG_IF(
        metrics.size() != m_config.slices.size(),
        "Controller metrics and slice configuration sizes differ");

    const double windowDuration =
        windowEnd - windowStart;

    // Require consecutive observation windows within one decision period.
    if (m_accumulatedDuration == 0.0)
    {
        m_observationStart = windowStart;
    }
    else
    {
        const double expectedWindowStart =
            m_observationStart + m_accumulatedDuration;

        NS_ABORT_MSG_IF(
            std::abs(windowStart - expectedWindowStart) > 1e-6,
            "Controller received non-consecutive metrics windows");
    }

    std::vector<bool> observedSlices(
        m_config.slices.size(),
        false);

    // Aggregate each slice's counters using its internal slice index.
    for (const SliceWindowMetrics& sliceMetrics : metrics)
    {
        const uint32_t sliceIndex =
            sliceMetrics.sliceIndex;

        NS_ABORT_MSG_IF(
            sliceIndex >= m_config.slices.size(),
            "Controller received an invalid internal slice index");

        NS_ABORT_MSG_IF(
            observedSlices[sliceIndex],
            "Controller received duplicate metrics for one slice");

        NS_ABORT_MSG_IF(
            sliceMetrics.sst !=
                m_config.slices[sliceIndex].sst,
            "Controller metric SST does not match its slice configuration");

        NS_ABORT_MSG_IF(
            !std::isfinite(sliceMetrics.delaySumSeconds) ||
                sliceMetrics.delaySumSeconds < 0.0,
            "Controller received an invalid delay sum");

        observedSlices[sliceIndex] = true;

        m_accumulatedTxBytes[sliceIndex] +=
            sliceMetrics.txBytes;

        m_accumulatedRxBytes[sliceIndex] +=
            sliceMetrics.rxBytes;

        m_accumulatedRxPackets[sliceIndex] +=
            sliceMetrics.rxPackets;

        m_accumulatedDelaySumSeconds[sliceIndex] +=
            sliceMetrics.delaySumSeconds;
    }

    m_accumulatedDuration += windowDuration;

    // Wait until enough metric windows have been accumulated.
    if (m_accumulatedDuration + 1e-9 <
        m_config.decisionInterval)
    {
        return;
    }

    ++m_decisionNumber;

    const std::size_t sliceCount =
        m_config.slices.size();

    std::vector<double> offeredMbps(sliceCount, 0.0);
    std::vector<double> throughputMbps(sliceCount, 0.0);
    std::vector<double> meanDelayMs(sliceCount, 0.0);
    std::vector<double> satisfaction(sliceCount, 0.0);

    // Convert accumulated counters into decision-period metrics.
    for (std::size_t sliceIndex = 0;
         sliceIndex < sliceCount;
         ++sliceIndex)
    {
        offeredMbps[sliceIndex] =
            m_accumulatedTxBytes[sliceIndex] * 8.0 /
            m_accumulatedDuration / 1e6;

        throughputMbps[sliceIndex] =
            m_accumulatedRxBytes[sliceIndex] * 8.0 /
            m_accumulatedDuration / 1e6;

        meanDelayMs[sliceIndex] =
            m_accumulatedRxPackets[sliceIndex] > 0
                ? m_accumulatedDelaySumSeconds[sliceIndex] /
                    m_accumulatedRxPackets[sliceIndex] * 1e3
                : 0.0;

        // Do not penalize a slice for traffic that it did not offer.
        const double effectiveTargetMbps =
            std::min(
                m_config.slices[sliceIndex]
                    .throughputTargetMbps,
                offeredMbps[sliceIndex]);

        satisfaction[sliceIndex] =
            effectiveTargetMbps > 1e-9
                ? throughputMbps[sliceIndex] /
                    effectiveTargetMbps
                : 1.0;
    }

    // Locate the worst- and best-served slices.
    std::size_t receiverIndex = 0;
    std::size_t donorIndex = 0;

    for (std::size_t sliceIndex = 1;
         sliceIndex < sliceCount;
         ++sliceIndex)
    {
        if (satisfaction[sliceIndex] <
            satisfaction[receiverIndex])
        {
            receiverIndex = sliceIndex;
        }

        if (satisfaction[sliceIndex] >
            satisfaction[donorIndex])
        {
            donorIndex = sliceIndex;
        }
    }

    std::ostringstream report;

    report << std::fixed << std::setprecision(2)
           << "[LOCAL CONTROLLER] decision="
           << m_decisionNumber
           << " window=["
           << m_observationStart
           << ","
           << windowEnd
           << "] duration="
           << m_accumulatedDuration
           << "s\n";

    for (std::size_t sliceIndex = 0;
         sliceIndex < sliceCount;
         ++sliceIndex)
    {
        report << "  slice="
               << sliceIndex
               << " sst="
               << static_cast<uint32_t>(
                      m_config.slices[sliceIndex].sst)
               << " offered="
               << offeredMbps[sliceIndex]
               << "Mbps throughput="
               << throughputMbps[sliceIndex]
               << "Mbps delay="
               << meanDelayMs[sliceIndex]
               << "ms target="
               << m_config.slices[sliceIndex]
                      .throughputTargetMbps
               << "Mbps satisfaction="
               << satisfaction[sliceIndex]
               << " quota="
               << m_currentQuotas[sliceIndex]
               << "%\n";
    }

    std::cout << report.str();

    const double satisfactionGap =
        satisfaction[donorIndex] -
        satisfaction[receiverIndex];

    const bool receiverNeedsQuota =
        satisfaction[receiverIndex] < 1.0;

    const bool canReceive =
        m_currentQuotas[receiverIndex] <
        m_config.maximumQuota;

    const bool canDonate =
        m_currentQuotas[donorIndex] >
        m_config.minimumQuota;

    // Transfer quota only when the satisfaction difference is meaningful.
    if (receiverIndex != donorIndex &&
        receiverNeedsQuota &&
        satisfactionGap >
            m_config.satisfactionHysteresis &&
        canReceive &&
        canDonate)
    {
        const uint32_t donorCapacity =
            m_currentQuotas[donorIndex] -
            m_config.minimumQuota;

        const uint32_t receiverCapacity =
            m_config.maximumQuota -
            m_currentQuotas[receiverIndex];

        const uint32_t transferredQuota =
            std::min(
                {m_config.quotaStep,
                 donorCapacity,
                 receiverCapacity});

        if (transferredQuota > 0)
        {
            m_currentQuotas[donorIndex] -=
                transferredQuota;

            m_currentQuotas[receiverIndex] +=
                transferredQuota;

            std::cout
                << "[LOCAL CONTROLLER] action: transfer "
                << transferredQuota
                << "% from SST "
                << static_cast<uint32_t>(
                       m_config.slices[donorIndex].sst)
                << " to SST "
                << static_cast<uint32_t>(
                       m_config.slices[receiverIndex].sst)
                << "; new quotas="
                << m_currentQuotas[donorIndex]
                << "/"
                << m_currentQuotas[receiverIndex]
                << "%\n";

            ApplyLocalSliceQuotas(
                m_gNbDevs,
                BuildCurrentQuotas());
        }
    }
    else
    {
        std::cout
            << "[LOCAL CONTROLLER] action: hold quotas"
            << " (satisfaction gap="
            << satisfactionGap
            << ")\n";
    }

    ResetObservationPeriod();
}
} // namespace ns3
