#include "slice-controller.h"

#include "ns3/core-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-rl-mac-scheduler-ofdma.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>

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
} // namespace ns3
