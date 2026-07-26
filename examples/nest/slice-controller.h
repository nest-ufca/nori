#ifndef NEST_SLICE_CONTROLLER_H
#define NEST_SLICE_CONTROLLER_H

#include "ns3/network-module.h"
#include "ns3/ric-control-message.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

/**
 * One locally generated PRB quota action.
 *
 * The action is applied at applyTime and contains one quota entry for
 * every SST configured in the scenario.
 */
struct LocalPrbQuotaAction
{
    double applyTime;
    std::vector<RicControlMessage::SlicePRBQuota> quotas;
};

/**
 * Apply locally generated slice quotas to every configured gNB.
 */
void ApplyLocalSliceQuotas(
    NetDeviceContainer gNbDevs,
    std::vector<RicControlMessage::SlicePRBQuota> quotas);

/**
 * Parse and validate one local PRB quota action.
 */
LocalPrbQuotaAction
ParseLocalPrbQuotaAction(
    const nlohmann::json& actionJson,
    const std::vector<uint8_t>& sstPerSlice,
    double simTime,
    const std::string& context);

} // namespace ns3

#endif // NEST_SLICE_CONTROLLER_H
