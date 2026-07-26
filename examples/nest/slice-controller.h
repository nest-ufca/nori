#ifndef NEST_SLICE_CONTROLLER_H
#define NEST_SLICE_CONTROLLER_H

#include "ns3/network-module.h"
#include "ns3/ric-control-message.h"
#include "slice-metrics-collector.h"

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

/**
 * Configuration of one slice managed by the local periodic controller.
 */
struct LocalSliceControllerSliceConfig
{
    uint8_t sst{0};
    uint32_t initialQuota{0};
    double throughputTargetMbps{0.0};
};

/**
 * Configuration of the local periodic slice controller.
 */
struct LocalSliceControllerConfig
{
    double initialApplyTime{1.1};
    double decisionInterval{0.5};
    uint32_t quotaStep{10};
    double satisfactionHysteresis{0.1};
    uint32_t minimumQuota{10};
    uint32_t maximumQuota{90};
    std::vector<LocalSliceControllerSliceConfig> slices;
};

/**
 * Local closed-loop controller for slice PRB quotas.
 *
 * FlowMonitor windows are accumulated until decisionInterval is reached.
 * The controller then compares normalized slice satisfaction levels and
 * transfers quota from a better-served slice to a worse-served slice.
 */
class LocalPeriodicSliceController
{
  public:
    LocalPeriodicSliceController(
        LocalSliceControllerConfig config,
        NetDeviceContainer gNbDevs);

    /**
     * Apply the configured initial quota distribution.
     */
    void ApplyInitialQuotas();

    /**
     * Receive one complete observation window from the metrics collector.
     */
    void ObserveWindow(
        double windowStart,
        double windowEnd,
        const std::vector<SliceWindowMetrics>& metrics);

    /**
     * Return the currently configured quota percentages.
     */
    const std::vector<uint32_t>& GetCurrentQuotas() const;

  private:
    std::vector<RicControlMessage::SlicePRBQuota>
    BuildCurrentQuotas() const;

    void ResetObservationPeriod();

    LocalSliceControllerConfig m_config;
    NetDeviceContainer m_gNbDevs;

    std::vector<uint32_t> m_currentQuotas;

    std::vector<uint64_t> m_accumulatedTxBytes;
    std::vector<uint64_t> m_accumulatedRxBytes;
    std::vector<uint64_t> m_accumulatedRxPackets;
    std::vector<double> m_accumulatedDelaySumSeconds;

    double m_observationStart{0.0};
    double m_accumulatedDuration{0.0};
    uint64_t m_decisionNumber{0};
    bool m_initialQuotasApplied{false};
};
} // namespace ns3

#endif // NEST_SLICE_CONTROLLER_H
