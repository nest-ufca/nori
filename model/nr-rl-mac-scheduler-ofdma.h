// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-mac-scheduler-ofdma.h"
#include "ns3/ric-control-message.h"
#include "ns3/traced-callback.h"
#include "ns3/traced-value.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace ns3
{

/**
 * @ingroup scheduler
 * @brief OFDMA scheduler that enforces per-slice PRB quota bounds.
 *
 * The historical class name contains "RL", but this scheduler does not
 * embed a learning agent. It applies dedicated, minimum and maximum quotas
 * supplied by static actions, the local feedback controller or E2 control.
 */
class NrRLMacSchedulerOfdma : public NrMacSchedulerOfdmaRR
{
  public:
    /**
     * @return The runtime type information for this scheduler.
     */
    static TypeId GetTypeId();

    /**
     * @brief Construct an empty quota-aware slicing scheduler.
     */
    NrRLMacSchedulerOfdma();

    /**
     * @brief Destructor.
     */
    ~NrRLMacSchedulerOfdma() override
    {
    }

    /**
     * @brief Replace the current per-slice PRB quota constraints.
     *
     * Each quota identifies a slice by SST and provides dedicated, minimum
     * and maximum PRB percentages. The decision source is external to this
     * scheduler.
     *
     * @param quotas Complete quota set to install.
     */
    void SetSlicingParameters(
        const std::vector<RicControlMessage::SlicePRBQuota>& quotas);

    /**
     * @brief Install the association between internal slices and UE RNTIs.
     *
     * @param numSlices Number of configured slices.
     * @param sliceUeRnti UE RNTIs grouped by internal slice index.
     */
    void SetSliceUeMapping(
        uint32_t numSlices,
        const std::vector<std::vector<uint32_t>>& sliceUeRnti);

    /**
     * Trace signature for per-slice DL RBG allocation.
     *
     * Arguments: internal slice index, SST, allocated RBG scheduling
     * units, and total available RBG scheduling units.
     */
    typedef void (*SliceRbgAllocationTracedCallback)(uint32_t sliceIdx,
                                                     uint8_t sst,
                                                     uint32_t allocatedRbg,
                                                     uint32_t availableRbg);

  protected:
    /**
     * @brief Create an UE representation aware of RAN slicing (SST lookup).
     *
     * This overrides the RR default and instantiates the historically
     * named NrMacSchedulerUeInfoRl representation so the scheduler and
     * E2/KPM paths can obtain the SST associated with an RNTI.
     */
    std::shared_ptr<NrMacSchedulerUeInfo> CreateUeRepresentation(
        const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const override;

    BeamSymbolMap AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const override;

  private:
    uint32_t m_numberSlices; //!< Number of slices
    std::vector<uint32_t> m_dedicatedRbPercSlices; //!< Dedicated RB percentage per slice
    std::vector<uint32_t> m_minRbPercSlices; //!< Minimum RB percentage per slice
    std::vector<uint32_t> m_maxRbPercSlices; //!< Maximum RB percentage per slice
    std::vector<std::vector<uint32_t>> m_sliceUeRnti; //!< UE RNTI per slice

    TracedValue<uint32_t> m_tracedValueSymPerBeam;

    mutable TracedCallback<uint32_t, uint8_t, uint32_t, uint32_t> m_sliceRbgAllocationTrace;
};
} // namespace ns3
