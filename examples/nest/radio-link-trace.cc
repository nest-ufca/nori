#include "radio-link-trace.h"

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace ns3
{

void NestRadioLinkTraceState::Initialize(std::ofstream* output,
                                         const NetDeviceContainer& gNbDevs,
                                         const NetDeviceContainer& ueDevs)
{
    NS_ABORT_MSG_IF(output == nullptr || !output->is_open(),
                    "Radio-link trace output must be open");
    NS_ABORT_MSG_IF(gNbDevs.GetN() == 0, "Radio-link trace requires at least one gNB");
    NS_ABORT_MSG_IF(ueDevs.GetN() == 0, "Radio-link trace requires at least one UE");

    m_output = output;
    m_gnbs.clear();
    m_ues.clear();

    for (uint32_t index = 0; index < gNbDevs.GetN(); ++index)
    {
        Ptr<NrGnbNetDevice> device = DynamicCast<NrGnbNetDevice>(gNbDevs.Get(index));
        NS_ABORT_MSG_UNLESS(device, "Radio-link trace could not cast a gNB device");

        Ptr<MobilityModel> mobility = device->GetNode()->GetObject<MobilityModel>();
        NS_ABORT_MSG_UNLESS(mobility, "Radio-link trace gNB does not contain a mobility model");

        GnbState state;
        state.cellId = device->GetCellId();
        state.mobility = mobility;
        m_gnbs.push_back(state);
    }

    for (uint32_t index = 0; index < ueDevs.GetN(); ++index)
    {
        Ptr<NrUeNetDevice> device = DynamicCast<NrUeNetDevice>(ueDevs.Get(index));
        NS_ABORT_MSG_UNLESS(device, "Radio-link trace could not cast a UE device");

        Ptr<MobilityModel> mobility = device->GetNode()->GetObject<MobilityModel>();
        NS_ABORT_MSG_UNLESS(mobility, "Radio-link trace UE does not contain a mobility model");

        UeState state;
        state.nodeId = device->GetNode()->GetId();
        state.mobility = mobility;
        m_ues.push_back(state);
    }
}

void NestRadioLinkTraceState::UpdatePathloss(uint32_t ueIndex, uint16_t cellId,
                                             uint8_t bwpId, uint32_t nodeId,
                                             double pathlossDb, uint8_t)
{
    NS_ABORT_MSG_IF(ueIndex >= m_ues.size(), "Radio-link pathloss contains an invalid UE index");
    NS_ABORT_MSG_IF(nodeId != m_ues.at(ueIndex).nodeId,
                    "Radio-link pathloss node ID does not match the configured UE");
    NS_ABORT_MSG_IF(!std::isfinite(pathlossDb),
                    "Radio-link pathloss must be finite");

    UeState& state = m_ues.at(ueIndex);
    state.pathlossCellId = cellId;
    state.pathlossBwpId = bwpId;
    state.pathlossDb = pathlossDb;
    state.hasPathloss = true;
}

void NestRadioLinkTraceState::WriteReception(uint32_t ueIndex,
                                             RxPacketTraceParams params)
{
    NS_ABORT_MSG_IF(m_output == nullptr || !m_output->is_open(),
                    "Radio-link trace output must be open");
    NS_ABORT_MSG_IF(ueIndex >= m_ues.size(),
                    "Radio-link reception contains an invalid UE index");

    const UeState& ue = m_ues.at(ueIndex);

    NS_ABORT_MSG_UNLESS(ue.hasPathloss,
                        "Radio-link reception does not have a pathloss observation");
    NS_ABORT_MSG_IF(ue.pathlossCellId != params.m_cellId ||
                        ue.pathlossBwpId != params.m_bwpId,
                    "Radio-link reception does not match the latest pathloss observation");

    auto gnb = std::find_if(m_gnbs.begin(), m_gnbs.end(),
                            [&params](const GnbState& state)
                            {
                                return state.cellId == params.m_cellId;
                            });

    NS_ABORT_MSG_IF(gnb == m_gnbs.end(),
                    "Radio-link reception references an unknown serving cell");

    const Vector uePosition = ue.mobility->GetPosition();
    const Vector gnbPosition = gnb->mobility->GetPosition();
    const double distance = CalculateDistance(uePosition, gnbPosition);
    const double sinrAvgDb = 10.0 * std::log10(params.m_sinr);
    const double sinrMinDb = 10.0 * std::log10(params.m_sinrMin);

    *m_output << std::fixed << std::setprecision(9)
              << Simulator::Now().GetSeconds() << ','
              << ueIndex << ','
              << ue.nodeId << ','
              << params.m_rnti << ','
              << params.m_cellId << ','
              << params.m_bwpId << ','
              << uePosition.x << ','
              << uePosition.y << ','
              << uePosition.z << ','
              << distance << ','
              << ue.pathlossDb << ','
              << sinrAvgDb << ','
              << sinrMinDb << ','
              << +params.m_cqi << ','
              << +params.m_mcs << ','
              << +params.m_rank << ','
              << params.m_rbAssignedNum << ','
              << params.m_tbSize << ','
              << params.m_tbler << ','
              << (params.m_corrupt ? 1 : 0) << '\n';

    m_output->flush();
}

void UpdateRadioLinkPathloss(NestRadioLinkTraceState* state, uint32_t ueIndex,
                             uint16_t cellId, uint8_t bwpId, uint32_t nodeId,
                             double pathlossDb, uint8_t cqi)
{
    NS_ABORT_MSG_IF(state == nullptr, "Radio-link trace state must not be null");
    state->UpdatePathloss(ueIndex, cellId, bwpId, nodeId, pathlossDb, cqi);
}

void WriteRadioLinkReception(NestRadioLinkTraceState* state, uint32_t ueIndex,
                             RxPacketTraceParams params)
{
    NS_ABORT_MSG_IF(state == nullptr, "Radio-link trace state must not be null");
    state->WriteReception(ueIndex, params);
}

} // namespace ns3
