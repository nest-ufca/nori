#ifndef NEST_RADIO_LINK_TRACE_H
#define NEST_RADIO_LINK_TRACE_H

#include "ns3/mobility-model.h"
#include "ns3/net-device-container.h"
#include "ns3/nr-phy-mac-common.h"

#include <cstdint>
#include <iosfwd>
#include <vector>

namespace ns3
{

class NestRadioLinkTraceState
{
  public:
    void Initialize(std::ofstream* output, const NetDeviceContainer& gNbDevs,
                    const NetDeviceContainer& ueDevs);

    void UpdatePathloss(uint32_t ueIndex, uint16_t cellId, uint8_t bwpId,
                        uint32_t nodeId, double pathlossDb, uint8_t cqi);

    void WriteReception(uint32_t ueIndex, RxPacketTraceParams params);

  private:
    struct GnbState
    {
        uint16_t cellId{0};
        Ptr<MobilityModel> mobility;
    };

    struct UeState
    {
        uint32_t nodeId{0};
        Ptr<MobilityModel> mobility;
        uint16_t pathlossCellId{0};
        uint8_t pathlossBwpId{0};
        double pathlossDb{0.0};
        bool hasPathloss{false};
    };

    std::ofstream* m_output{nullptr};
    std::vector<GnbState> m_gnbs;
    std::vector<UeState> m_ues;
};

void UpdateRadioLinkPathloss(NestRadioLinkTraceState* state, uint32_t ueIndex,
                             uint16_t cellId, uint8_t bwpId, uint32_t nodeId,
                             double pathlossDb, uint8_t cqi);

void WriteRadioLinkReception(NestRadioLinkTraceState* state, uint32_t ueIndex,
                             RxPacketTraceParams params);

} // namespace ns3

#endif // NEST_RADIO_LINK_TRACE_H
