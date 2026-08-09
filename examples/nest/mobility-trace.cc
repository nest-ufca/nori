#include "mobility-trace.h"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>

namespace ns3
{

namespace
{

void WritePositionRow(std::ofstream* output, double time, const char* nodeType, uint32_t nodeIndex,
                      int sliceIndex, int sst, const Vector& position)
{
    *output << time << ',' << nodeType << ',' << nodeIndex << ',';

    if (sliceIndex >= 0)
    {
        *output << sliceIndex << ',' << sst << ',';
    }
    else
    {
        *output << ",,";
    }

    *output << position.x << ',' << position.y << ',' << position.z << '\n';
}

} // namespace

void SampleMobilityTrace(const NodeContainer* gNbNodes, const NodeContainer* ueNodes,
                         const std::vector<int>* ueSliceId, const std::vector<uint8_t>* sstPerSlice,
                         double simTime, double interval, std::ofstream* output)
{
    NS_ABORT_MSG_IF(gNbNodes == nullptr || ueNodes == nullptr, "Mobility trace node containers must not be null");
    NS_ABORT_MSG_IF(ueSliceId == nullptr || sstPerSlice == nullptr, "Mobility trace slice metadata must not be null");
    NS_ABORT_MSG_IF(output == nullptr || !output->is_open(), "Mobility trace output must be open");
    NS_ABORT_MSG_IF(!std::isfinite(interval) || interval <= 0.0, "Mobility trace interval must be finite and greater than zero");

    const double sampleTime = Simulator::Now().GetSeconds();

    *output << std::fixed << std::setprecision(9);

    for (uint32_t nodeIndex = 0; nodeIndex < gNbNodes->GetN(); ++nodeIndex)
    {
        Ptr<MobilityModel> mobility = gNbNodes->Get(nodeIndex)->GetObject<MobilityModel>();
        NS_ABORT_MSG_UNLESS(mobility, "gNB node does not contain a mobility model");

        WritePositionRow(output, sampleTime, "gnb", nodeIndex, -1, -1, mobility->GetPosition());
    }

    NS_ABORT_MSG_IF(ueSliceId->size() != ueNodes->GetN(), "UE slice metadata size does not match the UE node count");

    for (uint32_t nodeIndex = 0; nodeIndex < ueNodes->GetN(); ++nodeIndex)
    {
        Ptr<MobilityModel> mobility = ueNodes->Get(nodeIndex)->GetObject<MobilityModel>();
        NS_ABORT_MSG_UNLESS(mobility, "UE node does not contain a mobility model");

        const int sliceIndex = ueSliceId->at(nodeIndex);
        NS_ABORT_MSG_IF(sliceIndex < 0 || static_cast<size_t>(sliceIndex) >= sstPerSlice->size(), "UE mobility trace contains an invalid slice index");

        WritePositionRow(output, sampleTime, "ue", nodeIndex, sliceIndex, sstPerSlice->at(sliceIndex), mobility->GetPosition());
    }

    output->flush();

    if (sampleTime + interval <= simTime - 1e-9)
    {
        Simulator::Schedule(Seconds(interval), &SampleMobilityTrace, gNbNodes, ueNodes, ueSliceId,
                            sstPerSlice, simTime, interval, output);
    }
}

} // namespace ns3
