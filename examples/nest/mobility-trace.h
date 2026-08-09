#ifndef NEST_MOBILITY_TRACE_H
#define NEST_MOBILITY_TRACE_H

#include "ns3/node-container.h"

#include <cstdint>
#include <iosfwd>
#include <vector>

namespace ns3
{

void SampleMobilityTrace(const NodeContainer* gNbNodes, const NodeContainer* ueNodes,
                         const std::vector<int>* ueSliceId, const std::vector<uint8_t>* sstPerSlice,
                         double simTime, double interval, std::ofstream* output);

} // namespace ns3

#endif // NEST_MOBILITY_TRACE_H
