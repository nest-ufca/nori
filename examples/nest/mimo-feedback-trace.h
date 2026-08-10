#ifndef NEST_MIMO_FEEDBACK_TRACE_H
#define NEST_MIMO_FEEDBACK_TRACE_H

#include <cstdint>
#include <iosfwd>

namespace ns3
{

void WriteMimoFeedbackTrace(std::ofstream* output, uint32_t ueIndex, uint16_t rnti,
                            uint8_t cqi, uint8_t mcs, uint8_t rank);

} // namespace ns3

#endif // NEST_MIMO_FEEDBACK_TRACE_H
