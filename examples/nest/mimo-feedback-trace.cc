#include "mimo-feedback-trace.h"

#include "ns3/core-module.h"

#include <fstream>
#include <iomanip>

namespace ns3
{

void WriteMimoFeedbackTrace(std::ofstream* output, uint32_t ueIndex, uint16_t rnti,
                            uint8_t cqi, uint8_t mcs, uint8_t rank)
{
    NS_ABORT_MSG_IF(output == nullptr || !output->is_open(),
                    "MIMO feedback trace output must be open");

    *output << std::fixed << std::setprecision(9) << Simulator::Now().GetSeconds() << ','
            << ueIndex << ',' << rnti << ',' << +cqi << ',' << +mcs << ',' << +rank << '\n';

    output->flush();
}

} // namespace ns3
