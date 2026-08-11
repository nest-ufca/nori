#ifndef NEST_TCP_TRANSPORT_TRACE_H
#define NEST_TCP_TRANSPORT_TRACE_H

#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/tcp-socket-state.h"

#include <cstdint>
#include <iosfwd>
#include <string>

namespace ns3
{

class OnOffApplication;

/**
 * Trace the transport state of one TCP traffic source.
 */
class NestTcpTransportTrace
{
  public:
    NestTcpTransportTrace(std::ofstream* output,
                          uint32_t ueIndex,
                          std::string direction,
                          uint32_t nodeId);

    /**
     * Connect the trace after OnOffApplication has created its TCP socket.
     */
    void Connect(Ptr<OnOffApplication> application);

  private:
    void WriteRow(const std::string& event,
                  const std::string& oldValue,
                  const std::string& newValue,
                  const std::string& unit);

    void CongestionWindowChanged(uint32_t oldValue, uint32_t newValue);
    void SlowStartThresholdChanged(uint32_t oldValue, uint32_t newValue);
    void BytesInFlightChanged(uint32_t oldValue, uint32_t newValue);
    void ReceiverWindowChanged(uint32_t oldValue, uint32_t newValue);
    void RttChanged(Time oldValue, Time newValue);
    void RtoChanged(Time oldValue, Time newValue);

    void CongestionStateChanged(
        TcpSocketState::TcpCongState_t oldValue,
        TcpSocketState::TcpCongState_t newValue);

    std::ofstream* m_output;
    uint32_t m_ueIndex;
    std::string m_direction;
    uint32_t m_nodeId;
};

} // namespace ns3

#endif // NEST_TCP_TRANSPORT_TRACE_H
