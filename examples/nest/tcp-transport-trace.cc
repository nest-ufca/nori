#include "tcp-transport-trace.h"

#include "ns3/core-module.h"
#include "ns3/onoff-application.h"
#include "ns3/tcp-socket-base.h"

#include <fstream>
#include <iomanip>
#include <utility>

namespace ns3
{

NestTcpTransportTrace::NestTcpTransportTrace(std::ofstream* output,
                                             uint32_t ueIndex,
                                             std::string direction,
                                             uint32_t nodeId)
    : m_output(output),
      m_ueIndex(ueIndex),
      m_direction(std::move(direction)),
      m_nodeId(nodeId)
{
}

void
NestTcpTransportTrace::Connect(Ptr<OnOffApplication> application)
{
    NS_ABORT_MSG_UNLESS(application,
                        "TCP transport trace requires an OnOffApplication");

    Ptr<TcpSocketBase> socket =
        DynamicCast<TcpSocketBase>(application->GetSocket());

    NS_ABORT_MSG_UNLESS(
        socket,
        "TCP transport trace could not retrieve the source TCP socket");

    const bool cwndConnected =
        socket->TraceConnectWithoutContext(
            "CongestionWindow",
            MakeCallback(
                &NestTcpTransportTrace::CongestionWindowChanged,
                this));

    const bool ssthreshConnected =
        socket->TraceConnectWithoutContext(
            "SlowStartThreshold",
            MakeCallback(
                &NestTcpTransportTrace::SlowStartThresholdChanged,
                this));

    const bool bytesInFlightConnected =
        socket->TraceConnectWithoutContext(
            "BytesInFlight",
            MakeCallback(
                &NestTcpTransportTrace::BytesInFlightChanged,
                this));

    const bool receiverWindowConnected =
        socket->TraceConnectWithoutContext(
            "RWND",
            MakeCallback(
                &NestTcpTransportTrace::ReceiverWindowChanged,
                this));

    const bool rttConnected =
        socket->TraceConnectWithoutContext(
            "RTT",
            MakeCallback(
                &NestTcpTransportTrace::RttChanged,
                this));

    const bool rtoConnected =
        socket->TraceConnectWithoutContext(
            "RTO",
            MakeCallback(
                &NestTcpTransportTrace::RtoChanged,
                this));

    const bool congestionStateConnected =
        socket->TraceConnectWithoutContext(
            "CongState",
            MakeCallback(
                &NestTcpTransportTrace::CongestionStateChanged,
                this));

    NS_ABORT_MSG_UNLESS(
        cwndConnected &&
            ssthreshConnected &&
            bytesInFlightConnected &&
            receiverWindowConnected &&
            rttConnected &&
            rtoConnected &&
            congestionStateConnected,
        "Could not connect every TCP transport trace source");

    WriteRow("connected", "", "", "");
}

void
NestTcpTransportTrace::WriteRow(const std::string& event,
                                const std::string& oldValue,
                                const std::string& newValue,
                                const std::string& unit)
{
    NS_ABORT_MSG_IF(
        m_output == nullptr || !m_output->is_open(),
        "TCP transport trace output must be open");

    *m_output
        << std::fixed
        << std::setprecision(9)
        << Simulator::Now().GetSeconds() << ','
        << m_ueIndex << ','
        << m_direction << ','
        << m_nodeId << ','
        << event << ','
        << oldValue << ','
        << newValue << ','
        << unit << '\n';
}

void
NestTcpTransportTrace::CongestionWindowChanged(uint32_t oldValue,
                                                uint32_t newValue)
{
    WriteRow("cwnd",
             std::to_string(oldValue),
             std::to_string(newValue),
             "bytes");
}

void
NestTcpTransportTrace::SlowStartThresholdChanged(uint32_t oldValue,
                                                 uint32_t newValue)
{
    WriteRow("ssthresh",
             std::to_string(oldValue),
             std::to_string(newValue),
             "bytes");
}

void
NestTcpTransportTrace::BytesInFlightChanged(uint32_t oldValue,
                                            uint32_t newValue)
{
    WriteRow("bytes_in_flight",
             std::to_string(oldValue),
             std::to_string(newValue),
             "bytes");
}

void
NestTcpTransportTrace::ReceiverWindowChanged(uint32_t oldValue,
                                             uint32_t newValue)
{
    WriteRow("receiver_window",
             std::to_string(oldValue),
             std::to_string(newValue),
             "bytes");
}

void
NestTcpTransportTrace::RttChanged(Time oldValue, Time newValue)
{
    WriteRow("rtt",
             std::to_string(oldValue.GetNanoSeconds()),
             std::to_string(newValue.GetNanoSeconds()),
             "ns");
}

void
NestTcpTransportTrace::RtoChanged(Time oldValue, Time newValue)
{
    WriteRow("rto",
             std::to_string(oldValue.GetNanoSeconds()),
             std::to_string(newValue.GetNanoSeconds()),
             "ns");
}

void
NestTcpTransportTrace::CongestionStateChanged(
    TcpSocketState::TcpCongState_t oldValue,
    TcpSocketState::TcpCongState_t newValue)
{
    WriteRow(
        "congestion_state",
        TcpSocketState::TcpCongStateName[oldValue],
        TcpSocketState::TcpCongStateName[newValue],
        "");
}

} // namespace ns3
