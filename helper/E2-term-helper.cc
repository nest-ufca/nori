#include "E2-term-helper.h"

#include "ns3/E2-report.h"
#include "ns3/antenna-module.h"
#include "ns3/config-store.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/grid-scenario-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nstime.h"
#include "ns3/object-map.h"
#include "ns3/object.h"
#include "ns3/oran-interface.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/type-id.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("E2TermHelper");
NS_OBJECT_ENSURE_REGISTERED(E2TermHelper);

namespace
{

/**
 * Encode an MCC/MNC pair as the three-byte TBCD PLMN identity required by
 * E2AP and other 3GPP identifiers.
 *
 * A two-digit MNC uses 0xF as the unused third MNC digit. For example,
 * MCC 001 and MNC 01 are encoded as the bytes 00 F1 10.
 *
 * @param mcc Three-decimal-digit Mobile Country Code.
 * @param mnc Two- or three-decimal-digit Mobile Network Code.
 * @return Binary string containing exactly three encoded PLMN bytes.
 */
std::string
EncodePlmnIdentity(
    const std::string& mcc,
    const std::string& mnc)
{
    NS_ABORT_MSG_UNLESS(
        mcc.size() == 3 &&
            (mnc.size() == 2 ||
             mnc.size() == 3),
        "PLMN requires a three-digit MCC and a two- or three-digit MNC");

    const auto digitValue =
        [](char character) -> uint8_t {
            NS_ABORT_MSG_UNLESS(
                character >= '0' &&
                    character <= '9',
                "MCC and MNC must contain only decimal digits");

            return static_cast<uint8_t>(
                character - '0');
        };

    const uint8_t thirdMncDigit =
        mnc.size() == 3
            ? digitValue(mnc[2])
            : 0x0f;

    std::string encodedPlmn(3, '\0');

    encodedPlmn[0] =
        static_cast<char>(
            (digitValue(mcc[1]) << 4) |
            digitValue(mcc[0]));

    encodedPlmn[1] =
        static_cast<char>(
            (thirdMncDigit << 4) |
            digitValue(mcc[2]));

    encodedPlmn[2] =
        static_cast<char>(
            (digitValue(mnc[1]) << 4) |
            digitValue(mnc[0]));

    return encodedPlmn;
}

} // namespace

E2TermHelper::E2TermHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId
E2TermHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::E2TermHelper")
                            .SetParent<Object>()
                            .AddConstructor<E2TermHelper>()
                            .AddAttribute("E2Term",
                                          "E2 term creation, using the eNB/gNB net device; creates "
                                          "an instance of the E2 term.",
                                          PointerValue(),
                                          MakePointerAccessor(&E2TermHelper::m_e2Term),
                                          MakePointerChecker<E2Termination>())
                            .AddAttribute("E2TermIp",
                                          "The IP address of the RIC E2 termination",
                                          StringValue("10.107.233.133"),
                                          MakeStringAccessor(&E2TermHelper::m_e2ip),
                                          MakeStringChecker())
                            .AddAttribute("E2Port",
                                          "Port number for E2",
                                          UintegerValue(36422),
                                          MakeUintegerAccessor(&E2TermHelper::m_e2port),
                                          MakeUintegerChecker<uint16_t>())
                            .AddAttribute("E2LocalPort",
                                          "The first port number for the local bind",
                                          UintegerValue(38470),
                                          MakeUintegerAccessor(&E2TermHelper::m_e2localPort),
                                          MakeUintegerChecker<uint16_t>())
                            .AddAttribute("Mcc",
                                          "Three-digit Mobile Country Code",
                                          StringValue("001"),
                                          MakeStringAccessor(&E2TermHelper::m_mcc),
                                          MakeStringChecker())
                            .AddAttribute("Mnc",
                                          "Two- or three-digit Mobile Network Code",
                                          StringValue("01"),
                                          MakeStringAccessor(&E2TermHelper::m_mnc),
                                          MakeStringChecker());
    return tid;
}

void
E2TermHelper::InstallE2Term(Ptr<NetDevice> NetDevice)
{
    NS_LOG_FUNCTION(this);

    // Create E2 messages scheduling
    auto e2Messages = CreateObject<E2Interface>(NetDevice);

    m_e2Report = e2Messages->GetE2DuCalculator();

    // NetDevice is a gNB or eNB
    auto nrGnbNetDev = DynamicCast<NrGnbNetDevice>(NetDevice);

    const std::string encodedPlmnId =
        EncodePlmnIdentity(m_mcc, m_mnc);

    // node cell ID
    uint16_t cellId{0};
    // Client local port
    uint16_t localPort{0};

    // Enable E2 traces
    EnableE2PdcpTraces();
    EnableE2RlcTraces();

    if (nrGnbNetDev != nullptr)
    {
        cellId = nrGnbNetDev->GetCellId();
        NS_LOG_DEBUG("Cell ID: " << cellId);
        localPort = m_e2localPort + cellId;
        e2Messages->SetE2PdcpStatsCalculator(m_e2PdcpStats);
        e2Messages->SetE2RlcStatsCalculator(m_e2RlcStats);
    }
    else
    {
        NS_ABORT_MSG("NetDevice is not a gNB or eNB");
    }

    printf(
        "E2TermHelper: PLMN %s-%s "
        "(TBCD %02X %02X %02X), "
        "Cell ID %u, Local Port %u\n",
        m_mcc.c_str(),
        m_mnc.c_str(),
        static_cast<unsigned int>(
            static_cast<unsigned char>(
                encodedPlmnId[0])),
        static_cast<unsigned int>(
            static_cast<unsigned char>(
                encodedPlmnId[1])),
        static_cast<unsigned int>(
            static_cast<unsigned char>(
                encodedPlmnId[2])),
        cellId,
        localPort);

    auto e2Term = CreateObject<E2Termination>(m_e2ip,
                                              m_e2port,
                                              localPort,
                                              std::to_string(cellId),
                                              encodedPlmnId);
    NetDevice->AggregateObject(e2Term);
    e2Messages->SetAttribute("E2Term", PointerValue(e2Term));

    // Connect PDU's packets to callback report, after UEs registration.
    // The schedule ensures that the connection is made after the UEs are registered.
    Simulator::Schedule(Seconds(0.2),
                        &E2TermHelper::ConnectPDUReports,
                        this,
                        NetDevice,
                        e2Messages);
    // Connect PHY traces
    ConnectPhyTraces();
    // Enable SINR traces
    EnableSinrTraces(e2Messages);

    // Connect E2 termination to E2 messages via KPM subscription callback
    Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription>();
    e2Term->RegisterKpmCallbackToE2Sm(200,
                                      kpmFd,
                                      std::bind(&E2Interface::FunctionServiceSubscriptionCallback,
                                                e2Messages,
                                                std::placeholders::_1));

    
    auto ricFd = Create<RicControlFunctionDescription>();
    e2Term->RegisterSmCallbackToE2Sm(300,
                                     ricFd,
                                     std::bind(&E2Interface::ControlMessageReceivedCallback,
                                               e2Messages,
                                               std::placeholders::_1));

    Simulator::Schedule(MicroSeconds(0), &E2Termination::Start, e2Term);

    NetDevice->AggregateObject(e2Messages);
}

void
E2TermHelper::InstallE2Term(NetDeviceContainer& NetDevices)
{
    NS_LOG_FUNCTION(this);
    for (size_t i = 0; i < NetDevices.GetN(); i++)
    {
        InstallE2Term(NetDevices.Get(i));
    }
}

void
E2TermHelper::EnableE2PdcpTraces()
{
    NS_LOG_FUNCTION(this);
    // Enable E2 PDCP traces
    m_e2PdcpStats = CreateObject<NrBearerStatsCalculator>("E2PDCP");
    m_e2PdcpStats->SetAttribute("DlPdcpOutputFilename", StringValue("DlE2PdcpStats.txt"));
    m_e2PdcpStats->SetAttribute("UlPdcpOutputFilename", StringValue("UlE2PdcpStats.txt"));
    m_e2StatsConnector.EnablePdcpStats(m_e2PdcpStats);
}

void
E2TermHelper::EnableE2RlcTraces()
{
    NS_LOG_FUNCTION(this);
    // Enable E2 RLC traces
    m_e2RlcStats = CreateObject<NrBearerStatsCalculator>("E2RLC");
    m_e2RlcStats->SetAttribute("DlRlcOutputFilename", StringValue("DlE2RlcStats.txt"));
    m_e2RlcStats->SetAttribute("UlRlcOutputFilename", StringValue("UlE2RlcStats.txt"));
    m_e2StatsConnector.EnableRlcStats(m_e2RlcStats);
}

void
E2TermHelper::EnableSinrTraces(Ptr<E2Interface> e2Messages)
{
    NS_LOG_FUNCTION(this);
    Config::ConnectFailSafe("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                            MakeCallback(&E2Interface::RegisterNewSinrReadingCallback, e2Messages));
}

void
E2TermHelper::ConnectPDUReports([[maybe_unused]] Ptr<NetDevice> NetDevice,
                                [[maybe_unused]] Ptr<E2Interface> e2Message)
{
    NS_LOG_FUNCTION(this << "Connecting PDU reports in E2 interface");

    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/NrRlc/TxPDU",
        MakeCallback(&E2Interface::ReportTxPDU, e2Message));
}

void
E2TermHelper::ConnectPhyTraces()
{
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/SpectrumPhy/RxPacketTraceUe",
        MakeCallback(&NoriE2Report::UpdateTraces, m_e2Report));
}

} // namespace ns3
