#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "E2-report.h"
#include "encode_e2apv1.hpp"
#include "oran-interface.h"

#ifdef NORI_ENABLE_RC_V5_CODEC
#include "rc-v5-codec.h"
#endif

#include "ns3/event-id.h"
#include "ns3/nr-bearer-stats-calculator.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-phy-rx-trace.h"

namespace ns3
{
class NoriE2Report;

typedef std::pair<uint64_t, uint16_t> ImsiCellIdPair_t;

/**
 * Identity and slice information of one simulated gNB-DU UE.
 *
 * The explicit mapping avoids assuming that a gNB-CU UE F1AP ID is equal to
 * an ns-3 IMSI. The scenario defines the synthetic F1AP identity exposed
 * through KPM and associates it with the corresponding simulated UE.
 */
struct KpmGnbDuUeContext
{
    uint64_t gnbCuUeF1apId{0};
    uint64_t imsi{0};
    uint8_t sst{0};
};

class E2Interface : public Object
{
  public:
    const static uint16_t E2SM_REPORT_MAX_NEIGH = 8; //<! Maximum number of neighbors

    /**
     * @brief Constructor
     */
    E2Interface();

    /**
     * @brief Constructor
     * @param netDev the net device of the nodeB
     */
    E2Interface(Ptr<NetDevice> netDev);

    /**
     * @brief Destructor
     */
    ~E2Interface() override
    {
    }

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * @brief Function Service Subscription Request callback.
     * This function is triggered whenever a RIC Subscription Request is received.
     *
     * @param pdu request message
     */
    void FunctionServiceSubscriptionCallback(E2AP_PDU_t* sub_req_pdu);

    /**
     * Handle a RIC Subscription Delete Request for the KPM service model.
     *
     * @param pdu subscription deletion request message
     */
    void FunctionServiceSubscriptionDeleteCallback(E2AP_PDU_t* pdu);

    /**
     * @brief Register new SINR reading callback
     * @param path the path
     * @param cellId the cell identifier
     * @param rnti the Radio Network Temporary Identifier
     * @param avgSinr the average SINR
     * @param bwpId the Bandwidth Part Identifier
     */
    void RegisterNewSinrReadingCallback([[maybe_unused]] std::string path,
                                        uint16_t cellId,
                                        uint16_t rnti,
                                        double avgSinr,
                                        uint16_t bwpId);

    /**
     * @brief Register new SINR reading
     */
    void RegisterNewSinrReading(uint16_t imsi, uint16_t cellId, double avgSinr);

    /**
     * Build and send one report for the currently active KPM subscription.
     */
    void BuildAndSendReportMessage();

    /**
     * Start or restart periodic KPM reporting inside the simulator thread.
     */
    void StartKpmReporting();

    /**
     * Cancel the pending periodic KPM report inside the simulator thread.
     */
    void StopKpmReporting();

    /**
     * Start the simulator-thread polling bridge for external KPM requests.
     *
     * This method must be called before Simulator::Run().
     */
    void StartKpmRequestPolling();

    /**
     * Configure the simulated gNB-DU UE identities exposed through KPM.
     *
     * @param contexts mapping between F1AP identities, ns-3 IMSIs and SSTs
     */
    void SetKpmGnbDuUeContexts(
        const std::vector<KpmGnbDuUeContext>& contexts);

    /**
     * @brief Report the number of TX PDU calls
     * @param rnti the current Radio network temporary identifier
     * @param lcid the current cell identifier
     * @param packetSize the size of the packet in bytes
     */
    void ReportTxPDU(uint16_t rnti, uint8_t lcid, uint32_t packetSize);

    /**
     * @brief Set E2 PDCP stats variable report
     */
    void SetE2PdcpStatsCalculator(Ptr<NrBearerStatsCalculator> e2PdcpStatsCalculator);

    /**
     * @brief Set E2 RLC stats variable report
     */
    void SetE2RlcStatsCalculator(Ptr<NrBearerStatsCalculator> e2RlcStatsCalculator);

    /**
     * @brief Control Message Received Callback: A handler that deals with the control message
     * received
     * @param sub_req_pdu the subscription request PDU
     */
    void ControlMessageReceivedCallback(E2AP_PDU_t* sub_req_pdu);

    Ptr<NoriE2Report> GetE2DuCalculator();

    
    void MLSliceInterface(double macPrb, uint64_t imsi);

  private:
    /**
     * Collect, encode and send one KPM v3 Style 5 indication.
     *
     * @return true when the indication was encoded and sent successfully
     */
    bool BuildAndSendKpmV3Style5Report();

    /**
     * @brief Build RIC Indication Header
     * @param plmId PLMN ID
     * @param gnbId gNB ID
     * @param CellId NR cell ID
     * @return the RIC Indication Header
     */

    Ptr<KpmIndicationHeader> BuildRicIndicationHeader(std::string plmId,
                                                      std::string gnbId,
                                                      uint16_t CellId) const;

    /**
     * @brief Get the IMSI string
     * @param imsi the IMSI
     */
    std::string GetImsiString(uint64_t imsi);

    /**
     * @brief Build RIC Indication Message for CU-UP
     * @param plmId PLMN ID
     * @return the RIC Indication Message
     */

    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuUp(std::string plmId);

    /**
     * @brief Build RIC Indication Message for CU-CP
     * @param plmId PLMN ID
     * @return the RIC Indication Message
     *
     */
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuCp(std::string plmId);

    /**
     * @brief Build RIC Indication Message for DU
     * @param plmId PLMN ID
     * @param nrCellId NR cell ID
     * @return the RIC Indication Message
     */
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId);

    /**
     * Process KPM start and stop requests received by the E2Sim thread.
     *
     * The method runs exclusively in the ns-3 simulator thread and
     * reschedules itself periodically.
     */
    void PollKpmRequests();

    /**
     * @brief Function to help us to flip the map
     * @param src the source map
     * @return the flipped map
     */
    std::multimap<long double, uint16_t> FlipMap(const std::map<uint16_t, long double>& src);

    // Explicit mapping from the synthetic gNB-CU UE F1AP ID exposed to the
    // xApp to the corresponding ns-3 IMSI and configured slice SST.
    std::map<uint64_t, KpmGnbDuUeContext> m_kpmGnbDuUeContexts;

    // Requests written by the E2Sim receiver thread and consumed by the
    // simulator-thread polling bridge.
    std::atomic_bool m_kpmStartRequested{false};
    std::atomic_bool m_kpmStopRequested{false};
    EventId m_kpmRequestPollEvent;

#ifdef NORI_ENABLE_RC_V5_CODEC
    struct PendingRcV5ControlRequest
    {
        RcV5ControlRequest control;
        long requestorId{0};
        long instanceId{0};
        long ranFunctionId{0};
        uint64_t callbackReceivedAtUnixNs{0};
        uint64_t queuedAtUnixNs{0};
    };

    // One decoded RC command waiting to be consumed by the ns-3 simulator thread.
    std::mutex m_rcControlMutex;
    std::optional<PendingRcV5ControlRequest> m_pendingRcControl;
#endif

    // State of the single KPM subscription currently managed by this
    // E2Interface instance.
    std::atomic_bool m_kpmSubscriptionActive{false};
    E2Termination::RicSubscriptionRequest_rval_s m_kpmSubscriptionParams{};
    EventId m_kpmReportEvent;

    // KPM v3 measurement selection copied from the accepted Action
    // Definition. Plain C++ values keep generated ASN.1 types outside this
    // public interface.
    uint32_t m_kpmReportStyle{0};
    uint32_t m_kpmReportingPeriodMs{0};
    uint32_t m_kpmGranularityPeriodMs{0};

    std::vector<std::string> m_kpmMeasurementNames;

    std::vector<uint64_t> m_kpmMatchingGnbCuUeF1apIds;

    uint32_t m_kpmIndicationSequenceNumber{0};
    // Absolute timestamp assigned to the beginning of the current simulated KPM collection window.
    uint64_t m_kpmCollectStartTimeUnixNanoseconds{0};

    double m_e2Periodicity;                                          //<! E2 periodicity
    Ptr<NrGnbRrc> m_rrc;                                             //<! RRC object
    std::map<uint64_t, std::map<uint16_t, long double>> m_l3sinrMap; //<! L3 SINR map

    Ptr<E2Termination> m_e2term;                          //<! E2 termination object
    Ptr<NetDevice> m_netDev;                              //<! Net device of the nodeB
    std::map<uint32_t, uint32_t> m_txPDU;                 //<! Number of TX PDU calls
    std::map<uint32_t, uint64_t> m_txPDUBytes;            //<! Number of TX PDU in bytes
    Ptr<NrBearerStatsCalculator> m_e2PdcpStatsCalculator; //<! E2 PDCP stats calculator
    Ptr<NrBearerStatsCalculator> m_e2RlcStatsCalculator;  //<! E2 RLC stats calculator
    Ptr<NoriE2Report> m_e2DuCalculator;                   //<! E2 DU calculator
    uint16_t m_cellId{0};                                 //<! Cell ID
    double m_cellTxDlPackets = 0;                         //<! Number of DL packets
    //double m_cellTxBytes = 0;                             //<! Number of DL bytes
    std::map <uint64_t, double> m_cellTxBytes;                             //<! Number of DL bytes
    
    double m_cellRxBytes = 0;                             //<! Number of UL bytes
    uint64_t m_startTime = 0;                             //<! Start time
    std::map<uint64_t, uint32_t>
        m_drbThrDlPdcpBasedComputationUeid;      //<! DRB throughput DL PDCP in UE IMSI
    std::map<uint64_t, uint32_t> m_drbThrDlUeid; //<! DRB throughput DL in UE ID
    std::string m_duFileName;                    //<! DU file name
    double macPrb;
    std::map<uint64_t, double> m_previousDlTxData;
    std::map<uint64_t, double> m_previousUlTxData;
    std::map<uint64_t, double> m_previousTime;
};
} // namespace ns3
