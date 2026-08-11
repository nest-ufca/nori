// NR eMBB/URLLC scenario with EPC/PGW (end-to-end IP UE <-> remoteHost)
#include "nest/scenario-config.h"
#include "nest/slice-metrics-collector.h"
#include "nest/slice-controller.h"
#include "nest/mimo-feedback-trace.h"
#include "nest/mobility-trace.h"
#include "nest/radio-link-trace.h"
#include "nest/tcp-transport-trace.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/E2-term-helper.h"
#include "ns3/E2-interface.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/three-gpp-antenna-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-rl-mac-scheduler-ofdma.h"
#include "ns3/ric-control-message.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nori-slicing-helper.h"

#include <iomanip>
#include <iostream>
#include <map>

#include <vector>
#include <numeric>
#include <cstdint>

#include <set>
#include <algorithm>
#include <fstream>
#include <limits>
#include <memory>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("nest-embb-urllc-slicing");

namespace
{

/**
 * Parse an explicitly provided boolean command-line override.
 */
bool ParseBooleanCommandLineOverride(const std::string& value, const std::string& optionName)
{
    if (value == "true" || value == "1")
    {
        return true;
    }

    if (value == "false" || value == "0")
    {
        return false;
    }

    NS_FATAL_ERROR(optionName << " must be true, false, 1 or 0");

    return false;
}

} // namespace

/**
 * Configure and execute the NEST eMBB/URLLC slicing scenario.
 *
 * The scenario creates an end-to-end NR network with EPC, installs one or
 * more traffic profiles, maps UEs to slices, optionally enables local slice
 * control, collects metrics and prints post-simulation flow statistics.
 */
int main(int argc, char* argv[])
{
    LogComponentEnable("nest-embb-urllc-slicing", LOG_LEVEL_INFO);
    // Enable scenario and E2 diagnostic logs.
    LogComponentEnable("E2Interface", LOG_LEVEL_INFO);
    LogComponentEnable("E2Termination", LOG_LEVEL_INFO);
    //LogComponentEnable("NrRLMacSchedulerOfdma", LOG_LEVEL_INFO);

    // Execution-level options controlled from the command line.
    // Network topology, radio parameters, slices and traffic profiles are
    // loaded from the JSON configuration file.
    std::string configFilePath = "contrib/nori/examples/config.json";

    bool enableRanSlicing = true;

    std::string enableE2Override;
    std::string ipE2TermRic;
    uint32_t e2TermPortOverride{0};
    uint32_t e2LocalPortBaseOverride{0};
    std::string e2RealtimeOverride;

    std::string rbgTraceFilePath;
    std::ofstream rbgTraceStream;

    std::string mobilityTraceFilePath;
    std::ofstream mobilityTraceStream;
    double mobilityTraceInterval = 0.1;

    std::string mimoTraceFilePath;
    std::ofstream mimoTraceStream;

    std::string radioLinkTraceFilePath;
    std::ofstream radioLinkTraceStream;
    NestRadioLinkTraceState radioLinkTraceState;

    std::string tcpTransportTraceFilePath;
    std::ofstream tcpTransportTraceStream;
    std::vector<std::unique_ptr<NestTcpTransportTrace>>
        tcpTransportTraces;

    std::string sliceMetricsFilePath;
    std::ofstream sliceMetricsStream;
    SliceMetricsCollectorState sliceMetricsState;

    double sliceMetricsInterval = 0.1;

    CommandLine cmd;

    cmd.AddValue("configFile", "Path to the scenario configuration file", configFilePath);

    cmd.AddValue("enableRanSlicing", "Enable RAN Slicing with RL scheduler", enableRanSlicing);

    cmd.AddValue("enableE2", "Override e2.enabled from JSON; true or false", enableE2Override);

    cmd.AddValue("ipE2TermRic", "Override e2.termAddress from JSON", ipE2TermRic);

    cmd.AddValue("e2TermPort", "Override e2.termPort from JSON; zero keeps the JSON value", e2TermPortOverride);

    cmd.AddValue("e2LocalPortBase", "Override e2.localPortBase from JSON; zero keeps the JSON value", e2LocalPortBaseOverride);

    cmd.AddValue("e2Realtime", "Override e2.realtime from JSON; true or false", e2RealtimeOverride);

    cmd.AddValue("rbgTraceFile", "CSV output path for per-slice RBG allocation; empty disables the trace", rbgTraceFilePath);

    cmd.AddValue("mobilityTraceFile", "CSV output path for periodic gNB and UE positions; empty disables the trace", mobilityTraceFilePath);

    cmd.AddValue("mobilityTraceInterval", "Mobility trace sampling interval in seconds", mobilityTraceInterval);

    cmd.AddValue("mimoTraceFile", "CSV output path for per-UE CQI, MCS and rank feedback; empty disables the trace", mimoTraceFilePath);

    cmd.AddValue("radioLinkTraceFile", "CSV output path for correlated position, pathloss and PHY reception measurements; empty disables the trace", radioLinkTraceFilePath);

    cmd.AddValue("tcpTransportTraceFile", "CSV output path for TCP congestion-window and transport-state events; empty disables the trace", tcpTransportTraceFilePath);

    cmd.AddValue("sliceMetricsFile", "CSV output path for per-slice downlink window metrics; empty disables collection", sliceMetricsFilePath);

    cmd.AddValue("sliceMetricsInterval", "Duration of each slice metric observation window in seconds", sliceMetricsInterval);

    cmd.Parse(argc, argv);

    // Load and validate all JSON-controlled network, radio, traffic and
    // slicing parameters after parsing the selected configuration path.
    const NestScenarioConfig scenarioConfig = LoadNestScenarioConfig(configFilePath, enableRanSlicing);

    NS_ABORT_MSG_IF(!mimoTraceFilePath.empty() && !scenarioConfig.mimo.enabled, "MIMO feedback trace requires mimo.enabled=true");

    const bool hasTcpTraffic =
        std::any_of(
            scenarioConfig.trafficProfiles.begin(),
            scenarioConfig.trafficProfiles.end(),
            [](const auto& entry)
            {
                return entry.second.protocol ==
                       NestTrafficProtocol::TCP;
            });

    NS_ABORT_MSG_IF(
        !tcpTransportTraceFilePath.empty() && !hasTcpTraffic,
        "TCP transport trace requires at least one TCP traffic profile");

    if (!tcpTransportTraceFilePath.empty())
    {
        tcpTransportTraceStream.open(
            tcpTransportTraceFilePath,
            std::ios::out | std::ios::trunc);

        NS_ABORT_MSG_UNLESS(
            tcpTransportTraceStream.is_open(),
            "Could not open TCP transport trace file: "
                << tcpTransportTraceFilePath);

        tcpTransportTraceStream
            << "time_s,ue_index,direction,node_id,"
            << "event,old_value,new_value,unit\n";
    }

    RngSeedManager::SetSeed(scenarioConfig.rngSeed);
    RngSeedManager::SetRun(scenarioConfig.rngRun);

    std::cout << "RNG seed: " << RngSeedManager::GetSeed() << std::endl;
    std::cout << "RNG run: " << RngSeedManager::GetRun() << std::endl;

    std::cout << "UE mobility model: " << NestMobilityModelToString(scenarioConfig.mobility.model) << std::endl;

    if (scenarioConfig.mobility.model == NestMobilityModel::RANDOM_WAYPOINT)
    {
        std::cout << "UE mobility speed range: [" << scenarioConfig.mobility.minSpeed << ", " << scenarioConfig.mobility.maxSpeed << "] m/s" << std::endl;
        std::cout << "UE mobility pause: " << scenarioConfig.mobility.pause << " s" << std::endl;
    }

    std::cout << "Channel model: " << scenarioConfig.channel.model << std::endl;
    std::cout << "Channel scenario: " << scenarioConfig.channel.scenario << std::endl;
    std::cout << "Channel condition: " << scenarioConfig.channel.condition << std::endl;
    std::cout << "Channel shadowing: " << (scenarioConfig.channel.shadowingEnabled ? "enabled" : "disabled") << std::endl;
    std::cout << "Channel condition update period: " << scenarioConfig.channel.conditionUpdatePeriod << " s" << std::endl;
    std::cout << "Channel realization update period: " << scenarioConfig.channel.channelUpdatePeriod << " s" << std::endl;
    std::cout << "gNB antenna: " << scenarioConfig.antennas.gnb.rows << "x" << scenarioConfig.antennas.gnb.columns << " element=" << NestAntennaElementModelToString(scenarioConfig.antennas.gnb.elementModel) << std::endl;
    std::cout << "UE antenna: " << scenarioConfig.antennas.ue.rows << "x" << scenarioConfig.antennas.ue.columns << " element=" << NestAntennaElementModelToString(scenarioConfig.antennas.ue.elementModel) << std::endl;
    std::cout << "Beamforming mode: " << NestBeamformingModeToString(scenarioConfig.antennas.beamforming.mode) << std::endl;
    std::cout << "Beamforming update period: " << scenarioConfig.antennas.beamforming.updatePeriod << " s" << std::endl;
    std::cout << "gNB antenna ports: horizontal=" << scenarioConfig.antennas.gnb.horizontalPorts << " vertical=" << scenarioConfig.antennas.gnb.verticalPorts << " dualPolarized=" << (scenarioConfig.antennas.gnb.dualPolarized ? "true" : "false") << std::endl;
    std::cout << "UE antenna ports: horizontal=" << scenarioConfig.antennas.ue.horizontalPorts << " vertical=" << scenarioConfig.antennas.ue.verticalPorts << " dualPolarized=" << (scenarioConfig.antennas.ue.dualPolarized ? "true" : "false") << std::endl;
    std::cout << "MIMO feedback: " << (scenarioConfig.mimo.enabled ? "enabled" : "disabled") << std::endl;

    if (scenarioConfig.mimo.enabled)
    {
        std::cout << "MIMO CSI feedback flags: " << +scenarioConfig.mimo.csiFeedbackFlags << std::endl;
        std::cout << "MIMO PMI update intervals: wideband=" << scenarioConfig.mimo.widebandPmiUpdateInterval << " s subband=" << scenarioConfig.mimo.subbandPmiUpdateInterval << " s" << std::endl;
        std::cout << "MIMO PMI search: method=" << scenarioConfig.mimo.pmSearchMethod << " codebook=" << scenarioConfig.mimo.codebook << " rankLimit=" << +scenarioConfig.mimo.rankLimit << " subbandSize=" << +scenarioConfig.mimo.subbandSize << " downsampling=" << scenarioConfig.mimo.downsamplingTechnique << std::endl;
    }

    NestE2Config e2Config = scenarioConfig.e2;

    // Apply only E2 options explicitly provided on the command line.
    if (!enableE2Override.empty())
    {
        e2Config.enabled = ParseBooleanCommandLineOverride(enableE2Override, "--enableE2");
    }

    if (!ipE2TermRic.empty())
    {
        e2Config.termAddress = ipE2TermRic;
    }

    if (e2TermPortOverride != 0)
    {
        NS_ABORT_MSG_IF(
            e2TermPortOverride > std::numeric_limits<uint16_t>::max(),
            "--e2TermPort must be in the range 1..65535");

        e2Config.termPort =
            static_cast<uint16_t>(e2TermPortOverride);
    }

    if (e2LocalPortBaseOverride != 0)
    {
        NS_ABORT_MSG_IF(
            e2LocalPortBaseOverride >
                std::numeric_limits<uint16_t>::max(),
            "--e2LocalPortBase must be in the range 1..65535");

        e2Config.localPortBase =
            static_cast<uint16_t>(e2LocalPortBaseOverride);
    }

    if (!e2RealtimeOverride.empty())
    {
        e2Config.realtime =
            ParseBooleanCommandLineOverride(
                e2RealtimeOverride,
                "--e2Realtime");
    }

    ValidateNestE2Config(
        e2Config,
        scenarioConfig.gNbNum);

    NS_ABORT_MSG_UNLESS(
        scenarioConfig.controlMode != NestControlMode::E2 ||
            e2Config.enabled,
        "controlMode=e2 requires E2 to be enabled after command-line overrides");

    const std::string controlModeName =
        NestControlModeToString(scenarioConfig.controlMode);

    std::cout
        << "Selected control mode: "
        << controlModeName
        << std::endl;

    std::cout
        << "RIC Control RAN Function 300: "
        << (scenarioConfig.controlMode == NestControlMode::E2
                ? "enabled"
                : "disabled")
        << std::endl;

    // Create immutable local aliases for the validated scenario parameters.
    // Vector and map aliases use references to avoid unnecessary copies.
    const uint16_t gNbNum =
        scenarioConfig.gNbNum;

    const uint32_t ueNum =
        scenarioConfig.ueNum;

    const double simTime =
        scenarioConfig.simTime;

    const std::vector<NestPosition3d>& gNbPositions =
        scenarioConfig.gNbPositions;

    const NestUePositionAreaConfig& uePositionArea =
        scenarioConfig.uePositionArea;

    const double centralFrequency =
        scenarioConfig.centralFrequency;

    const double bandwidth =
        scenarioConfig.bandwidth;

    const uint16_t numerology =
        scenarioConfig.numerology;

    const double txPower =
        scenarioConfig.txPower;

    const double ueTxPower =
        scenarioConfig.ueTxPower;

    const std::vector<int>& uesPerSlice =
        scenarioConfig.uesPerSlice;

    const std::vector<uint8_t>& sstPerSlice =
        scenarioConfig.sstPerSlice;

    const std::vector<std::string>& trafficTypes =
        scenarioConfig.trafficTypes;

    const std::map<std::string, NestTrafficProfile>& trafficProfiles =
        scenarioConfig.trafficProfiles;

    double trafficStartTime = simTime;

    for (const auto& [trafficName, profile] : trafficProfiles)
    {
        trafficStartTime =
            std::min(
                trafficStartTime,
                profile.startTimeSeconds);

        std::cout
            << "Traffic profile " << trafficName
            << ": protocol="
            << NestTrafficProtocolToString(profile.protocol)
            << " direction="
            << NestTrafficDirectionToString(profile.direction)
            << " rate=" << profile.dataRateMbps << " Mbps"
            << " packetSize=" << profile.packetSize << " bytes"
            << " window=[" << profile.startTimeSeconds
            << ", " << profile.stopTimeSeconds << "] s"
            << " on="
            << NestTrafficDistributionToString(
                   profile.onTime.distribution)
            << "(" << profile.onTime.parameterSeconds << " s)"
            << " off="
            << NestTrafficDistributionToString(
                   profile.offTime.distribution)
            << "(" << profile.offTime.parameterSeconds << " s)"
            << std::endl;
    }

    const std::vector<LocalPrbQuotaAction>& localPrbQuotaActions = scenarioConfig.localPrbQuotaActions;

    // Validate event ordering and guarantee enough simulated time for metric
    // collection and, when enabled, at least one controller decision.
    NS_ABORT_MSG_UNLESS(
        trafficStartTime < simTime,
        "At least one traffic profile must start before the end of the simulation");

    NS_ABORT_MSG_UNLESS(sliceMetricsInterval > 0.0, "sliceMetricsInterval must be greater than zero");

    NS_ABORT_MSG_IF(!mobilityTraceFilePath.empty() && (!std::isfinite(mobilityTraceInterval) || mobilityTraceInterval <= 0.0), "mobilityTraceInterval must be finite and greater than zero");

    const bool sliceMetricsRequired = !sliceMetricsFilePath.empty() || scenarioConfig.localSliceController.has_value();

    // The current closed-loop controller observes downlink FlowMonitor
    // counters. Reject uplink-only slices instead of silently feeding the
    // controller zero-valued measurements for those slices.
    if (scenarioConfig.localSliceController.has_value())
    {
        for (const std::string& trafficType : trafficTypes)
        {
            const NestTrafficProfile& profile =
                trafficProfiles.at(trafficType);

            NS_ABORT_MSG_IF(
                profile.direction == NestTrafficDirection::UPLINK,
                "controlMode=local-controller requires every slice traffic "
                "profile to include downlink traffic; profile '"
                    << trafficType << "' is uplink-only");
        }
    }

    if (sliceMetricsRequired)
    {
        NS_ABORT_MSG_UNLESS(trafficStartTime + sliceMetricsInterval <= simTime, "The simulation must contain at least one complete slice metric observation window");
    }

    if (scenarioConfig.localSliceController.has_value())
    {
        const LocalSliceControllerConfig& controllerConfig = scenarioConfig.localSliceController.value();

        NS_ABORT_MSG_UNLESS(controllerConfig.initialApplyTime < trafficStartTime, "Controller initial quotas must be applied before traffic starts");

        NS_ABORT_MSG_UNLESS(sliceMetricsInterval <= controllerConfig.decisionInterval, "sliceMetricsInterval cannot exceed the controller decisionInterval");

        NS_ABORT_MSG_UNLESS(trafficStartTime + controllerConfig.decisionInterval <= simTime, "The simulation must contain at least one complete controller decision period");
    }

    // Map each UE to its slice and traffic type (for post-processing)
    std::vector<int> ueSliceId(ueNum, -1);
    std::vector<std::string> ueSliceTrafficType(ueNum, "");

    // KPM exposes a stable synthetic F1AP ID for every simulated UE while
    // retaining the actual ns-3 IMSI and configured slice SST internally.
    std::vector<KpmGnbDuUeContext> kpmGnbDuUeContexts;

    kpmGnbDuUeContexts.reserve(ueNum);

    if (e2Config.enabled && e2Config.realtime)
    {
        GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));

        NS_LOG_INFO("Realtime simulator enabled for E2 communication");
    }
    else
    {
        NS_LOG_INFO("Using the default discrete-event simulator");
    }

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    if (scenarioConfig.antennas.beamforming.mode == NestBeamformingMode::IDEAL_DIRECT_PATH)
    {
        Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
        beamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
        beamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(Seconds(scenarioConfig.antennas.beamforming.updatePeriod)));
        nrHelper->SetBeamformingHelper(beamformingHelper);
    }

    if (scenarioConfig.mimo.enabled)
    {
        NrHelper::MimoPmiParams mimoPmiParams;
        mimoPmiParams.pmSearchMethod = std::string("ns3::NrPmSearch") + scenarioConfig.mimo.pmSearchMethod;
        mimoPmiParams.fullSearchCb = std::string("ns3::NrCb") + scenarioConfig.mimo.codebook;
        mimoPmiParams.rankLimit = scenarioConfig.mimo.rankLimit;
        mimoPmiParams.subbandSize = scenarioConfig.mimo.subbandSize;
        mimoPmiParams.downsamplingTechnique = scenarioConfig.mimo.downsamplingTechnique;

        nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(scenarioConfig.mimo.csiFeedbackFlags));
        nrHelper->SetupMimoPmi(mimoPmiParams);
        nrHelper->SetUePhyAttribute("WbPmiUpdateInterval", TimeValue(Seconds(scenarioConfig.mimo.widebandPmiUpdateInterval)));
        nrHelper->SetUePhyAttribute("SbPmiUpdateInterval", TimeValue(Seconds(scenarioConfig.mimo.subbandPmiUpdateInterval)));
    }

    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetGnbPhyAttribute("TbDecodeLatency", TimeValue(MicroSeconds(1.0)));
    nrHelper->SetUePhyAttribute("TbDecodeLatency", TimeValue(MicroSeconds(1.0)));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower));
    //nrHelper->SetGnbPhyAttribute("DciProcessingDelay", TimeValue(MicroSeconds(1.0)));
    //nrHelper->SetUePhyAttribute("DciProcessingDelay", TimeValue(MicroSeconds(1.0)));

    // Configurar scheduler: RL com slicing ou RoundRobin padrão
    std::string schedulerType = enableRanSlicing ? "ns3::NrRLMacSchedulerOfdma" : "ns3::NrMacSchedulerOfdmaRR";
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(schedulerType));
    NS_LOG_INFO("Scheduler selecionado: " << schedulerType);

    // EPC helper
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    NodeContainer gNbNodes;
    gNbNodes.Create(gNbNum);

    NodeContainer ueNodes;
    ueNodes.Create(ueNum);

    // Remote host (server) connected to PGW via P2P link
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);

    // Mobility
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();

    for (const NestPosition3d& position : gNbPositions)
    {
        gnbPositionAlloc->Add(Vector(position.x, position.y, position.z));
    }

    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gNbNodes);

    // Log initial gNB positions
    NS_LOG_INFO("*** gNB initial positions ***");
    for (uint32_t i = 0; i < gNbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = gNbNodes.Get(i)->GetObject<MobilityModel>();
        if (mob)
        {
            Vector pos = mob->GetPosition();
            NS_LOG_INFO("gNB[" << i << "] pos = (" << pos.x << ", " << pos.y << ", " << pos.z << ")");
        }
    }

    MobilityHelper ueMobility;

    Ptr<UniformRandomVariable> xPosition = CreateObject<UniformRandomVariable>();
    xPosition->SetAttribute("Min", DoubleValue(uePositionArea.xMin));
    xPosition->SetAttribute("Max", DoubleValue(uePositionArea.xMax));

    Ptr<UniformRandomVariable> yPosition = CreateObject<UniformRandomVariable>();
    yPosition->SetAttribute("Min", DoubleValue(uePositionArea.yMin));
    yPosition->SetAttribute("Max", DoubleValue(uePositionArea.yMax));

    Ptr<ConstantRandomVariable> zPosition = CreateObject<ConstantRandomVariable>();
    zPosition->SetAttribute("Constant", DoubleValue(uePositionArea.height));

    Ptr<RandomBoxPositionAllocator> positionAlloc = CreateObject<RandomBoxPositionAllocator>();
    positionAlloc->SetAttribute("X", PointerValue(xPosition));
    positionAlloc->SetAttribute("Y", PointerValue(yPosition));
    positionAlloc->SetAttribute("Z", PointerValue(zPosition));

    ueMobility.SetPositionAllocator(positionAlloc);

    if (scenarioConfig.mobility.model == NestMobilityModel::STATIC)
    {
        ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
    else
    {
        Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable>();
        speed->SetAttribute("Min", DoubleValue(scenarioConfig.mobility.minSpeed));
        speed->SetAttribute("Max", DoubleValue(scenarioConfig.mobility.maxSpeed));

        Ptr<ConstantRandomVariable> pause = CreateObject<ConstantRandomVariable>();
        pause->SetAttribute("Constant", DoubleValue(scenarioConfig.mobility.pause));

        ueMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel", "Speed", PointerValue(speed), "Pause", PointerValue(pause), "PositionAllocator", PointerValue(positionAlloc));
    }

    ueMobility.Install(ueNodes);

    // Log initial UE positions
    NS_LOG_INFO("*** UE initial positions ***");
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
        if (mob)
        {
            Vector pos = mob->GetPosition();
            NS_LOG_INFO("UE[" << i << "] pos = (" << pos.x << ", " << pos.y << ", " << pos.z << ")");
        }
    }

    // Configure antenna arrays before installing the NR devices.
    const auto createAntennaElement = [](NestAntennaElementModel model) -> Ptr<AntennaModel>
    {
        if (model == NestAntennaElementModel::ISOTROPIC)
        {
            return CreateObject<IsotropicAntennaModel>();
        }

        if (model == NestAntennaElementModel::THREE_GPP)
        {
            return CreateObject<ThreeGppAntennaModel>();
        }

        NS_ABORT_MSG("Unsupported antenna-element model");
        return nullptr;
    };

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(scenarioConfig.antennas.gnb.rows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(scenarioConfig.antennas.gnb.columns));
    nrHelper->SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(scenarioConfig.antennas.gnb.horizontalPorts));
    nrHelper->SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(scenarioConfig.antennas.gnb.verticalPorts));
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(scenarioConfig.antennas.gnb.dualPolarized));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(createAntennaElement(scenarioConfig.antennas.gnb.elementModel)));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(scenarioConfig.antennas.ue.rows));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(scenarioConfig.antennas.ue.columns));
    nrHelper->SetUeAntennaAttribute("NumHorizontalPorts", UintegerValue(scenarioConfig.antennas.ue.horizontalPorts));
    nrHelper->SetUeAntennaAttribute("NumVerticalPorts", UintegerValue(scenarioConfig.antennas.ue.verticalPorts));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(scenarioConfig.antennas.ue.dualPolarized));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(createAntennaElement(scenarioConfig.antennas.ue.elementModel)));

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    OperationBandInfo band;
    const uint8_t numOfCcs = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numOfCcs);
    bandConf.m_numBwp = 1;
    band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenarioConfig.channel.scenario, scenarioConfig.channel.condition, scenarioConfig.channel.model);

    Ptr<ThreeGppChannelModel> channelModel = CreateObject<ThreeGppChannelModel>();
    channelModel->SetAttribute("UpdatePeriod", TimeValue(Seconds(scenarioConfig.channel.channelUpdatePeriod)));

    channelHelper->SetPhasedArraySpectrumPropagationLossModelAttribute("ChannelModel", PointerValue(channelModel));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(scenarioConfig.channel.shadowingEnabled));

    if (scenarioConfig.channel.condition == "Default")
    {
        channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(Seconds(scenarioConfig.channel.conditionUpdatePeriod)));
    }

    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Install IP stack on remoteHost and UEs
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(ueNodes);

    NetDeviceContainer gNbDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    if (scenarioConfig.mimo.enabled)
    {
        Ptr<NrGnbPhy> gnbPhy = NrHelper::GetGnbPhy(gNbDevs.Get(0), 0);
        NS_ABORT_MSG_UNLESS(gnbPhy, "Could not retrieve the first gNB PHY for MIMO validation");
        NS_ABORT_MSG_IF(allBwps.empty(), "MIMO validation requires at least one BWP");

        const uint16_t channelBandwidthUnits = static_cast<uint16_t>(allBwps.at(0).get()->m_channelBandwidth / 100000.0);
        const uint32_t channelBandwidthHz = channelBandwidthUnits * 100000U;
        const uint32_t subcarrierSpacing = 15000U * (1U << numerology);
        const double rbOverhead = gnbPhy->GetRbOverhead();
        const uint32_t prbCount = static_cast<uint32_t>(channelBandwidthHz * (1.0 - rbOverhead) / (subcarrierSpacing * NrSpectrumValueHelper::SUBCARRIERS_PER_RB));
        const uint8_t subbandSize = scenarioConfig.mimo.subbandSize;
        bool validSubbandSize = false;
        std::string allowedSubbandSizes;

        NS_ABORT_MSG_IF(prbCount == 0, "The configured MIMO BWP contains no PRBs");

        if (prbCount < 24)
        {
            validSubbandSize = subbandSize == 1;
            allowedSubbandSizes = "1";
        }
        else if (prbCount <= 72)
        {
            validSubbandSize = subbandSize == 4 || subbandSize == 8;
            allowedSubbandSizes = "4 or 8";
        }
        else if (prbCount <= 144)
        {
            validSubbandSize = subbandSize == 8 || subbandSize == 16;
            allowedSubbandSizes = "8 or 16";
        }
        else if (prbCount <= 275)
        {
            validSubbandSize = subbandSize == 16 || subbandSize == 32;
            allowedSubbandSizes = "16 or 32";
        }
        else
        {
            NS_ABORT_MSG("MIMO PMI search does not support a BWP with " << prbCount << " PRBs");
        }

        NS_ABORT_MSG_UNLESS(validSubbandSize, "mimo.subbandSize=" << static_cast<uint32_t>(subbandSize) << " is invalid for a BWP with " << prbCount << " PRBs; allowed values: " << allowedSubbandSizes);

        std::cout << "MIMO BWP PRBs: " << prbCount << std::endl;
    }

    if (!mimoTraceFilePath.empty())
    {
        mimoTraceStream.open(mimoTraceFilePath, std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_UNLESS(mimoTraceStream.is_open(), "Could not open MIMO feedback trace file: " << mimoTraceFilePath);

        mimoTraceStream << "time_s,ue_index,rnti,cqi,mcs,rank\n";

        for (uint32_t ueIndex = 0; ueIndex < ueDevs.GetN(); ++ueIndex)
        {
            auto callback = MakeBoundCallback(&WriteMimoFeedbackTrace, &mimoTraceStream, ueIndex);
            bool connected = nrHelper->GetUePhy(ueDevs.Get(ueIndex), 0)->TraceConnectWithoutContext("CqiFeedbackTrace", callback);
            NS_ABORT_MSG_UNLESS(connected, "Could not connect the MIMO feedback trace for UE " << ueIndex);
        }
    }

    if (!radioLinkTraceFilePath.empty())
    {
        radioLinkTraceStream.open(radioLinkTraceFilePath, std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_UNLESS(radioLinkTraceStream.is_open(), "Could not open radio-link trace file: " << radioLinkTraceFilePath);

        radioLinkTraceStream << "time_s,ue_index,node_id,rnti,cell_id,bwp_id,x,y,z,distance_m,pathloss_db,sinr_avg_db,sinr_min_db,cqi,mcs,rank,rb_count,tb_size,tbler,corrupt\n";
        radioLinkTraceState.Initialize(&radioLinkTraceStream, gNbDevs, ueDevs);

        for (uint32_t ueIndex = 0; ueIndex < ueDevs.GetN(); ++ueIndex)
        {
            Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueDevs.Get(ueIndex), 0);
            NS_ABORT_MSG_UNLESS(uePhy, "Could not retrieve UE PHY for radio-link trace");

            Ptr<NrSpectrumPhy> spectrumPhy = uePhy->GetSpectrumPhy();
            NS_ABORT_MSG_UNLESS(spectrumPhy, "Could not retrieve UE spectrum PHY for radio-link trace");

            spectrumPhy->EnableDlDataPathlossTrace();

            bool pathlossConnected = spectrumPhy->TraceConnectWithoutContext(
                "DlDataPathloss",
                MakeBoundCallback(&UpdateRadioLinkPathloss, &radioLinkTraceState, ueIndex));

            NS_ABORT_MSG_UNLESS(pathlossConnected, "Could not connect DlDataPathloss for UE " << ueIndex);

            bool receptionConnected = spectrumPhy->TraceConnectWithoutContext(
                "RxPacketTraceUe",
                MakeBoundCallback(&WriteRadioLinkReception, &radioLinkTraceState, ueIndex));

            NS_ABORT_MSG_UNLESS(receptionConnected, "Could not connect RxPacketTraceUe for UE " << ueIndex);
        }
    }

    if (!rbgTraceFilePath.empty())
    {
        NS_ABORT_MSG_UNLESS(enableRanSlicing, "RBG trace requires enableRanSlicing=true");

        rbgTraceStream.open(rbgTraceFilePath, std::ios::out | std::ios::trunc);

        NS_ABORT_MSG_UNLESS(rbgTraceStream.is_open(), "Could not open RBG trace file: " << rbgTraceFilePath);

        rbgTraceStream << "time_ns,gnb_index,bwp_id,slice_index,sst,allocated_rbg,available_rbg\n";

        for (uint32_t gNbIdx = 0; gNbIdx < gNbDevs.GetN(); ++gNbIdx)
        {
            auto gNbDevice = DynamicCast<NrGnbNetDevice>(gNbDevs.Get(gNbIdx));

            NS_ABORT_MSG_UNLESS(gNbDevice,
                                "Could not cast device to NrGnbNetDevice");

            constexpr uint8_t bwpId = 0;

            auto scheduler = DynamicCast<NrRLMacSchedulerOfdma>(gNbDevice->GetScheduler(bwpId));

            NS_ABORT_MSG_UNLESS(scheduler, "RBG trace requires NrRLMacSchedulerOfdma");

            bool connected = scheduler->TraceConnectWithoutContext("SliceRbgAllocation", MakeBoundCallback(&WriteSliceRbgAllocation, &rbgTraceStream, gNbIdx, bwpId));

            NS_ABORT_MSG_UNLESS(connected, "Could not connect SliceRbgAllocation trace");
        }
    }

    // Retain the helper until simulation teardown because it owns scheduled
    // setup callbacks used by the E2 interfaces.
    Ptr<E2TermHelper> e2TermHelper;

    if (e2Config.enabled)
    {
        e2TermHelper = CreateObject<E2TermHelper>();

        e2TermHelper->SetAttribute("E2TermIp", StringValue(e2Config.termAddress));

        e2TermHelper->SetAttribute("E2Port", UintegerValue(e2Config.termPort));

        e2TermHelper->SetAttribute("E2LocalPort", UintegerValue(e2Config.localPortBase));

        e2TermHelper->SetAttribute("Mcc", StringValue(e2Config.mcc));

        e2TermHelper->SetAttribute("Mnc", StringValue(e2Config.mnc));

        e2TermHelper->SetAttribute("EnableRicControl", BooleanValue(scenarioConfig.controlMode == NestControlMode::E2));

        e2TermHelper->InstallE2Term(gNbDevs);

        NS_LOG_INFO("E2 enabled: " << e2Config.termAddress << ":" << e2Config.termPort);
    }

    nrHelper->AttachToClosestGnb(ueDevs, gNbDevs);

    // Schedule slice mapping after RRC connection has been established
    NoriSlicingHelper::ScheduleSliceMapping(Seconds(1.0), enableRanSlicing, uesPerSlice, sstPerSlice, gNbDevs, ueDevs);

    std::shared_ptr<LocalPeriodicSliceController> localSliceController;

    // Create the optional closed-loop controller and schedule its initial
    // quota distribution after the UE-to-slice mapping has been installed.
    if (scenarioConfig.localSliceController.has_value())
    {
        const LocalSliceControllerConfig& controllerConfig = scenarioConfig.localSliceController.value();

        localSliceController = std::make_shared<LocalPeriodicSliceController>(controllerConfig, gNbDevs);

        Simulator::Schedule(
            Seconds(controllerConfig.initialApplyTime),
            [localSliceController]() {
                localSliceController->ApplyInitialQuotas();
            });
    }

    // Schedule every locally configured PRB quota action.
    for (const auto& action : localPrbQuotaActions)
    {
        Simulator::Schedule(
            Seconds(action.applyTime),
            [gNbDevs, action]() {
                ApplyLocalSliceQuotas(gNbDevs, action.quotas);
            });
    }

    // Connect remoteHost to PGW via P2P link
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer internetDevices = p2ph.Install(epcHelper->GetPgwNode(), remoteHostContainer.Get(0));

    // Endereçamento do link PGW <-> remoteHost
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.255.255.252");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1); // endereço do servidor
    Ipv4Address pgwAddr = internetIpIfaces.GetAddress(0);

    // epcHelper assigns IPv4 addresses to UEs
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    // Em muitas versões: Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);

    // Map each EPC-assigned UE address to its scenario index.
    std::map<Ipv4Address, uint32_t> ueIpToIndex;

    NS_LOG_INFO("*** EPC-assigned addresses ***");
    NS_LOG_INFO("remoteHost (server): " << remoteHostAddr);
    NS_LOG_INFO("PGW: " << pgwAddr);
    for (uint32_t i = 0; i < ueIpIfaces.GetN(); ++i)
    {
        Ipv4Address addr = ueIpIfaces.GetAddress(i);
        ueIpToIndex[addr] = i;
        NS_LOG_INFO("UE[" << i << "] IP (via EPC): " << addr);
    }

    // Static route on remoteHost towards UE network via PGW
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4> remoteIpv4 = remoteHostContainer.Get(0)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> remoteStatic = ipv4RoutingHelper.GetStaticRouting(remoteIpv4);
    remoteStatic->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), Ipv4Address("1.0.0.1"), 1);

    // Default route on UEs towards EPC gateway
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Ipv4> ueIpv4 = ueNodes.Get(i)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> ueStatic = ipv4RoutingHelper.GetStaticRouting(ueIpv4);
        ueStatic->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        NS_LOG_INFO("UE[" << i << "] default route: GW=" << epcHelper->GetUeDefaultGatewayAddress() << " via interface 1");
    }

    // Application data uses one deterministic port range per direction.
    const uint16_t downlinkPortBase = 8080;

    NS_ABORT_MSG_IF(
        ueNum > (65535U - downlinkPortBase) / 2U,
        "The UE count exceeds the available application port ranges");

    const uint16_t uplinkPortBase =
        static_cast<uint16_t>(downlinkPortBase + ueNum);

    // These maps identify configured application-data flows and exclude
    // reverse TCP acknowledgement flows from application statistics.
    std::map<uint16_t, uint32_t> downlinkPortToUe;
    std::map<uint16_t, uint32_t> uplinkPortToUe;

    const auto hasDownlink =
        [](NestTrafficDirection direction)
    {
        return direction == NestTrafficDirection::DOWNLINK ||
               direction == NestTrafficDirection::BIDIRECTIONAL;
    };

    const auto hasUplink =
        [](NestTrafficDirection direction)
    {
        return direction == NestTrafficDirection::UPLINK ||
               direction == NestTrafficDirection::BIDIRECTIONAL;
    };

    const auto buildDurationRandomVariable =
        [](const NestTrafficDurationConfig& duration)
    {
        if (duration.distribution ==
            NestTrafficDistribution::CONSTANT)
        {
            return std::string(
                       "ns3::ConstantRandomVariable[Constant=") +
                   std::to_string(duration.parameterSeconds) +
                   "]";
        }

        return std::string(
                   "ns3::ExponentialRandomVariable[Mean=") +
               std::to_string(duration.parameterSeconds) +
               "]";
    };

    const auto configureSource =
        [&buildDurationRandomVariable](
            OnOffHelper& source,
            const NestTrafficProfile& profile)
    {
        source.SetAttribute(
            "DataRate",
            DataRateValue(
                DataRate(
                    static_cast<uint64_t>(
                        profile.dataRateMbps * 1e6))));

        source.SetAttribute(
            "PacketSize",
            UintegerValue(profile.packetSize));

        source.SetAttribute(
            "OnTime",
            StringValue(
                buildDurationRandomVariable(profile.onTime)));

        source.SetAttribute(
            "OffTime",
            StringValue(
                buildDurationRandomVariable(profile.offTime)));
    };

    // Install applications and one direction-aware TFT per UE.
    uint32_t currentUeIndex = 0;

    for (size_t sliceId = 0;
         sliceId < uesPerSlice.size();
         ++sliceId)
    {
        const int countUes = uesPerSlice[sliceId];
        const std::string& trafficType =
            trafficTypes.at(sliceId);

        const NestTrafficProfile& profile =
            trafficProfiles.at(trafficType);

        const std::string socketFactory =
            profile.protocol == NestTrafficProtocol::UDP
                ? "ns3::UdpSocketFactory"
                : "ns3::TcpSocketFactory";

        NS_LOG_INFO(
            "Slice " << sliceId
            << " configured with traffic type: "
            << trafficType);

        for (int sliceUeIndex = 0;
             sliceUeIndex < countUes;
             ++sliceUeIndex)
        {
            NS_ABORT_MSG_IF(
                currentUeIndex >= ueNodes.GetN(),
                "Slice UE mapping exceeds the installed UE count");

            const uint32_t nodeIdx =
                currentUeIndex++;

            const Ipv4Address ueAddress =
                ueIpIfaces.GetAddress(nodeIdx);

            const uint16_t downlinkPort =
                static_cast<uint16_t>(
                    downlinkPortBase + nodeIdx);

            const uint16_t uplinkPort =
                static_cast<uint16_t>(
                    uplinkPortBase + nodeIdx);

            ueSliceId[nodeIdx] =
                static_cast<int>(sliceId);

            ueSliceTrafficType[nodeIdx] =
                trafficType;

            if (e2Config.enabled)
            {
                Ptr<NrUeNetDevice> ueNetDevice =
                    DynamicCast<NrUeNetDevice>(
                        ueDevs.Get(nodeIdx));

                NS_ABORT_MSG_UNLESS(
                    ueNetDevice,
                    "Could not cast UE device to NrUeNetDevice");

                KpmGnbDuUeContext kpmUeContext;

                kpmUeContext.gnbCuUeF1apId =
                    nodeIdx;

                kpmUeContext.imsi =
                    ueNetDevice->GetImsi();

                kpmUeContext.sst =
                    sstPerSlice.at(sliceId);

                kpmGnbDuUeContexts.push_back(
                    kpmUeContext);

                NS_LOG_INFO(
                    "[KPM V3] UE context: gNB-CU-UE-F1AP-ID="
                    << kpmUeContext.gnbCuUeF1apId
                    << ", IMSI=" << kpmUeContext.imsi
                    << ", SST="
                    << static_cast<uint32_t>(
                           kpmUeContext.sst));
            }

            Ptr<NrEpcTft> tft =
                Create<NrEpcTft>();

            if (hasDownlink(profile.direction))
            {
                NrEpcTft::PacketFilter downlinkFilter;

                downlinkFilter.direction =
                    NrEpcTft::DOWNLINK;

                downlinkFilter.localPortStart =
                    downlinkPort;

                downlinkFilter.localPortEnd =
                    downlinkPort;

                tft->Add(downlinkFilter);

                downlinkPortToUe.emplace(
                    downlinkPort,
                    nodeIdx);

                PacketSinkHelper downlinkSink(
                    socketFactory,
                    InetSocketAddress(
                        Ipv4Address::GetAny(),
                        downlinkPort));

                ApplicationContainer sinkApps =
                    downlinkSink.Install(
                        ueNodes.Get(nodeIdx));

                sinkApps.Start(Seconds(0.0));
                sinkApps.Stop(Seconds(simTime));

                OnOffHelper downlinkSource(
                    socketFactory,
                    InetSocketAddress(
                        ueAddress,
                        downlinkPort));

                configureSource(
                    downlinkSource,
                    profile);

                ApplicationContainer sourceApps =
                    downlinkSource.Install(
                        remoteHostContainer.Get(0));

                sourceApps.Start(
                    Seconds(profile.startTimeSeconds));

                sourceApps.Stop(
                    Seconds(profile.stopTimeSeconds));

                if (!tcpTransportTraceFilePath.empty() &&
                    profile.protocol == NestTrafficProtocol::TCP)
                {
                    Ptr<OnOffApplication> tcpApplication =
                        DynamicCast<OnOffApplication>(
                            sourceApps.Get(0));

                    NS_ABORT_MSG_UNLESS(
                        tcpApplication,
                        "Could not retrieve the downlink TCP source application");

                    auto tcpTrace =
                        std::make_unique<NestTcpTransportTrace>(
                            &tcpTransportTraceStream,
                            nodeIdx,
                            "downlink",
                            remoteHostContainer.Get(0)->GetId());

                    Simulator::Schedule(
                        Seconds(profile.startTimeSeconds) +
                            NanoSeconds(1),
                        &NestTcpTransportTrace::Connect,
                        tcpTrace.get(),
                        tcpApplication);

                    tcpTransportTraces.push_back(
                        std::move(tcpTrace));
                }

                NS_LOG_INFO(
                    "UE[" << nodeIdx
                    << "] downlink "
                    << NestTrafficProtocolToString(
                           profile.protocol)
                    << " source scheduled: port="
                    << downlinkPort
                    << " start="
                    << profile.startTimeSeconds
                    << "s stop="
                    << profile.stopTimeSeconds
                    << "s");
            }

            if (hasUplink(profile.direction))
            {
                NrEpcTft::PacketFilter uplinkFilter;

                uplinkFilter.direction =
                    NrEpcTft::UPLINK;

                uplinkFilter.remotePortStart =
                    uplinkPort;

                uplinkFilter.remotePortEnd =
                    uplinkPort;

                tft->Add(uplinkFilter);

                uplinkPortToUe.emplace(
                    uplinkPort,
                    nodeIdx);

                PacketSinkHelper uplinkSink(
                    socketFactory,
                    InetSocketAddress(
                        Ipv4Address::GetAny(),
                        uplinkPort));

                ApplicationContainer sinkApps =
                    uplinkSink.Install(
                        remoteHostContainer.Get(0));

                sinkApps.Start(Seconds(0.0));
                sinkApps.Stop(Seconds(simTime));

                OnOffHelper uplinkSource(
                    socketFactory,
                    InetSocketAddress(
                        remoteHostAddr,
                        uplinkPort));

                configureSource(
                    uplinkSource,
                    profile);

                ApplicationContainer sourceApps =
                    uplinkSource.Install(
                        ueNodes.Get(nodeIdx));

                sourceApps.Start(
                    Seconds(profile.startTimeSeconds));

                sourceApps.Stop(
                    Seconds(profile.stopTimeSeconds));

                if (!tcpTransportTraceFilePath.empty() &&
                    profile.protocol == NestTrafficProtocol::TCP)
                {
                    Ptr<OnOffApplication> tcpApplication =
                        DynamicCast<OnOffApplication>(
                            sourceApps.Get(0));

                    NS_ABORT_MSG_UNLESS(
                        tcpApplication,
                        "Could not retrieve the uplink TCP source application");

                    auto tcpTrace =
                        std::make_unique<NestTcpTransportTrace>(
                            &tcpTransportTraceStream,
                            nodeIdx,
                            "uplink",
                            ueNodes.Get(nodeIdx)->GetId());

                    Simulator::Schedule(
                        Seconds(profile.startTimeSeconds) +
                            NanoSeconds(1),
                        &NestTcpTransportTrace::Connect,
                        tcpTrace.get(),
                        tcpApplication);

                    tcpTransportTraces.push_back(
                        std::move(tcpTrace));
                }

                NS_LOG_INFO(
                    "UE[" << nodeIdx
                    << "] uplink "
                    << NestTrafficProtocolToString(
                           profile.protocol)
                    << " source scheduled: port="
                    << uplinkPort
                    << " start="
                    << profile.startTimeSeconds
                    << "s stop="
                    << profile.stopTimeSeconds
                    << "s");
            }

            // QoS selection remains intentionally unchanged until the next
            // checkpoint dedicated to per-slice bearers and 5QI/QCI values.
            NrEpsBearer bearer(
                NrEpsBearer::NGBR_LOW_LAT_EMBB);

            nrHelper->ActivateDedicatedEpsBearer(
                ueDevs.Get(nodeIdx),
                bearer,
                tft);

            NS_LOG_INFO(
                "Dedicated bearer activated for UE["
                << nodeIdx
                << "] protocol="
                << NestTrafficProtocolToString(
                       profile.protocol)
                << " direction="
                << NestTrafficDirectionToString(
                       profile.direction));
        }
    }

    NS_ABORT_MSG_IF(
        currentUeIndex != ueNum,
        "Slice UE mapping does not cover every installed UE");

    if (e2Config.enabled)
    {
        NS_ABORT_MSG_UNLESS(kpmGnbDuUeContexts.size() == ueNum, "KPM UE context count does not match the scenario UE count");

        for (uint32_t gNbIndex = 0; gNbIndex < gNbDevs.GetN(); ++gNbIndex)
        {
            Ptr<E2Interface> e2Interface = gNbDevs.Get(gNbIndex)->GetObject<E2Interface>();

            NS_ABORT_MSG_UNLESS(e2Interface, "Could not obtain the E2Interface from the gNB");

            e2Interface->SetKpmGnbDuUeContexts(kpmGnbDuUeContexts);
        }
    }

    // FlowMonitor statistics
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    SliceWindowMetricsCallback sliceMetricsCallback;

    // Forward every completed metrics window to the optional controller.
    if (localSliceController)
    {
        sliceMetricsCallback =
            [localSliceController](
                double windowStart,
                double windowEnd,
                const std::vector<SliceWindowMetrics>& metrics) {
                localSliceController->ObserveWindow(
                    windowStart,
                    windowEnd,
                    metrics);
            };
    }

    if (sliceMetricsRequired)
    {
        std::ofstream* sliceMetricsOutput = nullptr;

        // CSV output remains optional even when the controller needs metrics.
        if (!sliceMetricsFilePath.empty())
        {
            sliceMetricsStream.open(
                sliceMetricsFilePath,
                std::ios::out | std::ios::trunc);

            NS_ABORT_MSG_UNLESS(
                sliceMetricsStream.is_open(),
                "Could not open slice metric file: "
                    << sliceMetricsFilePath);

            sliceMetricsStream
                << "window_start_s,window_end_s,window_duration_s,"
                << "slice_index,sst,tx_packets,rx_packets,"
                << "tx_bytes,rx_bytes,offered_mbps,"
                << "throughput_mbps,mean_delay_ms\n";

            sliceMetricsOutput =
                &sliceMetricsStream;
        }

        Simulator::Schedule(
            Seconds(trafficStartTime),
            &SampleSliceWindowMetrics,
            monitor,
            &flowmonHelper,
            ueIpToIndex,
            downlinkPortToUe,
            ueSliceId,
            sstPerSlice,
            simTime,
            sliceMetricsInterval,
            &sliceMetricsState,
            sliceMetricsOutput,
            sliceMetricsCallback);
    }

    if (!mobilityTraceFilePath.empty())
    {
        mobilityTraceStream.open(mobilityTraceFilePath, std::ios::out | std::ios::trunc);

        NS_ABORT_MSG_UNLESS(mobilityTraceStream.is_open(), "Could not open mobility trace file: " << mobilityTraceFilePath);

        mobilityTraceStream << "time_s,node_type,node_index,slice_index,sst,x,y,z\n";

        Simulator::ScheduleNow(&SampleMobilityTrace, &gNbNodes, &ueNodes, &ueSliceId, &sstPerSlice, simTime, mobilityTraceInterval, &mobilityTraceStream);
    }

    // Run
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    if (tcpTransportTraceStream.is_open())
    {
        tcpTransportTraceStream.close();
    }

    if (mimoTraceStream.is_open())
    {
        mimoTraceStream.close();
    }
    if (radioLinkTraceStream.is_open())
    {
        radioLinkTraceStream.close();
    }
    if (mobilityTraceStream.is_open())
    {
        mobilityTraceStream.close();
    }
    if (rbgTraceStream.is_open())
    {
        rbgTraceStream.close();
    }

    if (sliceMetricsStream.is_open())
    {
        sliceMetricsStream.close();
    }

    // Post-simulation analysis
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(
            flowmonHelper.GetClassifier());

    NS_ABORT_MSG_UNLESS(
        classifier,
        "Could not obtain the IPv4 FlowMonitor classifier");

    struct ApplicationFlowAggregate
    {
        double totalThroughputMbps{0.0};
        double totalDelayMs{0.0};
        uint32_t flowCount{0};
    };

    std::map<std::string, ApplicationFlowAggregate>
        trafficAggregates;

    std::map<std::string, ApplicationFlowAggregate>
        downlinkAggregates;

    std::map<std::string, ApplicationFlowAggregate>
        uplinkAggregates;

    uint32_t ignoredNonApplicationFlows = 0;

    const std::map<FlowId, FlowMonitor::FlowStats> statsMap =
        monitor->GetFlowStats();

    std::cout
        << "\n=== DEBUG: Total number of captured flows ==="
        << std::endl;

    std::cout
        << "Total flows: "
        << statsMap.size()
        << std::endl;

    uint64_t totalTxPackets = 0;
    uint64_t totalRxPackets = 0;

    for (const auto& [flowId, stats] : statsMap)
    {
        const Ipv4FlowClassifier::FiveTuple tuple =
            classifier->FindFlow(flowId);

        std::cout
            << "  Flow " << flowId
            << ": " << tuple.sourceAddress
            << ":" << tuple.sourcePort
            << " -> " << tuple.destinationAddress
            << ":" << tuple.destinationPort
            << " (Tx: " << stats.txPackets
            << ", Rx: " << stats.rxPackets
            << ")"
            << std::endl;

        totalTxPackets += stats.txPackets;
        totalRxPackets += stats.rxPackets;
    }

    std::cout
        << "Total Tx: " << totalTxPackets
        << " | Total Rx: " << totalRxPackets
        << std::endl;

    std::cout
        << "\n=== FLOW DETAILS ==="
        << std::endl;

    for (const auto& [flowId, stats] : statsMap)
    {
        const Ipv4FlowClassifier::FiveTuple tuple =
            classifier->FindFlow(flowId);

        int ueIndex = -1;
        std::string direction;

        const auto downlinkPortIt =
            downlinkPortToUe.find(
                tuple.destinationPort);

        if (downlinkPortIt != downlinkPortToUe.end())
        {
            const auto ueAddressIt =
                ueIpToIndex.find(
                    tuple.destinationAddress);

            if (
                ueAddressIt != ueIpToIndex.end() &&
                ueAddressIt->second == downlinkPortIt->second
            )
            {
                ueIndex =
                    static_cast<int>(
                        ueAddressIt->second);

                direction = "downlink";
            }
        }

        if (ueIndex < 0)
        {
            const auto uplinkPortIt =
                uplinkPortToUe.find(
                    tuple.destinationPort);

            if (uplinkPortIt != uplinkPortToUe.end())
            {
                const auto ueAddressIt =
                    ueIpToIndex.find(
                        tuple.sourceAddress);

                if (
                    ueAddressIt != ueIpToIndex.end() &&
                    ueAddressIt->second == uplinkPortIt->second
                )
                {
                    ueIndex =
                        static_cast<int>(
                            ueAddressIt->second);

                    direction = "uplink";
                }
            }
        }

        // This excludes infrastructure traffic, auxiliary traffic and
        // reverse TCP acknowledgement flows from application statistics.
        if (ueIndex < 0)
        {
            ignoredNonApplicationFlows++;

            std::cout
                << "  (Ignored non-application flow: "
                << tuple.sourceAddress
                << ":" << tuple.sourcePort
                << " -> "
                << tuple.destinationAddress
                << ":" << tuple.destinationPort
                << ") Tx: " << stats.txPackets
                << " Rx: " << stats.rxPackets
                << std::endl;

            continue;
        }

        NS_ABORT_MSG_IF(
            static_cast<uint32_t>(ueIndex) >= ueSliceId.size() ||
                static_cast<uint32_t>(ueIndex) >=
                    ueSliceTrafficType.size(),
            "Application flow UE index is outside the slice mapping");

        const int sliceId =
            ueSliceId.at(ueIndex);

        const std::string& trafficType =
            ueSliceTrafficType.at(ueIndex);

        NS_ABORT_MSG_IF(
            trafficType.empty(),
            "Application flow UE has no configured traffic type");

        const NestTrafficProfile& profile =
            trafficProfiles.at(trafficType);

        double throughputMbps = 0.0;
        double delayMs = 0.0;
        double lossRatio = 100.0;

        if (stats.rxPackets > 0)
        {
            double receptionDuration =
                stats.timeLastRxPacket.GetSeconds() -
                stats.timeFirstTxPacket.GetSeconds();

            if (receptionDuration <= 0.0)
            {
                receptionDuration = 1e-9;
            }

            throughputMbps =
                stats.rxBytes * 8.0 /
                receptionDuration /
                1e6;

            delayMs =
                stats.delaySum.GetSeconds() /
                stats.rxPackets *
                1e3;
        }

        if (stats.txPackets > 0)
        {
            lossRatio =
                static_cast<double>(
                    stats.txPackets - stats.rxPackets) /
                stats.txPackets *
                100.0;
        }

        ApplicationFlowAggregate& trafficAggregate =
            trafficAggregates[trafficType];

        trafficAggregate.totalThroughputMbps +=
            throughputMbps;

        trafficAggregate.totalDelayMs +=
            delayMs;

        trafficAggregate.flowCount++;

        ApplicationFlowAggregate& directionAggregate =
            direction == "downlink"
                ? downlinkAggregates[trafficType]
                : uplinkAggregates[trafficType];

        directionAggregate.totalThroughputMbps +=
            throughputMbps;

        directionAggregate.totalDelayMs +=
            delayMs;

        directionAggregate.flowCount++;

        std::cout
            << "Flow " << flowId
            << " (" << trafficType
            << ", slice "
            << (
                sliceId >= 0
                    ? std::to_string(sliceId)
                    : std::string("N/A")
            )
            << ", " << direction
            << ", "
            << NestTrafficProtocolToString(
                   profile.protocol)
            << "): "
            << tuple.sourceAddress
            << ":" << tuple.sourcePort
            << " -> "
            << tuple.destinationAddress
            << ":" << tuple.destinationPort
            << " | T-put: "
            << std::fixed
            << std::setprecision(2)
            << throughputMbps
            << " Mbps"
            << " | Delay: "
            << delayMs
            << " ms"
            << " | Loss: "
            << lossRatio
            << " %"
            << std::endl;
    }

    std::cout
        << "\n=== SUMMARY ==="
        << std::endl;

    std::cout
        << "Ignored non-application flows: "
        << ignoredNonApplicationFlows
        << std::endl;

    const auto printAggregate =
        [](
            const std::string& label,
            const ApplicationFlowAggregate& aggregate)
    {
        std::cout
            << "Average " << label
            << " (" << aggregate.flowCount
            << " flows) - Throughput: "
            << aggregate.totalThroughputMbps /
                aggregate.flowCount
            << " Mbps; Delay: "
            << aggregate.totalDelayMs /
                aggregate.flowCount
            << " ms"
            << std::endl;
    };

    std::vector<std::string> summarizedTrafficTypes;

    for (const std::string& trafficType : trafficTypes)
    {
        if (
            std::find(
                summarizedTrafficTypes.begin(),
                summarizedTrafficTypes.end(),
                trafficType) !=
            summarizedTrafficTypes.end()
        )
        {
            continue;
        }

        summarizedTrafficTypes.push_back(
            trafficType);

        const auto aggregateIt =
            trafficAggregates.find(trafficType);

        if (
            aggregateIt == trafficAggregates.end() ||
            aggregateIt->second.flowCount == 0
        )
        {
            std::cout
                << "No " << trafficType
                << " application-data flow detected."
                << std::endl;

            continue;
        }

        printAggregate(
            trafficType,
            aggregateIt->second);

        const auto downlinkIt =
            downlinkAggregates.find(trafficType);

        if (
            downlinkIt != downlinkAggregates.end() &&
            downlinkIt->second.flowCount > 0
        )
        {
            printAggregate(
                trafficType + " downlink",
                downlinkIt->second);
        }

        const auto uplinkIt =
            uplinkAggregates.find(trafficType);

        if (
            uplinkIt != uplinkAggregates.end() &&
            uplinkIt->second.flowCount > 0
        )
        {
            printAggregate(
                trafficType + " uplink",
                uplinkIt->second);
        }
    }

    Simulator::Destroy();
    return 0;
}