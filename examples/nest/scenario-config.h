#ifndef NEST_SCENARIO_CONFIG_H
#define NEST_SCENARIO_CONFIG_H

#include "slice-controller.h"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ns3
{

/**
 * Selects the only source allowed to change slice PRB quotas.
 */
enum class NestControlMode
{
    NONE,
    LOCAL_ACTIONS,
    LOCAL_CONTROLLER,
    E2
};

/**
 * Return the JSON and log name of a control mode.
 */
std::string NestControlModeToString(NestControlMode mode);

/**
 * Selects the UE mobility model installed by the scenario.
 */
enum class NestMobilityModel
{
    STATIC,
    RANDOM_WAYPOINT
};

/**
 * Return the JSON and log name of a mobility model.
 */
std::string NestMobilityModelToString(NestMobilityModel model);

/**
 * UE mobility parameters configured through JSON.
 */
struct NestMobilityConfig
{
    NestMobilityModel model{NestMobilityModel::STATIC};
    double minSpeed{0.0};
    double maxSpeed{0.0};
    double pause{0.0};
};

/**
 * 3GPP radio channel configuration.
 *
 * Update periods use seconds. A zero period disables periodic regeneration.
 */
struct NestChannelConfig
{
    std::string scenario{"UMa"};
    std::string condition{"Default"};
    std::string model{"ThreeGpp"};
    bool shadowingEnabled{false};
    double conditionUpdatePeriod{0.0};
    double channelUpdatePeriod{0.0};
};

/**
 * Radiation pattern used by one antenna-array element.
 */
enum class NestAntennaElementModel
{
    ISOTROPIC,
    THREE_GPP
};

/**
 * Return the JSON and log name of an antenna-element model.
 */
std::string NestAntennaElementModelToString(NestAntennaElementModel model);

/**
 * One uniform planar antenna array.
 */
struct NestAntennaArrayConfig
{
    uint32_t rows{1};
    uint32_t columns{1};
    uint16_t horizontalPorts{1};
    uint16_t verticalPorts{1};
    bool dualPolarized{false};
    NestAntennaElementModel elementModel{NestAntennaElementModel::ISOTROPIC};
};

/**
 * Beamforming behavior shared by the gNB and UE arrays.
 */
enum class NestBeamformingMode
{
    QUASI_OMNI,
    IDEAL_DIRECT_PATH
};

/**
 * Return the JSON and log name of a beamforming mode.
 */
std::string NestBeamformingModeToString(NestBeamformingMode mode);

/**
 * Beamforming configuration.
 *
 * The update period uses seconds. Quasi-omni requires zero, while
 * ideal-direct-path requires a strictly positive update period.
 */
struct NestBeamformingConfig
{
    NestBeamformingMode mode{NestBeamformingMode::QUASI_OMNI};
    double updatePeriod{0.0};
};

/**
 * Complete antenna configuration for both radio endpoints.
 */
struct NestAntennasConfig
{
    NestAntennaArrayConfig gnb;
    NestAntennaArrayConfig ue;
    NestBeamformingConfig beamforming;
};

/**
 * Downlink MIMO feedback and RI/PMI search configuration.
 *
 * PMI update intervals use seconds. The selected rank cannot exceed the
 * number of antenna ports available at either endpoint.
 */
struct NestMimoConfig
{
    bool enabled{false};
    uint8_t csiFeedbackFlags{1};
    double widebandPmiUpdateInterval{0.01};
    double subbandPmiUpdateInterval{0.002};
    std::string pmSearchMethod{"Full"};
    std::string codebook{"TwoPort"};
    uint8_t rankLimit{1};
    uint8_t subbandSize{4};
    std::string downsamplingTechnique{"FirstPRB"};
};

/**
 * Validated QoS class identifiers for the NEST service profiles.
 *
 * The pinned NrHelper configures NrEpsBearer release 18. This initial
 * contract exposes the classes required by the eMBB and URLLC references.
 */
enum class NestQci
{
    NGBR_VIDEO_TCP_DEFAULT,
    NGBR_LOW_LAT_EMBB,
    DGBR_DISCRETE_AUT_SMALL
};

/**
 * Return the JSON and log name of a QoS class identifier.
 */
std::string NestQciToString(NestQci qci);

/**
 * Global policy used by the gNB RRC to select the RLC implementation.
 *
 * NS3_DEFAULT leaves the pinned helper behavior unchanged. With EPC enabled,
 * that behavior resolves to RLC UM. PACKET_ERROR_RATE_BASED selects UM when
 * the bearer's packet error loss rate is greater than 1e-5 and AM otherwise.
 */
enum class NestRlcMapping
{
    NS3_DEFAULT,
    UM_ALWAYS,
    AM_ALWAYS,
    PACKET_ERROR_RATE_BASED
};

/**
 * Return the JSON and log name of an RLC mapping policy.
 */
std::string NestRlcMappingToString(NestRlcMapping mapping);

/**
 * Global RLC selection and transmission-buffer configuration.
 *
 * Buffer sizes use bytes and configure the defaults used by every new
 * instance of the corresponding RLC type.
 */
struct NestRlcConfig
{
    NestRlcMapping mapping{NestRlcMapping::NS3_DEFAULT};
    uint32_t umMaxTxBufferSize{10 * 1024};
    uint32_t amMaxTxBufferSize{10 * 1024};
};

/**
 * Allocation and Retention Priority configuration.
 *
 * When disabled, the default NrEpsBearer ARP values are preserved. The pinned
 * NrEpsBearer copy constructor does not preserve ARP fields, so enabled ARP is
 * rejected until that upstream copy path is corrected.
 */
struct NestArpConfig
{
    bool enabled{false};
    uint8_t priorityLevel{0};
    bool preemptionCapability{false};
    bool preemptionVulnerability{false};
};

/**
 * QoS and bearer parameters associated with one traffic profile.
 *
 * GBR and MBR values use bit/s. Non-GBR bearers require all four values to
 * remain zero. GBR and delay-critical GBR profiles require direction-aware
 * positive values validated against the associated traffic profile.
 */
struct NestBearerQosConfig
{
    NestQci qci{NestQci::NGBR_LOW_LAT_EMBB};
    uint64_t gbrDl{0};
    uint64_t gbrUl{0};
    uint64_t mbrDl{0};
    uint64_t mbrUl{0};
    NestArpConfig arp;
};

/**
 * Complete QoS contract for all configured traffic profiles.
 *
 * The contract configures and audits the installed bearers, RLC selection and
 * RLC transmission buffers. The current NORI slicing scheduler does not
 * enforce 5QI priority, packet-delay budget or GBR requirements across UEs;
 * QoS-aware inter-UE scheduling remains outside this checkpoint.
 */
struct NestQosConfig
{
    uint8_t release{18};
    NestRlcConfig rlc;
    std::map<std::string, NestBearerQosConfig> bearers;
};

/**
 * Transport protocol used by one traffic profile.
 */
enum class NestTrafficProtocol
{
    UDP,
    TCP
};

/**
 * Return the JSON and log name of a traffic transport protocol.
 */
std::string NestTrafficProtocolToString(NestTrafficProtocol protocol);

/**
 * Direction of application data relative to the UE.
 */
enum class NestTrafficDirection
{
    DOWNLINK,
    UPLINK,
    BIDIRECTIONAL
};

/**
 * Return the JSON and log name of a traffic direction.
 */
std::string NestTrafficDirectionToString(NestTrafficDirection direction);

/**
 * Supported random-variable models for ON and OFF durations.
 */
enum class NestTrafficDistribution
{
    CONSTANT,
    EXPONENTIAL
};

/**
 * Return the JSON and log name of a traffic-duration distribution.
 */
std::string NestTrafficDistributionToString(NestTrafficDistribution distribution);

/**
 * One ON or OFF duration distribution.
 *
 * For a constant distribution, parameterSeconds is the fixed duration. For
 * an exponential distribution, it is the mean duration.
 */
struct NestTrafficDurationConfig
{
    NestTrafficDistribution distribution{NestTrafficDistribution::CONSTANT};
    double parameterSeconds{0.0};
};

/**
 * Traffic generation parameters associated with one service profile.
 *
 * For bidirectional traffic, dataRateMbps is offered independently in the
 * downlink and uplink directions.
 */
struct NestTrafficProfile
{
    NestTrafficProtocol protocol{NestTrafficProtocol::UDP};
    NestTrafficDirection direction{NestTrafficDirection::DOWNLINK};
    double dataRateMbps{0.0};
    uint16_t packetSize{0};
    double startTimeSeconds{0.0};
    double stopTimeSeconds{0.0};
    NestTrafficDurationConfig onTime{
        NestTrafficDistribution::CONSTANT,
        1.0};
    NestTrafficDurationConfig offTime{
        NestTrafficDistribution::CONSTANT,
        0.0};
};

/**
 * Simulator execution behavior independent of the modeled radio scenario.
 *
 * Command-line values may override these fields for one execution. Realtime
 * pacing is independent of E2 enablement.
 */
struct NestExecutionConfig
{
    bool enableRanSlicing{true};
    bool realtime{false};
};

/**
 * One optional CSV artifact without a periodic sampling interval.
 *
 * The file name is resolved relative to outputs.directory. A disabled output
 * does not create or truncate its configured file.
 */
struct NestOutputFileConfig
{
    bool enabled{false};
    std::string file;
};

/**
 * One optional periodically sampled CSV artifact.
 *
 * The interval uses seconds and must remain finite and greater than zero,
 * including when the output is disabled, so every configuration is complete.
 */
struct NestPeriodicOutputFileConfig
{
    bool enabled{false};
    std::string file;
    double interval{0.1};
};

/**
 * Files produced directly by the maintained slicing scenario.
 *
 * The directory may be replaced by the future experiment runner for each
 * policy and RNG run. File names remain stable inside every run directory.
 * Simulator stdout and stderr are captured by that runner and therefore are
 * not represented as an output opened by the simulator.
 */
struct NestOutputsConfig
{
    std::string directory{"."};

    NestOutputFileConfig rbgAllocation{
        false,
        "rbg-allocation.csv"};

    NestPeriodicOutputFileConfig mobility{
        false,
        "mobility-trace.csv",
        0.1};

    NestOutputFileConfig mimoFeedback{
        false,
        "mimo-feedback.csv"};

    NestOutputFileConfig radioLink{
        false,
        "radio-link.csv"};

    NestOutputFileConfig tcpTransport{
        false,
        "tcp-transport.csv"};

    NestPeriodicOutputFileConfig sliceMetrics{
        false,
        "slice-metrics.csv",
        0.1};
};

/**
 * E2 connection parameters used by the NEST scenario.
 */
struct NestE2Config
{
    bool enabled{false};
    std::string mcc{"001"};
    std::string mnc{"01"};
    std::string termAddress{"10.0.2.10"};
    uint16_t termPort{36421};
    uint16_t localPortBase{38470};
};

/**
 * One explicit three-dimensional position configured through JSON.
 */
struct NestPosition3d
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/**
 * Uniform rectangular area used for UE placement and waypoint destinations.
 */
struct NestUePositionAreaConfig
{
    double xMin{0.0};
    double xMax{0.0};
    double yMin{0.0};
    double yMax{0.0};
    double height{0.0};
};

/**
 * Complete JSON configuration used by the NEST slicing scenario.
 *
 * The contract separates the modeled network and traffic from simulator
 * execution behavior, produced artifacts and the optional E2 endpoint.
 */
struct NestScenarioConfig
{
    // Topology and placement parameters.
    uint16_t gNbNum{0};
    uint32_t ueNum{0};
    std::vector<NestPosition3d> gNbPositions;
    NestUePositionAreaConfig uePositionArea;

    // UE mobility model and its model-specific parameters.
    NestMobilityConfig mobility;

    // Radio propagation, channel condition and fast-fading configuration.
    NestChannelConfig channel;

    // Antenna arrays, element patterns and beamforming configuration.
    NestAntennasConfig antennas;

    // Optional downlink spatial multiplexing and RI/PMI feedback.
    NestMimoConfig mimo;

    // Simulation timing and reproducibility.
    double simTime{10.0};
    uint32_t rngSeed{1};
    uint64_t rngRun{1};

    // Simulator behavior that does not describe the modeled radio network.
    NestExecutionConfig execution;

    // Optional files produced directly by the scenario.
    NestOutputsConfig outputs;

    // NR radio parameters. Frequency and bandwidth are stored in hertz.
    double centralFrequency{3.6e9};
    double bandwidth{100e6};
    uint16_t numerology{0};
    double txPower{0.0};
    double ueTxPower{0.0};

    // Exclusive source allowed to change slice PRB quotas.
    NestControlMode controlMode{NestControlMode::NONE};

    // Explicit E2 connection. enabled=false remains fully offline.
    NestE2Config e2;

    // Per-slice vectors. The same index identifies one slice in all vectors.
    std::vector<uint32_t> uesPerSlice;
    std::vector<uint8_t> sstPerSlice;
    std::vector<std::string> trafficTypes;

    // Traffic profiles indexed by their JSON names, such as eMBB and URLLC.
    std::map<std::string, NestTrafficProfile>
        trafficProfiles;

    // QoS bearers keyed by the same profile names used by traffic and slices.
    NestQosConfig qos;

    // Deterministic quota actions scheduled at predefined simulation times.
    std::vector<LocalPrbQuotaAction>
        localPrbQuotaActions;

    // Optional closed-loop controller configuration.
    std::optional<LocalSliceControllerConfig>
        localSliceController;
};

/**
 * Load and validate the complete JSON configuration for the NEST scenario.
 *
 * The function validates topology, NR parameters, traffic profiles, slices,
 * deterministic quota actions and the optional periodic controller.
 */
NestScenarioConfig
LoadNestScenarioConfig(
    const std::string& configFilePath,
    const std::optional<bool>& enableRanSlicingOverride =
        std::nullopt);

/**
 * Validate an E2 configuration after JSON parsing or CLI overrides.
 */
void
ValidateNestE2Config(
    const NestE2Config& config,
    uint16_t gNbNum);

} // namespace ns3

#endif // NEST_SCENARIO_CONFIG_H
