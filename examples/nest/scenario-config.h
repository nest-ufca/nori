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
 * Traffic generation parameters associated with one service profile.
 */
struct NestTrafficProfile
{
    double dataRateMbps{0.0};
    uint16_t packetSize{0};
    double onTimeSeconds{1.0};
    double offTimeSeconds{0.01};
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
    bool realtime{true};
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
 * Complete JSON configuration used by the NEST eMBB/URLLC scenario.
 *
 * Command-line output paths and collection intervals remain outside this
 * structure because they describe execution artifacts rather than the
 * simulated network.
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

    // NR radio parameters. Frequency and bandwidth are stored in hertz.
    double centralFrequency{3.6e9};
    double bandwidth{100e6};
    uint16_t numerology{0};
    double txPower{0.0};
    double ueTxPower{0.0};

    // Exclusive source allowed to change slice PRB quotas.
    NestControlMode controlMode{NestControlMode::NONE};

    // Optional E2 connection. Disabled configurations remain fully offline.
    NestE2Config e2;

    // Per-slice vectors. The same index identifies one slice in all vectors.
    std::vector<int> uesPerSlice;
    std::vector<uint8_t> sstPerSlice;
    std::vector<std::string> trafficTypes;

    // Traffic profiles indexed by their JSON names, such as eMBB and URLLC.
    std::map<std::string, NestTrafficProfile>
        trafficProfiles;

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
    bool enableRanSlicing);

/**
 * Validate an E2 configuration after JSON parsing or CLI overrides.
 */
void
ValidateNestE2Config(
    const NestE2Config& config,
    uint16_t gNbNum);

} // namespace ns3

#endif // NEST_SCENARIO_CONFIG_H
