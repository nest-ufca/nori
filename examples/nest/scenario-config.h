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
 * Complete JSON configuration used by the NEST eMBB/URLLC scenario.
 *
 * Command-line output paths and collection intervals remain outside this
 * structure because they describe execution artifacts rather than the
 * simulated network.
 */
struct NestScenarioConfig
{
    // Topology parameters.
    uint16_t gNbNum{1};
    uint32_t ueNum{0};

    // Simulation timing and topology spacing.
    double simTime{10.0};
    double interSiteDistance{20.0};

    // NR radio parameters. Frequency and bandwidth are stored in hertz.
    double centralFrequency{3.6e9};
    double bandwidth{100e6};
    uint16_t numerology{0};
    double txPower{0.0};
    double ueTxPower{0.0};

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

} // namespace ns3

#endif // NEST_SCENARIO_CONFIG_H
