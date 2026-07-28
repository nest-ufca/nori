#include "scenario-config.h"

#include "ns3/core-module.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <set>

namespace ns3
{
namespace
{

/**
 * Parse and validate all traffic profiles declared in the JSON document.
 */
void
ParseTrafficProfiles(
    const nlohmann::json& configJson,
    NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr,
                    "Scenario configuration pointer is null");

    NS_ABORT_MSG_UNLESS(
        configJson.contains("traffic") &&
            configJson["traffic"].is_object(),
        "The scenario must contain a traffic object");

    for (const auto& item : configJson["traffic"].items())
    {
        const std::string trafficName = item.key();
        const nlohmann::json& trafficJson = item.value();

        // Ignore metadata entries such as the optional "_comment" field.
        if (!trafficJson.is_object())
        {
            continue;
        }

        NestTrafficProfile profile;

        profile.dataRateMbps =
            trafficJson.value("bitrateMbps", -1.0);

        const int64_t packetSize =
            trafficJson.value("packetSize", -1);

        profile.onTimeSeconds =
            trafficJson.value("onTimeMean", 1.0);

        profile.offTimeSeconds =
            trafficJson.value("offTimeMean", 0.01);

        NS_ABORT_MSG_IF(
            !std::isfinite(profile.dataRateMbps) ||
                profile.dataRateMbps <= 0.0,
            "Traffic bitrateMbps must be finite and greater than zero");

        NS_ABORT_MSG_IF(
            packetSize <= 0 ||
                packetSize >
                    std::numeric_limits<uint16_t>::max(),
            "Traffic packetSize must fit in an unsigned 16-bit value");

        NS_ABORT_MSG_IF(
            !std::isfinite(profile.onTimeSeconds) ||
                profile.onTimeSeconds <= 0.0,
            "Traffic onTimeMean must be finite and greater than zero");

        NS_ABORT_MSG_IF(
            !std::isfinite(profile.offTimeSeconds) ||
                profile.offTimeSeconds < 0.0,
            "Traffic offTimeMean must be finite and non-negative");

        profile.packetSize =
            static_cast<uint16_t>(packetSize);

        config->trafficProfiles.emplace(
            trafficName,
            profile);
    }

    NS_ABORT_MSG_IF(
        config->trafficProfiles.empty(),
        "At least one traffic profile must be configured");
}

/**
 * Parse and validate the UE, SST and traffic-profile mapping of each slice.
 */
void
ParseSliceConfiguration(
    const nlohmann::json& configJson,
    NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr,
                    "Scenario configuration pointer is null");

    NS_ABORT_MSG_UNLESS(
        configJson.contains("slices") &&
            configJson["slices"].is_object(),
        "The scenario must contain a slices object");

    const nlohmann::json& slicesJson =
        configJson["slices"];

    NS_ABORT_MSG_UNLESS(
        slicesJson.contains("UesPerSlice") &&
            slicesJson["UesPerSlice"].is_array(),
        "slices.UesPerSlice must be a JSON array");

    NS_ABORT_MSG_UNLESS(
        slicesJson.contains("trafficTypes") &&
            slicesJson["trafficTypes"].is_array(),
        "slices.trafficTypes must be a JSON array");

    NS_ABORT_MSG_UNLESS(
        slicesJson.contains("SstPerSlice") &&
            slicesJson["SstPerSlice"].is_array(),
        "slices.SstPerSlice must be a JSON array");

    // Read the parallel per-slice arrays from the JSON document.
    const std::vector<uint32_t> ueCounts =
        slicesJson["UesPerSlice"]
            .get<std::vector<uint32_t>>();

    config->trafficTypes =
        slicesJson["trafficTypes"]
            .get<std::vector<std::string>>();

    const std::vector<uint32_t> ssts =
        slicesJson["SstPerSlice"]
            .get<std::vector<uint32_t>>();

    NS_ABORT_MSG_IF(
        ueCounts.empty(),
        "At least one slice must be configured");

    // Every array index must describe the same logical slice.
    NS_ABORT_MSG_IF(
        config->trafficTypes.size() != ueCounts.size() ||
            ssts.size() != ueCounts.size(),
        "UesPerSlice, trafficTypes and SstPerSlice sizes must match");

    const uint32_t declaredSliceCount =
        slicesJson.value(
            "numSlices",
            static_cast<uint32_t>(ueCounts.size()));

    NS_ABORT_MSG_IF(
        declaredSliceCount != ueCounts.size(),
        "slices.numSlices does not match the slice arrays");

    std::set<uint8_t> configuredSsts;
    uint64_t totalUes = 0;

    // Validate each slice and build the internal slice vectors.
    for (std::size_t sliceIndex = 0;
         sliceIndex < ueCounts.size();
         ++sliceIndex)
    {
        NS_ABORT_MSG_IF(
            ueCounts[sliceIndex] == 0 ||
                ueCounts[sliceIndex] >
                    static_cast<uint32_t>(
                        std::numeric_limits<int>::max()),
            "Every slice must contain a valid positive UE count");

        NS_ABORT_MSG_IF(
            ssts[sliceIndex] > 255,
            "Slice SST must be in the range 0..255");

        const uint8_t sst =
            static_cast<uint8_t>(ssts[sliceIndex]);

        NS_ABORT_MSG_IF(
            !configuredSsts.insert(sst).second,
            "Duplicate SST in slice configuration");

        NS_ABORT_MSG_IF(
            config->trafficProfiles.find(
                config->trafficTypes[sliceIndex]) ==
                config->trafficProfiles.end(),
            "Slice refers to an unknown traffic profile");

        config->uesPerSlice.push_back(
            static_cast<int>(ueCounts[sliceIndex]));

        config->sstPerSlice.push_back(sst);

        totalUes += ueCounts[sliceIndex];
    }

    NS_ABORT_MSG_IF(
        totalUes >
            std::numeric_limits<uint32_t>::max(),
        "The total UE count exceeds the supported range");

    // Derive the total number of UEs from all configured slices.
    config->ueNum =
        static_cast<uint32_t>(totalUes);
}

/**
 * Parse and validate deterministic local PRB quota actions.
 */
void
ParseLocalPrbQuotaActions(
    const nlohmann::json& configJson,
    bool enableRanSlicing,
    NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr,
                    "Scenario configuration pointer is null");

    // Reject the obsolete quota format to keep the configuration schema
    // unambiguous.
    NS_ABORT_MSG_IF(
        configJson.contains("localPrbQuotas"),
        "localPrbQuotas is no longer supported; "
        "use localPrbQuotaActions instead");

    // Deterministic local actions are optional.
    if (!configJson.contains("localPrbQuotaActions"))
    {
        return;
    }

    const nlohmann::json& actionsJson =
        configJson["localPrbQuotaActions"];

    NS_ABORT_MSG_UNLESS(
        actionsJson.is_array() &&
            !actionsJson.empty(),
        "localPrbQuotaActions must be a non-empty JSON array");

    NS_ABORT_MSG_UNLESS(
        enableRanSlicing,
        "Local PRB quota actions require enableRanSlicing=true");

    // Actions must occur after slice mapping at 1.0 s and in strict
    // chronological order.
    double previousApplyTime = 1.0;

    for (std::size_t actionIndex = 0;
         actionIndex < actionsJson.size();
         ++actionIndex)
    {
        const std::string context =
            "localPrbQuotaActions[" +
            std::to_string(actionIndex) +
            "]";

        LocalPrbQuotaAction action =
            ParseLocalPrbQuotaAction(
                actionsJson[actionIndex],
                config->sstPerSlice,
                config->simTime,
                context);

        NS_ABORT_MSG_UNLESS(
            action.applyTime > previousApplyTime,
            "localPrbQuotaActions must be ordered by strictly "
            "increasing applyTime values");

        previousApplyTime = action.applyTime;

        config->localPrbQuotaActions.push_back(
            std::move(action));
    }
}

/**
 * Parse the optional closed-loop local slice controller configuration.
 */
std::optional<LocalSliceControllerConfig>
ParseLocalSliceController(
    const nlohmann::json& configJson,
    bool enableRanSlicing,
    const NestScenarioConfig& scenarioConfig)
{
    // The local controller is optional and may also be explicitly disabled.
    if (!configJson.contains("localSliceController"))
    {
        return std::nullopt;
    }

    const nlohmann::json& controllerJson =
        configJson["localSliceController"];

    NS_ABORT_MSG_UNLESS(
        controllerJson.is_object(),
        "localSliceController must be a JSON object");

    if (!controllerJson.value("enabled", false))
    {
        return std::nullopt;
    }

    // Quota control requires the slice-aware NORI scheduler.
    NS_ABORT_MSG_UNLESS(
        enableRanSlicing,
        "The local slice controller requires enableRanSlicing=true");

    // Read policy parameters, using defaults for optional JSON fields.
    LocalSliceControllerConfig controllerConfig;

    controllerConfig.initialApplyTime =
        controllerJson.value("initialApplyTime", 1.1);

    controllerConfig.decisionInterval =
        controllerJson.value("decisionInterval", 0.5);

    controllerConfig.quotaStep =
        controllerJson.value("quotaStep", 10u);

    controllerConfig.satisfactionHysteresis =
        controllerJson.value(
            "satisfactionHysteresis",
            0.1);

    controllerConfig.minimumQuota =
        controllerJson.value("minimumQuota", 10u);

    controllerConfig.maximumQuota =
        controllerJson.value("maximumQuota", 90u);

    // Initial quotas must be applied after slice mapping and before
    // simulation end.
    NS_ABORT_MSG_IF(
        !std::isfinite(controllerConfig.initialApplyTime) ||
            controllerConfig.initialApplyTime <= 1.0 ||
            controllerConfig.initialApplyTime >=
                scenarioConfig.simTime,
        "localSliceController.initialApplyTime must be after "
        "slice mapping and before the end of the simulation");

    NS_ABORT_MSG_UNLESS(
        controllerJson.contains("slices") &&
            controllerJson["slices"].is_array() &&
            !controllerJson["slices"].empty(),
        "localSliceController.slices must be a non-empty JSON array");

    // Index controller entries by SST to detect duplicates and restore
    // scenario order.
    std::map<uint8_t, LocalSliceControllerSliceConfig>
        slicesBySst;

    for (const nlohmann::json& sliceJson :
         controllerJson["slices"])
    {
        NS_ABORT_MSG_UNLESS(
            sliceJson.is_object(),
            "Each controller slice must be a JSON object");

        const uint32_t sliceId =
            sliceJson.value("sliceId", 0u);

        NS_ABORT_MSG_IF(
            sliceId == 0 || sliceId > 255,
            "Controller sliceId must be in the range 1..255");

        const uint8_t sst =
            static_cast<uint8_t>(sliceId);

        NS_ABORT_MSG_IF(
            std::find(
                scenarioConfig.sstPerSlice.begin(),
                scenarioConfig.sstPerSlice.end(),
                sst) ==
                scenarioConfig.sstPerSlice.end(),
            "Controller refers to an SST not configured by the scenario");

        LocalSliceControllerSliceConfig sliceConfig;

        sliceConfig.sst = sst;

        sliceConfig.initialQuota =
            sliceJson.value("initialQuota", 101u);

        sliceConfig.throughputTargetMbps =
            sliceJson.value(
                "throughputTargetMbps",
                -1.0);

        NS_ABORT_MSG_IF(
            !slicesBySst.emplace(
                sst,
                sliceConfig).second,
            "Controller contains a duplicate SST");
    }

    NS_ABORT_MSG_IF(
        slicesBySst.size() !=
            scenarioConfig.sstPerSlice.size(),
        "The controller must configure every scenario SST");

    // Store controller slices in the scenario's internal slice order.
    for (uint8_t configuredSst :
         scenarioConfig.sstPerSlice)
    {
        controllerConfig.slices.push_back(
            slicesBySst.at(configuredSst));
    }

    return controllerConfig;
}

} // namespace

/**
 * Load and validate the complete JSON configuration for the NEST scenario.
 */
NestScenarioConfig
LoadNestScenarioConfig(
    const std::string& configFilePath,
    bool enableRanSlicing)
{
    // Open the scenario configuration file.
    std::ifstream configFile(configFilePath);

    NS_ABORT_MSG_UNLESS(
        configFile.is_open(),
        "Could not open configuration file: " +
            configFilePath);

    // Parse the JSON document and report syntax errors with file context.
    nlohmann::json configJson;

    try
    {
        configFile >> configJson;
    }
    catch (const std::exception& error)
    {
        NS_FATAL_ERROR(
            "Could not parse configuration file "
            << configFilePath
            << ": "
            << error.what());
    }

    NS_ABORT_MSG_UNLESS(
        configJson.is_object(),
        "The scenario configuration root must be a JSON object");

    NestScenarioConfig config;

    // Validate the required top-level JSON sections.
    NS_ABORT_MSG_UNLESS(
        configJson.contains("topology") &&
            configJson["topology"].is_object(),
        "The scenario must contain a topology object");

    NS_ABORT_MSG_UNLESS(
        configJson.contains("simulation") &&
            configJson["simulation"].is_object(),
        "The scenario must contain a simulation object");

    NS_ABORT_MSG_UNLESS(
        configJson.contains("NR") &&
            configJson["NR"].is_object(),
        "The scenario must contain an NR object");

    // Parse scalar topology, simulation and NR parameters.
    const uint32_t gNbNum =
        configJson["topology"].value(
            "numGNb",
            static_cast<uint32_t>(config.gNbNum));

    NS_ABORT_MSG_IF(
        gNbNum == 0 ||
            gNbNum >
                std::numeric_limits<uint16_t>::max(),
        "topology.numGNb must fit in a positive 16-bit value");

    config.gNbNum =
        static_cast<uint16_t>(gNbNum);

    config.interSiteDistance =
        configJson["topology"].value(
            "distance",
            config.interSiteDistance);

    config.simTime =
        configJson["simulation"].value(
            "duration",
            config.simTime);

    config.numerology =
        configJson["NR"].value(
            "numerology",
            config.numerology);

    const double bandwidthMHz =
        configJson["NR"].value(
            "bandwidthMHz",
            config.bandwidth / 1e6);

    // JSON exposes MHz for readability; ns-3 receives bandwidth in Hz.
    config.bandwidth =
        bandwidthMHz * 1e6;

    config.centralFrequency =
        configJson["NR"].value(
            "centralFrequency",
            config.centralFrequency);

    config.txPower =
        configJson["NR"].value(
            "txPower",
            config.txPower);

    config.ueTxPower =
        configJson["NR"].value(
            "ueTxPower",
            config.ueTxPower);

    // Validate scalar values before constructing the scenario.
    NS_ABORT_MSG_IF(
        !std::isfinite(config.interSiteDistance) ||
            config.interSiteDistance <= 0.0,
        "topology.distance must be finite and greater than zero");

    NS_ABORT_MSG_IF(
        !std::isfinite(config.simTime) ||
            config.simTime <= 1.0,
        "simulation.duration must be greater than one second");

    NS_ABORT_MSG_IF(
        config.numerology > 4,
        "NR.numerology must be in the range 0..4");

    NS_ABORT_MSG_IF(
        !std::isfinite(config.bandwidth) ||
            config.bandwidth <= 0.0,
        "NR.bandwidthMHz must be finite and greater than zero");

    NS_ABORT_MSG_IF(
        !std::isfinite(config.centralFrequency) ||
            config.centralFrequency <= 0.0,
        "NR.centralFrequency must be finite and greater than zero");

    NS_ABORT_MSG_IF(
        !std::isfinite(config.txPower) ||
            !std::isfinite(config.ueTxPower),
        "NR transmit powers must be finite");

    // Parse structured traffic, slice and control configurations.
    ParseTrafficProfiles(configJson, &config);

    ParseSliceConfiguration(configJson, &config);

    ParseLocalPrbQuotaActions(
        configJson,
        enableRanSlicing,
        &config);

    config.localSliceController =
        ParseLocalSliceController(
            configJson,
            enableRanSlicing,
            config);

    // Prevent two local quota sources from controlling the scheduler
    // simultaneously.
    NS_ABORT_MSG_IF(
        !config.localPrbQuotaActions.empty() &&
            config.localSliceController.has_value(),
        "Use either localPrbQuotaActions or localSliceController, "
        "not both");

    return config;
}

} // namespace ns3
