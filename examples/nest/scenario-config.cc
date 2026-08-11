#include "scenario-config.h"

#include "ns3/core-module.h"
#include "ns3/ipv4-address.h"

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
 * Parse the mandatory exclusive slice-control source.
 */
NestControlMode ParseNestControlMode(const nlohmann::json& configJson)
{
    NS_ABORT_MSG_UNLESS(configJson.contains("controlMode") && configJson["controlMode"].is_string(), "controlMode must be a string");

    const std::string mode = configJson["controlMode"].get<std::string>();

    if (mode == "none")
    {
        return NestControlMode::NONE;
    }

    if (mode == "local-actions")
    {
        return NestControlMode::LOCAL_ACTIONS;
    }

    if (mode == "local-controller")
    {
        return NestControlMode::LOCAL_CONTROLLER;
    }

    if (mode == "e2")
    {
        return NestControlMode::E2;
    }

    NS_ABORT_MSG("controlMode must be one of: none, local-actions, local-controller, e2");
    return NestControlMode::NONE;
}

/**
 * Validate that exactly the source selected by controlMode is configured.
 */
void ValidateNestControlMode(const NestScenarioConfig& config, bool enableRanSlicing)
{
    const bool hasLocalActions = !config.localPrbQuotaActions.empty();
    const bool hasLocalController = config.localSliceController.has_value();

    switch (config.controlMode)
    {
    case NestControlMode::NONE:
        NS_ABORT_MSG_IF(hasLocalActions || hasLocalController, "controlMode=none does not allow localPrbQuotaActions or localSliceController");
        break;

    case NestControlMode::LOCAL_ACTIONS:
        NS_ABORT_MSG_UNLESS(enableRanSlicing, "controlMode=local-actions requires enableRanSlicing=true");
        NS_ABORT_MSG_UNLESS(hasLocalActions && !hasLocalController, "controlMode=local-actions requires localPrbQuotaActions and forbids localSliceController");
        break;

    case NestControlMode::LOCAL_CONTROLLER:
        NS_ABORT_MSG_UNLESS(enableRanSlicing, "controlMode=local-controller requires enableRanSlicing=true");
        NS_ABORT_MSG_UNLESS(hasLocalController && !hasLocalActions, "controlMode=local-controller requires localSliceController and forbids localPrbQuotaActions");
        break;

    case NestControlMode::E2:
        NS_ABORT_MSG_UNLESS(enableRanSlicing, "controlMode=e2 requires enableRanSlicing=true");
        NS_ABORT_MSG_IF(hasLocalActions || hasLocalController, "controlMode=e2 does not allow localPrbQuotaActions or localSliceController");
        break;

    default:
        NS_ABORT_MSG("Unsupported controlMode value");
    }
}

/**
 * Parse and validate gNB positions and the UE position area.
 */
void ParseTopologyConfiguration(const nlohmann::json& topologyJson, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Scenario configuration pointer is null");

    NS_ABORT_MSG_UNLESS(topologyJson.contains("numGNb") && topologyJson["numGNb"].is_number_integer(), "topology.numGNb must be an integer");

    const int64_t gNbNum = topologyJson["numGNb"].get<int64_t>();

    NS_ABORT_MSG_IF(gNbNum <= 0 || gNbNum > std::numeric_limits<uint16_t>::max(), "topology.numGNb must fit in a positive 16-bit value");

    config->gNbNum = static_cast<uint16_t>(gNbNum);

    NS_ABORT_MSG_UNLESS(topologyJson.contains("gnbPositions") && topologyJson["gnbPositions"].is_array(), "topology.gnbPositions must be a JSON array");

    const nlohmann::json& gnbPositionsJson = topologyJson["gnbPositions"];

    NS_ABORT_MSG_IF(gnbPositionsJson.size() != config->gNbNum, "topology.gnbPositions must contain exactly topology.numGNb entries");

    config->gNbPositions.clear();
    config->gNbPositions.reserve(config->gNbNum);

    for (const nlohmann::json& positionJson : gnbPositionsJson)
    {
        NS_ABORT_MSG_UNLESS(
            positionJson.is_object() &&
                positionJson.contains("x") &&
                positionJson["x"].is_number() &&
                positionJson.contains("y") &&
                positionJson["y"].is_number() &&
                positionJson.contains("z") &&
                positionJson["z"].is_number(),
            "Each topology.gnbPositions entry must contain "
            "numeric x, y and z fields");

        NestPosition3d position{
            positionJson["x"].get<double>(),
            positionJson["y"].get<double>(),
            positionJson["z"].get<double>()};

        NS_ABORT_MSG_IF(!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || position.z < 0.0, "gNB coordinates must be finite and z must be non-negative");

        config->gNbPositions.push_back(position);
    }

    NS_ABORT_MSG_UNLESS(topologyJson.contains("uePositionArea") && topologyJson["uePositionArea"].is_object(), "topology.uePositionArea must be a JSON object");

    const nlohmann::json& uePositionAreaJson = topologyJson["uePositionArea"];

    NS_ABORT_MSG_UNLESS(uePositionAreaJson.contains("mode") && uePositionAreaJson["mode"].is_string(), "topology.uePositionArea.mode must be a string");

    const std::string mode = uePositionAreaJson["mode"].get<std::string>();

    NS_ABORT_MSG_IF(mode != "uniform-rectangle", "topology.uePositionArea.mode must be uniform-rectangle");

    for (const char* field : {"xMin", "xMax", "yMin", "yMax", "height"})
    {
        NS_ABORT_MSG_UNLESS(uePositionAreaJson.contains(field) && uePositionAreaJson[field].is_number(), "topology.uePositionArea bounds and height must be numeric");
    }

    config->uePositionArea.xMin = uePositionAreaJson["xMin"].get<double>();
    config->uePositionArea.xMax = uePositionAreaJson["xMax"].get<double>();
    config->uePositionArea.yMin = uePositionAreaJson["yMin"].get<double>();
    config->uePositionArea.yMax = uePositionAreaJson["yMax"].get<double>();
    config->uePositionArea.height = uePositionAreaJson["height"].get<double>();

    NS_ABORT_MSG_IF(
        !std::isfinite(config->uePositionArea.xMin) ||
            !std::isfinite(config->uePositionArea.xMax) ||
            !std::isfinite(config->uePositionArea.yMin) ||
            !std::isfinite(config->uePositionArea.yMax) ||
            !std::isfinite(config->uePositionArea.height),
        "UE position-area bounds and height must be finite");

    NS_ABORT_MSG_IF(
        config->uePositionArea.xMin >= config->uePositionArea.xMax ||
            config->uePositionArea.yMin >= config->uePositionArea.yMax,
        "UE position-area minimum bounds must be smaller "
        "than maximum bounds");

    NS_ABORT_MSG_IF(config->uePositionArea.height < 0.0, "UE position-area height must be non-negative");
}

/**
 * Parse and validate the UE mobility model and its parameters.
 */
void ParseMobilityConfiguration(const nlohmann::json& mobilityJson, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Scenario configuration pointer is null");
    NS_ABORT_MSG_UNLESS(mobilityJson.contains("model") && mobilityJson["model"].is_string(), "mobility.model must be a string");

    const std::string model = mobilityJson["model"].get<std::string>();

    if (model == "static")
    {
        NS_ABORT_MSG_IF(mobilityJson.contains("speed") || mobilityJson.contains("pause"), "mobility.model=static does not allow speed or pause parameters");

        config->mobility = NestMobilityConfig{};
        config->mobility.model = NestMobilityModel::STATIC;
        return;
    }

    NS_ABORT_MSG_IF(model != "random-waypoint", "mobility.model must be one of: static, random-waypoint");
    NS_ABORT_MSG_UNLESS(mobilityJson.contains("speed") && mobilityJson["speed"].is_object(), "mobility.speed must be a JSON object for random-waypoint");
    NS_ABORT_MSG_UNLESS(mobilityJson.contains("pause") && mobilityJson["pause"].is_number(), "mobility.pause must be numeric for random-waypoint");

    const nlohmann::json& speedJson = mobilityJson["speed"];

    NS_ABORT_MSG_UNLESS(speedJson.contains("min") && speedJson["min"].is_number(), "mobility.speed.min must be numeric for random-waypoint");
    NS_ABORT_MSG_UNLESS(speedJson.contains("max") && speedJson["max"].is_number(), "mobility.speed.max must be numeric for random-waypoint");

    const double minSpeed = speedJson["min"].get<double>();
    const double maxSpeed = speedJson["max"].get<double>();
    const double pause = mobilityJson["pause"].get<double>();

    NS_ABORT_MSG_IF(!std::isfinite(minSpeed) || minSpeed <= 0.0, "mobility.speed.min must be finite and greater than zero");
    NS_ABORT_MSG_IF(!std::isfinite(maxSpeed) || maxSpeed < minSpeed, "mobility.speed.max must be finite and greater than or equal to mobility.speed.min");
    NS_ABORT_MSG_IF(!std::isfinite(pause) || pause < 0.0, "mobility.pause must be finite and non-negative");

    config->mobility.model = NestMobilityModel::RANDOM_WAYPOINT;
    config->mobility.minSpeed = minSpeed;
    config->mobility.maxSpeed = maxSpeed;
    config->mobility.pause = pause;
}

/**
 * Parse and validate the 3GPP radio channel configuration.
 */
void ParseChannelConfiguration(const nlohmann::json& channelJson, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Scenario configuration pointer is null");
    NS_ABORT_MSG_UNLESS(channelJson.contains("scenario") && channelJson["scenario"].is_string(), "channel.scenario must be a string");
    NS_ABORT_MSG_UNLESS(channelJson.contains("condition") && channelJson["condition"].is_string(), "channel.condition must be a string");
    NS_ABORT_MSG_UNLESS(channelJson.contains("model") && channelJson["model"].is_string(), "channel.model must be a string");
    NS_ABORT_MSG_UNLESS(channelJson.contains("shadowingEnabled") && channelJson["shadowingEnabled"].is_boolean(), "channel.shadowingEnabled must be a boolean");
    NS_ABORT_MSG_UNLESS(channelJson.contains("conditionUpdatePeriod") && channelJson["conditionUpdatePeriod"].is_number(), "channel.conditionUpdatePeriod must be numeric");
    NS_ABORT_MSG_UNLESS(channelJson.contains("channelUpdatePeriod") && channelJson["channelUpdatePeriod"].is_number(), "channel.channelUpdatePeriod must be numeric");

    const std::string scenario = channelJson["scenario"].get<std::string>();
    const std::string condition = channelJson["condition"].get<std::string>();
    const std::string model = channelJson["model"].get<std::string>();
    const double conditionUpdatePeriod = channelJson["conditionUpdatePeriod"].get<double>();
    const double channelUpdatePeriod = channelJson["channelUpdatePeriod"].get<double>();

    const std::set<std::string> supportedScenarios{"RMa", "UMa", "UMi", "InH-OfficeOpen", "InH-OfficeMixed", "V2V-Highway"};

    const std::set<std::string> supportedConditions{"Default", "LOS", "NLOS"};

    NS_ABORT_MSG_IF(supportedScenarios.find(scenario) == supportedScenarios.end(), "channel.scenario must be one of: RMa, UMa, UMi, InH-OfficeOpen, InH-OfficeMixed, V2V-Highway");
    NS_ABORT_MSG_IF(supportedConditions.find(condition) == supportedConditions.end(), "channel.condition must be one of: Default, LOS, NLOS");
    NS_ABORT_MSG_IF(model != "ThreeGpp", "channel.model must be ThreeGpp");
    if (scenario == "RMa")
    {
        const bool hasNonPositiveGnbHeight = std::any_of(config->gNbPositions.begin(), config->gNbPositions.end(), [](const NestPosition3d& position) { return position.z <= 0.0; });

        NS_ABORT_MSG_IF(hasNonPositiveGnbHeight, "channel.scenario=RMa requires positive gNB z coordinates");
        NS_ABORT_MSG_IF(config->uePositionArea.height <= 0.0, "channel.scenario=RMa requires a positive topology.uePositionArea.height");
    }
    NS_ABORT_MSG_IF(!std::isfinite(conditionUpdatePeriod) || conditionUpdatePeriod < 0.0, "channel.conditionUpdatePeriod must be finite and non-negative");
    NS_ABORT_MSG_IF(!std::isfinite(channelUpdatePeriod) || channelUpdatePeriod < 0.0, "channel.channelUpdatePeriod must be finite and non-negative");
    NS_ABORT_MSG_IF(condition != "Default" && conditionUpdatePeriod != 0.0, "channel.conditionUpdatePeriod must be zero for fixed LOS or NLOS conditions");

    config->channel.scenario = scenario;
    config->channel.condition = condition;
    config->channel.model = model;
    config->channel.shadowingEnabled = channelJson["shadowingEnabled"].get<bool>();
    config->channel.conditionUpdatePeriod = conditionUpdatePeriod;
    config->channel.channelUpdatePeriod = channelUpdatePeriod;
}

/**
 * Parse and validate one antenna array.
 */
void ParseAntennaArrayConfiguration(const nlohmann::json& arrayJson, const std::string& endpoint, NestAntennaArrayConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Antenna-array configuration pointer is null");

    const std::string prefix = "antennas." + endpoint;

    NS_ABORT_MSG_UNLESS(arrayJson.contains("rows") && arrayJson["rows"].is_number_unsigned(), prefix + ".rows must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(arrayJson.contains("columns") && arrayJson["columns"].is_number_unsigned(), prefix + ".columns must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(arrayJson.contains("horizontalPorts") && arrayJson["horizontalPorts"].is_number_unsigned(), prefix + ".horizontalPorts must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(arrayJson.contains("verticalPorts") && arrayJson["verticalPorts"].is_number_unsigned(), prefix + ".verticalPorts must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(arrayJson.contains("dualPolarized") && arrayJson["dualPolarized"].is_boolean(), prefix + ".dualPolarized must be a boolean");
    NS_ABORT_MSG_UNLESS(arrayJson.contains("elementModel") && arrayJson["elementModel"].is_string(), prefix + ".elementModel must be a string");

    const uint64_t rows = arrayJson["rows"].get<uint64_t>();
    const uint64_t columns = arrayJson["columns"].get<uint64_t>();
    const uint64_t horizontalPorts = arrayJson["horizontalPorts"].get<uint64_t>();
    const uint64_t verticalPorts = arrayJson["verticalPorts"].get<uint64_t>();
    const std::string elementModel = arrayJson["elementModel"].get<std::string>();

    NS_ABORT_MSG_IF(rows == 0 || rows > std::numeric_limits<uint32_t>::max(), prefix + ".rows must fit in a positive 32-bit value");
    NS_ABORT_MSG_IF(columns == 0 || columns > std::numeric_limits<uint32_t>::max(), prefix + ".columns must fit in a positive 32-bit value");
    NS_ABORT_MSG_IF(horizontalPorts == 0 || horizontalPorts > std::numeric_limits<uint16_t>::max(), prefix + ".horizontalPorts must fit in a positive 16-bit value");
    NS_ABORT_MSG_IF(verticalPorts == 0 || verticalPorts > std::numeric_limits<uint16_t>::max(), prefix + ".verticalPorts must fit in a positive 16-bit value");
    NS_ABORT_MSG_IF(columns % horizontalPorts != 0, prefix + ".horizontalPorts must divide antennas." + endpoint + ".columns");
    NS_ABORT_MSG_IF(rows % verticalPorts != 0, prefix + ".verticalPorts must divide antennas." + endpoint + ".rows");

    config->rows = static_cast<uint32_t>(rows);
    config->columns = static_cast<uint32_t>(columns);
    config->horizontalPorts = static_cast<uint16_t>(horizontalPorts);
    config->verticalPorts = static_cast<uint16_t>(verticalPorts);
    config->dualPolarized = arrayJson["dualPolarized"].get<bool>();

    if (elementModel == "Isotropic")
    {
        config->elementModel = NestAntennaElementModel::ISOTROPIC;
    }
    else if (elementModel == "ThreeGpp")
    {
        config->elementModel = NestAntennaElementModel::THREE_GPP;
    }
    else
    {
        NS_ABORT_MSG(prefix + ".elementModel must be one of: Isotropic, ThreeGpp");
    }
}

/**
 * Parse and validate antenna arrays and beamforming behavior.
 */
void ParseAntennasConfiguration(const nlohmann::json& antennasJson, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Scenario configuration pointer is null");
    NS_ABORT_MSG_UNLESS(antennasJson.contains("gnb") && antennasJson["gnb"].is_object(), "antennas.gnb must be a JSON object");
    NS_ABORT_MSG_UNLESS(antennasJson.contains("ue") && antennasJson["ue"].is_object(), "antennas.ue must be a JSON object");
    NS_ABORT_MSG_UNLESS(antennasJson.contains("beamforming") && antennasJson["beamforming"].is_object(), "antennas.beamforming must be a JSON object");

    NestAntennasConfig antennas;
    ParseAntennaArrayConfiguration(antennasJson["gnb"], "gnb", &antennas.gnb);
    ParseAntennaArrayConfiguration(antennasJson["ue"], "ue", &antennas.ue);

    const nlohmann::json& beamformingJson = antennasJson["beamforming"];

    NS_ABORT_MSG_UNLESS(beamformingJson.contains("mode") && beamformingJson["mode"].is_string(), "antennas.beamforming.mode must be a string");
    NS_ABORT_MSG_UNLESS(beamformingJson.contains("updatePeriod") && beamformingJson["updatePeriod"].is_number(), "antennas.beamforming.updatePeriod must be numeric");

    const std::string mode = beamformingJson["mode"].get<std::string>();
    const double updatePeriod = beamformingJson["updatePeriod"].get<double>();

    NS_ABORT_MSG_IF(!std::isfinite(updatePeriod) || updatePeriod < 0.0, "antennas.beamforming.updatePeriod must be finite and non-negative");

    if (mode == "quasi-omni")
    {
        NS_ABORT_MSG_IF(updatePeriod != 0.0, "antennas.beamforming.updatePeriod must be zero in quasi-omni mode");
        antennas.beamforming.mode = NestBeamformingMode::QUASI_OMNI;
    }
    else if (mode == "ideal-direct-path")
    {
        NS_ABORT_MSG_IF(updatePeriod <= 0.0, "antennas.beamforming.updatePeriod must be greater than zero in ideal-direct-path mode");
        antennas.beamforming.mode = NestBeamformingMode::IDEAL_DIRECT_PATH;
    }
    else
    {
        NS_ABORT_MSG("antennas.beamforming.mode must be one of: quasi-omni, ideal-direct-path");
    }

    antennas.beamforming.updatePeriod = updatePeriod;
    config->antennas = antennas;
}

/**
 * Parse and validate downlink MIMO feedback and RI/PMI selection.
 */
void ParseMimoConfiguration(const nlohmann::json& mimoJson, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Scenario configuration pointer is null");

    NS_ABORT_MSG_UNLESS(mimoJson.contains("enabled") && mimoJson["enabled"].is_boolean(), "mimo.enabled must be a boolean");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("csiFeedbackFlags") && mimoJson["csiFeedbackFlags"].is_number_unsigned(), "mimo.csiFeedbackFlags must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("widebandPmiUpdateInterval") && mimoJson["widebandPmiUpdateInterval"].is_number(), "mimo.widebandPmiUpdateInterval must be numeric");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("subbandPmiUpdateInterval") && mimoJson["subbandPmiUpdateInterval"].is_number(), "mimo.subbandPmiUpdateInterval must be numeric");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("pmSearchMethod") && mimoJson["pmSearchMethod"].is_string(), "mimo.pmSearchMethod must be a string");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("codebook") && mimoJson["codebook"].is_string(), "mimo.codebook must be a string");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("rankLimit") && mimoJson["rankLimit"].is_number_unsigned(), "mimo.rankLimit must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("subbandSize") && mimoJson["subbandSize"].is_number_unsigned(), "mimo.subbandSize must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(mimoJson.contains("downsamplingTechnique") && mimoJson["downsamplingTechnique"].is_string(), "mimo.downsamplingTechnique must be a string");

    const bool enabled = mimoJson["enabled"].get<bool>();
    const uint64_t csiFeedbackFlags = mimoJson["csiFeedbackFlags"].get<uint64_t>();
    const double widebandPmiUpdateInterval = mimoJson["widebandPmiUpdateInterval"].get<double>();
    const double subbandPmiUpdateInterval = mimoJson["subbandPmiUpdateInterval"].get<double>();
    const std::string pmSearchMethod = mimoJson["pmSearchMethod"].get<std::string>();
    const std::string codebook = mimoJson["codebook"].get<std::string>();
    const uint64_t rankLimit = mimoJson["rankLimit"].get<uint64_t>();
    const uint64_t subbandSize = mimoJson["subbandSize"].get<uint64_t>();
    const std::string downsamplingTechnique = mimoJson["downsamplingTechnique"].get<std::string>();

    const std::set<uint64_t> supportedCsiFeedbackFlags{1, 2, 3, 6, 7, 8};

    NS_ABORT_MSG_IF(supportedCsiFeedbackFlags.find(csiFeedbackFlags) == supportedCsiFeedbackFlags.end(), "mimo.csiFeedbackFlags must be one of: 1, 2, 3, 6, 7, 8");
    NS_ABORT_MSG_IF(enabled && csiFeedbackFlags == 8, "mimo.csiFeedbackFlags cannot select PDSCH_SISO when MIMO is enabled");
    NS_ABORT_MSG_IF(!std::isfinite(widebandPmiUpdateInterval) || widebandPmiUpdateInterval <= 0.0, "mimo.widebandPmiUpdateInterval must be finite and greater than zero");
    NS_ABORT_MSG_IF(!std::isfinite(subbandPmiUpdateInterval) || subbandPmiUpdateInterval <= 0.0, "mimo.subbandPmiUpdateInterval must be finite and greater than zero");
    NS_ABORT_MSG_IF(subbandPmiUpdateInterval > widebandPmiUpdateInterval, "mimo.subbandPmiUpdateInterval cannot exceed mimo.widebandPmiUpdateInterval");
    NS_ABORT_MSG_IF(pmSearchMethod != "Full", "mimo.pmSearchMethod must be Full");
    NS_ABORT_MSG_IF(codebook != "TwoPort", "mimo.codebook must be TwoPort");
    NS_ABORT_MSG_IF(rankLimit == 0 || rankLimit > 2, "mimo.rankLimit must be 1 or 2 for the TwoPort codebook");
    NS_ABORT_MSG_IF(subbandSize == 0 || subbandSize > std::numeric_limits<uint8_t>::max(), "mimo.subbandSize must fit in a positive 8-bit value");
    NS_ABORT_MSG_IF(downsamplingTechnique != "FirstPRB", "mimo.downsamplingTechnique must be FirstPRB");

    const uint64_t gnbPorts = (config->antennas.gnb.dualPolarized ? 2u : 1u) * config->antennas.gnb.horizontalPorts * config->antennas.gnb.verticalPorts;
    const uint64_t uePorts = (config->antennas.ue.dualPolarized ? 2u : 1u) * config->antennas.ue.horizontalPorts * config->antennas.ue.verticalPorts;
    const uint64_t maximumRank = std::min(gnbPorts, uePorts);

    NS_ABORT_MSG_IF(gnbPorts > 2, "mimo.codebook=TwoPort requires at most two configured gNB antenna ports");
    NS_ABORT_MSG_IF(rankLimit > maximumRank, "mimo.rankLimit cannot exceed the configured antenna-port count");
    NS_ABORT_MSG_IF(enabled && (gnbPorts < 2 || uePorts < 2), "mimo.enabled=true requires at least two antenna ports at both endpoints");
    NS_ABORT_MSG_IF(enabled && (config->antennas.gnb.elementModel != NestAntennaElementModel::THREE_GPP || config->antennas.ue.elementModel != NestAntennaElementModel::THREE_GPP), "mimo.enabled=true requires ThreeGpp antenna elements at both endpoints");

    config->mimo.enabled = enabled;
    config->mimo.csiFeedbackFlags = static_cast<uint8_t>(csiFeedbackFlags);
    config->mimo.widebandPmiUpdateInterval = widebandPmiUpdateInterval;
    config->mimo.subbandPmiUpdateInterval = subbandPmiUpdateInterval;
    config->mimo.pmSearchMethod = pmSearchMethod;
    config->mimo.codebook = codebook;
    config->mimo.rankLimit = static_cast<uint8_t>(rankLimit);
    config->mimo.subbandSize = static_cast<uint8_t>(subbandSize);
    config->mimo.downsamplingTechnique = downsamplingTechnique;
}

/**
 * Parse one ON/OFF duration distribution.
 */
NestTrafficDurationConfig
ParseTrafficDuration(const nlohmann::json& durationJson,
                     const std::string& fieldPath,
                     bool allowZero)
{
    NS_ABORT_MSG_UNLESS(
        durationJson.is_object(),
        fieldPath << " must be an object");

    NS_ABORT_MSG_UNLESS(
        durationJson.contains("distribution") &&
            durationJson["distribution"].is_string(),
        fieldPath << ".distribution must be a string");

    const std::string distribution =
        durationJson["distribution"].get<std::string>();

    NestTrafficDurationConfig duration;

    if (distribution == "constant")
    {
        NS_ABORT_MSG_UNLESS(
            durationJson.contains("value") &&
                durationJson["value"].is_number(),
            fieldPath
                << ".value must be numeric for the constant distribution");

        duration.distribution =
            NestTrafficDistribution::CONSTANT;

        duration.parameterSeconds =
            durationJson["value"].get<double>();
    }
    else if (distribution == "exponential")
    {
        NS_ABORT_MSG_UNLESS(
            durationJson.contains("mean") &&
                durationJson["mean"].is_number(),
            fieldPath
                << ".mean must be numeric for the exponential distribution");

        duration.distribution =
            NestTrafficDistribution::EXPONENTIAL;

        duration.parameterSeconds =
            durationJson["mean"].get<double>();
    }
    else
    {
        NS_ABORT_MSG(
            fieldPath
            << ".distribution must be one of: constant, exponential");
    }

    NS_ABORT_MSG_IF(
        !std::isfinite(duration.parameterSeconds),
        fieldPath << " duration must be finite");

    if (allowZero)
    {
        NS_ABORT_MSG_IF(
            duration.parameterSeconds < 0.0,
            fieldPath << " duration must be non-negative");
    }
    else
    {
        NS_ABORT_MSG_IF(
            duration.parameterSeconds <= 0.0,
            fieldPath << " duration must be greater than zero");
    }

    return duration;
}

/**
 * Parse and validate all traffic profiles declared in the JSON document.
 */
void
ParseTrafficProfiles(const nlohmann::json& configJson,
                     NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(
        config == nullptr,
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

        const std::string fieldPrefix =
            "traffic." + trafficName;

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("protocol") &&
                trafficJson["protocol"].is_string(),
            fieldPrefix << ".protocol must be a string");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("direction") &&
                trafficJson["direction"].is_string(),
            fieldPrefix << ".direction must be a string");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("bitrateMbps") &&
                trafficJson["bitrateMbps"].is_number(),
            fieldPrefix << ".bitrateMbps must be numeric");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("packetSize") &&
                trafficJson["packetSize"].is_number_integer(),
            fieldPrefix << ".packetSize must be an integer");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("startTime") &&
                trafficJson["startTime"].is_number(),
            fieldPrefix << ".startTime must be numeric");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("stopTime") &&
                trafficJson["stopTime"].is_number(),
            fieldPrefix << ".stopTime must be numeric");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("onTime"),
            fieldPrefix << ".onTime is required");

        NS_ABORT_MSG_UNLESS(
            trafficJson.contains("offTime"),
            fieldPrefix << ".offTime is required");

        const std::string protocol =
            trafficJson["protocol"].get<std::string>();

        const std::string direction =
            trafficJson["direction"].get<std::string>();

        if (protocol == "udp")
        {
            config->trafficProfiles[trafficName].protocol =
                NestTrafficProtocol::UDP;
        }
        else if (protocol == "tcp")
        {
            config->trafficProfiles[trafficName].protocol =
                NestTrafficProtocol::TCP;
        }
        else
        {
            NS_ABORT_MSG(
                fieldPrefix << ".protocol must be one of: udp, tcp");
        }

        if (direction == "downlink")
        {
            config->trafficProfiles[trafficName].direction =
                NestTrafficDirection::DOWNLINK;
        }
        else if (direction == "uplink")
        {
            config->trafficProfiles[trafficName].direction =
                NestTrafficDirection::UPLINK;
        }
        else if (direction == "bidirectional")
        {
            config->trafficProfiles[trafficName].direction =
                NestTrafficDirection::BIDIRECTIONAL;
        }
        else
        {
            NS_ABORT_MSG(
                fieldPrefix
                << ".direction must be one of: downlink, uplink, bidirectional");
        }

        NestTrafficProfile& profile =
            config->trafficProfiles[trafficName];

        profile.dataRateMbps =
            trafficJson["bitrateMbps"].get<double>();

        const int64_t packetSize =
            trafficJson["packetSize"].get<int64_t>();

        profile.startTimeSeconds =
            trafficJson["startTime"].get<double>();

        profile.stopTimeSeconds =
            trafficJson["stopTime"].get<double>();

        profile.onTime =
            ParseTrafficDuration(
                trafficJson["onTime"],
                fieldPrefix + ".onTime",
                false);

        profile.offTime =
            ParseTrafficDuration(
                trafficJson["offTime"],
                fieldPrefix + ".offTime",
                true);

        NS_ABORT_MSG_IF(
            !std::isfinite(profile.dataRateMbps) ||
                profile.dataRateMbps <= 0.0,
            fieldPrefix
                << ".bitrateMbps must be finite and greater than zero");

        NS_ABORT_MSG_IF(
            packetSize <= 0 ||
                packetSize >
                    std::numeric_limits<uint16_t>::max(),
            fieldPrefix
                << ".packetSize must fit in an unsigned 16-bit value");

        NS_ABORT_MSG_IF(
            !std::isfinite(profile.startTimeSeconds) ||
                profile.startTimeSeconds <= 1.0,
            fieldPrefix
                << ".startTime must be finite and after slice mapping at 1.0 s");

        NS_ABORT_MSG_IF(
            !std::isfinite(profile.stopTimeSeconds) ||
                profile.stopTimeSeconds <= profile.startTimeSeconds,
            fieldPrefix
                << ".stopTime must be finite and greater than startTime");

        NS_ABORT_MSG_IF(
            profile.stopTimeSeconds > config->simTime,
            fieldPrefix
                << ".stopTime cannot exceed simulation.duration");

        profile.packetSize =
            static_cast<uint16_t>(packetSize);
    }

    NS_ABORT_MSG_IF(
        config->trafficProfiles.empty(),
        "At least one traffic profile must be configured");
}

/**
 * Parse and validate the UE, SST and traffic-profile mapping of each slice.
 */
void ParseSliceConfiguration(const nlohmann::json& configJson, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr, "Scenario configuration pointer is null");

    NS_ABORT_MSG_UNLESS(configJson.contains("slices") && configJson["slices"].is_object(), "The scenario must contain a slices object");

    const nlohmann::json& slicesJson = configJson["slices"];

    NS_ABORT_MSG_UNLESS(slicesJson.contains("UesPerSlice") && slicesJson["UesPerSlice"].is_array(), "slices.UesPerSlice must be a JSON array");

    NS_ABORT_MSG_UNLESS(slicesJson.contains("trafficTypes") && slicesJson["trafficTypes"].is_array(), "slices.trafficTypes must be a JSON array");

    NS_ABORT_MSG_UNLESS(slicesJson.contains("SstPerSlice") && slicesJson["SstPerSlice"].is_array(), "slices.SstPerSlice must be a JSON array");

    // Read the parallel per-slice arrays from the JSON document.
    const std::vector<uint32_t> ueCounts = slicesJson["UesPerSlice"].get<std::vector<uint32_t>>();

    config->trafficTypes = slicesJson["trafficTypes"].get<std::vector<std::string>>();

    const std::vector<uint32_t> ssts = slicesJson["SstPerSlice"].get<std::vector<uint32_t>>();

    NS_ABORT_MSG_IF(ueCounts.empty(), "At least one slice must be configured");

    // Every array index must describe the same logical slice.
    NS_ABORT_MSG_IF(config->trafficTypes.size() != ueCounts.size() || ssts.size() != ueCounts.size(), "UesPerSlice, trafficTypes and SstPerSlice sizes must match");

    const uint32_t declaredSliceCount = slicesJson.value("numSlices", static_cast<uint32_t>(ueCounts.size()));

    NS_ABORT_MSG_IF(declaredSliceCount != ueCounts.size(), "slices.numSlices does not match the slice arrays");

    std::set<uint8_t> configuredSsts;
    uint64_t totalUes = 0;

    // Validate each slice and build the internal slice vectors.
    for (std::size_t sliceIndex = 0; sliceIndex < ueCounts.size(); ++sliceIndex)
    {
        NS_ABORT_MSG_IF(
            ueCounts[sliceIndex] == 0 ||
                ueCounts[sliceIndex] >
                    static_cast<uint32_t>(
                        std::numeric_limits<int>::max()),
            "Every slice must contain a valid positive UE count");

        NS_ABORT_MSG_IF(ssts[sliceIndex] > 255, "Slice SST must be in the range 0..255");

        const uint8_t sst = static_cast<uint8_t>(ssts[sliceIndex]);

        NS_ABORT_MSG_IF(!configuredSsts.insert(sst).second, "Duplicate SST in slice configuration");

        NS_ABORT_MSG_IF(
            config->trafficProfiles.find(
                config->trafficTypes[sliceIndex]) ==
                config->trafficProfiles.end(),
            "Slice refers to an unknown traffic profile");

        config->uesPerSlice.push_back(static_cast<int>(ueCounts[sliceIndex]));

        config->sstPerSlice.push_back(sst);

        totalUes += ueCounts[sliceIndex];
    }

    NS_ABORT_MSG_IF(totalUes > std::numeric_limits<uint32_t>::max(), "The total UE count exceeds the supported range");

    // Derive the total number of UEs from all configured slices.
    config->ueNum =
        static_cast<uint32_t>(totalUes);
}

/**
 * Parse and validate deterministic local PRB quota actions.
 */
void
ParseLocalPrbQuotaActions(const nlohmann::json& configJson, bool enableRanSlicing, NestScenarioConfig* config)
{
    NS_ABORT_MSG_IF(config == nullptr,
                    "Scenario configuration pointer is null");

    // Reject the obsolete quota format to keep the configuration schema
    // unambiguous.
    NS_ABORT_MSG_IF(configJson.contains("localPrbQuotas"), "localPrbQuotas is no longer supported; use localPrbQuotaActions instead");

    // Deterministic local actions are optional.
    if (!configJson.contains("localPrbQuotaActions"))
    {
        return;
    }

    const nlohmann::json& actionsJson = configJson["localPrbQuotaActions"];

    NS_ABORT_MSG_UNLESS(actionsJson.is_array() && !actionsJson.empty(), "localPrbQuotaActions must be a non-empty JSON array");

    NS_ABORT_MSG_UNLESS(enableRanSlicing, "Local PRB quota actions require enableRanSlicing=true");

    // Actions must occur after slice mapping at 1.0 s and in strict
    // chronological order.
    double previousApplyTime = 1.0;

    for (std::size_t actionIndex = 0; actionIndex < actionsJson.size(); ++actionIndex)
    {
        const std::string context = "localPrbQuotaActions[" + std::to_string(actionIndex) + "]";

        LocalPrbQuotaAction action = ParseLocalPrbQuotaAction(actionsJson[actionIndex], config->sstPerSlice, config->simTime, context);

        NS_ABORT_MSG_UNLESS(action.applyTime > previousApplyTime, "localPrbQuotaActions must be ordered by strictly increasing applyTime values");

        previousApplyTime = action.applyTime;

        config->localPrbQuotaActions.push_back(std::move(action));
    }
}

/**
 * Parse the optional closed-loop local slice controller configuration.
 */
std::optional<LocalSliceControllerConfig> ParseLocalSliceController(const nlohmann::json& configJson, bool enableRanSlicing, const NestScenarioConfig& scenarioConfig)
{
    // The local controller is configured by the presence of this object.
    if (!configJson.contains("localSliceController"))
    {
        return std::nullopt;
    }

    const nlohmann::json& controllerJson = configJson["localSliceController"];

    NS_ABORT_MSG_UNLESS(controllerJson.is_object(), "localSliceController must be a JSON object");
    NS_ABORT_MSG_IF(controllerJson.contains("enabled"), "localSliceController.enabled is no longer supported; use controlMode");

    // Quota control requires the slice-aware NORI scheduler.
    NS_ABORT_MSG_UNLESS(enableRanSlicing, "The local slice controller requires enableRanSlicing=true");

    // Read policy parameters, using defaults for optional JSON fields.
    LocalSliceControllerConfig controllerConfig;

    controllerConfig.initialApplyTime = controllerJson.value("initialApplyTime", 1.1);

    controllerConfig.decisionInterval = controllerJson.value("decisionInterval", 0.5);

    controllerConfig.quotaStep = controllerJson.value("quotaStep", 10u);

    controllerConfig.satisfactionHysteresis = controllerJson.value("satisfactionHysteresis", 0.1);

    controllerConfig.minimumQuota = controllerJson.value("minimumQuota", 10u);

    controllerConfig.maximumQuota = controllerJson.value("maximumQuota", 90u);

    // Initial quotas must be applied after slice mapping and before
    // simulation end.
    NS_ABORT_MSG_IF(
        !std::isfinite(controllerConfig.initialApplyTime) ||
            controllerConfig.initialApplyTime <= 1.0 ||
            controllerConfig.initialApplyTime >=
                scenarioConfig.simTime,
        "localSliceController.initialApplyTime must be after "
        "slice mapping and before the end of the simulation");

    NS_ABORT_MSG_UNLESS(controllerJson.contains("slices") && controllerJson["slices"].is_array() && !controllerJson["slices"].empty(), "localSliceController.slices must be a non-empty JSON array");

    // Index controller entries by SST to detect duplicates and restore
    // scenario order.
    std::map<uint8_t, LocalSliceControllerSliceConfig> slicesBySst;

    for (const nlohmann::json& sliceJson : controllerJson["slices"])
    {
        NS_ABORT_MSG_UNLESS(sliceJson.is_object(), "Each controller slice must be a JSON object");

        const uint32_t sliceId = sliceJson.value("sliceId", 0u);

        NS_ABORT_MSG_IF(sliceId == 0 || sliceId > 255, "Controller sliceId must be in the range 1..255");

        const uint8_t sst = static_cast<uint8_t>(sliceId);

        NS_ABORT_MSG_IF(std::find(scenarioConfig.sstPerSlice.begin(), scenarioConfig.sstPerSlice.end(), sst) == scenarioConfig.sstPerSlice.end(), "Controller refers to an SST not configured by the scenario");

        LocalSliceControllerSliceConfig sliceConfig;

        sliceConfig.sst = sst;

        sliceConfig.initialQuota = sliceJson.value("initialQuota", 101u);

        sliceConfig.throughputTargetMbps = sliceJson.value("throughputTargetMbps", -1.0);

        NS_ABORT_MSG_IF(!slicesBySst.emplace(sst, sliceConfig).second, "Controller contains a duplicate SST");
    }

    NS_ABORT_MSG_IF(slicesBySst.size() != scenarioConfig.sstPerSlice.size(), "The controller must configure every scenario SST");

    // Store controller slices in the scenario's internal slice order.
    for (uint8_t configuredSst : scenarioConfig.sstPerSlice)
    {
        controllerConfig.slices.push_back(slicesBySst.at(configuredSst));
    }

    return controllerConfig;
}

/**
 * Parse the optional E2 connection section.
 */
NestE2Config ParseE2Configuration(const nlohmann::json& configJson, uint16_t gNbNum)
{
    NestE2Config e2Config;

    if (!configJson.contains("e2"))
    {
        ValidateNestE2Config(e2Config, gNbNum);
        return e2Config;
    }

    const nlohmann::json& e2Json = configJson["e2"];

    NS_ABORT_MSG_UNLESS(e2Json.is_object(), "e2 must be a JSON object");

    e2Config.enabled = e2Json.value("enabled", e2Config.enabled);

    e2Config.mcc = e2Json.value("mcc", e2Config.mcc);

    e2Config.mnc = e2Json.value("mnc", e2Config.mnc);

    e2Config.termAddress = e2Json.value("termAddress", e2Config.termAddress);

    e2Config.realtime = e2Json.value("realtime", e2Config.realtime);

    const int64_t termPort = e2Json.value("termPort", static_cast<int64_t>(e2Config.termPort));

    const int64_t localPortBase = e2Json.value("localPortBase", static_cast<int64_t>(e2Config.localPortBase));

    NS_ABORT_MSG_IF(termPort <= 0 || termPort > std::numeric_limits<uint16_t>::max(), "e2.termPort must be in the range 1..65535");

    NS_ABORT_MSG_IF(localPortBase <= 0 || localPortBase > std::numeric_limits<uint16_t>::max(), "e2.localPortBase must be in the range 1..65535");

    e2Config.termPort = static_cast<uint16_t>(termPort);

    e2Config.localPortBase = static_cast<uint16_t>(localPortBase);

    ValidateNestE2Config(e2Config, gNbNum);
    return e2Config;
}

} // namespace

std::string NestControlModeToString(NestControlMode mode)
{
    switch (mode)
    {
    case NestControlMode::NONE:
        return "none";

    case NestControlMode::LOCAL_ACTIONS:
        return "local-actions";

    case NestControlMode::LOCAL_CONTROLLER:
        return "local-controller";

    case NestControlMode::E2:
        return "e2";

    default:
        NS_ABORT_MSG("Unsupported controlMode value");
        return "unknown";
    }
}

std::string
NestTrafficProtocolToString(NestTrafficProtocol protocol)
{
    switch (protocol)
    {
    case NestTrafficProtocol::UDP:
        return "udp";

    case NestTrafficProtocol::TCP:
        return "tcp";
    }

    return "unknown";
}

std::string
NestTrafficDirectionToString(NestTrafficDirection direction)
{
    switch (direction)
    {
    case NestTrafficDirection::DOWNLINK:
        return "downlink";

    case NestTrafficDirection::UPLINK:
        return "uplink";

    case NestTrafficDirection::BIDIRECTIONAL:
        return "bidirectional";
    }

    return "unknown";
}

std::string
NestTrafficDistributionToString(NestTrafficDistribution distribution)
{
    switch (distribution)
    {
    case NestTrafficDistribution::CONSTANT:
        return "constant";

    case NestTrafficDistribution::EXPONENTIAL:
        return "exponential";
    }

    return "unknown";
}

std::string NestMobilityModelToString(NestMobilityModel model)
{
    switch (model)
    {
    case NestMobilityModel::STATIC:
        return "static";

    case NestMobilityModel::RANDOM_WAYPOINT:
        return "random-waypoint";

    default:
        NS_ABORT_MSG("Unsupported mobility model");
        return "unknown";
    }
}

std::string NestAntennaElementModelToString(NestAntennaElementModel model)
{
    switch (model)
    {
    case NestAntennaElementModel::ISOTROPIC:
        return "Isotropic";

    case NestAntennaElementModel::THREE_GPP:
        return "ThreeGpp";

    default:
        NS_ABORT_MSG("Unsupported antenna-element model");
        return "unknown";
    }
}

std::string NestBeamformingModeToString(NestBeamformingMode mode)
{
    switch (mode)
    {
    case NestBeamformingMode::QUASI_OMNI:
        return "quasi-omni";

    case NestBeamformingMode::IDEAL_DIRECT_PATH:
        return "ideal-direct-path";

    default:
        NS_ABORT_MSG("Unsupported beamforming mode");
        return "unknown";
    }
}

/**
 * Validate E2 endpoint values and the per-gNB local port range.
 */
void ValidateNestE2Config(const NestE2Config& config, uint16_t gNbNum)
{
    const Ipv4Address termAddress(config.termAddress.c_str());

    const auto containsOnlyDigits =
        [](const std::string& value) {
            return std::all_of(
                value.begin(),
                value.end(),
                [](char character) {
                    return character >= '0' &&
                        character <= '9';
                });
        };

    NS_ABORT_MSG_UNLESS(config.mcc.size() == 3 && containsOnlyDigits(config.mcc), "e2.mcc must contain exactly three decimal digits");

    NS_ABORT_MSG_UNLESS((config.mnc.size() == 2 || config.mnc.size() == 3) && containsOnlyDigits(config.mnc), "e2.mnc must contain two or three decimal digits");

    NS_ABORT_MSG_UNLESS(termAddress.IsInitialized() && !termAddress.IsAny() && !termAddress.IsBroadcast() && !termAddress.IsMulticast(), "e2.termAddress must be a valid unicast IPv4 address");

    NS_ABORT_MSG_IF(config.termPort == 0, "e2.termPort must be in the range 1..65535");

    NS_ABORT_MSG_IF(config.localPortBase == 0, "e2.localPortBase must be in the range 1..65535");

    const uint32_t highestLocalPort = static_cast<uint32_t>(config.localPortBase) + gNbNum;

    NS_ABORT_MSG_IF(highestLocalPort > std::numeric_limits<uint16_t>::max(), "e2.localPortBase does not leave enough ports for all configured gNBs");
}

/**
 * Load and validate the complete JSON configuration for the NEST scenario.
 */
NestScenarioConfig LoadNestScenarioConfig(const std::string& configFilePath, bool enableRanSlicing)
{
    // Open the scenario configuration file.
    std::ifstream configFile(configFilePath);

    NS_ABORT_MSG_UNLESS(configFile.is_open(), "Could not open configuration file: " + configFilePath);

    // Parse the JSON document and report syntax errors with file context.
    nlohmann::json configJson;

    try
    {
        configFile >> configJson;
    }
    catch (const std::exception& error)
    {
        NS_FATAL_ERROR("Could not parse configuration file " << configFilePath << ": " << error.what());
    }

    NS_ABORT_MSG_UNLESS(configJson.is_object(), "The scenario configuration root must be a JSON object");

    NestScenarioConfig config;

    // Validate the required top-level JSON sections.
    NS_ABORT_MSG_UNLESS(configJson.contains("topology") && configJson["topology"].is_object(), "The scenario must contain a topology object");

    NS_ABORT_MSG_UNLESS(configJson.contains("mobility") && configJson["mobility"].is_object(), "The scenario must contain a mobility object");

    NS_ABORT_MSG_UNLESS(configJson.contains("channel") && configJson["channel"].is_object(), "The scenario must contain a channel object");

    NS_ABORT_MSG_UNLESS(configJson.contains("antennas") && configJson["antennas"].is_object(), "The scenario must contain an antennas object");

    NS_ABORT_MSG_UNLESS(configJson.contains("mimo") && configJson["mimo"].is_object(), "The scenario must contain a mimo object");

    NS_ABORT_MSG_UNLESS(configJson.contains("simulation") && configJson["simulation"].is_object(), "The scenario must contain a simulation object");

    NS_ABORT_MSG_UNLESS(configJson.contains("NR") && configJson["NR"].is_object(), "The scenario must contain an NR object");

    // Parse topology, mobility, channel, antennas, MIMO, simulation and NR parameters.
    ParseTopologyConfiguration(configJson["topology"], &config);
    ParseMobilityConfiguration(configJson["mobility"], &config);
    ParseChannelConfiguration(configJson["channel"], &config);
    ParseAntennasConfiguration(configJson["antennas"], &config);
    ParseMimoConfiguration(configJson["mimo"], &config);

    const nlohmann::json& simulationJson = configJson["simulation"];

    config.simTime = simulationJson.value("duration", config.simTime);

    NS_ABORT_MSG_UNLESS(simulationJson.contains("rngSeed") && simulationJson["rngSeed"].is_number_unsigned(), "simulation.rngSeed must be an unsigned integer");
    NS_ABORT_MSG_UNLESS(simulationJson.contains("rngRun") && simulationJson["rngRun"].is_number_unsigned(), "simulation.rngRun must be an unsigned integer");

    const uint64_t rngSeed = simulationJson["rngSeed"].get<uint64_t>();
    const uint64_t rngRun = simulationJson["rngRun"].get<uint64_t>();

    NS_ABORT_MSG_IF(rngSeed == 0 || rngSeed > std::numeric_limits<uint32_t>::max(), "simulation.rngSeed must fit in a positive 32-bit value");
    NS_ABORT_MSG_IF(rngRun == 0, "simulation.rngRun must be greater than zero");

    config.rngSeed = static_cast<uint32_t>(rngSeed);
    config.rngRun = rngRun;

    config.numerology = configJson["NR"].value("numerology", config.numerology);

    const double bandwidthMHz = configJson["NR"].value("bandwidthMHz", config.bandwidth / 1e6);

    // JSON exposes MHz for readability; ns-3 receives bandwidth in Hz.
    config.bandwidth = bandwidthMHz * 1e6;

    config.centralFrequency = configJson["NR"].value("centralFrequency", config.centralFrequency);

    config.txPower = configJson["NR"].value("txPower", config.txPower);

    config.ueTxPower = configJson["NR"].value("ueTxPower", config.ueTxPower);

    // Validate scalar values before constructing the scenario.
    NS_ABORT_MSG_IF(!std::isfinite(config.simTime) || config.simTime <= 1.0, "simulation.duration must be greater than one second");

    NS_ABORT_MSG_IF(config.numerology > 4, "NR.numerology must be in the range 0..4");

    NS_ABORT_MSG_IF(!std::isfinite(config.bandwidth) || config.bandwidth <= 0.0, "NR.bandwidthMHz must be finite and greater than zero");

    NS_ABORT_MSG_IF(!std::isfinite(config.centralFrequency) || config.centralFrequency <= 0.0, "NR.centralFrequency must be finite and greater than zero");

    NS_ABORT_MSG_IF(!std::isfinite(config.txPower) || !std::isfinite(config.ueTxPower), "NR transmit powers must be finite");

    config.controlMode = ParseNestControlMode(configJson);

    // Parse the optional E2 endpoint independently from slicing control.
    config.e2 = ParseE2Configuration(configJson, config.gNbNum);

    // Parse structured traffic, slice and control configurations.
    ParseTrafficProfiles(configJson, &config);

    ParseSliceConfiguration(configJson, &config);

    ParseLocalPrbQuotaActions(configJson, enableRanSlicing, &config);

    config.localSliceController = ParseLocalSliceController(configJson, enableRanSlicing, config);

    // Validate the exclusive source allowed to change scheduler quotas.
    ValidateNestControlMode(config, enableRanSlicing);

    return config;
}

} // namespace ns3
