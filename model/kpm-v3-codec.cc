#include "kpm-v3-codec.h"

#include "E2SM-KPM-ActionDefinition-Format1.h"
#include "E2SM-KPM-ActionDefinition.h"
#include "LabelInfoItem.h"
#include "MeasurementInfoItem.h"

#include "E2SM-KPM-EventTriggerDefinition-Format1.h"
#include "E2SM-KPM-EventTriggerDefinition.h"
#include "aper_decoder.h"

#include <limits>
#include <memory>
#include <utility>

namespace ns3
{
namespace
{

/**
 * Build a failed decoding result without terminating the simulator.
 *
 * The public decoder uses this helper for malformed or unsupported
 * subscription payloads.
 */
KpmV3DecodeResult
MakeDecodeError(const std::string& message)
{
    KpmV3DecodeResult result;
    result.errorMessage = message;
    return result;
}

/**
 * Release an Event Trigger Definition allocated by the ASN.1 decoder.
 *
 * This function is used as a std::unique_ptr deleter so partially decoded
 * structures are also released when an error path returns early.
 */
void
FreeEventTriggerDefinition(
    E2SM_KPM_EventTriggerDefinition_t* definition)
{
    if (definition != nullptr)
    {
        ASN_STRUCT_FREE(
            asn_DEF_E2SM_KPM_EventTriggerDefinition,
            definition);
    }
}

using EventTriggerDefinitionPtr =
    std::unique_ptr<
        E2SM_KPM_EventTriggerDefinition_t,
        decltype(&FreeEventTriggerDefinition)>;

/**
 * Release an Action Definition allocated by the ASN.1 decoder.
 */
void
FreeActionDefinition(
    E2SM_KPM_ActionDefinition_t* definition)
{
    if (definition != nullptr)
    {
        ASN_STRUCT_FREE(
            asn_DEF_E2SM_KPM_ActionDefinition,
            definition);
    }
}

using ActionDefinitionPtr =
    std::unique_ptr<
        E2SM_KPM_ActionDefinition_t,
        decltype(&FreeActionDefinition)>;

/**
 * Decode a KPM Event Trigger Definition Format 1.
 *
 * The currently supported format carries the reporting period requested by
 * the xApp. The value is expressed in milliseconds.
 */
bool
DecodeEventTrigger(
    const uint8_t* data,
    std::size_t size,
    uint32_t& reportingPeriodMs,
    std::string& errorMessage)
{
    void* decodedStructure = nullptr;

    const asn_dec_rval_t decodeResult =
        aper_decode_complete(
            nullptr,
            &asn_DEF_E2SM_KPM_EventTriggerDefinition,
            &decodedStructure,
            data,
            size);

    EventTriggerDefinitionPtr definition(
        static_cast<E2SM_KPM_EventTriggerDefinition_t*>(
            decodedStructure),
        &FreeEventTriggerDefinition);

    if (decodeResult.code != RC_OK ||
        definition == nullptr)
    {
        errorMessage =
            "Could not decode the KPM Event Trigger Definition";
        return false;
    }

    if (definition->eventDefinition_formats.present !=
        E2SM_KPM_EventTriggerDefinition__eventDefinition_formats_PR_eventDefinition_Format1)
    {
        errorMessage =
            "Only KPM Event Trigger Definition Format 1 is supported";
        return false;
    }

    const E2SM_KPM_EventTriggerDefinition_Format1_t*
        format1 =
            definition->eventDefinition_formats.choice
                .eventDefinition_Format1;

    if (format1 == nullptr)
    {
        errorMessage =
            "KPM Event Trigger Definition Format 1 is empty";
        return false;
    }

    if (format1->reportingPeriod == 0 ||
        format1->reportingPeriod >
            std::numeric_limits<uint32_t>::max())
    {
        errorMessage =
            "KPM reporting period is outside the supported range";
        return false;
    }

    reportingPeriodMs =
        static_cast<uint32_t>(
            format1->reportingPeriod);

    return true;
}
/**
 * Extract one named measurement with the noLabel label.
 *
 * The first supported contract accepts measurement names rather than numeric
 * IDs and exactly one noLabel entry for each requested measurement.
 */
bool
DecodeMeasurement(
    const MeasurementInfoItem_t* item,
    KpmV3MeasurementRequest& measurement,
    std::string& errorMessage)
{
    if (item == nullptr)
    {
        errorMessage = "Measurement information item is empty";
        return false;
    }

    if (item->measType.present !=
        MeasurementType_PR_measName)
    {
        errorMessage =
            "Only KPM measurements identified by name are supported";
        return false;
    }

    const MeasurementTypeName_t& measurementName =
        item->measType.choice.measName;

    if (measurementName.buf == nullptr ||
        measurementName.size == 0)
    {
        errorMessage = "KPM measurement name is empty";
        return false;
    }

    measurement.name.assign(
        reinterpret_cast<const char*>(
            measurementName.buf),
        measurementName.size);

    if (item->labelInfoList.list.count != 1 ||
        item->labelInfoList.list.array == nullptr)
    {
        errorMessage =
            "Each KPM measurement must contain exactly one label";
        return false;
    }

    const LabelInfoItem_t* labelItem =
        item->labelInfoList.list.array[0];

    if (labelItem == nullptr)
    {
        errorMessage = "KPM measurement label is empty";
        return false;
    }

    const long* noLabel =
        labelItem->measLabel.noLabel;

    if (noLabel == nullptr ||
        *noLabel != MeasurementLabel__noLabel_true)
    {
        errorMessage =
            "Only the KPM noLabel measurement label is supported";
        return false;
    }

    measurement.noLabel = true;
    return true;
}

/**
 * Decode a KPM Action Definition for report style 1.
 *
 * The supported format provides the granularity period and the named
 * measurements requested by the xApp.
 */
bool
DecodeActionDefinition(
    const uint8_t* data,
    std::size_t size,
    KpmV3SubscriptionRequest& subscription,
    std::string& errorMessage)
{
    void* decodedStructure = nullptr;

    const asn_dec_rval_t decodeResult =
        aper_decode_complete(
            nullptr,
            &asn_DEF_E2SM_KPM_ActionDefinition,
            &decodedStructure,
            data,
            size);

    ActionDefinitionPtr definition(
        static_cast<E2SM_KPM_ActionDefinition_t*>(
            decodedStructure),
        &FreeActionDefinition);

    if (decodeResult.code != RC_OK ||
        definition == nullptr)
    {
        errorMessage =
            "Could not decode the KPM Action Definition";
        return false;
    }

    if (definition->ric_Style_Type != 1)
    {
        errorMessage =
            "Only KPM report style 1 is supported";
        return false;
    }

    if (definition->actionDefinition_formats.present !=
        E2SM_KPM_ActionDefinition__actionDefinition_formats_PR_actionDefinition_Format1)
    {
        errorMessage =
            "Only KPM Action Definition Format 1 is supported";
        return false;
    }

    const E2SM_KPM_ActionDefinition_Format1_t*
        format1 =
            definition->actionDefinition_formats.choice
                .actionDefinition_Format1;

    if (format1 == nullptr)
    {
        errorMessage =
            "KPM Action Definition Format 1 is empty";
        return false;
    }

    if (format1->granulPeriod == 0 ||
        format1->granulPeriod >
            std::numeric_limits<uint32_t>::max())
    {
        errorMessage =
            "KPM granularity period is outside the supported range";
        return false;
    }

    const int measurementCount =
        format1->measInfoList.list.count;

    if (measurementCount <= 0 ||
        format1->measInfoList.list.array == nullptr)
    {
        errorMessage =
            "KPM Action Definition contains no measurements";
        return false;
    }

    std::vector<KpmV3MeasurementRequest>
        decodedMeasurements;

    decodedMeasurements.reserve(
        static_cast<std::size_t>(measurementCount));

    for (int index = 0;
         index < measurementCount;
         ++index)
    {
        KpmV3MeasurementRequest measurement;

        if (!DecodeMeasurement(
                format1->measInfoList.list.array[index],
                measurement,
                errorMessage))
        {
            errorMessage =
                "KPM measurement " +
                std::to_string(index) +
                ": " +
                errorMessage;

            return false;
        }

        decodedMeasurements.push_back(
            std::move(measurement));
    }

    subscription.reportStyle = 1;

    subscription.granularityPeriodMs =
        static_cast<uint32_t>(
            format1->granulPeriod);

    subscription.measurements =
        std::move(decodedMeasurements);

    return true;
}

} // namespace

/**
 * Validate and decode one supported KPM subscription payload.
 *
 * The initial contract supports Event Trigger Format 1 and Action Definition
 * Format 1 for report style 1 with named noLabel measurements.
 */
KpmV3DecodeResult
DecodeKpmV3Subscription(
    const uint8_t* eventTriggerData,
    std::size_t eventTriggerSize,
    const uint8_t* actionDefinitionData,
    std::size_t actionDefinitionSize)
{
    if (eventTriggerData == nullptr ||
        eventTriggerSize == 0)
    {
        return MakeDecodeError(
            "KPM Event Trigger Definition is empty");
    }

    if (actionDefinitionData == nullptr ||
        actionDefinitionSize == 0)
    {
        return MakeDecodeError(
            "KPM Action Definition is empty");
    }

    KpmV3DecodeResult result;

    if (!DecodeEventTrigger(
            eventTriggerData,
            eventTriggerSize,
            result.subscription.reportingPeriodMs,
            result.errorMessage))
    {
        return result;
    }

    if (!DecodeActionDefinition(
            actionDefinitionData,
            actionDefinitionSize,
            result.subscription,
            result.errorMessage))
    {
        return result;
    }

    result.success = true;
    return result;
}

} // namespace ns3
