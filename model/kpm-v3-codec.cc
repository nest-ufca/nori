#include "kpm-v3-codec.h"

#include "E2SM-KPM-ActionDefinition-Format1.h"
#include "E2SM-KPM-ActionDefinition.h"
#include "LabelInfoItem.h"
#include "MeasurementInfoItem.h"
#include "E2SM-KPM-EventTriggerDefinition-Format1.h"
#include "E2SM-KPM-EventTriggerDefinition.h"
#include "E2SM-KPM-ActionDefinition-Format5.h"
#include "MatchingUEidPerSubItem.h"
#include "UEID-GNB-DU.h"
#include "aper_decoder.h"
#include "LabelInfoList.h"
#include "MeasurementInfoList.h"
#include "MeasurementLabel.h"
#include "MeasurementType.h"
#include "MeasurementTypeName.h"
#include "MatchingUEidPerSubList.h"
#include "UEID.h"

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

    if (item->measType == nullptr)
    {
        errorMessage = "KPM measurement type is empty";
        return false;
    }

    if (item->measType->present !=
        MeasurementType_PR_measName)
    {
        errorMessage =
            "Only KPM measurements identified by name are supported";
        return false;
    }

    const MeasurementTypeName_t& measurementName =
        item->measType->choice.measName;

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

    if (item->labelInfoList == nullptr ||
        item->labelInfoList->list.count != 1 ||
        item->labelInfoList->list.array == nullptr)
    {
        errorMessage =
            "Each KPM measurement must contain exactly one label";
        return false;
    }

    const LabelInfoItem_t* labelItem =
        item->labelInfoList->list.array[0];

    if (labelItem == nullptr ||
        labelItem->measLabel == nullptr)
    {
        errorMessage = "KPM measurement label is empty";
        return false;
    }

    const long* noLabel =
        labelItem->measLabel->noLabel;

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
 * Decode the measurement selection shared by KPM Action Definition
 * Formats 1 and 5.
 *
 * Format 5 embeds this Format 1 structure as its subscriptionInfo field.
 */
bool
DecodeFormat1SubscriptionInfo(
    const E2SM_KPM_ActionDefinition_Format1_t* format1,
    KpmV3SubscriptionRequest& subscription,
    std::string& errorMessage)
{
    if (format1 == nullptr)
    {
        errorMessage =
            "KPM Action Definition Format 1 information is empty";
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

    if (format1->measInfoList == nullptr ||
        format1->measInfoList->list.count <= 0 ||
        format1->measInfoList->list.array == nullptr)
    {
        errorMessage =
            "KPM Action Definition contains no measurements";
        return false;
    }

    const int measurementCount =
        format1->measInfoList->list.count;

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
                format1->measInfoList->list.array[index],
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

    subscription.granularityPeriodMs =
        static_cast<uint32_t>(
            format1->granulPeriod);

    subscription.measurements =
        std::move(decodedMeasurements);

    return true;
}

/**
 * Decode the explicit gNB-DU UE selection carried by KPM Action Definition
 * Format 5.
 *
 * The initial Style 5 contract supports UEID-GNB-DU identities containing a
 * gNB-CU UE F1AP ID. Other standardized UE identity alternatives are rejected
 * explicitly until their required fields are implemented.
 */
bool
DecodeStyle5MatchingUes(
    const MatchingUEidPerSubList_t* matchingUeIdList,
    KpmV3SubscriptionRequest& subscription,
    std::string& errorMessage)
{
    if (matchingUeIdList == nullptr ||
        matchingUeIdList->list.count <= 0 ||
        matchingUeIdList->list.array == nullptr)
    {
        errorMessage =
            "KPM Action Definition Format 5 contains no matching UEs";
        return false;
    }

    const int ueCount =
        matchingUeIdList->list.count;

    std::vector<KpmV3GnbDuUeRequest>
        decodedMatchingUes;

    decodedMatchingUes.reserve(
        static_cast<std::size_t>(ueCount));

    for (int index = 0;
         index < ueCount;
         ++index)
    {
        const MatchingUEidPerSubItem_t* item =
            matchingUeIdList->list.array[index];

        if (item == nullptr)
        {
            errorMessage =
                "KPM matching UE " +
                std::to_string(index) +
                " is empty";
            return false;
        }

        if (item->ueID == nullptr ||
            item->ueID->present !=
                UEID_PR_gNB_DU_UEID ||
            item->ueID->choice.gNB_DU_UEID == nullptr)
        {
            errorMessage =
                "KPM matching UE " +
                std::to_string(index) +
                ": only UEID-GNB-DU identities are supported";
            return false;
        }

        KpmV3GnbDuUeRequest matchingUe;

        matchingUe.gnbCuUeF1apId =
            static_cast<uint64_t>(
                item->ueID->choice
                    .gNB_DU_UEID
                    ->gNB_CU_UE_F1AP_ID);

        decodedMatchingUes.push_back(
            matchingUe);
    }

    subscription.matchingUes =
        std::move(decodedMatchingUes);

    return true;
}

/**
 * Decode the KPM Action Definition formats supported by NORI.
 *
 * Style 1 selects cell-level measurements directly through Format 1. Style 5
 * selects explicit gNB-DU UEs through Format 5 and embeds the same Format 1
 * measurement configuration as subscriptionInfo.
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
            "Could not decode the KPM Action Definition"
            " (code=" + std::to_string( static_cast<int>(decodeResult.code)) +
            ", consumed=" + std::to_string(decodeResult.consumed) +
            ", size=" + std::to_string(size) +")";
        return false;
    }

    switch (definition->ric_Style_Type)
    {
    case 1: {
        if (definition->actionDefinition_formats.present !=
            E2SM_KPM_ActionDefinition__actionDefinition_formats_PR_actionDefinition_Format1)
        {
            errorMessage =
                "KPM report style 1 requires Action Definition Format 1";
            return false;
        }

        const E2SM_KPM_ActionDefinition_Format1_t*
            format1 =
                definition->actionDefinition_formats.choice
                    .actionDefinition_Format1;

        if (!DecodeFormat1SubscriptionInfo(
                format1,
                subscription,
                errorMessage))
        {
            return false;
        }

        subscription.reportStyle = 1;
        subscription.matchingUes.clear();
        return true;
    }

    case 5: {
        if (definition->actionDefinition_formats.present !=
            E2SM_KPM_ActionDefinition__actionDefinition_formats_PR_actionDefinition_Format5)
        {
            errorMessage =
                "KPM report style 5 requires Action Definition Format 5";
            return false;
        }

        const E2SM_KPM_ActionDefinition_Format5_t*
            format5 =
                definition->actionDefinition_formats.choice
                    .actionDefinition_Format5;

        if (format5 == nullptr)
        {
            errorMessage =
                "KPM Action Definition Format 5 is empty";
            return false;
        }

        if (!DecodeFormat1SubscriptionInfo(
                format5->subscriptionInfo,
                subscription,
                errorMessage))
        {
            return false;
        }

        if (!DecodeStyle5MatchingUes(
                format5->matchingUEidList,
                subscription,
                errorMessage))
        {
            return false;
        }

        subscription.reportStyle = 5;
        return true;
    }

    default:
        errorMessage =
            "Only KPM report styles 1 and 5 are supported";
        return false;
    }
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
