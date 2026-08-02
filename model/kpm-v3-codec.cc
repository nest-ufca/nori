#include "kpm-v3-codec.h"

#include "E2SM-KPM-ActionDefinition-Format1.h"
#include "E2SM-KPM-ActionDefinition-Format5.h"
#include "E2SM-KPM-ActionDefinition.h"
#include "E2SM-KPM-EventTriggerDefinition-Format1.h"
#include "E2SM-KPM-EventTriggerDefinition.h"
#include "E2SM-KPM-IndicationHeader-Format1.h"
#include "E2SM-KPM-IndicationHeader.h"
#include "E2SM-KPM-IndicationMessage-Format1.h"
#include "E2SM-KPM-IndicationMessage-Format3.h"
#include "E2SM-KPM-IndicationMessage.h"
#include "LabelInfoItem.h"
#include "LabelInfoList.h"
#include "MatchingUEidPerSubItem.h"
#include "MatchingUEidPerSubList.h"
#include "MeasurementData.h"
#include "MeasurementDataItem.h"
#include "MeasurementInfoItem.h"
#include "MeasurementInfoList.h"
#include "MeasurementLabel.h"
#include "MeasurementRecord.h"
#include "MeasurementRecordItem.h"
#include "MeasurementType.h"
#include "MeasurementTypeName.h"
#include "OCTET_STRING.h"
#include "TimeStamp.h"
#include "UEID-GNB-DU.h"
#include "UEID.h"
#include "UEMeasurementReportItem.h"
#include "UEMeasurementReportList.h"
#include "aper_decoder.h"
#include "aper_encoder.h"

#include <array>
#include <cstdlib>
#include <limits>
#include <memory>
#include <set>
#include <utility>
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
 * Build a failed encoding result without terminating the simulator.
 */
KpmV3EncodeResult
MakeEncodeError(const std::string& message)
{
    KpmV3EncodeResult result;
    result.errorMessage = message;
    return result;
}

/**
 * Release an Indication Header allocated by the KPM v3 encoder.
 */
void
FreeIndicationHeader(
    E2SM_KPM_IndicationHeader_t* header)
{
    if (header != nullptr)
    {
        ASN_STRUCT_FREE(
            asn_DEF_E2SM_KPM_IndicationHeader,
            header);
    }
}

using IndicationHeaderPtr =
    std::unique_ptr<
        E2SM_KPM_IndicationHeader_t,
        decltype(&FreeIndicationHeader)>;

/**
 * Release an Indication Message allocated by the KPM v3 encoder.
 */
void
FreeIndicationMessage(
    E2SM_KPM_IndicationMessage_t* message)
{
    if (message != nullptr)
    {
        ASN_STRUCT_FREE(
            asn_DEF_E2SM_KPM_IndicationMessage,
            message);
    }
}

using IndicationMessagePtr =
    std::unique_ptr<
        E2SM_KPM_IndicationMessage_t,
        decltype(&FreeIndicationMessage)>;

/**
 * Encode one ASN.1 structure into a newly allocated APER byte buffer.
 *
 * The temporary ASN.1 buffer is copied into a C++ vector and released before
 * returning to the caller.
 */
bool
EncodeAsnStructure(
    const asn_TYPE_descriptor_t& descriptor,
    const void* structure,
    const std::string& structureName,
    std::vector<uint8_t>& encodedBytes,
    std::string& errorMessage)
{
    void* encodedBuffer = nullptr;

    const ssize_t encodedSize =
        aper_encode_to_new_buffer(
            &descriptor,
            nullptr,
            structure,
            &encodedBuffer);

    if (encodedSize <= 0 ||
        encodedBuffer == nullptr)
    {
        std::free(encodedBuffer);

        errorMessage =
            "Could not encode the KPM " +
            structureName;

        return false;
    }

    const auto* firstByte =
        static_cast<const uint8_t*>(
            encodedBuffer);

    encodedBytes.assign(
        firstByte,
        firstByte +
            static_cast<std::size_t>(
                encodedSize));

    std::free(encodedBuffer);
    return true;
}

/**
 * Convert Unix time in nanoseconds into the 64-bit NTP timestamp carried by
 * the KPM Indication Header.
 *
 * The upper 32 bits contain seconds since 1900 and the lower 32 bits contain
 * the fractional part of the second.
 */
bool
FillNtpTimestamp(
    uint64_t unixNanoseconds,
    TimeStamp_t& timestamp,
    std::string& errorMessage)
{
    constexpr uint64_t nanosecondsPerSecond =
        1000000000ULL;

    constexpr uint64_t ntpEpochOffsetSeconds =
        2208988800ULL;

    const uint64_t unixSeconds =
        unixNanoseconds /
        nanosecondsPerSecond;

    const uint64_t remainingNanoseconds =
        unixNanoseconds %
        nanosecondsPerSecond;

    if (unixSeconds >
        std::numeric_limits<uint32_t>::max() -
            ntpEpochOffsetSeconds)
    {
        errorMessage =
            "KPM collection timestamp is outside the supported NTP era";

        return false;
    }

    const uint64_t ntpSeconds =
        unixSeconds +
        ntpEpochOffsetSeconds;

    const uint64_t ntpFraction =
        (remainingNanoseconds << 32) /
        nanosecondsPerSecond;

    uint64_t ntpTimestamp =
        (ntpSeconds << 32) |
        ntpFraction;

    std::array<uint8_t, 8> timestampBytes{};

    for (std::size_t index = 0;
         index < timestampBytes.size();
         ++index)
    {
        timestampBytes[
            timestampBytes.size() - 1 - index] =
                static_cast<uint8_t>(
                    ntpTimestamp & 0xff);

        ntpTimestamp >>= 8;
    }

    if (OCTET_STRING_fromBuf(
            &timestamp,
            reinterpret_cast<const char*>(
                timestampBytes.data()),
            static_cast<int>(
                timestampBytes.size())) != 0)
    {
        errorMessage =
            "Could not allocate the KPM collection timestamp";

        return false;
    }

    return true;
}

/**
 * Build and encode KPM Indication Header Format 1.
 */
bool
EncodeStyle5IndicationHeader(
    const KpmV3Style5Indication& indication,
    std::vector<uint8_t>& encodedHeader,
    std::string& errorMessage)
{
    auto* header =
        static_cast<E2SM_KPM_IndicationHeader_t*>(
            std::calloc(
                1,
                sizeof(E2SM_KPM_IndicationHeader_t)));

    if (header == nullptr)
    {
        errorMessage =
            "Could not allocate the KPM Indication Header";

        return false;
    }

    IndicationHeaderPtr headerHolder(
        header,
        &FreeIndicationHeader);

    auto* format1 =
        static_cast<
            E2SM_KPM_IndicationHeader_Format1_t*>(
                std::calloc(
                    1,
                    sizeof(
                        E2SM_KPM_IndicationHeader_Format1_t)));

    if (format1 == nullptr)
    {
        errorMessage =
            "Could not allocate KPM Indication Header Format 1";

        return false;
    }

    header->indicationHeader_formats.present =
        E2SM_KPM_IndicationHeader__indicationHeader_formats_PR_indicationHeader_Format1;

    header->indicationHeader_formats.choice
        .indicationHeader_Format1 =
            format1;

    if (!FillNtpTimestamp(
            indication.collectStartTimeUnixNanoseconds,
            format1->colletStartTime,
            errorMessage))
    {
        return false;
    }

    return EncodeAsnStructure(
        asn_DEF_E2SM_KPM_IndicationHeader,
        header,
        "Indication Header",
        encodedHeader,
        errorMessage);
}

/**
 * Allocate and zero-initialize one generated ASN.1 structure.
 */
template <typename Structure>
Structure*
AllocateAsnStructure(
    const std::string& structureName,
    std::string& errorMessage)
{
    auto* structure =
        static_cast<Structure*>(
            std::calloc(
                1,
                sizeof(Structure)));

    if (structure == nullptr)
    {
        errorMessage =
            "Could not allocate KPM " +
            structureName;
    }

    return structure;
}

/**
 * Populate the measurement names and noLabel labels carried by one UE
 * measurement report.
 */
bool
PopulateMeasurementInfoList(
    MeasurementInfoList_t* measurementInfoList,
    const std::vector<std::string>& measurementNames,
    std::string& errorMessage)
{
    for (const std::string& measurementName :
         measurementNames)
    {
        auto* measurementInfo =
            AllocateAsnStructure<MeasurementInfoItem_t>(
                "MeasurementInfoItem",
                errorMessage);

        if (measurementInfo == nullptr)
        {
            return false;
        }

        if (ASN_SEQUENCE_ADD(
                &measurementInfoList->list,
                measurementInfo) != 0)
        {
            std::free(measurementInfo);

            errorMessage =
                "Could not append a KPM MeasurementInfoItem";

            return false;
        }

        measurementInfo->measType =
            AllocateAsnStructure<MeasurementType_t>(
                "MeasurementType",
                errorMessage);

        if (measurementInfo->measType == nullptr)
        {
            return false;
        }

        measurementInfo->measType->present =
            MeasurementType_PR_measName;

        if (OCTET_STRING_fromBuf(
                &measurementInfo->measType
                     ->choice.measName,
                measurementName.data(),
                static_cast<int>(
                    measurementName.size())) != 0)
        {
            errorMessage =
                "Could not allocate KPM measurement name " +
                measurementName;

            return false;
        }

        measurementInfo->labelInfoList =
            AllocateAsnStructure<LabelInfoList_t>(
                "LabelInfoList",
                errorMessage);

        if (measurementInfo->labelInfoList == nullptr)
        {
            return false;
        }

        auto* labelInfo =
            AllocateAsnStructure<LabelInfoItem_t>(
                "LabelInfoItem",
                errorMessage);

        if (labelInfo == nullptr)
        {
            return false;
        }

        if (ASN_SEQUENCE_ADD(
                &measurementInfo
                     ->labelInfoList
                     ->list,
                labelInfo) != 0)
        {
            std::free(labelInfo);

            errorMessage =
                "Could not append a KPM LabelInfoItem";

            return false;
        }

        labelInfo->measLabel =
            AllocateAsnStructure<MeasurementLabel_t>(
                "MeasurementLabel",
                errorMessage);

        if (labelInfo->measLabel == nullptr)
        {
            return false;
        }

        labelInfo->measLabel->noLabel =
            AllocateAsnStructure<long>(
                "noLabel value",
                errorMessage);

        if (labelInfo->measLabel->noLabel == nullptr)
        {
            return false;
        }

        *labelInfo->measLabel->noLabel =
            MeasurementLabel__noLabel_true;
    }

    return true;
}

/**
 * Append one explicitly selected gNB-DU UE and its measurements to a KPM
 * Indication Message Format 3 report list.
 */
bool
AppendStyle5UeReport(
    UEMeasurementReportList_t* ueReportList,
    const KpmV3Style5Indication& indication,
    const KpmV3Style5UeReport& ueReport,
    std::string& errorMessage)
{
    auto* reportItem =
        AllocateAsnStructure<UEMeasurementReportItem_t>(
            "UEMeasurementReportItem",
            errorMessage);

    if (reportItem == nullptr)
    {
        return false;
    }

    if (ASN_SEQUENCE_ADD(
            &ueReportList->list,
            reportItem) != 0)
    {
        std::free(reportItem);

        errorMessage =
            "Could not append a KPM UE measurement report";

        return false;
    }

    reportItem->ueID =
        AllocateAsnStructure<UEID_t>(
            "UEID",
            errorMessage);

    if (reportItem->ueID == nullptr)
    {
        return false;
    }

    reportItem->ueID->present =
        UEID_PR_gNB_DU_UEID;

    reportItem->ueID->choice.gNB_DU_UEID =
        AllocateAsnStructure<UEID_GNB_DU_t>(
            "UEID-GNB-DU",
            errorMessage);

    if (reportItem->ueID
            ->choice.gNB_DU_UEID == nullptr)
    {
        return false;
    }

    reportItem->ueID
        ->choice.gNB_DU_UEID
        ->gNB_CU_UE_F1AP_ID =
            static_cast<unsigned long>(
                ueReport.gnbCuUeF1apId);

    reportItem->measReport =
        AllocateAsnStructure<
            E2SM_KPM_IndicationMessage_Format1_t>(
                "Indication Message Format 1",
                errorMessage);

    if (reportItem->measReport == nullptr)
    {
        return false;
    }

    reportItem->measReport->measData =
        AllocateAsnStructure<MeasurementData_t>(
            "MeasurementData",
            errorMessage);

    if (reportItem->measReport->measData == nullptr)
    {
        return false;
    }

    auto* measurementDataItem =
        AllocateAsnStructure<MeasurementDataItem_t>(
            "MeasurementDataItem",
            errorMessage);

    if (measurementDataItem == nullptr)
    {
        return false;
    }

    if (ASN_SEQUENCE_ADD(
            &reportItem
                 ->measReport
                 ->measData
                 ->list,
            measurementDataItem) != 0)
    {
        std::free(measurementDataItem);

        errorMessage =
            "Could not append a KPM MeasurementDataItem";

        return false;
    }

    measurementDataItem->measRecord =
        AllocateAsnStructure<MeasurementRecord_t>(
            "MeasurementRecord",
            errorMessage);

    if (measurementDataItem->measRecord == nullptr)
    {
        return false;
    }

    for (uint64_t measurementValue :
         ueReport.measurementValues)
    {
        auto* recordItem =
            AllocateAsnStructure<MeasurementRecordItem_t>(
                "MeasurementRecordItem",
                errorMessage);

        if (recordItem == nullptr)
        {
            return false;
        }

        if (ASN_SEQUENCE_ADD(
                &measurementDataItem
                     ->measRecord
                     ->list,
                recordItem) != 0)
        {
            std::free(recordItem);

            errorMessage =
                "Could not append a KPM MeasurementRecordItem";

            return false;
        }

        recordItem->present =
            MeasurementRecordItem_PR_integer;

        recordItem->choice.integer =
            static_cast<unsigned long>(
                measurementValue);
    }

    reportItem->measReport->measInfoList =
        AllocateAsnStructure<MeasurementInfoList_t>(
            "MeasurementInfoList",
            errorMessage);

    if (reportItem->measReport
            ->measInfoList == nullptr)
    {
        return false;
    }

    if (!PopulateMeasurementInfoList(
            reportItem->measReport
                ->measInfoList,
            indication.measurementNames,
            errorMessage))
    {
        return false;
    }

    reportItem->measReport->granulPeriod =
        AllocateAsnStructure<GranularityPeriod_t>(
            "GranularityPeriod",
            errorMessage);

    if (reportItem->measReport
            ->granulPeriod == nullptr)
    {
        return false;
    }

    *reportItem->measReport->granulPeriod =
        static_cast<GranularityPeriod_t>(
            indication.granularityPeriodMs);

    return true;
}

/**
 * Build and encode KPM Indication Message Format 3.
 *
 * Format 3 carries one embedded Format 1 measurement report for every
 * explicitly selected gNB-DU UE.
 */
bool
EncodeStyle5IndicationMessage(
    const KpmV3Style5Indication& indication,
    std::vector<uint8_t>& encodedMessage,
    std::string& errorMessage)
{
    auto* message =
        AllocateAsnStructure<
            E2SM_KPM_IndicationMessage_t>(
                "Indication Message",
                errorMessage);

    if (message == nullptr)
    {
        return false;
    }

    IndicationMessagePtr messageHolder(
        message,
        &FreeIndicationMessage);

    auto* format3 =
        AllocateAsnStructure<
            E2SM_KPM_IndicationMessage_Format3_t>(
                "Indication Message Format 3",
                errorMessage);

    if (format3 == nullptr)
    {
        return false;
    }

    message->indicationMessage_formats.present =
        E2SM_KPM_IndicationMessage__indicationMessage_formats_PR_indicationMessage_Format3;

    message->indicationMessage_formats.choice
        .indicationMessage_Format3 =
            format3;

    format3->ueMeasReportList =
        AllocateAsnStructure<
            UEMeasurementReportList_t>(
                "UEMeasurementReportList",
                errorMessage);

    if (format3->ueMeasReportList == nullptr)
    {
        return false;
    }

    for (const KpmV3Style5UeReport& ueReport :
         indication.ueReports)
    {
        if (!AppendStyle5UeReport(
                format3->ueMeasReportList,
                indication,
                ueReport,
                errorMessage))
        {
            return false;
        }
    }

    return EncodeAsnStructure(
        asn_DEF_E2SM_KPM_IndicationMessage,
        message,
        "Indication Message",
        encodedMessage,
        errorMessage);
}

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

/**
 * Validate and encode one KPM Style 5 indication.
 */
KpmV3EncodeResult
EncodeKpmV3Style5Indication(
    const KpmV3Style5Indication& indication)
{
    if (indication.granularityPeriodMs == 0)
    {
        return MakeEncodeError(
            "KPM granularity period must be greater than zero");
    }

    if (indication.measurementNames.empty())
    {
        return MakeEncodeError(
            "KPM Style 5 indication contains no measurements");
    }

    if (indication.ueReports.empty())
    {
        return MakeEncodeError(
            "KPM Style 5 indication contains no UE reports");
    }

    std::set<std::string> uniqueMeasurementNames;

    for (const std::string& measurementName :
         indication.measurementNames)
    {
        if (measurementName.empty())
        {
            return MakeEncodeError(
                "KPM Style 5 indication contains an empty measurement name");
        }

        if (measurementName.size() >
            static_cast<std::size_t>(
                std::numeric_limits<int>::max()))
        {
            return MakeEncodeError(
                "KPM measurement name is too long");
        }

        if (!uniqueMeasurementNames
                 .insert(measurementName)
                 .second)
        {
            return MakeEncodeError(
                "KPM Style 5 indication contains a duplicate measurement name");
        }
    }

    std::set<uint64_t> uniqueUeIds;

    for (const KpmV3Style5UeReport& ueReport :
         indication.ueReports)
    {
        if (ueReport.gnbCuUeF1apId >
            std::numeric_limits<unsigned long>::max())
        {
            return MakeEncodeError(
                "gNB-CU UE F1AP ID is outside the supported range");
        }

        if (!uniqueUeIds
                 .insert(ueReport.gnbCuUeF1apId)
                 .second)
        {
            return MakeEncodeError(
                "KPM Style 5 indication contains a duplicate UE");
        }

        if (ueReport.measurementValues.size() !=
            indication.measurementNames.size())
        {
            return MakeEncodeError(
                "KPM UE measurement count does not match the measurement names");
        }

        for (uint64_t measurementValue :
             ueReport.measurementValues)
        {
            if (measurementValue >
                std::numeric_limits<unsigned long>::max())
            {
                return MakeEncodeError(
                    "KPM measurement value is outside the supported range");
            }
        }
    }

    KpmV3EncodeResult result;

    if (!EncodeStyle5IndicationHeader(
            indication,
            result.indicationHeader,
            result.errorMessage))
    {
        return result;
    }

    if (!EncodeStyle5IndicationMessage(
            indication,
            result.indicationMessage,
            result.errorMessage))
    {
        result.indicationHeader.clear();
        return result;
    }

    result.success = true;
    return result;
}

} // namespace ns3
