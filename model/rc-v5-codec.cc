#include "rc-v5-codec.h"

#include "ControlAction-RANParameter-Item.h"
#include "E2SM-RC-ControlHeader.h"
#include "E2SM-RC-ControlMessage.h"
#include "E2SM-RC-RANFunctionDefinition.h"
#include "RANFunctionDefinition-Control-Action-Item.h"
#include "RANFunctionDefinition-Control-Item.h"
#include "RANFunctionDefinition-Control.h"
#include "RANParameter-LIST.h"
#include "RANParameter-STRUCTURE.h"
#include "RANParameter-Value.h"
#include "RANParameter-ValueType.h"
#include "RANfunction-Name.h"
#include "UEID-GNB-DU.h"
#include "aper_decoder.h"
#include "aper_encoder.h"
#include "asn_SEQUENCE_OF.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

namespace ns3
{

namespace
{

constexpr long CONTROL_STYLE = 2;
constexpr long CONTROL_ACTION = 6;

constexpr RANParameter_ID_t RRM_POLICY_RATIO_LIST = 1;
constexpr RANParameter_ID_t RRM_POLICY_RATIO_GROUP = 2;
constexpr RANParameter_ID_t RRM_POLICY = 3;
constexpr RANParameter_ID_t RRM_POLICY_MEMBER_LIST = 5;
constexpr RANParameter_ID_t RRM_POLICY_MEMBER = 6;
constexpr RANParameter_ID_t PLMN_IDENTITY = 7;
constexpr RANParameter_ID_t S_NSSAI = 8;
constexpr RANParameter_ID_t SST = 9;
constexpr RANParameter_ID_t SD = 10;
constexpr RANParameter_ID_t MIN_PRB_RATIO = 11;
constexpr RANParameter_ID_t MAX_PRB_RATIO = 12;
constexpr RANParameter_ID_t DEDICATED_PRB_RATIO = 13;

constexpr std::array<std::pair<RANParameter_ID_t, const char*>, 12> CONTROL_PARAMETERS{{
    {RRM_POLICY_RATIO_LIST, "RRM Policy Ratio List"},
    {RRM_POLICY_RATIO_GROUP, "RRM Policy Ratio Group"},
    {RRM_POLICY, "RRM Policy"},
    {RRM_POLICY_MEMBER_LIST, "RRM Policy Member List"},
    {RRM_POLICY_MEMBER, "RRM Policy Member"},
    {PLMN_IDENTITY, "PLMN Identity"},
    {S_NSSAI, "S-NSSAI"},
    {SST, "SST"},
    {SD, "SD"},
    {MIN_PRB_RATIO, "Minimum PRB Policy Ratio"},
    {MAX_PRB_RATIO, "Maximum PRB Policy Ratio"},
    {DEDICATED_PRB_RATIO, "Dedicated PRB Policy Ratio"},
}};

void
FreeRanFunctionDefinition(E2SM_RC_RANFunctionDefinition_t* definition)
{
    ASN_STRUCT_FREE(asn_DEF_E2SM_RC_RANFunctionDefinition, definition);
}

template <typename Structure>
Structure*
AllocateAsnStructure(const std::string& structureName, std::string& errorMessage)
{
    auto* structure = static_cast<Structure*>(std::calloc(1, sizeof(Structure)));

    if (structure == nullptr)
    {
        errorMessage = "Could not allocate " + structureName;
    }

    return structure;
}

template <typename AsnString>
bool
AssignAsnString(
    AsnString& destination,
    const char* value,
    const std::string& fieldName,
    std::string& errorMessage)
{
    const std::size_t valueSize = std::strlen(value);

    destination.buf = static_cast<uint8_t*>(std::calloc(valueSize, sizeof(uint8_t)));

    if (destination.buf == nullptr)
    {
        errorMessage = "Could not allocate " + fieldName;
        return false;
    }

    std::memcpy(destination.buf, value, valueSize);
    destination.size = valueSize;
    return true;
}

using StructureItem = RANParameter_STRUCTURE_Item_t;

void
FreeControlHeader(E2SM_RC_ControlHeader_t* header)
{
    ASN_STRUCT_FREE(asn_DEF_E2SM_RC_ControlHeader, header);
}

void
FreeControlMessage(E2SM_RC_ControlMessage_t* message)
{
    ASN_STRUCT_FREE(asn_DEF_E2SM_RC_ControlMessage, message);
}

bool
ExpectStructureSize(
    const RANParameter_STRUCTURE_t* structure,
    int expectedSize,
    const std::string& context,
    std::string& errorMessage)
{
    if (structure == nullptr ||
        structure->sequence_of_ranParameters == nullptr)
    {
        errorMessage = context + " is missing";
        return false;
    }

    const int actualSize =
        structure->sequence_of_ranParameters->list.count;

    if (actualSize != expectedSize)
    {
        errorMessage =
            context + " must contain " +
            std::to_string(expectedSize) +
            " parameters, but contains " +
            std::to_string(actualSize);
        return false;
    }

    return true;
}

bool
FindStructureItem(
    const RANParameter_STRUCTURE_t* structure,
    RANParameter_ID_t parameterId,
    bool required,
    const std::string& context,
    const StructureItem*& result,
    std::string& errorMessage)
{
    result = nullptr;

    if (structure == nullptr ||
        structure->sequence_of_ranParameters == nullptr)
    {
        errorMessage = context + " is missing";
        return false;
    }

    const auto& list =
        structure->sequence_of_ranParameters->list;

    for (int index = 0; index < list.count; ++index)
    {
        const StructureItem* item = list.array[index];

        if (item == nullptr)
        {
            errorMessage =
                context + " contains a null parameter";
            return false;
        }

        if (item->ranParameter_ID == parameterId)
        {
            if (result != nullptr)
            {
                errorMessage =
                    context + " contains duplicated parameter " +
                    std::to_string(parameterId);
                return false;
            }

            result = item;
        }
    }

    if (required && result == nullptr)
    {
        errorMessage =
            context + " does not contain parameter " +
            std::to_string(parameterId);
        return false;
    }

    return true;
}

bool
GetStructure(
    const RANParameter_ValueType_t* valueType,
    const std::string& context,
    const RANParameter_STRUCTURE_t*& result,
    std::string& errorMessage)
{
    result = nullptr;

    if (valueType == nullptr ||
        valueType->present !=
            RANParameter_ValueType_PR_ranP_Choice_Structure ||
        valueType->choice.ranP_Choice_Structure == nullptr ||
        valueType->choice.ranP_Choice_Structure
                ->ranParameter_Structure == nullptr)
    {
        errorMessage =
            context + " is not a RAN parameter structure";
        return false;
    }

    result =
        valueType->choice.ranP_Choice_Structure
            ->ranParameter_Structure;

    return true;
}

bool
GetList(
    const RANParameter_ValueType_t* valueType,
    const std::string& context,
    const RANParameter_LIST_t*& result,
    std::string& errorMessage)
{
    result = nullptr;

    if (valueType == nullptr ||
        valueType->present !=
            RANParameter_ValueType_PR_ranP_Choice_List ||
        valueType->choice.ranP_Choice_List == nullptr ||
        valueType->choice.ranP_Choice_List
                ->ranParameter_List == nullptr)
    {
        errorMessage =
            context + " is not a RAN parameter list";
        return false;
    }

    result =
        valueType->choice.ranP_Choice_List
            ->ranParameter_List;

    return true;
}

bool
GetElementValue(
    const StructureItem* item,
    const std::string& context,
    const RANParameter_Value_t*& result,
    std::string& errorMessage)
{
    result = nullptr;

    if (item == nullptr ||
        item->ranParameter_valueType == nullptr ||
        item->ranParameter_valueType->present !=
            RANParameter_ValueType_PR_ranP_Choice_ElementFalse ||
        item->ranParameter_valueType->choice
                .ranP_Choice_ElementFalse == nullptr ||
        item->ranParameter_valueType->choice
                .ranP_Choice_ElementFalse
                ->ranParameter_value == nullptr)
    {
        errorMessage =
            context + " is not an element-false value";
        return false;
    }

    result =
        item->ranParameter_valueType->choice
            .ranP_Choice_ElementFalse
            ->ranParameter_value;

    return true;
}

template <std::size_t Size>
bool
ReadOctetString(
    const StructureItem* item,
    const std::string& context,
    std::array<uint8_t, Size>& result,
    std::string& errorMessage)
{
    const RANParameter_Value_t* value = nullptr;

    if (!GetElementValue(
            item,
            context,
            value,
            errorMessage))
    {
        return false;
    }

    if (value->present !=
            RANParameter_Value_PR_valueOctS ||
        value->choice.valueOctS.buf == nullptr ||
        value->choice.valueOctS.size != Size)
    {
        errorMessage =
            context + " must contain exactly " +
            std::to_string(Size) + " octets";
        return false;
    }

    std::copy_n(
        value->choice.valueOctS.buf,
        Size,
        result.begin());

    return true;
}

bool
ReadRatio(
    const StructureItem* item,
    const std::string& context,
    uint32_t& result,
    std::string& errorMessage)
{
    const RANParameter_Value_t* value = nullptr;

    if (!GetElementValue(
            item,
            context,
            value,
            errorMessage))
    {
        return false;
    }

    if (value->present !=
            RANParameter_Value_PR_valueInt ||
        value->choice.valueInt < 0 ||
        value->choice.valueInt > 100)
    {
        errorMessage =
            context + " must be an integer from 0 to 100";
        return false;
    }

    result =
        static_cast<uint32_t>(
            value->choice.valueInt);

    return true;
}

bool
DecodeSliceQuota(
    const RANParameter_STRUCTURE_t* ratioGroupWrapper,
    RcV5SliceQuota& quota,
    std::string& errorMessage)
{
    if (!ExpectStructureSize(
            ratioGroupWrapper,
            1,
            "RRM policy ratio group wrapper",
            errorMessage))
    {
        return false;
    }

    const StructureItem* ratioGroupItem = nullptr;

    if (!FindStructureItem(
            ratioGroupWrapper,
            RRM_POLICY_RATIO_GROUP,
            true,
            "RRM policy ratio group wrapper",
            ratioGroupItem,
            errorMessage))
    {
        return false;
    }

    const RANParameter_STRUCTURE_t* ratioGroup = nullptr;

    if (!GetStructure(
            ratioGroupItem->ranParameter_valueType,
            "RRM policy ratio group",
            ratioGroup,
            errorMessage) ||
        !ExpectStructureSize(
            ratioGroup,
            4,
            "RRM policy ratio group",
            errorMessage))
    {
        return false;
    }

    const StructureItem* policyItem = nullptr;
    const StructureItem* minRatioItem = nullptr;
    const StructureItem* maxRatioItem = nullptr;
    const StructureItem* dedicatedRatioItem = nullptr;

    if (!FindStructureItem(
            ratioGroup,
            RRM_POLICY,
            true,
            "RRM policy ratio group",
            policyItem,
            errorMessage) ||
        !FindStructureItem(
            ratioGroup,
            MIN_PRB_RATIO,
            true,
            "RRM policy ratio group",
            minRatioItem,
            errorMessage) ||
        !FindStructureItem(
            ratioGroup,
            MAX_PRB_RATIO,
            true,
            "RRM policy ratio group",
            maxRatioItem,
            errorMessage) ||
        !FindStructureItem(
            ratioGroup,
            DEDICATED_PRB_RATIO,
            true,
            "RRM policy ratio group",
            dedicatedRatioItem,
            errorMessage))
    {
        return false;
    }

    const RANParameter_STRUCTURE_t* policy = nullptr;

    if (!GetStructure(
            policyItem->ranParameter_valueType,
            "RRM policy",
            policy,
            errorMessage) ||
        !ExpectStructureSize(
            policy,
            1,
            "RRM policy",
            errorMessage))
    {
        return false;
    }

    const StructureItem* memberListItem = nullptr;

    if (!FindStructureItem(
            policy,
            RRM_POLICY_MEMBER_LIST,
            true,
            "RRM policy",
            memberListItem,
            errorMessage))
    {
        return false;
    }

    const RANParameter_LIST_t* memberList = nullptr;

    if (!GetList(
            memberListItem->ranParameter_valueType,
            "RRM policy member list",
            memberList,
            errorMessage))
    {
        return false;
    }

    if (memberList->list_of_ranParameter.list.count != 1 ||
        memberList->list_of_ranParameter.list.array[0] == nullptr)
    {
        errorMessage =
            "RRM policy member list must contain exactly one member";
        return false;
    }

    const RANParameter_STRUCTURE_t* memberWrapper =
        memberList->list_of_ranParameter.list.array[0];

    if (!ExpectStructureSize(
            memberWrapper,
            1,
            "RRM policy member wrapper",
            errorMessage))
    {
        return false;
    }

    const StructureItem* memberItem = nullptr;

    if (!FindStructureItem(
            memberWrapper,
            RRM_POLICY_MEMBER,
            true,
            "RRM policy member wrapper",
            memberItem,
            errorMessage))
    {
        return false;
    }

    const RANParameter_STRUCTURE_t* member = nullptr;

    if (!GetStructure(
            memberItem->ranParameter_valueType,
            "RRM policy member",
            member,
            errorMessage) ||
        !ExpectStructureSize(
            member,
            2,
            "RRM policy member",
            errorMessage))
    {
        return false;
    }

    const StructureItem* plmnItem = nullptr;
    const StructureItem* snssaiItem = nullptr;

    if (!FindStructureItem(
            member,
            PLMN_IDENTITY,
            true,
            "RRM policy member",
            plmnItem,
            errorMessage) ||
        !FindStructureItem(
            member,
            S_NSSAI,
            true,
            "RRM policy member",
            snssaiItem,
            errorMessage) ||
        !ReadOctetString(
            plmnItem,
            "PLMN identity",
            quota.plmnIdentity,
            errorMessage))
    {
        return false;
    }

    const RANParameter_STRUCTURE_t* snssai = nullptr;

    if (!GetStructure(
            snssaiItem->ranParameter_valueType,
            "S-NSSAI",
            snssai,
            errorMessage))
    {
        return false;
    }

    if (snssai->sequence_of_ranParameters == nullptr)
    {
        errorMessage = "S-NSSAI is missing";
        return false;
    }

    const int snssaiSize =
        snssai->sequence_of_ranParameters->list.count;

    if (snssaiSize != 1 && snssaiSize != 2)
    {
        errorMessage =
            "S-NSSAI must contain SST and optional SD";
        return false;
    }

    const StructureItem* sstItem = nullptr;
    const StructureItem* sdItem = nullptr;

    if (!FindStructureItem(
            snssai,
            SST,
            true,
            "S-NSSAI",
            sstItem,
            errorMessage) ||
        !FindStructureItem(
            snssai,
            SD,
            false,
            "S-NSSAI",
            sdItem,
            errorMessage))
    {
        return false;
    }

    std::array<uint8_t, 1> encodedSst{};

    if (!ReadOctetString(
            sstItem,
            "SST",
            encodedSst,
            errorMessage))
    {
        return false;
    }

    quota.sst = encodedSst[0];

    if (quota.sst == 0)
    {
        errorMessage = "SST must be in the range 1 to 255";
        return false;
    }

    if (sdItem != nullptr)
    {
        std::array<uint8_t, 3> encodedSd{};

        if (!ReadOctetString(
                sdItem,
                "SD",
                encodedSd,
                errorMessage))
        {
            return false;
        }

        quota.sd = encodedSd;
    }

    if (!ReadRatio(
            minRatioItem,
            "Minimum PRB ratio",
            quota.minPrbRatio,
            errorMessage) ||
        !ReadRatio(
            maxRatioItem,
            "Maximum PRB ratio",
            quota.maxPrbRatio,
            errorMessage) ||
        !ReadRatio(
            dedicatedRatioItem,
            "Dedicated PRB ratio",
            quota.dedicatedPrbRatio,
            errorMessage))
    {
        return false;
    }

    if (quota.dedicatedPrbRatio > quota.minPrbRatio ||
        quota.minPrbRatio > quota.maxPrbRatio)
    {
        errorMessage =
            "PRB ratios must satisfy dedicated <= minimum <= maximum";
        return false;
    }

    return true;
}

std::string
MakeDecodeError(
    const std::string& payloadName,
    const asn_dec_rval_t& decodeResult,
    std::size_t payloadSize)
{
    return
        "Could not decode the E2SM-RC " +
        payloadName +
        " (code=" +
        std::to_string(decodeResult.code) +
        ", consumed=" +
        std::to_string(decodeResult.consumed) +
        ", size=" +
        std::to_string(payloadSize) +
        ")";
}

} // namespace

RcV5FunctionDescriptionResult
EncodeRcV5FunctionDescription()
{
    RcV5FunctionDescriptionResult result;

    auto* rawDefinition =
        AllocateAsnStructure<E2SM_RC_RANFunctionDefinition_t>(
            "E2SM-RC RAN Function Definition",
            result.errorMessage);

    if (rawDefinition == nullptr)
    {
        return result;
    }

    std::unique_ptr<E2SM_RC_RANFunctionDefinition_t, decltype(&FreeRanFunctionDefinition)>
        definition(rawDefinition, &FreeRanFunctionDefinition);

    definition->ranFunction_Name =
        AllocateAsnStructure<RANfunction_Name_t>(
            "E2SM-RC RAN Function Name",
            result.errorMessage);

    if (definition->ranFunction_Name == nullptr)
    {
        return result;
    }

    if (!AssignAsnString(
            definition->ranFunction_Name->ranFunction_ShortName,
            "ORAN-WG3-E2SM-RC",
            "RAN function short name",
            result.errorMessage) ||
        !AssignAsnString(
            definition->ranFunction_Name->ranFunction_E2SM_OID,
            "1.3.6.1.4.1.53148.1.1.2.3",
            "RAN function OID",
            result.errorMessage) ||
        !AssignAsnString(
            definition->ranFunction_Name->ranFunction_Description,
            "NEST E2SM-RC v5 slice quota control",
            "RAN function description",
            result.errorMessage))
    {
        return result;
    }

    definition->ranFunctionDefinition_Control =
        AllocateAsnStructure<RANFunctionDefinition_Control_t>(
            "E2SM-RC Control Function Definition",
            result.errorMessage);

    if (definition->ranFunctionDefinition_Control == nullptr)
    {
        return result;
    }

    auto* style =
        AllocateAsnStructure<RANFunctionDefinition_Control_Item_t>(
            "E2SM-RC Control Style",
            result.errorMessage);

    if (style == nullptr)
    {
        return result;
    }

    if (ASN_SEQUENCE_ADD(
            &definition->ranFunctionDefinition_Control->ric_ControlStyle_List.list,
            style) != 0)
    {
        ASN_STRUCT_FREE(asn_DEF_RANFunctionDefinition_Control_Item, style);
        result.errorMessage = "Could not add E2SM-RC Control Style";
        return result;
    }

    style->ric_ControlStyle_Type = CONTROL_STYLE;
    style->ric_ControlHeaderFormat_Type = 1;
    style->ric_ControlMessageFormat_Type = 1;
    style->ric_ControlOutcomeFormat_Type = 1;

    if (!AssignAsnString(
            style->ric_ControlStyle_Name,
            "Radio resource allocation control",
            "RC Control Style name",
            result.errorMessage))
    {
        return result;
    }

    style->ric_ControlAction_List =
        static_cast<decltype(style->ric_ControlAction_List)>(
            std::calloc(1, sizeof(*style->ric_ControlAction_List)));

    if (style->ric_ControlAction_List == nullptr)
    {
        result.errorMessage = "Could not allocate RC Control Action list";
        return result;
    }

    auto* action =
        AllocateAsnStructure<RANFunctionDefinition_Control_Action_Item_t>(
            "E2SM-RC Control Action",
            result.errorMessage);

    if (action == nullptr)
    {
        return result;
    }

    if (ASN_SEQUENCE_ADD(&style->ric_ControlAction_List->list, action) != 0)
    {
        ASN_STRUCT_FREE(asn_DEF_RANFunctionDefinition_Control_Action_Item, action);
        result.errorMessage = "Could not add E2SM-RC Control Action";
        return result;
    }

    action->ric_ControlAction_ID = CONTROL_ACTION;

    if (!AssignAsnString(
            action->ric_ControlAction_Name,
            "Slice-level PRB quota control",
            "RC Control Action name",
            result.errorMessage))
    {
        return result;
    }

    action->ran_ControlActionParameters_List =
        static_cast<decltype(action->ran_ControlActionParameters_List)>(
            std::calloc(1, sizeof(*action->ran_ControlActionParameters_List)));

    if (action->ran_ControlActionParameters_List == nullptr)
    {
        result.errorMessage = "Could not allocate RC Control Action parameter list";
        return result;
    }

    for (const auto& [parameterId, parameterName] : CONTROL_PARAMETERS)
    {
        auto* parameter =
            AllocateAsnStructure<ControlAction_RANParameter_Item_t>(
                "E2SM-RC Control Action parameter",
                result.errorMessage);

        if (parameter == nullptr)
        {
            return result;
        }

        if (ASN_SEQUENCE_ADD(
                &action->ran_ControlActionParameters_List->list,
                parameter) != 0)
        {
            ASN_STRUCT_FREE(asn_DEF_ControlAction_RANParameter_Item, parameter);
            result.errorMessage = "Could not add RC Control Action parameter";
            return result;
        }

        parameter->ranParameter_ID = parameterId;

        if (!AssignAsnString(
                parameter->ranParameter_name,
                parameterName,
                "RC Control Action parameter name",
                result.errorMessage))
        {
            return result;
        }
    }

    void* encodedBuffer = nullptr;

    const ssize_t encodedSize =
        aper_encode_to_new_buffer(
            &asn_DEF_E2SM_RC_RANFunctionDefinition,
            nullptr,
            definition.get(),
            &encodedBuffer);

    if (encodedSize <= 0 || encodedBuffer == nullptr)
    {
        std::free(encodedBuffer);
        result.errorMessage = "Could not encode the E2SM-RC RAN Function Definition";
        return result;
    }

    const auto* firstByte = static_cast<const uint8_t*>(encodedBuffer);

    result.encodedDefinition.assign(
        firstByte,
        firstByte + static_cast<std::size_t>(encodedSize));

    std::free(encodedBuffer);
    result.success = true;
    return result;
}

RcV5DecodeResult
DecodeRcV5Control(
    const uint8_t* controlHeaderData,
    std::size_t controlHeaderSize,
    const uint8_t* controlMessageData,
    std::size_t controlMessageSize)
{
    RcV5DecodeResult result;

    if (controlHeaderData == nullptr ||
        controlHeaderSize == 0)
    {
        result.errorMessage =
            "E2SM-RC Control Header is empty";
        return result;
    }

    if (controlMessageData == nullptr ||
        controlMessageSize == 0)
    {
        result.errorMessage =
            "E2SM-RC Control Message is empty";
        return result;
    }

    E2SM_RC_ControlHeader_t* decodedHeader = nullptr;

    const asn_dec_rval_t headerDecodeResult =
        aper_decode_complete(
            nullptr,
            &asn_DEF_E2SM_RC_ControlHeader,
            reinterpret_cast<void**>(&decodedHeader),
            controlHeaderData,
            controlHeaderSize);

    std::unique_ptr<
        E2SM_RC_ControlHeader_t,
        decltype(&FreeControlHeader)>
        header(decodedHeader, &FreeControlHeader);

    if (headerDecodeResult.code != RC_OK ||
        decodedHeader == nullptr)
    {
        result.errorMessage =
            MakeDecodeError(
                "Control Header",
                headerDecodeResult,
                controlHeaderSize);
        return result;
    }

    if (decodedHeader->ric_controlHeader_formats.present !=
        E2SM_RC_ControlHeader__ric_controlHeader_formats_PR_controlHeader_Format1 ||
        decodedHeader->ric_controlHeader_formats.choice
                .controlHeader_Format1 == nullptr)
    {
        result.errorMessage =
            "Only E2SM-RC Control Header Format 1 is supported";
        return result;
    }

    const E2SM_RC_ControlHeader_Format1_t* headerFormat1 =
        decodedHeader->ric_controlHeader_formats.choice
            .controlHeader_Format1;

    if (headerFormat1->ric_Style_Type != CONTROL_STYLE)
    {
        result.errorMessage =
            "Only E2SM-RC control Style 2 is supported";
        return result;
    }

    if (headerFormat1->ric_ControlAction_ID != CONTROL_ACTION)
    {
        result.errorMessage =
            "Only E2SM-RC control Action 6 is supported";
        return result;
    }

    if (headerFormat1->ueID == nullptr ||
        headerFormat1->ueID->present !=
            UEID_PR_gNB_DU_UEID ||
        headerFormat1->ueID->choice.gNB_DU_UEID == nullptr)
    {
        result.errorMessage =
            "Only the gNB-DU UE identity is supported";
        return result;
    }

    result.control.gnbCuUeF1apId =
        static_cast<uint64_t>(
            headerFormat1->ueID->choice.gNB_DU_UEID
                ->gNB_CU_UE_F1AP_ID);

    result.control.controlStyle =
        static_cast<uint32_t>(CONTROL_STYLE);

    result.control.controlAction =
        static_cast<uint32_t>(CONTROL_ACTION);

    E2SM_RC_ControlMessage_t* decodedMessage = nullptr;

    const asn_dec_rval_t messageDecodeResult =
        aper_decode_complete(
            nullptr,
            &asn_DEF_E2SM_RC_ControlMessage,
            reinterpret_cast<void**>(&decodedMessage),
            controlMessageData,
            controlMessageSize);

    std::unique_ptr<
        E2SM_RC_ControlMessage_t,
        decltype(&FreeControlMessage)>
        message(decodedMessage, &FreeControlMessage);

    if (messageDecodeResult.code != RC_OK ||
        decodedMessage == nullptr)
    {
        result.errorMessage =
            MakeDecodeError(
                "Control Message",
                messageDecodeResult,
                controlMessageSize);
        return result;
    }

    if (decodedMessage->ric_controlMessage_formats.present !=
        E2SM_RC_ControlMessage__ric_controlMessage_formats_PR_controlMessage_Format1 ||
        decodedMessage->ric_controlMessage_formats.choice
                .controlMessage_Format1 == nullptr)
    {
        result.errorMessage =
            "Only E2SM-RC Control Message Format 1 is supported";
        return result;
    }

    const E2SM_RC_ControlMessage_Format1_t* messageFormat1 =
        decodedMessage->ric_controlMessage_formats.choice
            .controlMessage_Format1;

    if (messageFormat1->ranP_List.list.count != 1 ||
        messageFormat1->ranP_List.list.array[0] == nullptr)
    {
        result.errorMessage =
            "Control Message must contain one RRM policy ratio list";
        return result;
    }

    const E2SM_RC_ControlMessage_Format1_Item_t* ratioListItem =
        messageFormat1->ranP_List.list.array[0];

    if (ratioListItem->ranParameter_ID !=
        RRM_POLICY_RATIO_LIST)
    {
        result.errorMessage =
            "Control Message does not start with parameter 1";
        return result;
    }

    const RANParameter_LIST_t* ratioList = nullptr;

    if (!GetList(
            ratioListItem->ranParameter_valueType,
            "RRM policy ratio list",
            ratioList,
            result.errorMessage))
    {
        return result;
    }

    const int ratioGroupCount =
        ratioList->list_of_ranParameter.list.count;

    if (ratioGroupCount <= 0 ||
        ratioGroupCount > 255)
    {
        result.errorMessage =
            "RRM policy ratio list must contain from 1 to 255 groups";
        return result;
    }

    result.control.sliceQuotas.reserve(
        static_cast<std::size_t>(ratioGroupCount));

    for (int index = 0;
         index < ratioGroupCount;
         ++index)
    {
        const RANParameter_STRUCTURE_t* ratioGroup =
            ratioList->list_of_ranParameter.list.array[index];

        RcV5SliceQuota quota;

        if (!DecodeSliceQuota(
                ratioGroup,
                quota,
                result.errorMessage))
        {
            result.errorMessage =
                "Invalid ratio group " +
                std::to_string(index + 1) +
                ": " +
                result.errorMessage;
            return result;
        }

        const auto duplicate =
            std::find_if(
                result.control.sliceQuotas.begin(),
                result.control.sliceQuotas.end(),
                [&quota](const RcV5SliceQuota& existing)
                {
                    return
                        existing.plmnIdentity ==
                            quota.plmnIdentity &&
                        existing.sst == quota.sst &&
                        existing.sd == quota.sd;
                });

        if (duplicate !=
            result.control.sliceQuotas.end())
        {
            result.errorMessage =
                "Control Message contains a duplicated slice identity";
            return result;
        }

        result.control.sliceQuotas.push_back(
            std::move(quota));
    }

    result.success = true;
    return result;
}

} // namespace ns3
