#include "rc-v5-control-parser.h"

#include "InitiatingMessage.h"
#include "ProtocolIE-Field.h"
#include "RICcontrolRequest.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace ns3
{
namespace
{

constexpr long RC_RAN_FUNCTION_ID = 300;

struct RcV5ControlPayloadView
{
    const uint8_t* controlHeaderData{nullptr};
    std::size_t controlHeaderSize{0};

    const uint8_t* controlMessageData{nullptr};
    std::size_t controlMessageSize{0};
};

RcV5DecodeResult
MakeControlPayloadError(
    const std::string& message)
{
    RcV5DecodeResult result;
    result.errorMessage = message;
    return result;
}

bool
ExtractRcV5ControlPayload(
    const E2AP_PDU_t* requestPdu,
    RcV5ControlPayloadView& payload,
    std::string& errorMessage)
{
    if (requestPdu == nullptr ||
        requestPdu->present != E2AP_PDU_PR_initiatingMessage ||
        requestPdu->choice.initiatingMessage == nullptr)
    {
        errorMessage = "E2AP Control Request PDU is invalid";
        return false;
    }

    if (requestPdu->choice.initiatingMessage->value.present !=
        InitiatingMessage__value_PR_RICcontrolRequest)
    {
        errorMessage = "E2AP PDU does not contain a Control Request";
        return false;
    }

    const RICcontrolRequest_t& request =
        requestPdu->choice.initiatingMessage
            ->value.choice.RICcontrolRequest;

    if (request.protocolIEs.list.count <= 0 ||
        request.protocolIEs.list.array == nullptr)
    {
        errorMessage =
            "E2AP Control Request contains no information elements";
        return false;
    }

    bool ranFunctionFound = false;
    long ranFunctionId = 0;

    for (int index = 0;
         index < request.protocolIEs.list.count;
         ++index)
    {
        const auto* informationElement =
            reinterpret_cast<
                const RICcontrolRequest_IEs_t*>(
                request.protocolIEs.list.array[index]);

        if (informationElement == nullptr)
        {
            errorMessage =
                "E2AP Control Request contains a null information element";
            return false;
        }

        switch (informationElement->value.present)
        {
        case RICcontrolRequest_IEs__value_PR_RANfunctionID:
            if (ranFunctionFound)
            {
                errorMessage =
                    "E2AP Control Request contains duplicated RAN Function ID";
                return false;
            }

            ranFunctionId =
                informationElement->value.choice.RANfunctionID;

            ranFunctionFound = true;
            break;

        case RICcontrolRequest_IEs__value_PR_RICcontrolHeader:
        {
            if (payload.controlHeaderData != nullptr)
            {
                errorMessage =
                    "E2AP Control Request contains duplicated Control Header";
                return false;
            }

            const RICcontrolHeader_t& header =
                informationElement->value.choice.RICcontrolHeader;

            if (header.buf == nullptr || header.size == 0)
            {
                errorMessage =
                    "E2AP Control Request contains an empty Control Header";
                return false;
            }

            payload.controlHeaderData = header.buf;
            payload.controlHeaderSize = header.size;
            break;
        }

        case RICcontrolRequest_IEs__value_PR_RICcontrolMessage:
        {
            if (payload.controlMessageData != nullptr)
            {
                errorMessage =
                    "E2AP Control Request contains duplicated Control Message";
                return false;
            }

            const RICcontrolMessage_t& message =
                informationElement->value.choice.RICcontrolMessage;

            if (message.buf == nullptr || message.size == 0)
            {
                errorMessage =
                    "E2AP Control Request contains an empty Control Message";
                return false;
            }

            payload.controlMessageData = message.buf;
            payload.controlMessageSize = message.size;
            break;
        }

        default:
            break;
        }
    }

    if (!ranFunctionFound)
    {
        errorMessage =
            "E2AP Control Request contains no RAN Function ID";
        return false;
    }

    if (ranFunctionId != RC_RAN_FUNCTION_ID)
    {
        errorMessage =
            "E2AP Control Request does not target RAN Function 300";
        return false;
    }

    if (payload.controlHeaderData == nullptr)
    {
        errorMessage =
            "E2AP Control Request contains no Control Header";
        return false;
    }

    if (payload.controlMessageData == nullptr)
    {
        errorMessage =
            "E2AP Control Request contains no Control Message";
        return false;
    }

    return true;
}

} // namespace

RcV5DecodeResult
DecodeRcV5ControlRequest(
    const E2AP_PDU_t* requestPdu)
{
    RcV5ControlPayloadView payload;
    std::string extractionError;

    if (!ExtractRcV5ControlPayload(
            requestPdu,
            payload,
            extractionError))
    {
        return MakeControlPayloadError(extractionError);
    }

    return DecodeRcV5Control(
        payload.controlHeaderData,
        payload.controlHeaderSize,
        payload.controlMessageData,
        payload.controlMessageSize);
}

} // namespace ns3
