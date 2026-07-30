#include "kpm-subscription-parser.h"

#include "InitiatingMessage.h"
#include "ProtocolIE-Field.h"
#include "ProtocolIE-SingleContainer.h"
#include "RICactionType.h"
#include "RICsubscriptionRequest.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace ns3
{
namespace
{

/**
 * Non-owning view of the encoded E2SM payloads carried by an E2AP request.
 *
 * The pointers remain valid only while the original E2AP PDU is alive.
 */
struct KpmSubscriptionPayloadView
{
    const uint8_t* eventTriggerData{nullptr};
    std::size_t eventTriggerSize{0};

    const uint8_t* actionDefinitionData{nullptr};
    std::size_t actionDefinitionSize{0};
};

/**
 * Build a failed result for malformed E2AP subscription structures.
 */
KpmV3DecodeResult
MakeSubscriptionPayloadError(
    const std::string& message)
{
    KpmV3DecodeResult result;
    result.errorMessage = message;
    return result;
}

/**
 * Locate the Event Trigger and first REPORT Action Definition in a decoded
 * E2AP subscription request.
 *
 * E2Sim represents open-type lists with generic pointers, so each list item
 * is cast to the concrete type selected by the corresponding ASN.1 CHOICE.
 */
bool
ExtractKpmSubscriptionPayload(
    const E2AP_PDU_t* requestPdu,
    KpmSubscriptionPayloadView& payload,
    std::string& errorMessage)
{
    if (requestPdu == nullptr ||
        requestPdu->present !=
            E2AP_PDU_PR_initiatingMessage ||
        requestPdu->choice.initiatingMessage == nullptr)
    {
        errorMessage =
            "E2AP subscription request PDU is invalid";
        return false;
    }

    if (requestPdu->choice.initiatingMessage
            ->value.present !=
        InitiatingMessage__value_PR_RICsubscriptionRequest)
    {
        errorMessage =
            "E2AP PDU does not contain a subscription request";
        return false;
    }

    const RICsubscriptionRequest_t& request =
        requestPdu->choice.initiatingMessage
            ->value.choice.RICsubscriptionRequest;

    if (request.protocolIEs.list.count <= 0 ||
        request.protocolIEs.list.array == nullptr)
    {
        errorMessage =
            "E2AP subscription request contains no information elements";
        return false;
    }

    const RICsubscriptionDetails_t*
        subscriptionDetails = nullptr;

    for (int index = 0;
         index < request.protocolIEs.list.count;
         ++index)
    {
        auto* informationElement =
            reinterpret_cast<
                RICsubscriptionRequest_IEs_t*>(
                request.protocolIEs.list.array[index]);

        if (informationElement == nullptr ||
            informationElement->value.present !=
                RICsubscriptionRequest_IEs__value_PR_RICsubscriptionDetails)
        {
            continue;
        }

        subscriptionDetails =
            &informationElement->value.choice
                 .RICsubscriptionDetails;
        break;
    }

    if (subscriptionDetails == nullptr)
    {
        errorMessage =
            "E2AP subscription details were not found";
        return false;
    }

    const RICeventTriggerDefinition_t& eventTrigger =
        subscriptionDetails->ricEventTriggerDefinition;

    if (eventTrigger.buf == nullptr ||
        eventTrigger.size == 0)
    {
        errorMessage =
            "E2AP subscription contains an empty Event Trigger";
        return false;
    }

    payload.eventTriggerData = eventTrigger.buf;
    payload.eventTriggerSize = eventTrigger.size;

    const RICactions_ToBeSetup_List_t& actionList =
        subscriptionDetails->ricAction_ToBeSetup_List;

    if (actionList.list.count <= 0 ||
        actionList.list.array == nullptr)
    {
        errorMessage =
            "E2AP subscription contains no actions";
        return false;
    }

    for (int index = 0;
         index < actionList.list.count;
         ++index)
    {
        auto* actionInformationElement =
            reinterpret_cast<
                RICaction_ToBeSetup_ItemIEs_t*>(
                actionList.list.array[index]);

        if (actionInformationElement == nullptr ||
            actionInformationElement->value.present !=
                RICaction_ToBeSetup_ItemIEs__value_PR_RICaction_ToBeSetup_Item)
        {
            continue;
        }

        const RICaction_ToBeSetup_Item_t& action =
            actionInformationElement->value.choice
                .RICaction_ToBeSetup_Item;

        if (action.ricActionType !=
            RICactionType_report)
        {
            continue;
        }

        if (action.ricActionDefinition == nullptr ||
            action.ricActionDefinition->buf == nullptr ||
            action.ricActionDefinition->size == 0)
        {
            errorMessage =
                "KPM REPORT action contains no Action Definition";
            return false;
        }

        payload.actionDefinitionData =
            action.ricActionDefinition->buf;

        payload.actionDefinitionSize =
            action.ricActionDefinition->size;

        return true;
    }

    errorMessage =
        "E2AP subscription contains no KPM REPORT action";
    return false;
}

} // namespace

/**
 * Extract and decode the KPM payloads carried by one E2AP subscription.
 */
KpmV3DecodeResult
DecodeKpmV3SubscriptionRequest(
    const E2AP_PDU_t* requestPdu)
{
    KpmSubscriptionPayloadView payload;
    std::string extractionError;

    if (!ExtractKpmSubscriptionPayload(
            requestPdu,
            payload,
            extractionError))
    {
        return MakeSubscriptionPayloadError(
            extractionError);
    }

    return DecodeKpmV3Subscription(
        payload.eventTriggerData,
        payload.eventTriggerSize,
        payload.actionDefinitionData,
        payload.actionDefinitionSize);
}

} // namespace ns3
