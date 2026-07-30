#ifndef NORI_KPM_SUBSCRIPTION_PARSER_H
#define NORI_KPM_SUBSCRIPTION_PARSER_H

#include "kpm-v3-codec.h"

#include "E2AP-PDU.h"

namespace ns3
{

/**
 * Extract and decode the KPM payloads carried by one E2AP subscription.
 *
 * The legacy E2AP structures are used only to locate the opaque Event
 * Trigger and Action Definition byte strings. Their contents are decoded by
 * the isolated KPM v3 codec.
 */
KpmV3DecodeResult
DecodeKpmV3SubscriptionRequest(
    const E2AP_PDU_t* requestPdu);

} // namespace ns3

#endif // NORI_KPM_SUBSCRIPTION_PARSER_H