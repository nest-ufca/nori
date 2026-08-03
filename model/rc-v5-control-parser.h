#ifndef NORI_RC_V5_CONTROL_PARSER_H
#define NORI_RC_V5_CONTROL_PARSER_H

#include "rc-v5-codec.h"

#include "E2AP-PDU.h"

namespace ns3
{

/**
 * Extract and decode the RC payloads carried by one E2AP Control Request.
 *
 * Legacy E2AP types locate the opaque byte strings, whose contents are
 * decoded only by the isolated E2SM-RC v5 codec.
 */
RcV5DecodeResult
DecodeRcV5ControlRequest(
    const E2AP_PDU_t* requestPdu);

} // namespace ns3

#endif // NORI_RC_V5_CONTROL_PARSER_H
