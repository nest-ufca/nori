#ifndef NORI_KPM_V3_CODEC_H
#define NORI_KPM_V3_CODEC_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * Export the public codec wrapper while generated ASN.1 symbols remain
 * hidden inside the isolated shared library.
 */
#if defined(__GNUC__) || defined(__clang__)
#define NORI_KPM_V3_CODEC_API __attribute__((visibility("default")))
#else
#define NORI_KPM_V3_CODEC_API
#endif

namespace ns3
{

/**
 * One measurement requested through a KPM Action Definition.
 */
struct KpmV3MeasurementRequest
{
    std::string name;
    bool noLabel{false};
};

/**
 * One gNB-DU UE explicitly selected by a KPM Style 5 subscription.
 *
 * The value represents the standard gNB-CU UE F1AP ID carried inside a
 * UEID-GNB-DU identity.
 */
struct KpmV3GnbDuUeRequest
{
    uint64_t gnbCuUeF1apId{0};
};

/**
 * Decoded KPM subscription parameters supported by NORI.
 *
 * E2AP identifiers such as requestor ID, instance ID and action ID are not
 * included here because they belong to the E2AP subscription state rather
 * than to the E2SM-KPM payload.
 */
struct KpmV3SubscriptionRequest
{
    uint32_t reportingPeriodMs{0};
    uint32_t reportStyle{0};
    uint32_t granularityPeriodMs{0};

    std::vector<KpmV3MeasurementRequest> measurements;

    // Empty for cell-level Style 1 subscriptions and populated for the
    // explicitly selected gNB-DU UEs of Style 5.
    std::vector<KpmV3GnbDuUeRequest> matchingUes;
};

/**
 * Result returned by the KPM decoder.
 *
 * Decoding errors are returned to the caller instead of terminating the
 * simulator, allowing the E2 subscription path to reject invalid requests.
 */
struct KpmV3DecodeResult
{
    bool success{false};
    KpmV3SubscriptionRequest subscription;
    std::string errorMessage;
};

/**
 * Decode the Event Trigger Definition and Action Definition of one KPM
 * subscription request.
 *
 * The function exposes only plain C++ types. ASN.1-generated types remain
 * private to the isolated KPM v3 codec library, avoiding symbol collisions
 * with the legacy KPM implementation linked through E2Sim.
 *
 * \param eventTriggerData Encoded Event Trigger Definition bytes.
 * \param eventTriggerSize Number of bytes in the Event Trigger Definition.
 * \param actionDefinitionData Encoded Action Definition bytes.
 * \param actionDefinitionSize Number of bytes in the Action Definition.
 * \return The decoded subscription or a descriptive decoding error.
 */
NORI_KPM_V3_CODEC_API
KpmV3DecodeResult
DecodeKpmV3Subscription(
    const uint8_t* eventTriggerData,
    std::size_t eventTriggerSize,
    const uint8_t* actionDefinitionData,
    std::size_t actionDefinitionSize);

} // namespace ns3

#undef NORI_KPM_V3_CODEC_API

#endif // NORI_KPM_V3_CODEC_H
