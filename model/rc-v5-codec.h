#ifndef NORI_RC_V5_CODEC_H
#define NORI_RC_V5_CODEC_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#if defined(__GNUC__) || defined(__clang__)
#define NORI_RC_V5_CODEC_API __attribute__((visibility("default")))
#else
#define NORI_RC_V5_CODEC_API
#endif

namespace ns3
{

/**
 * One slice quota decoded from an E2SM-RC Style 2, Action 6 command.
 *
 * PLMN and SD retain their three-byte ASN.1 representation. SD is optional
 * because the validated NEST scenario currently identifies slices by SST.
 */
struct RcV5SliceQuota
{
    std::array<uint8_t, 3> plmnIdentity{};
    uint8_t sst{0};
    std::optional<std::array<uint8_t, 3>> sd;

    uint32_t minPrbRatio{0};
    uint32_t maxPrbRatio{0};
    uint32_t dedicatedPrbRatio{0};
};

/**
 * Plain C++ representation of the supported RC control request.
 */
struct RcV5ControlRequest
{
    uint64_t gnbCuUeF1apId{0};
    uint32_t controlStyle{0};
    uint32_t controlAction{0};

    std::vector<RcV5SliceQuota> sliceQuotas;
};

/**
 * Result returned by the isolated E2SM-RC v5 decoder.
 */
struct RcV5DecodeResult
{
    bool success{false};
    RcV5ControlRequest control;
    std::string errorMessage;
};

/**
 * Encoded E2SM-RC v5 RAN Function Definition.
 */
struct RcV5FunctionDescriptionResult
{
    bool success{false};
    std::vector<uint8_t> encodedDefinition;
    std::string errorMessage;
};

/**
 * Build the E2SM-RC v5 RAN Function Definition advertised by NORI.
 *
 * The definition advertises Control Style 2, Action 6, Control Header
 * Format 1 and Control Message Format 1.
 */
NORI_RC_V5_CODEC_API RcV5FunctionDescriptionResult EncodeRcV5FunctionDescription();

/**
 * Decode the E2SM-RC Control Header and Control Message supported by NORI.
 *
 * The initial contract accepts Control Header Format 1 and Control Message
 * Format 1 for Style 2, Action 6. All generated ASN.1 types remain private
 * to the isolated codec library.
 */
NORI_RC_V5_CODEC_API RcV5DecodeResult DecodeRcV5Control(
    const uint8_t* controlHeaderData,
    std::size_t controlHeaderSize,
    const uint8_t* controlMessageData,
    std::size_t controlMessageSize);

} // namespace ns3

#undef NORI_RC_V5_CODEC_API

#endif // NORI_RC_V5_CODEC_H
