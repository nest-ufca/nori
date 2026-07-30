option(
    NORI_ENABLE_KPM_V3_CODEC
    "Build the isolated O-RAN SC KPM v3 codec"
    OFF
)

set(
    NORI_KPM_V3_ASN1_DIR
    "$ENV{NORI_KPM_V3_ASN1_DIR}"
    CACHE PATH
    "Directory containing generated O-RAN SC ASN.1 sources"
)

if(NORI_ENABLE_KPM_V3_CODEC)
    if(NOT IS_DIRECTORY "${NORI_KPM_V3_ASN1_DIR}")
        message(
            FATAL_ERROR
            "NORI_KPM_V3_ASN1_DIR must point to the generated "
            "O-RAN SC ASN.1 directory"
        )
    endif()

    file(
        GLOB
        NORI_KPM_V3_ASN1_SOURCES
        CONFIGURE_DEPENDS
        "${NORI_KPM_V3_ASN1_DIR}/*.c"
    )

    if(NOT NORI_KPM_V3_ASN1_SOURCES)
        message(
            FATAL_ERROR
            "No generated ASN.1 C sources were found in "
            "${NORI_KPM_V3_ASN1_DIR}"
        )
    endif()

    # Compile generated ASN.1 sources separately so their symbols and
    # compiler warnings do not leak into the handwritten NORI code.
    add_library(
        nori_kpm_v3_asn1
        STATIC
        ${NORI_KPM_V3_ASN1_SOURCES}
    )

    target_include_directories(
        nori_kpm_v3_asn1
        PRIVATE
        "${NORI_KPM_V3_ASN1_DIR}"
    )

    set_target_properties(
        nori_kpm_v3_asn1
        PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        C_VISIBILITY_PRESET hidden
    )

    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(
            nori_kpm_v3_asn1
            PRIVATE
            -w
        )
    endif()

    # Expose only the plain C++ decoder wrapper while keeping all generated
    # ASN.1 symbols hidden inside this shared library.
    add_library(
        nori_kpm_v3_codec
        SHARED
        model/kpm-v3-codec.cc
    )

    target_include_directories(
        nori_kpm_v3_codec
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/model"
        "${NORI_KPM_V3_ASN1_DIR}"
    )

    target_link_libraries(
        nori_kpm_v3_codec
        PRIVATE
        nori_kpm_v3_asn1
        m
    )

    set_target_properties(
        nori_kpm_v3_codec
        PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_link_options(
            nori_kpm_v3_codec
            PRIVATE
            "-Wl,--no-undefined"
        )
    endif()

        # Keep the optional codec dependencies in the same CMake export set as
    # libnori so installed ns-3 package metadata remains consistent.
    install(
        TARGETS
            nori_kpm_v3_asn1
            nori_kpm_v3_codec
        EXPORT
            ns3ExportTargets
        ARCHIVE DESTINATION
            ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION
            ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION
            ${CMAKE_INSTALL_BINDIR}
    )
endif()

set(nori_optional_libraries)

if(NORI_ENABLE_KPM_V3_CODEC)
    list(
        APPEND
        nori_optional_libraries
        nori_kpm_v3_codec
    )

    add_compile_definitions(
        NORI_ENABLE_KPM_V3_CODEC=1
    )
endif()

set(source_files
    model/E2-report.cc
    model/E2-interface.cc
    helper/E2-term-helper.cc
    helper/nori-slicing-helper.cc
    model/asn1c-types.cc
    model/function-description.cc
    model/kpm-function-description.cc
    model/kpm-indication.cc
    model/oran-interface.cc
    model/ric-control-function-description.cc
    model/ric-control-message.cc
    model/nr-mac-scheduler-ue-info-rl.cc
    model/nr-rl-mac-scheduler-ofdma.cc
    helper/indication-message-helper.cc
    helper/lte-indication-message-helper.cc
    helper/mmwave-indication-message-helper.cc
    )

if(NORI_ENABLE_KPM_V3_CODEC)
    list(
        APPEND
        source_files
        model/kpm-subscription-parser.cc
    )
endif()

include_directories(/usr/local/include/e2sim)
set(E2SIM_LIBRARIES "/usr/local/lib/libe2sim.a")

set(header_files
    model/E2-report.h
    model/E2-interface.h
    helper/E2-term-helper.h
    helper/nori-slicing-helper.h
    model/asn1c-types.h
    model/function-description.h
    model/kpm-function-description.h
    model/kpm-indication.h
    model/oran-interface.h
    model/ric-control-function-description.h
    model/ric-control-message.h
    model/nr-mac-scheduler-ue-info-rl.h
    model/nr-rl-mac-scheduler-ofdma.h
    helper/indication-message-helper.h
    helper/lte-indication-message-helper.h
    helper/mmwave-indication-message-helper.h
    )

    build_lib(
        LIBNAME nori
        SOURCE_FILES ${source_files}
        HEADER_FILES ${header_files}
        LIBRARIES_TO_LINK
          ${libnr}
          ${E2SIM_LIBRARIES}
          ${libcore}
          ${liblte}
          ${nori_optional_libraries}
      )
