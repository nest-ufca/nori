set(base_examples
    nori-sample
    #oran-interface-example
    nori-mimo-demo
    nori-simple-rl-sched
    nori-embb-urllc-scenario
)

foreach(
  example
  ${base_examples}
)

build_lib_example(
    NAME ${example}
    SOURCE_FILES ${example}.cc
    LIBRARIES_TO_LINK ${libnori}
                      ${libnr}
                      ${liblte}
  )
endforeach()

build_lib_example(
    NAME nest-embb-urllc-slicing
    SOURCE_FILES nest-embb-urllc-slicing.cc
                 nest/mimo-feedback-trace.cc
                 nest/tcp-transport-trace.cc
                 nest/mobility-trace.cc
                 nest/radio-link-trace.cc
                 nest/slice-metrics-collector.cc
                 nest/slice-controller.cc
                 nest/scenario-config.cc
    LIBRARIES_TO_LINK ${libnori}
                      ${libnr}
                      ${liblte}
)
