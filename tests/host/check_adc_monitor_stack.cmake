file(GLOB_RECURSE stack_usage_files
    "${STACK_USAGE_DIR}/*adc_monitor*.su"
)

if(NOT stack_usage_files)
    message(FATAL_ERROR "No adc_monitor stack-usage report found")
endif()

set(adc_update_stack_bytes "")
foreach(stack_usage_file IN LISTS stack_usage_files)
    file(STRINGS "${stack_usage_file}" stack_usage_lines)
    foreach(stack_usage_line IN LISTS stack_usage_lines)
        if(stack_usage_line MATCHES "AdcMonitor_Update[ \t]+([0-9]+)[ \t]+")
            set(adc_update_stack_bytes "${CMAKE_MATCH_1}")
        endif()
    endforeach()
endforeach()

if(adc_update_stack_bytes STREQUAL "")
    message(FATAL_ERROR "AdcMonitor_Update stack usage was not reported")
endif()

if(adc_update_stack_bytes GREATER MAX_STACK_BYTES)
    message(FATAL_ERROR
        "AdcMonitor_Update uses ${adc_update_stack_bytes} bytes; budget is ${MAX_STACK_BYTES} bytes"
    )
endif()

message(STATUS
    "AdcMonitor_Update stack usage: ${adc_update_stack_bytes}/${MAX_STACK_BYTES} bytes"
)
