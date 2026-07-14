#ifndef RESET_TRACE_H
#define RESET_TRACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        RESET_TRACE_KIND_NONE          = 0,
        RESET_TRACE_KIND_NMI           = 1,
        RESET_TRACE_KIND_HARDFAULT     = 2,
        RESET_TRACE_KIND_MEMMANAGE     = 3,
        RESET_TRACE_KIND_BUSFAULT      = 4,
        RESET_TRACE_KIND_USAGEFAULT    = 5,
        RESET_TRACE_KIND_ERROR_HANDLER = 6,
        RESET_TRACE_KIND_FREERTOS      = 7,
        RESET_TRACE_KIND_DMA_GUARD     = 8
    } reset_trace_kind_t;

    typedef enum
    {
        RESET_TRACE_TASK_NONE   = 0,
        RESET_TRACE_TASK_SAFETY = 1,
        RESET_TRACE_TASK_MOTOR  = 2,
        RESET_TRACE_TASK_PS2    = 3,
        RESET_TRACE_TASK_ESP    = 4,
        RESET_TRACE_TASK_DEBUG  = 5
    } reset_trace_task_t;

    typedef struct
    {
        uint32_t magic;
        uint32_t version;
        uint32_t sequence;
        uint32_t kind;
        uint32_t reason;
        uint32_t line;
        uint32_t task;
        uint32_t cfsr;
        uint32_t hfsr;
        uint32_t bfar;
        uint32_t mmfar;
        uint32_t heartbeat_safety;
        uint32_t heartbeat_motor;
        uint32_t heartbeat_ps2;
        uint32_t heartbeat_esp;
        uint32_t heartbeat_debug;
        uint32_t source;
        uint32_t estop;
        uint32_t fault;
        uint32_t exc_return;
        uint32_t stack_ptr;
        uint32_t msp;
        uint32_t psp;
        uint32_t control;
        uint32_t fpccr;
        uint32_t stacked_lr;
        uint32_t stacked_pc;
        uint32_t stacked_xpsr;
        uint32_t dma2_lisr;
        uint32_t dma2_stream0_cr;
        uint32_t dma2_stream0_ndtr;
        uint32_t dma2_stream0_fcr;
        uint32_t adc1_sr;
        uint32_t adc1_cr2;
        uint32_t detail0;
        uint32_t detail1;
        uint32_t detail2;
        uint32_t detail3;
        uint32_t checksum;
    } reset_trace_record_t;

    void     PlatformResetTrace_TaskHeartbeat(reset_trace_task_t task, uint32_t tick_ms);
    uint32_t PlatformResetTrace_GetTaskHeartbeat(reset_trace_task_t task);
    void     PlatformResetTrace_UpdateControl(uint8_t source, uint8_t estop, uint8_t fault);
    void     PlatformResetTrace_Capture(reset_trace_kind_t kind, uint32_t reason, uint32_t line);
    void     PlatformResetTrace_CaptureWithTask(reset_trace_kind_t kind,
                                                uint32_t           reason,
                                                uint32_t           line,
                                                reset_trace_task_t task);
    void     PlatformResetTrace_CaptureWithDetails(reset_trace_kind_t kind,
                                                   uint32_t           reason,
                                                   uint32_t           line,
                                                   reset_trace_task_t task,
                                                   uint32_t           detail0,
                                                   uint32_t           detail1,
                                                   uint32_t           detail2,
                                                   uint32_t           detail3);
    void     PlatformResetTrace_CaptureFaultStack(reset_trace_kind_t kind, const uint32_t *stack, uint32_t exc_return);
    uint8_t  PlatformResetTrace_GetBootRecord(reset_trace_record_t *record);

#ifdef __cplusplus
}
#endif

#endif
