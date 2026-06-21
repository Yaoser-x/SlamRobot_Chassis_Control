#include "reset_trace.h"

#include "stm32f4xx.h"

#define RESET_TRACE_MAGIC   0x52545243UL
#define RESET_TRACE_VERSION 4UL
#define RESET_TRACE_BASIC_FRAME_WORDS 8U
#define RESET_TRACE_FP_FRAME_WORDS    18U

_Static_assert((sizeof(reset_trace_record_t) % sizeof(uint32_t)) == 0,
               "reset trace record must stay word aligned");

static volatile reset_trace_record_t reset_trace_live __attribute__((section(".noinit.reset_trace")));
static reset_trace_record_t reset_trace_boot;
static uint8_t reset_trace_boot_captured;

static uint8_t ResetTrace_StackFrameReadable(const uint32_t *stack)
{
  uintptr_t start = (uintptr_t)stack;
  uintptr_t end = start + (RESET_TRACE_BASIC_FRAME_WORDS * sizeof(uint32_t));
  uint8_t in_sram = (start >= 0x20000000UL && end <= 0x20020000UL) ? 1U : 0U;
  uint8_t in_ccm = (start >= 0x10000000UL && end <= 0x10010000UL) ? 1U : 0U;

  return (in_sram != 0U || in_ccm != 0U) ? 1U : 0U;
}

static uint32_t ResetTrace_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  return primask;
}

static void ResetTrace_ExitCritical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static reset_trace_task_t ResetTrace_NormalizeTask(reset_trace_task_t task)
{
  return ((uint32_t)task <= (uint32_t)RESET_TRACE_TASK_DEBUG) ? task : RESET_TRACE_TASK_NONE;
}

static uint32_t ResetTrace_CalcChecksum(const reset_trace_record_t *record)
{
  const uint32_t *words = (const uint32_t *)record;
  uint32_t checksum = 0xA5A55A5AUL;
  uint32_t count = (uint32_t)(sizeof(*record) / sizeof(uint32_t)) - 1U;

  for (uint32_t i = 0U; i < count; ++i)
  {
    checksum ^= words[i] + 0x9E3779B9UL + (checksum << 6) + (checksum >> 2);
  }
  return checksum;
}

static uint8_t ResetTrace_RecordValid(const reset_trace_record_t *record)
{
  if (record->magic != RESET_TRACE_MAGIC || record->version != RESET_TRACE_VERSION)
  {
    return 0U;
  }
  return (record->checksum == ResetTrace_CalcChecksum(record)) ? 1U : 0U;
}

static void ResetTrace_PrepareLive(void)
{
  if (reset_trace_live.magic != RESET_TRACE_MAGIC ||
      reset_trace_live.version != RESET_TRACE_VERSION)
  {
    reset_trace_live = (reset_trace_record_t){0};
    reset_trace_live.magic = RESET_TRACE_MAGIC;
    reset_trace_live.version = RESET_TRACE_VERSION;
  }
}

static void ResetTrace_UpdateChecksum(void)
{
  reset_trace_live.checksum = ResetTrace_CalcChecksum((const reset_trace_record_t *)&reset_trace_live);
}

static void ResetTrace_ClearLive(void)
{
  reset_trace_live = (reset_trace_record_t){0};
}

void ResetTrace_TaskHeartbeat(reset_trace_task_t task, uint32_t tick_ms)
{
  uint32_t primask = ResetTrace_EnterCritical();
  task = ResetTrace_NormalizeTask(task);
  ResetTrace_PrepareLive();
  reset_trace_live.task = (uint32_t)task;
  reset_trace_live.kind = RESET_TRACE_KIND_NONE;

  switch (task)
  {
    case RESET_TRACE_TASK_SAFETY:
      reset_trace_live.heartbeat_safety = tick_ms;
      break;
    case RESET_TRACE_TASK_MOTOR:
      reset_trace_live.heartbeat_motor = tick_ms;
      break;
    case RESET_TRACE_TASK_PS2:
      reset_trace_live.heartbeat_ps2 = tick_ms;
      break;
    case RESET_TRACE_TASK_ESP:
      reset_trace_live.heartbeat_esp = tick_ms;
      break;
    case RESET_TRACE_TASK_DEBUG:
      reset_trace_live.heartbeat_debug = tick_ms;
      break;
    default:
      break;
  }
  ResetTrace_UpdateChecksum();
  ResetTrace_ExitCritical(primask);
}

void ResetTrace_UpdateControl(uint8_t source, uint8_t estop, uint8_t fault)
{
  uint32_t primask = ResetTrace_EnterCritical();

  ResetTrace_PrepareLive();
  reset_trace_live.source = source;
  reset_trace_live.estop = estop;
  reset_trace_live.fault = fault;
  ResetTrace_UpdateChecksum();
  ResetTrace_ExitCritical(primask);
}

void ResetTrace_Capture(reset_trace_kind_t kind, uint32_t reason, uint32_t line)
{
  reset_trace_task_t task = RESET_TRACE_TASK_NONE;

  if (reset_trace_live.magic == RESET_TRACE_MAGIC &&
      reset_trace_live.version == RESET_TRACE_VERSION)
  {
    task = ResetTrace_NormalizeTask((reset_trace_task_t)reset_trace_live.task);
  }
  ResetTrace_CaptureWithTask(kind, reason, line, task);
}

void ResetTrace_CaptureWithTask(reset_trace_kind_t kind,
                                uint32_t reason,
                                uint32_t line,
                                reset_trace_task_t task)
{
  uint32_t primask = ResetTrace_EnterCritical();

  ResetTrace_PrepareLive();
  reset_trace_live.sequence++;
  reset_trace_live.kind = (uint32_t)kind;
  reset_trace_live.reason = reason;
  reset_trace_live.line = line;
  reset_trace_live.task = (uint32_t)ResetTrace_NormalizeTask(task);
  reset_trace_live.cfsr = SCB->CFSR;
  reset_trace_live.hfsr = SCB->HFSR;
  reset_trace_live.bfar = SCB->BFAR;
  reset_trace_live.mmfar = SCB->MMFAR;
  reset_trace_live.detail0 = 0U;
  reset_trace_live.detail1 = 0U;
  reset_trace_live.detail2 = 0U;
  reset_trace_live.detail3 = 0U;
  ResetTrace_UpdateChecksum();
  __DSB();
  __ISB();
  ResetTrace_ExitCritical(primask);
}

void ResetTrace_CaptureWithDetails(reset_trace_kind_t kind,
                                   uint32_t reason,
                                   uint32_t line,
                                   reset_trace_task_t task,
                                   uint32_t detail0,
                                   uint32_t detail1,
                                   uint32_t detail2,
                                   uint32_t detail3)
{
  uint32_t primask = ResetTrace_EnterCritical();

  ResetTrace_PrepareLive();
  reset_trace_live.sequence++;
  reset_trace_live.kind = (uint32_t)kind;
  reset_trace_live.reason = reason;
  reset_trace_live.line = line;
  reset_trace_live.task = (uint32_t)ResetTrace_NormalizeTask(task);
  reset_trace_live.cfsr = SCB->CFSR;
  reset_trace_live.hfsr = SCB->HFSR;
  reset_trace_live.bfar = ((SCB->CFSR & SCB_CFSR_BFARVALID_Msk) != 0UL) ? SCB->BFAR : 0U;
  reset_trace_live.mmfar = ((SCB->CFSR & SCB_CFSR_MMARVALID_Msk) != 0UL) ? SCB->MMFAR : 0U;
  reset_trace_live.detail0 = detail0;
  reset_trace_live.detail1 = detail1;
  reset_trace_live.detail2 = detail2;
  reset_trace_live.detail3 = detail3;
  ResetTrace_UpdateChecksum();
  __DSB();
  __ISB();
  ResetTrace_ExitCritical(primask);
}

void ResetTrace_CaptureFaultStack(reset_trace_kind_t kind,
                                  const uint32_t *stack,
                                  uint32_t exc_return)
{
  uint32_t primask = ResetTrace_EnterCritical();
  reset_trace_task_t task = RESET_TRACE_TASK_NONE;
  const uint32_t *basic_frame = stack;

  if (reset_trace_live.magic == RESET_TRACE_MAGIC &&
      reset_trace_live.version == RESET_TRACE_VERSION)
  {
    task = ResetTrace_NormalizeTask((reset_trace_task_t)reset_trace_live.task);
  }

  ResetTrace_PrepareLive();
  reset_trace_live.sequence++;
  reset_trace_live.kind = (uint32_t)kind;
  reset_trace_live.reason = 0U;
  reset_trace_live.line = 0U;
  reset_trace_live.task = (uint32_t)task;
  reset_trace_live.cfsr = SCB->CFSR;
  reset_trace_live.hfsr = SCB->HFSR;
  reset_trace_live.bfar = ((SCB->CFSR & SCB_CFSR_BFARVALID_Msk) != 0UL) ? SCB->BFAR : 0U;
  reset_trace_live.mmfar = ((SCB->CFSR & SCB_CFSR_MMARVALID_Msk) != 0UL) ? SCB->MMFAR : 0U;
  reset_trace_live.exc_return = exc_return;
  reset_trace_live.stack_ptr = (uint32_t)stack;
  reset_trace_live.msp = __get_MSP();
  reset_trace_live.psp = __get_PSP();
  reset_trace_live.control = __get_CONTROL();
  reset_trace_live.fpccr = FPU->FPCCR;
  reset_trace_live.dma2_lisr = DMA2->LISR;
  reset_trace_live.dma2_stream0_cr = DMA2_Stream0->CR;
  reset_trace_live.dma2_stream0_ndtr = DMA2_Stream0->NDTR;
  reset_trace_live.dma2_stream0_fcr = DMA2_Stream0->FCR;
  reset_trace_live.adc1_sr = ADC1->SR;
  reset_trace_live.adc1_cr2 = ADC1->CR2;
  reset_trace_live.detail0 = 0U;
  reset_trace_live.detail1 = 0U;
  reset_trace_live.detail2 = 0U;
  reset_trace_live.detail3 = 0U;
  if ((exc_return & (1UL << 4U)) == 0U && stack != 0)
  {
    basic_frame = &stack[RESET_TRACE_FP_FRAME_WORDS];
  }
  if (ResetTrace_StackFrameReadable(basic_frame) != 0U)
  {
    reset_trace_live.stacked_lr = basic_frame[5];
    reset_trace_live.stacked_pc = basic_frame[6];
    reset_trace_live.stacked_xpsr = basic_frame[7];
  }
  else
  {
    reset_trace_live.stacked_lr = 0U;
    reset_trace_live.stacked_pc = 0U;
    reset_trace_live.stacked_xpsr = 0U;
  }
  ResetTrace_UpdateChecksum();
  __DSB();
  __ISB();
  ResetTrace_ExitCritical(primask);
}

uint8_t ResetTrace_GetBootRecord(reset_trace_record_t *record)
{
  uint8_t captured;
  uint32_t primask = ResetTrace_EnterCritical();

  if (reset_trace_boot_captured == 0U)
  {
    reset_trace_boot = reset_trace_live;
    reset_trace_boot_captured = 1U;
    ResetTrace_ClearLive();
  }

  captured = ResetTrace_RecordValid(&reset_trace_boot);
  if (record != 0)
  {
    if (captured != 0U)
    {
      *record = reset_trace_boot;
    }
    else
    {
      *record = (reset_trace_record_t){0};
    }
  }

  ResetTrace_ExitCritical(primask);
  return (captured != 0U && reset_trace_boot.kind != RESET_TRACE_KIND_NONE) ? 1U : 0U;
}
