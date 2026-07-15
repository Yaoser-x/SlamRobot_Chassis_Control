#ifndef DEBUG_SYSTEM_STATUS_H
#define DEBUG_SYSTEM_STATUS_H

/** Print the complete debug system status report. */
void DebugSystemStatus_Print(void);
/** Print the captured MCU reset-cause summary. */
void DebugSystemStatus_PrintResetFlags(void);
/** Print the persisted reset trace record. */
void DebugSystemStatus_PrintResetTrace(void);

#endif
