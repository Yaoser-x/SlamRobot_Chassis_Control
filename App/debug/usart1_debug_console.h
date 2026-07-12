#ifndef USART1_DEBUG_CONSOLE_H
#define USART1_DEBUG_CONSOLE_H

#ifdef __cplusplus
extern "C"
{
#endif

    void Usart1DebugConsole_Init(void);
    void Usart1DebugConsole_ClearRxBuffer(void);
    void Usart1DebugConsole_RestartRx(void);
    void Usart1DebugConsole_OnRxCplt(void);
    void Usart1DebugConsole_OnUartError(void);
    void Usart1DebugConsole_RevokeMaintenanceAuthorization(void);
    void Task_Usart1DebugConsole(void *argument);

#ifdef __cplusplus
}
#endif

#endif
