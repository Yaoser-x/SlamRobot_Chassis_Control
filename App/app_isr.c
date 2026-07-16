#include "app_isr.h"

#include "main.h"
#include "platform_task_event.h"
#include "state_estimation_service.h"

void AppIsr_OnGpioExti(uint16_t pin)
{
    if (pin == IMU_INT1_Pin)
    {
        StateEstimation_OnImuDataReadyFromIsr();
        PlatformTaskEvent_SetFromIsr(PLATFORM_TASK_EVENT_IMU_DRDY);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    AppIsr_OnGpioExti(pin);
}
