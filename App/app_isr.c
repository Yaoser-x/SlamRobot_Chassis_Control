#include "app_isr.h"

#include "imu_service.h"
#include "main.h"
#include "platform_task_event.h"

void AppIsr_OnGpioExti(uint16_t pin)
{
    if (pin == IMU_INT1_Pin)
    {
        ImuService_OnDataReadyFromIsr();
        PlatformTaskEvent_SetFromIsr(PLATFORM_TASK_EVENT_IMU_DRDY);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    AppIsr_OnGpioExti(pin);
}
