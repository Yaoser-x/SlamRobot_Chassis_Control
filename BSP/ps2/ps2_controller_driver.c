#include "ps2_controller_driver.h"

#include "main.h"

#define PS2_CLK_HALF_PERIOD_US 10U
#define PS2_BYTE_GAP_US        20U
#define PS2_FRAME_GAP_US       30U

static const uint8_t ps2_poll_frame[PS2_HW_FRAME_LEN] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

static uint8_t ps2_timing_fault;

/* ---------- DWT 微秒延时 ---------- */

static void Ps2ControllerDriver_DwtDelayInit(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    ps2_timing_fault = 0U;
}

static void Ps2ControllerDriver_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    uint32_t guard = ticks + (ticks / 2U) + 1000U;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U || ticks == 0U)
    {
        ps2_timing_fault = 1U;
        return;
    }

    while ((DWT->CYCCNT - start) < ticks)
    {
        if (guard == 0U)
        {
            ps2_timing_fault = 1U;
            return;
        }
        guard--;
    }
}

/* ---------- GPIO 位带 ---------- */

static void Ps2ControllerDriver_SetCmd(uint8_t high)
{
    HAL_GPIO_WritePin(PS2_DO_GPIO_Port, PS2_DO_Pin, (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Ps2ControllerDriver_SetClk(uint8_t high)
{
    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Ps2ControllerDriver_SetCs(uint8_t high)
{
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Ps2ControllerDriver_ReadDat(void)
{
    return (HAL_GPIO_ReadPin(PS2_DI_GPIO_Port, PS2_DI_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Ps2ControllerDriver_ConfigurePins(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin  = PS2_DI_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PS2_DI_GPIO_Port, &gpio);

    gpio.Pin   = PS2_DO_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_DO_GPIO_Port, &gpio);

    gpio.Pin = PS2_CLK_Pin;
    HAL_GPIO_Init(PS2_CLK_GPIO_Port, &gpio);

    gpio.Pin = PS2_CS_Pin;
    HAL_GPIO_Init(PS2_CS_GPIO_Port, &gpio);

    Ps2ControllerDriver_SetCmd(1U);
    Ps2ControllerDriver_SetClk(1U);
    Ps2ControllerDriver_SetCs(1U);
}

/* ---------- SPI 位带传输 ---------- */

static uint8_t Ps2ControllerDriver_TransferByte(uint8_t tx)
{
    uint8_t rx = 0U;

    for (uint8_t bit = 0x01U; bit != 0U; bit <<= 1U)
    {
        Ps2ControllerDriver_SetCmd(((tx & bit) != 0U) ? 1U : 0U);
        Ps2ControllerDriver_DelayUs(PS2_CLK_HALF_PERIOD_US);
        Ps2ControllerDriver_SetClk(0U);
        Ps2ControllerDriver_DelayUs(PS2_CLK_HALF_PERIOD_US);

        if (Ps2ControllerDriver_ReadDat() != 0U)
        {
            rx |= bit;
        }

        Ps2ControllerDriver_SetClk(1U);
        Ps2ControllerDriver_DelayUs(PS2_CLK_HALF_PERIOD_US);
    }

    Ps2ControllerDriver_SetCmd(1U);
    return rx;
}

static uint8_t Ps2ControllerDriver_Send(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    ps2_timing_fault = 0U;
    Ps2ControllerDriver_SetCs(0U);
    Ps2ControllerDriver_DelayUs(PS2_FRAME_GAP_US);

    for (uint8_t i = 0U; i < len; ++i)
    {
        uint8_t value = Ps2ControllerDriver_TransferByte(tx[i]);
        if (ps2_timing_fault != 0U)
        {
            break;
        }
        if (rx != 0)
        {
            rx[i] = value;
        }
        Ps2ControllerDriver_DelayUs(PS2_BYTE_GAP_US);
        if (ps2_timing_fault != 0U)
        {
            break;
        }
    }

    Ps2ControllerDriver_SetCs(1U);
    Ps2ControllerDriver_DelayUs(PS2_FRAME_GAP_US);
    return (ps2_timing_fault == 0U) ? 1U : 0U;
}

/* ---------- 手柄协议 ---------- */

static uint8_t Ps2ControllerDriver_HandshakeOk(const uint8_t *rx)
{
    return (rx[1] != 0xFFU && rx[2] == 0x5AU) ? 1U : 0U;
}

uint8_t Ps2ControllerDriver_IsAnalogMode(uint8_t mode)
{
    return (mode == 0x73U || mode == 0x79U) ? 1U : 0U;
}

static void Ps2ControllerDriver_ShortPoll(void)
{
    static const uint8_t poll[5] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U};
    (void)Ps2ControllerDriver_Send(poll, 0, (uint8_t)sizeof(poll));
}

static void Ps2ControllerDriver_ConfigAnalog(void)
{
    static const uint8_t enter_cfg[PS2_HW_FRAME_LEN]  = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t analog_cfg[PS2_HW_FRAME_LEN] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t vibration_cfg[5]             = {0x01U, 0x4DU, 0x00U, 0x00U, 0x01U};
    static const uint8_t exit_cfg[PS2_HW_FRAME_LEN]   = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};

    Ps2ControllerDriver_ShortPoll();
    Ps2ControllerDriver_ShortPoll();
    Ps2ControllerDriver_ShortPoll();
    HAL_Delay(2U);
    (void)Ps2ControllerDriver_Send(enter_cfg, 0, (uint8_t)sizeof(enter_cfg));
    HAL_Delay(2U);
    (void)Ps2ControllerDriver_Send(analog_cfg, 0, (uint8_t)sizeof(analog_cfg));
    HAL_Delay(2U);
    (void)Ps2ControllerDriver_Send(vibration_cfg, 0, (uint8_t)sizeof(vibration_cfg));
    HAL_Delay(2U);
    (void)Ps2ControllerDriver_Send(exit_cfg, 0, (uint8_t)sizeof(exit_cfg));
    HAL_Delay(2U);
}

static uint8_t Ps2ControllerDriver_Probe(uint8_t require_analog)
{
    uint8_t rx[PS2_HW_FRAME_LEN] = {0};

    for (uint8_t i = 0U; i < 6U; ++i)
    {
        if (Ps2ControllerDriver_Send(ps2_poll_frame, rx, (uint8_t)sizeof(rx)) != 0U
            && Ps2ControllerDriver_HandshakeOk(rx) != 0U
            && (require_analog == 0U || Ps2ControllerDriver_IsAnalogMode(rx[1]) != 0U))
        {
            return 1U;
        }
        HAL_Delay(2U);
    }
    return 0U;
}

#define PS2_HW_RETRY_LIMIT 3U

static uint8_t Ps2ControllerDriver_InitMapping(void)
{
    for (uint8_t i = 0U; i < PS2_HW_RETRY_LIMIT; ++i)
    {
        Ps2ControllerDriver_ConfigAnalog();
        if (Ps2ControllerDriver_Probe(1U) != 0U)
        {
            return 1U;
        }
    }
    return Ps2ControllerDriver_Probe(0U);
}

/* ---------- 公开 API ---------- */

void Ps2ControllerDriver_Init(void)
{
    Ps2ControllerDriver_DwtDelayInit();
    Ps2ControllerDriver_ConfigurePins();
    (void)Ps2ControllerDriver_InitMapping();
}

uint8_t Ps2ControllerDriver_ReadSample(ps2_controller_driver_sample_t *sample)
{
    uint8_t rx[PS2_HW_FRAME_LEN] = {0};

    if (sample == 0)
    {
        return 0U;
    }

    if (Ps2ControllerDriver_Send(ps2_poll_frame, rx, (uint8_t)sizeof(ps2_poll_frame)) == 0U
        || Ps2ControllerDriver_HandshakeOk(rx) == 0U)
    {
        return 0U;
    }

    sample->mode = rx[1];
    sample->btn1 = (uint8_t)~rx[3];
    sample->btn2 = (uint8_t)~rx[4];
    if (Ps2ControllerDriver_IsAnalogMode(sample->mode) != 0U)
    {
        sample->right_x = rx[5];
        sample->right_y = rx[6];
        sample->left_x  = rx[7];
        sample->left_y  = rx[8];
    }
    else
    {
        sample->right_x = 128U;
        sample->right_y = 128U;
        sample->left_x  = 128U;
        sample->left_y  = 128U;
    }

    return 1U;
}
