#!/usr/bin/env python3
"""Check safety-critical CubeMX settings against generated timer/IRQ code."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(path: str, needle: str) -> None:
    content = (ROOT / path).read_text(encoding="utf-8")
    if needle not in content:
        raise AssertionError(f"{path}: missing {needle!r}")


def main() -> int:
    require("F407_V2.0.ioc", "TIM1.AutomaticOutput=TIM_AUTOMATICOUTPUT_DISABLE")
    require("F407_V2.0.ioc", "NVIC.TIM1_BRK_TIM9_IRQn=true\\:5\\:0")
    require("Core/Src/tim.c", "sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;")
    require("Core/Src/tim.c", "HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);")
    require("Core/Inc/stm32f4xx_it.h", "void TIM1_BRK_TIM9_IRQHandler(void);")
    require("Core/Src/stm32f4xx_it.c", "MotorDriver_OnTim1BreakFromIsr();")
    print("CubeMX safety configuration is consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
