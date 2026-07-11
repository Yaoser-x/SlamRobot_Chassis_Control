#!/usr/bin/env python3
"""Check safety-critical CubeMX settings against generated timer/IRQ code."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(path: str, needle: str) -> None:
    content = (ROOT / path).read_text(encoding="utf-8")
    if needle not in content:
        raise AssertionError(f"{path}: missing {needle!r}")


def function_body(path: str, function_name: str) -> str:
    content = (ROOT / path).read_text(encoding="utf-8")
    marker = f"void {function_name}(void)"
    start = content.find(marker)
    if start < 0:
        raise AssertionError(f"{path}: missing function {function_name}")
    next_function = content.find("\nvoid MX_", start + len(marker))
    return content[start:] if next_function < 0 else content[start:next_function]


def require_in_section(section: str, name: str, needle: str) -> None:
    if needle not in section:
        raise AssertionError(f"{name}: missing {needle!r}")


def main() -> int:
    require("CMakeLists.txt", "$<$<CONFIG:Release>:DEBUG_CONSOLE_RELEASE_REQUIRES_ARM=1>")
    require("App/debug/usart1_debug_console.c", "#ifndef DEBUG_CONSOLE_RELEASE_REQUIRES_ARM")
    require("App/debug/usart1_debug_console.c", "#define DEBUG_CONSOLE_RELEASE_REQUIRES_ARM 0U")
    require("F407_V2.0.ioc", "TIM1.AutomaticOutput=TIM_AUTOMATICOUTPUT_DISABLE")
    require("F407_V2.0.ioc", "NVIC.TIM1_BRK_TIM9_IRQn=true\\:5\\:0")
    tim1 = function_body("Core/Src/tim.c", "MX_TIM1_Init")
    tim8 = function_body("Core/Src/tim.c", "MX_TIM8_Init")
    require_in_section(tim1, "MX_TIM1_Init", "sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;")
    require_in_section(tim1, "MX_TIM1_Init", "sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;")
    require_in_section(tim1, "MX_TIM1_Init", "sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;")
    require_in_section(tim8, "MX_TIM8_Init", "sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;")
    require_in_section(tim8, "MX_TIM8_Init", "sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;")
    require("Core/Src/tim.c", "HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);")
    require("Core/Inc/stm32f4xx_it.h", "void TIM1_BRK_TIM9_IRQHandler(void);")
    require("Core/Src/stm32f4xx_it.c", "MotorDriver_OnTim1BreakFromIsr();")
    print("CubeMX safety configuration is consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
