@echo off
REM ============================================================================
REM Script: Restore Debug UART After CubeMX Regenerate
REM Purpose: Automatically restore debug_uart.h include and initialization
REM ============================================================================
REM Usage: restore_debug.bat
REM ============================================================================

setlocal enabledelayedexpansion

cd /d "%~dp0"
echo.
echo ============================================================================
echo   STM32H750 - Restore Debug UART After CubeMX Regenerate
echo ============================================================================
echo.

REM Check if main.c exists
if not exist "Core\Src\main.c" (
    echo ERROR: Core/Src/main.c not found!
    echo Please run this script from project root directory.
    pause
    exit /b 1
)

echo [1/3] Checking for debug_uart.h include in main.c...
findstr /M "debug_uart.h" "Core\Src\main.c" >nul
if %ERRORLEVEL% EQU 0 (
    echo         ✓ debug_uart.h include already present
) else (
    echo         ✗ debug_uart.h include missing - ADDING...
    REM Add include after "/* USER CODE BEGIN Includes */"
    setlocal enabledelayedexpansion
    for /f "tokens=*" %%A in (Core\Src\main.c) do (
        if "%%A"=="/* USER CODE BEGIN Includes */" (
            echo %%A >> Core\Src\main.c.tmp
            echo #include "debug_uart.h" >> Core\Src\main.c.tmp
        ) else (
            echo %%A >> Core\Src\main.c.tmp
        )
    )
    move /Y Core\Src\main.c.tmp Core\Src\main.c >nul
    echo         ✓ Include added!
)

echo.
echo [2/3] Checking for Debug_UART_Init() call in main.c...
findstr /M "Debug_UART_Init" "Core\Src\main.c" >nul
if %ERRORLEVEL% EQU 0 (
    echo         ✓ Debug_UART_Init() already present
) else (
    echo         ✗ Debug_UART_Init() missing - ADDING...
    REM This requires more complex insertion, show manual steps
    echo.
    echo         MANUAL STEP REQUIRED:
    echo         Open Core/Src/main.c and find:
    echo         "   MX_TIM2_Init();"
    echo         "   /* USER CODE BEGIN 2 */"
    echo         "   /* USER CODE END 2 */"
    echo.
    echo         Replace with:
    echo         "   MX_TIM2_Init();"
    echo         "   /* USER CODE BEGIN 2 */"
    echo         "   Debug_UART_Init();"
    echo         "   Debug_Test();"
    echo         "   /* USER CODE END 2 */"
    echo.
    pause
)

echo.
echo [3/3] Checking FreeRTOSConfig.h for FPU enablement...
findstr "configENABLE_FPU.*1" "Core\Inc\FreeRTOSConfig.h" >nul
if %ERRORLEVEL% EQU 0 (
    echo         ✓ FPU is enabled
) else (
    echo         ✗ FPU disabled - ENABLING...
    REM Replace configENABLE_FPU 0 with 1
    for /f "tokens=*" %%A in (Core\Inc\FreeRTOSConfig.h) do (
        if "%%A"=="#define configENABLE_FPU                         0" (
            echo #define configENABLE_FPU                         1 >> Core\Inc\FreeRTOSConfig.h.tmp
        ) else (
            echo %%A >> Core\Inc\FreeRTOSConfig.h.tmp
        )
    )
    move /Y Core\Inc\FreeRTOSConfig.h.tmp Core\Inc\FreeRTOSConfig.h >nul
    echo         ✓ FPU enabled!
)

echo.
echo ============================================================================
echo   ✓ Restoration Complete!
echo ============================================================================
echo.
echo Next steps:
echo   1. Build project: make clean && make
echo   2. Flash to board
echo   3. Open terminal at 115200 bps
echo   4. Reset board and verify debug output
echo.
pause
