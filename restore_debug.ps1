#!/usr/bin/env pwsh
# ============================================================================
# Script: Restore Debug UART After CubeMX Regenerate
# Purpose: Automatically restore debug_uart.h include and initialization
# Usage: ./restore_debug.ps1
# ============================================================================

param(
    [switch]$Force = $false
)

# Colors for output
$Green = "`e[32m"
$Red = "`e[31m"
$Yellow = "`e[33m"
$Blue = "`e[34m"
$Reset = "`e[0m"

function Write-Success { Write-Host "$Green✓ $args$Reset" }
function Write-Error-Custom { Write-Host "$Red✗ $args$Reset" }
function Write-Warn { Write-Host "$Yellow⚠ $args$Reset" }
function Write-Info { Write-Host "$Blue[*]$Reset $args" }

Write-Host ""
Write-Host "$(($Blue + '=' * 80 + $Reset))"
Write-Host "  STM32H750 - Restore Debug UART After CubeMX Regenerate"
Write-Host "$(($Blue + '=' * 80 + $Reset))"
Write-Host ""

# Check if files exist
$MainCPath = "Core\Src\main.c"
$FreeRTOSCfgPath = "Core\Inc\FreeRTOSConfig.h"
$DebugUartPath = "Core\Inc\debug_uart.h"

if (-not (Test-Path $MainCPath)) {
    Write-Error-Custom "Core/Src/main.c not found!"
    Write-Host "Please run this script from project root directory."
    exit 1
}

# ============================================================================
# STEP 1: Check and Restore #include "debug_uart.h"
# ============================================================================

Write-Info "Step 1: Checking for debug_uart.h include..."

$MainContent = Get-Content $MainCPath -Raw
if ($MainContent -match '#include "debug_uart\.h"') {
    Write-Success "debug_uart.h include already present"
} else {
    Write-Warn "debug_uart.h include missing - ADDING..."
    
    # Find the line "/* USER CODE BEGIN Includes */" and add include after it
    $Pattern = '(/\* USER CODE BEGIN Includes \*/)'
    $Replacement = '/* USER CODE BEGIN Includes */`n#include "debug_uart.h"'
    
    $NewContent = $MainContent -replace $Pattern, $Replacement
    
    # Save file
    Set-Content $MainCPath $NewContent -Encoding UTF8
    Write-Success "Include added successfully!"
}

# ============================================================================
# STEP 2: Check and Restore Debug_UART_Init() call
# ============================================================================

Write-Info "Step 2: Checking for Debug_UART_Init() initialization..."

$MainContent = Get-Content $MainCPath -Raw
if ($MainContent -match 'Debug_UART_Init\(\)') {
    Write-Success "Debug_UART_Init() already present"
} else {
    Write-Warn "Debug_UART_Init() missing - ADDING..."
    
    # Find "MX_TIM2_Init();" and add debug init after it
    $Pattern = '(MX_TIM2_Init\(\);)\s+(/\* USER CODE BEGIN 2 \*/)'
    $Replacement = '$1`n   /* USER CODE BEGIN 2 */`n   `n   /* Initialize Debug UART for PC communication */`n   Debug_UART_Init();`n   Debug_Test();'
    
    $NewContent = $MainContent -replace $Pattern, $Replacement
    
    # Save file
    Set-Content $MainCPath $NewContent -Encoding UTF8
    Write-Success "Debug initialization added successfully!"
}

# ============================================================================
# STEP 3: Check and Enable FPU in FreeRTOSConfig.h
# ============================================================================

Write-Info "Step 3: Checking FreeRTOSConfig.h for FPU enablement..."

$FreeRTOSContent = Get-Content $FreeRTOSCfgPath -Raw
if ($FreeRTOSContent -match '#define configENABLE_FPU\s+1') {
    Write-Success "FPU is enabled"
} else {
    Write-Warn "FPU disabled - ENABLING..."
    
    # Replace configENABLE_FPU 0 with 1
    $Pattern = '#define configENABLE_FPU\s+0'
    $Replacement = '#define configENABLE_FPU                         1'
    
    $NewContent = $FreeRTOSContent -replace $Pattern, $Replacement
    
    # Save file
    Set-Content $FreeRTOSCfgPath $NewContent -Encoding UTF8
    Write-Success "FPU enabled successfully!"
}

# ============================================================================
# VERIFICATION
# ============================================================================

Write-Info "Step 4: Verifying all fixes..."

$MainContent = Get-Content $MainCPath -Raw
$FreeRTOSContent = Get-Content $FreeRTOSCfgPath -Raw

$IncludeOK = $MainContent -match '#include "debug_uart\.h"'
$InitOK = $MainContent -match 'Debug_UART_Init\(\)'
$TestOK = $MainContent -match 'Debug_Test\(\)'
$FPUOK = $FreeRTOSContent -match '#define configENABLE_FPU\s+1'

Write-Host ""
if ($IncludeOK) { Write-Success "Include debug_uart.h" } else { Write-Error-Custom "Include debug_uart.h" }
if ($InitOK) { Write-Success "Debug_UART_Init() call" } else { Write-Error-Custom "Debug_UART_Init() call" }
if ($TestOK) { Write-Success "Debug_Test() call" } else { Write-Error-Custom "Debug_Test() call" }
if ($FPUOK) { Write-Success "FPU enabled" } else { Write-Error-Custom "FPU enabled" }

# ============================================================================
# SUMMARY
# ============================================================================

Write-Host ""
Write-Host "$(($Blue + '=' * 80 + $Reset))"

if ($IncludeOK -and $InitOK -and $TestOK -and $FPUOK) {
    Write-Host "  ✓ All Restorations Complete!" -ForegroundColor Green
} else {
    Write-Host "  ⚠ Some items need manual fixing" -ForegroundColor Yellow
}

Write-Host "$(($Blue + '=' * 80 + $Reset))"
Write-Host ""
Write-Host "Next steps:"
Write-Host "  1. Build project: ${Yellow}make clean && make$Reset"
Write-Host "  2. Flash to board: ${Yellow}make flash$Reset"
Write-Host "  3. Open terminal at ${Yellow}115200 bps$Reset"
Write-Host "  4. Reset board and verify debug output"
Write-Host ""
