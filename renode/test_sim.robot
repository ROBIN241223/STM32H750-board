*** Settings ***
Suite Setup                   Setup
Suite Teardown                Teardown
Library                       renode-testing-library/renode-testing-library.py

*** Variables ***
${SCRIPTS_DIR}                ${CURDIR}/../build
${REPL_OVERRIDE}              ${CURDIR}/stm32h750_board.repl
${UART1}                      sysbus.usart1
${UART2}                      sysbus.usart2

*** Keywords ***
Setup
    [Documentation]           Create machine and load platform
    Execute Command           mach create
    Execute Command           machine LoadPlatformDescription @${REPL_OVERRIDE}
    Execute Command           sysbus WriteDoubleWord 0x58024804 0x00002000
    Execute Command           sysbus WriteDoubleWord 0x5802480C 0x00002000
    Execute Command           sysbus WriteDoubleWord 0x58024400 0x3FFE0000

Teardown
    Execute Command           machine Destroy

*** Test Cases ***
Should Boot Without Hanging
    [Documentation]           Load ELF and check that CPU starts
    Execute Command           sysbus LoadELF @${SCRIPTS_DIR}/STM32H750.elf
    Execute Command           cpu VectorTableOffset 0x90000000
    Create Terminal Tester    ${UART1}
    Start Emulation
    Wait For Line On Uart     Debug UART Initialized    timeout=5
