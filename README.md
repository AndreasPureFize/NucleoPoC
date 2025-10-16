# NucleoPoC – STM32G0 Proof of Concept
**Version:** 0.1.0 (Beta)

## Overview
NucleoPoC is a proof-of-concept firmware for STM32 Nucleo boards using STM32CubeIDE. It demonstrates a modular architecture with HAL drivers, a CLI interface, and a custom DataLink protocol.

## Features
- GPIO LED heartbeat (PC6)
- TIM2 periodic interrupt
- USART1 (9600 baud), USART2 (115200 baud)
- CLI commands: CONFIG, READ, SET, RESET, HELP, EXIT
- DataLink protocol with CRC16 and retries

## Build Requirements
- STM32CubeIDE (latest version recommended)
- STM32G031K8T6 Nucleo board
- ST HAL and CMSIS drivers included

## Getting Started
1. Clone the repository:
   ```bash
   git clone https://github.com/AndreasPureFize/NucleoPoC.git
   cd NucleoPoC


Open in STM32CubeIDE and build.
Flash to the Nucleo board.

CLI Usage (USART2 @ 115200 baud)
Example commands:
HELP
CONFIG CHANNEL 2
CONFIG POWER 0.8
READ DATA
SET OUTPUT 1
RESET ERRORS
EXIT

Versioning

Current: v0.1.0 (Beta)
Next planned: v0.2.0 (feature enhancements)

License
ST HAL and CMSIS under their respective licenses. Custom code provided AS-IS.
