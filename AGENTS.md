**FC-DRON-P3 — agent map**  
Custom STM32F405 flight controller . Firmware lives in PinCubeMX/.  
   
 Hardware in hardware/. Host tools in tools/. Architecture notes in docs/.  
**Do not touch unless asked**  
- PinCubeMX/Core/, FATFS/, USB_DEVICE/ except USER CODE blocks  
- PinCubeMX/Drivers/, Middlewares/ (HAL stubs; real HAL is external)  
- Regenerating CubeMX without keeping UnderRoot=true (.ioc must stay next to Core/)  
- datasheets/ PDFs and hardware/**/Footprints/  
**Canonical files**  
| | |  
|-|-|  
| **What** | **Where** |   
| Pin aliases CubeMX missed | PinCubeMX/board/pinout.h |   
| Board / buses / rates | PinCubeMX/board/board.md |   
| Firmware rules | PinCubeMX/AGENTS.md |   
| Betaflight-inspired loop | docs/Diagrama_logica_Betaflight.mmd |   
| CubeMX project | PinCubeMX/PinCubeMX.ioc (not docs/NewPinCube-Drone.ioc) |   
   
**Firmware layers (dependencies only downward)**  
tests → app/fc, app/drivers, app/platform  
 fc / scheduler / control → sensors, drivers, platform  
 sensors → drivers  
 drivers → CubeMX HAL + board/pinout.h  
   
Work in PinCubeMX/app/ for production code and PinCubeMX/tests/ for bring-up.  
   
 Drivers must not live under tests/.  
**Build**  
cd PinCubeMX  
 cmake --preset Debug  
 cmake --build --preset Debug  
   
Requires arm-none-eabi-gcc and STM32Cube F4 V1.28.3 (STM32_CUBE_FW_F4_PATH if not in the default Cube repository path).  
