# STM32F0xx_StdPeriph_Lib_V1.3.1_IARICFAllocate

update @ 2019/04/12

Add function allocate example in IAR

function with specific function pointer and define section in ICF file , will allocate to specific address

1. check A_BootFunc_1 , ... , Z_BootFunc_6 in Hw_config.c for example.

2. function A_BootFunc_1 , ... , Z_BootFunc_6 , allocate in specific flash address

3. check BootLoaderSection section in stm32f030_flash.icf for example