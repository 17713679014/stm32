SET PATH=D:\new software\keil5_latest\ARM\ARMCC\Bin;C:\Program Files\Java\jdk1.8.0_121\bin;D:\Git\cmd
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=ADC-DEMO
SET CPU_CLOCK=0x00B71B00
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmAsm" --Via ".\startup_stm32f10x_hd._ia"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\main.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_it.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\usart1.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\adc.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\led.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\time_test.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_gpio.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_rcc.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_usart.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_adc.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_dma.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_flash.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\stm32f10x_tim.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\misc.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\core_cm3.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmCC" --Via ".\system_stm32f10x.__i"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\ArmLink" --Via ".\STM32-DEMO.lnp"
"D:\new software\keil5_latest\ARM\ARMCC\Bin\fromelf.exe" ".\STM32-DEMO.axf" --i32combined --output ".\STM32-DEMO.hex"
