################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CANLib/Defines/CAN_System_Def.c 

OBJS += \
./CANLib/Defines/CAN_System_Def.o 

C_DEPS += \
./CANLib/Defines/CAN_System_Def.d 


# Each subdirectory must supply rules for building sources it contributes
CANLib/Defines/%.o CANLib/Defines/%.su CANLib/Defines/%.cyclo: ../CANLib/Defines/%.c CANLib/Defines/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../CANLib/CAN_Main -I../CANLib/Defines/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CANLib-2f-Defines

clean-CANLib-2f-Defines:
	-$(RM) ./CANLib/Defines/CAN_System_Def.cyclo ./CANLib/Defines/CAN_System_Def.d ./CANLib/Defines/CAN_System_Def.o ./CANLib/Defines/CAN_System_Def.su

.PHONY: clean-CANLib-2f-Defines

