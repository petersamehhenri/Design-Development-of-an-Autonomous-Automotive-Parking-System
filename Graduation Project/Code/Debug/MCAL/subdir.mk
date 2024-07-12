################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/GPIO.c \
../MCAL/I2C.c \
../MCAL/ISR.c \
../MCAL/RCC.c \
../MCAL/TIMER.c \
../MCAL/UART.c \
../MCAL/WDGT.c 

OBJS += \
./MCAL/GPIO.o \
./MCAL/I2C.o \
./MCAL/ISR.o \
./MCAL/RCC.o \
./MCAL/TIMER.o \
./MCAL/UART.o \
./MCAL/WDGT.o 

C_DEPS += \
./MCAL/GPIO.d \
./MCAL/I2C.d \
./MCAL/ISR.d \
./MCAL/RCC.d \
./MCAL/TIMER.d \
./MCAL/UART.d \
./MCAL/WDGT.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/GPIO.o: ../MCAL/GPIO.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/GPIO.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MCAL/I2C.o: ../MCAL/I2C.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/I2C.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MCAL/ISR.o: ../MCAL/ISR.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/ISR.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MCAL/RCC.o: ../MCAL/RCC.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/RCC.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MCAL/TIMER.o: ../MCAL/TIMER.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/TIMER.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MCAL/UART.o: ../MCAL/UART.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/UART.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MCAL/WDGT.o: ../MCAL/WDGT.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MCAL/WDGT.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

