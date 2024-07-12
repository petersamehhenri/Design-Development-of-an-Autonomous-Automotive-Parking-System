################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/DC_MOTOR.c \
../HAL/Servo.c \
../HAL/ULTRASONIC.c 

OBJS += \
./HAL/DC_MOTOR.o \
./HAL/Servo.o \
./HAL/ULTRASONIC.o 

C_DEPS += \
./HAL/DC_MOTOR.d \
./HAL/Servo.d \
./HAL/ULTRASONIC.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/DC_MOTOR.o: ../HAL/DC_MOTOR.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"HAL/DC_MOTOR.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
HAL/Servo.o: ../HAL/Servo.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"HAL/Servo.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
HAL/ULTRASONIC.o: ../HAL/ULTRASONIC.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/HAL/inc" -I"F:/studying/final_project/self_parking/New folder (3)/Autonomous_Parking/MCAL/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"HAL/ULTRASONIC.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

