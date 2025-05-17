################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Repo/Depreciated/attack_routine.cpp \
../Core/Repo/Depreciated/input_driver.cpp \
../Core/Repo/Depreciated/motor_control.cpp \
../Core/Repo/Depreciated/movement.cpp \
../Core/Repo/Depreciated/queue.cpp \
../Core/Repo/Depreciated/search_routine.cpp \
../Core/Repo/Depreciated/semaphore.cpp \
../Core/Repo/Depreciated/sensor_array.cpp \
../Core/Repo/Depreciated/start_routine.cpp 

OBJS += \
./Core/Repo/Depreciated/attack_routine.o \
./Core/Repo/Depreciated/input_driver.o \
./Core/Repo/Depreciated/motor_control.o \
./Core/Repo/Depreciated/movement.o \
./Core/Repo/Depreciated/queue.o \
./Core/Repo/Depreciated/search_routine.o \
./Core/Repo/Depreciated/semaphore.o \
./Core/Repo/Depreciated/sensor_array.o \
./Core/Repo/Depreciated/start_routine.o 

CPP_DEPS += \
./Core/Repo/Depreciated/attack_routine.d \
./Core/Repo/Depreciated/input_driver.d \
./Core/Repo/Depreciated/motor_control.d \
./Core/Repo/Depreciated/movement.d \
./Core/Repo/Depreciated/queue.d \
./Core/Repo/Depreciated/search_routine.d \
./Core/Repo/Depreciated/semaphore.d \
./Core/Repo/Depreciated/sensor_array.d \
./Core/Repo/Depreciated/start_routine.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Repo/Depreciated/%.o Core/Repo/Depreciated/%.su Core/Repo/Depreciated/%.cyclo: ../Core/Repo/Depreciated/%.cpp Core/Repo/Depreciated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Repo-2f-Depreciated

clean-Core-2f-Repo-2f-Depreciated:
	-$(RM) ./Core/Repo/Depreciated/attack_routine.cyclo ./Core/Repo/Depreciated/attack_routine.d ./Core/Repo/Depreciated/attack_routine.o ./Core/Repo/Depreciated/attack_routine.su ./Core/Repo/Depreciated/input_driver.cyclo ./Core/Repo/Depreciated/input_driver.d ./Core/Repo/Depreciated/input_driver.o ./Core/Repo/Depreciated/input_driver.su ./Core/Repo/Depreciated/motor_control.cyclo ./Core/Repo/Depreciated/motor_control.d ./Core/Repo/Depreciated/motor_control.o ./Core/Repo/Depreciated/motor_control.su ./Core/Repo/Depreciated/movement.cyclo ./Core/Repo/Depreciated/movement.d ./Core/Repo/Depreciated/movement.o ./Core/Repo/Depreciated/movement.su ./Core/Repo/Depreciated/queue.cyclo ./Core/Repo/Depreciated/queue.d ./Core/Repo/Depreciated/queue.o ./Core/Repo/Depreciated/queue.su ./Core/Repo/Depreciated/search_routine.cyclo ./Core/Repo/Depreciated/search_routine.d ./Core/Repo/Depreciated/search_routine.o ./Core/Repo/Depreciated/search_routine.su ./Core/Repo/Depreciated/semaphore.cyclo ./Core/Repo/Depreciated/semaphore.d ./Core/Repo/Depreciated/semaphore.o ./Core/Repo/Depreciated/semaphore.su ./Core/Repo/Depreciated/sensor_array.cyclo ./Core/Repo/Depreciated/sensor_array.d ./Core/Repo/Depreciated/sensor_array.o ./Core/Repo/Depreciated/sensor_array.su ./Core/Repo/Depreciated/start_routine.cyclo ./Core/Repo/Depreciated/start_routine.d ./Core/Repo/Depreciated/start_routine.o ./Core/Repo/Depreciated/start_routine.su

.PHONY: clean-Core-2f-Repo-2f-Depreciated

