################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Repo/Src/ESC_Driver.cpp \
../Core/Repo/Src/Sumo_Robot.cpp \
../Core/Repo/Src/VL53L0X.cpp \
../Core/Repo/Src/WaveShareDriver.cpp \
../Core/Repo/Src/btn_matrix.cpp \
../Core/Repo/Src/cpp_main.cpp 

OBJS += \
./Core/Repo/Src/ESC_Driver.o \
./Core/Repo/Src/Sumo_Robot.o \
./Core/Repo/Src/VL53L0X.o \
./Core/Repo/Src/WaveShareDriver.o \
./Core/Repo/Src/btn_matrix.o \
./Core/Repo/Src/cpp_main.o 

CPP_DEPS += \
./Core/Repo/Src/ESC_Driver.d \
./Core/Repo/Src/Sumo_Robot.d \
./Core/Repo/Src/VL53L0X.d \
./Core/Repo/Src/WaveShareDriver.d \
./Core/Repo/Src/btn_matrix.d \
./Core/Repo/Src/cpp_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Repo/Src/%.o Core/Repo/Src/%.su Core/Repo/Src/%.cyclo: ../Core/Repo/Src/%.cpp Core/Repo/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/logan/STM32CubeIDE/workspace_1.18.0/sumo/Core/Repo/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Repo-2f-Src

clean-Core-2f-Repo-2f-Src:
	-$(RM) ./Core/Repo/Src/ESC_Driver.cyclo ./Core/Repo/Src/ESC_Driver.d ./Core/Repo/Src/ESC_Driver.o ./Core/Repo/Src/ESC_Driver.su ./Core/Repo/Src/Sumo_Robot.cyclo ./Core/Repo/Src/Sumo_Robot.d ./Core/Repo/Src/Sumo_Robot.o ./Core/Repo/Src/Sumo_Robot.su ./Core/Repo/Src/VL53L0X.cyclo ./Core/Repo/Src/VL53L0X.d ./Core/Repo/Src/VL53L0X.o ./Core/Repo/Src/VL53L0X.su ./Core/Repo/Src/WaveShareDriver.cyclo ./Core/Repo/Src/WaveShareDriver.d ./Core/Repo/Src/WaveShareDriver.o ./Core/Repo/Src/WaveShareDriver.su ./Core/Repo/Src/btn_matrix.cyclo ./Core/Repo/Src/btn_matrix.d ./Core/Repo/Src/btn_matrix.o ./Core/Repo/Src/btn_matrix.su ./Core/Repo/Src/cpp_main.cyclo ./Core/Repo/Src/cpp_main.d ./Core/Repo/Src/cpp_main.o ./Core/Repo/Src/cpp_main.su

.PHONY: clean-Core-2f-Repo-2f-Src

