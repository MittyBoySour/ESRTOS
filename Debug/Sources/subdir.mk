################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/LEDs.c \
../Sources/main.c 

OBJS += \
./Sources/LEDs.o \
./Sources/main.o 

C_DEPS += \
./Sources/LEDs.d \
./Sources/main.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\11238639\Desktop\ESRTOSPIT\Project\Library" -I"C:/Users/11238639/Desktop/ESRTOSPIT/Project/Static_Code/IO_Map" -I"C:/Users/11238639/Desktop/ESRTOSPIT/Project/Sources" -I"C:/Users/11238639/Desktop/ESRTOSPIT/Project/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


