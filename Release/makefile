################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./display/charter_module.obj" \
"./main/main.obj" \
"./peripherals/bq27441.obj" \
"./peripherals/gpio.obj" \
"./peripherals/i2c.obj" \
"./peripherals/misc.obj" \
"./peripherals/mpu9250_drv.obj" \
"./peripherals/mpu9x50.obj" \
"./peripherals/spi.obj" \
"./peripherals/st7735r128x128x18.obj" \
"./peripherals/timer.obj" \
"./power/dips_module.obj" \
"./radio/flare_module.obj" \
"./sensors/imu_tests.obj" \
"./sensors/windrose_module.obj" \
"./utils/sine.obj" \
"./utils/uartstdio.obj" \
"./utils/ustdlib.obj" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/TivaWare_C_Series-2.1.3.156/grlib/ccs/Debug/grlib.lib" \
-l"C:/ti/TivaWare_C_Series-2.1.3.156/driverlib/ccs/Debug/driverlib.lib" \
-l"C:/ti/TivaWare_C_Series-2.1.3.156/sensorlib/ccs/Debug/sensorlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include display/subdir_vars.mk
-include main/subdir_vars.mk
-include peripherals/subdir_vars.mk
-include power/subdir_vars.mk
-include radio/subdir_vars.mk
-include sensors/subdir_vars.mk
-include utils/subdir_vars.mk
-include display/subdir_rules.mk
-include main/subdir_rules.mk
-include peripherals/subdir_rules.mk
-include power/subdir_rules.mk
-include radio/subdir_rules.mk
-include sensors/subdir_rules.mk
-include utils/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Overlord\ System.out \

EXE_OUTPUTS__QUOTED += \
"Overlord System.out" \

BIN_OUTPUTS += \
Overlord\ System.bin \

BIN_OUTPUTS__QUOTED += \
"Overlord System.bin" \


# All Target
all: Overlord\ System.out

# Tool invocations
Overlord\ System.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --advice:power="all" --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PGE --define=TARGET_IS_TM4C123_RB1 --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual -z -m"blinky_ccs.map" --heap_size=0 --stack_size=512 -i"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/lib" -i"C:/ti/TivaWare_C_Series-2.1.3.156" -i"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/include" --reread_libs --display_error_number --diag_wrap=off --warn_sections --xml_link_info="Overlord System_linkInfo.xml" --rom_model -o "Overlord System.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

Overlord\ System.bin: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/bin/armhex"  -o "Overlord System.bin" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "display\charter_module.obj" "main\main.obj" "peripherals\bq27441.obj" "peripherals\gpio.obj" "peripherals\i2c.obj" "peripherals\misc.obj" "peripherals\mpu9250_drv.obj" "peripherals\mpu9x50.obj" "peripherals\spi.obj" "peripherals\st7735r128x128x18.obj" "peripherals\timer.obj" "power\dips_module.obj" "radio\flare_module.obj" "sensors\imu_tests.obj" "sensors\windrose_module.obj" "utils\sine.obj" "utils\uartstdio.obj" "utils\ustdlib.obj" 
	-$(RM) "display\charter_module.d" "main\main.d" "peripherals\bq27441.d" "peripherals\gpio.d" "peripherals\i2c.d" "peripherals\misc.d" "peripherals\mpu9250_drv.d" "peripherals\mpu9x50.d" "peripherals\spi.d" "peripherals\st7735r128x128x18.d" "peripherals\timer.d" "power\dips_module.d" "radio\flare_module.d" "sensors\imu_tests.d" "sensors\windrose_module.d" "utils\sine.d" "utils\uartstdio.d" "utils\ustdlib.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"C:/ti/ccsv7/utils/tiobj2bin/tiobj2bin" "Overlord System.out" "Overlord System.bin" "C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/bin/armofd" "C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/bin/armhex" "C:/ti/ccsv7/utils/tiobj2bin/mkhex4bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

