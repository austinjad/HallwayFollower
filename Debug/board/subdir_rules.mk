################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
board/%.obj: ../board/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/home/tiz/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/tiz/ti/ccs1110/ccs/ccs_base/arm/include" --include_path="/home/tiz/ti/ccs1110/ccs/ccs_base/arm/include/CMSIS" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/simplelink/include" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/simplelink/source" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/board" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/cli_uart" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/driverlib/MSP432P4xx" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/spi_cc3100" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/uart_cc3100" --include_path="/home/tiz/ti/tirslk_max_1_00_02/HallwayFollower/mqtt" --include_path="inc" --include_path="/home/tiz/ti/ccs1110/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs --define=_USE_CLI_ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="board/$(basename $(<F)).d_raw" --obj_directory="board" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


