################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
driverlib/MSP432P4xx/%.obj: ../driverlib/MSP432P4xx/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/opt/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/opt/ti/ccsv8/ccs_base/arm/include" --include_path="/home/lucas/Documents/git/balance-bot/msp-ws/balance-bot/driverlib/MSP432P4xx" --include_path="/opt/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/home/lucas/Documents/git/balance-bot/msp-ws/balance-bot" --include_path="/opt/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="driverlib/MSP432P4xx/$(basename $(<F)).d_raw" --obj_directory="driverlib/MSP432P4xx" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


