################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
CMD/28335_RAM_lnk.exe: F:/cableDrive/dsp_7DOF0/CMD/28335_RAM_lnk.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Linker'
	"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="F:/cableDrive/dsp_7DOF0/Debug" -z --stack_size=0x300 -m"F:/cableDrive/dsp_7DOF0/CMD/Debug/cdmcs.map" --heap_size=1000 --warn_sections -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/lib" -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/include" -i"F:/cableDrive/dsp_7DOF0/cdmcs_3dof" -i"F:/cableDrive/dsp_7DOF0" -i"F:/cableDrive/dsp_7DOF0/LIB" -i"F:/cableDrive/dsp_7DOF0/CMD/LIB" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="cdmcs_3dof_linkInfo.xml" --absolute_exe --entry_point=code_start --rom_model -o "$@" "$<" "../28335_RAM_lnk.cmd"
	@echo 'Finished building: $<'
	@echo ' '

CMD/DSP2833x_Headers_nonBIOS.exe: F:/cableDrive/dsp_7DOF0/CMD/DSP2833x_Headers_nonBIOS.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Linker'
	"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="F:/cableDrive/dsp_7DOF0/Debug" -z --stack_size=0x300 -m"F:/cableDrive/dsp_7DOF0/CMD/Debug/cdmcs.map" --heap_size=1000 --warn_sections -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/lib" -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/include" -i"F:/cableDrive/dsp_7DOF0/cdmcs_3dof" -i"F:/cableDrive/dsp_7DOF0" -i"F:/cableDrive/dsp_7DOF0/LIB" -i"F:/cableDrive/dsp_7DOF0/CMD/LIB" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="cdmcs_3dof_linkInfo.xml" --absolute_exe --entry_point=code_start --rom_model -o "$@" "$<" "../28335_RAM_lnk.cmd"
	@echo 'Finished building: $<'
	@echo ' '


