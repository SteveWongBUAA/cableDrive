################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
CMD/28335_RAM_lnk.exe: F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/CMD/28335_RAM_lnk.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Linker'
	"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/bin/cl2000" -v28 -ml --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/Debug" -z --stack_size=1000 -m"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/CMD/Debug/cdmcs.map" --heap_size=1000 --warn_sections -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/lib" -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/include" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/cdmcs_3dof" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/LIB" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/CMD/LIB" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="cdmcs_3dof_linkInfo.xml" --absolute_exe --rom_model -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMD/DSP2833x_Headers_nonBIOS.exe: F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/CMD/DSP2833x_Headers_nonBIOS.cmd $(GEN_CMDS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Linker'
	"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/bin/cl2000" -v28 -ml --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 --display_error_number --diag_wrap=off --obj_directory="F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/Debug" -z --stack_size=1000 -m"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/CMD/Debug/cdmcs.map" --heap_size=1000 --warn_sections -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/lib" -i"E:/TI/ccsv5/tools/compiler/c2000_6.1.3/include" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/cdmcs_3dof" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/LIB" -i"F:/studying/graduate/2014-9-22/lab/driveboard/mine/soft/15-7-16-7DOF/CMD/LIB" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="cdmcs_3dof_linkInfo.xml" --absolute_exe --rom_model -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


