read -overwrite
puts "Creating Vivado project skeleton"
set proj_name "sumoxml_fpga"
set proj_dir [pwd]
create_project $proj_name $proj_dir -part xc7a100t-1-bg484

# add sources
set srcs [list \
    ../rtl/top.v \
    ../rtl/framebuffer_rom.v \
    ../rtl/geometry_rom.v \
    ../rtl/lights_bram.v ]
foreach s $srcs { add_files $s }

# add tb
add_files -fileset sim_1 ../tb/tb_top.v

write_project_tcl ${proj_name}_project.tcl
puts "Project created. Manually run synthesis/impl in Vivado."
