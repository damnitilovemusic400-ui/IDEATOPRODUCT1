# PowerShell build script to create Vivado project (batch)
param(
    [string]$vivado = "vivado",
    [string]$tcl = "vivado/create_project.tcl"
)
Write-Host "Running Vivado in batch to create project..."
& $vivado -mode batch -source $tcl
Write-Host "TCL executed. Open Vivado project to run synthesis/implementation." 