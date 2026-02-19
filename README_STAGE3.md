Stage 3 — FPGA Integration (Nexys 4 DDR)
======================================

Overview
--------
This folder contains HDL wrappers, testbench, and helper scripts to integrate the COE assets produced in Stages 0–2 into a Vivado-based FPGA project targeting Nexys 4 DDR (Artix-7 XC7A100T). The deliverable is a project skeleton, memory map, and host API example.

Files added
-----------
- `rtl/` — Verilog wrappers: `framebuffer_rom.v`, `geometry_rom.v`, `lights_bram.v`, `top.v`
- `tb/` — Testbench: `tb_top.v`
- `vivado/create_project.tcl` — TCL to create a Vivado project and add sources
- `build.ps1` — PowerShell script to run Vivado in batch for project creation
- `coe_to_mem.py` — convert `.coe` files to `.mem` hex for simulation
- `host_api.py` — example UART client (Python / pyserial)
- `emit_runtime_coes.py` and `COE results/final_coes` — final COEs and memory_map.txt (from Stage‑2)

Quick start
-----------
1. Convert COEs to .mem (used by simulation and the testbench):

```powershell
python coe_to_mem.py "COE results/final_coes" "COE results/final_coes"
```

2. Create Vivado project (requires Vivado on PATH):

```powershell
.
\build.ps1
```

3. Open project in Vivado and run simulation / synthesis.

Notes & recommendations
-----------------------
- `framebuffer.coe` is large (307,200 entries). Store it in external DDR on Nexys 4 DDR. Use BRAM for small tables (`geometry32`, `lights`).
- The provided top-level is a skeleton — integrate your VGA controller, AXI/BRAM interfaces, and host UART logic.
- The `memory_map.txt` in `COE results/final_coes` contains recommended logical addresses and estimated BRAM usage.

What I can do next
------------------
- Add AXI-lite register file and UART command parser in Verilog.
- Integrate an existing VGA controller and adapt BRAM/DDR controllers for Nexys 4 DDR block design (Vivado IP integrator).
- Attempt synthesis and timing in a CI environment (requires Vivado installed).
