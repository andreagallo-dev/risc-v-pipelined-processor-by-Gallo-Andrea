# 5-Stage Pipelined RISC-V Processor (RV32I)

This project details the design, implementation, and verification of a 5-stage pipelined processor, fully compliant with the RISC-V RV32I base integer instruction set. The primary objective was to tackle the practical challenges of modern computer architecture, with a specific focus on pipeline hazard management to ensure correct program execution. The entire processor is described in SystemVerilog.

![Pipeline Block Diagram](Pipeline Block Diagram.png)

### Key Features

*   **5-Stage Pipelined Architecture:** Implementation of the classic RISC pipeline: Instruction Fetch (IF), Decode (ID), Execute (EX), Memory Access (MEM), and Write-Back (WB).
*   **Data Hazard Resolution:**
    *   **Comprehensive Forwarding (Bypassing) Logic:** A multi-path forwarding unit was implemented to resolve Read-After-Write (RAW) data hazards by routing results from the EX/MEM and MEM/WB stages directly to the ALU inputs.
    *   **Load-Use Stall Management:** Detection and handling of the critical "load-use" hazard by stalling the pipeline for one cycle and injecting a bubble (NOP).
*   **Control Hazard Management:** Implementation of flushing logic to squash speculatively-fetched instructions following a taken branch or a jump.
*   **Systematic Verification:** The processor's functional correctness was rigorously validated using a custom-written assembly program, specifically designed to trigger and test every forwarding path, stall condition, and control flow change.

### Technology Stack

*   **HDL Language:** `SystemVerilog`
*   **Instruction Set Architecture (ISA):** `RISC-V (RV32I)`
*   **Simulation & Verification Tools:** `Icarus Verilog`, `GTKWave`