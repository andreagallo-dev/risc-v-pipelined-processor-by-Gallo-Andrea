//-----------------------------------------------------------------------------
// Module:  HazardUnit
// Author:  Gallo Andrea 2359271
//
// Description:
//   This unit detects and resolves data and control hazards in the 5-stage
//   RISC-V pipeline. It generates control signals for forwarding, stalling,
//   and flushing to ensure correct pipeline operation.
//-----------------------------------------------------------------------------
module HazardUnit (
    // Inputs
    input [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,    
    input [1:0]  PcSrc,
    input [1:0]  ResultSrcE,
    input        RegWriteM, RegWriteW,
    
    // Outputs
    output logic [1:0] ForwardAE, ForwardBE,
    output logic       StallF, StallD, FlushD, FlushE
);

    always_comb begin
        // --- Default Assignments ---
        // By default, no forwarding is needed.
        ForwardAE = 2'b00;
        ForwardBE = 2'b00;

        // --- Forwarding Logic for Rs1 (Operand A) ---
        // Check for a potential data hazard for the first operand (Rs1).
        
        // Priority 1: Forward from MEM stage to EX stage.
        // This has higher priority because it's the most recent data.
        if (RegWriteM && (RdM != 5'b0) && (RdM == Rs1E)) begin
            ForwardAE = 2'b01; // Select data from MEM stage
        end
        // Priority 2: Forward from WB stage to EX stage.
        // Used only if there is no MEM->EX dependency for the same register.
        else if (RegWriteW && (RdW != 5'b0) && (RdW == Rs1E)) begin
            ForwardAE = 2'b10; // Select data from WB stage
        end

        // --- Forwarding Logic for Rs2 (Operand B) ---
        // Check for a potential data hazard for the second operand (Rs2).

        // Priority 1: Forward from MEM stage to EX stage.
        if (RegWriteM && (RdM != 5'b0) && (RdM == Rs2E)) begin
            ForwardBE = 2'b01; // Select data from MEM stage
        end
        // Priority 2: Forward from WB stage to EX stage.
        else if (RegWriteW && (RdW != 5'b0) && (RdW == Rs2E)) begin
            ForwardBE = 2'b10; // Select data from WB stage
        end

        // --- Stall and Flush Logic ---
        // Detects load-use hazards and control hazards.

        // Load-Use Hazard Detection:
        // Occurs if the instruction in EX is a load (ResultSrcE == 01) and its destination
        // register (RdE) is needed as a source by the instruction in ID (Rs1D or Rs2D).
        if (ResultSrcE == 2'b01 && (RdE != 5'b0) && ((RdE == Rs1D) || (RdE == Rs2D))) begin
            // Hazard detected: stall the pipeline and flush the EX stage.
            StallF = 1'b1; // Stop the Program Counter
            StallD = 1'b1; // Keep the same instruction in the IF/ID register
            FlushE = 1'b1; // Convert the instruction in EX to a NOP (bubble)
        end else begin
            // No load-use hazard, so no stall is needed.
            StallF = 1'b0;
            StallD = 1'b0;

            // However, a Control Hazard might still require flushing the EX stage.
            // If a branch/jump is taken (PcSrc != 00), we must flush.
            if (PcSrc != 2'b00) begin
                FlushE = 1'b1;
            end else begin
                FlushE = 1'b0;
            end
        end
        
        // Control Hazard Detection:
        // If a branch or jump is taken (PcSrc != 00), the instruction
        // fetched immediately after it (now in the ID stage) is incorrect and must be flushed.
        if (PcSrc != 2'b00) begin
            FlushD = 1'b1; // Convert the instruction in ID to a NOP (bubble)
        end else begin
            FlushD = 1'b0;
        end
    end

endmodule