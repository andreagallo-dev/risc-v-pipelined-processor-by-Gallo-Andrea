//-----------------------------------------------------------------------------
// Module:  topmodule
// Author:  Gallo Andrea 2359271
//
// Description:
// This module implements a 5-stage pipelined 32-bit RISC-V processor (RV32I).
// The pipeline stages are:
//   1. IF (Instruction Fetch)
//   2. ID (Instruction Decode & Register Fetch)
//   3. EX (Execute)
//   4. MEM (Memory Access)
//   5. WB (Write-Back)
//
// Key Features:
// - Pipelined Architecture: Instructions are processed in parallel across the five
//   stages to improve throughput. Pipeline registers (IF_ID, ID_EX, EX_MEM, MEM_WB)
//   are implemented as packed structs to pass data and control signals between stages.
//
// - Hazard Management: The processor includes a dedicated 'HazardUnit' to detect
//   and resolve data and control hazards.
//     - Data Hazards: Resolved primarily through forwarding (bypassing). The results
//       from the EX and MEM stages are forwarded directly to the inputs of the ALU
//       in the EX stage. For load-use hazards (a load instruction followed immediately
//       by an instruction that uses its result), the pipeline is stalled for one cycle.
//     - Control Hazards: Handled by predicting branches as not-taken. When a branch
//       is taken or a jump occurs, the instructions incorrectly fetched into the
//       IF and ID stages are flushed from the pipeline. The branch decision and
//       target address calculation are performed in the EX stage.
//
// - Modular Design: The processor is built from several key sub-modules:
//     - instruction_mem & data_mem: Separate memories for instructions and data.
//     - regfile: The 32-entry register file.
//     - decoder & alu_decoder: Generate control signals from the instruction.
//     - alu: Performs arithmetic and logical operations.
//     - extender: Handles sign-extension for immediate values.
//
// - I/O Interface: The module provides outputs (write_data, address, write_mem)
//   to observe or interface with the data memory operations from a testbench
//-----------------------------------------------------------------------------


module topmodule  (input  logic        clk, rst, 
                    output logic [31:0] write_data, 
                    output logic [31:0] address, 
                    output logic        write_mem);
  
  //----------------------------------------------------------------------------
  // PIPELINE REGISTERS
  // Description:
  // Structs are used to group signals passing between pipeline stages, improving
  // code readability. For each stage, a '_next' version holds the combinational
  // output that will be captured by the register on the next clock edge.
  // This is a standard pattern for designing synchronous sequential logic.
  //----------------------------------------------------------------------------

  // IF/ID Register: Carries data from Instruction Fetch to Instruction Decode stage.
  typedef struct packed {
      logic [31:0] Instr;     // The fetched instruction
      logic [31:0] PC;        // The Program Counter for this instruction
      logic [31:0] PCPlus4;   // The address of the next sequential instruction
  } IF_ID_reg;

  IF_ID_reg IF_ID, IF_ID_next; // IF/ID pipeline register and its next state input

  // ID/EX Register: Carries data from Instruction Decode to Execute stage.
  typedef struct packed {
      // Control Signals for later stages
      logic        RegWrite;    // Enables write to register file in WB stage
      logic [1:0]  ResultSrc;   // Selects the source for the write-back result (ALU, Mem, PC+4)
      logic        write_mem;   // Enables write to data memory in MEM stage
      logic        Jump;        // Indicates a JAL instruction
      logic        Branch;      // Indicates a branch instruction
      logic [3:0]  ALUControl;  // Defines the ALU operation
      logic        ALUSrc;      // Selects the second ALU operand (Register or Immediate)

      // Data Values
      logic [4:0]  Rs1, Rs2;    // Source register numbers
      logic [31:0] RD1, RD2;    // Data read from source registers
      logic [31:0] PC;          // Program Counter value
      logic [4:0]  Rd;          // Destination register number
      logic [31:0] ImmExt;      // Sign-extended immediate value
      logic [31:0] PCPlus4;     // PC + 4 value (for JAL/JALR)
      
      // Instruction fields for hazard unit / debugging
      logic [6:0]  op;
      logic [2:0]  funct3;
  } ID_EX_reg;

  ID_EX_reg ID_EX, ID_EX_next; // ID/EX pipeline register and its next state input

  // MEM/WB Register: Carries data from Memory Access to Write-Back stage.
  typedef struct packed {
      // Control Signals
      logic        RegWrite;    // Enables write to register file
      logic [1:0]  ResultSrc;   // Selects the final result to write back
      
      // Data Values
      logic [31:0] ALUResult;   // Result from the ALU
      logic [31:0] ReadData;    // Data read from data memory
      logic [4:0]  Rd;          // Destination register number
      logic [31:0] PCPlus4;     // PC + 4 value (needed for JAL/JALR)
      logic [31:0] ImmExt;      // Immediate value (needed for some forwarding cases)
      logic [31:0] PCTarget;    // Branch/Jump target address (for forwarding/debugging)
  } MEM_WB_reg;

  MEM_WB_reg MEM_WB, MEM_WB_next; // MEM/WB pipeline register and its next state input

  // EX/MEM Register: Carries data from Execute to Memory Access stage.
  typedef struct packed {
      // Control Signals
      logic        RegWrite;    // Propagated to WB stage
      logic [1:0]  ResultSrc;   // Propagated to WB stage
      logic        write_mem;   // Enables write to data memory in this stage
      
      // Data Values
      logic [31:0] ALUResult;   // Result from ALU (used as memory address or forwarded)
      logic [31:0] write_data;  // Data to be written to memory (from Rs2)
      logic [4:0]  Rd;          // Destination register number (propagated to WB)
      logic [31:0] PCPlus4;     // Propagated for JAL/JALR instructions
      
      // Instruction fields for debugging / memory logic
      logic [6:0]  op;
      logic [2:0]  funct3;

      // Values needed for forwarding or debugging
      logic [31:0] ImmExt;
      logic [31:0] PCTarget;
  } EX_MEM_reg;

  EX_MEM_reg EX_MEM, EX_MEM_next; // EX/MEM pipeline register and its next state input


  /* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // PIPELINE REGISTER UPDATE LOGIC
  // Description:
  // This always_ff block represents the sequential heart of the processor.
  // On each rising clock edge, it updates all pipeline registers based on control
  // signals from the Hazard Unit. It handles reset, normal operation, stalls
  // (for data hazards), and flushes (for control hazards).
  //----------------------------------------------------------------------------

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        // On reset, all pipeline registers are cleared to a known '0' state.
        // This effectively injects NOPs and prevents unpredictable behavior at startup.
        IF_ID  <= '0;
        ID_EX  <= '0;
        EX_MEM <= '0;
        MEM_WB <= '0;
    end 
    else if (StallD) begin
        // A stall signal (for a load-use data hazard) has been asserted.
        // The pipeline freezes at the IF/ID stage to wait for data to become available.
        // - The IF_ID register holds its current value, preventing the next instruction
        //   from entering the Decode stage. The PC is also stalled (see IF logic).
        // - A bubble (NOP) is inserted into the ID/EX register. This is achieved by
        //   the Hazard Unit forcing the control signals in 'ID_EX_next' to 0.
        // - The later stages (EX_MEM, MEM_WB) continue to advance to drain the pipeline.
        IF_ID  <= IF_ID;      // Hold the instruction in the IF/ID register.
        ID_EX  <= ID_EX_next; // A bubble (NOP) generated by the hazard unit is loaded.
        EX_MEM <= EX_MEM_next;
        MEM_WB <= MEM_WB_next;
    end
    else begin
        // Normal operation: no reset or stall.
        // All pipeline stages advance by loading the values from their '_next' signals.
        IF_ID  <= IF_ID_next;
        ID_EX  <= ID_EX_next;
        EX_MEM <= EX_MEM_next;
        MEM_WB <= MEM_WB_next;
    end
    
    // Flush signals handle control hazards (e.g., taken branches, jumps).
    // They are independent 'if' statements because they must override any other
    // condition (like a stall or normal operation) to squash incorrect instructions.
    
    if (FlushD) begin
        // Flush the instruction in the Decode stage (currently in IF_ID register).
        // This is necessary if an instruction in a later stage caused a control flow change.
        IF_ID <= '0; // Squash by replacing with a NOP.
    end 
    if (FlushE) begin
        // Flush the instruction in the Execute stage (currently in ID_EX register).
        // This is done when a branch is resolved in the EX stage, making the
        // instruction that was just fetched/decoded incorrect.
        ID_EX <= '0; // Squash by replacing with a NOP.
    end 
  end
  /* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // MODULE INSTANTIATIONS
  // Description:
  // This section instantiates all the major hardware blocks of the processor.
  // Each module represents a key component of the RISC-V datapath or control logic.
  //----------------------------------------------------------------------------

  // Instruction Memory: Fetches instructions based on the Program Counter (PC).
  // It's a combinational-read memory.
  instruction_mem instruction_mem_u (PCCurrent, InstrImem);
  logic [31:0] PCCurrent;   // Input: Current PC address
  logic [31:0] InstrImem;   // Output: Instruction read from memory

  // Data Memory: Handles load and store operations.
  // It's a synchronous-read/write memory.
  data_mem data_mem_u (clk, dmem_we, dmem_addr, dmem_wd, dmem_byte_enabler, dmem_rd);
  logic [31:0] dmem_rd;     // Output: Data read from memory
  logic        dmem_we;     // Input: Write enable signal (from EX/MEM stage)
  logic [31:0] dmem_addr;   // Input: Address for load/store (from ALU result)
  logic [31:0] dmem_wd;     // Input: Data to write to memory
  logic [3:0]  dmem_byte_enabler; // Input: Byte enable for SB/SH instructions

  // Register File: Stores the 32 general-purpose registers.
  // Features two combinational-read ports (for rs1, rs2) and one synchronous-write port (for rd).
  regfile rf_u (clk, rf_we, rf_read_addr1, rf_read_addr2, rf_write_addr, rf_write_data, rf_read_data1, rf_read_data2);
  logic        rf_we;           // Input: Write enable (from MEM/WB stage)
  logic [4:0]  rf_read_addr1, rf_read_addr2, rf_write_addr; // Input: Register addresses
  logic [31:0] rf_write_data, rf_read_data1, rf_read_data2; // Input/Output: Register data

  // Main Decoder: Decodes the instruction's opcode to generate primary control signals.
  // Located in the ID stage.
  decoder decoder_u (op, resultsrc, memwrite, branch, alusrc, regwrite, jump, immsrc, aluop);
  alu_decoder alu_decoder_u (op, funct3, funct7b5, aluop, alucontrol);
  logic [6:0]  op;
  logic [2:0]  immsrc, funct3;
  logic        memwrite, branch, alusrc, regwrite, jump, funct7b5;
  logic [1:0]  resultsrc, aluop;
  logic [3:0]  alucontrol;

  // Immediate Extender: Generates the 32-bit sign-extended immediate from the instruction.
  // Located in the ID stage.
  extender extender_u (imm_src, immsrc, imm_extended);
  logic [31:0] imm_src, imm_extended;

  // Arithmetic Logic Unit (ALU): Performs arithmetic and logical operations.
  // Located in the EX stage.
  alu alu_u (alu_src_a, alu_src_b, alu_alucontrol, alu_result, alu_zero, alu_sign, alu_overflow, alu_carry);
  logic [31:0] alu_src_a, alu_src_b, alu_result; // Input/Output: ALU operands and result
  logic [3:0]  alu_alucontrol;                   // Input: Control signal for ALU operation
  logic        alu_zero, alu_sign, alu_overflow, alu_carry; // Output: Status flags (for branches)

  // Adder: A simple adder, typically used for calculating PC-relative branch/jump target addresses.
  // Located in the EX stage.
  adder add_u (adder_src_a, adder_src_b, adder_output);
  logic  [31:0] adder_src_a, adder_src_b, adder_output;

  //----------------------------------------------------------------------------
  // HAZARD UNIT
  // Description:
  // The central unit for detecting and resolving data and control hazards. It generates
  // stall and flush signals to control the pipeline flow and forwarding signals to
  // provide data directly between stages, bypassing the register file.
  //----------------------------------------------------------------------------
  HazardUnit hazard (hazard_Rs1D, hazard_Rs2D, hazard_Rs1E, hazard_Rs2E, 
                      hazard_RdE, hazard_RdM, hazard_RdW,
                      hazard_PcSrc, hazard_ResultSrcE, 
                      hazard_RegWriteM, hazard_RegWriteW,
                      hazard_ForwardAE, hazard_ForwardBE,
                      hazard_StallF, hazard_StallD, 
                      hazard_FlushD, hazard_FlushE);
  // Inputs to the Hazard Unit
  logic [4:0] hazard_Rs1D, hazard_Rs2D, hazard_Rs1E, hazard_Rs2E, hazard_RdE, hazard_RdM, hazard_RdW;
  logic [1:0] hazard_PcSrc;
  logic [1:0] hazard_ResultSrcE;
  logic       hazard_RegWriteM, hazard_RegWriteW;
  // Outputs from the Hazard Unit
  logic [1:0] hazard_ForwardAE, hazard_ForwardBE;
  logic       hazard_StallF, hazard_StallD, hazard_FlushD, hazard_FlushE;

  // Connecting Hazard Unit inputs to the correct pipeline register fields
  assign hazard_Rs1D = IF_ID.Instr[19:15]; // rs1 in ID stage
  assign hazard_Rs2D = IF_ID.Instr[24:20]; // rs2 in ID stage
  assign hazard_Rs1E = ID_EX.Rs1;          // rs1 in EX stage
  assign hazard_Rs2E = ID_EX.Rs2;          // rs2 in EX stage
  assign hazard_RdE  = ID_EX.Rd;           // rd in EX stage
  assign hazard_RdM  = EX_MEM.Rd;          // rd in MEM stage
  assign hazard_RdW  = MEM_WB.Rd;          // rd in WB stage
  assign hazard_PcSrc = PCSrcE;
  assign hazard_ResultSrcE = ID_EX.ResultSrc;
  assign hazard_RegWriteM = EX_MEM.RegWrite;
  assign hazard_RegWriteW = MEM_WB.RegWrite;

  // Creating local signals from Hazard Unit outputs for easier use in the datapath
  logic [1:0] ForwardAE, ForwardBE; // Forwarding select signals for ALU inputs A and B
  assign ForwardAE = hazard_ForwardAE; 
  assign ForwardBE = hazard_ForwardBE;
  logic StallF, StallD, FlushD, FlushE; // Stall and flush control signals
  assign StallF = hazard_StallF; 
  assign StallD = hazard_StallD;
  assign FlushD = hazard_FlushD; 
  assign FlushE = hazard_FlushE;
  assign PCJalr = {alu_result[31:1], 1'b0}; // JALR target address calculation (as per RISC-V spec)

  /* ---------------------------------------------------------------------------- */


  //----------------------------------------------------------------------------
  // GLOBAL SIGNALS
  // Wires for inter-stage communication, mainly for control flow and forwarding.
  //----------------------------------------------------------------------------
  logic [1:0] PCSrcE;     // Selects the next PC source (generated in EX).
  logic [31:0] PCTargetE; // Target address for branches and JAL (calculated in EX).
  logic [31:0] ResultW;   // Final result for register file write-back (from WB stage).
  logic [31:0] PCJalr;    // Calculated target address for JALR instructions.









/* ---------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------- */







  //----------------------------------------------------------------------------
  // INSTRUCTION FETCH (IF) STAGE
  //----------------------------------------------------------------------------
  logic [31:0] PCNext; // Holds the address for the next instruction.

  // PC Register: updates on clock edge.
  always_ff @(posedge clk or posedge rst) begin
    if (rst) PCCurrent <= 32'b0;
    else     PCCurrent <= PCNext;
  end

  // Next PC Selection Logic: calculates the address of the next instruction.
  // This is where control flow changes (branches, jumps) take effect.
  always_comb begin
    if (StallF) begin
      PCNext = PCCurrent; // Hold PC if the pipeline is stalled.
    end
    else begin
      case (PCSrcE)      // Select based on control signals from EX stage.
        2'b01:   PCNext = PCTargetE;     // Taken branch or JAL
        2'b10:   PCNext = PCJalr;        // JALR target address
        default: PCNext = PCCurrent + 4; // Default: next sequential instruction
      endcase
    end
  end

  // Load inputs for the IF/ID pipeline register.
  assign IF_ID_next.PC      = PCCurrent;
  assign IF_ID_next.Instr   = InstrImem;
  assign IF_ID_next.PCPlus4 = PCCurrent + 4;

/* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // INSTRUCTION DECODE (ID) STAGE
  // Decodes the instruction, reads from the register file, and prepares data/control
  // signals for the EX stage.
  //----------------------------------------------------------------------------

  // Local wires for clarity within the ID stage.
  logic [4:0]  rd_d;
  logic [31:0] PC_localID;
  logic [31:0] PCPlus4_localID;
  logic [31:0] ReadData1_localID, ReadData2_localID;

  // Extract fields from the instruction in the IF/ID register for the decoders.
  assign rd_d     = IF_ID.Instr[11:7];
  assign op       = IF_ID.Instr[6:0];
  assign funct3   = IF_ID.Instr[14:12];
  assign funct7b5 = IF_ID.Instr[30];
  assign imm_src  = IF_ID.Instr; // Pass the full instruction to the immediate extender.

  // Connect ports to the Register File.
  // Read ports use addresses from the current instruction in ID.
  // Write port is driven by the final instruction in the WB stage.
  assign rf_we           = MEM_WB.RegWrite;
  assign rf_read_addr1   = IF_ID.Instr[19:15]; // rs1
  assign rf_read_addr2   = IF_ID.Instr[24:20]; // rs2
  assign rf_write_addr   = MEM_WB.Rd;
  assign rf_write_data   = ResultW;

  // Pass-through values from the IF/ID register.
  assign PC_localID      = IF_ID.PC;
  assign PCPlus4_localID = IF_ID.PCPlus4;
  
  // WB-to-ID Stage Forwarding:
  // If the instruction in WB is writing to a register that the current instruction
  // in ID needs to read, forward the result directly.
  assign ReadData1_localID = (MEM_WB.Rd == rf_read_addr1 & MEM_WB.RegWrite & rf_read_addr1 != 0) ? ResultW :
                             rf_read_data1;
  assign ReadData2_localID = (MEM_WB.Rd == rf_read_addr2 & MEM_WB.RegWrite & rf_read_addr2 != 0) ? ResultW :
                             rf_read_data2;

  // Load inputs for the ID/EX pipeline register.
  always_comb begin
    // Control signals from the main decoder.
    ID_EX_next.ResultSrc  = resultsrc;
    ID_EX_next.write_mem  = memwrite;
    ID_EX_next.Branch     = branch;
    ID_EX_next.ALUSrc     = alusrc;
    ID_EX_next.RegWrite   = regwrite;
    ID_EX_next.Jump       = jump;
    ID_EX_next.ALUControl = alucontrol;
    
    // Data values to be passed to the EX stage.
    ID_EX_next.ImmExt     = imm_extended;
    ID_EX_next.Rs1        = rf_read_addr1;
    ID_EX_next.Rs2        = rf_read_addr2;
    ID_EX_next.RD1        = ReadData1_localID; // Potentially forwarded data
    ID_EX_next.RD2        = ReadData2_localID; // Potentially forwarded data
    ID_EX_next.Rd         = rd_d;
    ID_EX_next.PC         = PC_localID; 
    ID_EX_next.PCPlus4    = PCPlus4_localID;
    
    // Instruction fields passed for hazard detection and control.
    ID_EX_next.op         = op;
    ID_EX_next.funct3     = funct3;
  end

/* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // FORWARDING & EXECUTE PREPARATION LOGIC
  // This combinational logic prepares inputs for the EX stage. It includes:
  // - Forwarding data from later stages to prevent data hazards.
  // - Selecting ALU operands.
  // - Calculating branch/jump target addresses and evaluating conditions.
  //----------------------------------------------------------------------------
  logic [31:0] ForwardedA, ForwardedB, SrcB, StoreData;

  // Forwarding Unit MUXes: Select the correct source for ALU operands.
  // Data can come from ID/EX, EX/MEM, or WB, as determined by the Hazard Unit.
  assign ForwardedA = ForwardAE[1] ? ResultW :
                      ForwardAE[0] ? EX_MEM.ALUResult :
                      ID_EX.RD1;

  assign ForwardedB = ForwardBE[1] ? ResultW :
                      ForwardBE[0] ? EX_MEM.ALUResult :
                      ID_EX.RD2;

  // ALU Input Selection.
  assign SrcB           = ID_EX.ALUSrc ? ID_EX.ImmExt : ForwardedB; // Select Imm or Reg for SrcB
  assign alu_src_a      = ForwardedA;
  assign alu_src_b      = SrcB;
  assign alu_alucontrol = ID_EX.ALUControl;

  // Branch/JAL Target Address Calculation (PC-relative).
  assign adder_src_a = ID_EX.PC;
  assign adder_src_b = ID_EX.ImmExt;
  assign PCTargetE   = adder_output;

  // Next PC Selection (PCSrcE) Logic: Determines if a branch or jump should be taken.
  // This signal is fed back to the IF stage to control the PC.
  assign PCSrcE = (ID_EX.op == 7'b1100111) ? 2'b10 : // jalr
                  (ID_EX.Branch)           ? {1'b0, ( // branches
                                                      ((ID_EX.funct3 == 3'b000) && alu_zero) |                    //beq
                                                      ((ID_EX.funct3 == 3'b001) && !alu_zero) |                   //bne
                                                      ((ID_EX.funct3 == 3'b100) && (alu_sign ^ alu_overflow)) |   //blt
                                                      ((ID_EX.funct3 == 3'b101) && !(alu_sign ^ alu_overflow)) |  //bge
                                                      ((ID_EX.funct3 == 3'b110) && !alu_carry) |                  //bltu
                                                      ((ID_EX.funct3 == 3'b111) && alu_carry)                     //bgeu
                                                    )} :
                  (ID_EX.Jump)             ? 2'b01 :                                                              // jal
                                             2'b00;                                                              // default: next instruction

  // Memory Store Data Preparation.
  assign StoreData = (ID_EX.funct3 == 3'b000) ? {24'b0, ForwardedB[7:0]} :    // sb
                     (ID_EX.funct3 == 3'b001) ? {16'b0, ForwardedB[15:0]} :   // sh
                                                ForwardedB;                   // sw

  /* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // EXECUTE (EX) STAGE - REGISTER LOADING
  //----------------------------------------------------------------------------

  // A local wire is kept for the ALU result multiplexer for readability.
  logic [31:0] ALUResult_localEX;

  // Select the final result from the EX stage. This isn't always the raw ALU output.
  assign ALUResult_localEX = (ID_EX.op == 7'b0110111) ? ID_EX.ImmExt : // lui
                             (ID_EX.op == 7'b0010111) ? PCTargetE :    // auipc
                             (ID_EX.op == 7'b1101111) ? PCTargetE :    // jal
                             (ID_EX.op == 7'b1100111) ? PCJalr :       // jalr
                                                        alu_result;   // Default (ALU-type, branches)

  // Load inputs for the EX/MEM pipeline register.
  always_comb begin
    // Control signals passed to later stages
    EX_MEM_next.RegWrite   = ID_EX.RegWrite;
    EX_MEM_next.ResultSrc  = ID_EX.ResultSrc;
    EX_MEM_next.write_mem  = ID_EX.write_mem;

    // Data values calculated or selected in EX
    EX_MEM_next.ALUResult  = ALUResult_localEX;
    EX_MEM_next.write_data = StoreData;         // From forwarding/store logic
    EX_MEM_next.PCTarget   = PCTargetE;         // From branch target adder

    // Data and instruction fields passed through from ID/EX
    EX_MEM_next.Rd         = ID_EX.Rd;
    EX_MEM_next.PCPlus4    = ID_EX.PCPlus4;
    EX_MEM_next.ImmExt     = ID_EX.ImmExt;      // Passed for potential use in later stages
    EX_MEM_next.op         = ID_EX.op;
    EX_MEM_next.funct3     = ID_EX.funct3;
  end


/* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // MEMORY ACCESS (MEM) STAGE
  // Handles memory access for load/store instructions and prepares data for WB.
  //----------------------------------------------------------------------------

  // --- Data Memory Connections ---
  // The memory ports are driven by signals from the EX/MEM register.
  assign dmem_we           = EX_MEM.write_mem;   // Write-enable for store operations
  assign dmem_addr         = EX_MEM.ALUResult;   // Address is calculated by the ALU
  assign dmem_wd           = EX_MEM.write_data;  // Data to be stored comes from EX

  // --- Store Logic ---
  // Generates the byte-enable mask for store instructions (sb, sh, sw).
  logic [3:0]  ByteEnabler;
  assign ByteEnabler = (EX_MEM.write_mem) ?
                       ( (EX_MEM.funct3 == 3'b010) ? 4'b1111 :                                       // sw: all bytes enabled
                         (EX_MEM.funct3 == 3'b001 && EX_MEM.ALUResult[0] == 1'b0) ?
                                                  (4'b0011 << (EX_MEM.ALUResult[1] * 2)) :           // sh: two bytes, shifted by addr
                         (EX_MEM.funct3 == 3'b000) ? (4'b0001 << EX_MEM.ALUResult[1:0]) :             // sb: one byte, shifted by addr
                                                   4'b0000 ) :
                       4'bxxxx;
  assign dmem_byte_enabler = ByteEnabler;

  // --- Load Logic ---
  // Processes data read from memory to handle different load instruction types.
  logic [7:0]  ByteFromMem;
  logic [15:0] HalfwordFromMem;
  logic [31:0] LoadData;

  // Extract the correct byte from the 32-bit word read from data memory.
  assign ByteFromMem = (EX_MEM.ALUResult[1:0] == 2'b00) ? dmem_rd[7:0]   :
                       (EX_MEM.ALUResult[1:0] == 2'b01) ? dmem_rd[15:8]  :
                       (EX_MEM.ALUResult[1:0] == 2'b10) ? dmem_rd[23:16] :
                                                          dmem_rd[31:24];

  // Extract the correct halfword (for aligned accesses).
  assign HalfwordFromMem = (EX_MEM.ALUResult[1:0] == 2'b00) ? dmem_rd[15:0]  :
                           (EX_MEM.ALUResult[1:0] == 2'b10) ? dmem_rd[31:16] :
                                                              16'bx;

  // Select the final loaded value, performing sign or zero extension as required.
  assign LoadData = (EX_MEM.funct3 == 3'b010) ? dmem_rd :                                     // lw
                    (EX_MEM.funct3 == 3'b000) ? {{24{ByteFromMem[7]}},   ByteFromMem}   :     // lb (sign-extend)
                    (EX_MEM.funct3 == 3'b001) ? {{16{HalfwordFromMem[15]}}, HalfwordFromMem} : // lh (sign-extend)
                    (EX_MEM.funct3 == 3'b100) ? {{24{1'b0}},             ByteFromMem}   :     // lbu (zero-extend)
                    (EX_MEM.funct3 == 3'b101) ? {{16{1'b0}},             HalfwordFromMem} :     // lhu (zero-extend)
                                                dmem_rd;

  // --- MEM/WB Register Loading ---
  always_comb begin
      MEM_WB_next.RegWrite  = EX_MEM.RegWrite;
      MEM_WB_next.ResultSrc = EX_MEM.ResultSrc;
      MEM_WB_next.ALUResult = EX_MEM.ALUResult;
      MEM_WB_next.ReadData  = LoadData; // Use the processed data from the load logic
      MEM_WB_next.Rd        = EX_MEM.Rd;
      MEM_WB_next.PCPlus4   = EX_MEM.PCPlus4;
      MEM_WB_next.ImmExt    = EX_MEM.ImmExt;
      MEM_WB_next.PCTarget  = EX_MEM.PCTarget;
  end

/* ---------------------------------------------------------------------------- */

  //----------------------------------------------------------------------------
  // WRITE-BACK (WB) STAGE
  // Selects the final result to be written back into the register file.
  //----------------------------------------------------------------------------

  // This MUX selects the final result based on the ResultSrc control signal.
  // The output 'ResultW' is connected to the register file's write data port
  // and is also used for forwarding to earlier stages.
  assign ResultW = (MEM_WB.ResultSrc == 2'b00) ? MEM_WB.ALUResult : // Result from an ALU operation
                   (MEM_WB.ResultSrc == 2'b01) ? MEM_WB.ReadData  : // Data loaded from memory
                   (MEM_WB.ResultSrc == 2'b10) ? MEM_WB.PCPlus4   : // PC+4 for JAL/JALR return address
                                                 32'bx;             // Default to 'x' for safety
endmodule