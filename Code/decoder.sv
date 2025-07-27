//-----------------------------------------------------------------------------
// Module:  Decoder
// Author:  Gallo Andrea 2359271
//
// Description:
//   This module decodes the opcode field of a RISC-V instruction to generate
//   the main control signals for the pipelined datapath. It is a purely
//   combinational circuit that maps each opcode to a specific set of control
//   values.
//-----------------------------------------------------------------------------
module decoder (
    // Input Port
    input  logic [6:0] i_opcode,

    // Output Ports - Control Signals
    output logic [1:0] o_ResultSrc, // Selects the source for the Write-Back stage (ALU result, Mem data, PC+4)
    output logic       o_MemWrite,  // Enables writing to data memory (for store instructions)
    output logic       o_Branch,    // Indicates a conditional branch instruction
    output logic       o_ALUSrc,    // Selects the second ALU operand (Register or Immediate)
    output logic       o_RegWrite,  // Enables writing to the register file
    output logic       o_Jump,      // Indicates an unconditional jump (JAL, JALR)
    output logic [2:0] o_ImmSrc,    // Selects the type of immediate extension (I, S, B, U, J)
    output logic [1:0] o_ALUOp      // Control signal for the ALU Control unit
);

    // -- Internal Constants for Readability --

    // RISC-V Opcode constants for RV32I instruction set
    localparam OP_LOAD   = 7'b0000011; // lw
    localparam OP_STORE  = 7'b0100011; // sw, sb, sh
    localparam OP_R_TYPE = 7'b0110011; // add, sub, and, or, etc.
    localparam OP_BRANCH = 7'b1100011; // beq, bne, etc.
    localparam OP_I_TYPE = 7'b0010011; // addi, slti, etc.
    localparam OP_JAL    = 7'b1101111; // jal
    localparam OP_JALR   = 7'b1100111; // jalr
    localparam OP_LUI    = 7'b0110111; // lui
    localparam OP_AUIPC  = 7'b0010111; // auipc

    // Constants for o_ResultSrc MUX selector
    localparam RES_SRC_ALU = 2'b00;
    localparam RES_SRC_MEM = 2'b01;
    localparam RES_SRC_PC4 = 2'b10;

    // Constants for o_ImmSrc MUX selector (Immediate type)
    localparam IMM_I = 3'b000;
    localparam IMM_S = 3'b001;
    localparam IMM_B = 3'b010;
    localparam IMM_J = 3'b011;
    localparam IMM_U = 3'b100;

    // Constants for o_ALUOp signal (used by ALU Control)
    localparam ALUOP_LD_ST = 2'b00; // For Load/Store address calculation (ADD)
    localparam ALUOP_BRANCH= 2'b01; // For Branch comparison (SUB)
    localparam ALUOP_R_I   = 2'b10; // For R-Type and I-Type arithmetic/logic operations

    // -- Main Combinational Decoding Logic --
    always_comb begin
        // 1. Set default values for all control signals.
        o_RegWrite  = 1'b0;
        o_MemWrite  = 1'b0;
        o_Branch    = 1'b0;
        o_Jump      = 1'b0;
        o_ALUSrc    = 1'b0;         // Default to Reg-Reg operation
        o_ResultSrc = RES_SRC_ALU;  // Default to ALU result
        o_ALUOp     = 'x;           // Don't care by default, as it's not used
        o_ImmSrc    = 'x;           // Don't care by default

        // 2. Decode the instruction based on its opcode.
        case (i_opcode)
            OP_LOAD: begin
                // Load Word (lw): Reg[rd] = Mem[Reg[rs1] + imm]
                o_ALUSrc    = 1'b1;         // ALU operand is immediate
                o_RegWrite  = 1'b1;         // Write result back to register file
                o_ResultSrc = RES_SRC_MEM;  // Result comes from data memory
                o_ImmSrc    = IMM_I;        // Use I-type immediate
                o_ALUOp     = ALUOP_LD_ST;  // ALU performs ADD for address calculation
            end

            OP_STORE: begin
                // Store Word (sw): Mem[Reg[rs1] + imm] = Reg[rs2]
                o_ALUSrc    = 1'b1;         // ALU operand is immediate
                o_MemWrite  = 1'b1;         // Enable memory write
                o_ImmSrc    = IMM_S;        // Use S-type immediate
                o_ALUOp     = ALUOP_LD_ST;  // ALU performs ADD for address calculation
            end

            OP_R_TYPE: begin
                // Register-Register (add, sub): Reg[rd] = Reg[rs1] op Reg[rs2]
                o_RegWrite  = 1'b1;         // Write result back to register file
                // ALUSrc is 0 (default), result source is ALU (default)
                o_ALUOp     = ALUOP_R_I;    // ALU performs operation specified by funct3/funct7
            end

            OP_BRANCH: begin
                // Conditional Branch (beq, bne): if (Reg[rs1] == Reg[rs2]) PC += imm
                o_Branch    = 1'b1;         // This is a branch instruction
                // ALUSrc is 0 (default) for Reg-Reg comparison
                o_ImmSrc    = IMM_B;        // Use B-type immediate for target address
                o_ALUOp     = ALUOP_BRANCH; // ALU performs SUB for comparison
            end

            OP_I_TYPE: begin
                // Register-Immediate (addi): Reg[rd] = Reg[rs1] + imm
                o_RegWrite  = 1'b1;         // Write result back to register file
                o_ALUSrc    = 1'b1;         // ALU operand is immediate
                o_ImmSrc    = IMM_I;        // Use I-type immediate
                o_ALUOp     = ALUOP_R_I;    // ALU performs operation specified by funct3
            end

            OP_JAL: begin
                // Jump And Link: Reg[rd] = PC + 4; PC = PC + imm
                o_RegWrite  = 1'b1;         // Write PC+4 to register file
                o_Jump      = 1'b1;         // This is a jump instruction
                o_ResultSrc = RES_SRC_PC4;  // Result is PC+4
                o_ImmSrc    = IMM_J;        // Use J-type immediate for target address
            end

            OP_JALR: begin
                // Jump And Link Register: Reg[rd] = PC + 4; PC = Reg[rs1] + imm
                o_RegWrite  = 1'b1;         // Write PC+4 to register file
                o_Jump      = 1'b1;         // This is a jump instruction
                o_ALUSrc    = 1'b1;         // ALU operand is immediate
                o_ResultSrc = RES_SRC_PC4;  // Result is PC+4
                o_ImmSrc    = IMM_I;        // Use I-type immediate for target address
                o_ALUOp     = ALUOP_R_I;    // ALU performs ADD
            end

            OP_LUI: begin
                // Load Upper Immediate: Reg[rd] = imm << 12
                o_RegWrite  = 1'b1;         // Write result to register file
                o_ALUSrc    = 1'b1;         // The immediate is the operand
                o_ImmSrc    = IMM_U;        // Use U-type immediate
                o_ALUOp     = ALUOP_R_I;    // The ALU will just pass the immediate through
            end

            OP_AUIPC: begin
                // Add Upper Immediate to PC: Reg[rd] = PC + (imm << 12)
                o_RegWrite  = 1'b1;         // Write result to register file
                o_ALUSrc    = 1'b1;         // The immediate is an operand
                o_ImmSrc    = IMM_U;        // Use U-type immediate
                o_ALUOp     = ALUOP_R_I;    // The ALU will perform an ADD with PC
            end

            default: begin
                // For any non-implemented or invalid opcodes
                o_RegWrite  = 1'b0;
                o_MemWrite  = 1'b0;
                o_Branch    = 1'b0;
                o_Jump      = 1'b0;
                o_ALUSrc    = 'x;
                o_ResultSrc = 'x;
                o_ALUOp     = 'x;
                o_ImmSrc    = 'x;
            end
        endcase
    end
endmodule