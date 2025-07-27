//-----------------------------------------------------------------------------
// Module:  AluDecoder
// Author:  Gallo Andrea 2359271
//
// Description:
//   This unit generates the specific 4-bit control signal for the ALU.
//   It decodes the ALUOp from the main decoder, along with instruction
//   fields (funct3, funct7), to select the final ALU operation.
//-----------------------------------------------------------------------------

module alu_decoder (
    // Input Ports
    input  logic [6:0] i_op,         // Instruction opcode field
    input  logic [2:0] i_funct3,     // Instruction funct3 field
    input  logic       i_funct7b5,   // Bit 5 of funct7 field (differentiates ADD/SUB, SRL/SRA)
    input  logic [1:0] i_ALUOp,      // Pre-decoded operation type from main decoder

    // Output Port
    output logic [3:0] o_ALUControl  // 4-bit control signal for the ALU
);

    // --- Define Constants for Readability ---

    // ALUOp values from main decoder
    localparam ALUOP_LD_ST  = 2'b00; // Corresponds to Load/Store address calculation (ADD)
    localparam ALUOP_BRANCH = 2'b01; // Corresponds to Branch comparison (SUB)
    localparam ALUOP_R_I    = 2'b10; // Corresponds to R-Type and I-Type operations

    // Funct3 values for R-Type and I-Type instructions
    localparam F3_ADD_SUB = 3'b000;
    localparam F3_SLL     = 3'b001;
    localparam F3_SLT     = 3'b010;
    localparam F3_SLTU    = 3'b011;
    localparam F3_XOR     = 3'b100;
    localparam F3_SR      = 3'b101; // Covers SRL and SRA
    localparam F3_OR      = 3'b110;
    localparam F3_AND     = 3'b111;

    // Final 4-bit ALU control signals
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_AND  = 4'b0010;
    localparam ALU_OR   = 4'b0011;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SLT  = 4'b0101;
    localparam ALU_SLL  = 4'b0110;
    localparam ALU_SRL  = 4'b0111;
    localparam ALU_SLTU = 4'b1000;
    localparam ALU_SRA  = 4'b1001;

    // --- Combinational Decoding Logic ---
    always_comb begin
        // Set a default output to avoid latches.
        o_ALUControl = 4'bx;

        case (i_ALUOp)
            // For Loads/Stores, the ALU must perform an addition for address calculation.
            ALUOP_LD_ST: begin
                o_ALUControl = ALU_ADD;
            end

            // For Branches, the ALU must perform a subtraction for comparison.
            ALUOP_BRANCH: begin
                o_ALUControl = ALU_SUB;
            end

            // For R-Type and I-Type, we need to decode further based on funct3.
            ALUOP_R_I: begin
                case (i_funct3)
                    F3_ADD_SUB: begin
                        // Check for R-Type SUB (op[5]=1 for R-type) vs ADD/ADDI
                        if (i_op[5] && i_funct7b5)
                            o_ALUControl = ALU_SUB; // R-Type SUB
                        else
                            o_ALUControl = ALU_ADD; // R-Type ADD or I-Type ADDI
                    end
                    F3_SLL:  o_ALUControl = ALU_SLL;  // SLL, SLLI
                    F3_SLT:  o_ALUControl = ALU_SLT;  // SLT, SLTI
                    F3_SLTU: o_ALUControl = ALU_SLTU; // SLTU, SLTIU
                    F3_XOR:  o_ALUControl = ALU_XOR;  // XOR, XORI
                    F3_SR: begin
                        // Check for Arithmetic Shift (SRA/SRAI) vs Logical Shift (SRL/SRLI)
                        if (i_funct7b5)
                            o_ALUControl = ALU_SRA;
                        else
                            o_ALUControl = ALU_SRL;
                    end
                    F3_OR:   o_ALUControl = ALU_OR;   // OR, ORI
                    F3_AND:  o_ALUControl = ALU_AND;  // AND, ANDI
                    default: o_ALUControl = 4'bx;   // Undefined funct3
                endcase
            end

            // For any other ALUOp value, output remains 'x'.
            default: o_ALUControl = 4'bx;
        endcase
    end

endmodule