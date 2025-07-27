//-----------------------------------------------------------------------------
// Module:  ArithmeticLogicUnit (ALU)
// Author:  Gallo Andrea 2359271
//
// Description:
//   This module performs arithmetic and logical operations for the RISC-V
//   datapath. It takes two 32-bit operands and a control signal to select
//   the operation. It outputs the result and status flags (zero, sign,
//   overflow, carry).
//-----------------------------------------------------------------------------
module alu (
    // Input Ports
    input  logic [31:0] i_operand_a,
    input  logic [31:0] i_operand_b,
    input  logic [3:0]  i_alu_control,

    // Output Ports
    output logic [31:0] o_result,
    output logic        o_zero_flag,
    output logic        o_sign_flag,
    output logic        o_overflow_flag,
    output logic        o_carry_flag
);

    // -- Internal Constants for ALU Operations --
    // Using localparams makes the case statement self-documenting.
    localparam ALU_ADD  = 4'b0000; // ADD / ADDI
    localparam ALU_SUB  = 4'b0001; // SUB
    localparam ALU_AND  = 4'b0010; // AND / ANDI
    localparam ALU_OR   = 4'b0011; // OR / ORI
    localparam ALU_XOR  = 4'b0100; // XOR / XORI
    localparam ALU_SLT  = 4'b0101; // SLT / SLTI
    localparam ALU_SLL  = 4'b0110; // SLL / SLLI
    localparam ALU_SRL  = 4'b0111; // SRL / SRLI
    localparam ALU_SLTU = 4'b1000; // SLTU / SLTIU
    localparam ALU_SRA  = 4'b1001; // SRA / SRAI

    // -- Internal Wires and Logic --
    logic [31:0] b_operand;
    logic [31:0] sum;
    logic        is_add_sub;

    // -- Adder/Subtractor Core Logic --
    // This implements a combined adder/subtractor using a common trick:
    // ADD: result = A + B + 0
    // SUB: result = A + ~B + 1  (which is A - B in two's complement)
    // The control signal's LSB determines if B is inverted and if 1 is added.
    assign b_operand = (i_alu_control[0]) ? ~i_operand_b : i_operand_b;
    assign sum       = i_operand_a + b_operand + i_alu_control[0];

    // Determine the shift amount from the lower 5 bits of operand B.
    wire [4:0] shift_amount = i_operand_b[4:0];

    // -- Main ALU Result Multiplexer --
    // Selects the final result based on the ALU control signal.
    always_comb begin
        case (i_alu_control)
            ALU_ADD,
            ALU_SUB:  o_result = sum;                   // Result for ADD, SUB is the sum
            ALU_AND:  o_result = i_operand_a & i_operand_b; // Bitwise AND
            ALU_OR:   o_result = i_operand_a | i_operand_b; // Bitwise OR
            ALU_XOR:  o_result = i_operand_a ^ i_operand_b; // Bitwise XOR
            ALU_SLL:  o_result = i_operand_a << shift_amount; // Shift Left Logical
            ALU_SRL:  o_result = i_operand_a >> shift_amount; // Shift Right Logical
            ALU_SRA:  o_result = $signed(i_operand_a) >>> shift_amount; // Shift Right Arithmetic

            // Set on Less Than (signed)
            ALU_SLT:  o_result = ($signed(i_operand_a) < $signed(i_operand_b)) ? 32'd1 : 32'd0;

            // Set on Less Than (unsigned)
            ALU_SLTU: o_result = (i_operand_a < i_operand_b) ? 32'd1 : 32'd0;

            default: o_result = 32'bx; // Default to 'x' for unimplemented operations
        endcase
    end

    // -- Status Flag Generation --
    // These flags are often used for branch condition evaluation.

    // Zero flag is asserted if the result of the main operation is exactly zero.
    assign o_zero_flag = (o_result == 32'b0);

    // Sign flag is simply the most significant bit of the result.
    assign o_sign_flag = o_result[31];
    
    // Check if the current operation is an ADD or SUB, as overflow/carry
    // are only meaningful for these. This is much clearer than the original boolean expression.
    assign is_add_sub = (i_alu_control == ALU_ADD) || (i_alu_control == ALU_SUB);

    // Signed overflow occurs if the sign of the result is incorrect.
    // Condition: (sign(A) == sign(B)) and (sign(A) != sign(Result))
    assign o_overflow_flag = (i_operand_a[31] == b_operand[31]) && (i_operand_a[31] != sum[31]) && is_add_sub;

    // Unsigned carry-out is detected by extending operands by one bit.
    logic [32:0] extended_sum;
    assign extended_sum = {1'b0, i_operand_a} + {1'b0, b_operand} + i_alu_control[0];
    assign o_carry_flag = extended_sum[32] && is_add_sub;

endmodule