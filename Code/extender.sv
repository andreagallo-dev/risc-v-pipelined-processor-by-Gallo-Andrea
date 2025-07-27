//-----------------------------------------------------------------------------
// Module:  Extender
// Author:  Gallo Andrea 2359271
//
// Description:
//   This module generates a 32-bit sign-extended immediate value from the
//   32-bit instruction word. It uses a selector signal (i_imm_src) from the
//   Instruction Decoder to determine which immediate format (I, S, B, U, J)
//   to extract and extend. This is a purely combinational circuit.
//-----------------------------------------------------------------------------
module extender(
    // Input Ports
    input  logic [31:0] i_instruction, // The 32-bit RISC-V instruction
    input  logic [2:0]  i_imm_src,     // Selector from the decoder to specify the immediate format
    
    // Output Port
    output logic [31:0] o_immediate    // The 32-bit sign-extended immediate value
);

    // This combinational block generates the immediate value.
    // The case statement selects the correct format based on the control signal.
    always_comb begin
        // Initialize the output to a default debug value. This prevents latches.
        // Using a distinct value like 0xDEADBEEF makes it easy to spot in simulations
        // if an unsupported 'i_imm_src' value is ever provided.
        o_immediate = 32'hDEADBEEF;

        // Select the appropriate immediate format based on the control signal.
        case (i_imm_src)
            // Case 0: I-type immediate (used for loads, I-type arithmetic, and jalr)
            3'b000: begin
                // Extract the 12-bit immediate from instr[31:20] and sign-extend it
                // by replicating the most significant bit (instr[31]).
                o_immediate = {{20{i_instruction[31]}}, i_instruction[31:20]};
            end

            // Case 1: S-type immediate (used for store instructions)
            3'b001: begin
                // Reassemble the 12-bit immediate from two separate fields and sign-extend.
                o_immediate = {{20{i_instruction[31]}}, i_instruction[31:25], i_instruction[11:7]};
            end
            
            // Case 2: B-type immediate (used for branch instructions)
            3'b010: begin
                // Reassemble the 13-bit immediate for branches. The LSB is an implicit 0,
                // as branch targets are 2-byte aligned. The result is sign-extended.
                o_immediate = {{20{i_instruction[31]}}, i_instruction[7], i_instruction[30:25], i_instruction[11:8], 1'b0};
            end

            // Case 3: J-type immediate (used for JAL)
            3'b011: begin
                // Reassemble the 21-bit immediate for jumps. The LSB is an implicit 0.
                // The result is sign-extended.
                o_immediate = {{12{i_instruction[31]}}, i_instruction[19:12], i_instruction[20], i_instruction[30:21], 1'b0};
            end

            // Case 4: U-type immediate (used for LUI and AUIPC)
            3'b100: begin
                // Extract the 20 most significant bits of the immediate and place them
                // in the upper 20 bits of the result. The lower 12 bits are zeroed.
                // This format does not require sign extension.
                o_immediate = {i_instruction[31:12], 12'b0};
            end

            // If i_imm_src is not one of the recognized values, the output remains 0xDEADBEEF.
            // For a final synthesis implementation, it's often better to assign 'x' here
            // (o_immediate = 32'bx;) to allow the synthesis tool to optimize the logic
            // as a "don't care" condition.
            default: o_immediate = 32'hDEADBEEF;
            
        endcase
    end

endmodule