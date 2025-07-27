//-----------------------------------------------------------------------------
// Module:  Adder
// Author:  Gallo Andrea 2359271
//
// Description:
//   A generic, N-bit combinational adder. The data width can be configured
//   via the DATA_WIDTH parameter.
//-----------------------------------------------------------------------------
module adder #(
    // Parameter to define the bit-width of the operands and the result.
    parameter int DATA_WIDTH = 32
)(
    // Input operands
    input  logic [DATA_WIDTH-1:0] i_a,
    input  logic [DATA_WIDTH-1:0] i_b,

    // Output sum
    output logic [DATA_WIDTH-1:0] o_sum
);

    // The addition is purely combinational logic.
    // The sum is calculated whenever any of the inputs (i_a or i_b) change.
    always_comb begin
        o_sum = i_a + i_b;
    end

endmodule