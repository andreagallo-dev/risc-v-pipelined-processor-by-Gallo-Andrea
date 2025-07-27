//-----------------------------------------------------------------------------
// Module:  InstructionMemory
// Author:  Gallo Andrea 2359271
//
// Description:
//   A model for a read-only instruction memory (ROM). It features a
//   combinational (asynchronous) read port. The memory content can be
//   initialized from a hex file at the start of a simulation.
//-----------------------------------------------------------------------------
module instruction_mem #(
    // Parameters for configuration
    parameter int    MEM_WORDS = 128,                          // Total number of 32-bit words in the memory
    parameter string INIT_FILE = "Code/instructions.txt"       // Path to the memory initialization file
)(
    // Input Ports
    input  logic [31:0]            i_addr,        // 32-bit byte address from the Program Counter

    // Output Ports
    output logic [31:0]            o_instruction  // 32-bit instruction read from memory
);

    // Calculate the required address width based on the memory size.
    localparam int ADDR_WIDTH = $clog2(MEM_WORDS);

    // The memory array itself, modeled as a packed array of logic.
    logic [31:0] mem [MEM_WORDS];

    // --- Memory Initialization (for simulation) ---
    // This block is executed once at the beginning of the simulation (time 0).
    // It only executes if an initialization file path is provided.
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end
    end

    // --- Combinational Read Logic ---
    
    wire [ADDR_WIDTH-1:0] word_addr = i_addr[ADDR_WIDTH+1:2];

    // Asynchronous read: the output is continuously assigned the value
    // from the memory location pointed to by the word address.
    assign o_instruction = mem[word_addr];

endmodule