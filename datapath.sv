module datapath(
    input logic clk, reset
);
    logic [31:0] pc, next_pc, extended_imm, instruction,read_data1, read_data2, alu_result, read_data, write_data;
    logic [4:0] rs1, rs2, rd;
    logic [6:0] opcode;
    logic [3:0] alu_ctrl;
    logic [2:0] br_type;
    logic [1:0] wb_sel;
    logic reg_wr, rd_en, wr_en, br_taken, sel_A, sel_B;

    // Instantiate modules
    pc pc_0 (clk, reset, br_taken ? alu_result : next_pc, pc);
    inst_mem inst_mem_0 (pc, instruction);
    reg_file reg_file_0 (clk, reg_wr, rs1, rs2, rd, write_data,read_data1, read_data2);
    alu alu_0 (alu_ctrl, sel_A ?read_data1 : pc, sel_B ? extended_imm : read_data2, alu_result);
    data_mem data_mem_0 (clk, wr_en, rd_en, alu_result, read_data2, read_data);
    branch_cond branch_cond_0 (br_type,read_data1, read_data2, br_taken);
    control_unit control_unit_0 (instruction, reg_wr, rd_en, wr_en, sel_A, sel_B, wb_sel, alu_ctrl, br_type);
    imd_generator imd_generator_0 (instruction, extended_imm);

    // Assignments
    assign opcode = instruction[6:0];
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign rd = instruction[11:7];
    assign next_pc = br_taken ? alu_result : pc + 4;

    always_comb begin
         case (wb_sel)
            2'b00: write_data = pc + 4;
            2'b01: write_data = alu_result;
            2'b10: write_data = read_data;
        endcase
    end

endmodule

module pc(
	input logic clk, reset,
	input logic [31:0] next_pc,
	output logic [31:0] pc
);
	
	always_ff @(posedge clk) begin

		if (reset)
			pc <= 32'b0;
		else
			pc <= next_pc;
	end

endmodule

module inst_mem(
	input logic [31:0] address,
	output logic [31:0] instruction
);
	
	logic [31:0] memory [0:1023];

	assign instruction = memory[address[11:2]];

endmodule

module reg_file(
	input logic clk, reg_write,
	input logic [4:0] rs1, rs2, rd, 
	input logic [31:0] write_data,
	output logic [31:0]  read_data1,read_data2
);

	logic [31:0] registers [0:31];

	always_ff @(negedge clk) begin
		registers[0] <= 32'b0;
		if (reg_write)
			registers[rd] <= write_data;
	end

	assign read_data1 = registers[rs1];
	assign read_data2 = registers[rs2];

endmodule

module alu(
	input logic [3:0] alu_op,
	input logic [31:0] A, B,
	output logic [31:0] C
);

	always_comb begin
        case (alu_op)
            4'b0000: C = A + B; // ADD
            4'b0001: C = A - B; // SUB
            4'b0010: C = A << B[4:0]; // SLL
            4'b0011: C = A >> B[4:0]; // SRL
            4'b0100: C = $signed(A) >>> B[4:0]; // SRA
            4'b0101: C = ($signed(A) < $signed(B)) ? 1 : 0; // SLT
            4'b0110: C = (A < B) ? 1 : 0; // SLTU
            4'b0111: C = A ^ B; // XOR
            4'b1000: C = A | B; // OR
            4'b1001: C = A & B; // AND
            4'b1010: C = B; // Just Pass B
        
            default: C = 32'b0;
        endcase
    end

endmodule

module data_mem(
	input logic clk, wr_en, rd_en,
	input logic [31:0] addr, write_data,
	output logic [31:0] read_data
);
	
	logic [31:0] memory [0:1023];
	
	always_ff @(negedge clk) begin
		if (wr_en)
			memory[addr] <= write_data;
	end

	assign read_data = (rd_en) ? memory[addr] : 32'b0;

endmodule

module branch_cond (
    input logic [2:0] br_type,
    input logic [31:0]read_data1, read_data2,
    output logic br_taken
);

    always_comb begin
        case (br_type)
            3'b000: br_taken = 0;
            3'b001: br_taken = (read_data1 == read_data2);  // BEQ
            3'b010: br_taken = (read_data1 != read_data2);  // BNE
            3'b011: br_taken = ($signedread_data1) < $signed((read_data2));  // BLT
            3'b100: br_taken = ($signedread_data1) >= $signed((read_data2)); // BGE
            3'b101: br_taken = (read_data1 < read_data2);   // BLTU
            3'b110: br_taken = (read_data1 >= read_data2);  // BGEU
            3'b111: br_taken = 1;                   // Unconditional Jump
            default: br_taken = 0;                  // Default case
        endcase
    end

endmodule

module control_unit (
    input logic [31:0] inst,
    output logic reg_wr, rd_en, wr_en, sel_A, sel_B,
    output logic [1:0] wb_sel,
    output logic [3:0] alu_op,
    output logic [2:0] br_type
);
    logic [4:0] rs1, rs2, rd;
    logic [6:0] opcode, func7;
    logic [2:0] func3;

    assign opcode = inst[6:0];
    assign rd = inst[11:7];
    assign func3 = inst[14:12];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
    assign func7 = inst[31:25];


    always_comb begin

        // R-Type
        if (opcode == 7'b0110011) begin
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 0;
            wb_sel = 2'b01;
            br_type = 3'b000;
            if (func3 == 3'b000 && func7 == 7'b0000000) // ADD
                alu_op = 4'b0000;
            else if (func3 == 3'b000 && func7 == 7'b0100000) // SUB
                alu_op = 4'b0001;
            else if (func3 == 3'b001 && func7 == 7'b0000000) // SLL
                alu_op = 4'b0010;
            else if (func3 == 3'b101 && func7 == 7'b0000000) // SRL
                alu_op = 4'b0011;
            else if (func3 == 3'b101 && func7 == 7'b0100000) // SRA
                alu_op = 4'b0100;
            else if (func3 == 3'b010 && func7 == 7'b0000000) // SLT
                alu_op = 4'b0101;
            else if (func3 == 3'b011 && func7 == 7'b0000000) // SLTU
                alu_op = 4'b0110;
            else if (func3 == 3'b100 && func7 == 7'b0000000) // XOR
                alu_op = 4'b0111;            
            else if (func3 == 3'b110 && func7 == 7'b0000000) // OR
                alu_op = 4'b1000;
            else if (func3 == 3'b111 && func7 == 7'b0000000) // AND
                alu_op = 4'b1001;
        end

        // I-Type
        else if (opcode == 7'b0010011) begin
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            if (func3 == 3'b000) // ADDI
                alu_op = 4'b0000;
            else if (func3 == 3'b001) // SLLI
                alu_op = 4'b0010;
            else if (func3 == 3'b101 && func7 == 7'b0000000) //SRLI
                alu_op = 4'b0011;
            else if (func3 == 3'b101 && func7 == 7'b0100000) //SRAI
                alu_op = 4'b0100;
            else if (func3 == 3'b010) // SLTI
                alu_op = 4'b0101;
            else if (func3 == 3'b011) // SLTIU
                alu_op = 4'b0110;
            else if (func3 == 3'b100) // XOR
                alu_op = 4'b0111;
            else if (func3 == 3'b110) // OR
                alu_op = 4'b1000;
            else if (func3 == 3'b111) // AND
                alu_op = 4'b1001;
        end

        // I-Type (Load)
        else if (opcode == 7'b0000011) begin
            reg_wr = 1;
            rd_en = 1;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b11;
            br_type = 3'b000;
            alu_op = 4'b0000;
        end

        // S-Type   
        else if (opcode == 7'b0100011) begin
            reg_wr = 0;
            rd_en = 0;
            wr_en = 1;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            alu_op = 4'b0000;
        end

        // B-Type   
        else if (opcode == 7'b1100011) begin
            reg_wr = 0;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b01;
            alu_op = 4'b0000;

            if (func3 == 3'b000) // BEQ
                br_type = 3'b001;
            else if (func3 == 3'b001) // BNE
                br_type = 3'b010;
            else if (func3 == 3'b100) // BLT
                br_type = 3'b011;
            else if (func3 == 3'b101) // BGE
                br_type = 3'b100;
            else if (func3 == 3'b110) // BLTU
                br_type = 3'b101;
            else if (func3 == 3'b111) // BGEU
                br_type = 3'b110;
        end

        // U-Type   
        else if (opcode == 7'b0110111) begin // LUI
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            alu_op = 4'b1010;
        end
        else if (opcode == 7'b0010111) begin // AUIPC
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            alu_op = 4'b0000;
        end

        // J-Type
        else if (opcode == 7'b1101111) begin // JAL
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b00;
            br_type = 3'b111;
            alu_op = 4'b0000;
        end
        else if (opcode == 7'b1100111) begin // JALR
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b00;
            br_type = 3'b111;
            alu_op = 4'b0000;
        end

    end
endmodule

module imd_generator (
    input logic [31:0] instruction,
    output logic [31:0] extended_imm
);
    logic [6:0] opcode;
    assign opcode = instruction[6:0];

    always_comb begin

        if (opcode == 7'b0100011) // S-Type
            extended_imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        else if (opcode == 7'b1100011) // B-Type
            extended_imm = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        else if (opcode == 7'b0110111 || opcode == 7'b0010111) // U-Type 
            extended_imm = {instruction[31:12], 12'b0};
        else if (opcode == 7'b1101111) // J-Type
            extended_imm = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        else
            extended_imm = {{20{instruction[31]}}, instruction[31:20]};

    end

endmodule