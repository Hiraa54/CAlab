module single_cycle(
    input logic clock, reset
);
    // Program counter
    logic [31:0] pc, next_pc;

    // instruction memory
    logic [31:0] ins;

    // Register file
    logic [6:0] operation_code;
    logic [4:0] source_reg1, source_reg2, dest_reg;
    logic [31:0] write_data, read_data1, read_data2;

    // ALU
    logic [3:0] alu_control;
    logic [31:0] alu_output;

    // Data memory
    logic [31:0] read_data;

    // Branch condition
    logic [2:0] branch_type;

    // Control unit
    logic [1:0] writeback_select;
    logic reg_wr, read_en, wr_en, br_taken, sel_A, sel_B;

    // Immediate Generator
    logic [31:0] imm_gen;

    // Instantiate modules with renamed variables
    pc pc_module (
        clock, reset, br_taken ? alu_output : next_pc, pc
    );
    ins_memory ins_memory_module (
        pc, ins
    );
    register_file register_file_module (
        clock, reg_wr, source_reg1, source_reg2, dest_reg, write_data, read_data1, read_data2
    );
    arithmetic_logic_unit alu_module (
        alu_control, sel_A ? read_data1 : pc, sel_B ? imm_gen : read_data2, alu_output
    );
    data_memory data_memory_module (
        clock, wr_en, read_en, alu_output, read_data2, read_data
    );
    branch_condition branch_condition_module (
        branch_type, read_data1, read_data2, br_taken
    );
    control_unit control_unit_module (
        ins, reg_wr, read_en, wr_en, sel_A, sel_B, writeback_select, alu_control, branch_type
    );
    immediate_generator immediate_generator_module (
        ins, imm_gen
    );

    // Assign ins fields
    assign operation_code = ins[6:0];
    assign source_reg1 = ins[19:15];
    assign source_reg2 = ins[24:20];
    assign dest_reg = ins[11:7];
    assign next_pc = br_taken ? alu_output : pc + 4;

    // Writeback logic
    always_comb begin
        case (writeback_select)
            2'b00: write_data = pc + 4;
            2'b01: write_data = alu_output;
            2'b10: write_data = read_data;
        endcase
    end

endmodule

module pc(
    input logic clock, reset,
    input logic [31:0] next_pc,
    output logic [31:0] pc
);
    always_ff @(posedge clock) begin
        if (reset)
            pc <= 32'b0;
        else
            pc <= next_pc;
    end
endmodule

module ins_memory(
    input logic [31:0] address,
    output logic [31:0] ins
);
    logic [31:0] memory [0:1023];
    assign ins = memory[address[11:2]];
endmodule

module register_file(
    input logic clock, reg_wr,
    input logic [4:0] source_reg1, source_reg2, dest_reg,
    input logic [31:0] write_data,
    output logic [31:0] read_data1, read_data2
);
    logic [31:0] registers [0:31];

    always_ff @(negedge clock) begin
        registers[0] <= 32'b0;
        if (reg_wr)
            registers[dest_reg] <= write_data;
    end

    assign read_data1 = registers[source_reg1];
    assign read_data2 = registers[source_reg2];
endmodule

module arithmetic_logic_unit(
    input logic [3:0] alu_control,
    input logic [31:0] operand_A, operand_B,
    output logic [31:0] result
);
    always_comb begin
        case (alu_control)
            4'b0000: result = operand_A + operand_B; 
            4'b0001: result = operand_A - operand_B; 
            4'b0010: result = operand_A << operand_B[4:0]; 
            4'b0011: result = operand_A >> operand_B[4:0]; 
            4'b0100: result = $signed(operand_A) >>> operand_B[4:0];
            4'b0101: result = ($signed(operand_A)) < ($signed(operand_B)) ? 1 : 0; 
            4'b0111: result = operand_A ^ operand_B;
            4'b1000: result = operand_A | operand_B; 
            4'b1001: result = operand_A & operand_B; 
            4'b1010: result = operand_B;
            default: result = 32'b0;
        endcase
    end
endmodule

module data_memory(
    input logic clock, wr_en, read_en,
    input logic [31:0] address, write_data,
    output logic [31:0] read_data
);
    logic [31:0] memory [0:1023];

    always_ff @(negedge clock) begin
        if (wr_en)
            memory[address] <= write_data;
    end

    assign read_data = (read_en) ? memory[address] : 32'b0;
endmodule

module branch_condition(
    input logic [2:0] branch_type,
    input logic [31:0] operand_A, operand_B,
    output logic br_taken
);
    always_comb begin
        case (branch_type)
            3'b000: br_taken = 0;
            3'b001: br_taken = (operand_A == operand_B);  // BEQ
            3'b010: br_taken = (operand_A != operand_B);  // BNE
            3'b011: br_taken = ($signed(operand_A)) < $signed((operand_B));  // BLT
            3'b100: br_taken = ($signed(operand_A)) >= $signed((operand_B)); // BGE
            3'b101: br_taken = (operand_A < operand_B);   // BLTU
            3'b110: br_taken = (operand_A >= operand_B);  // BGEU
            3'b111: br_taken = 1;                   // Unconditional Jump
            default: br_taken = 0;                  // Default case
        endcase
    end
endmodule

module control_unit(
    input logic [31:0] ins,
    output logic reg_wr, read_en, wr_en, sel_A, sel_B,
    output logic [1:0] writeback_select,
    output logic [3:0] alu_control,
    output logic [2:0] branch_type
);
    logic [4:0] source_reg1, source_reg2, dest_reg;
    logic [6:0] operation_code, function_7;
    logic [2:0] function_3;

    assign operation_code = ins[6:0];
    assign dest_reg = ins[11:7];
    assign function_3 = ins[14:12];
    assign source_reg1 = ins[19:15];
    assign source_reg2 = ins[24:20];
    assign function_7 = ins[31:25];

    always_comb begin
        // R-Type
        if (operation_code == 7'b0110011) begin
         reg_wr = 1;
            read_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 0;
            writeback_select = 2'b01;
            branch_type = 3'b000;
            if (function_3 == 3'b000 && function_7 == 7'b0000000) // ADD
                alu_control = 4'b0000;
            else if (function_3 == 3'b000 && function_7 == 7'b0100000) // SUB
                alu_control = 4'b0001;
            else if (function_3 == 3'b001 && function_7 == 7'b0000000) // SLL
                alu_control = 4'b0010;
            else if (function_3 == 3'b101 && function_7 == 7'b0000000) // SRL
                alu_control = 4'b0011;
            else if (function_3 == 3'b101 && function_7 == 7'b0100000) // SRA
                alu_control = 4'b0100;
            else if (function_3 == 3'b010 && function_7 == 7'b0000000) // SLT
                alu_control = 4'b0101;
            else if (function_3 == 3'b011 && function_7 == 7'b0000000) // SLTU
                alu_control = 4'b0110;
            else if (function_3 == 3'b100 && function_7 == 7'b0000000) // XOR
                alu_control = 4'b0111;            
            else if (function_3 == 3'b110 && function_7 == 7'b0000000) // OR
                alu_control = 4'b1000;
            else if (function_3 == 3'b111 && function_7 == 7'b0000000) // AND
                alu_control = 4'b1001;
        end

        // I-Type
        else if (operation_code == 7'b0010011) begin
         reg_wr = 1;
            read_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            writeback_select = 2'b01;
            branch_type = 3'b000;
            if (function_3 == 3'b000) // ADDI
                alu_control = 4'b0000;
            else if (function_3 == 3'b001) // SLLI
                alu_control = 4'b0010;
            else if (function_3 == 3'b101 && function_7 == 7'b0000000) //SRLI
                alu_control = 4'b0011;
            else if (function_3 == 3'b101 && function_7 == 7'b0100000) //SRAI
                alu_control = 4'b0100;
            else if (function_3 == 3'b010) // SLTI
                alu_control = 4'b0101;
            else if (function_3 == 3'b011) // SLTIU
                alu_control = 4'b0110;
            else if (function_3 == 3'b100) // XOR
                alu_control = 4'b0111;
            else if (function_3 == 3'b110) // OR
                alu_control = 4'b1000;
            else if (function_3 == 3'b111) // AND
                alu_control = 4'b1001;
        end

        // I-Type (Load)
        else if (operation_code == 7'b0000011) begin
         reg_wr = 1;
            read_en = 1;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            writeback_select = 2'b11;
            branch_type = 3'b000;
            alu_control = 4'b0000;
        end

        // S-Type   
        else if (operation_code == 7'b0100011) begin
         reg_wr = 0;
            read_en = 0;
            wr_en = 1;
            sel_A = 1;
            sel_B = 1;
            writeback_select = 2'b01;
            branch_type = 3'b000;
            alu_control = 4'b0000;
        end

        // B-Type   
        else if (operation_code == 7'b1100011) begin
         reg_wr = 0;
            read_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            writeback_select = 2'b01;
            alu_control = 4'b0000;

            if (function_3 == 3'b000) // BEQ
                branch_type = 3'b001;
            else if (function_3 == 3'b001) // BNE
                branch_type = 3'b010;
            else if (function_3 == 3'b100) // BLT
                branch_type = 3'b011;
            else if (function_3 == 3'b101) // BGE
                branch_type = 3'b100;
            else if (function_3 == 3'b110) // BLTU
                branch_type = 3'b101;
            else if (function_3 == 3'b111) // BGEU
                branch_type = 3'b110;
        end

        // U-Type   
        else if (operation_code == 7'b0110111) begin // LUI
         reg_wr = 1;
            read_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            writeback_select = 2'b01;
            branch_type = 3'b000;
            alu_control = 4'b1010;
        end
        else if (operation_code == 7'b0010111) begin // AUIPC
         reg_wr = 1;
            read_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            writeback_select = 2'b01;
            branch_type = 3'b000;
            alu_control = 4'b0000;
        end

        // J-Type
        else if (operation_code == 7'b1101111) begin // JAL
         reg_wr = 1;
            read_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            writeback_select = 2'b00;
            branch_type = 3'b111;
            alu_control = 4'b0000;
        end
        else if (operation_code == 7'b1100111) begin //JALR
         reg_wr = 1;
            read_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            writeback_select = 2'b00;
            branch_type = 3'b111;
            alu_control = 4'b0000;
        end
    end
endmodule

module immediate_generator(
    input logic [31:0] ins,
    output logic [31:0] imm_gen
);
    logic [6:0] operation_code;
    assign operation_code = ins[6:0];

    always_comb begin
        if (operation_code == 7'b0100011) 
            imm_gen = {{20{ins[31]}}, ins[31:25], ins[11:7]};
        else if (operation_code == 7'b1100011) 
            imm_gen = {{19{ins[31]}}, ins[31], ins[7], ins[30:25], ins[11:8], 1'b0};
        else if (operation_code == 7'b0110111 || operation_code == 7'b0010111)  
            imm_gen = {ins[31:12], 12'b0};
        else if (operation_code == 7'b1101111) 
            imm_gen = {{11{ins[31]}}, ins[31], ins[19:12], ins[20], ins[30:21], 1'b0};
        else
            imm_gen = {{20{ins[31]}}, ins[31:20]};
    end
endmodule