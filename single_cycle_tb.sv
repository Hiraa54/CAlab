module single_cycle_tb;

    logic clk;
    logic reset;

    single_cycle dut (
        .clock(clk),
        .reset(reset)
    );

    always begin
        #5 clk = ~clk;
    end

    always_ff @(negedge clk) begin
        $display("REG[1] = %h", dut.register_file_module.registers[1]);
    end

    initial begin
        clk = 0;
        reset = 1;

        dut.register_file_module.registers[1] =  32'd0;
        dut.register_file_module.registers[2] =  32'd100;
        dut.register_file_module.registers[3] =  32'd200;
        dut.register_file_module.registers[4] =  32'd300;
        dut.register_file_module.registers[5] =  32'd400;
        dut.register_file_module.registers[6] =  32'd500;
        dut.register_file_module.registers[7] =  32'd600;
        dut.register_file_module.registers[8] =  32'd700;
        dut.register_file_module.registers[9] =  32'd800;
        dut.register_file_module.registers[10] = 32'd900;
        dut.register_file_module.registers[11] = 32'd1000;
        dut.register_file_module.registers[12] = 32'd0;
        dut.register_file_module.registers[13] = 32'd0;
        dut.register_file_module.registers[14] = 32'd0;
        dut.register_file_module.registers[15] = 32'd0;
        dut.register_file_module.registers[16] = 32'd0;
        dut.register_file_module.registers[17] = 32'd0;
        dut.register_file_module.registers[18] = 32'd0;
        dut.register_file_module.registers[19] = 32'd0;
        dut.register_file_module.registers[20] = 32'd0;
        dut.register_file_module.registers[21] = 32'd0;
        dut.register_file_module.registers[22] = 32'd0;
        dut.register_file_module.registers[23] = 32'd0;
        dut.register_file_module.registers[24] = 32'd0;
        dut.register_file_module.registers[25] = 32'd0;
        dut.register_file_module.registers[26] = 32'd0;
        dut.register_file_module.registers[27] = 32'd0;
        dut.register_file_module.registers[28] = 32'd0;
        dut.register_file_module.registers[29] = 32'd0;
        dut.register_file_module.registers[30] = 32'd0;
        dut.register_file_module.registers[31] = 32'd0;

        dut.data_memory_module.memory[0] = 32'd5;
        dut.data_memory_module.memory[1] = 32'd10;
        dut.data_memory_module.memory[2] = 32'd20;
        dut.data_memory_module.memory[3] = 32'd30;
        dut.data_memory_module.memory[4] = 32'd40;

        dut.ins_memory_module.memory[0] = 32'hfe010113;
        dut.ins_memory_module.memory[1] = 32'h02812623;
        dut.ins_memory_module.memory[2] = 32'h03010413;
        dut.ins_memory_module.memory[3] = 32'h00500793;
        dut.ins_memory_module.memory[4] = 32'hfef42023;
        dut.ins_memory_module.memory[5] = 32'h00a00793;
        dut.ins_memory_module.memory[6] = 32'hfcf42e23;
        dut.ins_memory_module.memory[7] = 32'hfe042703;
        dut.ins_memory_module.memory[8] = 32'hfdc42783;
        dut.ins_memory_module.memory[9] = 32'h00f707b3;
        dut.ins_memory_module.memory[10] = 32'hfcf42c23;
        dut.ins_memory_module.memory[11] = 32'hfd842703;
        dut.ins_memory_module.memory[12] = 32'hfe042783;
        dut.ins_memory_module.memory[13] = 32'h40f707b3;
        dut.ins_memory_module.memory[14] = 32'hfcf42a23;
        dut.ins_memory_module.memory[15] = 32'hfe042623;
        dut.ins_memory_module.memory[16] = 32'hfe042423;
        dut.ins_memory_module.memory[17] = 32'hfe042223;
        dut.ins_memory_module.memory[18] = 32'h0200006f;
        dut.ins_memory_module.memory[19] = 32'hfec42703;
        dut.ins_memory_module.memory[20] = 32'hfd442783;
        dut.ins_memory_module.memory[21] = 32'h00f707b3;
        dut.ins_memory_module.memory[22] = 32'hfef42623;
        dut.ins_memory_module.memory[23] = 32'hfe442783;
        dut.ins_memory_module.memory[24] = 32'h00178793;
        dut.ins_memory_module.memory[25] = 32'hfef42223;
        dut.ins_memory_module.memory[26] = 32'hfe442703;
        dut.ins_memory_module.memory[27] = 32'hfdc42783;
        dut.ins_memory_module.memory[28] = 32'hfcf74ee3;
        dut.ins_memory_module.memory[29] = 32'h0200006f;
        dut.ins_memory_module.memory[30] = 32'hfec42703;
        dut.ins_memory_module.memory[31] = 32'hfe042783;
        dut.ins_memory_module.memory[32] = 32'h40f707b3;
        dut.ins_memory_module.memory[33] = 32'hfef42623;
        dut.ins_memory_module.memory[34] = 32'hfe842783;
        dut.ins_memory_module.memory[35] = 32'h00178793;
        dut.ins_memory_module.memory[36] = 32'hfef42423;
        dut.ins_memory_module.memory[37] = 32'hfec42703;
        dut.ins_memory_module.memory[38] = 32'hfe042783;
        dut.ins_memory_module.memory[39] = 32'hfcf75ee3;
        dut.ins_memory_module.memory[40] = 32'h0000006f;

    
        #10 reset = 0;

        // Display Memory Content
        #125 $display("MEM[2] = %h", dut.data_memory_module.memory[2]);


        #220;
        $finish;
    end

endmodule