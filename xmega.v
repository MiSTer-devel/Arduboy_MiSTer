/*
 * This IP is the MEGA/XMEGA WATCHDOG implementation.
 *
 * Copyright (C) 2018  Iulian Gheorghiu (morgoth.creator@gmail.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

`timescale 1ns / 1ps

`include "xmega_v.v"

/*
 * Watchdog.
 */
module watchdog # (
        parameter CNT_WIDTH = 0
        )(
        input rst,
        input clk,
        input wdt_clk,
        input wdt_rst_in,
        output wdt_rst_out);

reg [CNT_WIDTH:0]wdt_cnt;
reg reset;
reg reset_n;
reg wdt_reset;
reg wdt_reset_n;
reg wdt_rst_out_int;
assign wdt_rst_out = CNT_WIDTH != 0 ? (rst | wdt_rst_out_int) : rst;

always @ (posedge rst or posedge wdt_rst_in)
begin
    if(CNT_WIDTH != 0)
    begin
        if(rst)
            reset <= 1'b0;
        else
            reset <= ~reset_n;
    end
end

always @ (posedge rst or posedge wdt_clk)
begin
    if(CNT_WIDTH != 0)
    begin
        if(rst)
        begin
            reset_n <= 1'b0;
            wdt_reset <=1'b0;
            wdt_cnt <= 'h0000;
        end
        else if(reset != reset_n)
        begin
            reset_n <= reset;
            wdt_cnt <= 'h0000;
        end
        else if(wdt_clk)
        begin
            if(wdt_cnt[CNT_WIDTH])
            begin
                wdt_reset <= ~wdt_reset_n;
                wdt_cnt <= 'h0000;
            end
            else
            begin
                wdt_cnt <= wdt_cnt + 1;
            end
        end
    end
end

always @ (posedge rst or posedge clk)
begin
    if(CNT_WIDTH != 0)
    begin
        if(rst)
        begin
            wdt_reset_n <= 1'b1;
            wdt_rst_out_int <= 1'b0;
        end
        else if(wdt_reset != wdt_reset_n)
        begin
            wdt_reset_n <= wdt_reset;
            wdt_rst_out_int <= 1'b1;
        end
        else wdt_rst_out_int <= 1'b0;
    end
end

endmodule
/*
 * !Watchdog.
 */

/*
 * Interrupt and priority encoder.
 */
module int_encoder # (
        parameter VECTOR_INT_TABLE_SIZE = 0
        )(
        input rst,
        input clk,
        input [((VECTOR_INT_TABLE_SIZE == 0) ? 0 : VECTOR_INT_TABLE_SIZE-1):0]int_sig_in,
        output int_request,
        output reg[((VECTOR_INT_TABLE_SIZE > 127) ? 7 :
                    (VECTOR_INT_TABLE_SIZE > 63) ? 6 :
                    (VECTOR_INT_TABLE_SIZE > 31) ? 5 :
                    (VECTOR_INT_TABLE_SIZE > 15) ? 4 :
                    (VECTOR_INT_TABLE_SIZE > 7) ? 3 :
                    (VECTOR_INT_TABLE_SIZE > 3) ? 2 :
                    (VECTOR_INT_TABLE_SIZE > 1) ? 1 : 0) : 0]int_vect
        );

integer j;
always @ (posedge rst or posedge clk)
begin
    if(VECTOR_INT_TABLE_SIZE != 0)
    begin
        if(rst)
            int_vect <= 0;
        else
        begin
            int_vect <= 0;
            for (j=VECTOR_INT_TABLE_SIZE-1; j>=0; j=j-1)
            if (int_sig_in[j])
                int_vect <= j+1;
        end
    end
end

assign int_request = (int_vect != 0 && VECTOR_INT_TABLE_SIZE != 'h0);

endmodule
/*
 * !Interrupt and priority encoder.
 */

module xmega #(
    parameter PLATFORM = "XILINX",
    parameter  [`CORE_TYPE_BUS_LEN - 1:0]CORE_TYPE = `MEGA_XMEGA_1,
    parameter ROM_ADDR_WIDTH = 16,
    parameter RAM_ADDR_WIDTH = 16,
    parameter WATCHDOG_CNT_WIDTH = 0,
    parameter VECTOR_INT_TABLE_SIZE = 0
    )(
    input rst,
    output sys_rst_out,
    input clk,
    input clk_wdt,
    input hold,
    output reg[ROM_ADDR_WIDTH - 1:0]pgm_addr,
    input [15:0]pgm_data,
    output reg [RAM_ADDR_WIDTH - 1:0]data_addr,
    output reg [7:0]data_out,
    output reg data_write,
    input [7:0]data_in,
    output reg data_read,
    output reg [5:0]io_addr,
    output reg [7:0]io_out,
    output reg io_write,
    input [7:0]io_in,
    output reg io_read,
    input [(VECTOR_INT_TABLE_SIZE == 0 ? 0 : VECTOR_INT_TABLE_SIZE - 1):0]int_sig,
    output reg [(VECTOR_INT_TABLE_SIZE == 0 ? 0 : VECTOR_INT_TABLE_SIZE - 1):0]int_rst
    );

wire [15:0]index; // "index expression not wide enough to address all of the elements in the array"
assign index = {13'd0, pgm_data_registered[2:0]};

reg wdt_rst_out;
wire core_rst;
assign sys_rst_out = core_rst;

reg [RAM_ADDR_WIDTH - 1:0]data_addr_int;
reg [7:0]data_out_int;
reg data_write_int;
reg [7:0]data_in_int;
reg data_read_int;
reg [5:0]io_addr_int;
reg [7:0]io_out_int;
reg io_write_int;
reg [7:0]io_in_int;
reg io_bit_registered;
reg io_read_int;

reg [1:0]state_cnt;
reg [7:0]SREG;
reg [RAM_ADDR_WIDTH - 1:0]SP;
reg [7:0]EIND;
reg [7:0]RAMPZ;
reg [7:0]RAMPY;
reg [7:0]RAMPX;
reg [7:0]RAMPD;
reg [7:0]ramp_bits;


reg [ROM_ADDR_WIDTH - 1:0]PC;
wire [7:0]sreg_out;
reg [15:0]pgm_data_registered;
reg [15:0]pgm_data_int;

reg [4:0]rs1a;
reg [4:0]rs2a;
reg [4:0]rda;

reg [15:0]alu_rs1;
reg [15:0]alu_rs2;
wire [15:0]alu_rd;

wire [15:0]reg_rs1;
reg reg_rs1m;
wire [15:0]reg_rs2;
reg reg_rs2m;
reg [15:0]reg_rd;
reg reg_rdw;
reg reg_rdm;

reg skip_next_clock;

reg [7:0]PC_TMP_H;

wire [ROM_ADDR_WIDTH - 1:0]PC_PLUS_ONE = PC + 1;
wire [ROM_ADDR_WIDTH - 1:0]PC_MINUS_ONE = PC - 1;
wire [RAM_ADDR_WIDTH - 1:0]SP_PLUS_ONE = SP + 1;
reg [ROM_ADDR_WIDTH - 1:0]PC_PLUS_RJMP_RCALL;
reg [ROM_ADDR_WIDTH - 1:0]PC_PLUS_COND_BRANCH;

reg cnt_rst;
reg unlock_int_registered_step_2;

wire execute    = (~skip_next_clock & ~cnt_rst);

/**************** Interrupt instance  ********************/
localparam int_bus_size = (VECTOR_INT_TABLE_SIZE > 127) ? 8 :
                            (VECTOR_INT_TABLE_SIZE > 63) ? 7 :
                            (VECTOR_INT_TABLE_SIZE > 31) ? 6 :
                            (VECTOR_INT_TABLE_SIZE > 15) ? 5 :
                            (VECTOR_INT_TABLE_SIZE > 7) ? 4 :
                            (VECTOR_INT_TABLE_SIZE > 3) ? 3 :
                            (VECTOR_INT_TABLE_SIZE > 1) ? 2 : 1;

wire [int_bus_size - 1 : 0]current_int_vect_request;
reg [int_bus_size - 1 : 0]current_int_vect_registered;
wire int_request;
reg int_registered;

int_encoder # (
    .VECTOR_INT_TABLE_SIZE(VECTOR_INT_TABLE_SIZE)
    )int_encoder_inst(
    .rst(core_rst),
    .clk(clk),
    .int_sig_in(int_sig),
    .int_request(int_request),
    .int_vect(current_int_vect_request)
    );
/**************** !Interrupt instance  ********************/
initial begin
    //SREG <= 8'h00;
    //SP <= 'h0000;
end

reg [ROM_ADDR_WIDTH - 1:0]PC_SNAPSHOOT;

/*
 * Interrupt registration and CALL to vector table instruction inserter.
 */
always @*
begin
    pgm_data_int = pgm_data;
    int_registered = 1'b0;
    int_rst <= 1'b0;
    if(rst)
        current_int_vect_registered = {int_bus_size{1'b0}};
    else
    if(VECTOR_INT_TABLE_SIZE != 0)
    begin
        if(~skip_next_clock & ~cnt_rst)
        begin
            case({unlock_int_registered_step_2, int_request, sreg_out[`XMEGA_FLAG_I], state_cnt})
                {3'b011, `STEP0}:
                begin
                    current_int_vect_registered = current_int_vect_request;
                    pgm_data_int = 16'b1001010000001110;
                    int_registered = 1'b1;
                    int_rst <= 1'b1 << (current_int_vect_request - 1);
                    PC_SNAPSHOOT <= PC;
                end
                {3'b111, `STEP1}:
                begin
                    pgm_data_int = {current_int_vect_registered, 1'b0};
                    int_registered = 1'b1;
                end
                default:
                begin
                    pgm_data_int = pgm_data;
                    int_registered = 1'b0;
                    int_rst <= 1'b0;
                end
            endcase
        end
    end
end

always @ (posedge clk)
begin
    if(cnt_rst)
    begin
        if(ROM_ADDR_WIDTH > 16)
        begin
            EIND <= 8'h00;
            RAMPZ <= 8'h00;
            RAMPY <= 8'h00;
            RAMPX <= 8'h00;
            RAMPD <= 8'h00;
        end
    end
    else
    case({data_addr_int, data_write_int})
    {24'h058, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPD <= data_out_int;
    {24'h059, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPX <= data_out_int;
    {24'h05A, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPY <= data_out_int;
    {24'h05B, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPZ <= data_out_int;
    {24'h05C, 1'b1}: if(ROM_ADDR_WIDTH > 16) EIND <= data_out_int;
    endcase
    //if(`MEGA_REDUCED_INT === CORE_TYPE)
    begin
        case({io_addr_int, io_write_int})
        {6'h38, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPD <= io_out_int;
        {6'h39, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPX <= io_out_int;
        {6'h3A, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPY <= io_out_int;
        {6'h3B, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPZ <= io_out_int;
        {6'h3C, 1'b1}: if(ROM_ADDR_WIDTH > 16) EIND <= io_out_int;
        endcase
    end
end

always @*
begin
    alu_rs1 <= reg_rs1;
    alu_rs2 <= reg_rs2;
/* Data bus switch */ /*************************************************************/
    data_addr <= 'h000000;
    data_out <= 8'h00;
    data_write <= 1'b0;
    data_read <= 1'b0;
    data_in_int <= data_in;
    case(data_addr_int)
    24'h058: if(ROM_ADDR_WIDTH > 16) data_in_int <= RAMPD[7:0];
    24'h059: if(ROM_ADDR_WIDTH > 16) data_in_int <= RAMPX[7:0];
    24'h05A: if(ROM_ADDR_WIDTH > 16) data_in_int <= RAMPY[7:0];
    24'h05B: if(ROM_ADDR_WIDTH > 16) data_in_int <= RAMPZ[7:0];
    24'h05C: if(ROM_ADDR_WIDTH > 16) data_in_int <= EIND[7:0];
    24'h05D: data_in_int <= SP[7:0];
    24'h05E:
    begin
        if(RAM_ADDR_WIDTH > 8)
            data_in_int <= SP[RAM_ADDR_WIDTH - 1:8];
    end
    24'h05F: data_in_int <= SREG;
    default:
    begin
        data_addr <= data_addr_int;
        data_out <= data_out_int;
        data_write <= data_write_int;
        data_read <= data_read_int;
    end
    endcase
/* IO bus switch */ /*************************************************************/
    //if(`MEGA_REDUCED_INT === CORE_TYPE)
    begin
        io_addr <= 6'h00;
        io_out <= 8'h00;
        io_write <= 1'b0;
        io_read <= 1'b0;
        io_in_int = io_in;
        case(io_addr_int)
        6'h38: if(ROM_ADDR_WIDTH > 16) io_in_int = RAMPD[7:0];
        6'h39: if(ROM_ADDR_WIDTH > 16) io_in_int = RAMPX[7:0];
        6'h3A: if(ROM_ADDR_WIDTH > 16) io_in_int = RAMPY[7:0];
        6'h3B: if(ROM_ADDR_WIDTH > 16) io_in_int = RAMPZ[7:0];
        6'h3C: if(ROM_ADDR_WIDTH > 16) io_in_int = EIND[7:0];
        6'h3D: io_in_int = SP[7:0];
        6'h3E:
        begin
            if(RAM_ADDR_WIDTH > 8)
                io_in_int = SP[RAM_ADDR_WIDTH - 1:8];
        end
        'h3F: io_in_int = SREG;
        default:
        begin
            io_addr <= io_addr_int;
            io_out <= io_out_int;
            io_write <= io_write_int;
            io_read <= io_read_int;
        end
        endcase
    end
/* Set "reg_rd" */ /*************************************************************/
    casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
    {1'b1, `STEP0, `INSTRUCTION_LDS},
    {1'b1, `STEP0, `INSTRUCTION_LDS16},
    {1'b1, `STEP0, `INSTRUCTION_POP},
    {1'b1, `STEP0, `INSTRUCTION_LDD},
    {1'b1, `STEP0, `INSTRUCTION_LD_X},
    {1'b1, `STEP0, `INSTRUCTION_LD_XP},
    {1'b1, `STEP0, `INSTRUCTION_LD_YZP},
    {1'b1, `STEP0, `INSTRUCTION_LD_XN},
    {1'b1, `STEP0, `INSTRUCTION_LD_YZN}: reg_rd <= data_in_int;

    {1'b1, `STEP2, `INSTRUCTION_LD_XP},
    {1'b1, `STEP1, `INSTRUCTION_ST_XP}:
    begin
        if(ROM_ADDR_WIDTH > 16)
            {ramp_bits, reg_rd} <= {RAMPX, reg_rs2} + 1;
        else
            reg_rd <= reg_rs2 + 1;
    end
    {1'b1, `STEP2, `INSTRUCTION_LD_YZP},
    {1'b1, `STEP1, `INSTRUCTION_ST_YZP}:
    begin
        if(ROM_ADDR_WIDTH > 16)
            {ramp_bits, reg_rd} <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} + 1;
        else
            reg_rd <= reg_rs2 + 1;
    end
    {1'b1, `STEP1, `INSTRUCTION_LPM_R_P}:
    begin
        if(ROM_ADDR_WIDTH > 16)
            {ramp_bits, reg_rd} <= {RAMPZ, reg_rs2} + 1;
        else
            reg_rd <= reg_rs2 + 1;
    end

    {1'b1, `STEP2, `INSTRUCTION_LD_XN},
    {1'b1, `STEP2, `INSTRUCTION_ST_XN}:
    begin
        if(ROM_ADDR_WIDTH > 16)
            {ramp_bits, reg_rd} <= {RAMPX, reg_rs2} - 1;
        else
            reg_rd <= reg_rs2 - 1;
    end
    {1'b1, `STEP2, `INSTRUCTION_LD_YZN},
    {1'b1, `STEP2, `INSTRUCTION_ST_YZN}:
    begin
        if(ROM_ADDR_WIDTH > 16)
            {ramp_bits, reg_rd} <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} - 1;
        else
            reg_rd <= reg_rs2 - 1;
    end
    {1'b1, `STEP0, `INSTRUCTION_IN}: reg_rd <= io_in_int;
    {1'b1, `STEP2, `INSTRUCTION_LPM_R},
    {1'b1, `STEP2, `INSTRUCTION_LPM_ELPM}: reg_rd <= reg_rs2[0] ? pgm_data_int[15:8] : pgm_data_int[7:0];
    {1'b1, `STEP2, `INSTRUCTION_LPM_R_P}: reg_rd <= ~reg_rs2[0] ? pgm_data_int[15:8] : pgm_data_int[7:0];
    default: reg_rd <= alu_rd;
    endcase
/* Set "pgm_addr" */ /*************************************************************/
    casex({execute,  state_cnt, CORE_TYPE, pgm_data_registered})
    {1'b1, `STEP1, `INSTRUCTION_LPM_R},
    {1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
    {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}:
    begin
        if(ROM_ADDR_WIDTH > 16)
            pgm_addr <= {RAMPZ, reg_rs2[15:1]};
        else
            pgm_addr <= reg_rs2[15:1];
    end
    default: pgm_addr <= PC;
    endcase
end

always @ (posedge clk or posedge core_rst)
begin
    if(core_rst)
    begin
        cnt_rst = 1'b1;
        PC <= 'h000000;
        state_cnt <= `STEP0;
        //SP <= 8'h00;
        //SREG <= 8'h00;
        //data_addr_int <= 'h00000000;
        //data_out_int <= 8'h00;
        //io_addr_int <= 6'h00;
        //io_out_int = 8'h00;
        //PC_TMP_H <= 8'h00;
        //int_rst <= 1'b0;
    end
    else
    begin
        PC <= PC_PLUS_ONE;
        //data_addr_int <= 'h00000000;
        //data_out <= 8'h00;
        data_write_int <= 1'b0;
        data_read_int <= 1'b0;
        //io_addr <= 6'h00;
        //io_out <= 8'h00;
        //if(`MEGA_REDUCED_INT === CORE_TYPE)
        begin
            io_write_int <= 1'b0;
            io_read_int <= 1'b0;
            unlock_int_registered_step_2 <= 1'b0;
        end
        skip_next_clock <= 1'b0;
        if(state_cnt == `STEP0 && !skip_next_clock)
        begin
            pgm_data_registered = pgm_data_int;
        end
        if(cnt_rst)
        begin
            cnt_rst = 1'b0;
            pgm_data_registered = 16'h0000;
        end
        //PC_PLUS_RJMP_RCALL = PC + {{ROM_ADDR_WIDTH - 11{pgm_data_registered[11]}}, pgm_data_registered[10:0]};
        //PC_PLUS_COND_BRANCH = PC + {{ROM_ADDR_WIDTH - 6{pgm_data_registered[9]}}, pgm_data_registered[8:3]};
        rs2a <= {pgm_data_registered[9], pgm_data_registered[3:0]};
        rs1a <= pgm_data_registered[8:4];
        rda <= pgm_data_registered[8:4];
        reg_rs1m <= `REG_MODE_8_BIT;
        reg_rs2m <= `REG_MODE_8_BIT;
        reg_rdm <= `REG_MODE_8_BIT;
        reg_rdw <= 1'b0;
        state_cnt <= `STEP0;
        wdt_rst_out <= 1'b0;
        if(~skip_next_clock & ~cnt_rst)
        begin
/* Set "Load default registers state if othervice is not specifyed" */ /*************************************************************/
            SREG <= sreg_out;
/* Set "Interrupt flag reset & unlock CALL address insert" */ /*************************************************************/
            if(VECTOR_INT_TABLE_SIZE != 0)
            begin
                if(int_registered)
                begin
                    unlock_int_registered_step_2 <= 1'b1;
                end
            end
/* Set "WDR" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_WDR}: wdt_rst_out <= 1'b1;
            endcase
/* Set "io_addr" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP1, `INSTRUCTION_OUT},
            {1'b1, `STEP0, `INSTRUCTION_IN},
            {1'b1, `STEP1, `INSTRUCTION_IN}: io_addr_int <= {pgm_data_registered[10:9],pgm_data_registered[3:0]};
            {1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
            {1'b1, `STEP1, `INSTRUCTION_CBI_SBI},
            {1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
            {1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}: io_addr_int <= pgm_data_registered[7:3];
            endcase
/* Set "io_out" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP1, `INSTRUCTION_OUT}: io_out_int = reg_rs1;
            {1'b1, `STEP1, `INSTRUCTION_CBI_SBI}:
            begin
                if(pgm_data_registered[9])
                    io_out_int = io_in | (2 ** pgm_data_registered[2:0]);
                else
                    io_out_int = io_in & ~(2 ** pgm_data_registered[2:0]);
            end
            endcase
/* Set "io_read" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_IN},
            {1'b1, `STEP1, `INSTRUCTION_IN},
            {1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
            {1'b1, `STEP1, `INSTRUCTION_CBI_SBI},
            {1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
            {1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}: io_read_int <= 1'b1;
            endcase
/* Set "io_write" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP1, `INSTRUCTION_OUT},
            {1'b1, `STEP1, `INSTRUCTION_CBI_SBI}: io_write_int <= 1'b1;
            endcase
/* Set "SREG" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP1, `INSTRUCTION_CALL}:
            begin
                if(int_registered)
                    SREG[`XMEGA_FLAG_I] <= 1'b0;
            end
            {1'b1, `STEP1, `INSTRUCTION_STS}:
            begin
                case(data_addr_int)
                24'h05F: SREG <= reg_rs1; //SREG
                endcase
            end
            {1'b1, `STEP3, `INSTRUCTION_RET},
            {1'b1, `STEP3, `INSTRUCTION_RETI}:
            begin
                if(pgm_data_registered[4])
                    SREG[`XMEGA_FLAG_I] <= 1'b1;
            end
            {1'b1, `STEP1, `INSTRUCTION_OUT}:
            begin
                case({pgm_data_registered[10:9],pgm_data_registered[3:0]})
                6'h3F: SREG <= reg_rs1; //SREG
                endcase
            end
            {1'b1, `STEP1, `INSTRUCTION_CBI_SBI}:
            begin
                case({pgm_data_registered[10:9],pgm_data_registered[3:0]})
                6'h3F:
                begin
                    if(pgm_data_registered[9])
                        SREG <= io_in_int | (2 ** pgm_data_registered[2:0]); /*SREG*/
                    else
                        SREG <= io_in_int & ~(2 ** pgm_data_registered[2:0]); /*SREG*/
                end
                endcase
            end
            endcase
/* Set "SP" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_RCALL},
            {1'b1, `STEP1, `INSTRUCTION_RCALL},
            {1'b1, `STEP0, `INSTRUCTION_CALL},
            {1'b1, `STEP1, `INSTRUCTION_CALL},
            {1'b1, `STEP0, `INSTRUCTION_ICALL},
            {1'b1, `STEP1, `INSTRUCTION_ICALL},
            {1'b1, `STEP1, `INSTRUCTION_PUSH}: SP <= SP - 1;
            {1'b1, `STEP0, `INSTRUCTION_RET},
            {1'b1, `STEP0, `INSTRUCTION_RETI},
            {1'b1, `STEP1, `INSTRUCTION_RET},
            {1'b1, `STEP1, `INSTRUCTION_RETI},
            {1'b1, `STEP0, `INSTRUCTION_POP}: SP <= SP_PLUS_ONE;
            {1'b1, `STEP1, `INSTRUCTION_STS}:
            begin
                case(pgm_data_int)
                16'h005D: SP[7:0] <= reg_rs1;
                16'h005E:
                begin
                    if(RAM_ADDR_WIDTH > 8)
                        SP[RAM_ADDR_WIDTH - 1:8] <= reg_rs1; //SPH
                end
                endcase
            end
            {1'b1, `STEP1, `INSTRUCTION_OUT}:
            begin
                case({pgm_data_registered[10:9],pgm_data_registered[3:0]})
                6'h3D: SP[7:0] <= reg_rs1; //SPL
                6'h3E:
                begin
                    if(RAM_ADDR_WIDTH > 8)
                        SP[RAM_ADDR_WIDTH - 1:8] <= reg_rs1; //SPH
                end
                endcase
            end
            {1'b1, `STEP1, `INSTRUCTION_CBI_SBI}:
            begin
                case({pgm_data_registered[10:9],pgm_data_registered[3:0]})
                6'h3D: SP[7:0] <= io_out_int; /*SPL*/
                6'h3E:
                begin
                    if(RAM_ADDR_WIDTH > 8)
                        SP[RAM_ADDR_WIDTH - 1:8] <= io_out_int; /*SPH*/
                end
                endcase
            end
            endcase
/* Set "rs1a" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_MULS}: rs1a[4] <= 1'b1;
            {1'b1, `STEP0, `INSTRUCTION_FMUL},
            {1'b1, `STEP0, `INSTRUCTION_FMULS},
            {1'b1, `STEP0, `INSTRUCTION_MULSU},
            {1'b1, `STEP0, `INSTRUCTION_FMULSU}: rs1a[4:3] <= 2'b10;
            {1'b1, `STEP0, `INSTRUCTION_CPI}: rs1a[4] <= 1'b1;
            {1'b1, `STEP0, `INSTRUCTION_SUBI},
            {1'b1, `STEP0, `INSTRUCTION_SBCI},
            {1'b1, `STEP0, `INSTRUCTION_ORI_SBR},
            {1'b1, `STEP0, `INSTRUCTION_ANDI_CBR}: rs1a[4] <= 1'b1;
            {1'b1, `STEP0, `INSTRUCTION_ADIW},
            {1'b1, `STEP0, `INSTRUCTION_SBIW}: rs1a[4:2] <= 3'b111;
            {1'b1, `STEP0, `INSTRUCTION_IJMP},
            {1'b1, `STEP0, `INSTRUCTION_ICALL}: rs1a <= 5'b01111;
            endcase
/* Set "reg_rs1m" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_ADIW},
            {1'b1, `STEP0, `INSTRUCTION_SBIW},
            {1'b1, `STEP0, `INSTRUCTION_IJMP},
            {1'b1, `STEP0, `INSTRUCTION_ICALL}: reg_rs1m <= `REG_MODE_16_BIT;
            endcase
/* Set "rs2a" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_MULS}: rs2a[4] <= 1'b1;
            {1'b1, `STEP0, `INSTRUCTION_FMUL},
            {1'b1, `STEP0, `INSTRUCTION_FMULS},
            {1'b1, `STEP0, `INSTRUCTION_MULSU},
            {1'b1, `STEP0, `INSTRUCTION_FMULSU}: rs2a[4:3] <= 2'b10;
            {1'b1, `STEP0, `INSTRUCTION_LDD},
            {1'b1, `STEP0, `INSTRUCTION_STD},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN}: rs2a <= {3'b111, ~pgm_data_registered[3]};
            {1'b1, `STEP0, `INSTRUCTION_LD_X},
            {1'b1, `STEP1, `INSTRUCTION_LD_X},
            {1'b1, `STEP0, `INSTRUCTION_ST_X},
            {1'b1, `STEP1, `INSTRUCTION_ST_X},
            {1'b1, `STEP0, `INSTRUCTION_LD_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP1, `INSTRUCTION_ST_XP},
            {1'b1, `STEP0, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP0, `INSTRUCTION_ST_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN}: rs2a <= 4'b1101;
            {1'b1, `STEP0, `INSTRUCTION_LPM_R},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP0, `INSTRUCTION_LPM_ELPM},
            {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: rs2a <= 5'b01111;
            endcase
/* Set "reg_rs2m" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_MOVW},
            {1'b1, `STEP0, `INSTRUCTION_LDD},
            {1'b1, `STEP0, `INSTRUCTION_STD},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP0, `INSTRUCTION_LD_X},
            {1'b1, `STEP1, `INSTRUCTION_LD_X},
            {1'b1, `STEP0, `INSTRUCTION_ST_X},
            {1'b1, `STEP1, `INSTRUCTION_ST_X},
            {1'b1, `STEP0, `INSTRUCTION_LD_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP1, `INSTRUCTION_ST_XP},
            {1'b1, `STEP0, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP0, `INSTRUCTION_ST_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP0, `INSTRUCTION_LPM_ELPM},
            {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: reg_rs2m <= `REG_MODE_16_BIT;
            endcase
/* Set Load bit to test and freeze it because will be used in STAGE1 and STAGE 2 */ /**********/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
                {1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}: io_bit_registered = io_in_int[pgm_data_registered[2:0]];
            endcase
/* Set "skip_next_clock" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_COND_BRANCH}: /***********************/
            begin
                if(sreg_out[pgm_data_registered[2:0]] == ~pgm_data_registered[10])
                begin
                    skip_next_clock <= 1'b1;
                end
            end
            {1'b1, `STEP1, `INSTRUCTION_CPSE}:
            begin
                if(reg_rs1 == reg_rs2)
                begin
                    casex({CORE_TYPE, pgm_data_int})
                    `INSTRUCTION_LDS,
                    `INSTRUCTION_STS,
                    `INSTRUCTION_JMP,
                    `INSTRUCTION_CALL: ;
                    default: skip_next_clock <= 1'b1;
                    endcase
                end
            end
            {1'b1, `STEP1, `INSTRUCTION_SBRC_SBRS}:
            begin
                if(reg_rs1[index] == pgm_data_registered[9])
                begin
                    casex({CORE_TYPE, pgm_data_int})
                    `INSTRUCTION_LDS,
                    `INSTRUCTION_STS,
                    `INSTRUCTION_JMP,
                    `INSTRUCTION_CALL: ;
                    default: skip_next_clock <= 1'b1;
                    endcase
                end
            end
            {1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}:
            begin
                if(io_bit_registered == pgm_data_registered[9])
                begin
                    casex({CORE_TYPE, pgm_data_int})
                    `INSTRUCTION_LDS,
                    `INSTRUCTION_STS,
                    `INSTRUCTION_JMP,
                    `INSTRUCTION_CALL: ;
                    default: skip_next_clock <= 1'b1;
                    endcase
                end
            end
            {1'b1, `STEP0, `INSTRUCTION_RJMP},
            {1'b1, `STEP1, `INSTRUCTION_JMP},
            {1'b1, `STEP1, `INSTRUCTION_IJMP},
            {1'b1, `STEP1, `INSTRUCTION_RCALL},
            {1'b1, `STEP1, `INSTRUCTION_CALL},
            {1'b1, `STEP1, `INSTRUCTION_ICALL},
            {1'b1, `STEP3, `INSTRUCTION_RET},
            {1'b1, `STEP3, `INSTRUCTION_RETI}:skip_next_clock <= 1'b1;
            endcase
/* Set "rda" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_LDI}: rda[4] <= 1'b1;
            {1'b1, `STEP0, `INSTRUCTION_MUL},
            {1'b1, `STEP0, `INSTRUCTION_MULS},
            {1'b1, `STEP0, `INSTRUCTION_FMUL},
            {1'b1, `STEP0, `INSTRUCTION_FMULS},
            {1'b1, `STEP0, `INSTRUCTION_MULSU},
            {1'b1, `STEP0, `INSTRUCTION_FMULSU},
            {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: rda <= 5'h00;
            {1'b1, `STEP0, `INSTRUCTION_SUBI},
            {1'b1, `STEP0, `INSTRUCTION_SBCI},
            {1'b1, `STEP0, `INSTRUCTION_ORI_SBR},
            {1'b1, `STEP0, `INSTRUCTION_ANDI_CBR}: rda[4] <= 1'b1;
            {1'b1, `STEP0, `INSTRUCTION_ADIW},
            {1'b1, `STEP0, `INSTRUCTION_SBIW}:rda[4:2] <= 3'b111;
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN}: rda <= {3'b111, ~pgm_data_registered[3]};
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN}: rda <= 4'b1101;
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P}: rda <= 5'b01111;
            endcase
/* Set "reg_rdm" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_MOVW},
            {1'b1, `STEP0, `INSTRUCTION_MUL},
            {1'b1, `STEP0, `INSTRUCTION_MULS},
            {1'b1, `STEP0, `INSTRUCTION_FMUL},
            {1'b1, `STEP0, `INSTRUCTION_FMULS},
            {1'b1, `STEP0, `INSTRUCTION_MULSU},
            {1'b1, `STEP0, `INSTRUCTION_FMULSU},
            {1'b1, `STEP0, `INSTRUCTION_ADIW},
            {1'b1, `STEP0, `INSTRUCTION_SBIW},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P}: reg_rdm <= `REG_MODE_16_BIT;
            endcase
/* Set "reg_rdw" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_LDI},
            {1'b1, `STEP0, `INSTRUCTION_MOVW},
            {1'b1, `STEP0, `INSTRUCTION_MUL},
            {1'b1, `STEP0, `INSTRUCTION_MULS},
            {1'b1, `STEP0, `INSTRUCTION_FMUL},
            {1'b1, `STEP0, `INSTRUCTION_FMULS},
            {1'b1, `STEP0, `INSTRUCTION_MULSU},
            {1'b1, `STEP0, `INSTRUCTION_FMULSU},
            {1'b1, `STEP0, `INSTRUCTION_INC},
            {1'b1, `STEP0, `INSTRUCTION_DEC},
            {1'b1, `STEP0, `INSTRUCTION_ASR},
            {1'b1, `STEP0, `INSTRUCTION_LSR},
            {1'b1, `STEP0, `INSTRUCTION_ROR},
            {1'b1, `STEP0, `INSTRUCTION_SUB},
            {1'b1, `STEP0, `INSTRUCTION_SBC},
            {1'b1, `STEP0, `INSTRUCTION_ADD},
            {1'b1, `STEP0, `INSTRUCTION_ADC},
            {1'b1, `STEP0, `INSTRUCTION_SWAP},
            {1'b1, `STEP0, `INSTRUCTION_AND},
            {1'b1, `STEP0, `INSTRUCTION_EOR},
            {1'b1, `STEP0, `INSTRUCTION_OR},
            {1'b1, `STEP0, `INSTRUCTION_MOV},
            {1'b1, `STEP0, `INSTRUCTION_BLD},
            {1'b1, `STEP0, `INSTRUCTION_SUBI},
            {1'b1, `STEP0, `INSTRUCTION_SBCI},
            {1'b1, `STEP0, `INSTRUCTION_ORI_SBR},
            {1'b1, `STEP0, `INSTRUCTION_ANDI_CBR},
            {1'b1, `STEP0, `INSTRUCTION_COM},
            {1'b1, `STEP0, `INSTRUCTION_NEG},
            {1'b1, `STEP0, `INSTRUCTION_ADIW},
            {1'b1, `STEP0, `INSTRUCTION_SBIW},
            {1'b1, `STEP1, `INSTRUCTION_POP},
            {1'b1, `STEP2, `INSTRUCTION_LDS},
            {1'b1, `STEP1, `INSTRUCTION_LDS16},
            {1'b1, `STEP2, `INSTRUCTION_LDD},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP2, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP2, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP2, `INSTRUCTION_LD_X},
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP2, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP2, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN},
            {1'b1, `STEP1, `INSTRUCTION_IN},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: reg_rdw <= 1'b1;
            endcase
/* Set "data_addr" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_RCALL},
            {1'b1, `STEP1, `INSTRUCTION_RCALL},
            {1'b1, `STEP0, `INSTRUCTION_CALL},
            {1'b1, `STEP1, `INSTRUCTION_CALL},
            {1'b1, `STEP0, `INSTRUCTION_ICALL},
            {1'b1, `STEP1, `INSTRUCTION_ICALL},
            {1'b1, `STEP1, `INSTRUCTION_PUSH}: data_addr_int <= SP;
            {1'b1, `STEP0, `INSTRUCTION_RET},
            {1'b1, `STEP0, `INSTRUCTION_RETI},
            {1'b1, `STEP1, `INSTRUCTION_RET},
            {1'b1, `STEP1, `INSTRUCTION_RETI}: data_addr_int <= SP_PLUS_ONE;
            {1'b1, `STEP2, `INSTRUCTION_RET},
            {1'b1, `STEP2, `INSTRUCTION_RETI}: data_addr_int <= SP;
            {1'b1, `STEP0, `INSTRUCTION_POP}: data_addr_int <= SP_PLUS_ONE;
            {1'b1, `STEP1, `INSTRUCTION_POP}: data_addr_int <= SP;
            {1'b1, `STEP1, `INSTRUCTION_LDS}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPD, pgm_data_int};
                else
                    data_addr_int <= pgm_data_int;
            end
            {1'b1, `STEP2, `INSTRUCTION_LDS}: data_addr_int <= data_addr_int;
            {1'b1, `STEP0, `INSTRUCTION_LDS16}: data_addr_int <= {pgm_data_registered[10:8], pgm_data_registered[3:0]};
            {1'b1, `STEP1, `INSTRUCTION_LDS16}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_STS}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPD, pgm_data_int};
                else
                    data_addr_int <= pgm_data_int;
            end
            {1'b1, `STEP0, `INSTRUCTION_STS16}: data_addr_int <= {pgm_data_registered[10:8], pgm_data_registered[3:0]};
            {1'b1, `STEP1, `INSTRUCTION_LDD}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
                else
                    data_addr_int <= reg_rs2 + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
            end
            {1'b1, `STEP2, `INSTRUCTION_LDD}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_STD}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
                else
                    data_addr_int <= reg_rs2 + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
            end
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2};
                else
                    data_addr_int <= reg_rs2;
            end
            {1'b1, `STEP2, `INSTRUCTION_LD_YZP}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_ST_YZP}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2};
                else
                    data_addr_int <= reg_rs2;
            end
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} - 1;
                else
                    data_addr_int <= reg_rs2 - 1;
            end
            {1'b1, `STEP2, `INSTRUCTION_LD_YZN}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} - 1;
                else
                    data_addr_int <= reg_rs2 - 1;
            end
            {1'b1, `STEP2, `INSTRUCTION_ST_YZN}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_LD_X}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPX, reg_rs2};
                else
                    data_addr_int <= reg_rs2;
            end
            {1'b1, `STEP2, `INSTRUCTION_LD_X}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_ST_X}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPX, reg_rs2};
                else
                    data_addr_int <= reg_rs2;
            end
            {1'b1, `STEP1, `INSTRUCTION_LD_XP}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPX, reg_rs2};
                else
                    data_addr_int <= reg_rs2;
            end
            {1'b1, `STEP2, `INSTRUCTION_LD_XP}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_ST_XP}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPX, reg_rs2};
                else
                    data_addr_int <= reg_rs2;
            end
            {1'b1, `STEP1, `INSTRUCTION_LD_XN}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPX, reg_rs2} - 1;
                else
                    data_addr_int <= reg_rs2 - 1;
            end
            {1'b1, `STEP2, `INSTRUCTION_LD_XN}: data_addr_int <= data_addr_int;
            {1'b1, `STEP1, `INSTRUCTION_ST_XN}:
            begin
                if(ROM_ADDR_WIDTH > 16)
                    data_addr_int <= {RAMPX, reg_rs2} - 1;
                else
                    data_addr_int <= reg_rs2 - 1;
            end
            {1'b1, `STEP2, `INSTRUCTION_ST_XN}: data_addr_int <= data_addr_int;
            endcase
/* Set "data_out" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_RCALL}: data_out_int <= PC;
            {1'b1, `STEP1, `INSTRUCTION_RCALL}: data_out_int <= PC[ROM_ADDR_WIDTH - 1:8];
            {1'b1, `STEP0, `INSTRUCTION_CALL}:
            begin
                data_out_int <= PC_PLUS_ONE;
                if(int_registered)
                    data_out_int <= PC_SNAPSHOOT;
            end
            {1'b1, `STEP1, `INSTRUCTION_CALL}:data_out_int <= PC_TMP_H;
            {1'b1, `STEP0, `INSTRUCTION_ICALL}: data_out_int <= PC;
            {1'b1, `STEP1, `INSTRUCTION_ICALL}: data_out_int <= PC_TMP_H;
            {1'b1, `STEP1, `INSTRUCTION_PUSH},
            {1'b1, `STEP1, `INSTRUCTION_STS},
            {1'b1, `STEP0, `INSTRUCTION_STS16},
            {1'b1, `STEP1, `INSTRUCTION_STD},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP2, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_X},
            {1'b1, `STEP1, `INSTRUCTION_ST_XP},
            {1'b1, `STEP2, `INSTRUCTION_ST_XN}: data_out_int <= reg_rs1;
            endcase
/* Set "data_write" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_RCALL},
            {1'b1, `STEP1, `INSTRUCTION_RCALL},
            {1'b1, `STEP0, `INSTRUCTION_CALL},
            {1'b1, `STEP1, `INSTRUCTION_CALL},
            {1'b1, `STEP0, `INSTRUCTION_ICALL},
            {1'b1, `STEP1, `INSTRUCTION_ICALL},
            {1'b1, `STEP1, `INSTRUCTION_PUSH},
            {1'b1, `STEP0, `INSTRUCTION_STS16},
            {1'b1, `STEP1, `INSTRUCTION_STD},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP2, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_X},
            {1'b1, `STEP1, `INSTRUCTION_ST_XP},
            {1'b1, `STEP2, `INSTRUCTION_ST_XN},
            {1'b1, `STEP1, `INSTRUCTION_STS}: data_write_int <= 1'b1;
            endcase
/* Set "data_read" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP1, `INSTRUCTION_RET},
            {1'b1, `STEP1, `INSTRUCTION_RETI},
            {1'b1, `STEP2, `INSTRUCTION_RET},
            {1'b1, `STEP2, `INSTRUCTION_RETI},
            {1'b1, `STEP1, `INSTRUCTION_POP},
            {1'b1, `STEP1, `INSTRUCTION_LDS16},
            {1'b1, `STEP2, `INSTRUCTION_LDD},
            {1'b1, `STEP2, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP2, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP2, `INSTRUCTION_LD_X},
            {1'b1, `STEP2, `INSTRUCTION_LD_XP},
            {1'b1, `STEP2, `INSTRUCTION_LD_XN},
            {1'b1, `STEP2, `INSTRUCTION_LDS}: data_read_int <= 1'b1;
            endcase
/* Set "state_cnt" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_JMP},
            {1'b1, `STEP0, `INSTRUCTION_IJMP},
            {1'b1, `STEP0, `INSTRUCTION_RCALL},
            {1'b1, `STEP0, `INSTRUCTION_CALL},
            {1'b1, `STEP0, `INSTRUCTION_ICALL},
            {1'b1, `STEP0, `INSTRUCTION_PUSH},
            {1'b1, `STEP0, `INSTRUCTION_RET},
            {1'b1, `STEP0, `INSTRUCTION_RETI},
            {1'b1, `STEP0, `INSTRUCTION_POP},
            {1'b1, `STEP0, `INSTRUCTION_CPSE},
            {1'b1, `STEP0, `INSTRUCTION_SBRC_SBRS},
            {1'b1, `STEP0, `INSTRUCTION_LDS},
            {1'b1, `STEP0, `INSTRUCTION_LDS16},
            {1'b1, `STEP0, `INSTRUCTION_STS},
            {1'b1, `STEP0, `INSTRUCTION_LDD},
            {1'b1, `STEP0, `INSTRUCTION_STD},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP0, `INSTRUCTION_LD_X},
            {1'b1, `STEP0, `INSTRUCTION_ST_X},
            {1'b1, `STEP0, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP0, `INSTRUCTION_LD_XN},
            {1'b1, `STEP0, `INSTRUCTION_ST_XN},
            {1'b1, `STEP0, `INSTRUCTION_OUT},
            {1'b1, `STEP0, `INSTRUCTION_IN},
            {1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
            {1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP0, `INSTRUCTION_LPM_ELPM}: state_cnt <= `STEP1;
/*************************************************************/
            {1'b1, `STEP1, `INSTRUCTION_RET},
            {1'b1, `STEP1, `INSTRUCTION_RETI}:state_cnt <= `STEP2;
            {1'b1, `STEP1, `INSTRUCTION_CPSE}:
            begin
                casex({reg_rs1 == reg_rs2, {CORE_TYPE, pgm_data_int}})
                {1'b1, `INSTRUCTION_LDS},
                {1'b1, `INSTRUCTION_STS},
                {1'b1, `INSTRUCTION_JMP},
                {1'b1, `INSTRUCTION_CALL}: state_cnt <= `STEP2;
                endcase
            end
            {1'b1, `STEP1, `INSTRUCTION_SBRC_SBRS}:
            begin
                casex({reg_rs1[index] == pgm_data_registered[9], {CORE_TYPE, pgm_data_int}})
                {1'b1, `INSTRUCTION_LDS},
                {1'b1, `INSTRUCTION_STS},
                {1'b1, `INSTRUCTION_JMP},
                {1'b1, `INSTRUCTION_CALL}: state_cnt <= `STEP2;
                endcase
            end
            {1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}:
            begin
                casex({io_in_int[pgm_data_registered[2:0]] == pgm_data_registered[9], {CORE_TYPE, pgm_data_int}})
                {1'b1, `INSTRUCTION_LDS},
                {1'b1, `INSTRUCTION_STS},
                {1'b1, `INSTRUCTION_JMP},
                {1'b1, `INSTRUCTION_CALL}: state_cnt <= `STEP2;
                endcase
            end
            {1'b1, `STEP1, `INSTRUCTION_LDS},
            {1'b1, `STEP1, `INSTRUCTION_LDD},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_LD_X},
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: state_cnt <= `STEP2;
/*************************************************************/
            {1'b1, `STEP2, `INSTRUCTION_RET},
            {1'b1, `STEP2, `INSTRUCTION_RETI},
            {1'b1, `STEP2, `INSTRUCTION_CPSE},
            {1'b1, `STEP2, `INSTRUCTION_SBRC_SBRS},
            {1'b1, `STEP2, `INSTRUCTION_SBIC_SBIS}: state_cnt <= `STEP3;
            endcase
/* Set "PC_TMP_H" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_CALL}:
            begin
                PC_TMP_H <= PC_PLUS_ONE[ROM_ADDR_WIDTH - 1:8];
                if(int_registered)
                    PC_TMP_H <= PC_SNAPSHOOT[ROM_ADDR_WIDTH - 1:8];
            end
            {1'b1, `STEP0, `INSTRUCTION_ICALL}: PC_TMP_H <= PC[ROM_ADDR_WIDTH - 1:8];
            {1'b1, `STEP2, `INSTRUCTION_RET},
            {1'b1, `STEP2, `INSTRUCTION_RETI}: PC_TMP_H <= data_in_int;
            endcase
/* Set "PC" */ /*************************************************************/
            casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
            {1'b1, `STEP0, `INSTRUCTION_RJMP}: PC <= PC + {{ROM_ADDR_WIDTH - 11{pgm_data_registered[11]}}, pgm_data_registered[10:0]};
            {1'b1, `STEP0, `INSTRUCTION_COND_BRANCH}: /***********************/
            begin
                if(sreg_out[pgm_data_registered[2:0]] == ~pgm_data_registered[10])
                begin
                    PC <= PC + {{ROM_ADDR_WIDTH - 6{pgm_data_registered[9]}}, pgm_data_registered[8:3]};
                end
            end
            {1'b1, `STEP1, `INSTRUCTION_JMP}: PC <= {pgm_data_registered[8:4], pgm_data_registered[0], pgm_data_int};
            {1'b1, `STEP1, `INSTRUCTION_IJMP}: PC <= reg_rs1;
            {1'b1, `STEP0, `INSTRUCTION_RCALL}:  PC <= PC;
            {1'b1, `STEP1, `INSTRUCTION_RCALL}: PC <= PC + {{ROM_ADDR_WIDTH - 11{pgm_data_registered[11]}}, pgm_data_registered[10:0]};
            {1'b1, `STEP1, `INSTRUCTION_CALL}: PC <= {pgm_data_registered[8:4], pgm_data_registered[0], pgm_data_int};
            {1'b1, `STEP1, `INSTRUCTION_ICALL}: PC <= reg_rs1;
            {1'b1, `STEP0, `INSTRUCTION_PUSH},
            {1'b1, `STEP0, `INSTRUCTION_RET},
            {1'b1, `STEP0, `INSTRUCTION_RETI},
            {1'b1, `STEP1, `INSTRUCTION_RET},
            {1'b1, `STEP1, `INSTRUCTION_RETI},
            {1'b1, `STEP2, `INSTRUCTION_RET},
            {1'b1, `STEP2, `INSTRUCTION_RETI}: PC <= PC;
            {1'b1, `STEP3, `INSTRUCTION_RET}: PC <= {PC_TMP_H, data_in_int};
            {1'b1, `STEP3, `INSTRUCTION_RETI}: PC <= {PC_TMP_H, data_in_int} - 1;
            {1'b1, `STEP0, `INSTRUCTION_CPSE},
            {1'b1, `STEP0, `INSTRUCTION_SBRC_SBRS},
            {1'b1, `STEP0, `INSTRUCTION_POP},
            {1'b1, `STEP1, `INSTRUCTION_LDS},
            {1'b1, `STEP0, `INSTRUCTION_LDS16},
            {1'b1, `STEP0, `INSTRUCTION_LDD},
            {1'b1, `STEP1, `INSTRUCTION_LDD},
            {1'b1, `STEP0, `INSTRUCTION_STD},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZP},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZP},
            {1'b1, `STEP0, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP1, `INSTRUCTION_LD_YZN},
            {1'b1, `STEP0, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP1, `INSTRUCTION_ST_YZN},
            {1'b1, `STEP0, `INSTRUCTION_LD_X},
            {1'b1, `STEP1, `INSTRUCTION_LD_X},
            {1'b1, `STEP0, `INSTRUCTION_ST_X},
            {1'b1, `STEP0, `INSTRUCTION_LD_XP},
            {1'b1, `STEP1, `INSTRUCTION_LD_XP},
            {1'b1, `STEP0, `INSTRUCTION_ST_XP},
            {1'b1, `STEP0, `INSTRUCTION_LD_XN},
            {1'b1, `STEP1, `INSTRUCTION_LD_XN},
            {1'b1, `STEP0, `INSTRUCTION_ST_XN},
            {1'b1, `STEP1, `INSTRUCTION_ST_XN},
            {1'b1, `STEP0, `INSTRUCTION_OUT},
            {1'b1, `STEP0, `INSTRUCTION_IN},
            {1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
            {1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R},
            {1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
            {1'b1, `STEP0, `INSTRUCTION_LPM_ELPM},
            {1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: PC <= PC;
            endcase
        end
    end
end

xmega_alu #(
    .CORE_TYPE(CORE_TYPE)
)xmega_alu_inst(
    .inst(pgm_data_registered),
    .rda(rs1a),
    .rd(alu_rs1),
    .rra(rs2a),
    .rr(alu_rs2),
    .R(alu_rd),
    .sreg_in(SREG),
    .sreg_out(sreg_out)
);

xmega_regs #(
    .PLATFORM(PLATFORM)
)xmega_regs_inst(
    .rst(core_rst),
    .clk(clk),
    .rs1a(rs1a),
    .rs1(reg_rs1),
    .rs1m(reg_rs1m),
    .rs2a(rs2a),
    .rs2(reg_rs2),
    .rs2m(reg_rs2m),
    .rda(rda),
    .rd(reg_rd),
    .rdw(reg_rdw),
    .rdm(reg_rdm)
);

watchdog # (
    .CNT_WIDTH(WATCHDOG_CNT_WIDTH)
    )wdt_inst(
    .rst(rst),
    .clk(clk),
    .wdt_clk(clk_wdt),
    .wdt_rst_in(wdt_rst_out),
    .wdt_rst_out(core_rst)
);

endmodule
