/*
 * This IP is the MEGA/XMEGA ALU implementation.
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

module xmega_alu # (
        parameter  [`CORE_TYPE_BUS_LEN - 1:0]CORE_TYPE = `MEGA_XMEGA_1
        )(
    input [15:0]inst,
    input [4:0]rda,
    input [15:0]rd,
    input [4:0]rra,
    input [15:0]rr,
    output reg [15:0]R,
    input [7:0]sreg_in,
    output reg[7:0]sreg_out
);

reg v_16_fmul;

wire [15:0]mul_result_u_int = $unsigned(rd) * $unsigned(rr);
wire signed [7:0]mul_s_a = rd;
wire signed [7:0]mul_s_b = rr;
wire signed [8:0]mul_su_b = {1'b0, rr};
wire signed [15:0]mul_result_s_int = mul_s_a * mul_s_b;
wire signed [15:0]mul_result_s_u_int = mul_s_a * mul_su_b;

wire flag_h_adc_sub_cp = |{(~rd[3] & rr[3]), (rr[3] & R[3]), (R[3] & ~rd[3])};
wire flag_h_subi_sbci_cpi = |{(~rd[3] & inst[3]), (inst[3] & R[3]), (R[3] & ~rd[3])};
wire flag_v_add_adc = (&{rd[7], rr[7], ~R[7]}) | (&{~rd[7], ~rr[7], R[7]});
wire flag_v_sub_sbc = (&{rd[7], ~rr[7], ~R[7]}) | (&{~rd[7], rr[7], R[7]});
wire flag_v_subi_sbci_cpi = (&{rd[7], ~inst[11], ~R[7]}) | (&{~rd[7], inst[11], R[7]});

wire in_addr_1_and_2_equal = rda == rra;
wire RD_8_IS_ZERO = ~(|R[7:0]);
wire RD_16_IS_ZERO = ~(|R);
wire FLAG_S_NEG_INC_DEC_ADD_ADC = R[7] ^ sreg_out[`XMEGA_FLAG_V];
wire FLAG_S_SUB_SBC_CP_CPC = R[7] ^ flag_v_sub_sbc;
wire FLAG_S_SUBI_SBCI_CPI = R[7] ^ flag_v_subi_sbci_cpi;
wire FLAG_S_ADIW_SBIW = R[15] ^ sreg_out[`XMEGA_FLAG_V];
wire FLAG_S_ADD_ADC = R[7] ^ flag_v_add_adc;
wire [8:0]RESULT_SUBI_SBCI_CPI = rd[7:0] - {inst[11:8], inst[3:0]};
wire [8:0]RESULT_ADD_ADC = {1'b0, rd[7:0]} + {1'b0, rr[7:0]};
wire [8:0]RESULT_SUB_SBC = {1'b0, rd[7:0]} - {1'b0, rr[7:0]};
wire [7:0]RESULT_COM = 8'hFF - rd[7:0];
wire [7:0]RESULT_NEG = 8'h00 - rd[7:0];
//wire [15:0]RESULT_ADIW = rd + {inst[7:6], inst[3:0]};
//wire [15:0]RESULT_SBIW = rd - {inst[7:6], inst[3:0]};

wire [7:0]index; // "index expression not wide enough to address all of the elements in the array"
assign index = {4'd0, inst[2:0]};

always @*
begin
    sreg_out = sreg_in;
    R = 0;
    casex({CORE_TYPE, inst})
    `INSTRUCTION_MOVW,
    `INSTRUCTION_MOV:
    begin
        R = rr;
    end
    `INSTRUCTION_MUL:
    begin
        R = mul_result_u_int;
        sreg_out[`XMEGA_FLAG_C] = R[15];
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_MULS:
    begin
        R = mul_result_s_int;
        sreg_out[`XMEGA_FLAG_C] = R[15];
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_MULSU:
    begin
        R = mul_result_s_u_int;
        sreg_out[`XMEGA_FLAG_C] = R[15];
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_FMUL:
    begin
        {v_16_fmul, R} = {mul_result_u_int, 1'b0};
        sreg_out[`XMEGA_FLAG_C] = v_16_fmul;
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_FMULS:
    begin
        {v_16_fmul, R} = {mul_result_s_int, 1'b0};
        sreg_out[`XMEGA_FLAG_C] = v_16_fmul;
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_FMULSU:
    begin
        {v_16_fmul, R} = {mul_result_s_u_int, 1'b0};
        sreg_out[`XMEGA_FLAG_C] = v_16_fmul;
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_SUB:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUB_SBC;
        sreg_out[`XMEGA_FLAG_H] = flag_h_adc_sub_cp;
        sreg_out[`XMEGA_FLAG_V] = flag_v_sub_sbc;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUB_SBC_CP_CPC;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_SBC:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUB_SBC - sreg_in[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_H] = flag_h_adc_sub_cp;
        sreg_out[`XMEGA_FLAG_V] = flag_v_sub_sbc;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUB_SBC_CP_CPC;
        sreg_out[`XMEGA_FLAG_Z] = &{~R[7:0], sreg_in[`XMEGA_FLAG_Z]};
    end
    `INSTRUCTION_ADD:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_ADD_ADC;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
        if(in_addr_1_and_2_equal)
        begin // LSL
            //sreg_out[`XMEGA_FLAG_H] = rd[3];
            sreg_out[`XMEGA_FLAG_V] = R[7] ^ sreg_out[`XMEGA_FLAG_C];
            sreg_out[`XMEGA_FLAG_S] = FLAG_S_NEG_INC_DEC_ADD_ADC;
        end
        else
        begin // ADD
            sreg_out[`XMEGA_FLAG_H] = |{(rd[3] & rr[3]), (rr[3] & ~R[3]), (~R[3] & rd[3])};
            sreg_out[`XMEGA_FLAG_V] = flag_v_add_adc;
            sreg_out[`XMEGA_FLAG_S] = FLAG_S_ADD_ADC;
        end
    end
    `INSTRUCTION_ADC:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_ADD_ADC + sreg_in[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
        if(in_addr_1_and_2_equal)
        begin // ROL
            //sreg_out[`XMEGA_FLAG_H] = rd[3];
            sreg_out[`XMEGA_FLAG_V] = R[7] ^ sreg_out[`XMEGA_FLAG_C];
            sreg_out[`XMEGA_FLAG_S] = FLAG_S_NEG_INC_DEC_ADD_ADC;
        end
        else
        begin // ADC
            sreg_out[`XMEGA_FLAG_H] = |{(rd[3] & rr[3]), (rr[3] & ~R[3]), (~R[3] & rd[3])};
            sreg_out[`XMEGA_FLAG_V] = flag_v_add_adc;
            sreg_out[`XMEGA_FLAG_S] = FLAG_S_ADD_ADC;
        end
    end
    `INSTRUCTION_AND:
    begin // AND/TST
        //R[15:8] = 8'h00;
        R[7:0] = rd[7:0] & rr[7:0];
        sreg_out[`XMEGA_FLAG_V] = 1'b0;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_EOR:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = rd[7:0] ^ rr[7:0];
        sreg_out[`XMEGA_FLAG_V] = 1'b0;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_OR:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = rd[7:0] | rr[7:0];
        sreg_out[`XMEGA_FLAG_V] = 1'b0;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_SUBI:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUBI_SBCI_CPI;
        sreg_out[`XMEGA_FLAG_H] = flag_h_subi_sbci_cpi;
        sreg_out[`XMEGA_FLAG_V] = flag_v_subi_sbci_cpi;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUBI_SBCI_CPI;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_SBCI:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUBI_SBCI_CPI - sreg_in[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_H] = flag_h_subi_sbci_cpi;
        sreg_out[`XMEGA_FLAG_V] = flag_v_subi_sbci_cpi;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUBI_SBCI_CPI;
        sreg_out[`XMEGA_FLAG_Z] = &{~R[7:0], sreg_in[`XMEGA_FLAG_Z]};
    end
    `INSTRUCTION_ORI_SBR:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = rd | {inst[11:8], inst[3:0]};
        sreg_out[`XMEGA_FLAG_V] = 1'b0;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_ANDI_CBR:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = rd & {inst[11:8], inst[3:0]};
        sreg_out[`XMEGA_FLAG_V] = 1'b0;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_COM:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = {1'b1, RESULT_COM};
        sreg_out[`XMEGA_FLAG_V] = 1'b0;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_NEG:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = {|rd[7:0], RESULT_NEG};
        sreg_out[`XMEGA_FLAG_H] = R[3] | rd[3];
        sreg_out[`XMEGA_FLAG_V] = &{R[7], ~R[6:0]};
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_NEG_INC_DEC_ADD_ADC;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_SWAP:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = {rd[3:0] , rd[7:4]};
    end
    `INSTRUCTION_INC:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = rd[7:0] + 1;
        sreg_out[`XMEGA_FLAG_V] = &{R[7], ~R[6:0]};
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_NEG_INC_DEC_ADD_ADC;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_DEC:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = rd[7:0] - 1;
        sreg_out[`XMEGA_FLAG_V] = &{~R[7], R[6:0]};
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_NEG_INC_DEC_ADD_ADC;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_ASR:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = {rd[0], rd[7], rd[7:1]};
        sreg_out[`XMEGA_FLAG_V] = R[7] ^ rd[0];
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = R[7] ^ sreg_out[`XMEGA_FLAG_V];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_LSR:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = {rd[0], 1'b0, rd[7:1]};
        sreg_out[`XMEGA_FLAG_N] = 1'b0;
        sreg_out[`XMEGA_FLAG_V] = sreg_out[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_S] = sreg_out[`XMEGA_FLAG_V];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_ROR:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = {rd[0], sreg_in[`XMEGA_FLAG_C], rd[7:1]};
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_V] = sreg_out[`XMEGA_FLAG_N] ^ sreg_out[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_S] = sreg_out[`XMEGA_FLAG_V];
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_CP:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUB_SBC;
        sreg_out[`XMEGA_FLAG_H] = flag_h_adc_sub_cp;
        sreg_out[`XMEGA_FLAG_V] = flag_v_sub_sbc;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUB_SBC_CP_CPC;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_CPC:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUB_SBC - sreg_in[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_H] = flag_h_adc_sub_cp;
        sreg_out[`XMEGA_FLAG_V] = flag_v_sub_sbc;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUB_SBC_CP_CPC;
        sreg_out[`XMEGA_FLAG_Z] = &{~R[7:0], sreg_in[`XMEGA_FLAG_Z]};
    end
    `INSTRUCTION_CPI:
    begin
        //R[15:8] = 8'h00;
        {sreg_out[`XMEGA_FLAG_C], R[7:0]} = RESULT_SUBI_SBCI_CPI;
        sreg_out[`XMEGA_FLAG_H] = flag_h_subi_sbci_cpi;
        sreg_out[`XMEGA_FLAG_V] = flag_v_subi_sbci_cpi;
        sreg_out[`XMEGA_FLAG_N] = R[7];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_SUBI_SBCI_CPI;
        sreg_out[`XMEGA_FLAG_Z] = RD_8_IS_ZERO;
    end
    `INSTRUCTION_SEx_CLx:
    begin
        //R[15:8] = 8'h00;
        sreg_out[inst[6:4]] = ~inst[7];
    end
    `INSTRUCTION_ADIW:
    begin
        {sreg_out[`XMEGA_FLAG_C], R} = rd + {inst[7:6], inst[3:0]};
        sreg_out[`XMEGA_FLAG_V] = ~inst[7] & R[15];//sreg_out[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_N] = R[15];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_ADIW_SBIW;
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_SBIW:
    begin
        {sreg_out[`XMEGA_FLAG_C], R} = rd - {inst[7:6], inst[3:0]};
        sreg_out[`XMEGA_FLAG_V] = R[15] & ~inst[7];//sreg_out[`XMEGA_FLAG_C];
        sreg_out[`XMEGA_FLAG_N] = R[15];
        sreg_out[`XMEGA_FLAG_S] = FLAG_S_ADIW_SBIW;
        sreg_out[`XMEGA_FLAG_Z] = RD_16_IS_ZERO;
    end
    `INSTRUCTION_LDI:
    begin
        //R[15:8] = 8'h00;
        R[7:0] = {inst[11:8], inst[3:0]};
    end
    `INSTRUCTION_BLD:
    begin
        //R[15:8] = 8'h00;
        if(sreg_in[`XMEGA_FLAG_T])
            R[7:0] = rd[7:0] | (2 ** inst[2:0]);
        else
            R[7:0] = rd[7:0] & ~(2 ** inst[2:0]);
    end
    `INSTRUCTION_BST:
    begin
        sreg_out[`XMEGA_FLAG_T] = rd[index];
    end
    default:
    begin
        sreg_out = sreg_in;
        R = 0;
    end
    endcase
end
endmodule