/*
 * This IPs is the RAM memory for the Atmel MEGA CPU implementation.
 *
 * Copyright (C) 2017  Iulian Gheorghiu  (morgoth@devboard.tech)
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

 module mega_ram  #(
    parameter PLATFORM = "XILINX",
    parameter MEM_MODE = "BLOCK",
    parameter ADDR_BUS_WIDTH = 12,  /* < in address lines */
    parameter DATA_BUS_WIDTH = 8,
    parameter RAM_PATH = ""
) (
    input clk,
    input cs,
    input we,
    input re,
    input rst,
    output reg halt,
    input [ADDR_BUS_WIDTH-1:0] a,
    input [DATA_BUS_WIDTH - 1:0] d_in,
    output [DATA_BUS_WIDTH - 1:0] d_out
);

generate

if(PLATFORM == "XILINX")
begin

reg [DATA_BUS_WIDTH - 1:0] mem [(2**ADDR_BUS_WIDTH)-1:0];

initial begin
if (RAM_PATH != "")
    $readmemh({RAM_PATH, ".mem"}, mem);
end

integer clear_cnt;
always @ (posedge clk) begin
    if (rst) halt <= 1'b1;
    if (halt) begin
        for (clear_cnt = 0; clear_cnt < 256; clear_cnt = clear_cnt + 1)
        begin : CLEAR_RAM
            if (clear_cnt != 255) begin
                mem[clear_cnt] <= 8'h00;
            end
            else if (clear_cnt == 255) begin
                halt <= 1'b0;
            end
        end
    end
    else if (cs & we & (a < 12'hB00)) mem[a] <= d_in;
end

reg [DATA_BUS_WIDTH - 1:0]d_out_tmp;

always @ (posedge clk) begin
    d_out_tmp <= mem[a];
end
assign d_out = (cs & re) ? d_out_tmp : 8'b00;

end

endgenerate
endmodule
