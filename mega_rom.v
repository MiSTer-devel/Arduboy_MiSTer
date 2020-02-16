/*
 * This IP is the ROM memory for the Atmel MEGA CPU implementation.
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
 
module mega_rom  #(
	parameter ADDR_ROM_BUS_WIDTH = 14, /* < in address lines */
	parameter ROM_PATH = ""
) (
	input clk,
	input [ADDR_ROM_BUS_WIDTH-1:0] a,
	output reg[15:0]d
);

(* ram_style="block" *)
reg [15:0] mem [(2**ADDR_ROM_BUS_WIDTH)-1:0];

initial begin
if (ROM_PATH != "")
	$readmemh(ROM_PATH, mem);
end

reg [ADDR_ROM_BUS_WIDTH-1:0] a_int;

always @ (posedge clk)
begin
		d <= mem[a];
end

endmodule


