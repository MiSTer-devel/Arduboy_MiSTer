/*
 * This IP is the ATMEGA PIO implementation.
 *
 * Copyright (C) 2020  Iulian Gheorghiu (morgoth@devboard.tech)
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

/************************************************************/
/* Atention!  This file contain platform dependent modules. */
/************************************************************/

`timescale 1ns / 1ps


module atmega_pio # (
    parameter PLATFORM = "XILINX",
    parameter BUS_ADDR_DATA_LEN = 16,
    parameter PORT_ADDR = 0,
    parameter DDR_ADDR = 1,
    parameter PIN_ADDR = 2,
    parameter PINMASK = 8'hFF,
    parameter PULLUP_MASK = 8'h0,
    parameter PULLDN_MASK = 8'h0
)(
    input rst,
    input clk,
    input [BUS_ADDR_DATA_LEN-1:0]addr,
    input wr,
    input rd,
    input [7:0]bus_in,
    output reg [7:0]bus_out,

    input [7:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [7:0]bus_dat_in,
    output reg [7:0]bus_dat_out,

    input [7:0]io_in,
    output [7:0]io_out
    );

reg [7:0]DDR;
reg [7:0]PORT;

assign io_out[0] = DDR[0] ? PORT[0] : 1'b0;
assign io_out[1] = DDR[1] ? PORT[1] : 1'b0;
assign io_out[2] = DDR[2] ? PORT[2] : 1'b0;
assign io_out[3] = DDR[3] ? PORT[3] : 1'b0;
assign io_out[4] = DDR[4] ? PORT[4] : 1'b0;
assign io_out[5] = DDR[5] ? PORT[5] : 1'b0;
assign io_out[6] = DDR[6] ? PORT[6] : 1'b0;
assign io_out[7] = DDR[7] ? PORT[7] : 1'b0;

always @ (posedge rst or posedge clk)
begin
    if(rst)
    begin
        DDR <= 8'h00;
        PORT <= 8'h00;
    end
    else if(wr)
    begin
        case(addr)
        DDR_ADDR: DDR <= bus_in;
        PORT_ADDR: PORT <= bus_in;
        endcase
    end
    else if(wr_dat)
    begin
        case(addr_dat)
        (DDR_ADDR + 'h20): DDR <= bus_dat_in;
        (PORT_ADDR + 'h20): PORT <= bus_dat_in;
        endcase
    end
end

always @*
begin
    bus_out = 8'h00;
    if(rd & ~rst)
    begin
        case(addr)
        PORT_ADDR: bus_out = PORT;
        DDR_ADDR: bus_out = DDR;
        PIN_ADDR: bus_out = io_in;
        default: bus_out = 8'h00;
        endcase
    end
end

always @*
begin
    bus_dat_out = 8'h00;
    if(rd_dat & ~rst)
    begin
        case(addr_dat)
        (PORT_ADDR + 'h20): bus_dat_out = PORT;
        (DDR_ADDR + 'h20): bus_dat_out = DDR;
        (PIN_ADDR + 'h20): bus_dat_out = io_in;
        default: bus_dat_out = 8'h00;
        endcase
    end
end

/*
genvar cnt;
generate

for (cnt = 0; cnt < 8; cnt = cnt + 1)
begin:OUTS
    if (PINMASK[cnt])
    begin
        assign io_out[cnt] = DDR[cnt] ? PORT[cnt] : 1'b0;
    end
    else
    begin
        assign io_out[cnt] = 1'b0;
    end
end

for (cnt = 0; cnt < 8; cnt = cnt + 1)
begin:PULLUPS
    if (PULLUP_MASK[cnt] && PINMASK[cnt])
    begin
        if (PLATFORM == "XILINX")
        begin
            PULLUP PULLUP_inst (
                .O(io_out[cnt])     // PullUp output (connect directly to top-level port)
            );
        end
    end
end

for (cnt = 0; cnt < 8; cnt = cnt + 1)
begin:PULLDOWNS
    if (PULLDN_MASK[cnt] && PINMASK[cnt])
    begin
        if (PLATFORM == "XILINX")
        begin
            PULLDOWN PULLDOWN_inst (
                .O(io_out[cnt])     // PullDown output (connect directly to top-level port)
            );
        end
    end
end

endgenerate
*/

endmodule
