/*
 * This IP is the ATMEGA SPI implementation.
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

`timescale 1ns / 1ps

`define ATMEGA_SPI_SPCR_INT_EN_bp		7
`define ATMEGA_SPI_SPCR_EN_bp			6
`define ATMEGA_SPI_SPCR_DORD_bp			5
`define ATMEGA_SPI_SPCR_MSTR_bp			4
`define ATMEGA_SPI_SPCR_CPOL_bp			3
`define ATMEGA_SPI_SPCR_CPHA_bp			2 // Not implemented because are very rare the devices that implement this mode.
`define ATMEGA_SPI_SPCR_SPR1_bp			1
`define ATMEGA_SPI_SPCR_SPR0_bp			0

`define ATMEGA_SPI_SPSR_SPIF_bp			7
`define ATMEGA_SPI_SPSR_WCOL_bp			6
`define ATMEGA_SPI_SPSR_SPI2X_bp		0


module atmega_spi_m # (
	parameter PLATFORM = "XILINX",
	parameter BUS_ADDR_IO_LEN = 6,
	parameter SPCR_ADDR = 0,
	parameter SPSR_ADDR = 1,
	parameter SPDR_ADDR = 2,
	parameter DINAMIC_BAUDRATE = "TRUE",
	parameter BAUDRATE_DIVIDER = 1
)(
	input rst,
	input clk,
	input [BUS_ADDR_IO_LEN-1:0]addr,
	input wr,
	input rd,
	input [7:0]bus_in,
	output reg [7:0]bus_out,
	output int,
	input int_rst,
	output io_connect,
	output io_conn_slave,

	output scl,
	input miso,
	output mosi
	);


reg [7:0]SPCR;
reg [7:0]SPSR;
reg [7:0]SPDR;

reg spi_active;
reg sck_active;
	
localparam WORD_LEN = 4'h8;
localparam PRESCALLER_SIZE = 8;

reg stc_p;
reg stc_n;

reg [7:0]rx_shift_reg;
reg [7:0]tx_shift_reg;

reg [3:0]bit_cnt;

reg [7:0]prescaller_cnt;
reg sckint;
reg [7:0]prescdemux;

always @ (*)
begin
	bus_out = 8'b00;
	if(rd)
	begin
		case(addr)
		SPCR_ADDR: bus_out = SPCR;
		SPSR_ADDR: bus_out = SPSR;
		SPDR_ADDR: bus_out = SPDR;
		endcase
	end
end

always @ (*)
begin
	case({SPSR[`ATMEGA_SPI_SPSR_SPI2X_bp], SPCR[`ATMEGA_SPI_SPCR_SPR1_bp], SPCR[`ATMEGA_SPI_SPCR_SPR0_bp]})
	3'b000: prescdemux <= 1;
	3'b001: prescdemux <= 8;
	3'b010: prescdemux <= 32;
	3'b011: prescdemux <= 64;
	3'b100: prescdemux <= 0;
	3'b101: prescdemux <= 4;
	3'b110: prescdemux <= 16;
	3'b111: prescdemux <= 32;
	endcase
end

reg rd_old;
always @ (posedge clk)
begin
	if(rst)
	begin
		stc_n <= 1'b0;
		SPCR <= 8'h00;
		SPSR <= 8'h00;
		SPDR <= 8'h00;
		tx_shift_reg <= 8'h00;
		rx_shift_reg <= 8'hFF;
		prescaller_cnt <= 8'h00;
		bit_cnt <= WORD_LEN;
		sckint <= 1'b0;
		stc_p <= 1'b0;
		spi_active <= 1'b0;
		sck_active <= 1'b0;
		rd_old <= 1'b0;
	end
	else
	begin
		if(SPCR[`ATMEGA_SPI_SPCR_EN_bp] & spi_active)
		begin
			if(prescaller_cnt)
			begin
				prescaller_cnt <= prescaller_cnt - 1;
			end
			else
			begin
				prescaller_cnt <= prescdemux;
				sckint <= ~sckint;
//rx
				if(~sckint)
				begin
					bit_cnt <= bit_cnt + 4'd1;
					if(bit_cnt == WORD_LEN - 1)
					begin
						if(SPCR[`ATMEGA_SPI_SPCR_DORD_bp] == 1'b0)
						begin
							SPDR <= {rx_shift_reg[WORD_LEN - 2:0], miso};
						end
						else
						begin
							SPDR <= {miso, rx_shift_reg[WORD_LEN - 1:0]};
						end
					end
					if(SPCR[`ATMEGA_SPI_SPCR_DORD_bp] == 1'b0)
					begin
						rx_shift_reg <= {rx_shift_reg[WORD_LEN - 2:0], miso};
					end
					else
					begin
						rx_shift_reg <= {miso, rx_shift_reg[WORD_LEN - 1:0]};
					end
				end
				else
				begin
//tx
					if(~SPCR[`ATMEGA_SPI_SPCR_DORD_bp])
					begin
						tx_shift_reg <= {tx_shift_reg[WORD_LEN - 2:0], 1'b0};
					end
					else
					begin
						tx_shift_reg <= {1'b0, tx_shift_reg[WORD_LEN - 1:1]};
					end
				end
			end
		end
		rd_old <= rd; // Read takes two cycles, we clear the flag after second cycle.
		if(int_rst)
		begin
			SPSR[`ATMEGA_SPI_SPSR_SPIF_bp] <= 1'b0;
		end
		else
		if(rd_old & ~rd)
		begin
			case(addr)
			SPSR_ADDR: SPSR[`ATMEGA_SPI_SPSR_SPIF_bp] <= 1'b0;
			endcase
		end
		else
		if(stc_p ^ stc_n)
		begin
			SPSR[`ATMEGA_SPI_SPSR_SPIF_bp] <= 1'b1;
			stc_n <= stc_p;
			sck_active <= 1'b0;
		end	
		if(bit_cnt == WORD_LEN)
		begin
			if(wr)
			begin
				case(addr)
				SPCR_ADDR: SPCR <= bus_in;
				SPSR_ADDR: SPSR <= bus_in;
				SPDR_ADDR: 
				begin
					if(SPCR[`ATMEGA_SPI_SPCR_EN_bp])
					begin
						tx_shift_reg <= bus_in;
						bit_cnt <= 4'h0;
						prescaller_cnt <= prescdemux;
						sckint <= 1'b0;
						spi_active <= 1'b1;
						sck_active <= 1'b1;
					end
				end
				endcase
			end
			if(stc_p == stc_n && spi_active)
			begin
				stc_p <= ~stc_p;
				spi_active <= 1'b0;
			end
		end
	end
end

assign int = SPCR[`ATMEGA_SPI_SPCR_INT_EN_bp] ? SPSR[`ATMEGA_SPI_SPSR_SPIF_bp] : 1'b0;
assign scl = SPCR[`ATMEGA_SPI_SPCR_EN_bp] ? ((SPCR[`ATMEGA_SPI_SPCR_CPOL_bp]) ? (sck_active ? ~sckint : 1'b1) : (sck_active ? sckint : 1'b0)) : 1'b1;
assign mosi = (SPCR[`ATMEGA_SPI_SPCR_EN_bp]) ? (SPCR[`ATMEGA_SPI_SPCR_DORD_bp] ? tx_shift_reg[0] : tx_shift_reg[WORD_LEN - 1]) : 1'b1;
assign io_connect = SPCR[`ATMEGA_SPI_SPCR_EN_bp];
assign io_conn_slave = ~SPCR[`ATMEGA_SPI_SPCR_MSTR_bp];

endmodule
