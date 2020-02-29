/*
 * This IP is the ATMEGA 8bit TIMER implementation.
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

//`define TIFR0   ('h15)
`define TOV0 0
`define OCF0A 1
`define OCF0B 2

//`define GTCCR   ('h23)
`define PSRSYNC 0
`define PSRASY  1
`define TSM     7

//`define TCCR0A  ('h24)
`define WGM00   0
`define WGM01   1
`define COM0B0  4
`define COM0B1  5
`define COM0A0  6
`define COM0A1  7

//`define TCCR0B  ('h25)
`define CS00    0
`define CS01    1
`define CS02    2
`define WGM02   3
`define FOC0B   6
`define FOC0A   7

//`define TCNT0   ('h26)
`define OCR0A   ('h27)
`define OCR0B   ('h28)

//`define TIMSK0  ('h6E)
`undef TOIE0
`undef OCIE0A
`undef OCIE0B
`define TOIE0   0
`define OCIE0A  1
`define OCIE0B  2



module atmega_tim_8bit # (
    parameter PLATFORM = "XILINX",
    parameter USE_OCRB = "TRUE",
    parameter BUS_ADDR_IO_LEN = 6,
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter GTCCR_ADDR = 'h23,
    parameter TCCRA_ADDR = 'h24,
    parameter TCCRB_ADDR = 'h25,
    parameter TCNT_ADDR = 'h26,
    parameter OCRA_ADDR = 'h27,
    parameter OCRB_ADDR = 'h28,
    parameter TIMSK_ADDR = 'h6E,
    parameter TIFR_ADDR = 'h15
)(
    input rst,
    input clk,
    input clk8,
    input clk64,
    input clk256,
    input clk1024,
    input [BUS_ADDR_IO_LEN-1:0]addr_io,
    input wr_io,
    input rd_io,
    input [7:0]bus_io_in,
    output reg [7:0]bus_io_out,
    input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [7:0]bus_dat_in,
    output reg [7:0]bus_dat_out,
    output tov_int,
    input tov_int_rst,
    output ocra_int,
    input ocra_int_rst,
    output ocrb_int,
    input ocrb_int_rst,

    input t,
    output reg oca,
    output reg ocb,
    output oca_io_connect,
    output ocb_io_connect
    );

reg [7:0]GTCCR;
reg [7:0]TCCRA;
reg [7:0]TCCRB;
reg [7:0]TCNT;
reg [7:0]OCRA;
reg [7:0]OCRB;
reg [7:0]OCRA_int;
reg [7:0]OCRB_int;
reg [7:0]TIMSK;
reg [7:0]TIFR;

reg tov_p;
reg tov_n;
reg ocra_p;
reg ocra_n;
reg ocrb_p;
reg ocrb_n;

//reg l1;
//reg l2;
wire t0_fall = 0;
wire t0_rising = 0;
reg clk_int;
reg clk_int_del;

reg up_count;

wire clk_active = |TCCRB[`CS02:`CS00];

/* Sampling implementation */
// Accourding to timer sampling module.
/*always @*
begin
    if(rst)
    begin
        l1 = 1'b0;
    end
    else
    if(clk)
    begin
        l1 = t;
    end
end

always @ (posedge clk)
begin
    if(rst)
    begin
        l2 = 1'b0;
    end
    else
    if(clk)
    begin
        l2 = l1;
    end
end*/
/* !Sampling implementation */

/* Prescaller selection implementation */
always @*
begin
    case(TCCRB[`CS02:`CS00])
    //3'b001: clk_int = clk;
    3'b010: clk_int = clk8;
    3'b011: clk_int = clk64;
    3'b100: clk_int = clk256;
    3'b101: clk_int = clk1024;
    3'b110: clk_int = t0_fall;
    3'b111: clk_int = t0_rising;
    default: clk_int = 1'b0;
    endcase
end
reg updt_ocr_on_top;
always @*
begin
    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
        3'h0, 3'h2: updt_ocr_on_top = 1'b0;
        default: updt_ocr_on_top = 1'b1;
    endcase
end

reg [7:0]top_value;
always @*
begin
    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
        3'h2, 3'h5, 3'h7: top_value = OCRA_int;
        default: top_value = 8'hff;
    endcase
end

reg [7:0]t_ovf_value;
always @*
begin
    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
        3'd7: t_ovf_value = top_value;
        3'd0, 3'd2, 3'd3: t_ovf_value = 8'hFF;
        default: t_ovf_value = 8'h00;
    endcase
end

// Read registers.
always @*
begin
    if(rst)
    begin
        bus_io_out = 8'h00;
        bus_dat_out = 8'h00;
    end
    else
    begin
        bus_io_out = 8'h00;
        bus_dat_out = 8'h00;
        if(rd_io)
        begin
            case(addr_io)
                GTCCR_ADDR:
                begin
                    bus_io_out = GTCCR;
                end
                TCCRA_ADDR:
                begin
                    bus_io_out = TCCRA;
                end
                TCCRB_ADDR:
                begin
                    bus_io_out = TCCRB;
                end
                TCNT_ADDR:
                begin
                    bus_io_out = TCNT;
                end
                OCRA_ADDR:
                begin
                    bus_io_out = OCRA;
                end
                OCRB_ADDR:
                begin
                    bus_io_out = OCRB;
                end
                TIFR_ADDR:
                begin
                    bus_io_out = TIFR;
                end
                default: bus_io_out = 8'h00;
            endcase
        end
        if(rd_dat)
        begin
            case(addr_dat)
                (GTCCR_ADDR + 'h20):
                begin
                    bus_dat_out = GTCCR;
                end
                (TCCRA_ADDR + 'h20):
                begin
                    bus_dat_out = TCCRA;
                end
                (TCCRB_ADDR + 'h20):
                begin
                    bus_dat_out = TCCRB;
                end
                (TCNT_ADDR + 'h20):
                begin
                    bus_dat_out = TCNT;
                end
                (OCRA_ADDR + 'h20):
                begin
                    bus_dat_out = OCRA;
                end
                (OCRB_ADDR + 'h20):
                begin
                    bus_dat_out = OCRB;
                end
                (TIFR_ADDR + 'h20):
                begin
                    bus_dat_out = TIFR;
                end
                TIMSK_ADDR:
                begin
                    bus_dat_out = TIMSK;
                end
                default: bus_dat_out = 8'h00;
            endcase
        end
    end
end

/* Set "oc" pin on specified conditions*/
always @ (posedge clk)
begin
    if(rst)
    begin
        GTCCR <= 8'h00;
        TCCRA <= 8'h00;
        TCCRB <= 8'h00;
        TCNT <= 8'h00;
        OCRA <= 8'h00;
        OCRB <= 8'h00;
        OCRA_int <= 8'h00;
        OCRB_int <= 8'h00;
        TIMSK <= 8'h00;
        TIFR <= 8'h00;
        tov_p <= 1'b0;
        tov_n <= 1'b0;
        ocra_p <= 1'b0;
        ocra_n <= 1'b0;
        ocrb_p <= 1'b0;
        ocrb_n <= 1'b0;
        oca <= 1'b0;
        ocb <= 1'b0;
        up_count <= 1'b1;
        clk_int_del <= 1'b0;
    end
    else
    begin
        if(tov_p ^ tov_n)
        begin
            TIFR[`TOV0] <= 1'b1;
            tov_n <= tov_p;
        end
        if(ocra_p ^ ocra_n)
        begin
            TIFR[`OCF0A] <= 1'b1;
            ocra_n <= ocra_p;
        end
        if(ocrb_p ^ ocrb_n)
        begin
            TIFR[`OCF0B] <= 1'b1;
            ocrb_n <= ocrb_p;
        end
        if(tov_int_rst)
        begin
            TIFR[`TOV0] <= 1'b0;
        end
        if(ocra_int_rst)
        begin
            TIFR[`OCF0A] <= 1'b0;
        end
        if(ocrb_int_rst)
        begin
            TIFR[`OCF0B] <= 1'b0;
        end
        // Sample one IO core clock once every prescaller positive edge clock.
        clk_int_del <= clk_int; // Shift prescaller clock to a delay register every IO core positive edge clock to detect prescaller positive edges.
        if(((~clk_int_del & clk_int) || TCCRB[`CS02:`CS00] == 3'b001) && TCCRB[`CS02:`CS00] != 3'b000) // if prescaller clock = IO core clock disable prescaller positive edge detector.
        begin
            if(up_count)
            begin
                TCNT <= TCNT + 8'd1;
            end
            else
            begin
                TCNT <= TCNT - 8'd1;
            end
            // OCRA
            if(updt_ocr_on_top ? (TCNT == 8'hff):(TCNT == OCRA_int))
            begin
                OCRA_int <= OCRA;
            end
            if(TCNT == OCRA_int)
            begin
                case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                    3'h2: oca <= ~oca;
                    default:
                    begin
                        case(OCRA_int)
                            8'h00:  oca <= 1'b0;
                            8'hFF:  oca <= 1'b1;
                            default:
                            begin
                                if(up_count)
                                begin
                                    case(TCCRA[`COM0A1:`COM0A0])
                                        2'h1: oca <= ~oca;
                                        2'h2: oca <= 1'b0;
                                        2'h3: oca <= 1'b1;
                                    endcase
                                end
                                else
                                begin
                                    case(TCCRA[`COM0A1:`COM0A0])
                                        2'h1: oca <= ~oca;
                                        2'h2: oca <= 1'b1;
                                        2'h3: oca <= 1'b0;
                                    endcase
                                end
                            end
                        endcase
                    end
                endcase
                if(TIMSK[`OCIE0A] == 1'b1)
                begin
                    if(ocra_p == ocra_n && clk_active == 1'b1)
                    begin
                        ocra_p <= ~ocra_p;
                    end
                    else
                    begin
                        ocra_p <= 1'b0;
                        ocra_n <= 1'b0;
                    end
                end
            end
            // !OCRA
            if(USE_OCRB == "TRUE")
            begin
                // OCRB
                if(updt_ocr_on_top ? (TCNT == 8'hff):(TCNT == OCRB_int))
                begin
                    OCRB_int <= OCRB;
                end
                if(TCNT == OCRB_int)
                begin
                    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                        3'h2: ocb <= ~ocb;
                        default:
                        begin
                            case(OCRB_int)
                                8'h00:  ocb <= 1'b0;
                                8'hFF:  ocb <= 1'b1;
                                default:
                                begin
                                    if(up_count)
                                    begin
                                        case(TCCRA[`COM0B1:`COM0B0])
                                            2'h1: ocb <= ~ocb;
                                            2'h2: ocb <= 1'b0;
                                            2'h3: ocb <= 1'b1;
                                        endcase
                                    end
                                    else
                                    begin
                                        case(TCCRA[`COM0B1:`COM0B0])
                                            2'h1: ocb <= ~ocb;
                                            2'h2: ocb <= 1'b1;
                                            2'h3: ocb <= 1'b0;
                                        endcase
                                    end
                                end
                            endcase
                        end
                    endcase
                    if(TIMSK[`OCIE0B] == 1'b1)
                    begin
                        if(ocrb_p == ocrb_n && clk_active == 1'b1)
                        begin
                            ocrb_p <= ~ocrb_p;
                        end
                    end
                    else
                    begin
                        ocrb_p <= 1'b0;
                        ocrb_n <= 1'b0;
                    end
                end
            end // USE_OCRB != "TRUE"
            // TCNT overflow logick.
            if(TCNT == t_ovf_value)
            begin
                if(TIMSK[`TOIE0] == 1'b1)
                begin
                    if(tov_p == tov_n && clk_active == 1'b1)
                    begin
                        tov_p <= ~tov_p;
                    end
                end
                else
                begin
                    tov_p <= 1'b0;
                    tov_n <= 1'b0;
                end
            end
            if(TCNT == top_value)
            begin
                case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                    3'h1, 3'h5:
                    begin
                        up_count <= 1'b0;
                        TCNT <= TCNT - 8'd1;
                    end
                    default: TCNT <= 8'h00;
                endcase
            end
            else if(TCNT == 8'h00)
            begin
                case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                    3'h1, 3'h5:
                    begin
                        up_count <= 1'b1;
                        TCNT <= TCNT + 8'd1;
                    end
                endcase
            end
        end
        // Write registers
        if(wr_io)
        begin
            case(addr_io)
                GTCCR_ADDR:
                begin
                    GTCCR <= bus_io_in;
                end
                TCCRA_ADDR:
                begin
                    TCCRA <= bus_io_in;
                end
                TCCRB_ADDR:
                begin
                    TCCRB <= bus_io_in;
                end
                TCNT_ADDR:
                begin
                    TCNT <= bus_io_in;
                end
                OCRA_ADDR:
                begin
                    OCRA <= bus_io_in;
                end
                OCRB_ADDR:
                begin
                    OCRB <= bus_io_in;
                end
                TIFR_ADDR:
                begin
                    TIFR <= TIFR & ~bus_io_in;
                end
            endcase
        end
        if(wr_dat)
        begin
            case(addr_dat)
                (GTCCR_ADDR + 'h20):
                begin
                    GTCCR <= bus_dat_in;
                end
                (TCCRA_ADDR + 'h20):
                begin
                    TCCRA <= bus_dat_in;
                end
                (TCCRB_ADDR + 'h20):
                begin
                    TCCRB <= bus_dat_in;
                end
                (TCNT_ADDR + 'h20):
                begin
                    TCNT <= bus_dat_in;
                end
                (OCRA_ADDR + 'h20):
                begin
                    OCRA <= bus_dat_in;
                end
                (OCRB_ADDR + 'h20):
                begin
                    OCRB <= bus_dat_in;
                end
                (TIFR_ADDR + 'h20):
                begin
                    TIFR <= TIFR & ~bus_dat_in;
                end
                TIMSK_ADDR:
                begin
                    TIMSK <= bus_dat_in;
                end
            endcase
        end
    end
end

assign tov_int = TIFR[`TOV0];
assign ocra_int = TIFR[`OCF0A];
assign ocrb_int = TIFR[`OCF0B];

assign oca_io_connect = (TCCRA[`COM0A1:`COM0A0] == 2'b00) ? 1'b0 : (TCCRA[`COM0A1:`COM0A0] == 2'b01 ? ((TCCRA[`WGM01:`WGM00] == 2'd1 || TCCRA[`WGM01:`WGM00] == 2'd3) ? TCCRB[`WGM02] : 1'b1) : 1'b1);
assign ocb_io_connect = (TCCRA[`COM0B1:`COM0B0] == 2'b00) ? 1'b0 : (TCCRA[`COM0B1:`COM0B0] == 2'b01 ? ((TCCRA[`WGM01:`WGM00] == 2'd1 || TCCRA[`WGM01:`WGM00] == 2'd3) ? TCCRB[`WGM02] : 1'b1) : 1'b1);

endmodule
