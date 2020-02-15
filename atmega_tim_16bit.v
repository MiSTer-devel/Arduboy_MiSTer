/*
 * This IP is the ATMEGA 16bit TIMER implementation.
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

//`define TIFR0 ('h00)
`define TOV0 0
`define OCF0A 1
`define OCF0B 2
`define OCF0C 3
`define OCF0D 4

//`define GTCCR ('h00)
`define PSRSYNC 0
`define PSRASY  1
`define TSM     7

//`define TCCR0A    ('h00)
`define WGM00   0
`define WGM01   1
`define COM0C0  2
`define COM0C1  3
`define COM0B0  4
`define COM0B1  5
`define COM0A0  6
`define COM0A1  7

`undef COM0D0
`undef COM0D1
`define COM0D0  0
`define COM0D1  1

//`define TCCR0B    ('h00)
`define CS00    0
`define CS01    1
`define CS02    2
`define WGM02   3
`define WGM03   4
`define FOC0B   6
`define FOC0A   7

//`define TCNT0 ('h00)
//`define OCR0A ('h00)
//`define OCR0B ('h00)
//`define OCR0C ('h00)
//`define OCR0D ('h00)

//`define TIMSK0    ('h00)
`undef TOIE0
`undef OCIE0A
`undef OCIE0B
`undef OCIE0C
`undef OCIE0D
`define TOIE0   0
`define OCIE0A  1
`define OCIE0B  2
`define OCIE0C  3
`define OCIE0D  4



module atmega_tim_16bit # (
    parameter PLATFORM = "XILINX",
    parameter USE_OCRB = "TRUE",
    parameter USE_OCRC = "TRUE",
    parameter USE_OCRD = "FALSE",
    parameter BUS_ADDR_IO_LEN = 6,
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter GTCCR_ADDR = 'h23,
    parameter TCCRA_ADDR = 'h80,
    parameter TCCRB_ADDR = 'h81,
    parameter TCCRC_ADDR = 'h82,
    parameter TCCRD_ADDR = 'h0,
    parameter TCNTL_ADDR = 'h84,
    parameter TCNTH_ADDR = 'h85,
    parameter ICRL_ADDR = 'h86,
    parameter ICRH_ADDR = 'h87,
    parameter OCRAL_ADDR = 'h88,
    parameter OCRAH_ADDR = 'h89,
    parameter OCRBL_ADDR = 'h8A,
    parameter OCRBH_ADDR = 'h8B,
    parameter OCRCL_ADDR = 'h8C,
    parameter OCRCH_ADDR = 'h8D,
    parameter OCRDL_ADDR = 'h0,
    parameter OCRDH_ADDR = 'h0,
    parameter TIMSK_ADDR = 'h6F,
    parameter TIFR_ADDR = 'h16
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
    output ocrc_int,
    input ocrc_int_rst,
    output ocrd_int,
    input ocrd_int_rst,

    input t,
    output reg oca,
    output reg ocb,
    output reg occ,
    output reg ocd,
    output oca_io_connect,
    output ocb_io_connect,
    output occ_io_connect,
    output ocd_io_connect
    );

reg [7:0]GTCCR;
reg [7:0]TCCRA;
reg [7:0]TCCRB;
reg [7:0]TCCRC;
reg [7:0]TCCRD;
reg [7:0]TCNTL;
reg [7:0]TCNTH;
reg [7:0]OCRAL;
reg [7:0]OCRAH;
reg [7:0]OCRBL;
reg [7:0]OCRBH;
reg [7:0]OCRCL;
reg [7:0]OCRCH;
reg [7:0]OCRDL;
reg [7:0]OCRDH;
reg [7:0]ICRL;
reg [7:0]ICRH;
reg [15:0]OCRA_int;
reg [15:0]OCRB_int;
reg [15:0]OCRC_int;
reg [15:0]OCRD_int;
reg [7:0]TIMSK;
reg [7:0]TIFR;

reg [7:0]TMP_REG;

reg tov_p;
reg tov_n;
reg ocra_p;
reg ocra_n;
reg ocrb_p;
reg ocrb_n;
reg ocrc_p;
reg ocrc_n;
reg ocrd_p;
reg ocrd_n;

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
    3'b001: clk_int = clk;
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
reg updt_ocr_on_bottom;
always @*
begin
    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
        4'd0, 4'd4, 4'd12: // Imediate.
        begin
            updt_ocr_on_top = 1'b0;
            updt_ocr_on_bottom = 1'b0;
        end
        4'd8, 4'd9: // On bottom.
        begin
            updt_ocr_on_top = 1'b0;
            updt_ocr_on_bottom = 1'b1;
        end
        default: // On TOP.
        begin
            updt_ocr_on_top = 1'b1;
            updt_ocr_on_bottom = 1'b0;
        end
    endcase
end

reg [15:0]top_value;
always @*
begin
    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
        4'd0: top_value = 16'hFFFF;
        4'd1, 4'd5: top_value = 16'h00FF;
        4'd2, 4'd6: top_value = 16'h01FF;
        4'd3, 4'd7: top_value = 16'h03FF;
        4'd8, 4'd10, 4'd12, 4'd14: top_value = {ICRH, ICRL};
        default: top_value = OCRA_int;
    endcase
end

reg [15:0]t_ovf_value;
always @*
begin
    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
        4'd5, 4'd6, 4'd7, 4'd14, 4'd15: t_ovf_value = top_value;
        4'd0, 4'd4, 4'd12: t_ovf_value = 16'hFFFF;
        default: t_ovf_value = 16'h0000;
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
                    if(GTCCR_ADDR < 'h40)
                        bus_io_out = GTCCR;
                end
                TCCRA_ADDR:
                begin
                    if(TCCRA_ADDR < 'h40)
                        bus_io_out = TCCRA;
                end
                TCCRB_ADDR:
                begin
                    if(TCCRB_ADDR < 'h40)
                        bus_io_out = TCCRB;
                end
                TCCRC_ADDR:
                begin
                    if(TCCRC_ADDR < 'h40)
                        bus_io_out = TCCRC;
                end
                TCCRD_ADDR:
                begin
                    if(TCCRD_ADDR < 'h40)
                        bus_io_out = TCCRD;
                end
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR < 'h40)
                        bus_io_out = TCNTL;
                end
                TCNTH_ADDR:
                begin
                    if(TCNTH_ADDR < 'h40)
                        bus_io_out = TMP_REG;
                end
                ICRL_ADDR:
                begin
                    if(ICRL_ADDR < 'h40)
                        bus_io_out = ICRL;
                end
                ICRH_ADDR:
                begin
                    if(ICRH_ADDR < 'h40)
                        bus_io_out = TMP_REG;
                end
                OCRAL_ADDR:
                begin
                    if(OCRAL_ADDR < 'h40)
                        bus_io_out = OCRAL;
                end
                OCRAH_ADDR:
                begin
                    if(OCRAH_ADDR < 'h40)
                        bus_io_out = OCRAH;
                end
                OCRBL_ADDR:
                begin
                    if(OCRBL_ADDR < 'h40 && USE_OCRB == "TRUE")
                        bus_io_out = OCRBL;
                end
                OCRBH_ADDR:
                begin
                    if(OCRBH_ADDR < 'h40 && USE_OCRB == "TRUE")
                        bus_io_out = OCRBH;
                end
                OCRCL_ADDR:
                begin
                    if(OCRCL_ADDR < 'h40 && USE_OCRC == "TRUE")
                        bus_io_out = OCRCL;
                end
                OCRCH_ADDR:
                begin
                    if(OCRCH_ADDR < 'h40 && USE_OCRC == "TRUE")
                        bus_io_out = OCRCH;
                end
                OCRDL_ADDR:
                begin
                    if(OCRDL_ADDR < 'h40 && USE_OCRD == "TRUE")
                        bus_io_out = OCRDL;
                end
                OCRDH_ADDR:
                begin
                    if(OCRDH_ADDR < 'h40 && USE_OCRD == "TRUE")
                        bus_io_out = OCRDH;
                end
                TIFR_ADDR:
                begin
                    if(TIFR_ADDR < 'h40)
                        bus_io_out = TIFR;
                end
                TIMSK_ADDR:
                begin
                    if(TIMSK_ADDR < 'h40)
                        bus_io_out = TIMSK;
                end
                default: bus_io_out = 8'h00;
            endcase
        end
        if(rd_dat)
        begin
            case(addr_dat)
                GTCCR_ADDR:
                begin
                    if(GTCCR_ADDR >= 'h40)
                        bus_dat_out = GTCCR;
                end
                TCCRA_ADDR:
                begin
                    if(TCCRA_ADDR >= 'h40)
                        bus_dat_out = TCCRA;
                end
                TCCRB_ADDR:
                begin
                    if(TCCRB_ADDR >= 'h40)
                        bus_dat_out = TCCRB;
                end
                TCCRC_ADDR:
                begin
                    if(TCCRC_ADDR >= 'h40)
                        bus_dat_out = TCCRC;
                end
                TCCRD_ADDR:
                begin
                    if(TCCRD_ADDR >= 'h40)
                        bus_dat_out = TCCRD;
                end
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR >= 'h40)
                        bus_dat_out = TCNTL;
                end
                TCNTH_ADDR:
                begin
                    if(TCNTH_ADDR >= 'h40)
                        bus_dat_out = TMP_REG;
                end
                ICRL_ADDR:
                begin
                    if(ICRL_ADDR >= 'h40)
                        bus_dat_out = ICRL;
                end
                ICRH_ADDR:
                begin
                    if(ICRH_ADDR >= 'h40)
                        bus_dat_out = TMP_REG;
                end
                OCRAL_ADDR:
                begin
                    if(OCRAL_ADDR >= 'h40)
                        bus_dat_out = OCRAL;
                end
                OCRAH_ADDR:
                begin
                    if(OCRAH_ADDR >= 'h40)
                        bus_dat_out = OCRAH;
                end
                OCRBL_ADDR:
                begin
                    if(OCRBL_ADDR >= 'h40 && USE_OCRB == "TRUE")
                        bus_dat_out = OCRBL;
                end
                OCRBH_ADDR:
                begin
                    if(OCRBH_ADDR >= 'h40 && USE_OCRB == "TRUE")
                        bus_dat_out = OCRBH;
                end
                OCRCL_ADDR:
                begin
                    if(OCRCL_ADDR >= 'h40 && USE_OCRC == "TRUE")
                        bus_dat_out = OCRCL;
                end
                OCRCH_ADDR:
                begin
                    if(OCRCH_ADDR >= 'h40 && USE_OCRC == "TRUE")
                        bus_dat_out = OCRCH;
                end
                OCRDL_ADDR:
                begin
                    if(OCRDL_ADDR >= 'h40 && USE_OCRD == "TRUE")
                        bus_dat_out = OCRDL;
                end
                OCRDH_ADDR:
                begin
                    if(OCRDH_ADDR >= 'h40 && USE_OCRD == "TRUE")
                        bus_dat_out = OCRDH;
                end
                TIFR_ADDR:
                begin
                    if(TIFR_ADDR >= 'h40)
                        bus_dat_out = TIFR;
                end
                TIMSK_ADDR:
                begin
                    if(TIMSK_ADDR >= 'h40)
                        bus_dat_out = TIMSK;
                end
                default: bus_dat_out = 8'h00;
            endcase
        end
    end
end

/* Set "oc" pin on specified conditions*/
always @ (posedge rst or posedge clk)
begin
    if(rst)
    begin
        TMP_REG <= 8'h00;
        GTCCR <= 8'h00;
        TCCRA <= 8'h00;
        TCCRB <= 8'h00;
        TCCRC <= 8'h00;
        TCCRD <= 8'h00;
        TCNTL <= 8'h00;
        TCNTH <= 8'h00;
        OCRAL <= 8'h00;
        OCRAH <= 8'h00;
        OCRBL <= 8'h00;
        OCRBH <= 8'h00;
        OCRCL <= 8'h00;
        OCRCH <= 8'h00;
        OCRDL <= 8'h00;
        OCRDH <= 8'h00;
        ICRL <= 8'h00;
        ICRH <= 8'h00;
        OCRA_int <= 16'h0000;
        OCRB_int <= 16'h0000;
        OCRC_int <= 16'h0000;
        OCRD_int <= 16'h0000;
        TIMSK <= 8'h00;
        TIFR <= 8'h00;
        tov_p <= 1'b0;
        tov_n <= 1'b0;
        ocra_p <= 1'b0;
        ocra_n <= 1'b0;
        ocrb_p <= 1'b0;
        ocrb_n <= 1'b0;
        ocrc_p <= 1'b0;
        ocrc_n <= 1'b0;
        ocrd_p <= 1'b0;
        ocrd_n <= 1'b0;
        oca <= 1'b0;
        ocb <= 1'b0;
        occ <= 1'b0;
        ocd <= 1'b0;
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
        if(ocrc_p ^ ocrc_n)
        begin
            TIFR[`OCF0C] <= 1'b1;
            ocrc_n <= ocrc_p;
        end
        if(ocrd_p ^ ocrd_n)
        begin
            TIFR[`OCF0D] <= 1'b1;
            ocrd_n <= ocrd_p;
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
        if(ocrc_int_rst)
        begin
            TIFR[`OCF0C] <= 1'b0;
        end
        if(ocrd_int_rst)
        begin
            TIFR[`OCF0D] <= 1'b0;
        end
        // Sample one IO core clock once every prescaller positive edge clock.
        clk_int_del <= clk_int; // Shift prescaller clock to a delay register every IO core positive edge clock to detect prescaller positive edges.
        if(((~clk_int_del & clk_int) || TCCRB[`CS02:`CS00] == 3'b001) && TCCRB[`CS02:`CS00] != 3'b000) // if prescaller clock = IO core clock disable prescaller positive edge detector.
        begin
            if(up_count)
            begin
                {TCNTH, TCNTL} <= {TCNTH, TCNTL} + 16'd1;
            end
            else
            begin
                {TCNTH, TCNTL} <= {TCNTH, TCNTL} - 16'd1;
            end
            // OCRA
            if(updt_ocr_on_top ? ({TCNTH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCNTH, TCNTL} == 10'h000) : ({TCNTH, TCNTL} == OCRA_int)))
            begin
                OCRA_int <= {OCRAH, OCRAL};
            end
            if({TCNTH, TCNTL} == OCRA_int)
            begin
                case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                    4'd4, 4'd12: oca <= ~oca;
                    default:
                    begin
                        case(OCRA_int)
                            16'h0000:   oca <= 1'b0;
                            16'hFFFF:   oca <= 1'b1;
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
                end
                else
                begin
                    ocra_p <= 1'b0;
                    ocra_n <= 1'b0;
                end
            end
            // !OCRA
            if(USE_OCRB == "TRUE")
            begin
                // OCRB
                if(updt_ocr_on_top ? ({TCNTH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCNTH, TCNTL} == 10'h0000) : ({TCNTH, TCNTL} == OCRB_int)))
                begin
                    OCRB_int <= {OCRBH, OCRBL};
                end
                if({TCNTH, TCNTL} == OCRB_int)
                begin
                    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                        4'd4, 4'd12: ocb <= ~ocb;
                        default:
                        begin
                            case(OCRB_int)
                                16'h0000:   ocb <= 1'b0;
                                16'hFFFF:   ocb <= 1'b1;
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
            if(USE_OCRC == "TRUE")
            begin
                // OCRB
                if(updt_ocr_on_top ? ({TCNTH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCNTH, TCNTL} == 10'h0000) : ({TCNTH, TCNTL} == OCRC_int)))
                begin
                    OCRC_int <= {OCRCH, OCRCL};
                end
                if({TCNTH, TCNTL} == OCRC_int)
                begin
                    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                        4'd4, 4'd12: occ <= ~occ;
                        default:
                        begin
                            case(OCRC_int)
                                16'h0000:   occ <= 1'b0;
                                16'hFFFF:   occ <= 1'b1;
                                default:
                                begin
                                    if(up_count)
                                    begin
                                        case(TCCRA[`COM0C1:`COM0C0])
                                            2'h1: occ <= ~occ;
                                            2'h2: occ <= 1'b0;
                                            2'h3: occ <= 1'b1;
                                        endcase
                                    end
                                    else
                                    begin
                                        case(TCCRA[`COM0C1:`COM0C0])
                                            2'h1: occ <= ~occ;
                                            2'h2: occ <= 1'b1;
                                            2'h3: occ <= 1'b0;
                                        endcase
                                    end
                                end
                            endcase
                        end
                    endcase
                    if(TIMSK[`OCIE0C] == 1'b1)
                    begin
                        if(ocrc_p == ocrc_n && clk_active == 1'b1)
                        begin
                            ocrc_p <= ~ocrc_p;
                        end
                    end
                    else
                    begin
                        ocrc_p <= 1'b0;
                        ocrc_n <= 1'b0;
                    end
                end
            end // USE_OCRC != "TRUE"
            if(USE_OCRD == "TRUE")
            begin
                // OCRB
                if(updt_ocr_on_top ? ({TCNTH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCNTH, TCNTL} == 10'h0000) : ({TCNTH, TCNTL} == OCRD_int)))
                begin
                    OCRD_int <= {OCRDH, OCRDL};
                end
                if({TCNTH, TCNTL} == OCRD_int)
                begin
                    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                        4'd4, 4'd12: ocd <= ~ocd;
                        default:
                        begin
                            case(OCRB_int)
                                16'h0000:   ocd <= 1'b0;
                                16'hFFFF:   ocd <= 1'b1;
                                default:
                                begin
                                    if(up_count)
                                    begin
                                        case(TCCRA[`COM0D1:`COM0D0])
                                            2'h1: ocd <= ~ocd;
                                            2'h2: ocd <= 1'b0;
                                            2'h3: ocd <= 1'b1;
                                        endcase
                                    end
                                    else
                                    begin
                                        case(TCCRA[`COM0D1:`COM0D0])
                                            2'h1: ocd <= ~ocd;
                                            2'h2: ocd <= 1'b1;
                                            2'h3: ocd <= 1'b0;
                                        endcase
                                    end
                                end
                            endcase
                        end
                    endcase
                    if(TIMSK[`OCIE0D] == 1'b1)
                    begin
                        if(ocrd_p == ocrd_n && clk_active == 1'b1)
                        begin
                            ocrd_p <= ~ocrd_p;
                        end
                    end
                    else
                    begin
                        ocrd_p <= 1'b0;
                        ocrd_n <= 1'b0;
                    end
                end
            end // USE_OCRD != "TRUE"
            // TCNT overflow logick.
            if({TCNTH, TCNTL} == t_ovf_value)
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
            if({TCNTH, TCNTL} == top_value)
            begin
                case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                    4'd1, 4'd3, 4'd10, 4'd11:
                    begin
                        up_count <= 1'b0;
                        {TCNTH, TCNTL} <= {TCNTH, TCNTL} - 16'd1;
                    end
                    default: {TCNTH, TCNTL} <= 16'h0000;
                endcase
            end
            else if({TCNTH, TCNTL} == 16'h0000)
            begin
                case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                    4'd1, 4'd3, 4'd10, 4'd11:
                    begin
                        up_count <= 1'b1;
                        {TCNTH, TCNTL} <= {TCNTH, TCNTL} + 16'd1;
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
                    if(GTCCR_ADDR < 'h40)
                        GTCCR <= bus_io_in;
                end
                TCCRA_ADDR:
                begin
                    if(TCCRA_ADDR < 'h40)
                        TCCRA <= bus_io_in;
                end
                TCCRB_ADDR:
                begin
                    if(TCCRB_ADDR < 'h40)
                        TCCRB <= bus_io_in;
                end
                TCCRC_ADDR:
                begin
                    if(TCCRC_ADDR < 'h40)
                        TCCRC <= bus_io_in;
                end
                TCCRD_ADDR:
                begin
                    if(TCCRD_ADDR < 'h40)
                        TCCRD <= bus_io_in;
                end
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR < 'h40)
                    begin
                        TCNTL <= bus_io_in;
                        TCNTH <= TMP_REG;
                    end
                end
                ICRL_ADDR:
                begin
                    if(ICRL_ADDR < 'h40)
                    begin
                        ICRL <= bus_io_in;
                        ICRH <= TMP_REG;
                    end
                end
                OCRAL_ADDR:
                begin
                    if(OCRAL_ADDR < 'h40)
                        OCRAL <= bus_io_in;
                        OCRAH <= TMP_REG;
                end
                OCRBL_ADDR:
                begin
                    if(OCRBL_ADDR < 'h40 && USE_OCRB == "TRUE")
                    begin
                        OCRBL <= bus_io_in;
                        OCRBH <= TMP_REG;
                    end
                end
                OCRCL_ADDR:
                begin
                    if(OCRCL_ADDR < 'h40 && USE_OCRC == "TRUE")
                    begin
                        OCRCL <= bus_io_in;
                        OCRCH <= TMP_REG;
                    end
                end
                OCRDL_ADDR:
                begin
                    if(OCRDL_ADDR < 'h40 && USE_OCRD == "TRUE")
                    begin
                        OCRDL <= bus_io_in;
                        OCRDH <= TMP_REG;
                    end
                end
                TIFR_ADDR:
                begin
                    if(TIFR_ADDR < 'h40)
                        TIFR <= TIFR & ~bus_io_in;
                end
                TIMSK_ADDR:
                begin
                    if(TIMSK_ADDR < 'h40)
                        TIMSK <= bus_io_in;
                end
                TCNTH_ADDR, ICRH_ADDR, OCRAH_ADDR, OCRBH_ADDR, OCRCH_ADDR, OCRDH_ADDR:
                begin
                    if(TCNTH_ADDR < 'h40)
                        TMP_REG <= bus_io_in;
                end
            endcase
        end
        if(wr_dat)
        begin
            case(addr_dat)
                GTCCR_ADDR:
                begin
                    if(GTCCR_ADDR >= 'h40)
                        GTCCR <= bus_dat_in;
                end
                TCCRA_ADDR:
                begin
                    if(TCCRA_ADDR >= 'h40)
                        TCCRA <= bus_dat_in;
                end
                TCCRB_ADDR:
                begin
                    if(TCCRB_ADDR >= 'h40)
                        TCCRB <= bus_dat_in;
                end
                TCCRC_ADDR:
                begin
                    if(TCCRC_ADDR >= 'h40)
                        TCCRC <= bus_dat_in;
                end
                TCCRD_ADDR:
                begin
                    if(TCCRD_ADDR >= 'h40)
                        TCCRD <= bus_dat_in;
                end
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR >= 'h40)
                    begin
                        TCNTL <= bus_dat_in;
                        TCNTH <= TMP_REG;
                    end
                end
                ICRL_ADDR:
                begin
                    if(ICRL_ADDR >= 'h40)
                    begin
                        ICRL <= bus_dat_in;
                        ICRH <= TMP_REG;
                    end
                end
                OCRAL_ADDR:
                begin
                    if(OCRAL_ADDR >= 'h40)
                        OCRAL <= bus_dat_in;
                        OCRAH <= TMP_REG;
                end
                OCRBL_ADDR:
                begin
                    if(OCRBL_ADDR >= 'h40 && USE_OCRB == "TRUE")
                    begin
                        OCRBL <= bus_dat_in;
                        OCRBH <= TMP_REG;
                    end
                end
                OCRCL_ADDR:
                begin
                    if(OCRCL_ADDR >= 'h40 && USE_OCRC == "TRUE")
                    begin
                        OCRCL <= bus_dat_in;
                        OCRCH <= TMP_REG;
                    end
                end
                OCRDL_ADDR:
                begin
                    if(OCRDL_ADDR >= 'h40 && USE_OCRD == "TRUE")
                    begin
                        OCRDL <= bus_dat_in;
                        OCRDH <= TMP_REG;
                    end
                end
                TIFR_ADDR:
                begin
                    if(TIFR_ADDR >= 'h40)
                        TIFR <= TIFR & ~bus_dat_in;
                end
                TIMSK_ADDR:
                begin
                    if(TIMSK_ADDR >= 'h40)
                        TIMSK <= bus_dat_in;
                end
                TCNTH_ADDR, ICRH_ADDR, OCRAH_ADDR, OCRBH_ADDR, OCRCH_ADDR, OCRDH_ADDR:
                begin
                    if(TCNTH_ADDR >= 'h40)
                        TMP_REG <= bus_dat_in;
                end
            endcase
        end
        if(rd_io)
        begin
            case(addr_io)
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR < 'h40)
                        TMP_REG <= TCNTL;
                end
                ICRL_ADDR:
                begin
                    if(ICRL_ADDR < 'h40)
                        TMP_REG <= ICRL;
                end
            endcase
        end
        if(rd_dat)
        begin
            case(addr_dat)
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR >= 'h40)
                        TMP_REG <= TCNTL;
                end
                ICRL_ADDR:
                begin
                    if(ICRL_ADDR >= 'h40)
                        TMP_REG <= ICRL;
                end
            endcase
        end
    end
end

assign tov_int = TIFR[`TOV0];
assign ocra_int = TIFR[`OCF0A];
assign ocrb_int = TIFR[`OCF0B];
assign ocrc_int = TIFR[`OCF0C];
assign ocrd_int = TIFR[`OCF0D];

assign oca_io_connect = (TCCRA[`COM0A1:`COM0A0] == 2'b00) ? 1'b0 : (TCCRA[`COM0A1:`COM0A0] == 2'b01 ? (({TCCRA[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]} == 4'd14 || {TCCRA[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]} == 4'd15) ? 1'b1 : 1'b0) : 1'b1);
assign ocb_io_connect = (TCCRA[`COM0B1:`COM0B0] == 2'b00 || TCCRA[`COM0B1:`COM0B0] == 2'b01) ? 1'b0 : 1'b1;
assign occ_io_connect = (TCCRA[`COM0C1:`COM0C0] == 2'b00 || TCCRA[`COM0C1:`COM0C0] == 2'b01) ? 1'b0 : 1'b1;
assign ocd_io_connect = (TCCRA[`COM0D1:`COM0D0] == 2'b00 || TCCRA[`COM0D1:`COM0D0] == 2'b01) ? 1'b0 : 1'b1;

endmodule
