/*
 * This IP is the ATMEGA 10bit TIMER implementation.
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
`define PWMA    1
`define PWMB    0
`define COM0B0  4
`define COM0B1  5
`define COM0A0  6
`define COM0A1  7

//`define TCCR0B    ('h00)
`define CS00    0
`define CS01    1
`define CS02    2
`define CS03    3
`define PWM4X   7

//`define TCCR0C    ('h00)
`define PWMD    0
`undef COM0D0
`undef COM0D1
`define COM0D0  4
`define COM0D1  5

//`define TCCR0D    ('h00)
`define WGM00   0
`define WGM01   1
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
`define TOIE0   2
`define OCIE0A  6
`define OCIE0B  5
`define OCIE0C  0
`define OCIE0D  7




module atmega_tim_10bit # (
    parameter PLATFORM = "XILINX",
    parameter USE_OCRA = "TRUE",
    parameter USE_OCRB = "TRUE",
    parameter USE_OCRD = "TRUE",
    parameter BUS_ADDR_IO_LEN = 6,
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter TCCRA_ADDR = 'hc0,
    parameter TCCRB_ADDR = 'hc1,
    parameter TCCRC_ADDR = 'hc2,
    parameter TCCRD_ADDR = 'hc3,
    parameter TCCRE_ADDR = 'hc4,
    parameter TCNTL_ADDR = 'hbe,
    parameter TCH_ADDR = 'hbf,
    parameter OCRA_ADDR = 'hcf,
    parameter OCRB_ADDR = 'hd0,
    parameter OCRC_ADDR = 'hd1,
    parameter OCRD_ADDR = 'hd2,
    parameter TIMSK_ADDR = 'h72,
    parameter TIFR_ADDR = 'h19
)(
    input rst,
    input clk,
    input clk_pll,
    input pll_enabled,
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
    output ocap_io_connect,
    output ocan_io_connect,
    output ocbp_io_connect,
    output ocbn_io_connect,
    output occp_io_connect,
    output occn_io_connect,
    output ocdp_io_connect,
    output ocdn_io_connect
    );

reg [7:0]TCCRA;
reg [7:0]TCCRB;
reg [7:0]TCCRC;
reg [7:0]TCCRD;
reg [7:0]TCCRE;
reg [7:0]TCNTL;
reg [1:0]TCH;
reg [9:0]OCRA;
reg [9:0]OCRB;
reg [9:0]OCRC;
reg [9:0]OCRD;
reg [9:0]OCRA_int;
reg [9:0]OCRB_int;
reg [9:0]OCRC_int;
reg [9:0]OCRD_int;
reg [7:0]TIMSK;
reg [7:0]TIFR;

reg [1:0]TMP_REG;

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
reg clk_sys_del;

reg up_count;

wire clk_active = |TCCRB[`CS03:`CS00];

/* Prescaller */
reg [13:0]presc_cnt;

always @(posedge rst or posedge clk_pll)
begin
    if(rst)
    begin
        presc_cnt <= 10'h000;
    end
    else
    begin
        presc_cnt <= presc_cnt + 14'd1;
    end
end
/* !Prescaller */

/* Prescaller selection implementation */
wire [15:0]tim_clks = {presc_cnt, clk_pll, 1'b0};
always @*
begin
    clk_int = tim_clks[TCCRB[`CS03:`CS00]];
end
reg updt_ocr_on_top;
reg updt_ocr_on_bottom;
always @*
begin
    casex({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
        3'b0xx: // Imediate.
        begin
            updt_ocr_on_top = 1'b0;
            updt_ocr_on_bottom = 1'b0;
        end
        3'b101, 3'b111: // On bottom.
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

reg [10:0]top_value;
always @*
begin
    top_value = OCRC_int;
end

reg [10:0]t_ovf_value;
always @*
begin
    case({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
        3'b101, 3'b111: t_ovf_value = 10'h000;
        default: t_ovf_value = top_value;
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
                TCCRE_ADDR:
                begin
                    if(TCCRE_ADDR < 'h40)
                        bus_io_out = TCCRE;
                end
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR < 'h40)
                        bus_io_out = TCNTL;
                end
                TCH_ADDR:
                begin
                    if(TCH_ADDR < 'h40)
                        bus_io_out = TMP_REG;
                end
                OCRA_ADDR:
                begin
                    if(OCRA_ADDR < 'h40 && USE_OCRA == "TRUE")
                        bus_io_out = OCRA;
                end
                OCRB_ADDR:
                begin
                    if(OCRB_ADDR < 'h40 && USE_OCRB == "TRUE")
                        bus_io_out = OCRB;
                end
                OCRC_ADDR:
                begin
                    if(OCRC_ADDR < 'h40)
                        bus_io_out = OCRC;
                end
                OCRD_ADDR:
                begin
                    if(OCRD_ADDR < 'h40 && USE_OCRD == "TRUE")
                        bus_io_out = OCRD;
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
                TCCRE_ADDR:
                begin
                    if(TCCRE_ADDR >= 'h40)
                        bus_dat_out = TCCRE;
                end
                TCNTL_ADDR:
                begin
                    if(TCNTL_ADDR >= 'h40)
                        bus_dat_out = TCNTL;
                end
                TCH_ADDR:
                begin
                    if(TCH_ADDR >= 'h40)
                        bus_dat_out = TMP_REG;
                end
                OCRA_ADDR:
                begin
                    if(OCRA_ADDR >= 'h40 && USE_OCRA == "TRUE")
                        bus_dat_out = OCRA;
                end
                OCRB_ADDR:
                begin
                    if(OCRB_ADDR >= 'h40 && USE_OCRB == "TRUE")
                        bus_dat_out = OCRB;
                end
                OCRC_ADDR:
                begin
                    if(OCRC_ADDR >= 'h40)
                        bus_dat_out = OCRC;
                end
                OCRD_ADDR:
                begin
                    if(OCRD_ADDR >= 'h40 && USE_OCRD == "TRUE")
                        bus_dat_out = OCRD;
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
always @ (posedge rst or posedge pll_enabled ? clk_pll : clk)
begin
    if(rst)
    begin
        TMP_REG <= 2'h00;
        TCCRA <= 8'h00;
        TCCRB <= 8'h00;
        TCCRC <= 8'h00;
        TCCRD <= 8'h00;
        TCCRE <= 8'h00;
        TCNTL <= 8'h00;
        TCH <= 2'h0;
        OCRA <= 10'h000;
        OCRB <= 10'h000;
        OCRC <= 10'h000;
        OCRD <= 10'h000;
        OCRA_int = 10'h000;
        OCRB_int = 10'h000;
        OCRC_int = 10'h000;
        OCRD_int = 10'h000;
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
        if(((~clk_int_del & clk_int) || TCCRB[`CS03:`CS00] == 4'b0001) && TCCRB[`CS03:`CS00] != 4'b0000) // If prescaller is 1 we bypass the prescaller clock edge detector, if 0, we disable the timer, if pll is disabled, the counter clock is maximum clk/2.
        begin
            if(up_count)
            begin
                {TCH, TCNTL} <= {TCH, TCNTL} + 10'd1;
            end
            else
            begin
                {TCH, TCNTL} <= {TCH, TCNTL} - 10'd1;
            end
            // OCRA
            if(updt_ocr_on_top ? ({TCH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCH, TCNTL} == 10'h000) : ({TCH, TCNTL} == OCRA_int)))
            begin
                OCRA_int = OCRA;
            end
            if(USE_OCRA == "TRUE")
            begin
                if({TCH, TCNTL} == OCRA_int)
                begin
                    case({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
                        3'b101: oca <= ~oca;
                        default:
                        begin
                        case(OCRA_int)
                                10'h000:
                                begin
                                    case(TCCRC[`COM0D1:`COM0D0])
                                        2'h1: ocd <= ~ocd;
                                        default: ocd <= 1'b0;
                                    endcase
                                end
                                10'h3FF:
                                begin
                                    case(TCCRC[`COM0D1:`COM0D0])
                                        2'h1: ocd <= ~ocd;
                                        default: ocd <= 1'b1;
                                    endcase
                                end
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
                    if(TIMSK[`OCIE0A])
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
                else
                if(TCCRD[`WGM00] & TCCRA[`PWMA] & {TCH, TCNTL} == 10'h000)
                begin
                    oca <= 1'b0;
                end
            end
            // !OCRA
            if(USE_OCRB == "TRUE")
            begin
                // OCRB
                if(updt_ocr_on_top ? ({TCH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCH, TCNTL} == 10'h000) : ({TCH, TCNTL} == OCRB_int)))
                begin
                    OCRB_int = OCRB;
                end
                if({TCH, TCNTL} == OCRB_int)
                begin
                    case({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
                        3'b101: ocb <= ~ocb;
                        default:
                        begin
                            case(OCRB_int)
                                10'h000:
                                begin
                                    case(TCCRC[`COM0D1:`COM0D0])
                                        2'h1: ocb <= ~ocb;
                                        default: ocb <= 1'b0;
                                    endcase
                                end
                                10'h3FF:
                                begin
                                    case(TCCRC[`COM0D1:`COM0D0])
                                        2'h1: ocb <= ~ocb;
                                        default: ocb <= 1'b1;
                                    endcase
                                end
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
                    if(TIMSK[`OCIE0B])
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
                else
                if(TCCRD[`WGM00] & TCCRA[`PWMB] & {TCH, TCNTL} == 10'h000)
                begin
                    oca <= 1'b0;
                end
            end // USE_OCRB != "TRUE"
            // OCRC
            if(updt_ocr_on_top ? ({TCH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCH, TCNTL} == 10'h000) : ({TCH, TCNTL} == OCRC_int)))
            begin
                OCRC_int = OCRC;
            end
            if(USE_OCRD == "TRUE")
            begin
                // OCRD
                if(updt_ocr_on_top ? ({TCH, TCNTL} == top_value) : (updt_ocr_on_bottom ? ({TCH, TCNTL} == 10'h000) : ({TCH, TCNTL} == OCRD_int)))
                begin
                    OCRD_int = OCRD;
                end
                if({TCH, TCNTL} == OCRD_int)
                begin
                    case({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
                        3'b101: ocd <= ~ocd;
                        default:
                        begin
                            case(OCRD_int)
                                10'h000:
                                begin
                                    case(TCCRC[`COM0D1:`COM0D0])
                                        2'h1: ocd <= ~ocd;
                                        default: ocd <= 1'b0;
                                    endcase
                                end
                                10'h3FF:
                                begin
                                    case(TCCRC[`COM0D1:`COM0D0])
                                        2'h1: ocd <= ~ocd;
                                        default: ocd <= 1'b1;
                                    endcase
                                end
                                default:
                                begin
                                    if(up_count)
                                    begin
                                        case(TCCRC[`COM0D1:`COM0D0])
                                            2'h1: ocd <= ~ocd;
                                            2'h2: ocd <= 1'b0;
                                            2'h3: ocd <= 1'b1;
                                        endcase
                                    end
                                    else
                                    begin
                                        case(TCCRC[`COM0D1:`COM0D0])
                                            2'h1: ocd <= ~ocd;
                                            2'h2: ocd <= 1'b1;
                                            2'h3: ocd <= 1'b0;
                                        endcase
                                    end
                                end
                            endcase
                        end
                    endcase
                    if(TIMSK[`OCIE0D])
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
                else
                if(TCCRD[`WGM00] & TCCRA[`PWMD] & {TCH, TCNTL} == 10'h000)
                begin
                    oca <= 1'b0;
                end
            end // USE_OCRD != "TRUE"
            // TCNT overflow logick.
            if({TCH, TCNTL} == t_ovf_value)
            begin
                if(TIMSK[`TOIE0])
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
            if({TCH, TCNTL} == top_value)
            begin
                case({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
                    3'b101:
                    begin
                        up_count <= 1'b0;
                        {TCH, TCNTL} <= {TCH, TCNTL} - 10'd1;
                    end
                    default: {TCH, TCNTL} <= 10'h000;
                endcase
            end
            else if({TCH, TCNTL} == 10'h000)
            begin
                case({TCCRB[`PWM4X], TCCRD[`WGM01:`WGM00]})
                    3'b101:
                    begin
                        up_count <= 1'b1;
                        {TCH, TCNTL} <= {TCH, TCNTL} + 10'd1;
                    end
                endcase
            end
        end
        // Write registers
        clk_sys_del <= clk; // We need this because the system clk is slower than PLL clock for this timer.
        if({clk_sys_del, clk} == 2'b01 | ~pll_enabled)
        begin
            if(wr_io)
            begin
                case(addr_io)
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
                    TCCRE_ADDR:
                    begin
                        if(TCCRE_ADDR < 'h40)
                            TCCRE <= bus_io_in;
                    end
                    TCNTL_ADDR:
                    begin
                        if(TCNTL_ADDR < 'h40)
                            TCNTL <= bus_io_in;
                    end
                    TCH_ADDR:
                    begin
                        if(TCH_ADDR < 'h40)
                        begin
                            TCH <= bus_io_in;
                            TMP_REG <= bus_io_in;
                        end
                    end
                    OCRA_ADDR:
                    begin
                        if(OCRA_ADDR < 'h40 && USE_OCRA == "TRUE")
                        begin
                            OCRA <= {TMP_REG, bus_io_in};
                        end
                    end
                    OCRB_ADDR:
                    begin
                        if(OCRB_ADDR < 'h40 && USE_OCRB == "TRUE")
                        begin
                            OCRB <= {TMP_REG, bus_io_in};
                        end
                    end
                    OCRC_ADDR:
                    begin
                        if(OCRC_ADDR < 'h40)
                        begin
                            OCRC <= {TMP_REG, bus_io_in};
                        end
                    end
                    OCRD_ADDR:
                    begin
                        if(OCRD_ADDR < 'h40 && USE_OCRD == "TRUE")
                        begin
                            OCRD <= {TMP_REG, bus_io_in};
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
                endcase
            end
            if(wr_dat)
            begin
                case(addr_dat)
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
                    TCCRE_ADDR:
                    begin
                        if(TCCRE_ADDR >= 'h40)
                            TCCRE <= bus_dat_in;
                    end
                    TCNTL_ADDR:
                    begin
                        if(TCNTL_ADDR >= 'h40)
                            TCNTL <= bus_dat_in;
                    end
                    TCH_ADDR:
                    begin
                        if(TCH_ADDR >= 'h40)
                        begin
                            TCH <= bus_dat_in;
                            TMP_REG <= bus_dat_in;
                        end
                    end
                    OCRA_ADDR:
                    begin
                        if(OCRA_ADDR >= 'h40 && USE_OCRA == "TRUE")
                        begin
                            OCRA <= {TMP_REG, bus_dat_in};
                        end
                    end
                    OCRB_ADDR:
                    begin
                        if(OCRB_ADDR >= 'h40 && USE_OCRB == "TRUE")
                        begin
                            OCRB <= {TMP_REG, bus_dat_in};
                        end
                    end
                    OCRC_ADDR:
                    begin
                        if(OCRC_ADDR >= 'h40)
                        begin
                            OCRC <= {TMP_REG, bus_dat_in};
                        end
                    end
                    OCRD_ADDR:
                    begin
                        if(OCRD_ADDR >= 'h40 && USE_OCRD == "TRUE")
                        begin
                            OCRD <= {TMP_REG, bus_dat_in};
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
                endcase
            end
        end
    end
end

assign tov_int = TIFR[`TOV0];
assign ocra_int = TIFR[`OCF0A];
assign ocrb_int = TIFR[`OCF0B];
assign ocrc_int = TIFR[`OCF0C];
assign ocrd_int = TIFR[`OCF0D];

assign ocap_io_connect = (TCCRA[`COM0A1:`COM0A0] == 2'b00) ? 1'b0 : 1'b1;
assign ocan_io_connect = (TCCRA[`COM0A1:`COM0A0] == 2'b00) ? 1'b0 : 1'b1;
assign ocbp_io_connect = (TCCRA[`COM0B1:`COM0B0] == 2'b00) ? 1'b0 : 1'b1;
assign ocbn_io_connect = (TCCRA[`COM0B1:`COM0B0] == 2'b00) ? 1'b0 : 1'b1;
assign occp_io_connect = 1'b0;//(TCCRA[`COM0C1:`COM0C0] == 2'b00 || TCCRA[`COM0C1:`COM0C0] == 2'b01) ? 1'b0 : 1'b1;
assign occn_io_connect = 1'b0;//(TCCRA[`COM0C1:`COM0C0] == 2'b00 || TCCRA[`COM0C1:`COM0C0] == 2'b01) ? 1'b0 : 1'b1;
assign ocdp_io_connect = (TCCRC[`COM0D1:`COM0D0] == 2'b00) ? 1'b0 : 1'b1;
assign ocdn_io_connect = (TCCRC[`COM0D1:`COM0D0] == 2'b00) ? 1'b0 : 1'b1;

endmodule
