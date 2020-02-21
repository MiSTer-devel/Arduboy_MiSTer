/*
 * This IP is the MEGA/XMEGA ATMEGA32A4 implementation.
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

`include "xmega_v.v"

`define PLATFORM "XILINX"

/* For ATMEGA32U4 "MEGA_ENHANCED_128K" family*/
`define CORE_TYPE               `MEGA_ENHANCED_128K
`define ROM_ADDR_WIDTH          14
`define BUS_ADDR_DATA_LEN       12
`define RAM_ADDR_WIDTH          12
`define RESERVED_RAM_FOR_IO     'h100


`define VECTOR_INT_TABLE_SIZE   42
`define WATCHDOG_CNT_WIDTH      0//27

module io_bus_dmux # (
        parameter NR_OF_BUSSES_IN = 1
        )(
        input [(NR_OF_BUSSES_IN * 8) - 1 : 0]bus_in,
        output reg[7:0]bus_out
        );
reg [NR_OF_BUSSES_IN - 1 : 0]tmp_busses_bits;
integer cnt_add_busses;
integer cnt_add_bits;
        always @*
        begin
            for(cnt_add_bits = 0; cnt_add_bits < 8; cnt_add_bits = cnt_add_bits + 1)
            begin: DMUX_IO_DATA_BITS
                for(cnt_add_busses = 0; cnt_add_busses < NR_OF_BUSSES_IN; cnt_add_busses = cnt_add_busses + 1)
                begin: DMUX_IO_DATA_BUSES
                    tmp_busses_bits[cnt_add_busses] = bus_in[(cnt_add_busses * 8) + cnt_add_bits];
                end
                bus_out[cnt_add_bits] = |tmp_busses_bits;
            end
        end
endmodule

module tim_013_prescaller (
    input rst,
    input clk,
    output clk8,
    output clk64,
    output clk256,
    output clk1024
);
reg [9:0]cnt;

always @ (posedge clk)
begin
    if(rst)
    begin
        cnt <= 10'h000;
    end
    else
    begin
        cnt <= cnt + 10'd1;
    end
end

assign clk8 = cnt[2];
assign clk64 = cnt[5];
assign clk256 = cnt[7];
assign clk1024 = cnt[9];

endmodule

module atmega32u4(
    input rst_in,
    input clk,
    input clk_pll,
    output [`ROM_ADDR_WIDTH-1:0] pgm_addr,
    input [15:0] pgm_data,
    input [5:0] buttons,
    output [2:0] RGB,
    output Buzzer1, Buzzer2, DC, spi_scl, spi_mosi
    );

reg rst;

always @ (posedge clk)
begin
    rst <= rst_in;
end

assign pf_in[6] = buttons[0]; // BUTTON RIGHT
assign pf_in[5] = buttons[1]; // BUTTON LEFT
assign pf_in[4] = buttons[2]; // BUTTON DOWN
assign pf_in[7] = buttons[3]; // BUTTON UP
assign pe_in[6] = buttons[4]; // BUTTON A
assign pb_in[4] = buttons[5]; // BUTTON B

// RGB LED is common anode (ie. HIGH = OFF)
assign RGB[2] = tim1_ocb_io_connact ? tim1_ocb : ~(pb_out[6]);
assign RGB[1] = tim0_oca_io_connact ? ~tim0_oca : ~(pb_out[7]);
assign RGB[0] = tim1_oca_io_connact ? tim1_oca : ~(pb_out[5]);

assign Buzzer1 = pc_out[6];
assign Buzzer2 = tim4_ocap_io_connact ? tim4_oca : 1'b0;
assign DC = pd_out[4];

wire core_clk = clk;
wire wdt_rst;

wire [`BUS_ADDR_DATA_LEN-1:0]data_addr;
wire [7:0]core_data_out;
wire data_write;
wire [7:0]core_data_in;
wire data_read;
wire [5:0]io_addr;
wire [7:0]io_out;
wire io_write;
wire [7:0]io_in;
wire io_read;

wire [7:0]pb_in;
wire [7:0]pc_in;
wire [7:0]pd_in;
wire [7:0]pe_in;
wire [7:0]pf_in;
wire [7:0]pb_out;
wire [7:0]pc_out;
wire [7:0]pd_out;
wire [7:0]pe_out;
wire [7:0]pf_out;

wire tim0_oca_io_connact;
wire tim0_ocb_io_connact;
wire tim1_oca_io_connact;
wire tim1_ocb_io_connact;
wire tim1_occ_io_connact;
wire tim3_oca_io_connact;
wire tim4_ocap_io_connact;
wire tim4_ocan_io_connact;
wire tim4_ocbp_io_connact;
wire tim4_ocbn_io_connact;
wire tim4_occp_io_connact;
wire tim4_occn_io_connact;
wire tim4_ocdp_io_connact;
wire tim4_ocdn_io_connact;
wire spi_io_connect;
wire io_conn_slave;

wire pll_enabled;
wire tim0_oca;
wire tim0_ocb;
wire tim1_oca;
wire tim1_ocb;
wire tim1_occ;
wire tim3_oca;
wire tim4_oca;
wire tim4_ocb;
wire tim4_occ;
wire tim4_ocd;
wire spi_miso;
wire usb_ck_out;
wire tim_ck_out;

/* Interrupt wires */
wire ram_sel = |data_addr[`BUS_ADDR_DATA_LEN-1:8];
wire int_int0 = 0;
wire int_int1 = 0;
wire int_int2 = 0;
wire int_int3 = 0;
wire int_reserved0 = 0;
wire int_reserved1 = 0;
wire int_int6 = 0;
wire int_reserved3 = 0;
wire int_pcint0 = 0;
wire int_usb_general = 0;
wire int_usb_endpoint = 0;
wire int_wdt = 0;
wire int_reserved4 = 0;
wire int_reserved5 = 0;
wire int_reserved6 = 0;
wire int_timer1_capt = 0;
wire int_timer1_compa;
wire int_timer1_compb;
wire int_timer1_compc;
wire int_timer1_ovf;
wire int_timer0_compa;
wire int_timer0_compb;
wire int_timer0_ovf;
wire int_spi_stc;
wire int_usart1_rx = 0;
wire int_usart1_udre = 0;
wire int_usart1_tx = 0;
wire int_analog_comp = 0;
wire int_adc = 0;
wire int_ee_ready;
wire int_timer3_capt = 0;
wire int_timer3_compa;
wire int_timer3_compb = 0;
wire int_timer3_compc = 0;
wire int_timer3_ovf;
wire int_twi = 0;
wire int_spm_ready = 0;
wire int_timer4_compa;
wire int_timer4_compb;
wire int_timer4_compd;
wire int_timer4_ovf;
wire int_timer4_fpf = 0;
/* Interrupt reset wires */
wire int_int0_rst = 0;
wire int_int1_rst = 0;
wire int_int2_rst = 0;
wire int_int3_rst = 0;
wire int_reserved0_rst = 0;
wire int_reserved1_rst = 0;
wire int_int6_rst = 0;
wire int_reserved3_rst = 0;
wire int_pcint0_rst = 0;
wire int_usb_general_rst = 0;
wire int_usb_endpoint_rst = 0;
wire int_wdt_rst = 0;
wire int_reserved4_rst = 0;
wire int_reserved5_rst = 0;
wire int_reserved6_rst = 0;
wire int_timer1_capt_rst;
wire int_timer1_compa_rst;
wire int_timer1_compb_rst;
wire int_timer1_compc_rst;
wire int_timer1_ovf_rst;
wire int_timer0_compa_rst;
wire int_timer0_compb_rst;
wire int_timer0_ovf_rst;
wire int_spi_stc_rst;
wire int_usart1_rx_rst = 0;
wire int_usart1_udre_rst = 0;
wire int_usart1_tx_rst = 0;
wire int_analog_comp_rst = 0;
wire int_adc_rst = 0;
wire int_ee_ready_rst;
wire int_timer3_capt_rst = 0;
wire int_timer3_compa_rst;
wire int_timer3_compb_rst = 0;
wire int_timer3_compc_rst = 0;
wire int_timer3_ovf_rst;
wire int_twi_rst = 0;
wire int_spm_ready_rst = 0;
wire int_timer4_compa_rst;
wire int_timer4_compb_rst;
wire int_timer4_compd_rst;
wire int_timer4_ovf_rst;
wire int_timer4_fpf_rst = 0;

// PORTB
wire [7:0]io_pb_d_out;
atmega_pio # (
    .PLATFORM(`PLATFORM),
    .BUS_ADDR_DATA_LEN(6),
    .PORT_ADDR(6'h05),
    .DDR_ADDR(6'h04),
    .PIN_ADDR(6'h03),
    .PINMASK(8'b11110000),
    .PULLUP_MASK(8'b00010000),
    .PULLDN_MASK(8'b00000000)
)pio_b(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_pb_d_out),

    .io_in(pb_in),
    .io_out(pb_out)
    );

// PORTC
wire [7:0]io_pc_d_out;
atmega_pio # (
    .PLATFORM(`PLATFORM),
    .BUS_ADDR_DATA_LEN(6),
    .PORT_ADDR(6'h08),
    .DDR_ADDR(6'h07),
    .PIN_ADDR(6'h06),
    .PINMASK(8'b11000000),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000)
)pio_c(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_pc_d_out),

    .io_in(pc_in),
    .io_out(pc_out)
    );

// PORTD
wire [7:0]io_pd_d_out;
atmega_pio # (
    .PLATFORM(`PLATFORM),
    .BUS_ADDR_DATA_LEN(6),
    .PORT_ADDR(6'h0b),
    .DDR_ADDR(6'h0a),
    .PIN_ADDR(6'h09),
    .PINMASK(8'b11111111),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000)
)pio_d(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_pd_d_out),

    .io_in(pd_in),
    .io_out(pd_out)
    );

// PORTE
wire [7:0]io_pe_d_out;
atmega_pio # (
    .PLATFORM(`PLATFORM),
    .BUS_ADDR_DATA_LEN(6),
    .PORT_ADDR(6'h0e),
    .DDR_ADDR(6'h0d),
    .PIN_ADDR(6'h0c),
    .PINMASK(8'b01000000),
    .PULLUP_MASK(8'b01000000),
    .PULLDN_MASK(8'b00000000)
)pio_e(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_pe_d_out),

    .io_in(pe_in),
    .io_out(pe_out)
    );

// PORTF
wire [7:0]io_pf_d_out;
atmega_pio # (
    .PLATFORM(`PLATFORM),
    .BUS_ADDR_DATA_LEN(6),
    .PORT_ADDR(6'h11),
    .DDR_ADDR(6'h10),
    .PIN_ADDR(6'h0f),
    .PINMASK(8'b11110011),
    .PULLUP_MASK(8'b11110000),
    .PULLDN_MASK(8'b00000000)
)pio_f(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_pf_d_out),

    .io_in(pf_in),
    .io_out(pf_out)
    );

wire [7:0]io_spi_d_out;
atmega_spi_m # (
    .PLATFORM(`PLATFORM),
    .BUS_ADDR_DATA_LEN(6),
    .SPCR_ADDR(6'h2c),
    .SPSR_ADDR(6'h2d),
    .SPDR_ADDR(6'h2e),
    .DINAMIC_BAUDRATE("TRUE"),
    .BAUDRATE_DIVIDER(0)
)spi(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_spi_d_out),
    .int_out(int_spi_stc),
    .int_rst(int_spi_stc_rst),
    .io_connect(spi_io_connect),
    .io_conn_slave(io_conn_slave),

    .scl(spi_scl),
    .miso(spi_miso),
    .mosi(spi_mosi)
    );

wire clk8;
wire clk64;
wire clk256;
wire clk1024;
tim_013_prescaller tim_013_prescaller_inst(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024)
);

wire [7:0]io_tim0_d_out;
wire [7:0]dat_tim0_d_out;
atmega_tim_8bit # (
    .PLATFORM("XILINX"),
    .USE_OCRB("TRUE"),
    .BUS_ADDR_IO_LEN(6),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h23),
    .TCCRA_ADDR('h24),
    .TCCRB_ADDR('h25),
    .TCNT_ADDR('h26),
    .OCRA_ADDR('h27),
    .OCRB_ADDR('h28),
    .TIMSK_ADDR('h6E),
    .TIFR_ADDR('h15)
)tim_0(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_io(io_addr),
    .wr_io(io_write),
    .rd_io(io_read),
    .bus_io_in(io_out),
    .bus_io_out(io_tim0_d_out),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim0_d_out),
    .tov_int(int_timer0_ovf),
    .tov_int_rst(int_timer0_ovf_rst),
    .ocra_int(int_timer0_compa),
    .ocra_int_rst(int_timer0_compa_rst),
    .ocrb_int(int_timer0_compb),
    .ocrb_int_rst(int_timer0_compb_rst),

    .t(),
    .oca(tim0_oca),
    .ocb(tim0_ocb),
    .oca_io_connect(tim0_oca_io_connact),
    .ocb_io_connect(tim0_ocb_io_connact)
    );

wire [7:0]io_tim1_d_out;
wire [7:0]dat_tim1_d_out;
atmega_tim_16bit # (
    .PLATFORM("XILINX"),
    .USE_OCRB("TRUE"),
    .USE_OCRC("TRUE"),
    .BUS_ADDR_IO_LEN(6),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h23),
    .TCCRA_ADDR('h80),
    .TCCRB_ADDR('h81),
    .TCCRC_ADDR('h82),
    .TCNTL_ADDR('h84),
    .TCNTH_ADDR('h85),
    .ICRL_ADDR('h86),
    .ICRH_ADDR('h87),
    .OCRAL_ADDR('h88),
    .OCRAH_ADDR('h89),
    .OCRBL_ADDR('h8A),
    .OCRBH_ADDR('h8B),
    .OCRCL_ADDR('h8C),
    .OCRCH_ADDR('h8D),
    .TIMSK_ADDR('h6F),
    .TIFR_ADDR('h16)
)tim_1(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_io(io_addr),
    .wr_io(io_write),
    .rd_io(io_read),
    .bus_io_in(io_out),
    .bus_io_out(io_tim1_d_out),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim1_d_out),
    .tov_int(int_timer1_ovf),
    .tov_int_rst(int_timer1_ovf_rst),
    .ocra_int(int_timer1_compa),
    .ocra_int_rst(int_timer1_compa_rst),
    .ocrb_int(int_timer1_compb),
    .ocrb_int_rst(int_timer1_compb_rst),
    .ocrc_int(int_timer1_compc),
    .ocrc_int_rst(int_timer1_compc_rst),

    .t(),
    .oca(tim1_oca),
    .ocb(tim1_ocb),
    .occ(tim1_occ),
    .oca_io_connect(tim1_oca_io_connact),
    .ocb_io_connect(tim1_ocb_io_connact),
    .occ_io_connect(tim1_occ_io_connact)
    );

wire [7:0]io_tim3_d_out;
wire [7:0]dat_tim3_d_out;
atmega_tim_16bit # (
    .PLATFORM("XILINX"),
    .USE_OCRB("FALSE"),
    .USE_OCRC("FALSE"),
    .BUS_ADDR_IO_LEN(6),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h23),
    .TCCRA_ADDR('h90),
    .TCCRB_ADDR('h91),
    .TCCRC_ADDR('h92),
    .TCNTL_ADDR('h94),
    .TCNTH_ADDR('h95),
    .ICRL_ADDR('h96),
    .ICRH_ADDR('h97),
    .OCRAL_ADDR('h98),
    .OCRAH_ADDR('h99),
    .TIMSK_ADDR('h71),
    .TIFR_ADDR('h18)
)tim_3(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_io(io_addr),
    .wr_io(io_write),
    .rd_io(io_read),
    .bus_io_in(io_out),
    .bus_io_out(io_tim3_d_out),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim3_d_out),
    .tov_int(int_timer3_ovf),
    .tov_int_rst(int_timer3_ovf_rst),
    .ocra_int(int_timer3_compa),
    .ocra_int_rst(int_timer3_compa_rst),

    .t(),
    .oca(tim3_oca),
    .oca_io_connect(tim3_oca_io_connact)
    );

wire [7:0]io_pll_d_out;
atmega_pll # (
    .PLATFORM("XILINX"),
    .BUS_ADDR_DATA_LEN(6),
    .PLLCSR_ADDR('h29),
    .PLLFRQ_ADDR('h32),
    .USE_PLL("FALSE")
)pll(
    .rst(rst),
    .clk(clk),
    .clk_pll(clk_pll),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_pll_d_out),
    .pll_enabled(pll_enabled),

    .usb_ck_out(usb_ck_out),
    .tim_ck_out(tim_ck_out)
    );

wire [7:0]io_tim4_d_out;
wire [7:0]dat_tim4_d_out;
atmega_tim_10bit # (
    .PLATFORM("XILINX"),
    .USE_OCRA("TRUE"),
    .USE_OCRB("TRUE"),
    .USE_OCRD("TRUE"),
    .BUS_ADDR_IO_LEN(6),
    .BUS_ADDR_DATA_LEN(8),
    .TCCRA_ADDR('hc0),
    .TCCRB_ADDR('hc1),
    .TCCRC_ADDR('hc2),
    .TCCRD_ADDR('hc3),
    .TCCRE_ADDR('hc4),
    .TCNTL_ADDR('hbe),
    .TCH_ADDR('hbf),
    .OCRA_ADDR('hcf),
    .OCRB_ADDR('hd0),
    .OCRC_ADDR('hd1),
    .OCRD_ADDR('hd2),
    .TIMSK_ADDR('h72),
    .TIFR_ADDR('h19)
)tim_4(
    .rst(rst),
    .clk(clk),
    .clk_pll(tim_ck_out),
    .pll_enabled(pll_enabled),
    .addr_io(io_addr),
    .wr_io(io_write),
    .rd_io(io_read),
    .bus_io_in(io_out),
    .bus_io_out(io_tim4_d_out),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim4_d_out),
    .tov_int(int_timer4_ovf),
    .tov_int_rst(int_timer4_ovf_rst),
    .ocra_int(int_timer4_compa),
    .ocra_int_rst(int_timer4_compa_rst),
    .ocrb_int(int_timer4_compb),
    .ocrb_int_rst(int_timer4_compb_rst),
    .ocrc_int(),
    .ocrc_int_rst(),
    .ocrd_int(int_timer4_compd),
    .ocrd_int_rst(int_timer4_compd_rst),

    .t(),
    .oca(tim4_oca),
    .ocb(tim4_ocb),
    .occ(tim4_occ),
    .ocd(tim4_ocd),
    .ocap_io_connect(tim4_ocap_io_connact),
    .ocan_io_connect(tim4_ocan_io_connact),
    .ocbp_io_connect(tim4_ocbp_io_connact),
    .ocbn_io_connect(tim4_ocbn_io_connact),
    .occp_io_connect(tim4_occp_io_connact),
    .occn_io_connect(tim4_occn_io_connact),
    .ocdp_io_connect(tim4_ocdp_io_connact),
    .ocdn_io_connect(tim4_ocdn_io_connact)
    );

wire [7:0]io_eeprom_d_out;
atmega_eep # (
    .PLATFORM("XILINX"),
    .BUS_ADDR_DATA_LEN(16),
    .EEARH_ADDR('h22),
    .EEARL_ADDR('h21),
    .EEDR_ADDR('h20),
    .EECR_ADDR('h1F),
    .EEP_SIZE(1024)
)eep(
    .rst(rst),
    .clk(clk),
    .addr(io_addr),
    .wr(io_write),
    .rd(io_read),
    .bus_in(io_out),
    .bus_out(io_eeprom_d_out),
    .int_out(int_ee_ready),
    .int_rst(int_ee_ready_rst),
    .content_modifyed()
    );

/*
rom  #(
.ADDR_ROM_BUS_WIDTH(`ROM_ADDR_WIDTH),
.ROM_PATH("")
)rom(
    .clk(core_clk),
    .a(pgm_addr),
    .d(pgm_data)
);
*/

wire [7:0]ram_bus_out;
ram  #(
.ADDR_BUS_WIDTH(`RAM_ADDR_WIDTH),
.RAM_PATH("")
)ram(
    .clk(core_clk),
    .re(data_read & ram_sel),
    .we(data_write & ram_sel),
    .a(data_addr[`RAM_ADDR_WIDTH-1:0] - 'h100),
    .d_in(core_data_out),
    .d_out(ram_bus_out)
);

io_bus_dmux #(
    .NR_OF_BUSSES_IN(12)
    )
    io_bus_dmux_inst(
    .bus_in({
    io_pb_d_out,
    io_pc_d_out,
    io_pd_d_out,
    io_pe_d_out,
    io_pf_d_out,
    io_spi_d_out,
    io_tim0_d_out,
    io_tim1_d_out,
    io_tim3_d_out,
    io_tim4_d_out,
    io_pll_d_out,
    io_eeprom_d_out
    }),
    .bus_out(io_in)
    );

io_bus_dmux #(
    .NR_OF_BUSSES_IN(5)
    )
    ram_bus_dmux_inst(
    .bus_in({
    ram_bus_out,
    dat_tim0_d_out,
    dat_tim1_d_out,
    dat_tim3_d_out,
    dat_tim4_d_out
    }),
    .bus_out(core_data_in)
    );

xmega # (
    .PLATFORM(`PLATFORM),
    .CORE_TYPE(`CORE_TYPE),
    .ROM_ADDR_WIDTH(`ROM_ADDR_WIDTH),
    .RAM_ADDR_WIDTH(`BUS_ADDR_DATA_LEN),
    .WATCHDOG_CNT_WIDTH(`WATCHDOG_CNT_WIDTH),/* If is 0 the watchdog is disabled */
    .VECTOR_INT_TABLE_SIZE(`VECTOR_INT_TABLE_SIZE)/* If is 0 the interrupt module is disabled */
    )atmega32u4_inst(
    .rst(rst),
    .sys_rst_out(wdt_rst),
    // Core clock.
    .clk(core_clk),
    // Watchdog clock input that can be different of the core clock.
    .clk_wdt(core_clk),
    // Used to halt the core.
    .hold(1'b0),
    // FLASH space data interface.
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    // RAM space data interface.
    .data_addr(data_addr),
    .data_out(core_data_out),
    .data_write(data_write),
    .data_in(core_data_in),
    .data_read(data_read),
    // IO space data interface.
    .io_addr(io_addr),
    .io_out(io_out),
    .io_write(io_write),
    .io_in(io_in),
    .io_read(io_read),
    // Interrupt lines from all interfaces.
    .int_sig({
    int_timer4_fpf, int_timer4_ovf, int_timer4_compd, int_timer4_compb, int_timer4_compa,
    int_spm_ready,
    int_twi,
    int_timer3_ovf, int_timer3_compc, int_timer3_compb, int_timer3_compa, int_timer3_capt,
    int_ee_ready,
    int_adc,
    int_analog_comp,
    int_usart1_tx, int_usart1_udre, int_usart1_rx,
    int_spi_stc,
    int_timer0_ovf, int_timer0_compb, int_timer0_compa,
    int_timer1_ovf, int_timer1_compc, int_timer1_compb, int_timer1_compa, int_timer1_capt,
    int_reserved6, int_reserved5, int_reserved4,
    int_wdt,
    int_usb_endpoint, int_usb_general,
    int_pcint0,
    int_reserved3,
    int_int6,
    int_reserved1, int_reserved0,
    int_int3, int_int2, int_int1, int_int0}
    ),
    // Interrupt reset lines going to all interfaces.
    .int_rst({
    int_timer4_fpf_rst, int_timer4_ovf_rst, int_timer4_compd_rst, int_timer4_compb_rst, int_timer4_compa_rst,
    int_spm_ready_rst,
    int_twi_rst,
    int_timer3_ovf_rst, int_timer3_compc_rst, int_timer3_compb_rst, int_timer3_compa_rst, int_timer3_capt_rst,
    int_ee_ready_rst,
    int_adc_rst,
    int_analog_comp_rst,
    int_usart1_tx_rst, int_usart1_udre_rst, int_usart1_rx_rst,
    int_spi_stc_rst,
    int_timer0_ovf_rst, int_timer0_compb_rst, int_timer0_compa_rst,
    int_timer1_ovf_rst, int_timer1_compc_rst, int_timer1_compb_rst, int_timer1_compa_rst, int_timer1_capt_rst,
    int_reserved6_rst, int_reserved5_rst, int_reserved4_rst,
    int_wdt_rst,
    int_usb_endpoint_rst, int_usb_general_rst,
    int_pcint0_rst,
    int_reserved3_rst,
    int_int6_rst,
    int_reserved1_rst, int_reserved0_rst,
    int_int3_rst, int_int2_rst, int_int1_rst, int_int0_rst}
    )
);

endmodule
