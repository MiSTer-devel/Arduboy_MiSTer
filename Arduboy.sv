//============================================================================
//  Arduboy MiSTer core by uXeBoy (Dan O'Shea)
//
//  XMEGA-CORE by Iulian Gheorghiu (morgoth@devboard.tech) Copyright (C) 2020
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
(
    //Master input clock
    input         CLK_50M,

    //Async reset from top-level module.
    //Can be used as initial reset.
    input         RESET,

    //Must be passed to hps_io module
    inout  [45:0] HPS_BUS,

    //Base video clock. Usually equals to CLK_SYS.
    output        CLK_VIDEO,

    //Multiple resolutions are supported using different CE_PIXEL rates.
    //Must be based on CLK_VIDEO
    output        CE_PIXEL,

    //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
    output  [7:0] VIDEO_ARX,
    output  [7:0] VIDEO_ARY,

    output  [7:0] VGA_R,
    output  [7:0] VGA_G,
    output  [7:0] VGA_B,
    output        VGA_HS,
    output        VGA_VS,
    output        VGA_DE,    // = ~(VBlank | HBlank)
    output        VGA_F1,
    output  [1:0] VGA_SL,

    output        LED_USER,  // 1 - ON, 0 - OFF.

    // b[1]: 0 - LED status is system status OR'd with b[0]
    //       1 - LED status is controled solely by b[0]
    // hint: supply 2'b00 to let the system control the LED.
    output  [1:0] LED_POWER,
    output  [1:0] LED_DISK,

    // I/O board button press simulation (active high)
    // b[1]: user button
    // b[0]: osd button
    output  [1:0] BUTTONS,

    output [15:0] AUDIO_L,
    output [15:0] AUDIO_R,
    output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
    output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

    //ADC
    inout   [3:0] ADC_BUS,

    //SD-SPI
    output        SD_SCK,
    output        SD_MOSI,
    input         SD_MISO,
    output        SD_CS,
    input         SD_CD,

    //High latency DDR3 RAM interface
    //Use for non-critical time purposes
    output        DDRAM_CLK,
    input         DDRAM_BUSY,
    output  [7:0] DDRAM_BURSTCNT,
    output [28:0] DDRAM_ADDR,
    input  [63:0] DDRAM_DOUT,
    input         DDRAM_DOUT_READY,
    output        DDRAM_RD,
    output [63:0] DDRAM_DIN,
    output  [7:0] DDRAM_BE,
    output        DDRAM_WE,

    //SDRAM interface with lower latency
    output        SDRAM_CLK,
    output        SDRAM_CKE,
    output [12:0] SDRAM_A,
    output  [1:0] SDRAM_BA,
    inout  [15:0] SDRAM_DQ,
    output        SDRAM_DQML,
    output        SDRAM_DQMH,
    output        SDRAM_nCS,
    output        SDRAM_nCAS,
    output        SDRAM_nRAS,
    output        SDRAM_nWE,

    input         UART_CTS,
    output        UART_RTS,
    input         UART_RXD,
    output        UART_TXD,
    output        UART_DTR,
    input         UART_DSR,

    // Open-drain User port.
    // 0 - D+/RX
    // 1 - D-/TX
    // 2..6 - USR2..USR6
    // Set USER_OUT to 1 to read from USER_IN.
    input   [6:0] USER_IN,
    output  [6:0] USER_OUT,

    input         OSD_STATUS
);

assign CLK_VIDEO    = clk_100m;
assign VIDEO_ARX    = 4;
assign VIDEO_ARY    = 3;
assign VGA_F1       = 0;
assign VGA_SL       = 0;
assign LED_POWER[1] = 1;
assign LED_DISK[1]  = 1;
assign BUTTONS      = 0;
assign USER_OUT     = 0;
assign AUDIO_S      = 0;
assign AUDIO_MIX    = 3;
assign AUDIO_L      = (Buzzer1) ? 16'h7FFF : 16'd0;
assign AUDIO_R      = (Buzzer2) ? 16'hFFFF : 16'd0;
assign ADC_BUS      = 'Z;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_RD, DDRAM_DIN, DDRAM_BE, DDRAM_WE} = 0;
assign {SDRAM_CLK, SDRAM_CKE, SDRAM_A, SDRAM_BA, SDRAM_DQ, SDRAM_DQML, SDRAM_DQMH, SDRAM_nCS, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nWE} = 'Z;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;

`include "build_id.v"
localparam CONF_STR =
{
    "Arduboy;;",
    "F0,bin;",
    "R0,Reset;",
    "J1,A,B;",
    "V,v",`BUILD_DATE
};

wire [31:0] joystick;
wire [31:0] status;

hps_io #(.STRLEN($size(CONF_STR)>>3)) hps_io
(
    .clk_sys(clk_100m),
    .HPS_BUS(HPS_BUS),
    .conf_str(CONF_STR),
    .joystick_0(joystick),
    .status(status),

    .ioctl_download(ioctl_download),
    .ioctl_wr(ioctl_wr),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout)
);

wire [3:0] R,G,B;
wire VSync,HSync,HBlank,VBlank;

video_cleaner video_cleaner
(
    .clk_vid(clk_100m),
    .ce_pix(clk_25m),

    .R({4{pixelValue}}),
    .G({4{pixelValue}}),
    .B({4{pixelValue}}),

    .HSync(hsync),
    .VSync(vsync),
    .HBlank(hblank),
    .VBlank(vblank),

    .VGA_R(R),
    .VGA_G(G),
    .VGA_B(B),
    .VGA_VS(VSync),
    .VGA_HS(HSync),

    .HBlank_out(HBlank),
    .VBlank_out(VBlank)
);

video_mixer #(.LINE_LENGTH(800), .HALF_DEPTH(1)) video_mixer
(
    .*,

    .clk_vid(clk_100m),
    .ce_pix(clk_25m),
    .ce_pix_out(CE_PIXEL),

    .scandoubler(0),
    .scanlines(0),
    .hq2x(0),
    .mono(1),
    .gamma_bus()
);

wire clk_100m, clk_25m, clk_16m, clk_192m;
wire pll_locked;

pll_50m pll_50m
(
    .refclk(CLK_50M),
    .rst(RESET),
    .outclk_0(clk_100m),
    .outclk_1(clk_25m),
    .outclk_2(clk_16m),
    .outclk_3(clk_192m),
    .locked(pll_locked)
);

wire oled_dc, oled_clk, oled_data;
wire hsync, vsync;
wire hblank, vblank;
wire pixelValue;

vgaHdmi vgaHdmi
(
    .clock(clk_25m),
    .clock100(clk_100m),
    .reset(status[0] | RESET | ioctl_download),
    .oled_dc(oled_dc),
    .oled_clk(oled_clk),
    .oled_data(oled_data),
    .hsync(hsync),
    .vsync(vsync),
    .hblank(hblank),
    .vblank(vblank),
    .pixelValue(pixelValue)
);

wire Buzzer1, Buzzer2;

atmega32u4 atmega32u4
(
    .rst_in(status[0] | RESET | ioctl_download),
    .clk(clk_16m),
    .clk_pll(clk_192m),
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    .buttons(~(joystick[5:0])),
    .RGB({LED_POWER[0], LED_USER, LED_DISK[0]}),
    .Buzzer1(Buzzer1),
    .Buzzer2(Buzzer2),
    .DC(oled_dc),
    .spi_scl(oled_clk),
    .spi_mosi(oled_data)
);

(* ram_init_file = "Arduventure.mif" *)
reg [15:0] rom [16383:0];

wire [13:0] pgm_addr;
reg  [15:0] pgm_data;

always @ (posedge clk_16m)
begin
    pgm_data <= rom[pgm_addr];
end

wire ioctl_download;
wire ioctl_wr;
wire [14:0] ioctl_addr;
wire [7:0]  ioctl_dout;
reg  [7:0]  temp_data;

always @ (posedge clk_100m)
begin
    if (ioctl_download & ioctl_wr) begin
        if (ioctl_addr[0] == 1'b1) begin
            rom[ioctl_addr[14:1]] <= {ioctl_dout, temp_data}; // 16-bit little endian
        end
        else if (ioctl_addr[0] == 1'b0) temp_data <= ioctl_dout;
    end
end

endmodule
