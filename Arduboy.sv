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
	output        VGA_CLK,

	//Multiple resolutions are supported using different VGA_CE rates.
	//Must be based on CLK_VIDEO
	output        VGA_CE,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,

	//Base video clock. Usually equals to CLK_SYS.
	output        HDMI_CLK,

	//Multiple resolutions are supported using different HDMI_CE rates.
	//Must be based on CLK_VIDEO
	output        HDMI_CE,

	output  [7:0] HDMI_R,
	output  [7:0] HDMI_G,
	output  [7:0] HDMI_B,
	output        HDMI_HS,
	output        HDMI_VS,
	output        HDMI_DE,   // = ~(VBlank | HBlank)
	output  [1:0] HDMI_SL,   // scanlines fx

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] HDMI_ARX,
	output  [7:0] HDMI_ARY,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT
);

assign HDMI_ARX    = 2;
assign HDMI_ARY    = 1;
assign VGA_F1      = 0;

assign LED_POWER   = 0;
assign LED_DISK[1] = 0;
assign USER_OUT    = '1;

assign AUDIO_S     = 0;
assign AUDIO_L     = {1'b0,{15{Buzzer1}}} + {1'b0,{15{Buzzer2}}};
assign AUDIO_R     = AUDIO_L;


///////////////////////////////////////////////////////

wire clk_sys, clk_avr;
pll pll
(
    .refclk(CLK_50M),
    .outclk_0(clk_sys),
    .outclk_1(clk_avr)
);

wire reset = status[0] | buttons[1] | RESET | ioctl_download;

///////////////////////////////////////////////////////

`include "build_id.v"
localparam CONF_STR =
{
    "Arduboy;;",
    "F0,BIN;",
    "R0,Reset;",
    "J1,A,B;",
    "V,v",`BUILD_DATE
};

wire [31:0] joystick;
wire [31:0] status;
wire  [1:0] buttons;
wire        ioctl_download;
wire        ioctl_wr;
wire [14:0] ioctl_addr;
wire [15:0] ioctl_dout;

hps_io #(.STRLEN($size(CONF_STR)>>3), .WIDE(1)) hps_io
(
    .clk_sys(clk_sys),
    .HPS_BUS(HPS_BUS),
    .conf_str(CONF_STR),
    .joystick_0(joystick),
    .status(status),
	 .buttons(buttons),

    .ioctl_download(ioctl_download),
    .ioctl_wr(ioctl_wr),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout)
);

(* ram_init_file = "Arduventure.mif" *)
reg  [15:0] rom [16384];
wire [13:0] pgm_addr;
reg  [15:0] pgm_data;

always @ (posedge clk_avr) pgm_data <= rom[pgm_addr];
always @ (posedge clk_sys) if (ioctl_wr) rom[ioctl_addr[14:1]] <= ioctl_dout;

wire Buzzer1, Buzzer2;
wire oled_dc, oled_clk, oled_data;

atmega32u4 atmega32u4
(
    .clk(clk_avr),
    .rst_in(reset),
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    .buttons(~(joystick[5:0])),
    .RGB({LED_USER, LED_DISK[0]}),
    .Buzzer1(Buzzer1),
    .Buzzer2(Buzzer2),
    .DC(oled_dc),
    .spi_scl(oled_clk),
    .spi_mosi(oled_data)
);


wire pixelValue, ce_pix;
wire VSync,HSync,HBlank,VBlank;

vgaHdmi vgaHdmi
(
    .clock(clk_sys),
    .reset(reset),
    .oled_dc(oled_dc),
    .oled_clk(oled_clk),
    .oled_data(oled_data),
    .hsync(HSync),
    .vsync(VSync),
    .hblank(HBlank),
    .vblank(VBlank),
    .pixelValue(pixelValue),
    .ce_pix(ce_pix)
);

arcade_video #(256,512,6) arcade_video
(
	.*,
	.clk_video(clk_sys),
	.RGB_in({6{pixelValue}}),

	.forced_scandoubler(0),
	.gamma_bus(),
	.no_rotate(1),
	.rotate_ccw(0),
	.fx(0)
);


endmodule
