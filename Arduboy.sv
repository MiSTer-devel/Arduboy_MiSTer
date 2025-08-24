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
	inout  [48:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM (USE_FB=1 in qsf)
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

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

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
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

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

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

wire [1:0] ar = status[9:8];

assign VIDEO_ARX = (!ar) ? (status[1] ? 8'd9  : 8'd16) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (status[1] ? 8'd16 : 8'd9 ) : 12'd0;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign LED_POWER   = 0;
assign LED_DISK[1] = 0;
assign USER_OUT[0] = 1;
assign USER_OUT[6:2] = 5'd0;

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

// make reset at least 65K cycles long.
reg reset = 1;
always @(posedge clk_avr) begin
    reg [15:0] reset_cnt = 0;

    reset <= 0;
    if(~&reset_cnt) begin
        reset_cnt <= reset_cnt + 1'd1;
        reset <= 1;
    end

    if(status[0] | buttons[1] | RESET | ioctl_download) reset_cnt <= 0;
end

///////////////////////////////////////////////////////

`include "build_id.v"
localparam CONF_STR =
{
    "Arduboy;;",
    "F0,BINHEX;",
    "R0,Reset;",
    "-;",
    "O1,Orientation,Horizontal,Vertical;",
    "O89,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
    "O35,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
    "OFG,ADC,Random,AnalogStick,Paddle;",
    "J1,A,B;",
    "V,v",`BUILD_DATE
};

wire [31:0] joystick;
wire [15:0] joystick_analog;
wire  [7:0] paddle;
wire [31:0] status;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire        video_rotated;

wire        ioctl_download;
wire        ioctl_wr;
wire [14:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire  [7:0] ioctl_index;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
    .clk_sys(clk_sys),
    .HPS_BUS(HPS_BUS),

    .joystick_0(joystick),
    .joystick_l_analog_0(joystick_analog),
    .paddle_0(paddle),

    .status(status),
    .buttons(buttons),

    .forced_scandoubler(forced_scandoubler),
	 .video_rotated(video_rotated),

    .ioctl_download(ioctl_download),
    .ioctl_index(ioctl_index),
    .ioctl_wr(ioctl_wr),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout)
);

(* ram_init_file = "rtl/Arduventure.mif" *)
reg  [1:0][7:0] rom[16384];
wire [13:0] pgm_addr;
reg  [15:0] pgm_data;

always @ (posedge clk_avr) pgm_data <= rom[pgm_addr];

wire [3:0] digit = (ioctl_dout[7:4] != 3) ? (ioctl_dout[3:0] + 4'd9) : ioctl_dout[3:0];
always @ (posedge clk_sys) begin
    reg  [3:0] state = 0;
    reg  [7:0] cnt;
    reg [15:0] addr;
    reg  [3:0] code;

    if (ioctl_wr) begin
        if(!ioctl_index) rom[ioctl_addr[14:1]][ioctl_addr[0]] <= ioctl_dout;
        else begin
            if(state) state <= state + 1'd1;
            case(state)
                 0: if(ioctl_dout == ":") state <= state + 1'd1;
                 1: cnt[7:4]    <= digit;
                 2: cnt[3:0]    <= digit;
                 3: addr[15:12] <= digit;
                 4: addr[11:8]  <= digit;
                 5: addr[7:4]   <= digit;
                 6: addr[3:0]   <= digit;
                 7: code        <= digit;
                 8: if({code,digit}) state <= 0;
                 9: code        <= digit;
                10: begin
                        rom[addr[14:1]][addr[0]] <= {code,digit};
                        addr <= addr + 1'd1;
                        cnt <= cnt - 1'd1;
                        state <= state - 1'd1;
                        if(cnt == 1) state <= 0;
                    end
            endcase
        end
    end

    if(!ioctl_download) state <= 0;
end

wire Buzzer1, Buzzer2;
wire oled_dc, oled_clk, oled_data;

atmega32u4 atmega32u4
(
    .clk(clk_avr),
    .rst(reset),
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    .buttons(~(status[1] ? {joystick[5:4], joystick[1], joystick[0], joystick[2], joystick[3]} : joystick[5:0])),
    .joystick_analog(status[16] ? {~paddle[7],paddle[6:0]} : joystick_analog[7:0]),
    .status(|status[16:15]),
    .RGB({LED_USER, LED_DISK[0]}),
    .Buzzer1(Buzzer1),
    .Buzzer2(Buzzer2),
    .DC(oled_dc),
    .spi_scl(oled_clk),
    .spi_mosi(oled_data),
    .uart_rx(USER_IN[0]),
    .uart_tx(USER_OUT[1])
);

wire pixelValue, ce_pix;
wire VSync, HSync, HBlank, VBlank;

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

arcade_video #(256,6) arcade_video
(
    .*,
    .clk_video(clk_sys),
    .RGB_in({6{pixelValue}}),
    .gamma_bus(),
    .fx(status[5:3])
);

assign {FB_PAL_CLK, FB_FORCE_BLANK, FB_PAL_ADDR, FB_PAL_DOUT, FB_PAL_WR} = '0;

wire no_rotate = ~status[1];
wire rotate_ccw = 1;
wire flip = 0;
screen_rotate screen_rotate (.*);

endmodule
