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
	`include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;
assign FB_FORCE_BLANK = 0;

// Speaker is wired across Buzzer1/Buzzer2 with no ground reference, so the mix must be
// differential, not additive. AUDIO_S=1 (signed) avoids the unsigned bit-flip that would
// otherwise crush the swing to a couple of LSBs.
assign AUDIO_S     = 1;
assign AUDIO_L     = (Buzzer1 == Buzzer2) ? 16'sd0 : (Buzzer1 ? 16'sd32767 : -16'sd32767);
assign AUDIO_R     = AUDIO_L;
assign AUDIO_MIX   = 0;

assign LED_POWER   = 0;
assign LED_DISK[1] = 0;
assign USER_OUT[0] = 1;
assign USER_OUT[6:2] = 5'd0;
assign BUTTONS = 0;

wire [1:0] ar = status[9:8];

assign VIDEO_ARX = (!ar) ? (status[1] ? 8'd9  : 8'd16) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (status[1] ? 8'd16 : 8'd9 ) : 12'd0;

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

    if(status[0] | buttons[1] | RESET | cart_download) reset_cnt <= 0;
end

///////////////////////////////////////////////////////

// Status Bit Map: (0..31 => "O", 32..63 => "o")
// 0         1         2         3          4         5         6
// 01234567890123456789012345678901 23456789012345678901234567890123
// 0123456789ABCDEFGHIJKLMNOPQRSTUV 0123456789ABCDEFGHIJKLMNOPQRSTUV
// XXXXXXX XX     XX

`include "build_id.v"
localparam CONF_STR =
{
    "Arduboy;;",
    "F0,BINHEX;",
    "-;",
    "O1,Orientation,Horizontal,Vertical;",
    "O89,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
    "O35,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
    "OFG,ADC,Random,AnalogStick,Paddle;",
    "-;",
    "O2,Custom Palette,Off,On;",
    "D0FC1,GBP,Load Palette;",
    "D0O6,Palette Colors,Normal,Swapped;",
    "-;",
    "R0,Reset;",
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

// ioctl_index[5:0] tells files apart: game ROM (F0) = 0, palette (FC1) = 1.
// ioctl_index[7:6] is the extension index within "BINHEX" (0 = BIN, 1 = HEX).
wire cart_download    = (ioctl_index[5:0] == 0) && ioctl_download;
wire palette_download = (ioctl_index[5:0] == 1) && ioctl_download;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
    .clk_sys(clk_sys),
    .HPS_BUS(HPS_BUS),

    .joystick_0(joystick),
    .joystick_l_analog_0(joystick_analog),
    .paddle_0(paddle),

    .status(status),
    .status_menumask(~status[2]),
    .buttons(buttons),

    .forced_scandoubler(forced_scandoubler),
	 .video_rotated(video_rotated),

    .ioctl_download(ioctl_download),
    .ioctl_index(ioctl_index),
    .ioctl_wr(ioctl_wr),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout)
);

wire [13:0] pgm_addr;
wire [15:0] pgm_data;

// SPM writes a page word at a time through the ROM's CPU-side port. The core holds SPM for
// the whole walk and pulses spm_pgm_write for one cycle per word, so borrowing the read
// address costs the fetch nothing.
wire [13:0] spm_pgm_addr;
wire [15:0] spm_pgm_data;
wire        spm_pgm_write;
// A port-A write commits on the edge that presents it, so the word has landed a cycle later.
reg         spm_pgm_write_ack;
always @(posedge clk_avr) spm_pgm_write_ack <= spm_pgm_write;

// Loader writes, muxed onto the ROM's byte-wide port B. Both writers below drive these.
reg  [14:0] rom_wraddr;
reg   [7:0] rom_wrdata;
reg         rom_wr;

arduboy_rom #(.MEM_INIT_FILE("rtl/Arduventure.mif")) rom
(
	.clk_avr(clk_avr),
	.addr_a(spm_pgm_write ? spm_pgm_addr : pgm_addr),
	.data_a(spm_pgm_data),
	.wren_a(spm_pgm_write),
	.q_a(pgm_data),

	.clk_sys(clk_sys),
	.addr_b(rom_wraddr),
	.data_b(rom_wrdata),
	.wren_b(rom_wr)
);

wire [3:0] digit = (ioctl_dout[7:4] != 3) ? (ioctl_dout[3:0] + 4'd9) : ioctl_dout[3:0];
always @ (posedge clk_sys) begin
    reg  [3:0] state = 0;
    reg  [7:0] cnt;
    reg [15:0] addr;
    reg  [3:0] code;

    rom_wr <= 0;

    if (ioctl_wr && cart_download) begin
        if(!ioctl_index) begin
            rom_wraddr <= ioctl_addr[14:0];
            rom_wrdata <= ioctl_dout;
            rom_wr     <= 1;
        end
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
                        rom_wraddr <= addr[14:0];
                        rom_wrdata <= {code,digit};
                        rom_wr     <= 1;
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

arduboy_board arduboy_board
(
    .clk(clk_avr),
    .rst(reset),
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    .spm_pgm_addr(spm_pgm_addr),
    .spm_pgm_data(spm_pgm_data),
    .spm_pgm_write(spm_pgm_write),
    .spm_pgm_write_ack(spm_pgm_write_ack),
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
    .clk_avr(clk_avr),
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

// Kitrinx .gbp palette: file bytes shift in MSB-first.
// FG color = bytes 0-2, BG color = bytes 9-11 (same layout as AdventureVision core).
reg [127:0] palette = 128'hFFFFFF00000000000000000000000000; // default: stock white-on-black

always @ (posedge clk_sys) if (palette_download & ioctl_wr) palette <= {palette[119:0], ioctl_dout};

wire [23:0] color_fg = status[6] ? palette[55:32]   : palette[127:104];
wire [23:0] color_bg = status[6] ? palette[127:104] : palette[55:32];

arcade_video #(256,24) arcade_video
(
    .*,
    .clk_video(clk_sys),
    .RGB_in(status[2] ? (pixelValue ? color_fg : color_bg) : {24{pixelValue}}),
    .gamma_bus(),
    .fx(status[5:3])
);

wire no_rotate = ~status[1];
wire rotate_ccw = 1;
wire flip = 0;
screen_rotate screen_rotate (.*);

endmodule
