/**
Descripcion,
Modulo que sincroniza las senales (hsync y vsync)
de un controlador VGA de 640x480 60hz, funciona con un reloj de 25Mhz

Ademas tiene las coordenadas de los pixeles H (eje x),
y de los pixeles V (eje y). Para enviar la senal RGB correspondiente
a cada pixel

-----------------------------------------------------------------------------
Author : Nicolas Hasbun, nhasbun@gmail.com
File   : vgaHdmi.v
Create : 2017-06-15 15:07:05
Editor : sublime text3, tab size (2)
-----------------------------------------------------------------------------
*/

// **Info Source**
// https://eewiki.net/pages/viewpage.action?pageId=15925278

module vgaHdmi(
  input clock, clock100,
  input reset,
  input oled_dc,
  input oled_clk,
  input oled_data,
  output reg hsync, vsync,
  output hblank, vblank,
  output pixelValue
);

assign hblank = (pixelH > 640);
assign vblank = (pixelV > 480);

reg [7:0] mem [0:1023];
reg [9:0] waddr;

reg invert;
reg [2:0] shiftCount;
reg [7:0] shiftReg;
wire [7:0] shiftLeft;
assign shiftLeft = {shiftReg[6:0], oled_data};

always @ (posedge oled_clk or posedge reset)
begin
  if(reset) begin
    waddr         <= 0;
    invert        <= 0;
    shiftCount    <= 0;
    shiftReg      <= 0;
  end
  else begin
    if (oled_dc) begin // data
      if (shiftCount == 3'b111) begin
        mem[waddr] <= shiftLeft;
        waddr <= waddr + 1'b1; // Increment address
      end
      else shiftReg <= shiftLeft;
      shiftCount <= shiftCount + 1'b1;
    end
    else begin // commands
      if (shiftCount == 3'b111) begin
        case (shiftLeft)
          8'hA7 : invert <= 1'b1; // All pixels inverted
          8'hA6 : invert <= 1'b0; // All pixels normal
          8'hB0 : waddr <= 10'd0; // 'VSYNC' / Page 0
          8'hB1 : waddr <= 10'd128;
          8'hB2 : waddr <= 10'd256;
          8'hB3 : waddr <= 10'd384;
          8'hB4 : waddr <= 10'd512;
          8'hB5 : waddr <= 10'd640;
          8'hB6 : waddr <= 10'd768;
          8'hB7 : waddr <= 10'd896;
        endcase
      end
      else shiftReg <= shiftLeft;
      shiftCount <= shiftCount + 1'b1;
    end
  end
end

always @ (posedge clock100)
begin
  tempByte <= mem[bytePosition];
end

always @*
begin
  case (pixelY[5:3])
    3'd0 : pixelZ = 10'd0;
    3'd1 : pixelZ = 10'd128;
    3'd2 : pixelZ = 10'd256;
    3'd3 : pixelZ = 10'd384;
    3'd4 : pixelZ = 10'd512;
    3'd5 : pixelZ = 10'd640;
    3'd6 : pixelZ = 10'd768;
    3'd7 : pixelZ = 10'd896;
    default : pixelZ = 10'd0;
  endcase
end

wire dataEnable;
reg  invertLatched;
reg  [9:0] pixelZ;
wire [6:0] pixelX;
wire [5:0] pixelY;
wire [2:0] bitPosition;
wire [9:0] bytePosition;
reg  [7:0] tempByte;
assign pixelX = pixelH / 5;
assign pixelY = pixelV_offset / 5;
assign bitPosition = (pixelY & 3'd7);
assign bytePosition = pixelZ + pixelX;
assign pixelValue = (dataEnable) ? ((invertLatched) ? ~tempByte[bitPosition] : tempByte[bitPosition]) : 1'b0;
assign dataEnable = (pixelH > 0 && pixelH < 640 && pixelV_offset > 0 && pixelV_offset < 320) ? 1'b1 : 1'b0;

reg [9:0] pixelH, pixelV, pixelV_offset; // estado interno de pixeles del modulo

// Manejo de Pixeles y Sincronizacion

always @ (posedge clock or posedge reset) begin
  if(reset) begin
    hsync         <= 0;
    vsync         <= 0;
    pixelH        <= 0;
    pixelV        <= 0;
    pixelV_offset <= 0;
    invertLatched <= 0;
  end
  else begin
    // Display Horizontal
    if(pixelH==0 && pixelV!=524) begin
      pixelH <= pixelH + 1'b1;
      pixelV <= pixelV + 1'b1;
      if(pixelV==80) begin
        pixelV_offset <= 0;
        invertLatched <= invert;
      end
      else pixelV_offset <= pixelV_offset + 1'b1;
    end
    else if(pixelH==0 && pixelV==524) begin
      pixelH <= pixelH + 1'b1;
      pixelV <= 0; // pixel 525
    end
    else if(pixelH<=640) pixelH <= pixelH + 1'b1;
    // Front Porch
    else if(pixelH<=656) pixelH <= pixelH + 1'b1;
    // Sync Pulse
    else if(pixelH<=752) begin
      pixelH <= pixelH + 1'b1;
      hsync  <= 1;
    end
    // Back Porch
    else if(pixelH<799) begin
      pixelH <= pixelH+1'b1;
      hsync  <= 0;
    end
    else pixelH<=0; // pixel 800

    // Manejo Senal Vertical
    // Sync Pulse
    if(pixelV == 491 || pixelV == 492)
      vsync <= 1;
    else
      vsync <= 0;
  end
end

endmodule
