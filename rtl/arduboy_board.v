/*
 * Real Arduboy board wiring: instantiates the generic atmega32u4 module and connects it exactly
 * the way the real Arduboy PCB does. A different core reusing atmega32u4.v would replace this
 * file with its own board wiring, not touch the chip module itself.
 */

`timescale 1ns / 1ps

module arduboy_board
(
    input rst,
    input clk,
    input clk_pll,
    output [13:0] pgm_addr,
    input [15:0] pgm_data,
    input [5:0] buttons,
    input [7:0] joystick_analog,
    input status,
    output [2:0] RGB,
    output Buzzer1, Buzzer2, DC, spi_scl, spi_mosi,
    input spi_miso,
    output oled_cs, cart_cs,
    input uart_rx,
    output uart_tx
    );

atmega32u4 atmega32u4
(
    .rst(rst),
    .clk(clk),
    .clk_pll(clk_pll),
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    .joystick_analog(joystick_analog),
    .status(status),

    // Buttons are wired to specific real pins on the real Arduboy PCB.
    .PF6(buttons[0]), // BUTTON RIGHT
    .PF5(buttons[1]), // BUTTON LEFT
    .PF4(buttons[2]), // BUTTON DOWN
    .PF7(buttons[3]), // BUTTON UP
    .PE6(buttons[4]), // BUTTON A
    .PB4(buttons[5]), // BUTTON B

    .PB6(RGB[2]),
    .PB7(RGB[1]),
    .PB5(RGB[0]),
    .PC6(Buzzer1),
    .PC7(Buzzer2),
    .PD4(DC),
    .PB1(spi_scl),
    .PB2(spi_mosi),
    .PB3(spi_miso),
    .PD6(oled_cs),
    .PD1(cart_cs),
    .PD2(uart_rx),
    .PD3(uart_tx)
    );

endmodule
