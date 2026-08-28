/*
 * This IP is the MEGA/XMEGA core implementation.
 * 
 * Copyright (C) 2018  Iulian Gheorghiu (morgoth.creator@gmail.com)
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

`include "mega-def.v"

/*
 * Watchdog.
 */
module watchdog # (
		parameter CNT_WIDTH = 0
		)(
		input rst,
		input clk,
		input wdt_clk,
		input wdt_rst_in,
		output wdt_rst_out);
		
reg [CNT_WIDTH:0]wdt_cnt;
reg reset;
reg reset_n;
reg wdt_reset;
reg wdt_reset_n;
reg wdt_rst_out_int;
assign wdt_rst_out = CNT_WIDTH != 0 ? (rst | wdt_rst_out_int) : rst;

always @ (posedge rst or posedge wdt_rst_in)
begin
	if(CNT_WIDTH != 0)
	begin
		if(rst)
			reset <= 1'b0;
		else
			reset <= ~reset_n;
	end
end

always @ (posedge wdt_clk)
begin
	if(CNT_WIDTH != 0)
	begin
		if(rst)
		begin
			reset_n <= 1'b0;
			wdt_reset <=1'b0;
			wdt_cnt <= 'h0000;
		end
		else if(reset != reset_n)
		begin
			reset_n <= reset;
			wdt_cnt <= 'h0000;
		end
		else if(wdt_clk)
		begin
			if(wdt_cnt[CNT_WIDTH])
			begin
				wdt_reset <= ~wdt_reset_n;
				wdt_cnt <= 'h0000;
			end
			else
				wdt_cnt <= wdt_cnt + 1;
		end
	end
end

always @ (posedge clk)
begin
	if(CNT_WIDTH != 0)
	begin
		wdt_rst_out_int <= 1'b0;
		if(rst)
			wdt_reset_n <= 1'b1;
		else if(wdt_reset != wdt_reset_n)
		begin
			wdt_reset_n <= wdt_reset;
			wdt_rst_out_int <= 1'b1;
		end
	end
end

endmodule
/*
 * !Watchdog.
 */
  
/*
 * Interrupt and priority encoder.
 */
module int_encoder # (
		parameter VECTOR_INT_TABLE_SIZE = 0
		)(
		input rst,
		input halt_ack,
		input clk,
		input [((VECTOR_INT_TABLE_SIZE == 0) ? 0 : VECTOR_INT_TABLE_SIZE-1):0]int_sig_in,
		output int_request,
		output reg[((VECTOR_INT_TABLE_SIZE > 127) ? 7 :
					(VECTOR_INT_TABLE_SIZE > 63) ? 6 :
					(VECTOR_INT_TABLE_SIZE > 31) ? 5 :
					(VECTOR_INT_TABLE_SIZE > 15) ? 4 :
					(VECTOR_INT_TABLE_SIZE > 7) ? 3 :
					(VECTOR_INT_TABLE_SIZE > 3) ? 2 :
					(VECTOR_INT_TABLE_SIZE > 1) ? 1 : 0) : 0]int_vect
		);

integer j;
always @ (posedge clk)
begin
	if(VECTOR_INT_TABLE_SIZE != 0)
	begin
		if(rst)
			int_vect <= 0;
		else
		begin
			if(~halt_ack)
			begin
				int_vect <= 0;
				for (j=VECTOR_INT_TABLE_SIZE-1; j>=0; j=j-1)
				if (int_sig_in[j]) 
					int_vect <= j+1;
			end
		end
	end
end

assign int_request = (int_vect != 0 && VECTOR_INT_TABLE_SIZE != 'h0);

endmodule
/*
 * !Interrupt and priority encoder.
 */

module mega #(
	parameter BOOT_ADDR = 0,
	parameter USE_HALT = "FALSE",
	parameter CORE_TYPE = `MEGA_XMEGA_1,
	parameter ROM_ADDR_WIDTH = 16,
	parameter RAM_ADDR_WIDTH = 16,
	parameter WATCHDOG_CNT_WIDTH = 0,
	parameter VECTOR_INT_TABLE_SIZE = 0,
	parameter REGS_REGISTERED = "FALSE"
	)(
	input rst,
	output sys_rst_out,
	input clk,
	input clk_wdt,
	input halt,
	output reg halt_ack,
	output reg[ROM_ADDR_WIDTH - 1:0]pgm_addr,
	input [15:0]pgm_data,
	output reg [RAM_ADDR_WIDTH - 1:0]data_addr,
	output reg [7:0]data_out,
	output reg data_write,
	input [7:0]data_in,
	output reg data_read,
	// Dedicated second RAM read port, RET/RETI only -- lets the two stack-pop bytes be
	// addressed the same cycle instead of serially sharing the data_addr/data_in bus. Never
	// targets I/O space, so (unlike data_in_int) needs no address-mapped-register mux.
	output reg [RAM_ADDR_WIDTH - 1:0]data_addr2,
	input [7:0]data_in2,
	input [(VECTOR_INT_TABLE_SIZE == 0 ? 0 : VECTOR_INT_TABLE_SIZE - 1):0]int_sig,
	output reg [(VECTOR_INT_TABLE_SIZE == 0 ? 0 : VECTOR_INT_TABLE_SIZE - 1):0]int_rst
    );

reg wdt_rst_out;
wire core_rst;
assign sys_rst_out = core_rst;

reg [RAM_ADDR_WIDTH - 1:0]data_addr_int;
reg [RAM_ADDR_WIDTH - 1:0]data_addr2_int;
reg [7:0]data_out_int;
reg data_write_int;
reg [7:0]data_in_int;
reg data_read_int;

reg [1:0]state_cnt;
reg [7:0]SREG;
reg [RAM_ADDR_WIDTH - 1:0]SP;
reg [7:0]EIND;
reg [7:0]RAMPZ;
reg [7:0]RAMPY;
reg [7:0]RAMPX;
reg [7:0]RAMPD;
reg [7:0]ramp_bits;


reg [ROM_ADDR_WIDTH - 1:0]PC;
wire [ROM_ADDR_WIDTH:0]PC_x_2 = pgm_addr << 1;
wire [7:0]sreg_out;
reg [15:0]pgm_data_registered;
reg [15:0]pgm_data_int;


reg [4:0]rs1a;
reg [4:0]rs2a;
reg [4:0]rda;

// Deferred OUT write-back: rs1a/reg_rs1 for a same-cycle OUT read aren't settled until a cycle
// after decode (cross-module NBA through mega_regs), so OUT's SREG/SP/data-bus side effects are
// deferred to a pending retirement instead of completing at decode. out_retire_pending is a
// level signal; at most one retirement is pending at a time (a second OUT stalls rather than
// overwriting it). out_rs1a is a dedicated, non-shared latch of rs1a's operand field (read via
// mega_regs' third port rs3a/rs3) -- rs1a/reg_rs1 themselves belong to whichever instruction is
// currently decoding by the time a retiring OUT would read them.
reg out_retire_pending;
reg [15:0]out_retire_instr;
reg [4:0]out_rs1a;
wire [15:0]out_reg_rs1;

// Holds a real instruction that collided with a pending OUT retirement on fetch, for replay
// once the hazard clears. The held opcode is captured and replayed directly rather than
// re-fetched: PC runs two pipeline stages ahead of pgm_data_registered, so a freeze-and-refetch
// would land on the instruction after the blocked one and silently skip it.
reg holding_instr;
reg [15:0]held_instr;

// Set for one cycle after a 2-cycle arithmetic op decodes; unconditional, not a hazard check.
reg two_cycle_hold_next;

reg [15:0]alu_rs1;
reg [15:0]alu_rs2;
wire [15:0]alu_rd;

wire [15:0]reg_rs1;
wire [15:0]ijmp_z;
reg reg_rs1m;
wire [15:0]reg_rs2;
reg reg_rs2m;
reg [15:0]reg_rd;
reg reg_rdw;
reg reg_rdm;

reg skip_next_clock;

reg halt_int;

reg [7:0]PC_TMP_H;
// Latches CALL's 2nd instruction word at STEP1, before fetch moves past it -- STEP2's redirect
// can't read pgm_data_int directly, since by then it holds whatever address is fetching next.
reg [15:0]call_word2;

wire [ROM_ADDR_WIDTH - 1:0]PC_PLUS_ONE = PC + 1;
wire [ROM_ADDR_WIDTH - 1:0]PC_MINUS_ONE = PC - 1;
wire [RAM_ADDR_WIDTH - 1:0]SP_PLUS_ONE = SP + 1;
wire [RAM_ADDR_WIDTH - 1:0]SP_PLUS_TWO = SP + 2;
reg [ROM_ADDR_WIDTH - 1:0]PC_PLUS_RJMP_RCALL;
reg [ROM_ADDR_WIDTH - 1:0]PC_PLUS_COND_BRANCH;

reg cnt_rst;
reg unlock_int_registered_step_2;

wire execute	= (~skip_next_clock & ~cnt_rst);

/**************** Interrupt instance  ********************/
localparam int_bus_size = (VECTOR_INT_TABLE_SIZE > 127) ? 8 :
							(VECTOR_INT_TABLE_SIZE > 63) ? 7 :
							(VECTOR_INT_TABLE_SIZE > 31) ? 6 :
							(VECTOR_INT_TABLE_SIZE > 15) ? 5 :
							(VECTOR_INT_TABLE_SIZE > 7) ? 4 :
							(VECTOR_INT_TABLE_SIZE > 3) ? 3 :
							(VECTOR_INT_TABLE_SIZE > 1) ? 2 : 1;
 
wire [int_bus_size - 1 : 0]current_int_vect_request;
reg [int_bus_size - 1 : 0]current_int_vect_registered;
wire int_request;
reg int_registered;

// A function, not a wire/always@*, so it reads pgm_data_registered's freshly-blocking-updated
// value inline within this clocked block rather than one edge stale. OUT unconditionally defers
// its write to a pending retirement (out_retire_pending) rather than peeking ahead at the next
// instruction; the deferred write waits, cycle by cycle, for the data bus to be free. Mirrors
// every STEP where the "Set data_addr"/"Set data_out"/"Set data_write"/"Set data_read" blocks
// below touch the bus for a non-OUT instruction.
function bus_busy_user;
	input [1:0] chk_state;
	input [15:0] chk_instr;
	begin
		casex({chk_state, CORE_TYPE, chk_instr})
			{`STEP0, `INSTRUCTION_RCALL}, {`STEP1, `INSTRUCTION_RCALL},
			{`STEP0, `INSTRUCTION_CALL},  {`STEP1, `INSTRUCTION_CALL},
			{`STEP0, `INSTRUCTION_ICALL}, {`STEP1, `INSTRUCTION_ICALL},
			{`STEP0, `INSTRUCTION_RET},   {`STEP1, `INSTRUCTION_RET},
			{`STEP0, `INSTRUCTION_RETI},  {`STEP1, `INSTRUCTION_RETI},
			{`STEP0, `INSTRUCTION_POP},   {`STEP1, `INSTRUCTION_POP},
			{`STEP0, `INSTRUCTION_IN},
			{`STEP0, `INSTRUCTION_CBI_SBI},   {`STEP1, `INSTRUCTION_CBI_SBI},
			{`STEP0, `INSTRUCTION_SBIC_SBIS}, {`STEP1, `INSTRUCTION_SBIC_SBIS},
			// LD/ST/PUSH family: STEP1/STEP2 entries mark every real site each instruction
			// touches the shared bus at -- sufficient hazard coverage against a same-cycle
			// collision with a pending OUT retirement. Deliberately no STEP0 entry: it would
			// hold incoming instructions on every decode, costing stall cycles and shifting
			// interrupt-vs-foreground timing enough to trip a race.
			// (LDS16/STS16 omitted: gated to MEGA_REDUCED_LDS16_INT, unreachable at this CORE_TYPE.)
			{`STEP1, `INSTRUCTION_LDD},    {`STEP2, `INSTRUCTION_LDD},
			{`STEP1, `INSTRUCTION_LDS},    {`STEP2, `INSTRUCTION_LDS},
			{`STEP1, `INSTRUCTION_LD_X},   {`STEP2, `INSTRUCTION_LD_X},
			{`STEP1, `INSTRUCTION_LD_XP},  {`STEP2, `INSTRUCTION_LD_XP},
			{`STEP1, `INSTRUCTION_LD_XN},  {`STEP2, `INSTRUCTION_LD_XN},
			{`STEP1, `INSTRUCTION_LD_YZP}, {`STEP2, `INSTRUCTION_LD_YZP},
			{`STEP1, `INSTRUCTION_LD_YZN}, {`STEP2, `INSTRUCTION_LD_YZN},
			{`STEP1, `INSTRUCTION_STD},
			{`STEP1, `INSTRUCTION_STS},
			{`STEP1, `INSTRUCTION_ST_X},
			{`STEP1, `INSTRUCTION_ST_XP},
			{`STEP1, `INSTRUCTION_ST_XN},
			{`STEP1, `INSTRUCTION_ST_YZP},
			{`STEP1, `INSTRUCTION_ST_YZN},
			{`STEP1, `INSTRUCTION_PUSH}: bus_busy_user = 1'b1;
			default: bus_busy_user = 1'b0;
		endcase
	end
endfunction
// Blocking-assigned early in the clocked block (right after pgm_data_registered's own update),
// not declared as wires -- see that assignment site.
reg bus_busy_this_cycle;
reg out_retire_fire;

// OUT targeting SREG (I/O 0x3F) always uses the 2-cycle path, unconditionally -- SREG is read
// implicitly by nearly the whole ISA (every ALU flag computation, every conditional branch via
// sreg_out), so it isn't a narrow, enumerable hazard like the data bus.
`define OUT_TARGETS_SREG ({pgm_data_registered[10:9],pgm_data_registered[3:0]} == 6'h3F)

int_encoder # (
	.VECTOR_INT_TABLE_SIZE(VECTOR_INT_TABLE_SIZE)
	)int_encoder_inst(
	.rst(core_rst),
	.halt_ack(halt_ack),
	.clk(clk),
	.int_sig_in(int_sig),
	.int_request(int_request),
	.int_vect(current_int_vect_request)
	);
/**************** !Interrupt instance  ********************/
reg [ROM_ADDR_WIDTH - 1:0]PC_SNAPSHOOT;
reg [1:0]last_state;

/*
 * Interrupt registration and CALL to vector table instruction inserter.
 */
always @ *
begin
	pgm_data_int = pgm_data;
	int_registered = 1'b0;
	int_rst = 1'b0;
	PC_SNAPSHOOT = PC;
	if(VECTOR_INT_TABLE_SIZE != 0)
	begin
		if(&{~skip_next_clock, ~cnt_rst, (USE_HALT != "TRUE" | ~halt_int | state_cnt != `STEP0)})
		begin
			case({unlock_int_registered_step_2, int_request, sreg_out[`XMEGA_FLAG_I], state_cnt})
				{3'b011, `STEP0}:
				begin
					//if(~|last_state)
					//begin
					// Synthetic CALL (0x940E) shares CALL's STEP0/STEP1 datapath, so it needs the
					// same bus hazard check plus ~holding_instr: a releasing hold has priority below
					// and would otherwise fire int_registered/int_rst without ever latching the
					// synthetic word -- silently dropping the interrupt.
					if(~(out_retire_pending & bus_busy_user(`STEP0, 16'b1001010000001110)) & ~holding_instr)
					begin
						pgm_data_int = 16'b1001010000001110;
						int_registered = 1'b1;
						int_rst = 1'b1 << (current_int_vect_request - 1);
					end
					//end
				end
				{3'b111, `STEP1}:
				begin
					pgm_data_int = {current_int_vect_registered, 1'b0};
					int_registered = 1'b1;
				end
			endcase
		end
	end
end

// current_int_vect_registered is read a cycle later than PC_SNAPSHOOT, so it needs a real register.
always @ (posedge clk)
begin
	if(VECTOR_INT_TABLE_SIZE != 0)
	begin
		if(&{~skip_next_clock, ~cnt_rst, (USE_HALT != "TRUE" | ~halt_int | state_cnt != `STEP0)})
		begin
			// Same guard as the STEP0 dispatch case above, kept in lockstep.
			if(({unlock_int_registered_step_2, int_request, sreg_out[`XMEGA_FLAG_I], state_cnt} == {3'b011, `STEP0})
				& ~(out_retire_pending & bus_busy_user(`STEP0, 16'b1001010000001110)) & ~holding_instr)
				current_int_vect_registered <= current_int_vect_request;
		end
	end
end

/*
 * System REG write.
 */
always @ (posedge clk)
begin
	if(cnt_rst)
	begin
		if(ROM_ADDR_WIDTH > 16)
		begin
			EIND <= 8'h00;
			RAMPZ <= 8'h00;
			RAMPY <= 8'h00;
			RAMPX <= 8'h00;
			RAMPD <= 8'h00;
		end
	end
	else
	begin
		case({data_addr_int, data_write_int})
			{24'h058, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPD <= data_out_int;
			{24'h059, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPX <= data_out_int;
			{24'h05A, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPY <= data_out_int;
			{24'h05B, 1'b1}: if(ROM_ADDR_WIDTH > 16) RAMPZ <= data_out_int;
			{24'h05C, 1'b1}: if(ROM_ADDR_WIDTH > 16) EIND <= data_out_int;
		endcase
	end
end
/*
 * System REG read & REG write selector.
 */
always @ *
begin
	alu_rs1 = reg_rs1;
	alu_rs2 = reg_rs2;
/* Data bus switch */ /*************************************************************/
	data_addr = data_addr_int;
	data_addr2 = data_addr2_int;
	data_out = data_out_int;
	data_write = data_write_int;
	data_read = data_read_int;
	data_in_int = data_in;
	case(data_addr_int)
		24'h058: if(ROM_ADDR_WIDTH > 16) data_in_int = RAMPD[7:0];
		24'h059: if(ROM_ADDR_WIDTH > 16) data_in_int = RAMPX[7:0];
		24'h05A: if(ROM_ADDR_WIDTH > 16) data_in_int = RAMPY[7:0];
		24'h05B: if(ROM_ADDR_WIDTH > 16) data_in_int = RAMPZ[7:0];
		24'h05C: if(ROM_ADDR_WIDTH > 16) data_in_int = EIND[7:0];
		24'h05D: data_in_int = SP[7:0];
		24'h05E: if(RAM_ADDR_WIDTH > 8) data_in_int = SP[RAM_ADDR_WIDTH - 1:8];
		24'h05F: data_in_int = SREG;
	endcase
/* Set "reg_rd" */ /*************************************************************/
	casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
		{1'b1, `STEP0, `INSTRUCTION_LDS},
		{1'b1, `STEP0, `INSTRUCTION_LDS16},
		{1'b1, `STEP0, `INSTRUCTION_POP},
		{1'b1, `STEP0, `INSTRUCTION_LDD},
		{1'b1, `STEP0, `INSTRUCTION_LD_X},
		{1'b1, `STEP0, `INSTRUCTION_LD_XP},
		{1'b1, `STEP0, `INSTRUCTION_LD_YZP},
		{1'b1, `STEP0, `INSTRUCTION_LD_XN},
		{1'b1, `STEP0, `INSTRUCTION_LD_YZN}: reg_rd = data_in_int;
		
		{1'b1, `STEP2, `INSTRUCTION_LD_XP},
		{1'b1, `STEP1, `INSTRUCTION_ST_XP}: 
		begin
			if(ROM_ADDR_WIDTH > 16)
				{ramp_bits, reg_rd} = {RAMPX, reg_rs2} + 1;
			else
				reg_rd = reg_rs2 + 1;
		end
		{1'b1, `STEP2, `INSTRUCTION_LD_YZP},
		{1'b1, `STEP1, `INSTRUCTION_ST_YZP}: 
		begin
			if(ROM_ADDR_WIDTH > 16)
				{ramp_bits, reg_rd} = {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} + 1;
			else
				reg_rd = reg_rs2 + 1;
		end
		{1'b1, `STEP1, `INSTRUCTION_LPM_R_P}: 
		begin
			if(ROM_ADDR_WIDTH > 16)
				{ramp_bits, reg_rd} = {RAMPZ, reg_rs2} + 1;
			else
				reg_rd = reg_rs2 + 1;
		end
		
		{1'b1, `STEP2, `INSTRUCTION_LD_XN},
		{1'b1, `STEP1, `INSTRUCTION_ST_XN}: 
		begin
			if(ROM_ADDR_WIDTH > 16)
				{ramp_bits, reg_rd} = {RAMPX, reg_rs2} - 1;
			else
				reg_rd = reg_rs2 - 1;
		end
		{1'b1, `STEP2, `INSTRUCTION_LD_YZN},
		{1'b1, `STEP1, `INSTRUCTION_ST_YZN}: 
		begin
			if(ROM_ADDR_WIDTH > 16)
				{ramp_bits, reg_rd} = {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} - 1;
			else
				reg_rd = reg_rs2 - 1;
		end
		{1'b1, `STEP0, `INSTRUCTION_IN}: reg_rd = data_in_int;
		{1'b1, `STEP2, `INSTRUCTION_LPM_R},
		{1'b1, `STEP2, `INSTRUCTION_LPM_ELPM}: reg_rd = reg_rs2[0] ? pgm_data_int[15:8] : pgm_data_int[7:0];
		{1'b1, `STEP2, `INSTRUCTION_LPM_R_P}: reg_rd = ~reg_rs2[0] ? pgm_data_int[15:8] : pgm_data_int[7:0];
		default: reg_rd = alu_rd;
	endcase
/* Set "pgm_addr" */ /*************************************************************/
	casex({execute,  state_cnt, CORE_TYPE, pgm_data_registered})
		{1'b1, `STEP1, `INSTRUCTION_LPM_R},
		{1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
		{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: 
		begin
			if(ROM_ADDR_WIDTH > 16)
				pgm_addr = {RAMPZ, reg_rs2[15:1]};
			else
				pgm_addr = reg_rs2[15:1];
		end
		default: pgm_addr = PC;
	endcase
end

initial begin
	SP = 8'h00;
	SREG = 8'h00;
	data_addr_int = 'h00000000;
	data_addr2_int = 'h00000000;
	data_out_int = 8'h00;
	PC_TMP_H = 8'h00;
	call_word2 = 16'h0000;
	int_rst = 1'b0;
	// Explicit init avoids X in sim / an undefined power-up value in synthesis.
	out_retire_pending = 1'b0;
	out_retire_instr = 16'h0000;
	out_rs1a = 5'h00;
	held_instr = 16'h0000;
	holding_instr = 1'b0;
	two_cycle_hold_next = 1'b0;
end

reg halt_ack_n;

always @ (posedge clk or posedge core_rst)
begin
	if(core_rst)
	begin
		halt_int <= 'h0;
		halt_ack_n <= 'h0;
		cnt_rst = 1'b1;
		PC <= BOOT_ADDR;
		state_cnt <= `STEP0;
		//SP <= 8'h00;
		//SREG <= 8'h00;
		//data_addr_int <= 'h00000000;
		//data_out_int <= 8'h00;
		//io_addr_int <= 6'h00;
		//io_out_int = 8'h00;
		//PC_TMP_H <= 8'h00;
		// Live control state (unlike SP/SREG/etc above): core_rst must actively clear these on
		// a soft reset, since nothing in software can unstick a stuck retirement/hold otherwise.
		out_retire_pending <= 1'b0;
		holding_instr <= 1'b0;
		two_cycle_hold_next <= 1'b0;
	end
	else
	begin
		halt_int <=  halt;
		data_write_int <= 1'b0;
		data_read_int <= 1'b0;
		reg_rdw <= 1'b0;
		skip_next_clock <= 1'b0;
		two_cycle_hold_next <= 1'b0;
		if(&{USE_HALT == "TRUE", halt_int, state_cnt == `STEP0, ~skip_next_clock, ~int_registered})
		begin
			halt_ack <= 1'b1;
			if(WATCHDOG_CNT_WIDTH)
				wdt_rst_out <= 1'b1;
			if({~halt_ack_n, ~|last_state})
			begin
				halt_ack_n <= 1'b1;
				PC <= PC_MINUS_ONE;
			end
		end
		else
		begin
			halt_ack <= 1'b0;
			halt_ack_n <= 1'b0;
			PC <= PC_PLUS_ONE;
			//data_addr_int <= 'h00000000;
			//data_out_int <= 8'h00;
			//io_addr_int <= 6'h00;
			//io_out_int = 8'h00;
			if(cnt_rst)
			begin
				cnt_rst = 1'b0;
				pgm_data_registered = 16'h0000;
			end
			else if(holding_instr)
			begin
					// Re-check the same conflict against the held opcode (not a re-fetch) every cycle
					// until it's safe.
				if(out_retire_pending & bus_busy_user(`STEP0, held_instr))
				begin
						// Still blocked: present NOP, keep PC frozen.
					pgm_data_registered = 16'h0000;
					PC <= PC;
				end
				else
				begin
						// PC is left alone -- already points past this instruction, so the normal
						// fetch pipeline resumes on its own.
					pgm_data_registered = held_instr;
					holding_instr <= 1'b0;
				end
			end
			else if(&{state_cnt == `STEP0, ~skip_next_clock})
			begin
				pgm_data_registered = pgm_data_int;
				// Read-after-write hazard: a pending OUT retirement writes an I/O address one or
				// more cycles after its own decode. If the instruction just latched is itself an
				// I/O-space reader/writer (IN/CBI/SBI/SBIC/SBIS), letting it proceed now would
				// read/write stale data -- so capture it into held_instr, revert pgm_data_registered
				// to NOP, and freeze PC until the pending write clears.
				if(out_retire_pending & bus_busy_user(`STEP0, pgm_data_registered))
				begin
					held_instr <= pgm_data_registered;
					holding_instr <= 1'b1;
					pgm_data_registered = 16'h0000;
					PC <= PC;
				end
				// ADIW/SBIW and the MUL family are 2 cycles (ISM Table 5-2). An extra state_cnt
				// state would delay pgm_data_registered and break the one-decode-behind ALU timing
				// that 1-cycle flag write-back and COND_BRANCH depend on, so hold the next
				// instruction instead. ~int_registered: on an interrupt dispatch cycle
				// pgm_data_registered is the synthetic CALL (0x940e), and holding that corrupts the
				// vector fetch.
				else if(two_cycle_hold_next & ~int_registered)
				begin
					held_instr <= pgm_data_registered;
					holding_instr <= 1'b1;
					pgm_data_registered = 16'h0000;
					PC <= PC;
				end
				casex({CORE_TYPE, pgm_data_registered})
					`INSTRUCTION_ADIW,
					`INSTRUCTION_SBIW,
					`INSTRUCTION_MUL,
					`INSTRUCTION_MULS,
					`INSTRUCTION_MULSU,
					`INSTRUCTION_FMUL,
					`INSTRUCTION_FMULS,
					`INSTRUCTION_FMULSU: two_cycle_hold_next <= 1'b1;
				endcase
			end
				// Blocking-assigned here (right after pgm_data_registered's own update) so every
				// later statement in this pass sees the correct, freshly-decoded value.
			bus_busy_this_cycle = bus_busy_user(state_cnt, pgm_data_registered) | int_registered;
			out_retire_fire = out_retire_pending & ~bus_busy_this_cycle;
			unlock_int_registered_step_2 <= 1'b0;
			rs2a <= {pgm_data_registered[9], pgm_data_registered[3:0]};
			rs1a <= pgm_data_registered[8:4];
			rda <= pgm_data_registered[8:4];
			reg_rs1m <= `REG_MODE_8_BIT;
			reg_rs2m <= `REG_MODE_8_BIT;
			reg_rdm <= `REG_MODE_8_BIT;
			state_cnt <= `STEP0;
			wdt_rst_out <= 1'b0;
	/* Set "Interrupt flag reset & unlock CALL address insert" */ /*************************************************************/
			if(VECTOR_INT_TABLE_SIZE != 0 & int_registered)
				unlock_int_registered_step_2 <= 1'b1;
			if(&{~skip_next_clock, ~cnt_rst})
			begin
				last_state <= state_cnt;
	/* Set "Load default registers state if othervice is not specifyed" */ /*************************************************************/
				SREG <= sreg_out;
	/* Set "WDR" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
				{1'b1, `STEP0, `INSTRUCTION_WDR}: wdt_rst_out <= WATCHDOG_CNT_WIDTH ? 1'b1 : 1'b0;
				endcase
	/* Set "SREG" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP1, `INSTRUCTION_CALL}:
					begin
						if(int_registered)
							SREG[`XMEGA_FLAG_I] <= 1'b0;
					end
					{1'b1, `STEP1, `INSTRUCTION_STS}:
					begin
						// Real "STS 0x5F, Rr" (direct write to SREG). Uses the freshly-decoded
						// address, not the registered data_addr_int (not updated until this
						// same STEP1 -- see the data_addr_int-setting casex further down).
						case(ROM_ADDR_WIDTH > 16 ? {RAMPD, pgm_data_int} : pgm_data_int)
							24'h05F: SREG <= reg_rs1;
						endcase
					end
					{1'b1, `STEP2, `INSTRUCTION_RET},
					{1'b1, `STEP2, `INSTRUCTION_RETI}:
					begin
						if(pgm_data_registered[4])
							SREG[`XMEGA_FLAG_I] <= 1'b1;
					end
					{1'b1, `STEP1, `INSTRUCTION_OUT}:
					begin
						// OUT_TARGETS_SREG path only (that's the only way state_cnt reaches STEP1
						// for OUT now) -- mutually exclusive with the out_retire_fire block below.
						case({pgm_data_registered[10:9],pgm_data_registered[3:0]})
							6'h3F: SREG <= reg_rs1;
						endcase
					end
				endcase
				// OUT-retire write-back (SREG): fires once out_retire_pending's write is free to
				// go -- see out_retire_pending's declaration above for why.
				if(out_retire_fire)
				begin
					case({out_retire_instr[10:9],out_retire_instr[3:0]})
						6'h3F: SREG <= out_reg_rs1;
					endcase
				end
	/* Set "SP" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
				{1'b1, `STEP0, `INSTRUCTION_RCALL},
				{1'b1, `STEP1, `INSTRUCTION_RCALL},
				{1'b1, `STEP0, `INSTRUCTION_CALL},
				{1'b1, `STEP1, `INSTRUCTION_CALL},
				{1'b1, `STEP0, `INSTRUCTION_ICALL},
				{1'b1, `STEP1, `INSTRUCTION_ICALL},
				{1'b1, `STEP1, `INSTRUCTION_PUSH}: SP <= SP - 1;
				{1'b1, `STEP0, `INSTRUCTION_POP}: SP <= SP_PLUS_ONE;
				{1'b1, `STEP0, `INSTRUCTION_RET},
				{1'b1, `STEP0, `INSTRUCTION_RETI}: SP <= SP_PLUS_TWO;
				{1'b1, `STEP1, `INSTRUCTION_STS}:
				begin
					case(pgm_data_int)
						24'h005D: SP[7:0] <= reg_rs1;
						24'h005E:
						begin
							if(RAM_ADDR_WIDTH > 8)
								SP[RAM_ADDR_WIDTH - 1:8] <= reg_rs1;
						end
					endcase
				end
				{1'b1, `STEP1, `INSTRUCTION_OUT}:
				begin
					// Hazard-fallback path only -- see the matching comment in "Set SREG" above.
					case({pgm_data_registered[10:9],pgm_data_registered[3:0]})
						6'h3D: SP[7:0] <= reg_rs1;
						6'h3E:
						begin
							if(RAM_ADDR_WIDTH > 8)
								SP[RAM_ADDR_WIDTH - 1:8] <= reg_rs1;
						end
					endcase
				end
				endcase
				// OUT-retire write-back (SP): one cycle after decode, decoupled from state_cnt.
				if(out_retire_fire)
				begin
					case({out_retire_instr[10:9],out_retire_instr[3:0]})
						6'h3D: SP[7:0] <= out_reg_rs1;
						6'h3E:
						begin
							if(RAM_ADDR_WIDTH > 8)
								SP[RAM_ADDR_WIDTH - 1:8] <= out_reg_rs1;
						end
					endcase
				end
	/* Set "rs1a" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_MULS}: rs1a[4] <= 1'b1;
					{1'b1, `STEP0, `INSTRUCTION_FMUL},
					{1'b1, `STEP0, `INSTRUCTION_FMULS},
					{1'b1, `STEP0, `INSTRUCTION_MULSU},
					{1'b1, `STEP0, `INSTRUCTION_FMULSU}: rs1a[4:3] <= 2'b10;
					{1'b1, `STEP0, `INSTRUCTION_CPI}: rs1a[4] <= 1'b1;
					{1'b1, `STEP0, `INSTRUCTION_SUBI},
					{1'b1, `STEP0, `INSTRUCTION_SBCI},
					{1'b1, `STEP0, `INSTRUCTION_ORI_SBR},
					{1'b1, `STEP0, `INSTRUCTION_ANDI_CBR}: rs1a[4] <= 1'b1;
					{1'b1, `STEP0, `INSTRUCTION_ADIW},
					{1'b1, `STEP0, `INSTRUCTION_SBIW}: rs1a <= {2'b11, pgm_data_registered[5:4], 1'b0};
					{1'b1, `STEP0, `INSTRUCTION_ICALL}: rs1a <= 5'b11110;
				endcase
	/* Set "reg_rs1m" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_ADIW},
					{1'b1, `STEP0, `INSTRUCTION_SBIW},
					{1'b1, `STEP0, `INSTRUCTION_ICALL}: reg_rs1m <= `REG_MODE_16_BIT;
				endcase
	/* Set "rs2a" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_MOVW}: rs2a <= {pgm_data_registered[3:0], 1'b0};
					{1'b1, `STEP0, `INSTRUCTION_MULS}: rs2a[4] <= 1'b1;
					{1'b1, `STEP0, `INSTRUCTION_FMUL},
					{1'b1, `STEP0, `INSTRUCTION_FMULS},
					{1'b1, `STEP0, `INSTRUCTION_MULSU},
					{1'b1, `STEP0, `INSTRUCTION_FMULSU}: rs2a[4:3] <= 2'b10;
					{1'b1, `STEP0, `INSTRUCTION_LDD},
					{1'b1, `STEP0, `INSTRUCTION_STD},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZN}: rs2a <= {3'b111, ~pgm_data_registered[3], 1'b0};
					{1'b1, `STEP0, `INSTRUCTION_LD_X},
					{1'b1, `STEP1, `INSTRUCTION_LD_X},
					{1'b1, `STEP0, `INSTRUCTION_ST_X},
					{1'b1, `STEP1, `INSTRUCTION_ST_X},
					{1'b1, `STEP0, `INSTRUCTION_LD_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_ST_XP},
					{1'b1, `STEP0, `INSTRUCTION_LD_XN},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN},
					{1'b1, `STEP1, `INSTRUCTION_ST_XN}: rs2a <= 5'b11010;
					{1'b1, `STEP0, `INSTRUCTION_LPM_R},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP0, `INSTRUCTION_LPM_ELPM},
					{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: rs2a <= 5'b11110;
				endcase
	/* Set "reg_rs2m" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_MOVW},
					{1'b1, `STEP0, `INSTRUCTION_LDD},
					{1'b1, `STEP0, `INSTRUCTION_STD},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP0, `INSTRUCTION_LD_X},
					{1'b1, `STEP1, `INSTRUCTION_LD_X},
					{1'b1, `STEP0, `INSTRUCTION_ST_X},
					{1'b1, `STEP1, `INSTRUCTION_ST_X},
					{1'b1, `STEP0, `INSTRUCTION_LD_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_ST_XP},
					{1'b1, `STEP0, `INSTRUCTION_LD_XN},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN},
					{1'b1, `STEP1, `INSTRUCTION_ST_XN},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP0, `INSTRUCTION_LPM_ELPM},
					{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: reg_rs2m <= `REG_MODE_16_BIT;
				endcase
	/* Set "skip_next_clock" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_COND_BRANCH}: /***********************/
					begin
						if(sreg_out[pgm_data_registered[2:0]] == ~pgm_data_registered[10])
							skip_next_clock <= 1'b1;
					end
					{1'b1, `STEP1, `INSTRUCTION_CPSE}:
					begin
						if(reg_rs1 == reg_rs2)
						begin
							casex({CORE_TYPE, pgm_data_int})
								`INSTRUCTION_LDS,
								`INSTRUCTION_STS,
								`INSTRUCTION_JMP,
								`INSTRUCTION_CALL: ;
								default: skip_next_clock <= 1'b1;
							endcase
						end
					end
					{1'b1, `STEP1, `INSTRUCTION_SBRC_SBRS}:
					begin
						if(reg_rs1[pgm_data_registered[2:0]] == pgm_data_registered[9])
						begin
							casex({CORE_TYPE, pgm_data_int})
								`INSTRUCTION_LDS,
								`INSTRUCTION_STS,
								`INSTRUCTION_JMP,
								`INSTRUCTION_CALL: ;
								default: skip_next_clock <= 1'b1;
							endcase
						end
					end
					{1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}:
					begin
						if(data_in_int[pgm_data_registered[2:0]] == pgm_data_registered[9])
						begin
							casex({CORE_TYPE, pgm_data_int})
								`INSTRUCTION_LDS,
								`INSTRUCTION_STS,
								`INSTRUCTION_JMP,
								`INSTRUCTION_CALL: ;
								default: skip_next_clock <= 1'b1;
							endcase
						end
					end
					{1'b1, `STEP0, `INSTRUCTION_RJMP},
					{1'b1, `STEP0, `INSTRUCTION_IJMP},
					{1'b1, `STEP1, `INSTRUCTION_JMP},
					{1'b1, `STEP1, `INSTRUCTION_RCALL},
					{1'b1, `STEP1, `INSTRUCTION_ICALL},
					{1'b1, `STEP2, `INSTRUCTION_CALL},
					{1'b1, `STEP2, `INSTRUCTION_RET},
					{1'b1, `STEP2, `INSTRUCTION_RETI}: skip_next_clock <= 1'b1;
				endcase
	/* Set "rda" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_MOVW}: rda <= {pgm_data_registered[7:4], 1'b0};
					{1'b1, `STEP0, `INSTRUCTION_LDI}: rda[4] <= 1'b1;
					{1'b1, `STEP0, `INSTRUCTION_MUL},
					{1'b1, `STEP0, `INSTRUCTION_MULS},
					{1'b1, `STEP0, `INSTRUCTION_FMUL},
					{1'b1, `STEP0, `INSTRUCTION_FMULS},
					{1'b1, `STEP0, `INSTRUCTION_MULSU},
					{1'b1, `STEP0, `INSTRUCTION_FMULSU},
					{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: rda <= 5'h00;
					{1'b1, `STEP0, `INSTRUCTION_SUBI},
					{1'b1, `STEP0, `INSTRUCTION_SBCI},
					{1'b1, `STEP0, `INSTRUCTION_ORI_SBR},
					{1'b1, `STEP0, `INSTRUCTION_ANDI_CBR}: rda[4] <= 1'b1;
					{1'b1, `STEP0, `INSTRUCTION_ADIW},
					{1'b1, `STEP0, `INSTRUCTION_SBIW}: rda <= {2'b11, pgm_data_registered[5:4], 1'b0};
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN}: rda <= {3'b111, ~pgm_data_registered[3], 1'b0};
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN}: rda <= 5'b11010;
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P}: rda <= 5'b11110;
				endcase
	/* Set "reg_rdm" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_MOVW},
					{1'b1, `STEP0, `INSTRUCTION_MUL},
					{1'b1, `STEP0, `INSTRUCTION_MULS},
					{1'b1, `STEP0, `INSTRUCTION_FMUL},
					{1'b1, `STEP0, `INSTRUCTION_FMULS},
					{1'b1, `STEP0, `INSTRUCTION_MULSU},
					{1'b1, `STEP0, `INSTRUCTION_FMULSU},
					{1'b1, `STEP0, `INSTRUCTION_ADIW},
					{1'b1, `STEP0, `INSTRUCTION_SBIW},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P}: reg_rdm <= `REG_MODE_16_BIT;
				endcase
	/* Set "reg_rdw" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_LDI},
					{1'b1, `STEP0, `INSTRUCTION_MOVW},
					{1'b1, `STEP0, `INSTRUCTION_MUL},
					{1'b1, `STEP0, `INSTRUCTION_MULS},
					{1'b1, `STEP0, `INSTRUCTION_FMUL},
					{1'b1, `STEP0, `INSTRUCTION_FMULS},
					{1'b1, `STEP0, `INSTRUCTION_MULSU},
					{1'b1, `STEP0, `INSTRUCTION_FMULSU},
					{1'b1, `STEP0, `INSTRUCTION_INC},
					{1'b1, `STEP0, `INSTRUCTION_DEC},
					{1'b1, `STEP0, `INSTRUCTION_ASR},
					{1'b1, `STEP0, `INSTRUCTION_LSR},
					{1'b1, `STEP0, `INSTRUCTION_ROR},
					{1'b1, `STEP0, `INSTRUCTION_SUB},
					{1'b1, `STEP0, `INSTRUCTION_SBC},
					{1'b1, `STEP0, `INSTRUCTION_ADD},
					{1'b1, `STEP0, `INSTRUCTION_ADC},
					{1'b1, `STEP0, `INSTRUCTION_SWAP},
					{1'b1, `STEP0, `INSTRUCTION_AND},
					{1'b1, `STEP0, `INSTRUCTION_EOR},
					{1'b1, `STEP0, `INSTRUCTION_OR},
					{1'b1, `STEP0, `INSTRUCTION_MOV},
					{1'b1, `STEP0, `INSTRUCTION_BLD},
					{1'b1, `STEP0, `INSTRUCTION_SUBI},
					{1'b1, `STEP0, `INSTRUCTION_SBCI},
					{1'b1, `STEP0, `INSTRUCTION_ORI_SBR},
					{1'b1, `STEP0, `INSTRUCTION_ANDI_CBR},
					{1'b1, `STEP0, `INSTRUCTION_COM},
					{1'b1, `STEP0, `INSTRUCTION_NEG},
					{1'b1, `STEP0, `INSTRUCTION_ADIW},
					{1'b1, `STEP0, `INSTRUCTION_SBIW},
					{1'b1, `STEP1, `INSTRUCTION_POP},
					{1'b1, `STEP2, `INSTRUCTION_LDS},
					{1'b1, `STEP1, `INSTRUCTION_LDS16},
					{1'b1, `STEP2, `INSTRUCTION_LDD},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP2, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP2, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP2, `INSTRUCTION_LD_X},
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP2, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP2, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN},
					{1'b1, `STEP0, `INSTRUCTION_IN}, // IN-only: doesn't read reg_rs1, only writes via rda/reg_rdw -- independent of OUT's own (still 2-cycle) timing.
					{1'b1, `STEP1, `INSTRUCTION_LPM_R},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: reg_rdw <= 1'b1;
				endcase
	/* Set "data_addr" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_RCALL},
					{1'b1, `STEP1, `INSTRUCTION_RCALL},
					{1'b1, `STEP0, `INSTRUCTION_CALL},
					{1'b1, `STEP1, `INSTRUCTION_CALL},
					{1'b1, `STEP0, `INSTRUCTION_ICALL},
					{1'b1, `STEP1, `INSTRUCTION_ICALL},
					{1'b1, `STEP1, `INSTRUCTION_PUSH}: data_addr_int <= SP;
					{1'b1, `STEP0, `INSTRUCTION_RET},
					{1'b1, `STEP0, `INSTRUCTION_RETI}: data_addr_int <= SP_PLUS_ONE;
					{1'b1, `STEP0, `INSTRUCTION_POP}: data_addr_int <= SP_PLUS_ONE;
					{1'b1, `STEP1, `INSTRUCTION_POP}: data_addr_int <= SP;
					{1'b1, `STEP1, `INSTRUCTION_LDS}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPD, pgm_data_int};
						else
							data_addr_int <= pgm_data_int;
					end
					{1'b1, `STEP2, `INSTRUCTION_LDS}: data_addr_int <= data_addr_int;
					{1'b1, `STEP0, `INSTRUCTION_LDS16}: data_addr_int <= {pgm_data_registered[10:8], pgm_data_registered[3:0]};
					{1'b1, `STEP1, `INSTRUCTION_LDS16}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_STS}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPD, pgm_data_int};
						else
							data_addr_int <= pgm_data_int;
					end
					{1'b1, `STEP0, `INSTRUCTION_STS16}: data_addr_int <= {pgm_data_registered[10:8], pgm_data_registered[3:0]};
					{1'b1, `STEP1, `INSTRUCTION_LDD}:  
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
						else
							data_addr_int <= reg_rs2 + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
					end
					{1'b1, `STEP2, `INSTRUCTION_LDD}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_STD}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
						else
							data_addr_int <= reg_rs2 + {pgm_data_registered[13], pgm_data_registered[11:10], pgm_data_registered[2:0]};
					end
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2};
						else
							data_addr_int <= reg_rs2;
					end
					{1'b1, `STEP2, `INSTRUCTION_LD_YZP}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_ST_YZP}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2};
						else
							data_addr_int <= reg_rs2;
					end
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} - 1;
						else
							data_addr_int <= reg_rs2 - 1;
					end
					{1'b1, `STEP2, `INSTRUCTION_LD_YZN}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_ST_YZN}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {pgm_data_registered[3] ? RAMPY : RAMPZ, reg_rs2} - 1;
						else
							data_addr_int <= reg_rs2 - 1;
					end
					{1'b1, `STEP1, `INSTRUCTION_LD_X}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPX, reg_rs2};
						else
							data_addr_int <= reg_rs2;
					end
					{1'b1, `STEP2, `INSTRUCTION_LD_X}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_ST_X}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPX, reg_rs2};
						else
							data_addr_int <= reg_rs2;
					end
					{1'b1, `STEP1, `INSTRUCTION_LD_XP}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPX, reg_rs2};
						else
							data_addr_int <= reg_rs2;
					end
					{1'b1, `STEP2, `INSTRUCTION_LD_XP}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_ST_XP}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPX, reg_rs2};
						else
							data_addr_int <= reg_rs2;
					end
					{1'b1, `STEP1, `INSTRUCTION_LD_XN}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPX, reg_rs2} - 1;
						else
							data_addr_int <= reg_rs2 - 1;
					end
					{1'b1, `STEP2, `INSTRUCTION_LD_XN}: data_addr_int <= data_addr_int;
					{1'b1, `STEP1, `INSTRUCTION_ST_XN}: 
					begin
						if(ROM_ADDR_WIDTH > 16) 
							data_addr_int <= {RAMPX, reg_rs2} - 1;
						else
							data_addr_int <= reg_rs2 - 1;
					end
					{1'b1, `STEP0, `INSTRUCTION_IN},
					{1'b1, `STEP1, `INSTRUCTION_IN}: data_addr_int <= {pgm_data_registered[10:9],pgm_data_registered[3:0]} + 'h20;
					{1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
					{1'b1, `STEP1, `INSTRUCTION_CBI_SBI},
					{1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
					{1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}: data_addr_int <= pgm_data_registered[7:3] + 'h20;
					{1'b1, `STEP1, `INSTRUCTION_OUT}: data_addr_int <= {pgm_data_registered[10:9],pgm_data_registered[3:0]} + 'h20;
				endcase
				// OUT-retire write-back (I/O address): one cycle after decode.
				if(out_retire_fire)
					data_addr_int <= {out_retire_instr[10:9],out_retire_instr[3:0]} + 'h20;
	/* Set "data_addr2" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_RET},
					{1'b1, `STEP0, `INSTRUCTION_RETI}: data_addr2_int <= SP_PLUS_TWO;
				endcase
	/* Set "data_out" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_RCALL}: data_out_int <= PC;
					{1'b1, `STEP1, `INSTRUCTION_RCALL}: data_out_int <= PC[ROM_ADDR_WIDTH - 1:8];
					{1'b1, `STEP0, `INSTRUCTION_CALL}: 
					begin
						data_out_int <= PC_PLUS_ONE[7:0];
						if(int_registered)
							data_out_int <= PC_SNAPSHOOT;
					end
					{1'b1, `STEP1, `INSTRUCTION_CALL}: data_out_int <= PC_TMP_H;
					{1'b1, `STEP0, `INSTRUCTION_ICALL}: data_out_int <= PC[7:0];
					{1'b1, `STEP1, `INSTRUCTION_ICALL}: data_out_int <= PC_TMP_H;
					{1'b1, `STEP1, `INSTRUCTION_PUSH},
					{1'b1, `STEP1, `INSTRUCTION_STS},
					{1'b1, `STEP0, `INSTRUCTION_STS16},
					{1'b1, `STEP1, `INSTRUCTION_STD},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP1, `INSTRUCTION_ST_X},
					{1'b1, `STEP1, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_ST_XN}: data_out_int <= reg_rs1;
					{1'b1, `STEP1, `INSTRUCTION_CBI_SBI}:
					begin
						if(pgm_data_registered[9])
							data_out_int <= data_in | (2 ** pgm_data_registered[2:0]);
						else
							data_out_int <= data_in & ~(2 ** pgm_data_registered[2:0]);
					end
					{1'b1, `STEP1, `INSTRUCTION_OUT}: data_out_int <= reg_rs1;
				endcase
				// OUT-retire write-back (data value): one cycle after decode.
				// Must stay non-blocking: Quartus 17.0 rejects mixing blocking and non-blocking
				// assignment to the same reg within one always block (error 10110).
				if(out_retire_fire)
					data_out_int <= out_reg_rs1;
	/* Set "data_write" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_RCALL},
					{1'b1, `STEP1, `INSTRUCTION_RCALL},
					{1'b1, `STEP0, `INSTRUCTION_CALL},
					{1'b1, `STEP1, `INSTRUCTION_CALL},
					{1'b1, `STEP0, `INSTRUCTION_ICALL},
					{1'b1, `STEP1, `INSTRUCTION_ICALL},
					{1'b1, `STEP1, `INSTRUCTION_PUSH},
					{1'b1, `STEP0, `INSTRUCTION_STS16},
					{1'b1, `STEP1, `INSTRUCTION_STD},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP1, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP1, `INSTRUCTION_ST_X},
					{1'b1, `STEP1, `INSTRUCTION_ST_XP},
					{1'b1, `STEP1, `INSTRUCTION_ST_XN},
					{1'b1, `STEP1, `INSTRUCTION_STS},
					{1'b1, `STEP1, `INSTRUCTION_CBI_SBI},
					{1'b1, `STEP1, `INSTRUCTION_OUT}: data_write_int <= 1'b1;
				endcase
				// OUT-retire write-back (write strobe): one cycle after decode.
				if(out_retire_fire)
					data_write_int <= 1'b1;
	/* Set "data_read" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP1, `INSTRUCTION_RET},
					{1'b1, `STEP1, `INSTRUCTION_RETI},
					{1'b1, `STEP1, `INSTRUCTION_POP},
					{1'b1, `STEP1, `INSTRUCTION_LDS16},
					{1'b1, `STEP2, `INSTRUCTION_LDD},
					{1'b1, `STEP2, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP2, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP2, `INSTRUCTION_LD_X},
					{1'b1, `STEP2, `INSTRUCTION_LD_XP},
					{1'b1, `STEP2, `INSTRUCTION_LD_XN},
					{1'b1, `STEP2, `INSTRUCTION_LDS},
					{1'b1, `STEP0, `INSTRUCTION_IN}, // IN-only 1-cycle fix
					{1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
					{1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
					{1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}: data_read_int <= 1'b1;
				endcase
	/* Set "state_cnt" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_JMP},
					{1'b1, `STEP0, `INSTRUCTION_RCALL},
					{1'b1, `STEP0, `INSTRUCTION_CALL},
					{1'b1, `STEP0, `INSTRUCTION_ICALL},
					{1'b1, `STEP0, `INSTRUCTION_PUSH},
					{1'b1, `STEP0, `INSTRUCTION_RET},
					{1'b1, `STEP0, `INSTRUCTION_RETI},
					{1'b1, `STEP0, `INSTRUCTION_POP},
					{1'b1, `STEP0, `INSTRUCTION_CPSE},
					{1'b1, `STEP0, `INSTRUCTION_SBRC_SBRS},
					{1'b1, `STEP0, `INSTRUCTION_LDS},
					{1'b1, `STEP0, `INSTRUCTION_LDS16},
					{1'b1, `STEP0, `INSTRUCTION_STS},
					{1'b1, `STEP0, `INSTRUCTION_LDD},
					{1'b1, `STEP0, `INSTRUCTION_STD},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP0, `INSTRUCTION_LD_X},
					{1'b1, `STEP0, `INSTRUCTION_ST_X},
					{1'b1, `STEP0, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP0, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN},
					{1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
					{1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP0, `INSTRUCTION_LPM_ELPM}: state_cnt <= `STEP1;
					// OUT stays STEP0 (1-cycle) except OUT_TARGETS_SREG, which always uses the
					// 2-cycle STEP0->STEP1 path. A stalled OUT (blocked behind a pending
					// retirement) also stays at STEP0 and simply re-latches next cycle.
					{1'b1, `STEP0, `INSTRUCTION_OUT}: if(`OUT_TARGETS_SREG) state_cnt <= `STEP1;
	/*************************************************************/
					{1'b1, `STEP1, `INSTRUCTION_CALL},
					{1'b1, `STEP1, `INSTRUCTION_RET},
					{1'b1, `STEP1, `INSTRUCTION_RETI}: state_cnt <= `STEP2;
					{1'b1, `STEP1, `INSTRUCTION_CPSE}:
					begin
						if(reg_rs1 == reg_rs2)
						begin
							casex({CORE_TYPE, pgm_data_int})
							`INSTRUCTION_LDS,
							`INSTRUCTION_STS,
							`INSTRUCTION_JMP,
							`INSTRUCTION_CALL: state_cnt <= `STEP2;
							endcase
						end
					end
					{1'b1, `STEP1, `INSTRUCTION_SBRC_SBRS}:
					begin
						if(reg_rs1[pgm_data_registered[2:0]] == pgm_data_registered[9])
						begin
							casex({CORE_TYPE, pgm_data_int})
							`INSTRUCTION_LDS,
							`INSTRUCTION_STS,
							`INSTRUCTION_JMP,
							`INSTRUCTION_CALL: state_cnt <= `STEP2;
							endcase
						end
					end
					{1'b1, `STEP1, `INSTRUCTION_SBIC_SBIS}:
					begin
						if(data_in_int[pgm_data_registered[2:0]] == pgm_data_registered[9])
						begin
							casex({CORE_TYPE, pgm_data_int})
							`INSTRUCTION_LDS,
							`INSTRUCTION_STS,
							`INSTRUCTION_JMP,
							`INSTRUCTION_CALL: state_cnt <= `STEP2;
							endcase
						end
					end
					{1'b1, `STEP1, `INSTRUCTION_LDS},
					{1'b1, `STEP1, `INSTRUCTION_LDD},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP1, `INSTRUCTION_LD_X},
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: state_cnt <= `STEP2;
	/*************************************************************/
					{1'b1, `STEP2, `INSTRUCTION_CPSE},
					{1'b1, `STEP2, `INSTRUCTION_SBRC_SBRS},
					{1'b1, `STEP2, `INSTRUCTION_SBIC_SBIS}: state_cnt <= `STEP3;
				endcase
	/* Set "out_retire_pending" */ /*************************************************************/
				// A pending write fires whenever the bus is free, regardless of what's decoding this
				// cycle -- evaluated first so a new OUT arming below can immediately re-arm it.
				if(out_retire_fire)
					out_retire_pending <= 1'b0;
				// Arms a new pending write only when safe: not SREG (unconditionally the STEP1
				// path above) and not stalled behind an existing pending write. out_rs1a mirrors
				// rs1a's own default assignment into the dedicated, non-shared register above.
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_OUT}:
					begin
						if(~`OUT_TARGETS_SREG & ~(out_retire_pending & bus_busy_this_cycle))
						begin
							out_retire_pending <= 1'b1;
							out_retire_instr <= pgm_data_registered;
							out_rs1a <= pgm_data_registered[8:4];
						end
					end
				endcase
	/* Set "PC_TMP_H" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_CALL}: 
					begin
						PC_TMP_H <= PC_PLUS_ONE[ROM_ADDR_WIDTH - 1:8];
						if(int_registered)
							PC_TMP_H <= PC_SNAPSHOOT[ROM_ADDR_WIDTH - 1:8];
					end
					{1'b1, `STEP0, `INSTRUCTION_ICALL}: PC_TMP_H <= PC[ROM_ADDR_WIDTH - 1:8];
					{1'b1, `STEP1, `INSTRUCTION_CALL}: call_word2 <= pgm_data_int;
				endcase
	/* Set "PC" */ /*************************************************************/
				casex({execute, state_cnt, CORE_TYPE, pgm_data_registered})
					{1'b1, `STEP0, `INSTRUCTION_RJMP}: PC <= PC + {{ROM_ADDR_WIDTH - 11{pgm_data_registered[11]}}, pgm_data_registered[10:0]};
					{1'b1, `STEP0, `INSTRUCTION_COND_BRANCH}: /***********************/
					begin
						if(sreg_out[pgm_data_registered[2:0]] == ~pgm_data_registered[10])
							PC <= PC + {{ROM_ADDR_WIDTH - 6{pgm_data_registered[9]}}, pgm_data_registered[8:3]};
					end
					{1'b1, `STEP0, `INSTRUCTION_IJMP}: PC <= ijmp_z;
					{1'b1, `STEP1, `INSTRUCTION_JMP}: PC <= {pgm_data_registered[8:4], pgm_data_registered[0], pgm_data_int};
					{1'b1, `STEP0, `INSTRUCTION_RCALL}:  PC <= PC;
					{1'b1, `STEP1, `INSTRUCTION_RCALL}: PC <= PC + {{ROM_ADDR_WIDTH - 11{pgm_data_registered[11]}}, pgm_data_registered[10:0]};
					{1'b1, `STEP1, `INSTRUCTION_CALL}: PC <= PC;
					{1'b1, `STEP2, `INSTRUCTION_CALL}: PC <= {pgm_data_registered[8:4], pgm_data_registered[0], call_word2};
					{1'b1, `STEP1, `INSTRUCTION_ICALL}: PC <= reg_rs1;
					{1'b1, `STEP0, `INSTRUCTION_PUSH},
					{1'b1, `STEP0, `INSTRUCTION_RET},
					{1'b1, `STEP0, `INSTRUCTION_RETI},
					{1'b1, `STEP1, `INSTRUCTION_RET},
					{1'b1, `STEP1, `INSTRUCTION_RETI}: PC <= PC;
					// Both stack bytes land the same cycle (data_addr2/data_in2 are RET/RETI's
					// own dedicated port, presented in parallel with data_addr/data_in) --
					// data_in_int is the high byte (addr SP+1), data_in2 the low byte (SP+2).
					{1'b1, `STEP2, `INSTRUCTION_RET}: PC <= {data_in_int, data_in2};
					{1'b1, `STEP2, `INSTRUCTION_RETI}: PC <= {data_in_int, data_in2} - 1;
					{1'b1, `STEP0, `INSTRUCTION_CPSE},
					{1'b1, `STEP0, `INSTRUCTION_SBRC_SBRS},
					{1'b1, `STEP0, `INSTRUCTION_POP},
					{1'b1, `STEP1, `INSTRUCTION_LDS},
					{1'b1, `STEP0, `INSTRUCTION_LDS16},
					{1'b1, `STEP0, `INSTRUCTION_LDD},
					{1'b1, `STEP1, `INSTRUCTION_LDD},
					{1'b1, `STEP0, `INSTRUCTION_STD},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZP},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZP},
					{1'b1, `STEP0, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP1, `INSTRUCTION_LD_YZN},
					{1'b1, `STEP0, `INSTRUCTION_ST_YZN},
					{1'b1, `STEP0, `INSTRUCTION_LD_X},
					{1'b1, `STEP1, `INSTRUCTION_LD_X},
					{1'b1, `STEP0, `INSTRUCTION_ST_X},
					{1'b1, `STEP0, `INSTRUCTION_LD_XP},
					{1'b1, `STEP1, `INSTRUCTION_LD_XP},
					{1'b1, `STEP0, `INSTRUCTION_ST_XP},
					{1'b1, `STEP0, `INSTRUCTION_LD_XN},
					{1'b1, `STEP1, `INSTRUCTION_LD_XN},
					{1'b1, `STEP0, `INSTRUCTION_ST_XN},
					{1'b1, `STEP0, `INSTRUCTION_CBI_SBI},
					{1'b1, `STEP0, `INSTRUCTION_SBIC_SBIS},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R},
					{1'b1, `STEP0, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP1, `INSTRUCTION_LPM_R_P},
					{1'b1, `STEP0, `INSTRUCTION_LPM_ELPM},
					{1'b1, `STEP1, `INSTRUCTION_LPM_ELPM}: PC <= PC;
					// OUT freezes PC only for OUT_TARGETS_SREG's old path or while genuinely stalled
					// (a retirement pending and the bus busy this cycle) -- otherwise 1-cycle throughput.
					{1'b1, `STEP0, `INSTRUCTION_OUT}: if(`OUT_TARGETS_SREG | (out_retire_pending & bus_busy_this_cycle)) PC <= PC;
				endcase
			end
		end
	end
end

mega_alu #(
	.CORE_TYPE(CORE_TYPE)
)mega_alu_inst(
	.inst(pgm_data_registered),
	.rda(rs1a),
	.rd(alu_rs1),
	.rra(rs2a),
	.rr(alu_rs2),
	.R(alu_rd),
	.sreg_in(SREG),
	.sreg_out(sreg_out)
);

mega_regs #(
	.PLATFORM("XILINX"),
	.REGISTERED(REGS_REGISTERED)
)mega_regs_inst(
	.rst(core_rst),
	.clk(clk),
	.rs1a(rs1a),
	.rs1(reg_rs1),
	.rs1m(reg_rs1m),
	.rs2a(rs2a),
	.rs2(reg_rs2),
	.rs2m(reg_rs2m),
	.rda(rda),
	.rd(reg_rd),
	.rdw(reg_rdw),
	.rdm(reg_rdm),
	.rs3a(out_rs1a),
	.rs3(out_reg_rs1),
	.z_reg(ijmp_z)
);

watchdog # (
	.CNT_WIDTH(WATCHDOG_CNT_WIDTH)
	)wdt_inst(
	.rst(rst),
	.clk(clk),
	.wdt_clk(clk_wdt),
	.wdt_rst_in(wdt_rst_out),
	.wdt_rst_out(core_rst)
);

/* DEBUG */
/*reg [ROM_ADDR_WIDTH - 1:0]pgm_addr_del;
always @ (posedge clk)
begin
	pgm_addr_del <= pgm_addr;
end
initial begin
	$monitor("Time=%t : ADDR_OUT=%h, INST=%h, INST_REG=%h", $realtime, pgm_addr_del * 2, pgm_data_int, pgm_data_registered);
end*/
/* !DEBUG */

endmodule
