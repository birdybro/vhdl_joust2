--===========================================================================--
--                                                                           --
--        Synthesizable 6809 instruction compatible VHDL CPU core            --
--                                                                           --
--===========================================================================--
--
-- File name      : cpu09l.vhd
--
-- Entity name    : cpu09
--
-- Purpose        : 6809 instruction compatible CPU core written in VHDL
--                  with Last Instruction Cycle, bus available, bus status,
--                  and instruction fetch signals.
--                  Not cycle compatible with the original 6809 CPU
--
-- Dependencies   : ieee.std_logic_1164
--                  ieee.std_logic_unsigned
--
-- Author         : John E. Kent
--
-- Email          : dilbert57@opencores.org      
--
-- Web            : http://opencores.org/project,system09
--
-- Description    : VMA (valid memory address) is hight whenever a valid memory
--                  access is made by an instruction fetch, interrupt vector fetch
--                  or a data read or write otherwise it is low indicating an idle
--                  bus cycle.
--                  IFETCH (instruction fetch output) is high whenever an
--                  instruction byte is read i.e. the program counter is applied 
--                  to the address bus.
--                  LIC (last instruction cycle output) is normally low
--                  but goes high on the last cycle of an instruction.
--                  BA (bus available output) is normally low but goes high while
--                  waiting in a Sync instruction state or the CPU is halted
--                  i.e. a DMA grant.
--                  BS (bus status output) is normally low but goes high during an
--                  interrupt or reset vector fetch or the processor is halted
--                  i.e. a DMA grant.
-- 
--  Copyright (C) 2003 - 2010 John Kent
--
--  This program is free software: you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation, either version 3 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.
--
--===========================================================================--
--                                                                           --
--                                Revision History                           --
--                                                                           --
--===========================================================================--
--
-- Version 0.1 - 26 June 2003 - John Kent
-- Added extra level in state stack
-- fixed some calls to the extended addressing state
--
-- Version 0.2 - 5 Sept 2003 - John Kent
-- Fixed 16 bit indexed offset (was doing read rather than fetch)
-- Added/Fixed STY and STS instructions.
-- ORCC_STATE ANDed CC state rather than ORed it - Now fixed
-- CMPX Loaded ACCA and ACCB - Now fixed 
--
-- Version 1.0 - 6 Sep 2003 - John Kent 
-- Initial release to Open Cores
-- reversed clock edge
--
-- Version 1.1 - 29 November 2003 John kent
--	ACCA and ACCB indexed offsets are 2's complement.
-- ALU Right Mux now sign extends ACCA & ACCB offsets
-- Absolute Indirect addressing performed a read on the
-- second byte of the address rather than a fetch
-- so it formed an incorrect address. Now fixed. 
--
-- Version 1.2 - 29 November 2003 John Kent
-- LEAX and LEAY affect the Z bit only
--	LEAS and LEAU do not affect any condition codes
-- added an extra ALU control for LEA.
--
-- Version 1.3 - 12 December 2003 John Kent
-- CWAI did not work, was missed a PUSH_ST on calling
-- the ANDCC_STATE. Thanks go to Ghassan Kraidy for
-- finding this fault.
--
-- Version 1.4 - 12 December 2003 John Kent
-- Missing cc_ctrl assignment in otherwise case of 
-- lea_state resulted in cc_ctrl being latched in
-- that state.	
-- The otherwise statement should never be reached,
-- and has been fixed simply to resolve synthesis warnings.
--
-- Version 1.5 - 17 january 2004 John kent
-- The clear instruction used "alu_ld8" to control the ALU
-- rather than "alu_clr". This mean the Carry was not being
-- cleared correctly.
--
-- Version 1.6 - 24 January 2004 John Kent
-- Fixed problems in PSHU instruction
--
-- Version 1.7 - 25 January 2004 John Kent
-- removed redundant "alu_inx" and "alu_dex'
-- Removed "test_alu" and "test_cc"
-- STD instruction did not set condition codes
-- JMP direct was not decoded properly
-- CLR direct performed an unwanted read cycle
-- Bogus "latch_md" in Page2 indexed addressing
--
-- Version 1.8 - 27 January 2004 John Kent
-- CWAI in decode1_state should increment the PC.
-- ABX is supposed to be an unsigned addition.
-- Added extra ALU function
-- ASR8 slightly changed in the ALU.
--
--	Version 1.9 - 20 August 2005
-- LSR8 is now handled in ASR8 and ROR8 case in the ALU,
-- rather than LSR16. There was a problem with single 
-- operand instructions using the MD register which is
-- sign extended on the first 8 bit fetch.
--
-- Version 1.10 - 13 September 2005
-- TFR & EXG instructions did not work for the Condition Code Register
-- An extra case has been added to the ALU for the alu_tfr control 
-- to assign the left ALU input (alu_left) to the condition code
-- outputs (cc_out). 
--
-- Version 1.11 - 16 September 2005
-- JSR ,X should not predecrement S before calculating the jump address.
-- The reason is that JSR [0,S] needs S to point to the top of the stack
-- to fetch a valid vector address. The solution is to have the addressing
-- mode microcode called before decrementing S and then decrementing S in
-- JSR_STATE. JSR_STATE in turn calls PUSH_RETURN_LO_STATE rather than
-- PUSH_RETURN_HI_STATE so that both the High & Low halves of the PC are
-- pushed on the stack. This adds one extra bus cycle, but resolves the
-- addressing conflict. I've also removed the pre-decement S in 
-- JSR EXTENDED as it also calls JSR_STATE.
--
-- Version 1.12 - 6th June 2006
-- 6809 Programming reference manual says V is not affected by ASR, LSR and ROR
-- This is different to the 6800. CLR should reset the V bit.
--
-- Version 1.13 - 7th July 2006
-- Disable NMI on reset until S Stack pointer has been loaded.
-- Added nmi_enable signal in sp_reg process and nmi_handler process.
--
-- Version 1.14 - 11th July 2006
-- 1. Added new state to RTI called rti_entire_state.
-- This state tests the CC register after it has been loaded
-- from the stack. Previously the current CC was tested which
-- was incorrect. The Entire Flag should be set before the
-- interrupt stacks the CC.
-- 2. On bogus Interrupts, int_cc_state went to rti_state,
-- which was an enumerated state, but not defined anywhere.
-- rti_state has been changed to rti_cc_state so that bogus interrupt
-- will perform an RTI after entering that state.
-- 3. Sync should generate an interrupt if the interrupt masks
-- are cleared. If the interrupt masks are set, then an interrupt
-- will cause the the PC to advance to the next instruction.
-- Note that I don't wait for an interrupt to be asserted for
-- three clock cycles.
-- 4. Added new ALU control state "alu_mul". "alu_mul" is used in
-- the Multiply instruction replacing "alu_add16". This is similar 
-- to "alu_add16" except it sets the Carry bit to B7 of the result
-- in ACCB, sets the Zero bit if the 16 bit result is zero, but
-- does not affect The Half carry (H), Negative (N) or Overflow (V)
-- flags. The logic was re-arranged so that it adds md or zero so 
-- that the Carry condition code is set on zero multiplicands.
-- 5. DAA (Decimal Adjust Accumulator) should set the Negative (N)
-- and Zero Flags. It will also affect the Overflow (V) flag although
-- the operation is undefined. It's anyones guess what DAA does to V.
--
-- Version 1.15 - 25th Feb 2007 - John Kent
-- line 9672 changed "if Halt <= '1' then" to "if Halt = '1' then"
-- Changed sensitivity lists.
--
-- Version 1.16 - 5th February 2008 - John Kent
-- FIRQ interrupts should take priority over IRQ Interrupts.
-- This presumably means they should be tested for before IRQ
-- when they happen concurrently.
--
-- Version 1.17 - 18th February 2008 - John Kent
-- NMI in CWAI should mask IRQ and FIRQ interrupts
--
-- Version 1.18 - 21st February 2008 - John Kent
-- Removed default register settings in each case statement
-- and placed them at the beginning of the state sequencer.
-- Modified the SYNC instruction so that the interrupt vector(iv)
-- is not set unless an unmasked FIRQ or IRQ is received.
--
-- Version 1.19 - 25th February 2008 - John Kent
-- Enumerated separate states for FIRQ/FAST and NMIIRQ/ENTIRE
-- Enumerated separate states for MASKI and MASKIF states
-- Removed code on BSR/JSR in fetch cycle
--
-- Version 1.20 - 8th October 2011 - John Kent
-- added fetch output which should go high during the fetch cycle
--
-- Version 1.21 - 8th October 2011 - John Kent
-- added Last Instruction Cycle signal
-- replaced fetch with ifetch (instruction fetch) signal
-- added ba & bs (bus available & bus status) signals
--
-- Version 1.22 - 2011-10-29 John Kent
-- The halt state isn't correct. 
-- The halt state is entered into from the fetch_state
-- It returned to the fetch state which may re-run an execute cycle
-- on the accumulator and it won't necessarily be the last instruction cycle
-- I've changed the halt state to return to the decode1_state
--
-- Version 1.23 - 2011-10-30 John Kent
-- sample halt in the change_state process if lic is high (last instruction cycle)
--
-- Version 1.24 - 2011-11-01 John Kent
-- Handle interrupts in change_state process
-- Sample interrupt inputs on last instruction cycle
-- Remove iv_ctrl and implement iv (interrupt vector) in change_state process.
-- Generate fic (first instruction cycle) from lic (last instruction cycle)
-- and use it to complete the dual operand execute cycle before servicing
-- halt or interrupts requests.
-- rename lic to lic_out on the entity declaration so that lic can be tested internally.
-- add int_firq1_state and int_nmirq1_state to allow for the dual operand execute cycle
-- integrated nmi_ctrl into change_state process
-- Reduces the microcode state stack to one entry (saved_state)
-- imm16_state jumps directly to the fetch_state
-- pull_return_lo states jumps directly to the fetch_state
-- duplicate andcc_state as cwai_state
-- rename exg1_state as exg2 state and duplicate tfr_state as exg1_state
--
-- Version 1.25 - 2011-11-27 John Kent
-- Changed the microcode for saving registers on an interrupt into a microcode subroutine.
-- Removed SWI servicing from the change state process and made SWI, SWI2 & SWI3
-- call the interrupt microcode subroutine.
-- Added additional states for nmi, and irq for interrupt servicing.
-- Added additional states for nmi/irq, firq, and swi interrupts to mask I & F flags.
--
-- Version 1.26 - 2013-03-18 John Kent
-- pre-initialized cond_true variable to true in state sequencer
-- re-arranged change_state process slightly
--
-- Version 1.27 - 2015-05-30 John Kent
-- Added test in state machine for masked IRQ and FIRQ in Sync_state.
--
-- Version 1.28 - 2015-05-30 John Kent.
-- Moved IRQ and FIRQ test from state machine to the state sequencer Sync_state.
-- 
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;

ENTITY cpu09 IS
	PORT (
		clk : IN STD_LOGIC; -- E clock input (falling edge)
		rst : IN STD_LOGIC; -- reset input (active high)
		vma : OUT STD_LOGIC; -- valid memory address (active high)
		lic_out : OUT STD_LOGIC; -- last instruction cycle (active high)
		ifetch : OUT STD_LOGIC; -- instruction fetch cycle (active high)
		opfetch : OUT STD_LOGIC; -- opcode fetch (active high)
		ba : OUT STD_LOGIC; -- bus available (high on sync wait or DMA grant)
		bs : OUT STD_LOGIC; -- bus status (high on interrupt or reset vector fetch or DMA grant)
		addr : OUT STD_LOGIC_VECTOR(15 DOWNTO 0); -- address bus output
		rw : OUT STD_LOGIC; -- read not write output
		data_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0); -- data bus output
		data_in : IN STD_LOGIC_VECTOR(7 DOWNTO 0); -- data bus input
		irq : IN STD_LOGIC; -- interrupt request input (active high)
		firq : IN STD_LOGIC; -- fast interrupt request input (active high)
		nmi : IN STD_LOGIC; -- non maskable interrupt request input (active high)
		halt : IN STD_LOGIC; -- halt input (active high) grants DMA
		hold : IN STD_LOGIC -- hold input (active high) extend bus cycle
	);
END cpu09;

ARCHITECTURE rtl OF cpu09 IS

	CONSTANT EBIT : INTEGER := 7;
	CONSTANT FBIT : INTEGER := 6;
	CONSTANT HBIT : INTEGER := 5;
	CONSTANT IBIT : INTEGER := 4;
	CONSTANT NBIT : INTEGER := 3;
	CONSTANT ZBIT : INTEGER := 2;
	CONSTANT VBIT : INTEGER := 1;
	CONSTANT CBIT : INTEGER := 0;

	--
	-- Interrupt vector modifiers
	--
	CONSTANT RST_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "111";
	CONSTANT NMI_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "110";
	CONSTANT SWI_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "101";
	CONSTANT IRQ_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "100";
	CONSTANT FIRQ_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "011";
	CONSTANT SWI2_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "010";
	CONSTANT SWI3_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "001";
	CONSTANT RESV_VEC : STD_LOGIC_VECTOR(2 DOWNTO 0) := "000";

	TYPE state_type IS (-- Start off in Reset
		reset_state,
		-- Fetch Interrupt Vectors (including reset)
		vect_lo_state, vect_hi_state, vect_idle_state,
		-- Fetch Instruction Cycle
		fetch_state,
		-- Decode Instruction Cycles
		decode1_state, decode2_state, decode3_state,
		-- Calculate Effective Address
		imm16_state,
		indexed_state, index8_state, index16_state, index16_2_state,
		pcrel8_state, pcrel16_state, pcrel16_2_state,
		indexaddr_state, indexaddr2_state,
		postincr1_state, postincr2_state,
		indirect_state, indirect2_state, indirect3_state,
		extended_state,
		-- single ops
		single_op_read_state,
		single_op_exec_state,
		single_op_write_state,
		-- Dual op states
		dual_op_read8_state, dual_op_read16_state, dual_op_read16_2_state,
		dual_op_write8_state, dual_op_write16_state,
		-- 
		sync_state, halt_state, cwai_state,
		--
		andcc_state, orcc_state,
		tfr_state,
		exg_state, exg1_state, exg2_state,
		lea_state,
		-- Multiplication
		mul_state, mulea_state, muld_state,
		mul0_state, mul1_state, mul2_state, mul3_state,
		mul4_state, mul5_state, mul6_state, mul7_state,
		--  Branches
		lbranch_state, sbranch_state,
		-- Jumps, Subroutine Calls and Returns
		jsr_state, jmp_state,
		push_return_hi_state, push_return_lo_state,
		pull_return_hi_state, pull_return_lo_state,
		-- Interrupt cycles
		int_nmi_state, int_nmi1_state,
		int_irq_state, int_irq1_state,
		int_firq_state, int_firq1_state,
		int_entire_state, int_fast_state,
		int_pcl_state, int_pch_state,
		int_upl_state, int_uph_state,
		int_iyl_state, int_iyh_state,
		int_ixl_state, int_ixh_state,
		int_dp_state,
		int_accb_state, int_acca_state,
		int_cc_state,
		int_cwai_state,
		int_nmimask_state, int_firqmask_state, int_swimask_state, int_irqmask_state,
		-- Return From Interrupt
		rti_cc_state, rti_entire_state,
		rti_acca_state, rti_accb_state,
		rti_dp_state,
		rti_ixl_state, rti_ixh_state,
		rti_iyl_state, rti_iyh_state,
		rti_upl_state, rti_uph_state,
		rti_pcl_state, rti_pch_state,
		-- Push Registers using SP
		pshs_state,
		pshs_pcl_state, pshs_pch_state,
		pshs_upl_state, pshs_uph_state,
		pshs_iyl_state, pshs_iyh_state,
		pshs_ixl_state, pshs_ixh_state,
		pshs_dp_state,
		pshs_acca_state, pshs_accb_state,
		pshs_cc_state,
		-- Pull Registers using SP
		puls_state,
		puls_cc_state,
		puls_acca_state, puls_accb_state,
		puls_dp_state,
		puls_ixl_state, puls_ixh_state,
		puls_iyl_state, puls_iyh_state,
		puls_upl_state, puls_uph_state,
		puls_pcl_state, puls_pch_state,
		-- Push Registers using UP
		pshu_state,
		pshu_pcl_state, pshu_pch_state,
		pshu_spl_state, pshu_sph_state,
		pshu_iyl_state, pshu_iyh_state,
		pshu_ixl_state, pshu_ixh_state,
		pshu_dp_state,
		pshu_acca_state, pshu_accb_state,
		pshu_cc_state,
		-- Pull Registers using UP
		pulu_state,
		pulu_cc_state,
		pulu_acca_state, pulu_accb_state,
		pulu_dp_state,
		pulu_ixl_state, pulu_ixh_state,
		pulu_iyl_state, pulu_iyh_state,
		pulu_spl_state, pulu_sph_state,
		pulu_pcl_state, pulu_pch_state);

	TYPE st_type IS (reset_st, push_st, idle_st);
	TYPE iv_type IS (latch_iv, swi3_iv, swi2_iv, firq_iv, irq_iv, swi_iv, nmi_iv, reset_iv);
	TYPE addr_type IS (idle_ad, fetch_ad, read_ad, write_ad, pushu_ad, pullu_ad, pushs_ad, pulls_ad, int_hi_ad, int_lo_ad);
	TYPE dout_type IS (cc_dout, acca_dout, accb_dout, dp_dout,
		ix_lo_dout, ix_hi_dout, iy_lo_dout, iy_hi_dout,
		up_lo_dout, up_hi_dout, sp_lo_dout, sp_hi_dout,
		pc_lo_dout, pc_hi_dout, md_lo_dout, md_hi_dout);
	TYPE op_type IS (reset_op, fetch_op, latch_op);
	TYPE pre_type IS (reset_pre, fetch_pre, latch_pre);
	TYPE cc_type IS (reset_cc, load_cc, pull_cc, latch_cc);
	TYPE acca_type IS (reset_acca, load_acca, load_hi_acca, pull_acca, latch_acca);
	TYPE accb_type IS (reset_accb, load_accb, pull_accb, latch_accb);
	TYPE dp_type IS (reset_dp, load_dp, pull_dp, latch_dp);
	TYPE ix_type IS (reset_ix, load_ix, pull_lo_ix, pull_hi_ix, latch_ix);
	TYPE iy_type IS (reset_iy, load_iy, pull_lo_iy, pull_hi_iy, latch_iy);
	TYPE sp_type IS (reset_sp, latch_sp, load_sp, pull_hi_sp, pull_lo_sp);
	TYPE up_type IS (reset_up, latch_up, load_up, pull_hi_up, pull_lo_up);
	TYPE pc_type IS (reset_pc, latch_pc, load_pc, pull_lo_pc, pull_hi_pc, incr_pc);
	TYPE md_type IS (reset_md, latch_md, load_md, fetch_first_md, fetch_next_md, shiftl_md);
	TYPE ea_type IS (reset_ea, latch_ea, load_ea, fetch_first_ea, fetch_next_ea);
	TYPE left_type IS (cc_left, acca_left, accb_left, dp_left,
		ix_left, iy_left, up_left, sp_left,
		accd_left, md_left, pc_left, ea_left);
	TYPE right_type IS (ea_right, zero_right, one_right, two_right,
		acca_right, accb_right, accd_right,
		md_right, md_sign5_right, md_sign8_right);
	TYPE alu_type IS (alu_add8, alu_sub8, alu_add16, alu_sub16, alu_adc, alu_sbc,
		alu_and, alu_ora, alu_eor,
		alu_tst, alu_inc, alu_dec, alu_clr, alu_neg, alu_com,
		alu_lsr16, alu_lsl16,
		alu_ror8, alu_rol8, alu_mul,
		alu_asr8, alu_asl8, alu_lsr8,
		alu_andcc, alu_orcc, alu_sex, alu_tfr, alu_abx,
		alu_seif, alu_sei, alu_see, alu_cle,
		alu_ld8, alu_st8, alu_ld16, alu_st16, alu_lea, alu_nop, alu_daa);

	SIGNAL op_code : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL pre_code : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL acca : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL accb : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cc : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cc_out : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL dp : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL xreg : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL yreg : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL sp : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL up : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL ea : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL pc : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL md : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL left : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL right : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL out_alu : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL iv : STD_LOGIC_VECTOR(2 DOWNTO 0);
	SIGNAL nmi_req : STD_LOGIC;
	SIGNAL nmi_ack : STD_LOGIC;
	SIGNAL nmi_enable : STD_LOGIC;
	SIGNAL fic : STD_LOGIC; -- first instruction cycle
	SIGNAL lic : STD_LOGIC; -- last instruction cycle

	SIGNAL state : state_type;
	SIGNAL next_state : state_type;
	SIGNAL return_state : state_type;
	SIGNAL saved_state : state_type;
	SIGNAL st_ctrl : st_type;
	SIGNAL iv_ctrl : iv_type;
	SIGNAL pc_ctrl : pc_type;
	SIGNAL ea_ctrl : ea_type;
	SIGNAL op_ctrl : op_type;
	SIGNAL pre_ctrl : pre_type;
	SIGNAL md_ctrl : md_type;
	SIGNAL acca_ctrl : acca_type;
	SIGNAL accb_ctrl : accb_type;
	SIGNAL ix_ctrl : ix_type;
	SIGNAL iy_ctrl : iy_type;
	SIGNAL cc_ctrl : cc_type;
	SIGNAL dp_ctrl : dp_type;
	SIGNAL sp_ctrl : sp_type;
	SIGNAL up_ctrl : up_type;
	SIGNAL left_ctrl : left_type;
	SIGNAL right_ctrl : right_type;
	SIGNAL alu_ctrl : alu_type;
	SIGNAL addr_ctrl : addr_type;
	SIGNAL dout_ctrl : dout_type;
BEGIN

	----------------------------------
	--
	-- State machine stack
	--
	----------------------------------
	--state_stack_proc: process( clk, hold, state_stack, st_ctrl, 
	--                           return_state, fetch_state  )
	state_stack_proc : PROCESS (clk, st_ctrl, return_state)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE st_ctrl IS
					WHEN reset_st =>
						saved_state <= fetch_state;
					WHEN push_st =>
						saved_state <= return_state;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- Interrupt Vector control
	--
	----------------------------------
	--
	int_vec_proc : PROCESS (clk, iv_ctrl)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE iv_ctrl IS
					WHEN reset_iv =>
						iv <= RST_VEC;
					WHEN nmi_iv =>
						iv <= NMI_VEC;
					WHEN swi_iv =>
						iv <= SWI_VEC;
					WHEN irq_iv =>
						iv <= IRQ_VEC;
					WHEN firq_iv =>
						iv <= FIRQ_VEC;
					WHEN swi2_iv =>
						iv <= SWI2_VEC;
					WHEN swi3_iv =>
						iv <= SWI3_VEC;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF; -- hold
		END IF; -- clk
	END PROCESS;

	----------------------------------
	--
	-- Program Counter Control
	--
	----------------------------------

	--pc_reg: process( clk, pc_ctrl, hold, pc, out_alu, data_in )
	pc_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE pc_ctrl IS
					WHEN reset_pc =>
						pc <= (OTHERS => '0');
					WHEN load_pc =>
						pc <= out_alu(15 DOWNTO 0);
					WHEN pull_lo_pc =>
						pc(7 DOWNTO 0) <= data_in;
					WHEN pull_hi_pc =>
						pc(15 DOWNTO 8) <= data_in;
					WHEN incr_pc =>
						pc <= pc + 1;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- Effective Address  Control
	--
	----------------------------------

	--ea_reg: process( clk, ea_ctrl, hold, ea, out_alu, data_in, dp )
	ea_reg : PROCESS (clk)
	BEGIN

		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE ea_ctrl IS
					WHEN reset_ea =>
						ea <= (OTHERS => '0');
					WHEN fetch_first_ea =>
						ea(7 DOWNTO 0) <= data_in;
						ea(15 DOWNTO 8) <= dp;
					WHEN fetch_next_ea =>
						ea(15 DOWNTO 8) <= ea(7 DOWNTO 0);
						ea(7 DOWNTO 0) <= data_in;
					WHEN load_ea =>
						ea <= out_alu(15 DOWNTO 0);
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Accumulator A
	--
	--------------------------------
	--acca_reg : process( clk, acca_ctrl, hold, out_alu, acca, data_in )
	acca_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE acca_ctrl IS
					WHEN reset_acca =>
						acca <= (OTHERS => '0');
					WHEN load_acca =>
						acca <= out_alu(7 DOWNTO 0);
					WHEN load_hi_acca =>
						acca <= out_alu(15 DOWNTO 8);
					WHEN pull_acca =>
						acca <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Accumulator B
	--
	--------------------------------
	--accb_reg : process( clk, accb_ctrl, hold, out_alu, accb, data_in )
	accb_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE accb_ctrl IS
					WHEN reset_accb =>
						accb <= (OTHERS => '0');
					WHEN load_accb =>
						accb <= out_alu(7 DOWNTO 0);
					WHEN pull_accb =>
						accb <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- X Index register
	--
	--------------------------------
	--ix_reg : process( clk, ix_ctrl, hold, out_alu, xreg, data_in )
	ix_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE ix_ctrl IS
					WHEN reset_ix =>
						xreg <= (OTHERS => '0');
					WHEN load_ix =>
						xreg <= out_alu(15 DOWNTO 0);
					WHEN pull_hi_ix =>
						xreg(15 DOWNTO 8) <= data_in;
					WHEN pull_lo_ix =>
						xreg(7 DOWNTO 0) <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Y Index register
	--
	--------------------------------
	--iy_reg : process( clk, iy_ctrl, hold, out_alu, yreg, data_in )
	iy_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE iy_ctrl IS
					WHEN reset_iy =>
						yreg <= (OTHERS => '0');
					WHEN load_iy =>
						yreg <= out_alu(15 DOWNTO 0);
					WHEN pull_hi_iy =>
						yreg(15 DOWNTO 8) <= data_in;
					WHEN pull_lo_iy =>
						yreg(7 DOWNTO 0) <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- S stack pointer
	--
	--------------------------------
	--sp_reg : process( clk, sp_ctrl, hold, sp, out_alu, data_in, nmi_enable )
	sp_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE sp_ctrl IS
					WHEN reset_sp =>
						sp <= (OTHERS => '0');
						nmi_enable <= '0';
					WHEN load_sp =>
						sp <= out_alu(15 DOWNTO 0);
						nmi_enable <= '1';
					WHEN pull_hi_sp =>
						sp(15 DOWNTO 8) <= data_in;
					WHEN pull_lo_sp =>
						sp(7 DOWNTO 0) <= data_in;
						nmi_enable <= '1';
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- U stack pointer
	--
	--------------------------------
	--up_reg : process( clk, up_ctrl, hold, up, out_alu, data_in )
	up_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE up_ctrl IS
					WHEN reset_up =>
						up <= (OTHERS => '0');
					WHEN load_up =>
						up <= out_alu(15 DOWNTO 0);
					WHEN pull_hi_up =>
						up(15 DOWNTO 8) <= data_in;
					WHEN pull_lo_up =>
						up(7 DOWNTO 0) <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Memory Data
	--
	--------------------------------
	--md_reg : process( clk, md_ctrl, hold, out_alu, data_in, md )
	md_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE md_ctrl IS
					WHEN reset_md =>
						md <= (OTHERS => '0');
					WHEN load_md =>
						md <= out_alu(15 DOWNTO 0);
					WHEN fetch_first_md => -- sign extend md for branches
						md(15 DOWNTO 8) <= data_in(7) & data_in(7) & data_in(7) & data_in(7) &
						data_in(7) & data_in(7) & data_in(7) & data_in(7);
						md(7 DOWNTO 0) <= data_in;
					WHEN fetch_next_md =>
						md(15 DOWNTO 8) <= md(7 DOWNTO 0);
						md(7 DOWNTO 0) <= data_in;
					WHEN shiftl_md =>
						md(15 DOWNTO 1) <= md(14 DOWNTO 0);
						md(0) <= '0';
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;
	----------------------------------
	--
	-- Condition Codes
	--
	----------------------------------

	--cc_reg: process( clk, cc_ctrl, hold, cc_out, cc, data_in )
	cc_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE cc_ctrl IS
					WHEN reset_cc =>
						cc <= "11010000"; -- set EBIT, FBIT & IBIT
					WHEN load_cc =>
						cc <= cc_out;
					WHEN pull_cc =>
						cc <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- Direct Page register
	--
	----------------------------------

	--dp_reg: process( clk, dp_ctrl, hold, out_alu, dp, data_in )
	dp_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE dp_ctrl IS
					WHEN reset_dp =>
						dp <= (OTHERS => '0');
					WHEN load_dp =>
						dp <= out_alu(7 DOWNTO 0);
					WHEN pull_dp =>
						dp <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;
	----------------------------------
	--
	-- op code register
	--
	----------------------------------

	--op_reg: process( clk, op_ctrl, hold, op_code, data_in )
	op_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE op_ctrl IS
					WHEN reset_op =>
						op_code <= "00010010";
					WHEN fetch_op =>
						op_code <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;
	----------------------------------
	--
	-- pre byte op code register
	--
	----------------------------------

	--pre_reg: process( clk, pre_ctrl, hold, pre_code, data_in )
	pre_reg : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '0' THEN
				CASE pre_ctrl IS
					WHEN reset_pre =>
						pre_code <= (OTHERS => '0');
					WHEN fetch_pre =>
						pre_code <= data_in;
					WHEN OTHERS =>
						NULL;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- state machine
	--
	--------------------------------

	--change_state: process( clk, rst, state, hold, next_state )
	change_state : PROCESS (clk)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF rst = '1' THEN
				fic <= '0';
				nmi_ack <= '0';
				state <= reset_state;
			ELSIF hold = '0' THEN
				fic <= lic;
				--
				-- nmi request is not cleared until nmi input goes low
				--
				IF (nmi_req = '0') AND (nmi_ack = '1') THEN
					nmi_ack <= '0';
				END IF;

				IF (nmi_req = '1') AND (nmi_ack = '0') AND (state = int_nmimask_state) THEN
					nmi_ack <= '1';
				END IF;

				IF lic = '1' THEN
					IF halt = '1' THEN
						state <= halt_state;

						-- service non maskable interrupts
					ELSIF (nmi_req = '1') AND (nmi_ack = '0') THEN
						state <= int_nmi_state;
						--
						-- FIRQ & IRQ are level sensitive
						--
					ELSIF (firq = '1') AND (cc(FBIT) = '0') THEN
						state <= int_firq_state;

					ELSIF (irq = '1') AND (cc(IBIT) = '0') THEN
						state <= int_irq_state;
						--
						-- Version 1.27 2015-05-30
						-- Exit sync_state on masked interrupt.
						--
						-- Version 1.28 2015-05-30
						-- Move this code to the state sequencer
						-- near line 5566.
						--
						-- elsif  (state = sync_state) and ((firq = '1') or (irq = '1'))then
						--   state <= fetch_state;
						--
					ELSE
						state <= next_state;
					END IF; -- halt, nmi, firq, irq
				ELSE
					state <= next_state;
				END IF; -- lic
			END IF; -- reset/hold
		END IF; -- clk
	END PROCESS;

	------------------------------------
	--
	-- Detect Edge of NMI interrupt
	--
	------------------------------------

	--nmi_handler : process( clk, rst, nmi, nmi_ack, nmi_req, nmi_enable )
	nmi_handler : PROCESS (rst, clk)
	BEGIN
		IF rst = '1' THEN
			nmi_req <= '0';
		ELSIF clk'event AND clk = '0' THEN
			IF (nmi = '1') AND (nmi_ack = '0') AND (nmi_enable = '1') THEN
				nmi_req <= '1';
			ELSE
				IF (nmi = '0') AND (nmi_ack = '1') THEN
					nmi_req <= '0';
				END IF;
			END IF;
		END IF;
	END PROCESS;
	----------------------------------
	--
	-- Address output multiplexer
	--
	----------------------------------

	addr_mux : PROCESS (addr_ctrl, pc, ea, up, sp, iv)
	BEGIN
		ifetch <= '0';
		vma <= '1';
		CASE addr_ctrl IS
			WHEN fetch_ad =>
				addr <= pc;
				rw <= '1';
				ifetch <= '1';
			WHEN read_ad =>
				addr <= ea;
				rw <= '1';
			WHEN write_ad =>
				addr <= ea;
				rw <= '0';
			WHEN pushs_ad =>
				addr <= sp;
				rw <= '0';
			WHEN pulls_ad =>
				addr <= sp;
				rw <= '1';
			WHEN pushu_ad =>
				addr <= up;
				rw <= '0';
			WHEN pullu_ad =>
				addr <= up;
				rw <= '1';
			WHEN int_hi_ad =>
				addr <= "111111111111" & iv & "0";
				rw <= '1';
			WHEN int_lo_ad =>
				addr <= "111111111111" & iv & "1";
				rw <= '1';
			WHEN OTHERS =>
				addr <= "1111111111111111";
				rw <= '1';
				vma <= '0';
		END CASE;
	END PROCESS;

	--------------------------------
	--
	-- Data Bus output
	--
	--------------------------------
	dout_mux : PROCESS (dout_ctrl, md, acca, accb, dp, xreg, yreg, sp, up, pc, cc)
	BEGIN
		CASE dout_ctrl IS
			WHEN cc_dout => -- condition code register
				data_out <= cc;
			WHEN acca_dout => -- accumulator a
				data_out <= acca;
			WHEN accb_dout => -- accumulator b
				data_out <= accb;
			WHEN dp_dout => -- direct page register
				data_out <= dp;
			WHEN ix_lo_dout => -- X index reg
				data_out <= xreg(7 DOWNTO 0);
			WHEN ix_hi_dout => -- X index reg
				data_out <= xreg(15 DOWNTO 8);
			WHEN iy_lo_dout => -- Y index reg
				data_out <= yreg(7 DOWNTO 0);
			WHEN iy_hi_dout => -- Y index reg
				data_out <= yreg(15 DOWNTO 8);
			WHEN up_lo_dout => -- U stack pointer
				data_out <= up(7 DOWNTO 0);
			WHEN up_hi_dout => -- U stack pointer
				data_out <= up(15 DOWNTO 8);
			WHEN sp_lo_dout => -- S stack pointer
				data_out <= sp(7 DOWNTO 0);
			WHEN sp_hi_dout => -- S stack pointer
				data_out <= sp(15 DOWNTO 8);
			WHEN md_lo_dout => -- alu output
				data_out <= md(7 DOWNTO 0);
			WHEN md_hi_dout => -- alu output
				data_out <= md(15 DOWNTO 8);
			WHEN pc_lo_dout => -- low order pc
				data_out <= pc(7 DOWNTO 0);
			WHEN pc_hi_dout => -- high order pc
				data_out <= pc(15 DOWNTO 8);
		END CASE;
	END PROCESS;

	----------------------------------
	--
	-- Left Mux
	--
	----------------------------------

	left_mux : PROCESS (left_ctrl, acca, accb, cc, dp, xreg, yreg, up, sp, pc, ea, md)
	BEGIN
		CASE left_ctrl IS
			WHEN cc_left =>
				left(15 DOWNTO 8) <= "00000000";
				left(7 DOWNTO 0) <= cc;
			WHEN acca_left =>
				left(15 DOWNTO 8) <= "00000000";
				left(7 DOWNTO 0) <= acca;
			WHEN accb_left =>
				left(15 DOWNTO 8) <= "00000000";
				left(7 DOWNTO 0) <= accb;
			WHEN dp_left =>
				left(15 DOWNTO 8) <= "00000000";
				left(7 DOWNTO 0) <= dp;
			WHEN accd_left =>
				left(15 DOWNTO 8) <= acca;
				left(7 DOWNTO 0) <= accb;
			WHEN md_left =>
				left <= md;
			WHEN ix_left =>
				left <= xreg;
			WHEN iy_left =>
				left <= yreg;
			WHEN sp_left =>
				left <= sp;
			WHEN up_left =>
				left <= up;
			WHEN pc_left =>
				left <= pc;
			WHEN OTHERS =>
				--	 when ea_left =>
				left <= ea;
		END CASE;
	END PROCESS;

	----------------------------------
	--
	-- Right Mux
	--
	----------------------------------

	right_mux : PROCESS (right_ctrl, md, acca, accb, ea)
	BEGIN
		CASE right_ctrl IS
			WHEN ea_right =>
				right <= ea;
			WHEN zero_right =>
				right <= "0000000000000000";
			WHEN one_right =>
				right <= "0000000000000001";
			WHEN two_right =>
				right <= "0000000000000010";
			WHEN acca_right =>
				IF acca(7) = '0' THEN
					right <= "00000000" & acca(7 DOWNTO 0);
				ELSE
					right <= "11111111" & acca(7 DOWNTO 0);
				END IF;
			WHEN accb_right =>
				IF accb(7) = '0' THEN
					right <= "00000000" & accb(7 DOWNTO 0);
				ELSE
					right <= "11111111" & accb(7 DOWNTO 0);
				END IF;
			WHEN accd_right =>
				right <= acca & accb;
			WHEN md_sign5_right =>
				IF md(4) = '0' THEN
					right <= "00000000000" & md(4 DOWNTO 0);
				ELSE
					right <= "11111111111" & md(4 DOWNTO 0);
				END IF;
			WHEN md_sign8_right =>
				IF md(7) = '0' THEN
					right <= "00000000" & md(7 DOWNTO 0);
				ELSE
					right <= "11111111" & md(7 DOWNTO 0);
				END IF;
			WHEN OTHERS =>
				--	 when md_right =>
				right <= md;
		END CASE;
	END PROCESS;

	----------------------------------
	--
	-- Arithmetic Logic Unit
	--
	----------------------------------

	alu : PROCESS (alu_ctrl, cc, left, right, out_alu, cc_out)
		VARIABLE valid_lo, valid_hi : BOOLEAN;
		VARIABLE carry_in : STD_LOGIC;
		VARIABLE daa_reg : STD_LOGIC_VECTOR(7 DOWNTO 0);
	BEGIN

		CASE alu_ctrl IS
			WHEN alu_adc | alu_sbc |
				alu_rol8 | alu_ror8 =>
				carry_in := cc(CBIT);
			WHEN alu_asr8 =>
				carry_in := left(7);
			WHEN OTHERS =>
				carry_in := '0';
		END CASE;

		valid_lo := left(3 DOWNTO 0) <= 9;
		valid_hi := left(7 DOWNTO 4) <= 9;

		--
		-- CBIT HBIT VHI VLO DAA
		--    0    0   0   0 66 (!VHI : hi_nybble>8)
		--    0    0   0   1 60
		--    0    0   1   1 00
		--    0    0   1   0 06 ( VHI : hi_nybble<=8)
		--
		--    0    1   1   0 06
		--    0    1   1   1 06
		--    0    1   0   1 66
		--    0    1   0   0 66
		--
		--    1    1   0   0 66
		--    1    1   0   1 66
		--    1    1   1   1 66
		--    1    1   1   0 66
		--
		--    1    0   1   0 66
		--    1    0   1   1 60
		--    1    0   0   1 60
		--    1    0   0   0 66
		--
		-- 66 = (!VHI & !VLO) + (CBIT & HBIT) + (HBIT & !VHI) + (CBIT & !VLO) 
		--    = (CBIT & (HBIT + !VLO)) + (!VHI & (HBIT + !VLO))
		--    = (!VLO & (CBIT + !VHI)) + (HBIT & (CBIT + !VHI))
		-- 60 = (CBIT & !HBIT & VLO) + (!HBIT & !VHI & VLO) 
		--    = (!HBIT & VLO & (CBIT + !VHI))
		-- 06 = (!CBIT & VHI & (!VLO + VHI)
		-- 00 = (!CBIT & !HBIT & VHI & VLO)
		--
		IF (cc(CBIT) = '0') THEN
			-- CBIT=0
			IF (cc(HBIT) = '0') THEN
				-- HBIT=0
				IF valid_lo THEN
					-- lo <= 9 (no overflow in low nybble)
					IF valid_hi THEN
						-- hi <= 9 (no overflow in either low or high nybble)
						daa_reg := "00000000";
					ELSE
						-- hi > 9 (overflow in high nybble only)
						daa_reg := "01100000";
					END IF;
				ELSE
					-- lo > 9 (overflow in low nybble)
					--
					-- since there is already an overflow in the low nybble
					-- you need to make room in the high nybble for the low nybble carry
					-- so compare the high nybble with 8 rather than 9
					-- if the high nybble is 9 there will be an overflow on the high nybble
					-- after the decimal adjust which means it will roll over to an invalid BCD digit
					--
					IF (left(7 DOWNTO 4) <= 8) THEN
						-- hi <= 8 (overflow in low nybble only)
						daa_reg := "00000110";
					ELSE
						-- hi > 8 (overflow in low and high nybble)
						daa_reg := "01100110";
					END IF;
				END IF;
			ELSE
				-- HBIT=1 (overflow in low nybble)
				IF valid_hi THEN
					-- hi <= 9 (overflow in low nybble only)
					daa_reg := "00000110";
				ELSE
					-- hi > 9 (overflow in low and high nybble)
					daa_reg := "01100110";
				END IF;
			END IF;
		ELSE
			-- CBIT=1 (carry => overflow in high nybble)
			IF (cc(HBIT) = '0') THEN
				-- HBIT=0 (half carry clear => may or may not be an overflow in the low nybble)
				IF valid_lo THEN
					-- lo <=9  (overflow in high nybble only)
					daa_reg := "01100000";
				ELSE
					-- lo >9  (overflow in low and high nybble)
					daa_reg := "01100110";
				END IF;
			ELSE
				-- HBIT=1 (overflow in low and high nybble)
				daa_reg := "01100110";
			END IF;
		END IF;

		CASE alu_ctrl IS
			WHEN alu_add8 | alu_inc |
				alu_add16 | alu_adc | alu_mul =>
				out_alu <= left + right + ("000000000000000" & carry_in);
			WHEN alu_sub8 | alu_dec |
				alu_sub16 | alu_sbc =>
				out_alu <= left - right - ("000000000000000" & carry_in);
			WHEN alu_abx =>
				out_alu <= left + ("00000000" & right(7 DOWNTO 0));
			WHEN alu_and =>
				out_alu <= left AND right; -- and/bit
			WHEN alu_ora =>
				out_alu <= left OR right; -- or
			WHEN alu_eor =>
				out_alu <= left XOR right; -- eor/xor
			WHEN alu_lsl16 | alu_asl8 | alu_rol8 =>
				out_alu <= left(14 DOWNTO 0) & carry_in; -- rol8/asl8/lsl16
			WHEN alu_lsr16 =>
				out_alu <= carry_in & left(15 DOWNTO 1); -- lsr16
			WHEN alu_lsr8 | alu_asr8 | alu_ror8 =>
				out_alu <= "00000000" & carry_in & left(7 DOWNTO 1); -- ror8/asr8/lsr8
			WHEN alu_neg =>
				out_alu <= right - left; -- neg (right=0)
			WHEN alu_com =>
				out_alu <= NOT left;
			WHEN alu_clr | alu_ld8 | alu_ld16 | alu_lea =>
				out_alu <= right; -- clr, ld
			WHEN alu_st8 | alu_st16 | alu_andcc | alu_orcc | alu_tfr =>
				out_alu <= left;
			WHEN alu_daa =>
				out_alu <= left + ("00000000" & daa_reg);
			WHEN alu_sex =>
				IF left(7) = '0' THEN
					out_alu <= "00000000" & left(7 DOWNTO 0);
				ELSE
					out_alu <= "11111111" & left(7 DOWNTO 0);
				END IF;
			WHEN OTHERS =>
				out_alu <= left; -- nop
		END CASE;

		--
		-- carry bit
		--
		CASE alu_ctrl IS
			WHEN alu_add8 | alu_adc =>
				cc_out(CBIT) <= (left(7) AND right(7)) OR
				(left(7) AND NOT out_alu(7)) OR
				(right(7) AND NOT out_alu(7));
			WHEN alu_sub8 | alu_sbc =>
				cc_out(CBIT) <= ((NOT left(7)) AND right(7)) OR
				((NOT left(7)) AND out_alu(7)) OR
				(right(7) AND out_alu(7));
			WHEN alu_add16 =>
				cc_out(CBIT) <= (left(15) AND right(15)) OR
				(left(15) AND NOT out_alu(15)) OR
				(right(15) AND NOT out_alu(15));
			WHEN alu_sub16 =>
				cc_out(CBIT) <= ((NOT left(15)) AND right(15)) OR
				((NOT left(15)) AND out_alu(15)) OR
				(right(15) AND out_alu(15));
			WHEN alu_ror8 | alu_lsr16 | alu_lsr8 | alu_asr8 =>
				cc_out(CBIT) <= left(0);
			WHEN alu_rol8 | alu_asl8 =>
				cc_out(CBIT) <= left(7);
			WHEN alu_lsl16 =>
				cc_out(CBIT) <= left(15);
			WHEN alu_com =>
				cc_out(CBIT) <= '1';
			WHEN alu_neg | alu_clr =>
				cc_out(CBIT) <= out_alu(7) OR out_alu(6) OR out_alu(5) OR out_alu(4) OR
				out_alu(3) OR out_alu(2) OR out_alu(1) OR out_alu(0);
			WHEN alu_mul =>
				cc_out(CBIT) <= out_alu(7);
			WHEN alu_daa =>
				IF (daa_reg(7 DOWNTO 4) = "0110") THEN
					cc_out(CBIT) <= '1';
				ELSE
					cc_out(CBIT) <= '0';
				END IF;
			WHEN alu_andcc =>
				cc_out(CBIT) <= left(CBIT) AND cc(CBIT);
			WHEN alu_orcc =>
				cc_out(CBIT) <= left(CBIT) OR cc(CBIT);
			WHEN alu_tfr =>
				cc_out(CBIT) <= left(CBIT);
			WHEN OTHERS =>
				cc_out(CBIT) <= cc(CBIT);
		END CASE;
		--
		-- Zero flag
		--
		CASE alu_ctrl IS
			WHEN alu_add8 | alu_sub8 |
				alu_adc | alu_sbc |
				alu_and | alu_ora | alu_eor |
				alu_inc | alu_dec |
				alu_neg | alu_com | alu_clr |
				alu_rol8 | alu_ror8 | alu_asr8 | alu_asl8 | alu_lsr8 |
				alu_ld8 | alu_st8 | alu_sex | alu_daa =>
				cc_out(ZBIT) <= NOT(out_alu(7) OR out_alu(6) OR out_alu(5) OR out_alu(4) OR
				out_alu(3) OR out_alu(2) OR out_alu(1) OR out_alu(0));
			WHEN alu_add16 | alu_sub16 | alu_mul |
				alu_lsl16 | alu_lsr16 |
				alu_ld16 | alu_st16 | alu_lea =>
				cc_out(ZBIT) <= NOT(out_alu(15) OR out_alu(14) OR out_alu(13) OR out_alu(12) OR
				out_alu(11) OR out_alu(10) OR out_alu(9) OR out_alu(8) OR
				out_alu(7) OR out_alu(6) OR out_alu(5) OR out_alu(4) OR
				out_alu(3) OR out_alu(2) OR out_alu(1) OR out_alu(0));
			WHEN alu_andcc =>
				cc_out(ZBIT) <= left(ZBIT) AND cc(ZBIT);
			WHEN alu_orcc =>
				cc_out(ZBIT) <= left(ZBIT) OR cc(ZBIT);
			WHEN alu_tfr =>
				cc_out(ZBIT) <= left(ZBIT);
			WHEN OTHERS =>
				cc_out(ZBIT) <= cc(ZBIT);
		END CASE;

		--
		-- negative flag
		--
		CASE alu_ctrl IS
			WHEN alu_add8 | alu_sub8 |
				alu_adc | alu_sbc |
				alu_and | alu_ora | alu_eor |
				alu_rol8 | alu_ror8 | alu_asr8 | alu_asl8 | alu_lsr8 |
				alu_inc | alu_dec | alu_neg | alu_com | alu_clr |
				alu_ld8 | alu_st8 | alu_sex | alu_daa =>
				cc_out(NBIT) <= out_alu(7);
			WHEN alu_add16 | alu_sub16 |
				alu_lsl16 | alu_lsr16 |
				alu_ld16 | alu_st16 =>
				cc_out(NBIT) <= out_alu(15);
			WHEN alu_andcc =>
				cc_out(NBIT) <= left(NBIT) AND cc(NBIT);
			WHEN alu_orcc =>
				cc_out(NBIT) <= left(NBIT) OR cc(NBIT);
			WHEN alu_tfr =>
				cc_out(NBIT) <= left(NBIT);
			WHEN OTHERS =>
				cc_out(NBIT) <= cc(NBIT);
		END CASE;

		--
		-- Interrupt mask flag
		--
		CASE alu_ctrl IS
			WHEN alu_andcc =>
				cc_out(IBIT) <= left(IBIT) AND cc(IBIT);
			WHEN alu_orcc =>
				cc_out(IBIT) <= left(IBIT) OR cc(IBIT);
			WHEN alu_tfr =>
				cc_out(IBIT) <= left(IBIT);
			WHEN alu_seif | alu_sei =>
				cc_out(IBIT) <= '1';
			WHEN OTHERS =>
				cc_out(IBIT) <= cc(IBIT); -- interrupt mask
		END CASE;

		--
		-- Half Carry flag
		--
		CASE alu_ctrl IS
			WHEN alu_add8 | alu_adc =>
				cc_out(HBIT) <= (left(3) AND right(3)) OR
				(right(3) AND NOT out_alu(3)) OR
				(left(3) AND NOT out_alu(3));
			WHEN alu_andcc =>
				cc_out(HBIT) <= left(HBIT) AND cc(HBIT);
			WHEN alu_orcc =>
				cc_out(HBIT) <= left(HBIT) OR cc(HBIT);
			WHEN alu_tfr =>
				cc_out(HBIT) <= left(HBIT);
			WHEN OTHERS =>
				cc_out(HBIT) <= cc(HBIT);
		END CASE;

		--
		-- Overflow flag
		--
		CASE alu_ctrl IS
			WHEN alu_add8 | alu_adc =>
				cc_out(VBIT) <= (left(7) AND right(7) AND (NOT out_alu(7))) OR
				((NOT left(7)) AND (NOT right(7)) AND out_alu(7));
			WHEN alu_sub8 | alu_sbc =>
				cc_out(VBIT) <= (left(7) AND (NOT right(7)) AND (NOT out_alu(7))) OR
				((NOT left(7)) AND right(7) AND out_alu(7));
			WHEN alu_add16 =>
				cc_out(VBIT) <= (left(15) AND right(15) AND (NOT out_alu(15))) OR
				((NOT left(15)) AND (NOT right(15)) AND out_alu(15));
			WHEN alu_sub16 =>
				cc_out(VBIT) <= (left(15) AND (NOT right(15)) AND (NOT out_alu(15))) OR
				((NOT left(15)) AND right(15) AND out_alu(15));
			WHEN alu_inc =>
				cc_out(VBIT) <= ((NOT left(7)) AND left(6) AND left(5) AND left(4) AND
				left(3) AND left(2) AND left(1) AND left(0));
			WHEN alu_dec | alu_neg =>
				cc_out(VBIT) <= (left(7) AND (NOT left(6)) AND (NOT left(5)) AND (NOT left(4)) AND
				(NOT left(3)) AND (NOT left(2)) AND (NOT left(1)) AND (NOT left(0)));
				-- 6809 Programming reference manual says
				-- V not affected by ASR, LSR and ROR
				-- This is different to the 6800
				-- John Kent 6th June 2006
				--	 when alu_asr8 =>
				--	   cc_out(VBIT) <= left(0) xor left(7);
				--	 when alu_lsr8 | alu_lsr16 =>
				--	   cc_out(VBIT) <= left(0);
				--	 when alu_ror8 =>
				--      cc_out(VBIT) <= left(0) xor cc(CBIT);
			WHEN alu_lsl16 =>
				cc_out(VBIT) <= left(15) XOR left(14);
			WHEN alu_rol8 | alu_asl8 =>
				cc_out(VBIT) <= left(7) XOR left(6);
				--
				-- 11th July 2006 - John Kent
				-- What DAA does with V is anyones guess
				-- It is undefined in the 6809 programming manual
				--
			WHEN alu_daa =>
				cc_out(VBIT) <= left(7) XOR out_alu(7) XOR cc(CBIT);
				-- CLR resets V Bit
				-- John Kent 6th June 2006
			WHEN alu_and | alu_ora | alu_eor | alu_com | alu_clr |
				alu_st8 | alu_st16 | alu_ld8 | alu_ld16 | alu_sex =>
				cc_out(VBIT) <= '0';
			WHEN alu_andcc =>
				cc_out(VBIT) <= left(VBIT) AND cc(VBIT);
			WHEN alu_orcc =>
				cc_out(VBIT) <= left(VBIT) OR cc(VBIT);
			WHEN alu_tfr =>
				cc_out(VBIT) <= left(VBIT);
			WHEN OTHERS =>
				cc_out(VBIT) <= cc(VBIT);
		END CASE;

		CASE alu_ctrl IS
			WHEN alu_andcc =>
				cc_out(FBIT) <= left(FBIT) AND cc(FBIT);
			WHEN alu_orcc =>
				cc_out(FBIT) <= left(FBIT) OR cc(FBIT);
			WHEN alu_tfr =>
				cc_out(FBIT) <= left(FBIT);
			WHEN alu_seif =>
				cc_out(FBIT) <= '1';
			WHEN OTHERS =>
				cc_out(FBIT) <= cc(FBIT);
		END CASE;

		CASE alu_ctrl IS
			WHEN alu_andcc =>
				cc_out(EBIT) <= left(EBIT) AND cc(EBIT);
			WHEN alu_orcc =>
				cc_out(EBIT) <= left(EBIT) OR cc(EBIT);
			WHEN alu_tfr =>
				cc_out(EBIT) <= left(EBIT);
			WHEN alu_see =>
				cc_out(EBIT) <= '1';
			WHEN alu_cle =>
				cc_out(EBIT) <= '0';
			WHEN OTHERS =>
				cc_out(EBIT) <= cc(EBIT);
		END CASE;
	END PROCESS;

	------------------------------------
	--
	-- state sequencer
	--
	------------------------------------
	PROCESS (state, saved_state,
		op_code, pre_code,
		cc, ea, md, iv, fic, halt,
		nmi_req, firq, irq, lic)
		VARIABLE cond_true : BOOLEAN; -- variable used to evaluate coditional branches
	BEGIN
		cond_true := (1 = 1);
		ba <= '0';
		bs <= '0';
		lic <= '0';
		opfetch <= '0';
		iv_ctrl <= latch_iv;
		-- Registers preserved
		cc_ctrl <= latch_cc;
		acca_ctrl <= latch_acca;
		accb_ctrl <= latch_accb;
		dp_ctrl <= latch_dp;
		ix_ctrl <= latch_ix;
		iy_ctrl <= latch_iy;
		up_ctrl <= latch_up;
		sp_ctrl <= latch_sp;
		pc_ctrl <= latch_pc;
		md_ctrl <= latch_md;
		ea_ctrl <= latch_ea;
		op_ctrl <= latch_op;
		pre_ctrl <= latch_pre;
		-- ALU Idle
		left_ctrl <= pc_left;
		right_ctrl <= zero_right;
		alu_ctrl <= alu_nop;
		-- Bus idle
		addr_ctrl <= idle_ad;
		dout_ctrl <= cc_dout;
		-- Next State Fetch
		st_ctrl <= idle_st;
		return_state <= fetch_state;
		next_state <= fetch_state;

		CASE state IS
			WHEN reset_state => --  released from reset
				-- reset the registers
				iv_ctrl <= reset_iv;
				op_ctrl <= reset_op;
				pre_ctrl <= reset_pre;
				cc_ctrl <= reset_cc;
				acca_ctrl <= reset_acca;
				accb_ctrl <= reset_accb;
				dp_ctrl <= reset_dp;
				ix_ctrl <= reset_ix;
				iy_ctrl <= reset_iy;
				up_ctrl <= reset_up;
				sp_ctrl <= reset_sp;
				pc_ctrl <= reset_pc;
				ea_ctrl <= reset_ea;
				md_ctrl <= reset_md;
				st_ctrl <= reset_st;
				next_state <= vect_hi_state;

				--
				-- Jump via interrupt vector
				-- iv holds interrupt type
				-- fetch PC hi from vector location
				--
			WHEN vect_hi_state =>
				-- fetch pc low interrupt vector
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= int_hi_ad;
				bs <= '1';
				next_state <= vect_lo_state;

				--
				-- jump via interrupt vector
				-- iv holds vector type
				-- fetch PC lo from vector location
				--
			WHEN vect_lo_state =>
				-- fetch the vector low byte
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= int_lo_ad;
				bs <= '1';
				next_state <= fetch_state;

			WHEN vect_idle_state =>
				--
				-- Last Instruction Cycle for SWI, SWI2 & SWI3
				--
				IF op_code = "00111111" THEN
					lic <= '1';
				END IF;
				next_state <= fetch_state;

				--
				-- Here to fetch an instruction
				-- PC points to opcode
				--
			WHEN fetch_state =>
				-- fetch the op code
				opfetch <= '1';
				op_ctrl <= fetch_op;
				pre_ctrl <= fetch_pre;
				ea_ctrl <= reset_ea;
				-- Fetch op code
				addr_ctrl <= fetch_ad;
				-- Advance the PC to fetch next instruction byte
				pc_ctrl <= incr_pc;
				next_state <= decode1_state;

				--
				-- Here to decode instruction
				-- and fetch next byte of intruction
				-- whether it be necessary or not
				--
			WHEN decode1_state =>
				-- fetch first byte of address or immediate data
				ea_ctrl <= fetch_first_ea;
				md_ctrl <= fetch_first_md;
				addr_ctrl <= fetch_ad;
				CASE op_code(7 DOWNTO 4) IS
						--
						-- direct single op (2 bytes)
						-- 6809 => 6 cycles
						-- cpu09 => 5 cycles
						-- 1 op=(pc) / pc=pc+1
						-- 2 ea_hi=dp / ea_lo=(pc) / pc=pc+1
						-- 3 md_lo=(ea) / pc=pc
						-- 4 alu_left=md / md=alu_out / pc=pc
						-- 5 (ea)=md_lo / pc=pc
						--
						-- Exception is JMP
						-- 6809 => 3 cycles
						-- cpu09 => 3 cycles
						-- 1 op=(pc) / pc=pc+1
						-- 2 ea_hi=dp / ea_lo=(pc) / pc=pc+1
						-- 3 pc=ea
						--
					WHEN "0000" =>
						-- advance the PC
						pc_ctrl <= incr_pc;

						CASE op_code(3 DOWNTO 0) IS
							WHEN "1110" => -- jmp
								next_state <= jmp_state;

							WHEN "1111" => -- clr
								next_state <= single_op_exec_state;

							WHEN OTHERS =>
								next_state <= single_op_read_state;

						END CASE;

						-- acca / accb inherent instructions
					WHEN "0001" =>
						CASE op_code(3 DOWNTO 0) IS
								--
								-- Page2 pre byte
								-- pre=(pc) / pc=pc+1
								-- op=(pc) / pc=pc+1
								--
							WHEN "0000" => -- page2
								opfetch <= '1';
								op_ctrl <= fetch_op;
								-- advance pc
								pc_ctrl <= incr_pc;
								next_state <= decode2_state;

								--
								-- Page3 pre byte
								-- pre=(pc) / pc=pc+1
								-- op=(pc) / pc=pc+1
								--
							WHEN "0001" => -- page3
								opfetch <= '1';
								op_ctrl <= fetch_op;
								-- advance pc
								pc_ctrl <= incr_pc;
								next_state <= decode3_state;

								--
								-- nop - No operation ( 1 byte )
								-- 6809 => 2 cycles
								-- cpu09 => 2 cycles
								-- 1 op=(pc) / pc=pc+1
								-- 2 decode
								-- 
							WHEN "0010" => -- nop
								lic <= '1';
								next_state <= fetch_state;

								--
								-- sync - halt execution until an interrupt is received
								-- interrupt may be NMI, IRQ or FIRQ
								-- program execution continues if the 
								-- interrupt is asserted for 3 clock cycles
								-- note that registers are not pushed onto the stack
								-- CPU09 => Interrupts need only be asserted for one clock cycle
								--
							WHEN "0011" => -- sync
								next_state <= sync_state;

								--
								-- lbra -- long branch (3 bytes)
								-- 6809 => 5 cycles
								-- cpu09 => 4 cycles
								-- 1 op=(pc) / pc=pc+1
								-- 2 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1
								-- 3 md_hi=md_lo / md_lo=(pc) / pc=pc+1
								-- 4 pc=pc+md
								--
							WHEN "0110" =>
								-- increment the pc
								pc_ctrl <= incr_pc;
								next_state <= lbranch_state;

								--
								-- lbsr - long branch to subroutine (3 bytes)
								-- 6809 => 9 cycles
								-- cpu09 => 6 cycles
								-- 1 op=(pc) /pc=pc+1
								-- 2 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1 / sp=sp-1
								-- 3 md_hi=md_lo / md_lo=(pc) / pc=pc+1
								-- 4 (sp)= pc_lo / sp=sp-1 / pc=pc
								-- 5 (sp)=pc_hi / pc=pc
								-- 6 pc=pc+md
								--
							WHEN "0111" =>
								-- pre decrement sp
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								-- increment the pc
								pc_ctrl <= incr_pc;
								next_state <= lbranch_state;

								--
								-- Decimal Adjust Accumulator
								--
							WHEN "1001" => -- daa
								left_ctrl <= acca_left;
								right_ctrl <= accb_right;
								alu_ctrl <= alu_daa;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								lic <= '1';
								next_state <= fetch_state;

								--
								-- OR Condition Codes
								--
							WHEN "1010" => -- orcc
								-- increment the pc
								pc_ctrl <= incr_pc;
								next_state <= orcc_state;

								--
								-- AND Condition Codes
								--
							WHEN "1100" => -- andcc
								-- increment the pc
								pc_ctrl <= incr_pc;
								next_state <= andcc_state;

								--
								-- Sign Extend
								--
							WHEN "1101" => -- sex
								left_ctrl <= accb_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_sex;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_hi_acca;
								lic <= '1';
								next_state <= fetch_state;

								--
								-- Exchange Registers
								--
							WHEN "1110" => -- exg
								-- increment the pc
								pc_ctrl <= incr_pc;
								next_state <= exg_state;

								--
								-- Transfer Registers
								--
							WHEN "1111" => -- tfr
								-- increment the pc
								pc_ctrl <= incr_pc;
								next_state <= tfr_state;

							WHEN OTHERS =>
								-- increment the pc
								pc_ctrl <= incr_pc;
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

						--
						-- Short branch conditional
						-- 6809 => always 3 cycles
						-- cpu09 => always = 3 cycles
						-- 1 op=(pc) / pc=pc+1
						-- 2 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1 / test cc
						-- 3 if cc tru pc=pc+md else pc=pc
						--
					WHEN "0010" => -- branch conditional
						-- increment the pc
						pc_ctrl <= incr_pc;
						next_state <= sbranch_state;

						--
						-- Single byte stack operators
						-- Do not advance PC
						--
					WHEN "0011" =>
						--
						-- lea - load effective address (2+ bytes)
						-- 6809 => 4 cycles + addressing mode
						-- cpu09 => 4 cycles + addressing mode
						-- 1 op=(pc) / pc=pc+1
						-- 2 md_lo=(pc) / pc=pc+1
						-- 3 calculate ea
						-- 4 ix/iy/sp/up = ea
						--
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" | -- leax
								"0001" | -- leay
								"0010" | -- leas
								"0011" => -- leau
								-- advance PC
								pc_ctrl <= incr_pc;
								st_ctrl <= push_st;
								return_state <= lea_state;
								next_state <= indexed_state;

								--
								-- pshs - push registers onto sp stack
								-- 6809 => 5 cycles + registers
								-- cpu09 => 3 cycles + registers
								--  1 op=(pc) / pc=pc+1
								--  2 ea_lo=(pc) / pc=pc+1 
								--  3 if ea(7 downto 0) != "00000000" then sp=sp-1
								--  4 if ea(7) = 1 (sp)=pcl, sp=sp-1
								--  5 if ea(7) = 1 (sp)=pch
								--    if ea(6 downto 0) != "0000000" then sp=sp-1
								--  6 if ea(6) = 1 (sp)=upl, sp=sp-1
								--  7 if ea(6) = 1 (sp)=uph
								--    if ea(5 downto 0) != "000000" then sp=sp-1
								--  8 if ea(5) = 1 (sp)=iyl, sp=sp-1
								--  9 if ea(5) = 1 (sp)=iyh
								--    if ea(4 downto 0) != "00000" then sp=sp-1
								-- 10 if ea(4) = 1 (sp)=ixl, sp=sp-1
								-- 11 if ea(4) = 1 (sp)=ixh
								--    if ea(3 downto 0) != "0000" then sp=sp-1
								-- 12 if ea(3) = 1 (sp)=dp
								--    if ea(2 downto 0) != "000" then sp=sp-1
								-- 13 if ea(2) = 1 (sp)=accb
								--    if ea(1 downto 0) != "00" then sp=sp-1
								-- 14 if ea(1) = 1 (sp)=acca
								--    if ea(0 downto 0) != "0" then sp=sp-1
								-- 15 if ea(0) = 1 (sp)=cc
								--
							WHEN "0100" => -- pshs
								-- advance PC
								pc_ctrl <= incr_pc;
								next_state <= pshs_state;

								--
								-- puls - pull registers of sp stack
								-- 6809 => 5 cycles + registers
								-- cpu09 => 3 cycles + registers
								--
							WHEN "0101" => -- puls
								-- advance PC
								pc_ctrl <= incr_pc;
								next_state <= puls_state;

								--
								-- pshu - push registers onto up stack
								-- 6809 => 5 cycles + registers
								-- cpu09 => 3 cycles + registers
								--
							WHEN "0110" => -- pshu
								-- advance PC
								pc_ctrl <= incr_pc;
								next_state <= pshu_state;

								--
								-- pulu - pull registers of up stack
								-- 6809 => 5 cycles + registers
								-- cpu09 => 3 cycles + registers
								--
							WHEN "0111" => -- pulu
								-- advance PC
								pc_ctrl <= incr_pc;
								next_state <= pulu_state;

								--
								-- rts - return from subroutine
								-- 6809 => 5 cycles
								-- cpu09 => 4 cycles 
								-- 1 op=(pc) / pc=pc+1
								-- 2 decode op
								-- 3 pc_hi = (sp) / sp=sp+1
								-- 4 pc_lo = (sp) / sp=sp+1
								--
							WHEN "1001" =>
								next_state <= pull_return_hi_state;

								--
								-- ADD accb to index register
								-- *** Note: this is an unsigned addition.
								--           does not affect any condition codes
								-- 6809 => 3 cycles
								-- cpu09 => 2 cycles
								-- 1 op=(pc) / pc=pc+1
								-- 2 alu_left=ix / alu_right=accb / ix=alu_out / pc=pc
								--
							WHEN "1010" => -- abx
								lic <= '1';
								left_ctrl <= ix_left;
								right_ctrl <= accb_right;
								alu_ctrl <= alu_abx;
								ix_ctrl <= load_ix;
								next_state <= fetch_state;

								--
								-- Return From Interrupt
								--
							WHEN "1011" => -- rti
								next_state <= rti_cc_state;

								--
								-- CWAI
								--
							WHEN "1100" => -- cwai #$<cc_mask>
								-- pre decrement sp
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								-- increment pc
								pc_ctrl <= incr_pc;
								next_state <= cwai_state;

								--
								-- MUL Multiply
								--
							WHEN "1101" => -- mul
								next_state <= mul_state;

								--
								-- SWI Software Interrupt
								--
							WHEN "1111" => -- swi
								-- predecrement SP
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								iv_ctrl <= swi_iv;
								st_ctrl <= push_st;
								return_state <= int_swimask_state;
								next_state <= int_entire_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;
						--
						-- Accumulator A Single operand
						-- source = acca, dest = acca
						-- Do not advance PC
						-- Typically 2 cycles 1 bytes
						-- 1 opcode fetch
						-- 2 post byte fetch / instruction decode
						-- Note that there is no post byte
						-- so do not advance PC in decode cycle
						-- Re-run opcode fetch cycle after decode
						-- 
					WHEN "0100" => -- acca single op
						left_ctrl <= acca_left;
						CASE op_code(3 DOWNTO 0) IS

							WHEN "0000" => -- neg
								right_ctrl <= zero_right;
								alu_ctrl <= alu_neg;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "0011" => -- com
								right_ctrl <= zero_right;
								alu_ctrl <= alu_com;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "0100" => -- lsr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_lsr8;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "0110" => -- ror
								right_ctrl <= zero_right;
								alu_ctrl <= alu_ror8;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "0111" => -- asr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_asr8;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "1000" => -- asl
								right_ctrl <= zero_right;
								alu_ctrl <= alu_asl8;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "1001" => -- rol
								right_ctrl <= zero_right;
								alu_ctrl <= alu_rol8;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "1010" => -- dec
								right_ctrl <= one_right;
								alu_ctrl <= alu_dec;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "1011" => -- undefined
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								acca_ctrl <= latch_acca;
								cc_ctrl <= latch_cc;

							WHEN "1100" => -- inc
								right_ctrl <= one_right;
								alu_ctrl <= alu_inc;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN "1101" => -- tst
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								acca_ctrl <= latch_acca;
								cc_ctrl <= load_cc;

							WHEN "1110" => -- jmp (not defined)
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								acca_ctrl <= latch_acca;
								cc_ctrl <= latch_cc;

							WHEN "1111" => -- clr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_clr;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;

							WHEN OTHERS =>
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								acca_ctrl <= latch_acca;
								cc_ctrl <= latch_cc;

						END CASE;
						lic <= '1';
						next_state <= fetch_state;

						--
						-- Single Operand accb
						-- source = accb, dest = accb
						-- Typically 2 cycles 1 bytes
						-- 1 opcode fetch
						-- 2 post byte fetch / instruction decode
						-- Note that there is no post byte
						-- so do not advance PC in decode cycle
						-- Re-run opcode fetch cycle after decode
						--
					WHEN "0101" =>
						left_ctrl <= accb_left;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- neg
								right_ctrl <= zero_right;
								alu_ctrl <= alu_neg;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "0011" => -- com
								right_ctrl <= zero_right;
								alu_ctrl <= alu_com;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "0100" => -- lsr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_lsr8;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "0110" => -- ror
								right_ctrl <= zero_right;
								alu_ctrl <= alu_ror8;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "0111" => -- asr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_asr8;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "1000" => -- asl
								right_ctrl <= zero_right;
								alu_ctrl <= alu_asl8;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "1001" => -- rol
								right_ctrl <= zero_right;
								alu_ctrl <= alu_rol8;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "1010" => -- dec
								right_ctrl <= one_right;
								alu_ctrl <= alu_dec;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "1011" => -- undefined
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								accb_ctrl <= latch_accb;
								cc_ctrl <= latch_cc;

							WHEN "1100" => -- inc
								right_ctrl <= one_right;
								alu_ctrl <= alu_inc;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN "1101" => -- tst
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								accb_ctrl <= latch_accb;
								cc_ctrl <= load_cc;

							WHEN "1110" => -- jmp (undefined)
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								accb_ctrl <= latch_accb;
								cc_ctrl <= latch_cc;

							WHEN "1111" => -- clr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_clr;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;

							WHEN OTHERS =>
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								accb_ctrl <= latch_accb;
								cc_ctrl <= latch_cc;
						END CASE;
						lic <= '1';
						next_state <= fetch_state;

						--
						-- Single operand indexed
						-- Two byte instruction so advance PC
						-- EA should hold index offset
						--
					WHEN "0110" => -- indexed single op
						-- increment the pc 
						pc_ctrl <= incr_pc;
						st_ctrl <= push_st;

						CASE op_code(3 DOWNTO 0) IS
							WHEN "1110" => -- jmp
								return_state <= jmp_state;

							WHEN "1111" => -- clr
								return_state <= single_op_exec_state;

							WHEN OTHERS =>
								return_state <= single_op_read_state;

						END CASE;
						next_state <= indexed_state;

						--
						-- Single operand extended addressing
						-- three byte instruction so advance the PC
						-- Low order EA holds high order address
						--
					WHEN "0111" => -- extended single op
						-- increment PC
						pc_ctrl <= incr_pc;
						st_ctrl <= push_st;

						CASE op_code(3 DOWNTO 0) IS
							WHEN "1110" => -- jmp
								return_state <= jmp_state;

							WHEN "1111" => -- clr
								return_state <= single_op_exec_state;

							WHEN OTHERS =>
								return_state <= single_op_read_state;

						END CASE;
						next_state <= extended_state;

					WHEN "1000" => -- acca immediate
						-- increment the pc
						pc_ctrl <= incr_pc;

						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- subd #
								"1100" | -- cmpx #
								"1110" => -- ldx #
								next_state <= imm16_state;

								--
								-- bsr offset - Branch to subroutine (2 bytes)
								-- 6809 => 7 cycles
								-- cpu09 => 5 cycles
								-- 1 op=(pc) / pc=pc+1
								-- 2 md_hi=sign(pc) / md_lo=(pc) / sp=sp-1 / pc=pc+1
								-- 3 (sp)=pc_lo / sp=sp-1
								-- 4 (sp)=pc_hi
								-- 5 pc=pc+md
								--
							WHEN "1101" => -- bsr
								-- pre decrement SP
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								--
								st_ctrl <= push_st;
								return_state <= sbranch_state;
								next_state <= push_return_lo_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1001" => -- acca direct
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- subd
								"1100" | -- cmpx
								"1110" => -- ldx
								next_state <= dual_op_read16_state;

							WHEN "0111" => -- sta direct
								next_state <= dual_op_write8_state;

								--
								-- jsr direct - Jump to subroutine in direct page (2 bytes)
								-- 6809 => 7 cycles
								-- cpu09 => 5 cycles
								-- 1 op=(pc) / pc=pc+1
								-- 2 ea_hi=0 / ea_lo=(pc) / sp=sp-1 / pc=pc+1
								-- 3 (sp)=pc_lo / sp=sp-1
								-- 4 (sp)=pc_hi
								-- 5 pc=ea
								--
							WHEN "1101" => -- jsr direct
								-- pre decrement sp
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								--
								st_ctrl <= push_st;
								return_state <= jmp_state;
								next_state <= push_return_lo_state;
							WHEN "1111" => -- stx direct
								-- idle ALU
								left_ctrl <= ix_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								sp_ctrl <= latch_sp;
								next_state <= dual_op_write16_state;

							WHEN OTHERS =>
								next_state <= dual_op_read8_state;

						END CASE;

					WHEN "1010" => -- acca indexed
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- subd
								"1100" | -- cmpx
								"1110" => -- ldx
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= indexed_state;

							WHEN "0111" => -- staa ,x
								st_ctrl <= push_st;
								return_state <= dual_op_write8_state;
								next_state <= indexed_state;

							WHEN "1101" => -- jsr ,x
								-- DO NOT pre decrement SP
								st_ctrl <= push_st;
								return_state <= jsr_state;
								next_state <= indexed_state;

							WHEN "1111" => -- stx ,x
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= indexed_state;

							WHEN OTHERS =>
								st_ctrl <= push_st;
								return_state <= dual_op_read8_state;
								next_state <= indexed_state;

						END CASE;

					WHEN "1011" => -- acca extended
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- subd
								"1100" | -- cmpx
								"1110" => -- ldx
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= extended_state;

							WHEN "0111" => -- staa >
								st_ctrl <= push_st;
								return_state <= dual_op_write8_state;
								next_state <= extended_state;

							WHEN "1101" => -- jsr >extended
								-- DO NOT pre decrement sp
								st_ctrl <= push_st;
								return_state <= jsr_state;
								next_state <= extended_state;

							WHEN "1111" => -- stx >
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= extended_state;

							WHEN OTHERS =>
								st_ctrl <= push_st;
								return_state <= dual_op_read8_state;
								next_state <= extended_state;

						END CASE;

					WHEN "1100" => -- accb immediate
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- addd #
								"1100" | -- ldd #
								"1110" => -- ldu #
								next_state <= imm16_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1101" => -- accb direct
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- addd
								"1100" | -- ldd
								"1110" => -- ldu
								next_state <= dual_op_read16_state;

							WHEN "0111" => -- stab direct
								next_state <= dual_op_write8_state;

							WHEN "1101" => -- std direct
								next_state <= dual_op_write16_state;

							WHEN "1111" => -- stu direct
								next_state <= dual_op_write16_state;

							WHEN OTHERS =>
								next_state <= dual_op_read8_state;

						END CASE;

					WHEN "1110" => -- accb indexed
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- addd
								"1100" | -- ldd
								"1110" => -- ldu
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= indexed_state;

							WHEN "0111" => -- stab indexed
								st_ctrl <= push_st;
								return_state <= dual_op_write8_state;
								next_state <= indexed_state;

							WHEN "1101" => -- std indexed
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= indexed_state;

							WHEN "1111" => -- stu indexed
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= indexed_state;

							WHEN OTHERS =>
								st_ctrl <= push_st;
								return_state <= dual_op_read8_state;
								next_state <= indexed_state;

						END CASE;

					WHEN "1111" => -- accb extended
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- addd
								"1100" | -- ldd
								"1110" => -- ldu
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= extended_state;

							WHEN "0111" => -- stab extended
								st_ctrl <= push_st;
								return_state <= dual_op_write8_state;
								next_state <= extended_state;

							WHEN "1101" => -- std extended
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= extended_state;

							WHEN "1111" => -- stu  extended
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= extended_state;

							WHEN OTHERS =>
								st_ctrl <= push_st;
								return_state <= dual_op_read8_state;
								next_state <= extended_state;
						END CASE;
						--
						-- not sure why I need this
						--	
					WHEN OTHERS =>
						lic <= '1';
						next_state <= fetch_state;
				END CASE;

				--
				-- Here to decode prefix 2 instruction
				-- and fetch next byte of intruction
				-- whether it be necessary or not
				--
			WHEN decode2_state =>
				-- fetch first byte of address or immediate data
				ea_ctrl <= fetch_first_ea;
				md_ctrl <= fetch_first_md;
				addr_ctrl <= fetch_ad;
				CASE op_code(7 DOWNTO 4) IS
						--
						-- lbcc -- long branch conditional
						-- 6809 => branch 6 cycles, no branch 5 cycles
						-- cpu09 => always 5 cycles
						-- 1 pre=(pc) / pc=pc+1
						-- 2 op=(pc) / pc=pc+1
						-- 3 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1
						-- 4 md_hi=md_lo / md_lo=(pc) / pc=pc+1
						-- 5 if cond pc=pc+md else pc=pc
						--
					WHEN "0010" =>
						-- increment the pc
						pc_ctrl <= incr_pc;
						next_state <= lbranch_state;

						--
						-- Single byte stack operators
						-- Do not advance PC
						--
					WHEN "0011" =>
						CASE op_code(3 DOWNTO 0) IS
							WHEN "1111" => -- swi 2
								-- predecrement sp
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								iv_ctrl <= swi2_iv;
								st_ctrl <= push_st;
								return_state <= vect_hi_state;
								next_state <= int_entire_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

					WHEN "1000" => -- acca immediate
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpd #
								"1100" | -- cmpy #
								"1110" => -- ldy #
								next_state <= imm16_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1001" => -- acca direct
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpd <
								"1100" | -- cmpy <
								"1110" => -- ldy <
								next_state <= dual_op_read16_state;

							WHEN "1111" => -- sty <
								next_state <= dual_op_write16_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1010" => -- acca indexed
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpd ,ind
								"1100" | -- cmpy ,ind
								"1110" => -- ldy ,ind
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= indexed_state;

							WHEN "1111" => -- sty ,ind
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= indexed_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

					WHEN "1011" => -- acca extended
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpd <
								"1100" | -- cmpy <
								"1110" => -- ldy <
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= extended_state;

							WHEN "1111" => -- sty >
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= extended_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1100" => -- accb immediate
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- undef #
								"1100" | -- undef #
								"1110" => -- lds #
								next_state <= imm16_state;

							WHEN OTHERS =>
								next_state <= fetch_state;

						END CASE;

					WHEN "1101" => -- accb direct
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- undef <
								"1100" | -- undef <
								"1110" => -- lds <
								next_state <= dual_op_read16_state;

							WHEN "1111" => -- sts <
								next_state <= dual_op_write16_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1110" => -- accb indexed
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- undef ,ind
								"1100" | -- undef ,ind
								"1110" => -- lds ,ind
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= indexed_state;

							WHEN "1111" => -- sts ,ind
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= indexed_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1111" => -- accb extended
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- undef >
								"1100" | -- undef >
								"1110" => -- lds >
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= extended_state;

							WHEN "1111" => -- sts >
								st_ctrl <= push_st;
								return_state <= dual_op_write16_state;
								next_state <= extended_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

					WHEN OTHERS =>
						lic <= '1';
						next_state <= fetch_state;
				END CASE;
				--
				-- Here to decode instruction
				-- and fetch next byte of intruction
				-- whether it be necessary or not
				--
			WHEN decode3_state =>
				ea_ctrl <= fetch_first_ea;
				md_ctrl <= fetch_first_md;
				addr_ctrl <= fetch_ad;
				dout_ctrl <= md_lo_dout;
				CASE op_code(7 DOWNTO 4) IS
						--
						-- Single byte stack operators
						-- Do not advance PC
						--
					WHEN "0011" =>
						CASE op_code(3 DOWNTO 0) IS
							WHEN "1111" => -- swi3
								-- predecrement sp
								left_ctrl <= sp_left;
								right_ctrl <= one_right;
								alu_ctrl <= alu_sub16;
								sp_ctrl <= load_sp;
								iv_ctrl <= swi3_iv;
								st_ctrl <= push_st;
								return_state <= vect_hi_state;
								next_state <= int_entire_state;
							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

					WHEN "1000" => -- acca immediate
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpu #
								"1100" | -- cmps #
								"1110" => -- undef #
								next_state <= imm16_state;
							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

					WHEN "1001" => -- acca direct
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpu <
								"1100" | -- cmps <
								"1110" => -- undef <
								next_state <= dual_op_read16_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1010" => -- acca indexed
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpu ,X
								"1100" | -- cmps ,X
								"1110" => -- undef ,X
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= indexed_state;

							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;

						END CASE;

					WHEN "1011" => -- acca extended
						-- increment the pc
						pc_ctrl <= incr_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- cmpu >
								"1100" | -- cmps >
								"1110" => -- undef >
								st_ctrl <= push_st;
								return_state <= dual_op_read16_state;
								next_state <= extended_state;
							WHEN OTHERS =>
								lic <= '1';
								next_state <= fetch_state;
						END CASE;

					WHEN OTHERS =>
						lic <= '1';
						next_state <= fetch_state;
				END CASE;

				--
				-- here if ea holds low byte
				-- Direct
				-- Extended
				-- Indexed
				-- read memory location
				--
			WHEN single_op_read_state =>
				-- read memory into md
				md_ctrl <= fetch_first_md;
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= single_op_exec_state;

			WHEN single_op_exec_state =>
				CASE op_code(3 DOWNTO 0) IS
					WHEN "0000" => -- neg
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_neg;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "0011" => -- com
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_com;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "0100" => -- lsr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_lsr8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "0110" => -- ror
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ror8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "0111" => -- asr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asr8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "1000" => -- asl
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asl8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "1001" => -- rol
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_rol8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "1010" => -- dec
						left_ctrl <= md_left;
						right_ctrl <= one_right;
						alu_ctrl <= alu_dec;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "1011" => -- undefined
						lic <= '1';
						next_state <= fetch_state;
					WHEN "1100" => -- inc
						left_ctrl <= md_left;
						right_ctrl <= one_right;
						alu_ctrl <= alu_inc;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN "1101" => -- tst
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st8;
						cc_ctrl <= load_cc;
						lic <= '1';
						next_state <= fetch_state;
					WHEN "1110" => -- jmp
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						pc_ctrl <= load_pc;
						lic <= '1';
						next_state <= fetch_state;
					WHEN "1111" => -- clr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= single_op_write_state;
					WHEN OTHERS =>
						lic <= '1';
						next_state <= fetch_state;
				END CASE;
				--
				-- single operand 8 bit write
				-- Write low 8 bits of ALU output
				-- EA holds address
				-- MD holds data
				--
			WHEN single_op_write_state =>
				-- write ALU low byte output
				addr_ctrl <= write_ad;
				dout_ctrl <= md_lo_dout;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- here if ea holds address of low byte
				-- read memory location
				--
			WHEN dual_op_read8_state =>
				-- read first data byte from ea
				md_ctrl <= fetch_first_md;
				addr_ctrl <= read_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- Here to read a 16 bit value into MD
				-- pointed to by the EA register
				-- The first byte is read
				-- and the EA is incremented
				--
			WHEN dual_op_read16_state =>
				-- increment the effective address
				left_ctrl <= ea_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- read the high byte of the 16 bit data
				md_ctrl <= fetch_first_md;
				addr_ctrl <= read_ad;
				next_state <= dual_op_read16_2_state;

				--
				-- here to read the second byte
				-- pointed to by EA into MD
				--
			WHEN dual_op_read16_2_state =>
				-- read the low byte of the 16 bit data
				md_ctrl <= fetch_next_md;
				addr_ctrl <= read_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- 16 bit Write state
				-- EA hold address of memory to write to
				-- Advance the effective address in ALU
				-- decode op_code to determine which
				-- register to write
				--
			WHEN dual_op_write16_state =>
				-- increment the effective address
				left_ctrl <= ea_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- write the ALU hi byte at ea
				addr_ctrl <= write_ad;
				IF op_code(6) = '0' THEN
					CASE op_code(3 DOWNTO 0) IS
						WHEN "1111" => -- stx / sty
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- sty
									dout_ctrl <= iy_hi_dout;
								WHEN OTHERS => -- page 1 -- stx
									dout_ctrl <= ix_hi_dout;
							END CASE;
						WHEN OTHERS =>
							dout_ctrl <= md_hi_dout;
					END CASE;
				ELSE
					CASE op_code(3 DOWNTO 0) IS
						WHEN "1101" => -- std
							dout_ctrl <= acca_dout; -- acca is high byte of ACCD
						WHEN "1111" => -- stu / sts
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- sts
									dout_ctrl <= sp_hi_dout;
								WHEN OTHERS => -- page 1 -- stu
									dout_ctrl <= up_hi_dout;
							END CASE;
						WHEN OTHERS =>
							dout_ctrl <= md_hi_dout;
					END CASE;
				END IF;
				next_state <= dual_op_write8_state;

				--
				-- Dual operand 8 bit write
				-- Write 8 bit accumulator
				-- or low byte of 16 bit register
				-- EA holds address
				-- decode opcode to determine
				-- which register to apply to the bus
				-- Also set the condition codes here
				--
			WHEN dual_op_write8_state =>
				IF op_code(6) = '0' THEN
					CASE op_code(3 DOWNTO 0) IS
						WHEN "0111" => -- sta
							dout_ctrl <= acca_dout;
						WHEN "1111" => -- stx / sty
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- sty
									dout_ctrl <= iy_lo_dout;
								WHEN OTHERS => -- page 1 -- stx
									dout_ctrl <= ix_lo_dout;
							END CASE;
						WHEN OTHERS =>
							dout_ctrl <= md_lo_dout;
					END CASE;
				ELSE
					CASE op_code(3 DOWNTO 0) IS
						WHEN "0111" => -- stb
							dout_ctrl <= accb_dout;
						WHEN "1101" => -- std
							dout_ctrl <= accb_dout; -- accb is low byte of accd
						WHEN "1111" => -- stu / sts
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- sts
									dout_ctrl <= sp_lo_dout;
								WHEN OTHERS => -- page 1 -- stu
									dout_ctrl <= up_lo_dout;
							END CASE;
						WHEN OTHERS =>
							dout_ctrl <= md_lo_dout;
					END CASE;
				END IF;
				-- write ALU low byte output
				addr_ctrl <= write_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- 16 bit immediate addressing mode
				--
			WHEN imm16_state =>
				-- increment pc
				pc_ctrl <= incr_pc;
				-- fetch next immediate byte
				md_ctrl <= fetch_next_md;
				addr_ctrl <= fetch_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- md & ea holds 8 bit index offset
				-- calculate the effective memory address
				-- using the alu
				--
			WHEN indexed_state =>
				--
				-- decode indexing mode
				--
				IF md(7) = '0' THEN
					CASE md(6 DOWNTO 5) IS
						WHEN "00" =>
							left_ctrl <= ix_left;
						WHEN "01" =>
							left_ctrl <= iy_left;
						WHEN "10" =>
							left_ctrl <= up_left;
						WHEN OTHERS =>
							-- when "11" =>
							left_ctrl <= sp_left;
					END CASE;
					right_ctrl <= md_sign5_right;
					alu_ctrl <= alu_add16;
					ea_ctrl <= load_ea;
					next_state <= saved_state;

				ELSE
					CASE md(3 DOWNTO 0) IS
						WHEN "0000" => -- ,R+
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									left_ctrl <= sp_left;
							END CASE;
							--
							right_ctrl <= zero_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							next_state <= postincr1_state;

						WHEN "0001" => -- ,R++
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= zero_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							next_state <= postincr2_state;

						WHEN "0010" => -- ,-R
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
									ix_ctrl <= load_ix;
								WHEN "01" =>
									left_ctrl <= iy_left;
									iy_ctrl <= load_iy;
								WHEN "10" =>
									left_ctrl <= up_left;
									up_ctrl <= load_up;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
									sp_ctrl <= load_sp;
							END CASE;
							right_ctrl <= one_right;
							alu_ctrl <= alu_sub16;
							ea_ctrl <= load_ea;
							next_state <= saved_state;

						WHEN "0011" => -- ,--R
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
									ix_ctrl <= load_ix;
								WHEN "01" =>
									left_ctrl <= iy_left;
									iy_ctrl <= load_iy;
								WHEN "10" =>
									left_ctrl <= up_left;
									up_ctrl <= load_up;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
									sp_ctrl <= load_sp;
							END CASE;
							right_ctrl <= two_right;
							alu_ctrl <= alu_sub16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "0100" => -- ,R (zero offset)
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= zero_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "0101" => -- ACCB,R
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= accb_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "0110" => -- ACCA,R
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= acca_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "0111" => -- undefined
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= zero_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "1000" => -- offset8,R
							md_ctrl <= fetch_first_md; -- pick up 8 bit offset
							addr_ctrl <= fetch_ad;
							pc_ctrl <= incr_pc;
							next_state <= index8_state;

						WHEN "1001" => -- offset16,R
							md_ctrl <= fetch_first_md; -- pick up first byte of 16 bit offset
							addr_ctrl <= fetch_ad;
							pc_ctrl <= incr_pc;
							next_state <= index16_state;

						WHEN "1010" => -- undefined
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= zero_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							--
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "1011" => -- ACCD,R
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= accd_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN "1100" => -- offset8,PC
							-- fetch 8 bit offset
							md_ctrl <= fetch_first_md;
							addr_ctrl <= fetch_ad;
							pc_ctrl <= incr_pc;
							next_state <= pcrel8_state;

						WHEN "1101" => -- offset16,PC
							-- fetch offset
							md_ctrl <= fetch_first_md;
							addr_ctrl <= fetch_ad;
							pc_ctrl <= incr_pc;
							next_state <= pcrel16_state;

						WHEN "1110" => -- undefined
							CASE md(6 DOWNTO 5) IS
								WHEN "00" =>
									left_ctrl <= ix_left;
								WHEN "01" =>
									left_ctrl <= iy_left;
								WHEN "10" =>
									left_ctrl <= up_left;
								WHEN OTHERS =>
									-- when "11" =>
									left_ctrl <= sp_left;
							END CASE;
							right_ctrl <= zero_right;
							alu_ctrl <= alu_add16;
							ea_ctrl <= load_ea;
							IF md(4) = '0' THEN
								next_state <= saved_state;
							ELSE
								next_state <= indirect_state;
							END IF;

						WHEN OTHERS =>
							--    			when "1111" =>     -- [,address]
							-- advance PC to pick up address
							md_ctrl <= fetch_first_md;
							addr_ctrl <= fetch_ad;
							pc_ctrl <= incr_pc;
							next_state <= indexaddr_state;
					END CASE;
				END IF;

				-- load index register with ea plus one
			WHEN postincr1_state =>
				left_ctrl <= ea_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				CASE md(6 DOWNTO 5) IS
					WHEN "00" =>
						ix_ctrl <= load_ix;
					WHEN "01" =>
						iy_ctrl <= load_iy;
					WHEN "10" =>
						up_ctrl <= load_up;
					WHEN OTHERS =>
						-- when "11" =>
						sp_ctrl <= load_sp;
				END CASE;
				-- return to previous state
				IF md(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;

				-- load index register with ea plus two
			WHEN postincr2_state =>
				-- increment register by two (address)
				left_ctrl <= ea_left;
				right_ctrl <= two_right;
				alu_ctrl <= alu_add16;
				CASE md(6 DOWNTO 5) IS
					WHEN "00" =>
						ix_ctrl <= load_ix;
					WHEN "01" =>
						iy_ctrl <= load_iy;
					WHEN "10" =>
						up_ctrl <= load_up;
					WHEN OTHERS =>
						-- when "11" =>
						sp_ctrl <= load_sp;
				END CASE;
				-- return to previous state
				IF md(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;
				--
				-- ea = index register + md (8 bit signed offset)
				-- ea holds post byte
				--
			WHEN index8_state =>
				CASE ea(6 DOWNTO 5) IS
					WHEN "00" =>
						left_ctrl <= ix_left;
					WHEN "01" =>
						left_ctrl <= iy_left;
					WHEN "10" =>
						left_ctrl <= up_left;
					WHEN OTHERS =>
						-- when "11" =>
						left_ctrl <= sp_left;
				END CASE;
				-- ea = index reg + md
				right_ctrl <= md_sign8_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- return to previous state
				IF ea(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;

				-- fetch low byte of 16 bit indexed offset
			WHEN index16_state =>
				-- advance pc
				pc_ctrl <= incr_pc;
				-- fetch low byte
				md_ctrl <= fetch_next_md;
				addr_ctrl <= fetch_ad;
				next_state <= index16_2_state;

				-- ea = index register + md (16 bit offset)
				-- ea holds post byte
			WHEN index16_2_state =>
				CASE ea(6 DOWNTO 5) IS
					WHEN "00" =>
						left_ctrl <= ix_left;
					WHEN "01" =>
						left_ctrl <= iy_left;
					WHEN "10" =>
						left_ctrl <= up_left;
					WHEN OTHERS =>
						-- when "11" =>
						left_ctrl <= sp_left;
				END CASE;
				-- ea = index reg + md
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- return to previous state
				IF ea(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;
				--
				-- pc relative with 8 bit signed offest
				-- md holds signed offset
				--
			WHEN pcrel8_state =>
				-- ea = pc + signed md
				left_ctrl <= pc_left;
				right_ctrl <= md_sign8_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- return to previous state
				IF ea(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;

				-- pc relative addressing with 16 bit offset
				-- pick up the low byte of the offset in md
				-- advance the pc
			WHEN pcrel16_state =>
				-- advance pc
				pc_ctrl <= incr_pc;
				-- fetch low byte
				md_ctrl <= fetch_next_md;
				addr_ctrl <= fetch_ad;
				next_state <= pcrel16_2_state;

				-- pc relative with16 bit signed offest
				-- md holds signed offset
			WHEN pcrel16_2_state =>
				-- ea = pc +  md
				left_ctrl <= pc_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- return to previous state
				IF ea(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;

				-- indexed to address
				-- pick up the low byte of the address
				-- advance the pc
			WHEN indexaddr_state =>
				-- advance pc
				pc_ctrl <= incr_pc;
				-- fetch low byte
				md_ctrl <= fetch_next_md;
				addr_ctrl <= fetch_ad;
				next_state <= indexaddr2_state;

				-- indexed to absolute address
				-- md holds address
				-- ea hold indexing mode byte
			WHEN indexaddr2_state =>
				-- ea = md
				left_ctrl <= pc_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_ld16;
				ea_ctrl <= load_ea;
				-- return to previous state
				IF ea(4) = '0' THEN
					next_state <= saved_state;
				ELSE
					next_state <= indirect_state;
				END IF;

				--
				-- load md with high byte of indirect address
				-- pointed to by ea
				-- increment ea
				--
			WHEN indirect_state =>
				-- increment ea
				left_ctrl <= ea_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				ea_ctrl <= load_ea;
				-- fetch high byte
				md_ctrl <= fetch_first_md;
				addr_ctrl <= read_ad;
				next_state <= indirect2_state;
				--
				-- load md with low byte of indirect address
				-- pointed to by ea
				-- ea has previously been incremented
				--
			WHEN indirect2_state =>
				-- fetch high byte
				md_ctrl <= fetch_next_md;
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= indirect3_state;
				--
				-- complete idirect addressing
				-- by loading ea with md
				--
			WHEN indirect3_state =>
				-- load ea with md
				left_ctrl <= ea_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_ld16;
				ea_ctrl <= load_ea;
				-- return to previous state
				next_state <= saved_state;

				--
				-- ea holds the low byte of the absolute address
				-- Move ea low byte into ea high byte
				-- load new ea low byte to for absolute 16 bit address
				-- advance the program counter
				--
			WHEN extended_state => -- fetch ea low byte
				-- increment pc
				pc_ctrl <= incr_pc;
				-- fetch next effective address bytes
				ea_ctrl <= fetch_next_ea;
				addr_ctrl <= fetch_ad;
				-- return to previous state
				next_state <= saved_state;

			WHEN lea_state => -- here on load effective address
				-- load index register with effective address
				left_ctrl <= pc_left;
				right_ctrl <= ea_right;
				alu_ctrl <= alu_lea;
				CASE op_code(3 DOWNTO 0) IS
					WHEN "0000" => -- leax
						cc_ctrl <= load_cc;
						ix_ctrl <= load_ix;
					WHEN "0001" => -- leay
						cc_ctrl <= load_cc;
						iy_ctrl <= load_iy;
					WHEN "0010" => -- leas
						sp_ctrl <= load_sp;
					WHEN "0011" => -- leau
						up_ctrl <= load_up;
					WHEN OTHERS =>
						NULL;
				END CASE;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- jump to subroutine
				-- sp=sp-1
				-- call push_return_lo_state to save pc
				-- return to jmp_state
				--
			WHEN jsr_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- call push_return_state
				st_ctrl <= push_st;
				return_state <= jmp_state;
				next_state <= push_return_lo_state;

				--
				-- Load pc with ea
				-- (JMP)
				--
			WHEN jmp_state =>
				-- load PC with effective address
				left_ctrl <= pc_left;
				right_ctrl <= ea_right;
				alu_ctrl <= alu_ld16;
				pc_ctrl <= load_pc;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- long branch or branch to subroutine
				-- pick up next md byte
				-- md_hi = md_lo
				-- md_lo = (pc)
				-- pc=pc+1
				-- if a lbsr push return address
				-- continue to sbranch_state
				-- to evaluate conditional branches
				--
			WHEN lbranch_state =>
				pc_ctrl <= incr_pc;
				-- fetch the next byte into md_lo
				md_ctrl <= fetch_next_md;
				addr_ctrl <= fetch_ad;
				-- if lbsr - push return address
				-- then continue on to short branch
				IF op_code = "00010111" THEN
					st_ctrl <= push_st;
					return_state <= sbranch_state;
					next_state <= push_return_lo_state;
				ELSE
					next_state <= sbranch_state;
				END IF;

				--
				-- here to execute conditional branch
				-- short conditional branch md = signed 8 bit offset
				-- long branch md = 16 bit offset
				-- 
			WHEN sbranch_state =>
				left_ctrl <= pc_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				-- Test condition for branch
				IF op_code(7 DOWNTO 4) = "0010" THEN -- conditional branch
					CASE op_code(3 DOWNTO 0) IS
						WHEN "0000" => -- bra
							cond_true := (1 = 1);
						WHEN "0001" => -- brn
							cond_true := (1 = 0);
						WHEN "0010" => -- bhi
							cond_true := ((cc(CBIT) OR cc(ZBIT)) = '0');
						WHEN "0011" => -- bls
							cond_true := ((cc(CBIT) OR cc(ZBIT)) = '1');
						WHEN "0100" => -- bcc/bhs
							cond_true := (cc(CBIT) = '0');
						WHEN "0101" => -- bcs/blo
							cond_true := (cc(CBIT) = '1');
						WHEN "0110" => -- bne
							cond_true := (cc(ZBIT) = '0');
						WHEN "0111" => -- beq
							cond_true := (cc(ZBIT) = '1');
						WHEN "1000" => -- bvc
							cond_true := (cc(VBIT) = '0');
						WHEN "1001" => -- bvs
							cond_true := (cc(VBIT) = '1');
						WHEN "1010" => -- bpl
							cond_true := (cc(NBIT) = '0');
						WHEN "1011" => -- bmi
							cond_true := (cc(NBIT) = '1');
						WHEN "1100" => -- bge
							cond_true := ((cc(NBIT) XOR cc(VBIT)) = '0');
						WHEN "1101" => -- blt
							cond_true := ((cc(NBIT) XOR cc(VBIT)) = '1');
						WHEN "1110" => -- bgt
							cond_true := ((cc(ZBIT) OR (cc(NBIT) XOR cc(VBIT))) = '0');
						WHEN "1111" => -- ble
							cond_true := ((cc(ZBIT) OR (cc(NBIT) XOR cc(VBIT))) = '1');
						WHEN OTHERS =>
							NULL;
					END CASE;
				END IF;
				IF cond_true THEN
					pc_ctrl <= load_pc;
				END IF;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- push return address onto the S stack
				--
				-- (sp) = pc_lo
				-- sp = sp - 1
				--
			WHEN push_return_lo_state =>
				-- decrement the sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write PC low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= push_return_hi_state;

				--
				-- push program counter hi byte onto the stack
				-- (sp) = pc_hi
				-- sp = sp
				-- return to originating state
				--
			WHEN push_return_hi_state =>
				-- write pc hi bytes
				addr_ctrl <= pushs_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= saved_state;

				--
				-- RTS pull return address from stack
				--
			WHEN pull_return_hi_state =>
				-- increment the sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pulls_ad;
				next_state <= pull_return_lo_state;

			WHEN pull_return_lo_state =>
				-- increment the SP
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pulls_ad;
				dout_ctrl <= pc_lo_dout;
				--
				lic <= '1';
				next_state <= fetch_state;

			WHEN andcc_state =>
				-- AND CC with md
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_andcc;
				cc_ctrl <= load_cc;
				--
				lic <= '1';
				next_state <= fetch_state;

			WHEN orcc_state =>
				-- OR CC with md
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_orcc;
				cc_ctrl <= load_cc;
				--
				lic <= '1';
				next_state <= fetch_state;

			WHEN tfr_state =>
				-- select source register
				CASE md(7 DOWNTO 4) IS
					WHEN "0000" =>
						left_ctrl <= accd_left;
					WHEN "0001" =>
						left_ctrl <= ix_left;
					WHEN "0010" =>
						left_ctrl <= iy_left;
					WHEN "0011" =>
						left_ctrl <= up_left;
					WHEN "0100" =>
						left_ctrl <= sp_left;
					WHEN "0101" =>
						left_ctrl <= pc_left;
					WHEN "1000" =>
						left_ctrl <= acca_left;
					WHEN "1001" =>
						left_ctrl <= accb_left;
					WHEN "1010" =>
						left_ctrl <= cc_left;
					WHEN "1011" =>
						left_ctrl <= dp_left;
					WHEN OTHERS =>
						left_ctrl <= md_left;
				END CASE;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_tfr;
				-- select destination register
				CASE md(3 DOWNTO 0) IS
					WHEN "0000" => -- accd
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
					WHEN "0001" => -- ix
						ix_ctrl <= load_ix;
					WHEN "0010" => -- iy
						iy_ctrl <= load_iy;
					WHEN "0011" => -- up
						up_ctrl <= load_up;
					WHEN "0100" => -- sp
						sp_ctrl <= load_sp;
					WHEN "0101" => -- pc
						pc_ctrl <= load_pc;
					WHEN "1000" => -- acca
						acca_ctrl <= load_acca;
					WHEN "1001" => -- accb
						accb_ctrl <= load_accb;
					WHEN "1010" => -- cc
						cc_ctrl <= load_cc;
					WHEN "1011" => --dp
						dp_ctrl <= load_dp;
					WHEN OTHERS =>
						NULL;
				END CASE;
				--
				lic <= '1';
				next_state <= fetch_state;

			WHEN exg_state =>
				-- save destination register
				CASE md(3 DOWNTO 0) IS
					WHEN "0000" =>
						left_ctrl <= accd_left;
					WHEN "0001" =>
						left_ctrl <= ix_left;
					WHEN "0010" =>
						left_ctrl <= iy_left;
					WHEN "0011" =>
						left_ctrl <= up_left;
					WHEN "0100" =>
						left_ctrl <= sp_left;
					WHEN "0101" =>
						left_ctrl <= pc_left;
					WHEN "1000" =>
						left_ctrl <= acca_left;
					WHEN "1001" =>
						left_ctrl <= accb_left;
					WHEN "1010" =>
						left_ctrl <= cc_left;
					WHEN "1011" =>
						left_ctrl <= dp_left;
					WHEN OTHERS =>
						left_ctrl <= md_left;
				END CASE;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_tfr;
				ea_ctrl <= load_ea;
				-- call tranfer microcode
				next_state <= exg1_state;

			WHEN exg1_state =>
				-- select source register
				CASE md(7 DOWNTO 4) IS
					WHEN "0000" =>
						left_ctrl <= accd_left;
					WHEN "0001" =>
						left_ctrl <= ix_left;
					WHEN "0010" =>
						left_ctrl <= iy_left;
					WHEN "0011" =>
						left_ctrl <= up_left;
					WHEN "0100" =>
						left_ctrl <= sp_left;
					WHEN "0101" =>
						left_ctrl <= pc_left;
					WHEN "1000" =>
						left_ctrl <= acca_left;
					WHEN "1001" =>
						left_ctrl <= accb_left;
					WHEN "1010" =>
						left_ctrl <= cc_left;
					WHEN "1011" =>
						left_ctrl <= dp_left;
					WHEN OTHERS =>
						left_ctrl <= md_left;
				END CASE;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_tfr;
				-- select destination register
				CASE md(3 DOWNTO 0) IS
					WHEN "0000" => -- accd
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
					WHEN "0001" => -- ix
						ix_ctrl <= load_ix;
					WHEN "0010" => -- iy
						iy_ctrl <= load_iy;
					WHEN "0011" => -- up
						up_ctrl <= load_up;
					WHEN "0100" => -- sp
						sp_ctrl <= load_sp;
					WHEN "0101" => -- pc
						pc_ctrl <= load_pc;
					WHEN "1000" => -- acca
						acca_ctrl <= load_acca;
					WHEN "1001" => -- accb
						accb_ctrl <= load_accb;
					WHEN "1010" => -- cc
						cc_ctrl <= load_cc;
					WHEN "1011" => --dp
						dp_ctrl <= load_dp;
					WHEN OTHERS =>
						NULL;
				END CASE;
				next_state <= exg2_state;

			WHEN exg2_state =>
				-- restore destination
				left_ctrl <= ea_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_tfr;
				-- save as source register
				CASE md(7 DOWNTO 4) IS
					WHEN "0000" => -- accd
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
					WHEN "0001" => -- ix
						ix_ctrl <= load_ix;
					WHEN "0010" => -- iy
						iy_ctrl <= load_iy;
					WHEN "0011" => -- up
						up_ctrl <= load_up;
					WHEN "0100" => -- sp
						sp_ctrl <= load_sp;
					WHEN "0101" => -- pc
						pc_ctrl <= load_pc;
					WHEN "1000" => -- acca
						acca_ctrl <= load_acca;
					WHEN "1001" => -- accb
						accb_ctrl <= load_accb;
					WHEN "1010" => -- cc
						cc_ctrl <= load_cc;
					WHEN "1011" => --dp
						dp_ctrl <= load_dp;
					WHEN OTHERS =>
						NULL;
				END CASE;
				lic <= '1';
				next_state <= fetch_state;

			WHEN mul_state =>
				-- move acca to md
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_st16;
				md_ctrl <= load_md;
				next_state <= mulea_state;

			WHEN mulea_state =>
				-- move accb to ea
				left_ctrl <= accb_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_st16;
				ea_ctrl <= load_ea;
				next_state <= muld_state;

			WHEN muld_state =>
				-- clear accd
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_ld8;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				next_state <= mul0_state;

			WHEN mul0_state =>
				-- if bit 0 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(0) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul1_state;

			WHEN mul1_state =>
				-- if bit 1 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(1) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul2_state;

			WHEN mul2_state =>
				-- if bit 2 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(2) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul3_state;

			WHEN mul3_state =>
				-- if bit 3 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(3) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul4_state;

			WHEN mul4_state =>
				-- if bit 4 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(4) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul5_state;

			WHEN mul5_state =>
				-- if bit 5 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(5) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul6_state;

			WHEN mul6_state =>
				-- if bit 6 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(6) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				next_state <= mul7_state;

			WHEN mul7_state =>
				-- if bit 7 of ea set, add accd to md
				left_ctrl <= accd_left;
				IF ea(7) = '1' THEN
					right_ctrl <= md_right;
				ELSE
					right_ctrl <= zero_right;
				END IF;
				alu_ctrl <= alu_mul;
				cc_ctrl <= load_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				md_ctrl <= shiftl_md;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- Enter here on pushs
				-- ea holds post byte
				--
			WHEN pshs_state =>
				-- decrement sp if any registers to be pushed
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				-- idle	address
				addr_ctrl <= idle_ad;
				dout_ctrl <= cc_dout;
				IF ea(7 DOWNTO 0) = "00000000" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				IF ea(7) = '1' THEN
					next_state <= pshs_pcl_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pshs_upl_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pshs_iyl_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pshs_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshs_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshs_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_pcl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= pshs_pch_state;

			WHEN pshs_pch_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(6 DOWNTO 0) = "0000000" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				-- write pc hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= pc_hi_dout;
				IF ea(6) = '1' THEN
					next_state <= pshs_upl_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pshs_iyl_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pshs_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshs_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshs_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;
			WHEN pshs_upl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= up_lo_dout;
				next_state <= pshs_uph_state;

			WHEN pshs_uph_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(5 DOWNTO 0) = "000000" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				-- write pc hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= up_hi_dout;
				IF ea(5) = '1' THEN
					next_state <= pshs_iyl_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pshs_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshs_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshs_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_iyl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write iy low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= iy_lo_dout;
				next_state <= pshs_iyh_state;

			WHEN pshs_iyh_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(4 DOWNTO 0) = "00000" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				-- write iy hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= iy_hi_dout;
				IF ea(4) = '1' THEN
					next_state <= pshs_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshs_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshs_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_ixl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write ix low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= pshs_ixh_state;

			WHEN pshs_ixh_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(3 DOWNTO 0) = "0000" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				-- write ix hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= ix_hi_dout;
				IF ea(3) = '1' THEN
					next_state <= pshs_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshs_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_dp_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(2 DOWNTO 0) = "000" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				-- write dp
				addr_ctrl <= pushs_ad;
				dout_ctrl <= dp_dout;
				IF ea(2) = '1' THEN
					next_state <= pshs_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_accb_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(1 DOWNTO 0) = "00" THEN
					sp_ctrl <= latch_sp;
				ELSE
					sp_ctrl <= load_sp;
				END IF;
				-- write accb
				addr_ctrl <= pushs_ad;
				dout_ctrl <= accb_dout;
				IF ea(1) = '1' THEN
					next_state <= pshs_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_acca_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(0) = '1' THEN
					sp_ctrl <= load_sp;
				ELSE
					sp_ctrl <= latch_sp;
				END IF;
				-- write acca
				addr_ctrl <= pushs_ad;
				dout_ctrl <= acca_dout;
				IF ea(0) = '1' THEN
					next_state <= pshs_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshs_cc_state =>
				-- idle sp
				-- write cc
				addr_ctrl <= pushs_ad;
				dout_ctrl <= cc_dout;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- enter here on PULS
				-- ea hold register mask
				--
			WHEN puls_state =>
				IF ea(0) = '1' THEN
					next_state <= puls_cc_state;
				ELSIF ea(1) = '1' THEN
					next_state <= puls_acca_state;
				ELSIF ea(2) = '1' THEN
					next_state <= puls_accb_state;
				ELSIF ea(3) = '1' THEN
					next_state <= puls_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= puls_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= puls_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_cc_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read cc
				cc_ctrl <= pull_cc;
				addr_ctrl <= pulls_ad;
				IF ea(1) = '1' THEN
					next_state <= puls_acca_state;
				ELSIF ea(2) = '1' THEN
					next_state <= puls_accb_state;
				ELSIF ea(3) = '1' THEN
					next_state <= puls_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= puls_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= puls_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_acca_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read acca
				acca_ctrl <= pull_acca;
				addr_ctrl <= pulls_ad;
				IF ea(2) = '1' THEN
					next_state <= puls_accb_state;
				ELSIF ea(3) = '1' THEN
					next_state <= puls_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= puls_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= puls_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_accb_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read accb
				accb_ctrl <= pull_accb;
				addr_ctrl <= pulls_ad;
				IF ea(3) = '1' THEN
					next_state <= puls_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= puls_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= puls_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_dp_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read dp
				dp_ctrl <= pull_dp;
				addr_ctrl <= pulls_ad;
				IF ea(4) = '1' THEN
					next_state <= puls_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= puls_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_ixh_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- pull ix hi
				ix_ctrl <= pull_hi_ix;
				addr_ctrl <= pulls_ad;
				next_state <= puls_ixl_state;

			WHEN puls_ixl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read ix low
				ix_ctrl <= pull_lo_ix;
				addr_ctrl <= pulls_ad;
				IF ea(5) = '1' THEN
					next_state <= puls_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_iyh_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- pull iy hi
				iy_ctrl <= pull_hi_iy;
				addr_ctrl <= pulls_ad;
				next_state <= puls_iyl_state;

			WHEN puls_iyl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read iy low
				iy_ctrl <= pull_lo_iy;
				addr_ctrl <= pulls_ad;
				IF ea(6) = '1' THEN
					next_state <= puls_uph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_uph_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- pull up hi
				up_ctrl <= pull_hi_up;
				addr_ctrl <= pulls_ad;
				next_state <= puls_upl_state;

			WHEN puls_upl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read up low
				up_ctrl <= pull_lo_up;
				addr_ctrl <= pulls_ad;
				IF ea(7) = '1' THEN
					next_state <= puls_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN puls_pch_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- pull pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pulls_ad;
				next_state <= puls_pcl_state;

			WHEN puls_pcl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pulls_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- Enter here on pshu
				-- ea holds post byte
				--
			WHEN pshu_state =>
				-- decrement up if any registers to be pushed
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(7 DOWNTO 0) = "00000000" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write idle bus
				IF ea(7) = '1' THEN
					next_state <= pshu_pcl_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pshu_spl_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pshu_iyl_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pshu_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshu_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshu_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;
				--
				-- push PC onto U stack
				--
			WHEN pshu_pcl_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				up_ctrl <= load_up;
				-- write pc low
				addr_ctrl <= pushu_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= pshu_pch_state;

			WHEN pshu_pch_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(6 DOWNTO 0) = "0000000" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write pc hi
				addr_ctrl <= pushu_ad;
				dout_ctrl <= pc_hi_dout;
				IF ea(6) = '1' THEN
					next_state <= pshu_spl_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pshu_iyl_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pshu_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshu_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshu_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_spl_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				up_ctrl <= load_up;
				-- write sp low
				addr_ctrl <= pushu_ad;
				dout_ctrl <= sp_lo_dout;
				next_state <= pshu_sph_state;

			WHEN pshu_sph_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(5 DOWNTO 0) = "000000" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write sp hi
				addr_ctrl <= pushu_ad;
				dout_ctrl <= sp_hi_dout;
				IF ea(5) = '1' THEN
					next_state <= pshu_iyl_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pshu_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshu_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshu_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_iyl_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				up_ctrl <= load_up;
				-- write iy low
				addr_ctrl <= pushu_ad;
				dout_ctrl <= iy_lo_dout;
				next_state <= pshu_iyh_state;

			WHEN pshu_iyh_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(4 DOWNTO 0) = "00000" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write iy hi
				addr_ctrl <= pushu_ad;
				dout_ctrl <= iy_hi_dout;
				IF ea(4) = '1' THEN
					next_state <= pshu_ixl_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pshu_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshu_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_ixl_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				up_ctrl <= load_up;
				-- write ix low
				addr_ctrl <= pushu_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= pshu_ixh_state;

			WHEN pshu_ixh_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(3 DOWNTO 0) = "0000" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write ix hi
				addr_ctrl <= pushu_ad;
				dout_ctrl <= ix_hi_dout;
				IF ea(3) = '1' THEN
					next_state <= pshu_dp_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pshu_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_dp_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(2 DOWNTO 0) = "000" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write dp
				addr_ctrl <= pushu_ad;
				dout_ctrl <= dp_dout;
				IF ea(2) = '1' THEN
					next_state <= pshu_accb_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_accb_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(1 DOWNTO 0) = "00" THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write accb
				addr_ctrl <= pushu_ad;
				dout_ctrl <= accb_dout;
				IF ea(1) = '1' THEN
					next_state <= pshu_acca_state;
				ELSIF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_acca_state =>
				-- decrement up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				IF ea(0) = '0' THEN
					up_ctrl <= latch_up;
				ELSE
					up_ctrl <= load_up;
				END IF;
				-- write acca
				addr_ctrl <= pushu_ad;
				dout_ctrl <= acca_dout;
				IF ea(0) = '1' THEN
					next_state <= pshu_cc_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pshu_cc_state =>
				-- idle up
				-- write cc
				addr_ctrl <= pushu_ad;
				dout_ctrl <= cc_dout;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- enter here on PULU
				-- ea hold register mask
				--
			WHEN pulu_state =>
				-- idle UP
				-- idle bus
				IF ea(0) = '1' THEN
					next_state <= pulu_cc_state;
				ELSIF ea(1) = '1' THEN
					next_state <= pulu_acca_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pulu_accb_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pulu_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pulu_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pulu_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_cc_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read cc
				cc_ctrl <= pull_cc;
				addr_ctrl <= pullu_ad;
				IF ea(1) = '1' THEN
					next_state <= pulu_acca_state;
				ELSIF ea(2) = '1' THEN
					next_state <= pulu_accb_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pulu_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pulu_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pulu_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_acca_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read acca
				acca_ctrl <= pull_acca;
				addr_ctrl <= pullu_ad;
				IF ea(2) = '1' THEN
					next_state <= pulu_accb_state;
				ELSIF ea(3) = '1' THEN
					next_state <= pulu_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pulu_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pulu_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_accb_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read accb
				accb_ctrl <= pull_accb;
				addr_ctrl <= pullu_ad;
				IF ea(3) = '1' THEN
					next_state <= pulu_dp_state;
				ELSIF ea(4) = '1' THEN
					next_state <= pulu_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pulu_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_dp_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read dp
				dp_ctrl <= pull_dp;
				addr_ctrl <= pullu_ad;
				IF ea(4) = '1' THEN
					next_state <= pulu_ixh_state;
				ELSIF ea(5) = '1' THEN
					next_state <= pulu_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_ixh_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read ix hi
				ix_ctrl <= pull_hi_ix;
				addr_ctrl <= pullu_ad;
				next_state <= pulu_ixl_state;

			WHEN pulu_ixl_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read ix low
				ix_ctrl <= pull_lo_ix;
				addr_ctrl <= pullu_ad;
				IF ea(5) = '1' THEN
					next_state <= pulu_iyh_state;
				ELSIF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_iyh_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read iy hi
				iy_ctrl <= pull_hi_iy;
				addr_ctrl <= pullu_ad;
				next_state <= pulu_iyl_state;

			WHEN pulu_iyl_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read iy low
				iy_ctrl <= pull_lo_iy;
				addr_ctrl <= pullu_ad;
				IF ea(6) = '1' THEN
					next_state <= pulu_sph_state;
				ELSIF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_sph_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read sp hi
				sp_ctrl <= pull_hi_sp;
				addr_ctrl <= pullu_ad;
				next_state <= pulu_spl_state;

			WHEN pulu_spl_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read sp low
				sp_ctrl <= pull_lo_sp;
				addr_ctrl <= pullu_ad;
				IF ea(7) = '1' THEN
					next_state <= pulu_pch_state;
				ELSE
					lic <= '1';
					next_state <= fetch_state;
				END IF;

			WHEN pulu_pch_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- pull pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pullu_ad;
				next_state <= pulu_pcl_state;

			WHEN pulu_pcl_state =>
				-- increment up
				left_ctrl <= up_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				up_ctrl <= load_up;
				-- read pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pullu_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- pop the Condition codes
				--
			WHEN rti_cc_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read cc
				cc_ctrl <= pull_cc;
				addr_ctrl <= pulls_ad;
				next_state <= rti_entire_state;

				--
				-- Added RTI cycle 11th July 2006 John Kent.
				-- test the "Entire" Flag
				-- that has just been popped off the stack
				--
			WHEN rti_entire_state =>
				--
				-- The Entire flag must be recovered from the stack
				-- before testing.
				--
				IF cc(EBIT) = '1' THEN
					next_state <= rti_acca_state;
				ELSE
					next_state <= rti_pch_state;
				END IF;

			WHEN rti_acca_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read acca
				acca_ctrl <= pull_acca;
				addr_ctrl <= pulls_ad;
				next_state <= rti_accb_state;

			WHEN rti_accb_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read accb
				accb_ctrl <= pull_accb;
				addr_ctrl <= pulls_ad;
				next_state <= rti_dp_state;

			WHEN rti_dp_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read dp
				dp_ctrl <= pull_dp;
				addr_ctrl <= pulls_ad;
				next_state <= rti_ixh_state;

			WHEN rti_ixh_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read ix hi
				ix_ctrl <= pull_hi_ix;
				addr_ctrl <= pulls_ad;
				next_state <= rti_ixl_state;

			WHEN rti_ixl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read ix low
				ix_ctrl <= pull_lo_ix;
				addr_ctrl <= pulls_ad;
				next_state <= rti_iyh_state;

			WHEN rti_iyh_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read iy hi
				iy_ctrl <= pull_hi_iy;
				addr_ctrl <= pulls_ad;
				next_state <= rti_iyl_state;

			WHEN rti_iyl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read iy low
				iy_ctrl <= pull_lo_iy;
				addr_ctrl <= pulls_ad;
				next_state <= rti_uph_state;
			WHEN rti_uph_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read up hi
				up_ctrl <= pull_hi_up;
				addr_ctrl <= pulls_ad;
				next_state <= rti_upl_state;

			WHEN rti_upl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read up low
				up_ctrl <= pull_lo_up;
				addr_ctrl <= pulls_ad;
				next_state <= rti_pch_state;

			WHEN rti_pch_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- pull pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pulls_ad;
				next_state <= rti_pcl_state;

			WHEN rti_pcl_state =>
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- pull pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pulls_ad;
				lic <= '1';
				next_state <= fetch_state;

				--
				-- here on NMI interrupt
				-- Complete execute cycle of the last instruction.
				-- If it was a dual operand instruction
				--
			WHEN int_nmi_state =>
				next_state <= int_nmi1_state;

				-- Idle bus cycle
			WHEN int_nmi1_state =>
				-- pre decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				iv_ctrl <= nmi_iv;
				st_ctrl <= push_st;
				return_state <= int_nmimask_state;
				next_state <= int_entire_state;

				--
				-- here on IRQ interrupt
				-- Complete execute cycle of the last instruction.
				-- If it was a dual operand instruction
				--
			WHEN int_irq_state =>
				next_state <= int_irq1_state;

				-- pre decrement the sp
				-- Idle bus cycle
			WHEN int_irq1_state =>
				-- pre decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				iv_ctrl <= irq_iv;
				st_ctrl <= push_st;
				return_state <= int_irqmask_state;
				next_state <= int_entire_state;

				--
				-- here on FIRQ interrupt
				-- Complete execution cycle of the last instruction
				-- if it was a dual operand instruction
				--
			WHEN int_firq_state =>
				next_state <= int_firq1_state;

				-- Idle bus cycle
			WHEN int_firq1_state =>
				-- pre decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				iv_ctrl <= firq_iv;
				st_ctrl <= push_st;
				return_state <= int_firqmask_state;
				next_state <= int_fast_state;

				--
				-- CWAI entry point
				-- stack pointer already pre-decremented
				-- mask condition codes
				--
			WHEN cwai_state =>
				-- AND CC with md
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_andcc;
				cc_ctrl <= load_cc;
				st_ctrl <= push_st;
				return_state <= int_cwai_state;
				next_state <= int_entire_state;

				--
				-- wait here for an interrupt
				--
			WHEN int_cwai_state =>
				IF (nmi_req = '1') THEN
					iv_ctrl <= nmi_iv;
					next_state <= int_nmimask_state;
					--
					-- FIRQ & IRQ are level sensitive
					--
				ELSIF (firq = '1') AND (cc(FBIT) = '0') THEN
					iv_ctrl <= firq_iv;
					next_state <= int_firqmask_state;

				ELSIF (irq = '1') AND (cc(IBIT) = '0') THEN
					iv_ctrl <= irq_iv;
					next_state <= int_irqmask_state;
				ELSE
					next_state <= int_cwai_state;
				END IF;

				--
				-- State to mask I Flag and F Flag (NMI)
				--
			WHEN int_nmimask_state =>
				alu_ctrl <= alu_seif;
				cc_ctrl <= load_cc;
				next_state <= vect_hi_state;

				--
				-- State to mask I Flag and F Flag (FIRQ)
				--
			WHEN int_firqmask_state =>
				alu_ctrl <= alu_seif;
				cc_ctrl <= load_cc;
				next_state <= vect_hi_state;
				--
				-- State to mask I Flag and F Flag (SWI)
				--
			WHEN int_swimask_state =>
				alu_ctrl <= alu_seif;
				cc_ctrl <= load_cc;
				next_state <= vect_hi_state;

				--
				-- State to mask I Flag only (IRQ)
				--
			WHEN int_irqmask_state =>
				alu_ctrl <= alu_sei;
				cc_ctrl <= load_cc;
				next_state <= vect_hi_state;

				--
				-- set Entire Flag on SWI, SWI2, SWI3 and CWAI, IRQ and NMI
				-- before stacking all registers
				--
			WHEN int_entire_state =>
				-- set entire flag
				alu_ctrl <= alu_see;
				cc_ctrl <= load_cc;
				next_state <= int_pcl_state;

				--
				-- clear Entire Flag on FIRQ
				-- before stacking all registers
				--
			WHEN int_fast_state =>
				-- clear entire flag
				alu_ctrl <= alu_cle;
				cc_ctrl <= load_cc;
				next_state <= int_pcl_state;

			WHEN int_pcl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= int_pch_state;

			WHEN int_pch_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write pc hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= pc_hi_dout;
				IF cc(EBIT) = '1' THEN
					next_state <= int_upl_state;
				ELSE
					next_state <= int_cc_state;
				END IF;

			WHEN int_upl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write up low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= up_lo_dout;
				next_state <= int_uph_state;

			WHEN int_uph_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write ix hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= up_hi_dout;
				next_state <= int_iyl_state;

			WHEN int_iyl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write ix low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= iy_lo_dout;
				next_state <= int_iyh_state;

			WHEN int_iyh_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write ix hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= iy_hi_dout;
				next_state <= int_ixl_state;

			WHEN int_ixl_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write ix low
				addr_ctrl <= pushs_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= int_ixh_state;

			WHEN int_ixh_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write ix hi
				addr_ctrl <= pushs_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= int_dp_state;

			WHEN int_dp_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write accb
				addr_ctrl <= pushs_ad;
				dout_ctrl <= dp_dout;
				next_state <= int_accb_state;

			WHEN int_accb_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write accb
				addr_ctrl <= pushs_ad;
				dout_ctrl <= accb_dout;
				next_state <= int_acca_state;

			WHEN int_acca_state =>
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= one_right;
				alu_ctrl <= alu_sub16;
				sp_ctrl <= load_sp;
				-- write acca
				addr_ctrl <= pushs_ad;
				dout_ctrl <= acca_dout;
				next_state <= int_cc_state;

			WHEN int_cc_state =>
				-- write cc
				addr_ctrl <= pushs_ad;
				dout_ctrl <= cc_dout;
				next_state <= saved_state;

				--
				-- According to the 6809 programming manual:
				-- If an interrupt is received and is masked 
				-- or lasts for less than three cycles, the PC 
				-- will advance to the next instruction.
				-- If an interrupt is unmasked and lasts
				-- for more than three cycles, an interrupt
				-- will be generated.
				-- Note that I don't wait 3 clock cycles.
				-- John Kent 11th July 2006
				--
			WHEN sync_state =>
				lic <= '1';
				ba <= '1';
				--
				-- Version 1.28 2015-05-30
				-- Exit sync_state on interrupt.
				-- If the interrupts are active
				-- they will be caught in the state_machine process
				-- and the interrupt service routine microcode will be executed.
				-- Masked interrupts will exit the sync_state.
				-- Moved from the state_machine process to the state_sequencer process
				--
				IF (firq = '1') OR (irq = '1') THEN
					next_state <= fetch_state;
				ELSE
					next_state <= sync_state;
				END IF;

			WHEN halt_state =>
				--
				-- 2011-10-30 John Kent
				-- ba & bs should be high
				ba <= '1';
				bs <= '1';
				IF halt = '1' THEN
					next_state <= halt_state;
				ELSE
					next_state <= fetch_state;
				END IF;

		END CASE;

		--
		-- Ver 1.23 2011-10-30 John Kent
		-- First instruction cycle might be
		-- fetch_state
		-- halt_state
		-- int_nmirq_state
		-- int_firq_state
		--
		IF fic = '1' THEN
			--
			CASE op_code(7 DOWNTO 6) IS
				WHEN "10" => -- acca
					CASE op_code(3 DOWNTO 0) IS
						WHEN "0000" => -- suba
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_sub8;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "0001" => -- cmpa
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_sub8;
							cc_ctrl <= load_cc;
						WHEN "0010" => -- sbca
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_sbc;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "0011" =>
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- cmpd
									left_ctrl <= accd_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_sub16;
									cc_ctrl <= load_cc;
								WHEN "00010001" => -- page 3 -- cmpu
									left_ctrl <= up_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_sub16;
									cc_ctrl <= load_cc;
								WHEN OTHERS => -- page 1 -- subd
									left_ctrl <= accd_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_sub16;
									cc_ctrl <= load_cc;
									acca_ctrl <= load_hi_acca;
									accb_ctrl <= load_accb;
							END CASE;
						WHEN "0100" => -- anda
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_and;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "0101" => -- bita
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_and;
							cc_ctrl <= load_cc;
						WHEN "0110" => -- ldaa
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_ld8;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "0111" => -- staa
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_st8;
							cc_ctrl <= load_cc;
						WHEN "1000" => -- eora
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_eor;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "1001" => -- adca
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_adc;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "1010" => -- oraa
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_ora;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "1011" => -- adda
							left_ctrl <= acca_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_add8;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_acca;
						WHEN "1100" =>
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- cmpy
									left_ctrl <= iy_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_sub16;
									cc_ctrl <= load_cc;
								WHEN "00010001" => -- page 3 -- cmps
									left_ctrl <= sp_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_sub16;
									cc_ctrl <= load_cc;
								WHEN OTHERS => -- page 1 -- cmpx
									left_ctrl <= ix_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_sub16;
									cc_ctrl <= load_cc;
							END CASE;
						WHEN "1101" => -- bsr / jsr
							NULL;
						WHEN "1110" => -- ldx
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- ldy
									left_ctrl <= iy_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_ld16;
									cc_ctrl <= load_cc;
									iy_ctrl <= load_iy;
								WHEN OTHERS => -- page 1 -- ldx
									left_ctrl <= ix_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_ld16;
									cc_ctrl <= load_cc;
									ix_ctrl <= load_ix;
							END CASE;
						WHEN "1111" => -- stx
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- sty
									left_ctrl <= iy_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_st16;
									cc_ctrl <= load_cc;
								WHEN OTHERS => -- page 1 -- stx
									left_ctrl <= ix_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_st16;
									cc_ctrl <= load_cc;
							END CASE;
						WHEN OTHERS =>
							NULL;
					END CASE;
				WHEN "11" => -- accb dual op
					CASE op_code(3 DOWNTO 0) IS
						WHEN "0000" => -- subb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_sub8;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "0001" => -- cmpb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_sub8;
							cc_ctrl <= load_cc;
						WHEN "0010" => -- sbcb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_sbc;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "0011" => -- addd
							left_ctrl <= accd_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_add16;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_hi_acca;
							accb_ctrl <= load_accb;
						WHEN "0100" => -- andb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_and;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "0101" => -- bitb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_and;
							cc_ctrl <= load_cc;
						WHEN "0110" => -- ldab
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_ld8;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "0111" => -- stab
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_st8;
							cc_ctrl <= load_cc;
						WHEN "1000" => -- eorb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_eor;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "1001" => -- adcb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_adc;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "1010" => -- orab
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_ora;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "1011" => -- addb
							left_ctrl <= accb_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_add8;
							cc_ctrl <= load_cc;
							accb_ctrl <= load_accb;
						WHEN "1100" => -- ldd
							left_ctrl <= accd_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_ld16;
							cc_ctrl <= load_cc;
							acca_ctrl <= load_hi_acca;
							accb_ctrl <= load_accb;
						WHEN "1101" => -- std
							left_ctrl <= accd_left;
							right_ctrl <= md_right;
							alu_ctrl <= alu_st16;
							cc_ctrl <= load_cc;
						WHEN "1110" => -- ldu
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- lds
									left_ctrl <= sp_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_ld16;
									cc_ctrl <= load_cc;
									sp_ctrl <= load_sp;
								WHEN OTHERS => -- page 1 -- ldu
									left_ctrl <= up_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_ld16;
									cc_ctrl <= load_cc;
									up_ctrl <= load_up;
							END CASE;
						WHEN "1111" =>
							CASE pre_code IS
								WHEN "00010000" => -- page 2 -- sts
									left_ctrl <= sp_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_st16;
									cc_ctrl <= load_cc;
								WHEN OTHERS => -- page 1 -- stu
									left_ctrl <= up_left;
									right_ctrl <= md_right;
									alu_ctrl <= alu_st16;
									cc_ctrl <= load_cc;
							END CASE;
						WHEN OTHERS =>
							NULL;
					END CASE;
				WHEN OTHERS =>
					NULL;
			END CASE;

		END IF; -- first instruction cycle (fic)
		lic_out <= lic;
	END PROCESS;

END rtl;