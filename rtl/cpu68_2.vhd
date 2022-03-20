--===========================================================================--
--
-- S Y N T H E Z I A B L E CPU68 C O R E
--
-- www.OpenCores.Org - December 2002
-- This core adheres to the GNU public license 
--
-- File name : cpu68.vhd
--
-- Purpose : Implements a 6800 compatible CPU core with some
-- additional instructions found in the 6801
-- 
-- Dependencies : ieee.Std_Logic_1164
-- ieee.std_logic_unsigned
--
-- Author : John E. Kent 
--
--===========================================================================----
--
-- Revision History:
--
-- Date: Revision Author
-- 22 Sep 2002 0.1 John Kent
--
-- 30 Oct 2002 0.2 John Kent
-- made NMI edge triggered
--
-- 30 Oct 2002 0.3 John Kent
-- more corrections to NMI
-- added wai_wait_state to prevent stack overflow on wai.
--
-- 1 Nov 2002 0.4 John Kent
-- removed WAI states and integrated WAI with the interrupt service routine
-- replace Data out (do) and Data in (di) register with a single Memory Data (md) reg.
-- Added Multiply instruction states.
-- run ALU and CC out of CPU module for timing measurements.
--
-- 3 Nov 2002 0.5 John Kent
-- Memory Data Register was not loaded on Store instructions
-- SEV and CLV were not defined in the ALU
-- Overflow Flag on NEG was incorrect
--
-- 16th Feb 2003 0.6 John Kent
-- Rearranged the execution cycle for dual operand instructions
-- so that occurs during the following fetch cycle.
-- This allows the reduction of one clock cycle from dual operand
-- instruction. Note that this also necessitated re-arranging the
-- program counter so that it is no longer incremented in the ALU.
-- The effective address has also been re-arranged to include a
-- separate added. The STD (store accd) now sets the condition codes.
--
-- 28th Jun 2003 0.7 John Kent
-- Added Hold and Halt signals. Hold is used to steal cycles from the
-- CPU or add wait states. Halt puts the CPU in the inactive state
-- and is only honoured in the fetch cycle. Both signals are active high.
--
-- 9th Jan 2004 0.8 John Kent
-- Clear instruction did an alu_ld8 rather than an alu_clr, so
-- the carry bit was not cleared correctly.
-- This error was picked up by Michael Hassenfratz.
--
-- 2019 Jared Boone
-- Stall1_State1 & Stall2|_State added for better cycle accuracy.
--
-- March 2020 Gyorgy Szombathelyi
-- Runned through VHDLFormatter
-- Set I bit at reset
-- Set I bit on NMI
-- Fixes many Irem sound board issues, where NMI is constantly
-- issued by the ADPCM chip, and interrupted by normal IRQ

LIBRARY ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;

ENTITY cpu68 IS
	PORT (
		clk : IN STD_LOGIC;
		rst : IN STD_LOGIC;
		rw : OUT STD_LOGIC;
		vma : OUT STD_LOGIC;
		address : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		data_in : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		data_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		hold : IN STD_LOGIC;
		halt : IN STD_LOGIC;
		irq : IN STD_LOGIC;
		nmi : IN STD_LOGIC;
		test_alu : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		test_cc : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END;

ARCHITECTURE CPU_ARCH OF cpu68 IS

	CONSTANT SBIT : INTEGER := 7;
	CONSTANT XBIT : INTEGER := 6;
	CONSTANT HBIT : INTEGER := 5;
	CONSTANT IBIT : INTEGER := 4;
	CONSTANT NBIT : INTEGER := 3;
	CONSTANT ZBIT : INTEGER := 2;
	CONSTANT VBIT : INTEGER := 1;
	CONSTANT CBIT : INTEGER := 0;

	TYPE state_type IS (reset_state, fetch_state, decode_state,
		extended_state, indexed_state, read8_state, read16_state, immediate16_state,
		write8_state, write16_state,
		execute_state, halt_state, error_state,
		mul_state, mulea_state, muld_state,
		mul0_state, mul1_state, mul2_state, mul3_state,
		mul4_state, mul5_state, mul6_state, mul7_state,
		jmp_state, jsr_state, jsr1_state,
		branch_state, bsr_state, bsr1_state,
		rts_hi_state, rts_lo_state,
		int_pcl_state, int_pch_state,
		int_ixl_state, int_ixh_state,
		int_cc_state, int_acca_state, int_accb_state,
		int_wai_state, int_mask_state,
		rti_state, rti_cc_state, rti_acca_state, rti_accb_state,
		rti_ixl_state, rti_ixh_state,
		rti_pcl_state, rti_pch_state,
		pula_state, psha_state, pulb_state, pshb_state,
		pulx_lo_state, pulx_hi_state, pshx_lo_state, pshx_hi_state,
		vect_lo_state, vect_hi_state,
		stall1_state, stall2_state);
	TYPE addr_type IS (idle_ad, fetch_ad, read_ad, write_ad, push_ad, pull_ad, int_hi_ad, int_lo_ad);
	TYPE dout_type IS (md_lo_dout, md_hi_dout, acca_dout, accb_dout, ix_lo_dout, ix_hi_dout, cc_dout, pc_lo_dout, pc_hi_dout);
	TYPE op_type IS (reset_op, fetch_op, latch_op);
	TYPE acca_type IS (reset_acca, load_acca, load_hi_acca, pull_acca, latch_acca);
	TYPE accb_type IS (reset_accb, load_accb, pull_accb, latch_accb);
	TYPE cc_type IS (reset_cc, load_cc, pull_cc, latch_cc);
	TYPE ix_type IS (reset_ix, load_ix, pull_lo_ix, pull_hi_ix, latch_ix);
	TYPE sp_type IS (reset_sp, latch_sp, load_sp);
	TYPE pc_type IS (reset_pc, latch_pc, load_ea_pc, add_ea_pc, pull_lo_pc, pull_hi_pc, inc_pc);
	TYPE md_type IS (reset_md, latch_md, load_md, fetch_first_md, fetch_next_md, shiftl_md);
	TYPE ea_type IS (reset_ea, latch_ea, add_ix_ea, load_accb_ea, inc_ea, fetch_first_ea, fetch_next_ea);
	TYPE iv_type IS (reset_iv, latch_iv, swi_iv, nmi_iv, irq_iv);
	TYPE nmi_type IS (reset_nmi, set_nmi, latch_nmi);
	TYPE left_type IS (acca_left, accb_left, accd_left, md_left, ix_left, sp_left);
	TYPE right_type IS (md_right, zero_right, plus_one_right, accb_right);
	TYPE alu_type IS (alu_add8, alu_sub8, alu_add16, alu_sub16, alu_adc, alu_sbc,
		alu_and, alu_ora, alu_eor,
		alu_tst, alu_inc, alu_dec, alu_clr, alu_neg, alu_com,
		alu_inx, alu_dex, alu_cpx,
		alu_lsr16, alu_lsl16,
		alu_ror8, alu_rol8,
		alu_asr8, alu_asl8, alu_lsr8,
		alu_sei, alu_cli, alu_sec, alu_clc, alu_sev, alu_clv, alu_tpa, alu_tap,
		alu_ld8, alu_st8, alu_ld16, alu_st16, alu_nop, alu_daa);

	SIGNAL op_code : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL acca : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL accb : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cc : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cc_out : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL xreg : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL sp : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL ea : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL pc : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL md : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL left : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL right : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL out_alu : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL iv : STD_LOGIC_VECTOR(1 DOWNTO 0);
	SIGNAL nmi_req : STD_LOGIC;
	SIGNAL nmi_ack : STD_LOGIC;

	SIGNAL state : state_type;
	SIGNAL next_state : state_type;
	SIGNAL pc_ctrl : pc_type;
	SIGNAL ea_ctrl : ea_type;
	SIGNAL op_ctrl : op_type;
	SIGNAL md_ctrl : md_type;
	SIGNAL acca_ctrl : acca_type;
	SIGNAL accb_ctrl : accb_type;
	SIGNAL ix_ctrl : ix_type;
	SIGNAL cc_ctrl : cc_type;
	SIGNAL sp_ctrl : sp_type;
	SIGNAL iv_ctrl : iv_type;
	SIGNAL left_ctrl : left_type;
	SIGNAL right_ctrl : right_type;
	SIGNAL alu_ctrl : alu_type;
	SIGNAL addr_ctrl : addr_type;
	SIGNAL dout_ctrl : dout_type;
	SIGNAL nmi_ctrl : nmi_type;
BEGIN
	----------------------------------
	--
	-- Address bus multiplexer
	--
	----------------------------------

	addr_mux : PROCESS (clk, addr_ctrl, pc, ea, sp, iv)
	BEGIN
		CASE addr_ctrl IS
			WHEN idle_ad =>
				address <= "1111111111111111";
				vma <= '0';
				rw <= '1';
			WHEN fetch_ad =>
				address <= pc;
				vma <= '1';
				rw <= '1';
			WHEN read_ad =>
				address <= ea;
				vma <= '1';
				rw <= '1';
			WHEN write_ad =>
				address <= ea;
				vma <= '1';
				rw <= '0';
			WHEN push_ad =>
				address <= sp;
				vma <= '1';
				rw <= '0';
			WHEN pull_ad =>
				address <= sp;
				vma <= '1';
				rw <= '1';
			WHEN int_hi_ad =>
				address <= "1111111111111" & iv & "0";
				vma <= '1';
				rw <= '1';
			WHEN int_lo_ad =>
				address <= "1111111111111" & iv & "1";
				vma <= '1';
				rw <= '1';
			WHEN OTHERS =>
				address <= "1111111111111111";
				vma <= '0';
				rw <= '1';
		END CASE;
	END PROCESS;

	--------------------------------
	--
	-- Data Bus output
	--
	--------------------------------
	dout_mux : PROCESS (clk, dout_ctrl, md, acca, accb, xreg, pc, cc)
	BEGIN
		CASE dout_ctrl IS
			WHEN md_hi_dout => -- alu output
				data_out <= md(15 DOWNTO 8);
			WHEN md_lo_dout =>
				data_out <= md(7 DOWNTO 0);
			WHEN acca_dout => -- accumulator a
				data_out <= acca;
			WHEN accb_dout => -- accumulator b
				data_out <= accb;
			WHEN ix_lo_dout => -- index reg
				data_out <= xreg(7 DOWNTO 0);
			WHEN ix_hi_dout => -- index reg
				data_out <= xreg(15 DOWNTO 8);
			WHEN cc_dout => -- condition codes
				data_out <= cc;
			WHEN pc_lo_dout => -- low order pc
				data_out <= pc(7 DOWNTO 0);
			WHEN pc_hi_dout => -- high order pc
				data_out <= pc(15 DOWNTO 8);
			WHEN OTHERS =>
				data_out <= "00000000";
		END CASE;
	END PROCESS;
	----------------------------------
	--
	-- Program Counter Control
	--
	----------------------------------

	pc_mux : PROCESS (clk, pc_ctrl, pc, out_alu, data_in, ea, hold)
		VARIABLE tempof : STD_LOGIC_VECTOR(15 DOWNTO 0);
		VARIABLE temppc : STD_LOGIC_VECTOR(15 DOWNTO 0);
	BEGIN
		CASE pc_ctrl IS
			WHEN add_ea_pc =>
				IF ea(7) = '0' THEN
					tempof := "00000000" & ea(7 DOWNTO 0);
				ELSE
					tempof := "11111111" & ea(7 DOWNTO 0);
				END IF;
			WHEN inc_pc =>
				tempof := "0000000000000001";
			WHEN OTHERS =>
				tempof := "0000000000000000";
		END CASE;

		CASE pc_ctrl IS
			WHEN reset_pc =>
				temppc := "1111111111111110";
			WHEN load_ea_pc =>
				temppc := ea;
			WHEN pull_lo_pc =>
				temppc(7 DOWNTO 0) := data_in;
				temppc(15 DOWNTO 8) := pc(15 DOWNTO 8);
			WHEN pull_hi_pc =>
				temppc(7 DOWNTO 0) := pc(7 DOWNTO 0);
				temppc(15 DOWNTO 8) := data_in;
			WHEN OTHERS =>
				temppc := pc;
		END CASE;

		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				pc <= pc;
			ELSE
				pc <= temppc + tempof;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- Effective Address Control
	--
	----------------------------------

	ea_mux : PROCESS (clk, ea_ctrl, ea, out_alu, data_in, accb, xreg, hold)
		VARIABLE tempind : STD_LOGIC_VECTOR(15 DOWNTO 0);
		VARIABLE tempea : STD_LOGIC_VECTOR(15 DOWNTO 0);
	BEGIN
		CASE ea_ctrl IS
			WHEN add_ix_ea =>
				tempind := "00000000" & ea(7 DOWNTO 0);
			WHEN inc_ea =>
				tempind := "0000000000000001";
			WHEN OTHERS =>
				tempind := "0000000000000000";
		END CASE;

		CASE ea_ctrl IS
			WHEN reset_ea =>
				tempea := "0000000000000000";
			WHEN load_accb_ea =>
				tempea := "00000000" & accb(7 DOWNTO 0);
			WHEN add_ix_ea =>
				tempea := xreg;
			WHEN fetch_first_ea =>
				tempea(7 DOWNTO 0) := data_in;
				tempea(15 DOWNTO 8) := "00000000";
			WHEN fetch_next_ea =>
				tempea(7 DOWNTO 0) := data_in;
				tempea(15 DOWNTO 8) := ea(7 DOWNTO 0);
			WHEN OTHERS =>
				tempea := ea;
		END CASE;

		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				ea <= ea;
			ELSE
				ea <= tempea + tempind;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Accumulator A
	--
	--------------------------------
	acca_mux : PROCESS (clk, acca_ctrl, out_alu, acca, data_in, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				acca <= acca;
			ELSE
				CASE acca_ctrl IS
					WHEN reset_acca =>
						acca <= "00000000";
					WHEN load_acca =>
						acca <= out_alu(7 DOWNTO 0);
					WHEN load_hi_acca =>
						acca <= out_alu(15 DOWNTO 8);
					WHEN pull_acca =>
						acca <= data_in;
					WHEN OTHERS =>
						-- when latch_acca =>
						acca <= acca;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Accumulator B
	--
	--------------------------------
	accb_mux : PROCESS (clk, accb_ctrl, out_alu, accb, data_in, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				accb <= accb;
			ELSE
				CASE accb_ctrl IS
					WHEN reset_accb =>
						accb <= "00000000";
					WHEN load_accb =>
						accb <= out_alu(7 DOWNTO 0);
					WHEN pull_accb =>
						accb <= data_in;
					WHEN OTHERS =>
						-- when latch_accb =>
						accb <= accb;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- X Index register
	--
	--------------------------------
	ix_mux : PROCESS (clk, ix_ctrl, out_alu, xreg, data_in, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				xreg <= xreg;
			ELSE
				CASE ix_ctrl IS
					WHEN reset_ix =>
						xreg <= "0000000000000000";
					WHEN load_ix =>
						xreg <= out_alu(15 DOWNTO 0);
					WHEN pull_hi_ix =>
						xreg(15 DOWNTO 8) <= data_in;
					WHEN pull_lo_ix =>
						xreg(7 DOWNTO 0) <= data_in;
					WHEN OTHERS =>
						-- when latch_ix =>
						xreg <= xreg;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- stack pointer
	--
	--------------------------------
	sp_mux : PROCESS (clk, sp_ctrl, out_alu, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				sp <= sp;
			ELSE
				CASE sp_ctrl IS
					WHEN reset_sp =>
						sp <= "0000000000000000";
					WHEN load_sp =>
						sp <= out_alu(15 DOWNTO 0);
					WHEN OTHERS =>
						-- when latch_sp =>
						sp <= sp;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	--------------------------------
	--
	-- Memory Data
	--
	--------------------------------
	md_mux : PROCESS (clk, md_ctrl, out_alu, data_in, md, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				md <= md;
			ELSE
				CASE md_ctrl IS
					WHEN reset_md =>
						md <= "0000000000000000";
					WHEN load_md =>
						md <= out_alu(15 DOWNTO 0);
					WHEN fetch_first_md =>
						md(15 DOWNTO 8) <= "00000000";
						md(7 DOWNTO 0) <= data_in;
					WHEN fetch_next_md =>
						md(15 DOWNTO 8) <= md(7 DOWNTO 0);
						md(7 DOWNTO 0) <= data_in;
					WHEN shiftl_md =>
						md(15 DOWNTO 1) <= md(14 DOWNTO 0);
						md(0) <= '0';
					WHEN OTHERS =>
						-- when latch_md =>
						md <= md;
				END CASE;
			END IF;
		END IF;
	END PROCESS;
	----------------------------------
	--
	-- Condition Codes
	--
	----------------------------------

	cc_mux : PROCESS (clk, cc_ctrl, cc_out, cc, data_in, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				cc <= cc;
			ELSE
				CASE cc_ctrl IS
					WHEN reset_cc =>
						cc <= "11010000";
					WHEN load_cc =>
						cc <= cc_out;
					WHEN pull_cc =>
						cc <= data_in;
					WHEN OTHERS =>
						-- when latch_cc =>
						cc <= cc;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- interrupt vector
	--
	----------------------------------

	iv_mux : PROCESS (clk, iv_ctrl, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				iv <= iv;
			ELSE
				CASE iv_ctrl IS
					WHEN reset_iv =>
						iv <= "11";
					WHEN nmi_iv =>
						iv <= "10";
					WHEN swi_iv =>
						iv <= "01";
					WHEN irq_iv =>
						iv <= "00";
					WHEN OTHERS =>
						iv <= iv;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- op code fetch
	--
	----------------------------------

	op_fetch : PROCESS (clk, data_in, op_ctrl, op_code, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				op_code <= op_code;
			ELSE
				CASE op_ctrl IS
					WHEN reset_op =>
						op_code <= "00000001"; -- nop
					WHEN fetch_op =>
						op_code <= data_in;
					WHEN OTHERS =>
						-- when latch_op =>
						op_code <= op_code;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	----------------------------------
	--
	-- Left Mux
	--
	----------------------------------

	left_mux : PROCESS (left_ctrl, acca, accb, xreg, sp, pc, ea, md)
	BEGIN
		CASE left_ctrl IS
			WHEN acca_left =>
				left(15 DOWNTO 8) <= "00000000";
				left(7 DOWNTO 0) <= acca;
			WHEN accb_left =>
				left(15 DOWNTO 8) <= "00000000";
				left(7 DOWNTO 0) <= accb;
			WHEN accd_left =>
				left(15 DOWNTO 8) <= acca;
				left(7 DOWNTO 0) <= accb;
			WHEN ix_left =>
				left <= xreg;
			WHEN sp_left =>
				left <= sp;
			WHEN OTHERS =>
				-- when md_left =>
				left <= md;
		END CASE;
	END PROCESS;
	----------------------------------
	--
	-- Right Mux
	--
	----------------------------------

	right_mux : PROCESS (right_ctrl, data_in, md, accb, ea)
	BEGIN
		CASE right_ctrl IS
			WHEN zero_right =>
				right <= "0000000000000000";
			WHEN plus_one_right =>
				right <= "0000000000000001";
			WHEN accb_right =>
				right <= "00000000" & accb;
			WHEN OTHERS =>
				-- when md_right =>
				right <= md;
		END CASE;
	END PROCESS;

	----------------------------------
	--
	-- Arithmetic Logic Unit
	--
	----------------------------------

	mux_alu : PROCESS (alu_ctrl, cc, left, right, out_alu, cc_out)
		VARIABLE valid_lo, valid_hi : BOOLEAN;
		VARIABLE carry_in : STD_LOGIC;
		VARIABLE daa_reg : STD_LOGIC_VECTOR(7 DOWNTO 0);
	BEGIN
		CASE alu_ctrl IS
			WHEN alu_adc | alu_sbc |
				alu_rol8 | alu_ror8 =>
				carry_in := cc(CBIT);
			WHEN OTHERS =>
				carry_in := '0';
		END CASE;

		valid_lo := left(3 DOWNTO 0) <= 9;
		valid_hi := left(7 DOWNTO 4) <= 9;

		IF (cc(CBIT) = '0') THEN
			IF (cc(HBIT) = '1') THEN
				IF valid_hi THEN
					daa_reg := "00000110";
				ELSE
					daa_reg := "01100110";
				END IF;
			ELSE
				IF valid_lo THEN
					IF valid_hi THEN
						daa_reg := "00000000";
					ELSE
						daa_reg := "01100000";
					END IF;
				ELSE
					IF (left(7 DOWNTO 4) <= 8) THEN
						daa_reg := "00000110";
					ELSE
						daa_reg := "01100110";
					END IF;
				END IF;
			END IF;
		ELSE
			IF (cc(HBIT) = '1') THEN
				daa_reg := "01100110";
			ELSE
				IF valid_lo THEN
					daa_reg := "01100000";
				ELSE
					daa_reg := "01100110";
				END IF;
			END IF;
		END IF;

		CASE alu_ctrl IS
			WHEN alu_add8 | alu_inc |
				alu_add16 | alu_inx |
				alu_adc =>
				out_alu <= left + right + ("000000000000000" & carry_in);
			WHEN alu_sub8 | alu_dec |
				alu_sub16 | alu_dex |
				alu_sbc | alu_cpx =>
				out_alu <= left - right - ("000000000000000" & carry_in);
			WHEN alu_and =>
				out_alu <= left AND right; -- and/bit
			WHEN alu_ora =>
				out_alu <= left OR right; -- or
			WHEN alu_eor =>
				out_alu <= left XOR right; -- eor/xor
			WHEN alu_lsl16 | alu_asl8 | alu_rol8 =>
				out_alu <= left(14 DOWNTO 0) & carry_in; -- rol8/asl8/lsl16
			WHEN alu_lsr16 | alu_lsr8 =>
				out_alu <= carry_in & left(15 DOWNTO 1); -- lsr
			WHEN alu_ror8 =>
				out_alu <= "00000000" & carry_in & left(7 DOWNTO 1); -- ror
			WHEN alu_asr8 =>
				out_alu <= "00000000" & left(7) & left(7 DOWNTO 1); -- asr
			WHEN alu_neg =>
				out_alu <= right - left; -- neg (right=0)
			WHEN alu_com =>
				out_alu <= NOT left;
			WHEN alu_clr | alu_ld8 | alu_ld16 =>
				out_alu <= right; -- clr, ld
			WHEN alu_st8 | alu_st16 =>
				out_alu <= left;
			WHEN alu_daa =>
				out_alu <= left + ("00000000" & daa_reg);
			WHEN alu_tpa =>
				out_alu <= "00000000" & cc;
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
			WHEN alu_daa =>
				IF (daa_reg(7 DOWNTO 4) = "0110") THEN
					cc_out(CBIT) <= '1';
				ELSE
					cc_out(CBIT) <= '0';
				END IF;
			WHEN alu_sec =>
				cc_out(CBIT) <= '1';
			WHEN alu_clc =>
				cc_out(CBIT) <= '0';
			WHEN alu_tap =>
				cc_out(CBIT) <= left(CBIT);
			WHEN OTHERS => -- carry is not affected by cpx
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
				alu_ld8 | alu_st8 =>
				cc_out(ZBIT) <= NOT(out_alu(7) OR out_alu(6) OR out_alu(5) OR out_alu(4) OR
				out_alu(3) OR out_alu(2) OR out_alu(1) OR out_alu(0));
			WHEN alu_add16 | alu_sub16 |
				alu_lsl16 | alu_lsr16 |
				alu_inx | alu_dex |
				alu_ld16 | alu_st16 | alu_cpx =>
				cc_out(ZBIT) <= NOT(out_alu(15) OR out_alu(14) OR out_alu(13) OR out_alu(12) OR
				out_alu(11) OR out_alu(10) OR out_alu(9) OR out_alu(8) OR
				out_alu(7) OR out_alu(6) OR out_alu(5) OR out_alu(4) OR
				out_alu(3) OR out_alu(2) OR out_alu(1) OR out_alu(0));
			WHEN alu_tap =>
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
				alu_ld8 | alu_st8 =>
				cc_out(NBIT) <= out_alu(7);
			WHEN alu_add16 | alu_sub16 |
				alu_lsl16 | alu_lsr16 |
				alu_ld16 | alu_st16 | alu_cpx =>
				cc_out(NBIT) <= out_alu(15);
			WHEN alu_tap =>
				cc_out(NBIT) <= left(NBIT);
			WHEN OTHERS =>
				cc_out(NBIT) <= cc(NBIT);
		END CASE;

		--
		-- Interrupt mask flag
		--
		CASE alu_ctrl IS
			WHEN alu_sei =>
				cc_out(IBIT) <= '1'; -- set interrupt mask
			WHEN alu_cli =>
				cc_out(IBIT) <= '0'; -- clear interrupt mask
			WHEN alu_tap =>
				cc_out(IBIT) <= left(IBIT);
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
			WHEN alu_tap =>
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
			WHEN alu_sub16 | alu_cpx =>
				cc_out(VBIT) <= (left(15) AND (NOT right(15)) AND (NOT out_alu(15))) OR
				((NOT left(15)) AND right(15) AND out_alu(15));
			WHEN alu_inc =>
				cc_out(VBIT) <= ((NOT left(7)) AND left(6) AND left(5) AND left(4) AND
				left(3) AND left(2) AND left(1) AND left(0));
			WHEN alu_dec | alu_neg =>
				cc_out(VBIT) <= (left(7) AND (NOT left(6)) AND (NOT left(5)) AND (NOT left(4)) AND
				(NOT left(3)) AND (NOT left(2)) AND (NOT left(1)) AND (NOT left(0)));
			WHEN alu_asr8 =>
				cc_out(VBIT) <= left(0) XOR left(7);
			WHEN alu_lsr8 | alu_lsr16 =>
				cc_out(VBIT) <= left(0);
			WHEN alu_ror8 =>
				cc_out(VBIT) <= left(0) XOR cc(CBIT);
			WHEN alu_lsl16 =>
				cc_out(VBIT) <= left(15) XOR left(14);
			WHEN alu_rol8 | alu_asl8 =>
				cc_out(VBIT) <= left(7) XOR left(6);
			WHEN alu_tap =>
				cc_out(VBIT) <= left(VBIT);
			WHEN alu_and | alu_ora | alu_eor | alu_com |
				alu_st8 | alu_st16 | alu_ld8 | alu_ld16 |
				alu_clv =>
				cc_out(VBIT) <= '0';
			WHEN alu_sev =>
				cc_out(VBIT) <= '1';
			WHEN OTHERS =>
				cc_out(VBIT) <= cc(VBIT);
		END CASE;

		CASE alu_ctrl IS
			WHEN alu_tap =>
				cc_out(XBIT) <= cc(XBIT) AND left(XBIT);
				cc_out(SBIT) <= left(SBIT);
			WHEN OTHERS =>
				cc_out(XBIT) <= cc(XBIT) AND left(XBIT);
				cc_out(SBIT) <= cc(SBIT);
		END CASE;

		test_alu <= out_alu;
		test_cc <= cc_out;
	END PROCESS;

	------------------------------------
	--
	-- Detect Edge of NMI interrupt
	--
	------------------------------------

	nmi_handler : PROCESS (clk, rst, nmi, nmi_ack)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				nmi_req <= nmi_req;
			ELSE
				IF rst = '1' THEN
					nmi_req <= '0';
				ELSE
					IF (nmi = '1') AND (nmi_ack = '0') THEN
						nmi_req <= '1';
					ELSE
						IF (nmi = '0') AND (nmi_ack = '1') THEN
							nmi_req <= '0';
						ELSE
							nmi_req <= nmi_req;
						END IF;
					END IF;
				END IF;
			END IF;
		END IF;
	END PROCESS;

	------------------------------------
	--
	-- Nmi mux
	--
	------------------------------------

	nmi_mux : PROCESS (clk, nmi_ctrl, nmi_ack, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF hold = '1' THEN
				nmi_ack <= nmi_ack;
			ELSE
				CASE nmi_ctrl IS
					WHEN set_nmi =>
						nmi_ack <= '1';
					WHEN reset_nmi =>
						nmi_ack <= '0';
					WHEN OTHERS =>
						-- when latch_nmi =>
						nmi_ack <= nmi_ack;
				END CASE;
			END IF;
		END IF;
	END PROCESS;

	------------------------------------
	--
	-- state sequencer
	--
	------------------------------------
	PROCESS (state, op_code, cc, ea, irq, nmi_req, nmi_ack, hold, halt)
	BEGIN
		CASE state IS
			WHEN reset_state => -- released from reset
				-- reset the registers
				op_ctrl <= reset_op;
				acca_ctrl <= reset_acca;
				accb_ctrl <= reset_accb;
				ix_ctrl <= reset_ix;
				sp_ctrl <= reset_sp;
				pc_ctrl <= reset_pc;
				ea_ctrl <= reset_ea;
				md_ctrl <= reset_md;
				iv_ctrl <= reset_iv;
				nmi_ctrl <= reset_nmi;
				-- idle the ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= reset_cc;
				-- idle the bus
				dout_ctrl <= md_lo_dout;
				addr_ctrl <= idle_ad;
				next_state <= vect_hi_state;

				--
				-- Jump via interrupt vector
				-- iv holds interrupt type
				-- fetch PC hi from vector location
				--
			WHEN vect_hi_state =>
				-- default the registers
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				ea_ctrl <= latch_ea;
				iv_ctrl <= latch_iv;
				-- idle the ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- fetch pc low interrupt vector
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= int_hi_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= vect_lo_state;
				--
				-- jump via interrupt vector
				-- iv holds vector type
				-- fetch PC lo from vector location
				--
			WHEN vect_lo_state =>
				-- default the registers
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				ea_ctrl <= latch_ea;
				iv_ctrl <= latch_iv;
				-- idle the ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- fetch the vector low byte
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= int_lo_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= fetch_state;

				--
				-- Here to fetch an instruction
				-- PC points to opcode
				-- Should service interrupt requests at this point
				-- either from the timer
				-- or from the external input.
				--
			WHEN fetch_state =>
				CASE op_code(7 DOWNTO 4) IS
					WHEN "0000" |
						"0001" |
						"0010" | -- branch conditional
						"0011" |
						"0100" | -- acca single op
						"0101" | -- accb single op
						"0110" | -- indexed single op
						"0111" => -- extended single op
						-- idle ALU
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;

					WHEN "1000" | -- acca immediate
						"1001" | -- acca direct
						"1010" | -- acca indexed
						"1011" => -- acca extended
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- suba
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sub8;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0001" => -- cmpa
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sub8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0010" => -- sbca
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sbc;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0011" => -- subd
								left_ctrl <= accd_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sub16;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_hi_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0100" => -- anda
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_and;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0101" => -- bita
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_and;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0110" => -- ldaa
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ld8;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0111" => -- staa
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1000" => -- eora
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_eor;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1001" => -- adca
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_adc;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1010" => -- oraa
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ora;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1011" => -- adda
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_add8;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1100" => -- cpx
								left_ctrl <= ix_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_cpx;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1101" => -- bsr / jsr
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1110" => -- lds
								left_ctrl <= sp_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ld16;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
							WHEN "1111" => -- sts
								left_ctrl <= sp_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
						END CASE;
					WHEN "1100" | -- accb immediate
						"1101" | -- accb direct
						"1110" | -- accb indexed
						"1111" => -- accb extended
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- subb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sub8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0001" => -- cmpb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sub8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0010" => -- sbcb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_sbc;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0011" => -- addd
								left_ctrl <= accd_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_hi_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0100" => -- andb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_and;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0101" => -- bitb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_and;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0110" => -- ldab
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ld8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "0111" => -- stab
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1000" => -- eorb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_eor;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1001" => -- adcb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_adc;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1010" => -- orab
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ora;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1011" => -- addb
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_add8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1100" => -- ldd
								left_ctrl <= accd_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ld16;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_hi_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1101" => -- std
								left_ctrl <= accd_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN "1110" => -- ldx
								left_ctrl <= ix_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_ld16;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= load_ix;
								sp_ctrl <= latch_sp;
							WHEN "1111" => -- stx
								left_ctrl <= ix_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
							WHEN OTHERS =>
								left_ctrl <= accb_left;
								right_ctrl <= md_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
						END CASE;
					WHEN OTHERS =>
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
				END CASE;
				md_ctrl <= latch_md;
				-- fetch the op code
				op_ctrl <= fetch_op;
				ea_ctrl <= reset_ea;
				addr_ctrl <= fetch_ad;
				dout_ctrl <= md_lo_dout;
				iv_ctrl <= latch_iv;
				IF halt = '1' THEN
					pc_ctrl <= latch_pc;
					nmi_ctrl <= latch_nmi;
					next_state <= halt_state;
					-- service non maskable interrupts
				ELSIF (nmi_req = '1') AND (nmi_ack = '0') THEN
					pc_ctrl <= latch_pc;
					nmi_ctrl <= set_nmi;
					next_state <= int_pcl_state;
					-- service maskable interrupts
				ELSE
					--
					-- nmi request is not cleared until nmi input goes low
					--
					IF (nmi_req = '0') AND (nmi_ack = '1') THEN
						nmi_ctrl <= reset_nmi;
					ELSE
						nmi_ctrl <= latch_nmi;
					END IF;
					--
					-- IRQ is level sensitive
					--
					IF (irq = '1') AND (cc(IBIT) = '0') THEN
						pc_ctrl <= latch_pc;
						next_state <= int_pcl_state;
					ELSE
						-- Advance the PC to fetch next instruction byte
						pc_ctrl <= inc_pc;
						next_state <= decode_state;
					END IF;
				END IF;
				--
				-- Here to decode instruction
				-- and fetch next byte of intruction
				-- whether it be necessary or not
				--
			WHEN decode_state =>
				-- fetch first byte of address or immediate data
				ea_ctrl <= fetch_first_ea;
				addr_ctrl <= fetch_ad;
				dout_ctrl <= md_lo_dout;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				iv_ctrl <= latch_iv;
				CASE op_code(7 DOWNTO 4) IS
					WHEN "0000" =>
						md_ctrl <= fetch_first_md;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0001" => -- nop
								left_ctrl <= accd_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "0100" => -- lsrd
								left_ctrl <= accd_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_lsr16;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_hi_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "0101" => -- lsld
								left_ctrl <= accd_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_lsl16;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_hi_acca;
								accb_ctrl <= load_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "0110" => -- tap
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_tap;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "0111" => -- tpa
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_tpa;
								cc_ctrl <= latch_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "1000" => -- inx
								left_ctrl <= ix_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_inx;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= load_ix;
								next_state <= fetch_state;
							WHEN "1001" => -- dex
								left_ctrl <= ix_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_dex;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= load_ix;
								next_state <= stall2_state;
							WHEN "1010" => -- clv
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_clv;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "1011" => -- sev
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_sev;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "1100" => -- clc
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_clc;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "1101" => -- sec
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_sec;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "1110" => -- cli
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_cli;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN "1111" => -- sei
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_sei;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
								ix_ctrl <= latch_ix;
								next_state <= fetch_state;
						END CASE;
						-- acca / accb inherent instructions
					WHEN "0001" =>
						md_ctrl <= fetch_first_md;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						left_ctrl <= acca_left;
						right_ctrl <= accb_right;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- sba
								alu_ctrl <= alu_sub8;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
							WHEN "0001" => -- cba
								alu_ctrl <= alu_sub8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
							WHEN "0110" => -- tab
								alu_ctrl <= alu_st8;
								cc_ctrl <= load_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= load_accb;
							WHEN "0111" => -- tba
								alu_ctrl <= alu_ld8;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
							WHEN "1001" => -- daa
								alu_ctrl <= alu_daa;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
							WHEN "1011" => -- aba
								alu_ctrl <= alu_add8;
								cc_ctrl <= load_cc;
								acca_ctrl <= load_acca;
								accb_ctrl <= latch_accb;
							WHEN OTHERS =>
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								acca_ctrl <= latch_acca;
								accb_ctrl <= latch_accb;
						END CASE;
						next_state <= fetch_state;
					WHEN "0010" => -- branch conditional
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						-- increment the pc
						pc_ctrl <= inc_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- bra
								next_state <= branch_state;
							WHEN "0001" => -- brn
								next_state <= stall2_state;
							WHEN "0010" => -- bhi
								IF (cc(CBIT) OR cc(ZBIT)) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "0011" => -- bls
								IF (cc(CBIT) OR cc(ZBIT)) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "0100" => -- bcc/bhs
								IF cc(CBIT) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "0101" => -- bcs/blo
								IF cc(CBIT) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "0110" => -- bne
								IF cc(ZBIT) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "0111" => -- beq
								IF cc(ZBIT) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1000" => -- bvc
								IF cc(VBIT) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1001" => -- bvs
								IF cc(VBIT) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1010" => -- bpl
								IF cc(NBIT) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1011" => -- bmi
								IF cc(NBIT) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1100" => -- bge
								IF (cc(NBIT) XOR cc(VBIT)) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1101" => -- blt
								IF (cc(NBIT) XOR cc(VBIT)) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1110" => -- bgt
								IF (cc(ZBIT) OR (cc(NBIT) XOR cc(VBIT))) = '0' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN "1111" => -- ble
								IF (cc(ZBIT) OR (cc(NBIT) XOR cc(VBIT))) = '1' THEN
									next_state <= branch_state;
								ELSE
									next_state <= stall2_state;
								END IF;
							WHEN OTHERS =>
								next_state <= stall2_state;
						END CASE;
						--
						-- Single byte stack operators
						-- Do not advance PC
						--
					WHEN "0011" =>
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- tsx
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= load_ix;
								sp_ctrl <= latch_sp;
								next_state <= fetch_state;
							WHEN "0001" => -- ins
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= fetch_state;
							WHEN "0010" => -- pula
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= pula_state;
							WHEN "0011" => -- pulb
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= pulb_state;
							WHEN "0100" => -- des
								-- decrement sp
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_sub16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= fetch_state;
							WHEN "0101" => -- txs
								left_ctrl <= ix_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_sub16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= fetch_state;
							WHEN "0110" => -- psha
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= psha_state;
							WHEN "0111" => -- pshb
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= pshb_state;
							WHEN "1000" => -- pulx
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= pulx_hi_state;
							WHEN "1001" => -- rts
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= rts_hi_state;
							WHEN "1010" => -- abx
								left_ctrl <= ix_left;
								right_ctrl <= accb_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= load_ix;
								sp_ctrl <= latch_sp;
								next_state <= fetch_state;
							WHEN "1011" => -- rti
								left_ctrl <= sp_left;
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= load_sp;
								next_state <= rti_cc_state;
							WHEN "1100" => -- pshx
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= pshx_lo_state;
							WHEN "1101" => -- mul
								left_ctrl <= acca_left;
								right_ctrl <= accb_right;
								alu_ctrl <= alu_add16;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= mul_state;
							WHEN "1110" => -- wai
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= int_pcl_state;
							WHEN "1111" => -- swi
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= int_pcl_state;
							WHEN OTHERS =>
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								ix_ctrl <= latch_ix;
								sp_ctrl <= latch_sp;
								next_state <= fetch_state;
						END CASE;
						--
						-- Accumulator A Single operand
						-- source = Acc A dest = Acc A
						-- Do not advance PC
						--
					WHEN "0100" => -- acca single op
						md_ctrl <= fetch_first_md;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
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
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_dec;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;
							WHEN "1011" => -- undefined
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								acca_ctrl <= latch_acca;
								cc_ctrl <= latch_cc;
							WHEN "1100" => -- inc
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_inc;
								acca_ctrl <= load_acca;
								cc_ctrl <= load_cc;
							WHEN "1101" => -- tst
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								acca_ctrl <= latch_acca;
								cc_ctrl <= load_cc;
							WHEN "1110" => -- jmp
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
						next_state <= fetch_state;
						--
						-- single operand acc b
						-- Do not advance PC
						--
					WHEN "0101" =>
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						pc_ctrl <= latch_pc;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
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
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_dec;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;
							WHEN "1011" => -- undefined
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								accb_ctrl <= latch_accb;
								cc_ctrl <= latch_cc;
							WHEN "1100" => -- inc
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_inc;
								accb_ctrl <= load_accb;
								cc_ctrl <= load_cc;
							WHEN "1101" => -- tst
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								accb_ctrl <= latch_accb;
								cc_ctrl <= load_cc;
							WHEN "1110" => -- jmp
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
						next_state <= fetch_state;
						--
						-- Single operand indexed
						-- Two byte instruction so advance PC
						-- EA should hold index offset
						--
					WHEN "0110" => -- indexed single op
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						next_state <= indexed_state;
						--
						-- Single operand extended addressing
						-- three byte instruction so advance the PC
						-- Low order EA holds high order address
						--
					WHEN "0111" => -- extended single op
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						next_state <= extended_state;

					WHEN "1000" => -- acca immediate
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- subdd #
								"1100" | -- cpx #
								"1110" => -- lds #
								next_state <= immediate16_state;
							WHEN "1101" => -- bsr
								next_state <= bsr_state;
							WHEN OTHERS =>
								next_state <= fetch_state;
						END CASE;

					WHEN "1001" => -- acca direct
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						pc_ctrl <= inc_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0111" => -- staa direct
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1111" => -- sts direct
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN "1101" => -- jsr direct
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								next_state <= jsr_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								next_state <= read8_state;
						END CASE;

					WHEN "1010" => -- acca indexed
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						next_state <= indexed_state;

					WHEN "1011" => -- acca extended
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						next_state <= extended_state;

					WHEN "1100" => -- accb immediate
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- addd #
								"1100" | -- ldd #
								"1110" => -- ldx #
								next_state <= immediate16_state;
							WHEN OTHERS =>
								next_state <= fetch_state;
						END CASE;

					WHEN "1101" => -- accb direct
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						pc_ctrl <= inc_pc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0111" => -- stab direct
								left_ctrl <= accb_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1101" => -- std direct
								left_ctrl <= accd_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN "1111" => -- stx direct
								left_ctrl <= ix_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								next_state <= read8_state;
						END CASE;

					WHEN "1110" => -- accb indexed
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						next_state <= indexed_state;

					WHEN "1111" => -- accb extended
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- increment the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= inc_pc;
						next_state <= extended_state;

					WHEN OTHERS =>
						md_ctrl <= fetch_first_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						-- idle the pc
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						pc_ctrl <= latch_pc;
						next_state <= fetch_state;
				END CASE;

			WHEN immediate16_state =>
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				op_ctrl <= latch_op;
				iv_ctrl <= latch_iv;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment pc
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				pc_ctrl <= inc_pc;
				-- fetch next immediate byte
				md_ctrl <= fetch_next_md;
				addr_ctrl <= fetch_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;
				--
				-- ea holds 8 bit index offet
				-- calculate the effective memory address
				-- using the alu
				--
			WHEN indexed_state =>
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- calculate effective address from index reg
				-- index offest is not sign extended
				ea_ctrl <= add_ix_ea;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				-- work out next state
				CASE op_code(7 DOWNTO 4) IS
					WHEN "0110" => -- single op indexed
						md_ctrl <= latch_md;
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "1011" => -- undefined
								next_state <= fetch_state;
							WHEN "1110" => -- jmp
								next_state <= jmp_state;
							WHEN OTHERS =>
								next_state <= read8_state;
						END CASE;
					WHEN "1010" => -- acca indexed
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0111" => -- staa
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1101" => -- jsr
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= jsr_state;
							WHEN "1111" => -- sts
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= read8_state;
						END CASE;
					WHEN "1110" => -- accb indexed
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0111" => -- stab direct
								left_ctrl <= accb_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1101" => -- std direct
								left_ctrl <= accd_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN "1111" => -- stx direct
								left_ctrl <= ix_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= read8_state;
						END CASE;
					WHEN OTHERS =>
						md_ctrl <= latch_md;
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= fetch_state;
				END CASE;
				--
				-- ea holds the low byte of the absolute address
				-- Move ea low byte into ea high byte
				-- load new ea low byte to for absolute 16 bit address
				-- advance the program counter
				--
			WHEN extended_state => -- fetch ea low byte
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- increment pc
				pc_ctrl <= inc_pc;
				-- fetch next effective address bytes
				ea_ctrl <= fetch_next_ea;
				addr_ctrl <= fetch_ad;
				dout_ctrl <= md_lo_dout;
				-- work out the next state
				CASE op_code(7 DOWNTO 4) IS
					WHEN "0111" => -- single op extended
						md_ctrl <= latch_md;
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "1011" => -- undefined
								next_state <= fetch_state;
							WHEN "1110" => -- jmp
								next_state <= jmp_state;
							WHEN OTHERS =>
								next_state <= read8_state;
						END CASE;
					WHEN "1011" => -- acca extended
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0111" => -- staa
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1101" => -- jsr
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= jsr_state;
							WHEN "1111" => -- sts
								left_ctrl <= sp_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= read8_state;
						END CASE;
					WHEN "1111" => -- accb extended
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0111" => -- stab
								left_ctrl <= accb_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1101" => -- std
								left_ctrl <= accd_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN "1111" => -- stx
								left_ctrl <= ix_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st16;
								cc_ctrl <= latch_cc;
								md_ctrl <= load_md;
								next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= read8_state;
						END CASE;
					WHEN OTHERS =>
						md_ctrl <= latch_md;
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= fetch_state;
				END CASE;
				--
				-- here if ea holds low byte (direct page)
				-- can enter here from extended addressing
				-- read memory location
				-- note that reads may be 8 or 16 bits
				--
			WHEN read8_state => -- read data
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				--
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				CASE op_code(7 DOWNTO 4) IS
					WHEN "0110" | "0111" => -- single operand
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						md_ctrl <= fetch_first_md;
						ea_ctrl <= latch_ea;
						next_state <= execute_state;

					WHEN "1001" | "1010" | "1011" => -- acca
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- subd
								"1110" | -- lds
								"1100" => -- cpx
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								-- increment the effective address in case of 16 bit load
								ea_ctrl <= inc_ea;
								next_state <= read16_state;
								-- when "0111" => -- staa
								-- left_ctrl <= acca_left;
								-- right_ctrl <= zero_right;
								-- alu_ctrl <= alu_st8;
								-- cc_ctrl <= latch_cc;
								-- md_ctrl <= load_md;
								-- ea_ctrl <= latch_ea;
								-- next_state <= write8_state;
								-- when "1101" => -- jsr
								-- left_ctrl <= acca_left;
								-- right_ctrl <= zero_right;
								-- alu_ctrl <= alu_nop;
								-- cc_ctrl <= latch_cc;
								-- md_ctrl <= latch_md;
								-- ea_ctrl <= latch_ea;
								-- next_state <= jsr_state;
								-- when "1111" => -- sts
								-- left_ctrl <= sp_left;
								-- right_ctrl <= zero_right;
								-- alu_ctrl <= alu_st16;
								-- cc_ctrl <= latch_cc;
								-- md_ctrl <= load_md;
								-- ea_ctrl <= latch_ea;
								-- next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								ea_ctrl <= latch_ea;
								next_state <= fetch_state;
						END CASE;

					WHEN "1101" | "1110" | "1111" => -- accb
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0011" | -- addd
								"1100" | -- ldd
								"1110" => -- ldx
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								-- increment the effective address in case of 16 bit load
								ea_ctrl <= inc_ea;
								next_state <= read16_state;
								-- when "0111" => -- stab
								-- left_ctrl <= accb_left;
								-- right_ctrl <= zero_right;
								-- alu_ctrl <= alu_st8;
								-- cc_ctrl <= latch_cc;
								-- md_ctrl <= load_md;
								-- ea_ctrl <= latch_ea;
								-- next_state <= write8_state;
								-- when "1101" => -- std
								-- left_ctrl <= accd_left;
								-- right_ctrl <= zero_right;
								-- alu_ctrl <= alu_st16;
								-- cc_ctrl <= latch_cc;
								-- md_ctrl <= load_md;
								-- ea_ctrl <= latch_ea;
								-- next_state <= write16_state;
								-- when "1111" => -- stx
								-- left_ctrl <= ix_left;
								-- right_ctrl <= zero_right;
								-- alu_ctrl <= alu_st16;
								-- cc_ctrl <= latch_cc;
								-- md_ctrl <= load_md;
								-- ea_ctrl <= latch_ea;
								-- next_state <= write16_state;
							WHEN OTHERS =>
								left_ctrl <= acca_left;
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= fetch_first_md;
								ea_ctrl <= latch_ea;
								next_state <= execute_state;
						END CASE;
					WHEN OTHERS =>
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						md_ctrl <= fetch_first_md;
						ea_ctrl <= latch_ea;
						next_state <= fetch_state;
				END CASE;

			WHEN read16_state => -- read second data byte from ea
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle the effective address
				ea_ctrl <= latch_ea;
				-- read the low byte of the 16 bit data
				md_ctrl <= fetch_next_md;
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;
				--
				-- 16 bit Write state
				-- write high byte of ALU output.
				-- EA hold address of memory to write to
				-- Advance the effective address in ALU
				--
			WHEN write16_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- increment the effective address
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				ea_ctrl <= inc_ea;
				-- write the ALU hi byte to ea
				addr_ctrl <= write_ad;
				dout_ctrl <= md_hi_dout;
				next_state <= write8_state;
				--
				-- 8 bit write
				-- Write low 8 bits of ALU output
				--
			WHEN write8_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle the ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- write ALU low byte output
				addr_ctrl <= write_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

			WHEN jmp_state =>
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- load PC with effective address
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				pc_ctrl <= load_ea_pc;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

			WHEN jsr_state => -- JSR
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= jsr1_state;

			WHEN jsr1_state => -- JSR
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc hi
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= jmp_state;

			WHEN branch_state => -- Bcc
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- calculate signed branch
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				pc_ctrl <= add_ea_pc;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_state;

			WHEN bsr_state => -- BSR
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= bsr1_state;

			WHEN bsr1_state => -- BSR
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc hi
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= branch_state;

			WHEN rts_hi_state => -- RTS
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment the sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= rts_lo_state;

			WHEN rts_lo_state => -- RTS1
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle the ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- read pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= fetch_state;

			WHEN mul_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- move acca to md
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_st16;
				cc_ctrl <= latch_cc;
				md_ctrl <= load_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mulea_state;

			WHEN mulea_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				md_ctrl <= latch_md;
				-- idle ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- move accb to ea
				ea_ctrl <= load_accb_ea;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= muld_state;

			WHEN muld_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				md_ctrl <= latch_md;
				-- clear accd
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_ld8;
				cc_ctrl <= latch_cc;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul0_state;

			WHEN mul0_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 0 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(0) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul1_state;

			WHEN mul1_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 1 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(1) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul2_state;

			WHEN mul2_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 2 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(2) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul3_state;

			WHEN mul3_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 3 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(3) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul4_state;

			WHEN mul4_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 4 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(4) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul5_state;

			WHEN mul5_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 5 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(5) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul6_state;

			WHEN mul6_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 6 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(6) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul7_state;

			WHEN mul7_state =>
				-- default
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- if bit 7 of ea set, add accd to md
				left_ctrl <= accd_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				IF ea(7) = '1' THEN
					cc_ctrl <= load_cc;
					acca_ctrl <= load_hi_acca;
					accb_ctrl <= load_accb;
				ELSE
					cc_ctrl <= latch_cc;
					acca_ctrl <= latch_acca;
					accb_ctrl <= latch_accb;
				END IF;
				md_ctrl <= shiftl_md;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

			WHEN execute_state => -- execute single operand instruction
				-- default
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				CASE op_code(7 DOWNTO 4) IS
					WHEN "0110" | -- indexed single op
						"0111" => -- extended single op
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						iv_ctrl <= latch_iv;
						ea_ctrl <= latch_ea;
						-- idle the bus
						addr_ctrl <= idle_ad;
						dout_ctrl <= md_lo_dout;
						left_ctrl <= md_left;
						CASE op_code(3 DOWNTO 0) IS
							WHEN "0000" => -- neg
								right_ctrl <= zero_right;
								alu_ctrl <= alu_neg;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "0011" => -- com
								right_ctrl <= zero_right;
								alu_ctrl <= alu_com;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "0100" => -- lsr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_lsr8;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "0110" => -- ror
								right_ctrl <= zero_right;
								alu_ctrl <= alu_ror8;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "0111" => -- asr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_asr8;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1000" => -- asl
								right_ctrl <= zero_right;
								alu_ctrl <= alu_asl8;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1001" => -- rol
								right_ctrl <= zero_right;
								alu_ctrl <= alu_rol8;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1010" => -- dec
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_dec;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1011" => -- undefined
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= fetch_state;
							WHEN "1100" => -- inc
								right_ctrl <= plus_one_right;
								alu_ctrl <= alu_inc;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN "1101" => -- tst
								right_ctrl <= zero_right;
								alu_ctrl <= alu_st8;
								cc_ctrl <= load_cc;
								md_ctrl <= latch_md;
								next_state <= fetch_state;
							WHEN "1110" => -- jmp
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= fetch_state;
							WHEN "1111" => -- clr
								right_ctrl <= zero_right;
								alu_ctrl <= alu_clr;
								cc_ctrl <= load_cc;
								md_ctrl <= load_md;
								next_state <= write8_state;
							WHEN OTHERS =>
								right_ctrl <= zero_right;
								alu_ctrl <= alu_nop;
								cc_ctrl <= latch_cc;
								md_ctrl <= latch_md;
								next_state <= fetch_state;
						END CASE;

					WHEN OTHERS =>
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						iv_ctrl <= latch_iv;
						ea_ctrl <= latch_ea;
						-- idle the bus
						addr_ctrl <= idle_ad;
						dout_ctrl <= md_lo_dout;
						next_state <= fetch_state;
				END CASE;

			WHEN psha_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write acca
				addr_ctrl <= push_ad;
				dout_ctrl <= acca_dout;
				next_state <= fetch_state;

			WHEN pula_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle sp
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				sp_ctrl <= latch_sp;
				-- read acca
				acca_ctrl <= pull_acca;
				addr_ctrl <= pull_ad;
				dout_ctrl <= acca_dout;
				next_state <= fetch_state;

			WHEN pshb_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write accb
				addr_ctrl <= push_ad;
				dout_ctrl <= accb_dout;
				next_state <= fetch_state;

			WHEN pulb_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle sp
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				sp_ctrl <= latch_sp;
				-- read accb
				accb_ctrl <= pull_accb;
				addr_ctrl <= pull_ad;
				dout_ctrl <= accb_dout;
				next_state <= fetch_state;

			WHEN pshx_lo_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write ix low
				addr_ctrl <= push_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= pshx_hi_state;

			WHEN pshx_hi_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write ix hi
				addr_ctrl <= push_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= fetch_state;

			WHEN pulx_hi_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- pull ix hi
				ix_ctrl <= pull_hi_ix;
				addr_ctrl <= pull_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= pulx_lo_state;

			WHEN pulx_lo_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle sp
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				sp_ctrl <= latch_sp;
				-- read ix low
				ix_ctrl <= pull_lo_ix;
				addr_ctrl <= pull_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= fetch_state;

				--
				-- return from interrupt
				-- enter here from bogus interrupts
				--
			WHEN rti_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- idle address bus
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= cc_dout;
				next_state <= rti_cc_state;

			WHEN rti_cc_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read cc
				cc_ctrl <= pull_cc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= cc_dout;
				next_state <= rti_accb_state;

			WHEN rti_accb_state =>
				-- default registers
				acca_ctrl <= latch_acca;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read accb
				accb_ctrl <= pull_accb;
				addr_ctrl <= pull_ad;
				dout_ctrl <= accb_dout;
				next_state <= rti_acca_state;

			WHEN rti_acca_state =>
				-- default registers
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read acca
				acca_ctrl <= pull_acca;
				addr_ctrl <= pull_ad;
				dout_ctrl <= acca_dout;
				next_state <= rti_ixh_state;

			WHEN rti_ixh_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read ix hi
				ix_ctrl <= pull_hi_ix;
				addr_ctrl <= pull_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= rti_ixl_state;

			WHEN rti_ixl_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read ix low
				ix_ctrl <= pull_lo_ix;
				addr_ctrl <= pull_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= rti_pch_state;

			WHEN rti_pch_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- pull pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= rti_pcl_state;

			WHEN rti_pcl_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle sp
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				sp_ctrl <= latch_sp;
				-- pull pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= fetch_state;

				--
				-- here on interrupt
				-- iv register hold interrupt type
				--
			WHEN int_pcl_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= int_pch_state;

			WHEN int_pch_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc hi
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= int_ixl_state;

			WHEN int_ixl_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write ix low
				addr_ctrl <= push_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= int_ixh_state;

			WHEN int_ixh_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write ix hi
				addr_ctrl <= push_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= int_acca_state;

			WHEN int_acca_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write acca
				addr_ctrl <= push_ad;
				dout_ctrl <= acca_dout;
				next_state <= int_accb_state;
			WHEN int_accb_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write accb
				addr_ctrl <= push_ad;
				dout_ctrl <= accb_dout;
				next_state <= int_cc_state;

			WHEN int_cc_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write cc
				addr_ctrl <= push_ad;
				dout_ctrl <= cc_dout;
				nmi_ctrl <= latch_nmi;
				--
				-- nmi is edge triggered
				-- nmi_req is cleared when nmi goes low.
				--
				IF nmi_req = '1' THEN
					iv_ctrl <= nmi_iv;
					next_state <= int_mask_state;
				ELSE
					--
					-- IRQ is level sensitive
					--
					IF (irq = '1') AND (cc(IBIT) = '0') THEN
						iv_ctrl <= irq_iv;
						next_state <= int_mask_state;
					ELSE
						CASE op_code IS
							WHEN "00111110" => -- WAI (wait for interrupt)
								iv_ctrl <= latch_iv;
								next_state <= int_wai_state;
							WHEN "00111111" => -- SWI (Software interrupt)
								iv_ctrl <= swi_iv;
								next_state <= vect_hi_state;
							WHEN OTHERS => -- bogus interrupt (return)
								iv_ctrl <= latch_iv;
								next_state <= rti_state;
						END CASE;
					END IF;
				END IF;

			WHEN int_wai_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				op_ctrl <= latch_op;
				ea_ctrl <= latch_ea;
				-- enable interrupts
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_cli;
				cc_ctrl <= load_cc;
				sp_ctrl <= latch_sp;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= cc_dout;
				IF (nmi_req = '1') AND (nmi_ack = '0') THEN
					iv_ctrl <= nmi_iv;
					nmi_ctrl <= set_nmi;
					next_state <= vect_hi_state;
				ELSE
					--
					-- nmi request is not cleared until nmi input goes low
					--
					IF (nmi_req = '0') AND (nmi_ack = '1') THEN
						nmi_ctrl <= reset_nmi;
					ELSE
						nmi_ctrl <= latch_nmi;
					END IF;
					--
					-- IRQ is level sensitive
					--
					IF (irq = '1') AND (cc(IBIT) = '0') THEN
						iv_ctrl <= irq_iv;
						next_state <= int_mask_state;
					ELSE
						iv_ctrl <= latch_iv;
						next_state <= int_wai_state;
					END IF;
				END IF;

			WHEN int_mask_state =>
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- Mask IRQ
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_sei;
				cc_ctrl <= load_cc;
				sp_ctrl <= latch_sp;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= vect_hi_state;

			WHEN halt_state => -- halt CPU.
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				IF halt = '1' THEN
					next_state <= halt_state;
				ELSE
					next_state <= fetch_state;
				END IF;

			WHEN stall2_state => -- Do nothing for two cycles
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_state;

			WHEN stall1_state => -- Do nothing for one cycle
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

			WHEN OTHERS => -- error state halt on undefine states
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= error_state;
		END CASE;
	END PROCESS;

	--------------------------------
	--
	-- state machine
	--
	--------------------------------

	change_state : PROCESS (clk, rst, state, hold)
	BEGIN
		IF clk'event AND clk = '0' THEN
			IF rst = '1' THEN
				state <= reset_state;
			ELSIF hold = '1' THEN
				state <= state;
			ELSE
				state <= next_state;
			END IF;
		END IF;
	END PROCESS;
	-- output

END CPU_ARCH;