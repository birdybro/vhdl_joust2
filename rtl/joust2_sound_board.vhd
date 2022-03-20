---------------------------------------------------------------------------------
-- Joust2 sound board by Dar (darfpga@aol.fr)
-- http://darfpga.blogspot.fr
-- https://sourceforge.net/projects/darfpga/files
-- github.com/darfpga
---------------------------------------------------------------------------------
-- gen_ram.vhd 
-------------------------------- 
-- Copyright 2005-2008 by Peter Wendrich (pwsoft@syntiac.com)
-- http://www.syntiac.com/fpga64.html
---------------------------------------------------------------------------------
-- cpu68 - Version 9th Jan 2004 0.8+
-- 6800/01 compatible CPU core 
-- GNU public license - December 2002 : John E. Kent
-- + 2019 Jared Boone
-- + March 2020 Gyorgy Szombathelyi
---------------------------------------------------------------------------------
-- Educational use only
-- Do not redistribute synthetized file with roms
-- Do not redistribute roms whatever the form
-- Use at your own risk
---------------------------------------------------------------------------------
-- Version 0.0 -- 04/03/2022 -- 
--		    initial version
---------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;
USE ieee.numeric_std.ALL;

ENTITY joust2_sound_board IS
	PORT (
		clock_12 : IN STD_LOGIC;
		reset : IN STD_LOGIC;

		sound_select : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		sound_trig : IN STD_LOGIC;
		sound_ack : OUT STD_LOGIC;
		audio_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);

		dbg_cpu_addr : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END joust2_sound_board;

ARCHITECTURE struct OF joust2_sound_board IS

	-- signal reset_n   : std_logic;
	SIGNAL clock_div : STD_LOGIC_VECTOR(3 DOWNTO 0);

	SIGNAL cpu_clock : STD_LOGIC;
	SIGNAL cpu_addr : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL cpu_di : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cpu_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cpu_rw_n : STD_LOGIC;
	SIGNAL cpu_irq : STD_LOGIC;

	SIGNAL wram_cs : STD_LOGIC;
	SIGNAL wram_we : STD_LOGIC;
	SIGNAL wram_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL rom_cs : STD_LOGIC;
	SIGNAL rom_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	-- pia port a
	--      bit 0-7 audio output

	-- pia port b
	--      bit 0-7 sound select input

	-- pia io ca/cb
	--      ca1 => pia_02_cb2 (main cpu part - sound_trig)
	--      cb1 gnd
	--      ca2 => pia_02_cb1 (main cpu part - sound ack)
	--      cb2 gnd

	SIGNAL pia_clock : STD_LOGIC;
	SIGNAL pia_rw_n : STD_LOGIC;
	SIGNAL pia_cs : STD_LOGIC;
	SIGNAL pia_irqa : STD_LOGIC;
	SIGNAL pia_irqb : STD_LOGIC;
	SIGNAL pia_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

BEGIN

	dbg_cpu_addr <= cpu_addr;

	-- clock divider
	PROCESS (reset, clock_12)
	BEGIN
		IF rising_edge(clock_12) THEN
			IF clock_div < 11 THEN
				clock_div <= clock_div + '1';
			ELSE
				clock_div <= (OTHERS => '0');
			END IF;
			IF clock_div > 6 THEN
				cpu_clock <= '1';
			ELSE
				cpu_clock <= '0';
			END IF;

			IF clock_div > 7 AND clock_div < 9 THEN
				pia_clock <= '1';
			ELSE
				pia_clock <= '0';
			END IF;

		END IF;
	END PROCESS;

	-- chip select
	wram_cs <= '1' WHEN cpu_addr(15 DOWNTO 13) = "000" ELSE
		'0';
	pia_cs <= '1' WHEN cpu_addr(15 DOWNTO 13) = "001" ELSE
		'0';
	rom_cs <= '1' WHEN cpu_addr(15 DOWNTO 13) = "111" ELSE
		'0';

	-- write enables
	wram_we <= '1' WHEN cpu_rw_n = '0' AND cpu_clock = '1' AND wram_cs = '1' ELSE
		'0';
	pia_rw_n <= '0' WHEN cpu_rw_n = '0' AND pia_cs = '1' ELSE
		'1';

	-- mux cpu in data between roms/io/wram
	cpu_di <=
		wram_do WHEN wram_cs = '1' ELSE
		pia_do WHEN pia_cs = '1' ELSE
		rom_do WHEN rom_cs = '1' ELSE
		X"55";

	-- pia irqs to cpu
	cpu_irq <= pia_irqa OR pia_irqb;

	-- microprocessor 6800
	main_cpu : ENTITY work.cpu68
		PORT MAP(
			clk => cpu_clock, -- E clock input (falling edge)
			rst => reset, -- reset input (active high)
			rw => cpu_rw_n, -- read not write output
			vma => OPEN, -- valid memory address (active high)
			address => cpu_addr, -- address bus output
			data_in => cpu_di, -- data bus input
			data_out => cpu_do, -- data bus output
			hold => '0', -- hold input (active high) extend bus cycle
			halt => '0', -- halt input (active high) grants DMA
			irq => cpu_irq, -- interrupt request input (active high)
			nmi => '0', -- non maskable interrupt request input (active high)
			test_alu => OPEN,
			test_cc => OPEN
		);

	-- cpu program rom
	cpu_prog_rom : ENTITY work.joust2_sound
		PORT MAP(
			clk => clock_12,
			addr => cpu_addr(12 DOWNTO 0),
			data => rom_do
		);

	-- cpu wram 
	cpu_ram : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 8, aWidth => 8)
		PORT MAP(
			clk => clock_12,
			we => wram_we,
			addr => cpu_addr(7 DOWNTO 0),
			d => cpu_do,
			q => wram_do
		);

	-- pia 
	pia : ENTITY work.pia6821
		PORT MAP
		(
			clk => pia_clock,
			rst => reset,
			cs => pia_cs,
			rw => pia_rw_n,
			addr => cpu_addr(1 DOWNTO 0),
			data_in => cpu_do,
			data_out => pia_do,
			irqa => pia_irqa,
			irqb => pia_irqb,
			pa_i => sound_select,
			pa_o => OPEN,
			pa_oe => OPEN,
			ca1 => sound_trig,
			ca2_i => '0',
			ca2_o => sound_ack,
			ca2_oe => OPEN,
			pb_i => x"00",
			pb_o => audio_out,
			pb_oe => OPEN,
			cb1 => '0',
			cb2_i => '0',
			cb2_o => OPEN,
			cb2_oe => OPEN
		);

END struct;