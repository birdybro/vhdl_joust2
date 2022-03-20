---------------------------------------------------------------------------------
-- Williams by Dar (darfpga@aol.fr)
-- http://darfpga.blogspot.fr
-- https://sourceforge.net/projects/darfpga/files
-- github.com/darfpga
---------------------------------------------------------------------------------
-- gen_ram.vhd & io_ps2_keyboard
-------------------------------- 
-- Copyright 2005-2008 by Peter Wendrich (pwsoft@syntiac.com)
-- http://www.syntiac.com/fpga64.html
---------------------------------------------------------------------------------
-- cpu09l - Version : 0128
-- Synthesizable 6809 instruction compatible VHDL CPU core
-- Copyright (C) 2003 - 2010 John Kent
---------------------------------------------------------------------------------
-- cpu68 - Version 9th Jan 2004 0.8
-- 6800/01 compatible CPU core 
-- GNU public license - December 2002 : John E. Kent
---------------------------------------------------------------------------------
-- Educational use only
-- Do not redistribute synthetized file with roms
-- Do not redistribute roms whatever the form
-- Use at your own risk
---------------------------------------------------------------------------------
-- Version 0.0 -- 05/02/2022 -- 
--		    initial version
---------------------------------------------------------------------------------
--  Features :
--   TV 15KHz mode only (atm)
--   Cocktail mode : todo
-- 
--  Use with MAME roms from joust2.zip
--
---------------------------------------------------------------------------------
--  Use make_joust2_proms.bat to build vhd file and bin from binaries
--  Load sdram with external rom bank -sdram_loader_de10_lite.sof-
---------------------------------------------------------------------------------
---------------------------------------------------------------------------------
-- see joust2 settings whithin joust2_cmos_ram.vhd
---------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.ALL;
USE ieee.numeric_std.ALL;

ENTITY williams2 IS
	PORT (
		clock_12 : IN STD_LOGIC;
		reset : IN STD_LOGIC;

		rom_addr : OUT STD_LOGIC_VECTOR(16 DOWNTO 0);
		rom_do : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		rom_rd : OUT STD_LOGIC;

		-- tv15Khz_mode : in std_logic;
		video_r : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		video_g : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		video_b : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		video_i : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		video_csync : OUT STD_LOGIC;
		video_blankn : OUT STD_LOGIC;
		video_hs : OUT STD_LOGIC;
		video_vs : OUT STD_LOGIC;

		audio_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);

		btn_auto_up : IN STD_LOGIC;
		btn_advance : IN STD_LOGIC;
		btn_high_score_reset : IN STD_LOGIC;
		btn_coin : IN STD_LOGIC;
		btn_start_1 : IN STD_LOGIC;
		btn_start_2 : IN STD_LOGIC;

		btn_left_1 : IN STD_LOGIC;
		btn_right_1 : IN STD_LOGIC;
		btn_trigger1_1 : IN STD_LOGIC;

		btn_left_2 : IN STD_LOGIC;
		btn_right_2 : IN STD_LOGIC;
		btn_trigger1_2 : IN STD_LOGIC;

		sw_coktail_table : IN STD_LOGIC;
		seven_seg : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);

		dbg_out : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)

	);
END williams2;

ARCHITECTURE struct OF williams2 IS

	SIGNAL en_pixel : STD_LOGIC := '0';
	SIGNAL video_access : STD_LOGIC;
	SIGNAL graph_access : STD_LOGIC;

	SIGNAL color_cs : STD_LOGIC;
	SIGNAL rom_bank_cs : STD_LOGIC;

	SIGNAL en_cpu : STD_LOGIC := '0';
	SIGNAL cpu_addr : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL cpu_di : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cpu_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL cpu_rw_n : STD_LOGIC;
	SIGNAL cpu_irq : STD_LOGIC;

	SIGNAL addr_bus : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL data_bus_high : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL data_bus_low : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL data_bus : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL we_bus : STD_LOGIC;

	SIGNAL decod_addr : STD_LOGIC_VECTOR(8 DOWNTO 0);
	SIGNAL decod_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL vram_addr : STD_LOGIC_VECTOR(13 DOWNTO 0);
	SIGNAL vram_cs : STD_LOGIC;
	SIGNAL vram_we : STD_LOGIC;
	SIGNAL vram_l0_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL vram_l0_we : STD_LOGIC;
	SIGNAL vram_h0_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL vram_h0_we : STD_LOGIC;
	SIGNAL vram_l1_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL vram_l1_we : STD_LOGIC;
	SIGNAL vram_h1_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL vram_h1_we : STD_LOGIC;
	SIGNAL vram_l2_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL vram_l2_we : STD_LOGIC;
	SIGNAL vram_h2_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL vram_h2_we : STD_LOGIC;

	SIGNAL rom_bank_a_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL rom_bank_b_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL rom_bank_c_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL rom_bank_d_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL rom_prog1_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL rom_prog2_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	-- signal sram0_we        : std_logic;
	-- signal sram0_do        : std_logic_vector( 7 downto 0);

	SIGNAL page : STD_LOGIC_VECTOR(2 DOWNTO 0);
	SIGNAL page_cs : STD_LOGIC;

	SIGNAL seven_seg_cs : STD_LOGIC;

	SIGNAL flip : STD_LOGIC;
	SIGNAL flip_cs : STD_LOGIC;
	SIGNAL flip_bg : STD_LOGIC;
	SIGNAL flip_bg_a : STD_LOGIC;

	SIGNAL dma_inh_n : STD_LOGIC;
	SIGNAL dma_inh_cs : STD_LOGIC;

	SIGNAL cmos_do : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL cmos_we : STD_LOGIC;

	SIGNAL palette_addr : STD_LOGIC_VECTOR(9 DOWNTO 0);
	SIGNAL palette_lo_we : STD_LOGIC;
	SIGNAL palette_lo_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL palette_hi_we : STD_LOGIC;
	SIGNAL palette_hi_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL fg_color_bank : STD_LOGIC_VECTOR(5 DOWNTO 0);
	SIGNAL fg_color_bank_cs : STD_LOGIC;
	SIGNAL bg_color_bank : STD_LOGIC_VECTOR(5 DOWNTO 0);
	SIGNAL bg_color_bank_cs : STD_LOGIC;

	SIGNAL map_we : STD_LOGIC;
	SIGNAL map_addr : STD_LOGIC_VECTOR(10 DOWNTO 0);
	SIGNAL map_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL map_x : STD_LOGIC_VECTOR(8 DOWNTO 0);
	SIGNAL xscroll_high_cs : STD_LOGIC;
	SIGNAL xscroll_low_cs : STD_LOGIC;
	SIGNAL xscroll : STD_LOGIC_VECTOR(11 DOWNTO 0);

	SIGNAL graph_addr : STD_LOGIC_VECTOR(13 DOWNTO 0);
	SIGNAL graph1_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL graph2_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL graph3_do : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL pias_clock : STD_LOGIC;

	SIGNAL pia_io1_cs : STD_LOGIC;
	SIGNAL pia_io1_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL pia_io1_pa_i : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL pia_io1_pb_i : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL pia_io1_irqa : STD_LOGIC;
	SIGNAL pia_io1_irqb : STD_LOGIC;
	SIGNAL pia_io1_ca2_o : STD_LOGIC;

	SIGNAL pia_io2_cs : STD_LOGIC;
	SIGNAL pia_io2_do : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL pia_io2_irqa : STD_LOGIC;
	SIGNAL pia_io2_irqb : STD_LOGIC;
	SIGNAL pia_io2_pa_i : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL vcnt_240 : STD_LOGIC;
	SIGNAL cnt_4ms : STD_LOGIC;

	SIGNAL pixel_cnt : STD_LOGIC_VECTOR(2 DOWNTO 0) := "000";
	SIGNAL hcnt : STD_LOGIC_VECTOR(5 DOWNTO 0) := "000000";
	SIGNAL vcnt : STD_LOGIC_VECTOR(8 DOWNTO 0) := "000000000";

	SIGNAL fg_pixels : STD_LOGIC_VECTOR(23 DOWNTO 0);
	SIGNAL fg_pixels_0 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels : STD_LOGIC_VECTOR(23 DOWNTO 0);
	SIGNAL bg_pixels_0 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_1 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_2 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_3 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_4 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_5 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_6 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_7 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_8 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_9 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_10 : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL bg_pixels_shifted : STD_LOGIC_VECTOR(3 DOWNTO 0);

	SIGNAL hsync0, hsync1, hsync2, csync, hblank, vblank : STD_LOGIC;

	SIGNAL blit_cs : STD_LOGIC;
	SIGNAL blit_has_bus : STD_LOGIC;
	SIGNAL blit_start : STD_LOGIC;
	SIGNAL blit_cmd : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL blit_color : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL blit_src : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL blit_dst : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL blit_width : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL blit_height : STD_LOGIC_VECTOR(7 DOWNTO 0);

	SIGNAL blit_go : STD_LOGIC;
	SIGNAL blit_cur_src : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL blit_cur_dst : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL blit_cur_width : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL blit_cur_height : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL blit_dst_ori : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL blit_src_ori : STD_LOGIC_VECTOR(15 DOWNTO 0);

	SIGNAL blit_rw_n : STD_LOGIC := '1';
	SIGNAL blit_addr : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL blit_data : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL blit_halt : STD_LOGIC := '0';
	SIGNAL blit_wr_inh_h : STD_LOGIC := '0';
	SIGNAL blit_wr_inh_l : STD_LOGIC := '0';
	SIGNAL right_nibble : STD_LOGIC_VECTOR(3 DOWNTO 0);

	SIGNAL cpu_halt : STD_LOGIC;
	SIGNAL cpu_ba : STD_LOGIC;
	SIGNAL cpu_bs : STD_LOGIC;

	-- signal gun_bin_code  : std_logic_vector(5 downto 0);
	-- signal gun_gray_code : std_logic_vector(5 downto 0);

	SIGNAL sound_select : STD_LOGIC_VECTOR(7 DOWNTO 0);
	SIGNAL sound_trig : STD_LOGIC;
	SIGNAL sound_ack : STD_LOGIC;

	SIGNAL sound_cpu_addr : STD_LOGIC_VECTOR(15 DOWNTO 0);

BEGIN

	-- for debug
	PROCESS (clock_12)
	BEGIN
		IF rising_edge(clock_12) THEN
			--		dbg_out(15 downto 0) <= cpu_addr;
			dbg_out(15 DOWNTO 0) <= sound_cpu_addr;
		END IF;
	END PROCESS;

	-- make pixels counters and cpu clock
	-- in original hardware cpu clock = 1us = 6pixels
	-- here one make 2 cpu clock within 1us
	PROCESS (reset, clock_12)
	BEGIN
		IF rising_edge(clock_12) THEN

			en_pixel <= NOT en_pixel;
			en_cpu <= '0';
			video_access <= '0';
			graph_access <= '0';
			rom_rd <= '0';

			IF pixel_cnt = "000" THEN
				en_cpu <= '1';
			END IF;
			IF pixel_cnt = "001" THEN
				rom_rd <= '1';
			END IF;
			IF pixel_cnt = "011" THEN
				video_access <= '1';
			END IF;
			IF pixel_cnt = "100" THEN
				graph_access <= '1';
			END IF;

			IF en_pixel = '1' THEN
				IF pixel_cnt = "101" THEN
					pixel_cnt <= "000";
				ELSE
					pixel_cnt <= pixel_cnt + '1';
				END IF;

			END IF;

		END IF;
	END PROCESS;

	-- make hcnt and vcnt video scanner from pixel clocks and counts
	-- 
	--  pixels   |0|1|2|3|4|5|0|1|2|3|4|5|
	--  hcnt     |     N     |  N+1      | 
	--
	--  hcnt [0..63] => 64 x 6 = 384 pixels,  1 pixel is 1us => 1 line is 64us (15.625KHz)
	--  vcnt [252..255,256..511] => 260 lines, 1 frame is 260 x 64us = 16.640ms (60.1Hz)
	--
	PROCESS (reset, clock_12)
	BEGIN
		IF reset = '1' THEN
			hcnt <= "000000";
			vcnt <= '0' & X"FC";
		ELSE
			IF rising_edge(clock_12) THEN

				IF (pixel_cnt = "101") AND (en_pixel = '1') THEN
					hcnt <= hcnt + '1';
					IF hcnt = "111101" THEN
						IF vcnt = '1' & X"FF" THEN
							vcnt <= '0' & X"FC";
						ELSE
							vcnt <= vcnt + '1';
						END IF;
					END IF;
				END IF;

			END IF;
		END IF;
	END PROCESS;

	-- mux cpu addr and blitter addr to bus addr
	blit_has_bus <= '1' WHEN cpu_halt = '1' AND cpu_ba = '1' AND cpu_bs = '1' ELSE
		'0';
	addr_bus <= blit_addr WHEN blit_has_bus = '1' ELSE
		cpu_addr;

	-- decod bus addr to vram addr
	decod_addr <= addr_bus(15 DOWNTO 13) & '0' & addr_bus(12 DOWNTO 8);

	-- mux bus addr and video scanner to vram addr
	vram_addr <=
		addr_bus (7 DOWNTO 0) & decod_do(5 DOWNTO 0) WHEN video_access = '0' ELSE
		vcnt(7 DOWNTO 0) & hcnt;

	-- mux bus addr and video scanner to map ram addr
	map_x <= (("000" & (hcnt(5 DOWNTO 0) + 1)) + ('0' & xscroll(10 DOWNTO 3))) XOR (xscroll(11) & x"00");
	map_addr <=
		addr_bus(3 DOWNTO 0) & addr_bus(10 DOWNTO 4) WHEN video_access = '0' ELSE
		vcnt(7 DOWNTO 4) & map_x(8 DOWNTO 2);

	-- bg pixel delay
	PROCESS (clock_12)
	BEGIN
		IF rising_edge(clock_12) THEN
			IF en_pixel = '0' THEN
				IF flip_bg = '0' THEN
					bg_pixels_0 <= bg_pixels(23 DOWNTO 20);
				ELSE
					bg_pixels_0 <= bg_pixels(3 DOWNTO 0);
				END IF;
				bg_pixels_1 <= bg_pixels_0;
				bg_pixels_2 <= bg_pixels_1;
				bg_pixels_3 <= bg_pixels_2;
				bg_pixels_4 <= bg_pixels_3;
				bg_pixels_5 <= bg_pixels_4;
				bg_pixels_6 <= bg_pixels_5;
				bg_pixels_7 <= bg_pixels_6;
				bg_pixels_8 <= bg_pixels_7;
				bg_pixels_9 <= bg_pixels_8;
				bg_pixels_10 <= bg_pixels_9;

				IF flip = '0' THEN
					fg_pixels_0 <= fg_pixels(23 DOWNTO 20);
				ELSE
					fg_pixels_0 <= fg_pixels(3 DOWNTO 0);
				END IF;

			END IF;
		END IF;
	END PROCESS;

	WITH xscroll(2 DOWNTO 0) SELECT
	bg_pixels_shifted <=
		bg_pixels_7 WHEN "000",
		bg_pixels_6 WHEN "001",
		bg_pixels_5 WHEN "010",
		bg_pixels_4 WHEN "011",
		bg_pixels_3 WHEN "100",
		bg_pixels_2 WHEN "101",
		bg_pixels_1 WHEN "110",
		bg_pixels_0 WHEN OTHERS;

	--	mux bus addr and pixels data to palette addr
	palette_addr <=
		addr_bus(10 DOWNTO 1) WHEN color_cs = '1' ELSE
		fg_color_bank & fg_pixels_0 WHEN fg_pixels_0 /= x"0" ELSE
		bg_color_bank(5 DOWNTO 0) & bg_pixels_shifted;
	--	bg_color_bank(5 downto 3) & vcnt(7 downto 5) & bg_pixels_shifted;

	-- palette output to colors bits
	video_r <= palette_lo_do(3 DOWNTO 0);
	video_g <= palette_lo_do(7 DOWNTO 4);
	video_b <= palette_hi_do(3 DOWNTO 0);
	video_i <= palette_hi_do(7 DOWNTO 4);

	-- debug -- bypass palette
	--video_r <= bg_pixels(23) & bg_pixels(22) & "00" when fg_pixels(23 downto 20) = x"0" else fg_pixels(23) & fg_pixels(22) & "00";
	--video_g <= bg_pixels(21) & bg_pixels(21) & "00" when fg_pixels(23 downto 20) = x"0" else fg_pixels(21) & fg_pixels(21) & "00";
	--video_b <= bg_pixels(20) & bg_pixels(20) & "00" when fg_pixels(23 downto 20) = x"0" else fg_pixels(20) & fg_pixels(20) & "00";
	--video_i <= x"F";

	---- 24 bits pixels shift register
	---- 6 pixels of 4 bits
	PROCESS (clock_12)
	BEGIN
		IF rising_edge(clock_12) THEN
			IF en_pixel = '0' THEN
				--		if screen_ctrl = '0' then
				IF video_access = '1' THEN
					fg_pixels <= vram_h0_do & vram_l0_do & vram_h1_do & vram_l1_do & vram_h2_do & vram_l2_do;
					-- map graphics address
					flip_bg_a <= '0'; -- map_do(7);
					--if map_do(7) = '0' then
					graph_addr <= map_do(7 DOWNTO 0) & vcnt(3 DOWNTO 0) & map_x(1) & map_x(0); -- /!\ bit supplementaire 
					--else
					--	graph_addr <= map_do(7 downto 0) & vcnt(3 downto 0) & not map_x(1) & not map_x(0); -- /!\ bit supplementaire
					--end if;
				ELSE
					fg_pixels <= fg_pixels(19 DOWNTO 0) & X"0";
				END IF;

				IF graph_access = '1' THEN
					flip_bg <= flip_bg_a;
					bg_pixels <= graph1_do & graph2_do & graph3_do;
				ELSE
					IF flip_bg = '0' THEN
						bg_pixels <= bg_pixels(19 DOWNTO 0) & X"0";
					ELSE
						bg_pixels <= X"0" & bg_pixels(23 DOWNTO 4);
					END IF;
				END IF;
				--		else
				--		end if;
			END IF;
		END IF;
	END PROCESS;
	pias_clock <= NOT clock_12;
	pia_io1_pa_i(7 DOWNTO 4) <= NOT ("00" & btn_start_1 & btn_start_2);
	pia_io1_pa_i(3 DOWNTO 0) <=
	NOT ('0' & btn_trigger1_1 & btn_right_1 & btn_left_1) WHEN pia_io1_ca2_o = '0' ELSE
	NOT ('0' & btn_trigger1_2 & btn_right_2 & btn_left_2);

	pia_io1_pb_i <= x"ff";
	pia_io2_pa_i <= sw_coktail_table & "000" & btn_coin & btn_high_score_reset & btn_advance & btn_auto_up;

	-- video syncs to pia
	vcnt_240 <= '1' WHEN vcnt = '1' & X"F0" ELSE
		'0';
	cnt_4ms <= vcnt(5);
	--cnt_4ms_o <= vcnt(5);

	-- pia rom irqs to cpu
	cpu_irq <= pia_io2_irqa OR pia_io2_irqb;
	--cpu_irq  <= pia_io1_irqa or pia_io1_irqb or pia_io2_irqa or pia_io2_irqb;

	-- chip select/we
	we_bus <= '1' WHEN (cpu_rw_n = '0' OR blit_rw_n = '0') AND en_pixel = '1' AND en_cpu = '1' ELSE
		'0';

	vram_cs <= '1' WHEN color_cs = '0' AND addr_bus < x"C000" AND
		((blit_has_bus = '0') OR
		(blit_has_bus = '1' AND (addr_bus < x"9000" OR dma_inh_n = '0'))) ELSE
		'0';

	color_cs <= '1' WHEN addr_bus(15 DOWNTO 12) = X"8" AND page(1 DOWNTO 0) = "11" ELSE
		'0'; -- 8000-8FFF & page 3
	rom_bank_cs <= '1' WHEN addr_bus(15) = '0' AND (page /= "000" AND page /= "111") ELSE
		'0'; -- 0000-7000

	blit_cs <= '1' WHEN addr_bus(15 DOWNTO 7) = X"C8" & '1' ELSE
		'0'; -- C880-C8FF ? TBC
	page_cs <= '1' WHEN addr_bus(15 DOWNTO 7) = X"C8" & '0' ELSE
		'0'; -- C800-C87F
	seven_seg_cs <= '1' WHEN addr_bus(15 DOWNTO 0) = X"C98C" ELSE
		'0'; -- C98C
	fg_color_bank_cs <= '1' WHEN addr_bus(15 DOWNTO 5) = X"CB" & "000" ELSE
		'0'; -- CB00-CB1F
	bg_color_bank_cs <= '1' WHEN addr_bus(15 DOWNTO 5) = X"CB" & "001" ELSE
		'0'; -- CB20-CB3F
	xscroll_low_cs <= '1' WHEN addr_bus(15 DOWNTO 5) = X"CB" & "010" ELSE
		'0'; -- CB40-CB5F
	xscroll_high_cs <= '1' WHEN addr_bus(15 DOWNTO 5) = X"CB" & "011" ELSE
		'0'; -- CB60-CB7F
	flip_cs <= '1' WHEN addr_bus(15 DOWNTO 5) = X"CB" & "100" ELSE
		'0'; -- CB80-CB9F
	dma_inh_cs <= '1' WHEN addr_bus(15 DOWNTO 5) = X"CB" & "101" ELSE
		'0'; -- CBA0-CBBF
	pia_io2_cs <= '1' WHEN addr_bus(15 DOWNTO 7) = X"C9" & "1" AND addr_bus(3 DOWNTO 2) = "00" ELSE
		'0'; -- C980-C983
	pia_io1_cs <= '1' WHEN addr_bus(15 DOWNTO 7) = X"C9" & "1" AND addr_bus(3 DOWNTO 2) = "01" ELSE
		'0'; -- C984-C987

	palette_lo_we <= '1' WHEN we_bus = '1' AND color_cs = '1' AND addr_bus(0) = '0' ELSE
		'0';
	palette_hi_we <= '1' WHEN we_bus = '1' AND color_cs = '1' AND addr_bus(0) = '1' ELSE
		'0';
	map_we <= '1' WHEN we_bus = '1' AND addr_bus(15 DOWNTO 11) = X"C" & '0' ELSE
		'0'; -- C000-C7FF
	cmos_we <= '1' WHEN we_bus = '1' AND addr_bus(15 DOWNTO 10) = x"C" & "11" ELSE
		'0'; -- CC00-CFFF
	vram_we <= '1' WHEN we_bus = '1' AND vram_cs = '1' ELSE
		'0';

	-- dispatch we to devices with respect to decoder bits 7-6 and blitter inhibit
	vram_l0_we <= '1' WHEN vram_we = '1' AND blit_wr_inh_l = '0' AND decod_do(7 DOWNTO 6) = "00" ELSE
		'0';
	vram_l1_we <= '1' WHEN vram_we = '1' AND blit_wr_inh_l = '0' AND decod_do(7 DOWNTO 6) = "01" ELSE
		'0';
	vram_l2_we <= '1' WHEN vram_we = '1' AND blit_wr_inh_l = '0' AND decod_do(7 DOWNTO 6) = "10" ELSE
		'0';
	vram_h0_we <= '1' WHEN vram_we = '1' AND blit_wr_inh_h = '0' AND decod_do(7 DOWNTO 6) = "00" ELSE
		'0';
	vram_h1_we <= '1' WHEN vram_we = '1' AND blit_wr_inh_h = '0' AND decod_do(7 DOWNTO 6) = "01" ELSE
		'0';
	vram_h2_we <= '1' WHEN vram_we = '1' AND blit_wr_inh_h = '0' AND decod_do(7 DOWNTO 6) = "10" ELSE
		'0';

	-- mux banked rom address to external (d)ram
	rom_addr <= "00" & addr_bus(14 DOWNTO 0) WHEN (page = "010") ELSE -- bank a
		"01" & addr_bus(14 DOWNTO 0) WHEN (page = "110") ELSE -- bank b
		"10" & addr_bus(14 DOWNTO 0) WHEN (page = "001" OR page = "011") ELSE -- bank c
		"11" & addr_bus(14 DOWNTO 0) WHEN (page = "100" OR page = "101") ELSE -- bank d
		"00" & addr_bus(14 DOWNTO 0); -- bank a

	-- mux data bus between cpu/blitter/roms/io/vram
	data_bus_high <=
		rom_prog2_do WHEN addr_bus(15 DOWNTO 12) >= X"E" ELSE -- 8K
		rom_prog1_do WHEN addr_bus(15 DOWNTO 12) >= X"D" ELSE -- 4K	
		vcnt(7 DOWNTO 0) WHEN addr_bus(15 DOWNTO 4) = X"CBE" ELSE
		map_do WHEN addr_bus(15 DOWNTO 11) = X"C" & '0' ELSE
		x"0" & cmos_do WHEN addr_bus(15 DOWNTO 10) = X"C" & "11" ELSE
		pia_io1_do WHEN pia_io1_cs = '1' ELSE
		pia_io2_do WHEN pia_io2_cs = '1' ELSE
		X"00";

	data_bus_low <=
		palette_lo_do WHEN color_cs = '1' AND addr_bus(0) = '0' ELSE
		palette_hi_do WHEN color_cs = '1' AND addr_bus(0) = '1' ELSE
		rom_do WHEN rom_bank_cs = '1' ELSE
		--	rom_bank_a_do           when rom_bank_cs = '1' and (page = "010"                ) else
		--	rom_bank_b_do           when rom_bank_cs = '1' and (page = "110"                ) else
		--	rom_bank_c_do           when rom_bank_cs = '1' and (page = "001" or page = "011") else
		--	rom_bank_d_do           when rom_bank_cs = '1' and (page = "100" or page = "101") else
		vram_h0_do & vram_l0_do WHEN decod_do(7 DOWNTO 6) = "00" ELSE
		vram_h1_do & vram_l1_do WHEN decod_do(7 DOWNTO 6) = "01" ELSE
		vram_h2_do & vram_l2_do WHEN decod_do(7 DOWNTO 6) = "10" ELSE
		X"00";

	data_bus <=
		cpu_do WHEN cpu_rw_n = '0' ELSE
		blit_data WHEN blit_rw_n = '0' ELSE
		data_bus_low WHEN addr_bus(15 DOWNTO 12) < x"C" ELSE
		data_bus_high;

	PROCESS (clock_12)
	BEGIN
		IF rising_edge(clock_12) THEN
			cpu_di <= data_bus;
		END IF;
	END PROCESS;

	-- misc. registers
	PROCESS (reset, clock_12)
		VARIABLE blit_h, blit_l : STD_LOGIC;
		VARIABLE data : STD_LOGIC_VECTOR(7 DOWNTO 0);
	BEGIN
		IF reset = '1' THEN
			page <= "000";
			seven_seg <= X"00";
			flip <= '0';
			dma_inh_n <= '0';
			fg_color_bank <= (OTHERS => '0');
			bg_color_bank <= (OTHERS => '0');
		ELSE
			IF rising_edge(clock_12) THEN
				IF page_cs = '1' AND we_bus = '1' THEN
					page <= data_bus(2 DOWNTO 0);
				END IF;
				IF seven_seg_cs = '1' AND we_bus = '1' THEN
					seven_seg <= data_bus;
				END IF;
				IF flip_cs = '1' AND we_bus = '1' THEN
					flip <= data_bus(0);
				END IF;
				IF dma_inh_cs = '1' AND we_bus = '1' THEN
					dma_inh_n <= data_bus(0);
				END IF;
				IF fg_color_bank_cs = '1' AND we_bus = '1' THEN
					fg_color_bank <= data_bus(5 DOWNTO 0);
				END IF;
				IF bg_color_bank_cs = '1' AND we_bus = '1' THEN
					bg_color_bank <= data_bus(5 DOWNTO 0);
				END IF;
				IF xscroll_low_cs = '1' AND we_bus = '1' THEN
					xscroll(3 DOWNTO 0) <= data_bus(7) & data_bus(2 DOWNTO 0);
				END IF;
				IF xscroll_high_cs = '1' AND we_bus = '1' THEN
					xscroll(11 DOWNTO 4) <= data_bus;
				END IF;
			END IF;
		END IF;
	END PROCESS;

	-- blitter registers
	PROCESS (reset, clock_12)
		VARIABLE blit_h, blit_l : STD_LOGIC;
		VARIABLE data : STD_LOGIC_VECTOR(7 DOWNTO 0);
	BEGIN
		IF reset = '1' THEN
			blit_start <= '0';
			blit_cmd <= (OTHERS => '0');
			blit_color <= (OTHERS => '0');
			blit_src <= (OTHERS => '0');
			blit_dst <= (OTHERS => '0');
			blit_width <= (OTHERS => '0');
			blit_height <= (OTHERS => '0');
		ELSE
			IF rising_edge(clock_12) THEN
				IF blit_cs = '1' AND we_bus = '1' THEN
					CASE addr_bus(2 DOWNTO 0) IS
						WHEN "000" => blit_cmd <= data_bus;
							blit_start <= '1';
						WHEN "001" => blit_color <= data_bus;
						WHEN "010" => blit_src(15 DOWNTO 8) <= data_bus;
						WHEN "011" => blit_src(7 DOWNTO 0) <= data_bus;
						WHEN "100" => blit_dst(15 DOWNTO 8) <= data_bus;
						WHEN "101" => blit_dst(7 DOWNTO 0) <= data_bus;
						WHEN "110" =>
							IF data_bus = X"00" THEN
								blit_width <= x"00";
							ELSE
								blit_width <= data_bus - 1;
							END IF;

						WHEN "111" =>
							IF data_bus = X"00" THEN
								blit_height <= x"00";
							ELSE
								blit_height <= data_bus - 1;
							END IF;
						WHEN OTHERS => NULL;
					END CASE;
				END IF;

				IF blit_halt = '1' THEN
					blit_start <= '0';
				END IF;

			END IF;
		END IF;
	END PROCESS;

	-- blitter - IC29-30
	PROCESS (reset, clock_12)
		VARIABLE blit_h, blit_l : STD_LOGIC;
		VARIABLE data : STD_LOGIC_VECTOR(7 DOWNTO 0);
	BEGIN
		IF reset = '1' THEN
			blit_rw_n <= '1';
			cpu_halt <= '0';
			blit_halt <= '0';
			blit_wr_inh_h <= '0';
			blit_wr_inh_l <= '0';
		ELSE
			IF rising_edge(clock_12) THEN

				-- sync cpu_halt in the middle of cpu cycle
				IF video_access = '1' THEN
					cpu_halt <= blit_halt;
				END IF;

				--	intialize blit
				IF blit_start = '1' AND blit_halt = '0' AND video_access = '1' THEN
					blit_halt <= '1';
					blit_cur_src <= blit_src;
					blit_src_ori <= blit_src;
					blit_cur_dst <= blit_dst;
					blit_dst_ori <= blit_dst;
					blit_cur_width <= blit_width;
					blit_cur_height <= blit_height;
					right_nibble <= x"0";
					blit_go <= '1';
					-- begin with read step
					blit_addr <= blit_src;
					blit_rw_n <= '1';
				END IF;

				-- do blit
				IF blit_has_bus = '1' THEN

					-- read step (use graph access)			
					IF graph_access = '1' AND en_pixel = '0' AND blit_go = '1' THEN

						-- next step will be write
						blit_addr <= blit_cur_dst;
						blit_rw_n <= '0';

						-- also prepare next source address w.r.t source stride
						IF blit_cmd(0) = '0' THEN
							blit_cur_src <= blit_cur_src + 1;
						ELSE
							IF blit_cur_width = 0 THEN
								blit_cur_src <= blit_src_ori + 1;
								blit_src_ori <= blit_src_ori + 1;
							ELSE
								blit_cur_src <= blit_cur_src + 256;
							END IF;
						END IF;

						-- get data from source and prepare data to be written
						blit_h := NOT blit_cmd(7);
						blit_l := NOT blit_cmd(6);

						-- right shift mode
						IF blit_cmd(5) = '0' THEN
							data := data_bus;
						ELSE
							data := right_nibble & data_bus(7 DOWNTO 4);
							right_nibble <= data_bus(3 DOWNTO 0);
						END IF;

						-- transparent mode : don't write pixel if src = 0
						IF blit_cmd(3) = '1' THEN
							IF data(7 DOWNTO 4) = x"0" THEN
								blit_h := '0';
							END IF;
							IF data(3 DOWNTO 0) = x"0" THEN
								blit_l := '0';
							END IF;
						END IF;

						-- solid mode : write color instead of src data
						IF blit_cmd(4) = '1' THEN
							data := blit_color;
						ELSE
							data := data;
						END IF;

						-- put data to written on bus with write inhibits
						blit_data <= data;
						blit_wr_inh_h <= NOT blit_h;
						blit_wr_inh_l <= NOT blit_l;

					END IF;

					-- write step (use cpu access)
					IF en_cpu = '1' AND en_pixel = '0' AND blit_go = '1' THEN
						-- next step will be read
						blit_addr <= blit_cur_src;
						blit_rw_n <= '1';

						-- also prepare next destination address w.r.t destination stride
						-- or stop blit
						IF blit_cur_width = 0 THEN
							IF blit_cur_height = 0 THEN
								-- end of blit
								blit_halt <= '0';
								blit_wr_inh_h <= '0';
								blit_wr_inh_l <= '0';
							ELSE
								blit_cur_width <= blit_width;
								blit_cur_height <= blit_cur_height - 1;

								IF blit_cmd(1) = '0' THEN
									blit_cur_dst <= blit_cur_dst + 1;
								ELSE
									blit_cur_dst <= blit_dst_ori + 1;
									blit_dst_ori <= blit_dst_ori + 1;
								END IF;

							END IF;
						ELSE
							blit_cur_width <= blit_cur_width - 1;

							IF blit_cmd(1) = '0' THEN
								blit_cur_dst <= blit_cur_dst + 1;
							ELSE
								blit_cur_dst <= blit_cur_dst + 256;
							END IF;
						END IF;
					END IF;

					-- slow mode
					IF en_cpu = '1' AND en_pixel = '0' AND blit_cmd(2) = '1' THEN
						blit_go <= NOT blit_go;
					END IF;

				END IF; -- cpu halted	
			END IF;
		END IF;
	END PROCESS;

	-- microprocessor 6809 -IC28
	main_cpu : ENTITY work.cpu09
		PORT MAP(
			clk => en_cpu, -- E clock input (falling edge)
			rst => reset, -- reset input (active high)
			vma => OPEN, -- valid memory address (active high)
			lic_out => OPEN, -- last instruction cycle (active high)
			ifetch => OPEN, -- instruction fetch cycle (active high)
			opfetch => OPEN, -- opcode fetch (active high)
			ba => cpu_ba, -- bus available (high on sync wait or DMA grant)
			bs => cpu_bs, -- bus status (high on interrupt or reset vector fetch or DMA grant)
			addr => cpu_addr, -- address bus output
			rw => cpu_rw_n, -- read not write output
			data_out => cpu_do, -- data bus output
			data_in => cpu_di, -- data bus input
			irq => cpu_irq, -- interrupt request input (active high)
			firq => '0', -- fast interrupt request input (active high)
			nmi => '0', -- non maskable interrupt request input (active high)
			halt => cpu_halt, -- halt input (active high) grants DMA
			hold => '0' -- hold input (active high) extend bus cycle
		);

	-- cpu program roms - IC9-10-54
	prog1_rom : ENTITY work.joust2_prog1
		PORT MAP(
			clk => clock_12,
			addr => addr_bus(11 DOWNTO 0),
			data => rom_prog1_do
		);

	prog2_rom : ENTITY work.joust2_prog2
		PORT MAP(
			clk => clock_12,
			addr => addr_bus(12 DOWNTO 0),
			data => rom_prog2_do
		);
	-- rom17.ic26 + rom15.ic24 
	bank_a_rom : ENTITY work.joust2_bank_a
		PORT MAP(
			clk => clock_12,
			addr => addr_bus(14 DOWNTO 0),
			data => rom_bank_a_do
		);

	-- rom16.ic25 + rom14.ic23 + rom13.ic21 + rom12.ic19 
	bank_b_rom : ENTITY work.joust2_bank_b
		PORT MAP(
			clk => clock_12,
			addr => addr_bus(14 DOWNTO 0),
			data => rom_bank_b_do
		);

	-- rom11.ic18 + rom9.ic16 + rom7.ic14 + rom5.ic12 
	bank_c_rom : ENTITY work.joust2_bank_c
		PORT MAP(
			clk => clock_12,
			addr => addr_bus(14 DOWNTO 0),
			data => rom_bank_c_do
		);

	-- rom10.ic17 + rom8.ic15 + rom6.ic13 + rom4.ic11
	bank_d_rom : ENTITY work.joust2_bank_d
		PORT MAP(
			clk => clock_12,
			addr => addr_bus(14 DOWNTO 0),
			data => rom_bank_d_do
		);

	-- rom20.ic57
	graph1_rom : ENTITY work.joust2_graph1
		PORT MAP(
			clk => clock_12,
			addr => graph_addr,
			data => graph1_do
		);

	-- rom20.ic58
	graph2_rom : ENTITY work.joust2_graph2
		PORT MAP(
			clk => clock_12,
			addr => graph_addr,
			data => graph2_do
		);

	-- rom20.ic41
	graph3_rom : ENTITY work.joust2_graph3
		PORT MAP(
			clk => clock_12,
			addr => graph_addr,
			data => graph3_do
		);

	-- cpu/video wram low 0 - IC102-105
	cpu_video_ram_l0 : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 4, aWidth => 14)
		PORT MAP(
			clk => clock_12,
			we => vram_l0_we,
			addr => vram_addr,
			d => data_bus(3 DOWNTO 0),
			q => vram_l0_do
		);

	-- cpu/video wram high 0 - IC98-101
	cpu_video_ram_h0 : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 4, aWidth => 14)
		PORT MAP(
			clk => clock_12,
			we => vram_h0_we,
			addr => vram_addr,
			d => data_bus(7 DOWNTO 4),
			q => vram_h0_do
		);

	-- cpu/video wram low 1 - IC110-113
	cpu_video_ram_l1 : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 4, aWidth => 14)
		PORT MAP(
			clk => clock_12,
			we => vram_l1_we,
			addr => vram_addr,
			d => data_bus(3 DOWNTO 0),
			q => vram_l1_do
		);

	-- cpu/video wram high 1 - IC106-109
	cpu_video_ram_h1 : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 4, aWidth => 14)
		PORT MAP(
			clk => clock_12,
			we => vram_h1_we,
			addr => vram_addr,
			d => data_bus(7 DOWNTO 4),
			q => vram_h1_do
		);

	-- cpu/video wram low 2 - IC118-121
	cpu_video_ram_l2 : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 4, aWidth => 14)
		PORT MAP(
			clk => clock_12,
			we => vram_l2_we,
			addr => vram_addr,
			d => data_bus(3 DOWNTO 0),
			q => vram_l2_do
		);

	-- cpu/video wram high 2 - IC115-117
	cpu_video_ram_h2 : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 4, aWidth => 14)
		PORT MAP(
			clk => clock_12,
			we => vram_h2_we,
			addr => vram_addr,
			d => data_bus(7 DOWNTO 4),
			q => vram_h2_do
		);
	-- palette rams - IC78-77
	palette_ram_lo : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 8, aWidth => 10)
		PORT MAP(
			clk => clock_12,
			we => palette_lo_we,
			addr => palette_addr,
			d => data_bus,
			q => palette_lo_do
		);

	-- palette rams - IC76-75
	palette_ram_hi : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 8, aWidth => 10)
		PORT MAP(
			clk => clock_12,
			we => palette_hi_we,
			addr => palette_addr,
			d => data_bus,
			q => palette_hi_do
		);
	-- map ram - IC40
	map_ram : ENTITY work.gen_ram
		GENERIC MAP(dWidth => 8, aWidth => 11)
		PORT MAP(
			clk => clock_12,
			we => map_we,
			addr => map_addr,
			d => data_bus,
			q => map_do
		);

	--sram0 : entity work.gen_ram
	--generic map( dWidth => 8, aWidth => 11)
	--port map(
	-- clk  => clock_12,
	-- we   => sram0_we,
	-- addr => addr_bus(10 downto 0),
	-- d    => data_bus,
	-- q    => sram0_do
	--);

	-- cmos ram - IC59
	cmos_ram : ENTITY work.joust2_cmos_ram
		GENERIC MAP(dWidth => 4, aWidth => 10)
		PORT MAP(
			clk => clock_12,
			we => cmos_we,
			addr => addr_bus(9 DOWNTO 0),
			d => data_bus(3 DOWNTO 0),
			q => cmos_do
		);

	-- addr bus to video addr decoder - IC60
	video_addr_decoder : ENTITY work.joust2_decoder
		PORT MAP(
			clk => clock_12,
			addr => decod_addr,
			data => decod_do
		);

	-- gun gray code encoder
	--gun_gray_encoder : entity work.gray_code
	--port map(
	-- clk  => clock_12,
	-- addr => gun_bin_code,
	-- data => gun_gray_code
	--);
	-- pia iO1 : ic6 (5C)
	pia_io1 : ENTITY work.pia6821
		PORT MAP
		(
			clk => pias_clock, -- rising edge
			rst => reset, -- active high
			cs => pia_io1_cs,
			rw => cpu_rw_n, -- write low
			addr => addr_bus(1 DOWNTO 0),
			data_in => cpu_do,
			data_out => pia_io1_do,
			irqa => pia_io1_irqa, -- active high
			irqb => pia_io1_irqb, -- active high
			pa_i => pia_io1_pa_i,
			pa_o => OPEN,
			pa_oe => OPEN,
			ca1 => vcnt_240,
			ca2_i => '0',
			ca2_o => pia_io1_ca2_o,
			ca2_oe => OPEN,
			pb_i => pia_io1_pb_i,
			pb_o => OPEN,
			pb_oe => OPEN,
			cb1 => cnt_4ms,
			cb2_i => '0',
			cb2_o => OPEN,
			cb2_oe => OPEN
		);

	-- pia iO2 : ic5 (2C)
	pia_rom : ENTITY work.pia6821
		PORT MAP
		(
			clk => pias_clock,
			rst => reset,
			cs => pia_io2_cs,
			rw => cpu_rw_n,
			addr => addr_bus(1 DOWNTO 0),
			data_in => cpu_do,
			data_out => pia_io2_do,
			irqa => pia_io2_irqa,
			irqb => pia_io2_irqb,
			pa_i => pia_io2_pa_i,
			pa_o => OPEN,
			pa_oe => OPEN,
			ca1 => cnt_4ms, -- '0',
			ca2_i => '0',
			ca2_o => OPEN,
			ca2_oe => OPEN,
			pb_i => (OTHERS => '0'),
			pb_o => sound_select,
			pb_oe => OPEN,
			cb1 => sound_ack,
			cb2_i => '0',
			cb2_o => sound_trig,
			cb2_oe => OPEN
		);

	-- video syncs and blanks
	video_csync <= csync;

	PROCESS (clock_12)
		CONSTANT hcnt_base : INTEGER := 52;
		VARIABLE vsync_cnt : STD_LOGIC_VECTOR(3 DOWNTO 0);
	BEGIN

		IF rising_edge(clock_12) THEN

			IF hcnt = hcnt_base + 0 THEN
				hsync0 <= '0';
			ELSIF hcnt = hcnt_base + 6 THEN
				hsync0 <= '1';
			END IF;

			IF hcnt = hcnt_base + 0 THEN
				hsync1 <= '0';
			ELSIF hcnt = hcnt_base + 3 THEN
				hsync1 <= '1';
			ELSIF hcnt = hcnt_base + 32 - 64 THEN
				hsync1 <= '0';
			ELSIF hcnt = hcnt_base + 35 - 64 THEN
				hsync1 <= '1';
			END IF;

			IF hcnt = hcnt_base + 0 THEN
				hsync2 <= '0';
			ELSIF hcnt = hcnt_base + 32 - 3 - 64 THEN
				hsync2 <= '1';
			ELSIF hcnt = hcnt_base + 32 - 64 THEN
				hsync2 <= '0';
			ELSIF hcnt = hcnt_base + 64 - 3 - 128 THEN
				hsync2 <= '1';
			END IF;

			IF hcnt = 63 AND pixel_cnt = 5 THEN
				IF vcnt = 502 THEN
					vsync_cnt := X"0";
				ELSE
					IF vsync_cnt < X"F" THEN
						vsync_cnt := vsync_cnt + '1';
					END IF;
				END IF;
			END IF;

			IF vsync_cnt = 0 THEN
				csync <= hsync1;
			ELSIF vsync_cnt = 1 THEN
				csync <= hsync1;
			ELSIF vsync_cnt = 2 THEN
				csync <= hsync1;
			ELSIF vsync_cnt = 3 THEN
				csync <= hsync2;
			ELSIF vsync_cnt = 4 THEN
				csync <= hsync2;
			ELSIF vsync_cnt = 5 THEN
				csync <= hsync2;
			ELSIF vsync_cnt = 6 THEN
				csync <= hsync1;
			ELSIF vsync_cnt = 7 THEN
				csync <= hsync1;
			ELSIF vsync_cnt = 8 THEN
				csync <= hsync1;
			ELSE
				csync <= hsync0;
			END IF;

			IF hcnt = 48 AND pixel_cnt = 3 THEN
				hblank <= '1';
			ELSIF hcnt = 1 AND pixel_cnt = 3 THEN
				hblank <= '0';
			END IF;

			IF vcnt = 504 THEN
				vblank <= '1';
			ELSIF vcnt = 262 THEN
				vblank <= '0';
			END IF;

			-- external sync and blank outputs
			video_blankn <= NOT (hblank OR vblank);

			video_hs <= hsync0;

			IF vsync_cnt = 0 THEN
				video_vs <= '0';
			ELSIF vsync_cnt = 8 THEN
				video_vs <= '1';
			END IF;

		END IF;
	END PROCESS;

	-- sound board - IC4-7-8-27
	joust2_sound_board : ENTITY work.joust2_sound_board
		PORT MAP(
			clock_12 => clock_12,
			reset => reset,

			sound_select => sound_select,
			sound_trig => sound_trig,
			sound_ack => sound_ack,
			audio_out => audio_out,

			dbg_cpu_addr => sound_cpu_addr
		);

END struct;