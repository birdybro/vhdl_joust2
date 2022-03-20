-- -----------------------------------------------------------------------
--
--                                 FPGA 64
--
--     A fully functional commodore 64 implementation in a single FPGA
--
-- -----------------------------------------------------------------------
-- Copyright 2005-2008 by Peter Wendrich (pwsoft@syntiac.com)
-- http://www.syntiac.com/fpga64.html
-- -----------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;

ENTITY io_ps2_keyboard IS
	PORT (
		clk : IN STD_LOGIC;
		kbd_clk : IN STD_LOGIC;
		kbd_dat : IN STD_LOGIC;

		interrupt : OUT STD_LOGIC;
		scanCode : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END io_ps2_keyboard;

ARCHITECTURE Behavioral OF io_ps2_keyboard IS
	SIGNAL clk_reg : STD_LOGIC;
	SIGNAL clk_waitNextBit : STD_LOGIC;
	SIGNAL clk_filter : INTEGER RANGE 0 TO 15;
	SIGNAL shift_reg : STD_LOGIC_VECTOR(10 DOWNTO 0) := (OTHERS => '0');

	SIGNAL bitsCount : INTEGER RANGE 0 TO 10 := 0;
	SIGNAL timeout : INTEGER RANGE 0 TO 5000 := 0; -- 2* 50 us at 50 Mhz
BEGIN
	PROCESS (clk)
	BEGIN
		IF rising_edge(clk) THEN
			-- Interrupt is edge triggered. Only 1 clock high.
			interrupt <= '0';

			-- Timeout if keyboard does not send anymore.
			IF timeout /= 0 THEN
				timeout <= timeout - 1;
			ELSE
				bitsCount <= 0;
			END IF;

			-- Filter glitches on the clock
			IF (clk_reg /= kbd_clk) THEN
				clk_filter <= 15; -- Wait 15 ticks 
				clk_reg <= kbd_clk; -- Store clock edge to detect changes
				clk_waitNextBit <= '0'; -- Next bit comming up...
			ELSIF (clk_filter /= 0) THEN
				-- Wait for clock to stabilise
				-- Clock must be stable before we sample the data line.
				clk_filter <= clk_filter - 1;
			ELSIF (clk_reg = '1') AND (clk_waitNextBit = '0') THEN
				-- We have a stable clock, so assume stable data too.
				clk_waitNextBit <= '1';

				-- Move data into shift register
				shift_reg <= kbd_dat & shift_reg(10 DOWNTO 1);
				timeout <= 5000;
				IF bitsCount < 10 THEN
					bitsCount <= bitsCount + 1;
				ELSE
					-- 10 bits received. Output new scancode
					bitsCount <= 0;
					interrupt <= '1';
					scanCode <= shift_reg(9 DOWNTO 2);
				END IF;
			END IF;
		END IF;
	END PROCESS;

END Behavioral;