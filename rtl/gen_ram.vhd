-- -----------------------------------------------------------------------
--
-- Syntiac's generic VHDL support files.
--
-- -----------------------------------------------------------------------
-- Copyright 2005-2008 by Peter Wendrich (pwsoft@syntiac.com)
-- http://www.syntiac.com/fpga64.html
--
-- Modified April 2016 by Dar (darfpga@aol.fr) 
-- http://darfpga.blogspot.fr
--   Remove address register when writing
--
-- -----------------------------------------------------------------------
--
-- gen_rwram.vhd
--
-- -----------------------------------------------------------------------
--
-- generic ram.
--
-- -----------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;

-- -----------------------------------------------------------------------

ENTITY gen_ram IS
	GENERIC (
		dWidth : INTEGER := 8;
		aWidth : INTEGER := 10
	);
	PORT (
		clk : IN STD_LOGIC;
		we : IN STD_LOGIC;
		addr : IN STD_LOGIC_VECTOR((aWidth - 1) DOWNTO 0);
		d : IN STD_LOGIC_VECTOR((dWidth - 1) DOWNTO 0);
		q : OUT STD_LOGIC_VECTOR((dWidth - 1) DOWNTO 0)
	);
END ENTITY;

-- -----------------------------------------------------------------------

ARCHITECTURE rtl OF gen_ram IS
	SUBTYPE addressRange IS INTEGER RANGE 0 TO ((2 ** aWidth) - 1);
	TYPE ramDef IS ARRAY(addressRange) OF STD_LOGIC_VECTOR((dWidth - 1) DOWNTO 0);
	SIGNAL ram : ramDef;

	SIGNAL rAddrReg : STD_LOGIC_VECTOR((aWidth - 1) DOWNTO 0);
	SIGNAL qReg : STD_LOGIC_VECTOR((dWidth - 1) DOWNTO 0);
BEGIN
	-- -----------------------------------------------------------------------
	-- Signals to entity interface
	-- -----------------------------------------------------------------------
	--	q <= qReg;

	-- -----------------------------------------------------------------------
	-- Memory write
	-- -----------------------------------------------------------------------
	PROCESS (clk)
	BEGIN
		IF rising_edge(clk) THEN
			IF we = '1' THEN
				ram(to_integer(unsigned(addr))) <= d;
			END IF;
		END IF;
	END PROCESS;

	-- -----------------------------------------------------------------------
	-- Memory read
	-- -----------------------------------------------------------------------
	PROCESS (clk)
	BEGIN
		IF rising_edge(clk) THEN
			--			qReg <= ram(to_integer(unsigned(rAddrReg)));
			--			rAddrReg <= addr;
			----			qReg <= ram(to_integer(unsigned(addr)));
			q <= ram(to_integer(unsigned(addr)));
		END IF;
	END PROCESS;
	--q <= ram(to_integer(unsigned(addr)));
END ARCHITECTURE;