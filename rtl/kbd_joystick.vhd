LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.numeric_std.ALL;

ENTITY Kbd_Joystick IS
  PORT (
    Clk : IN STD_LOGIC;
    KbdInt : IN STD_LOGIC;
    KbdScanCode : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    JoyHBCPPFRLDU : OUT STD_LOGIC_VECTOR(9 DOWNTO 0);
    Keys_HUA : OUT STD_LOGIC_VECTOR(2 DOWNTO 0)
  );
END Kbd_Joystick;

ARCHITECTURE Behavioral OF Kbd_Joystick IS

  SIGNAL IsReleased : STD_LOGIC;

BEGIN

  PROCESS (Clk)
  BEGIN
    IF rising_edge(Clk) THEN

      IF KbdInt = '1' THEN
        IF KbdScanCode = "11110000" THEN
          IsReleased <= '1';
        ELSE
          IsReleased <= '0';
        END IF;
        IF KbdScanCode = "01110101" THEN
          JoyHBCPPFRLDU(0) <= NOT(IsReleased);
        END IF; -- up    arrow : 0x75
        IF KbdScanCode = "01110010" THEN
          JoyHBCPPFRLDU(1) <= NOT(IsReleased);
        END IF; -- down  arrow : 0x72
        IF KbdScanCode = "01101011" THEN
          JoyHBCPPFRLDU(2) <= NOT(IsReleased);
        END IF; -- left  arrow : 0x6B
        IF KbdScanCode = "01110100" THEN
          JoyHBCPPFRLDU(3) <= NOT(IsReleased);
        END IF; -- right arrow : 0x74
        IF KbdScanCode = "00101001" THEN
          JoyHBCPPFRLDU(4) <= NOT(IsReleased);
        END IF; -- space : 0x29
        IF KbdScanCode = "00000101" THEN
          JoyHBCPPFRLDU(5) <= NOT(IsReleased);
        END IF; -- F1 : 0x05
        IF KbdScanCode = "00000110" THEN
          JoyHBCPPFRLDU(6) <= NOT(IsReleased);
        END IF; -- F2 : 0x06
        IF KbdScanCode = "00000100" THEN
          JoyHBCPPFRLDU(7) <= NOT(IsReleased);
        END IF; -- F3 : 0x04
        IF KbdScanCode = "00010100" THEN
          JoyHBCPPFRLDU(8) <= NOT(IsReleased);
        END IF; -- ctrl : 0x14
        IF KbdScanCode = "00011101" THEN
          JoyHBCPPFRLDU(9) <= NOT(IsReleased);
        END IF; -- W    : 0x1D
        IF KbdScanCode = "00011100" THEN
          Keys_HUA(0) <= NOT(IsReleased);
        END IF; -- A    : 0x1C
        IF KbdScanCode = "00111100" THEN
          Keys_HUA(1) <= NOT(IsReleased);
        END IF; -- U    : 0x3C			
        IF KbdScanCode = "00110011" THEN
          Keys_HUA(2) <= NOT(IsReleased);
        END IF; -- H    : 0x33			
      END IF;

    END IF;
  END PROCESS;

END Behavioral;