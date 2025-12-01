----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09/19/2023 09:57:49 PM
-- Design Name: 
-- Module Name: Top - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_unsigned.ALL;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

-- import the string memory package
use work.preset_memory_pkg.all;

-- Assignment of Pins
entity GPIO_demo is
    Port ( SW 			: in  STD_LOGIC_VECTOR (15 downto 0);
           BTN 			: in  STD_LOGIC_VECTOR (4 downto 0);
           CLK 			: in  STD_LOGIC;
           LED 			: out  STD_LOGIC_VECTOR (15 downto 0);
           SSEG_CA 		: out  STD_LOGIC_VECTOR (7 downto 0);
           SSEG_AN 		: out  STD_LOGIC_VECTOR (7 downto 0);
           UART_TXD 	: out  STD_LOGIC;
           RGB1_Red		: out  STD_LOGIC;
           RGB1_Green	: out  STD_LOGIC;
           RGB1_Blue	: out  STD_LOGIC;	
           RGB2_Red		: out  STD_LOGIC;
           RGB2_Green	: out  STD_LOGIC;
           RGB2_Blue	: out  STD_LOGIC; -- assigned to JA pins
           ACL_MISO  : IN     STD_LOGIC; 
           ACL_MOSI  : OUT    STD_LOGIC;  
           ACL_SCLK : OUT    STD_LOGIC; 
           ACL_CSN : OUT    STD_LOGIC;
           JB : OUT  STD_LOGIC_VECTOR(8 downto 1);
           JC : OUT    STD_LOGIC_VECTOR(8 downto 1);
           JD : OUT    STD_LOGIC_VECTOR(8 downto 1)
			  );
end GPIO_demo;

architecture Behavioral of GPIO_demo is

component spi_master  
    PORT(
        clk     : IN     STD_LOGIC;                             --system clock
        reset_n : IN     STD_LOGIC;                             --asynchronous active low reset
        enable  : IN     STD_LOGIC;                             --initiate communication
        cpol    : IN     STD_LOGIC;  									--clock polarity mode
        cpha    : IN     STD_LOGIC;  									--clock phase mode
        miso    : IN     STD_LOGIC;                             --master in slave out
        sclk    : OUT    STD_LOGIC;                             --spi clock
        ss_n    : OUT    STD_LOGIC;                             --slave select
        mosi    : OUT    STD_LOGIC;                             --master out slave in
        busy    : OUT    STD_LOGIC;                             --master busy signal
        tx		: IN     STD_LOGIC_VECTOR(8*8-1 DOWNTO 0);  --data to transmit
        rx	   : OUT    STD_LOGIC_VECTOR(8*8-1 DOWNTO 0)); --data received
end component;

component UART_TX_CTRL
Port(
	SEND : in std_logic;
	DATA : in std_logic_vector(7 downto 0);
	CLK : in std_logic;          
	READY : out std_logic;
	UART_TX : out std_logic
	);
end component;

component debouncer
Generic(
        DEBNC_CLOCKS : integer;
        PORT_WIDTH : integer);
Port(
		SIGNAL_I : in std_logic_vector(4 downto 0);
		CLK_I : in std_logic;          
		SIGNAL_O : out std_logic_vector(4 downto 0)
		);
end component;

component RGB_controller 
Port(
	GCLK 			: in std_logic;
	RGB_LED_1_O	: out std_logic_vector(2 downto 0);
	RGB_LED_2_O	: out std_logic_vector(2 downto 0)
	);
end component;

type UART_STATE_TYPE is (RST_REG, LD_INIT_STR, SEND_CHAR, RDY_LOW, WAIT_RDY, WAIT_BTN, LD_BTN_STR);
type SPI_STATE_TYPE is (RST_REG, SPI_START, SPI_BUSY, SPI_RDY);
type DEVICE_STATE_TYPE is (RST_DEV, TX, RX, IDLE, DEVICE_ID);

--Contains the current string being sent over uart.
signal sendStr : CHAR_ARRAY(0 to (MAX_STR_LEN - 1));

--Contains the length of the current string being sent over uart.
signal strEnd : natural;

--Contains the index of the next character to be sent over uart
--within the sendStr variable.
signal strIndex : natural;

--Used to determine when a button press has occured
signal btnReg : std_logic_vector (3 downto 0) := (others => '0');
signal btnDetect : std_logic;

--UART_TX_CTRL control signals
signal uartRdy : std_logic;
signal uartSend : std_logic := '0';
signal uartData : std_logic_vector (7 downto 0):= (others => '0');
signal uartTX : std_logic;
signal uartState : UART_STATE_TYPE := RST_REG; --Current uart state signal

signal spiState : SPI_STATE_TYPE := RST_REG; --Current uart state signal
signal prescaler_A: STD_LOGIC_VECTOR(23 downto 0) := "000000000000000001111011"; 
signal prescaler_counter_A: STD_LOGIC_VECTOR(23 downto 0) := (others => '0'); 
signal prescaler_B: STD_LOGIC_VECTOR(23 downto 0) := "000000000000000011111000"; 
signal prescaler_counter_B: STD_LOGIC_VECTOR(23 downto 0) := (others => '0'); 
signal prescaler_C: STD_LOGIC_VECTOR(23 downto 0) := "000000000000000000001111"; 
signal prescaler_counter_C: STD_LOGIC_VECTOR(23 downto 0) := (others => '0');
signal Device_State : DEVICE_STATE_TYPE := RST_DEV; --Current magnet position signal
signal Device_Prev_State : DEVICE_STATE_TYPE := RST_DEV; --Current magnet position signal

signal btnDeBnc : std_logic_vector(4 downto 0); --Debounced btn signals 

signal clk_cntr_reg : std_logic_vector (4 downto 0) := (others=>'0'); 

--this counter counts the amount of time paused in the UART reset state
signal reset_cntr : std_logic_vector (17 downto 0) := (others=>'0');

signal busy_master, enable, spi_trigger_clk, spi_clk :STD_LOGIC := '0';

-- Data to be transfered by master and by slave:
signal tx_master : std_logic_vector(DATA_Size_SPI downto 0) := (others => '0');
signal rx_master : std_logic_vector(DATA_Size_SPI downto 0) := (others => '0');
signal regaddr   : std_logic_vector(6 downto 0) := (others => '0');
signal data_in   : std_logic_vector(47 downto 0) := (others => '0');

begin
-----------------------------------------------------------------
-- UART Decode Process
-----------------------------------------------------------------
DEV_fsm :process(btnDetect)
begin
    if btnDetect = '1' then
        regaddr <= (others => '0');
        data_in <= (others => '0');
    end if;

    if uartRdy = '1' then
        case uartData is
            ------------------------------------------------
            -- ASCII "1" = 0x31
            ------------------------------------------------
            when X"31" =>
                regaddr <= ADDR_1;
                data_in <= DATA_1;

            ------------------------------------------------
            -- ASCII "2" = 0x32
            ------------------------------------------------
            when X"32" =>
                regaddr <= ADDR_2;
                data_in <= DATA_2;

            ------------------------------------------------
            -- ASCII "3" = 0x33
            ------------------------------------------------
            when X"33" =>
                regaddr <= ADDR_3;
                data_in <= DATA_3;

            ------------------------------------------------
            -- ASCII "4" = 0x34
            ------------------------------------------------
            when X"34" =>
                regaddr <= ADDR_4;
                data_in <= DATA_4;

            ------------------------------------------------
            -- Default: do nothing
            ------------------------------------------------
            when others =>
                regaddr <= (others => '0');
                data_in <= (others => '0');
        end case;
    end if;
end process;

-- -------------------------------------------------------------------------
--  SPI MASTER
-- -------------------------------------------------------------------------
AWMF_CTRL : spi_master PORT MAP (
		clk => spi_clk,
		reset_n => '1',
		enable => enable,
		cpol => '0',
		cpha => '1',
		miso => ACL_MISO,
		sclk => ACL_SCLK,
		ss_n => ACL_CSN,
		mosi => ACL_MOSI,
		busy => busy_master,
		tx => tx_master,
		rx => rx_master);

SPI_fsm : PROCESS(clk, btnDetect, spi_clk)
    begin
        if btnDetect = '1' then
            Device_State <= RST_DEV;
            tx_master <= (others => '0');
        end if;    

        if rising_edge(clk) then
            -- Device Handler
            case Device_State is
                -- RESET: Initialize registers
                when RST_DEV =>
                    regaddr   <= (others => '0');
                    data_in   <= (others => '0');
                    tx_master <= (others => '0');
                    Device_State <= IDLE;

                -- TX: Populate tx_master with addr + data
                when TX =>
                    -- Format:
                    -- [63:60] = "0000"
                    -- [59:53] = regaddr(6 downto 0)
                    -- [52:5]  = data_in(47 downto 0)
                    -- [4:0]   = "00000" (padding)
                    tx_master <= (others => '0');
                    tx_master(59 downto 53) <= regaddr;
                    tx_master(52 downto 5)  <= data_in;
                    -- Lower 5 bits remain "00000"
                    Device_State <= RX;

                -- RX: Capture response from slave
                when RX =>
                    rx_master <= tx_master;  -- Replace with MISO shift logic
                    Device_State <= IDLE;

                -- DEVICE_ID: Example read sequence
                when DEVICE_ID =>
                    regaddr   <= "0000001";  -- Example register for ID
                    data_in   <= (others => '0');
                    Device_State <= TX;

                -- IDLE: Wait for external trigger
                when IDLE =>
                    -- Do nothing, wait for enable
                    null;

                -- Safe fallback
                when others =>
                    Device_State <= RST_DEV;
            end case;
    
            -- Clock Handler
            prescaler_counter_A <= prescaler_counter_A + 1;
            if(prescaler_counter_A > prescaler_A) then
                    prescaler_counter_A <= (others => '0');
                    spi_clk <= not spi_clk;
                    
                    prescaler_counter_B <= prescaler_counter_B + 1;
                    if(prescaler_counter_B > prescaler_B) then
                        spi_trigger_clk <= not spi_trigger_clk;
                        
                        if (spi_trigger_clk = '1') then
                            prescaler_counter_C <= prescaler_counter_C + 1;
                            if(prescaler_counter_C > prescaler_C) then
                                prescaler_counter_C <= (others => '0');
                            
                                enable <= spi_trigger_clk;
                            end if;	
                        else 
                            enable <= spi_trigger_clk;
                        end if;
                                    
                        prescaler_counter_B <= (others => '0');
                    end if;
                end if;
        end if;
end process;

----------------------------------------------------------
------                LED Control                  -------
----------------------------------------------------------
LED_fsm : process (Device_State)
begin
    case Device_State is 
        when RST_DEV => 
            LED <= "1111111111111111" ;
	        SSEG_CA <= "11000000";
        when TX => 
	        SSEG_CA <= "11111001";
        when RX => 
            LED <= "0001110000000000" ;
	        SSEG_CA <= "10100100";
        when IDLE => 
            LED <= "0000001111000000" ;
	        SSEG_CA <= "10110000";
        when DEVICE_ID => 
            LED <= "0000000000111000" ;
	        SSEG_CA <= "10011001";
        --when POSITION_5 => 
        --    LED <= "0000000000000111" ;
	    --    SSEG_CA <= "10010010";
    end case;
end process;

----------------------------------------------------------
------              UART Control                   -------
----------------------------------------------------------
--Component used to send a byte of data over a UART line.
UART_CTRL : UART_TX_CTRL port map(
		SEND => uartSend,
		DATA => uartData,
		CLK => CLK,
		READY => uartRdy,
		UART_TX => uartTX);

-- UART State Machine
uart_fsm : process(CLK,btnDetect)
begin
    if rising_edge(CLK) then
        if btnDetect = '1' then
            uartState        <= RST_REG;
            reset_cntr       <= (others => '0');
            strIndex         <= 0;
            uartSend         <= '0';
            uartData         <= (others => '0');
        else
            ------------------------------------------------------
            -- Default outputs
            ------------------------------------------------------
            uartSend <= '0';

            -- State Machine
            case uartState is
                -- Reset state, hold UART idle ~2ms
                when RST_REG =>
                    if reset_cntr = RESET_CNTR_MAX then
                        reset_cntr <= (others => '0');
                        uartState  <= LD_INIT_STR;
                    else
                        reset_cntr <= reset_cntr + 1;
                    end if;

                -- Load initial welcome string
                when LD_INIT_STR =>
                    strIndex  <= 0;
                    uartState <= SEND_CHAR;

                -- Send current character
                when SEND_CHAR =>
                    uartSend <= '1';
                    uartData <= sendStr(strIndex);
                    uartState <= RDY_LOW;

                -- Hold uartSend low for one cycle
                when RDY_LOW =>
                    uartSend  <= '0';
                    uartState <= WAIT_RDY;

                -- Wait for UART ready before next char
                when WAIT_RDY =>
                    if uartRdy = '1' then
                        if strEnd = strIndex then
                            uartState <= WAIT_BTN;
                        else
                            strIndex  <= strIndex + 1;
                            uartState <= SEND_CHAR;
                        end if;
                    end if;

                -- Load button string when state changes
                when LD_BTN_STR =>
                    strIndex  <= 0;
                    uartState <= SEND_CHAR;

                -- Wait for device state change or button press
                when WAIT_BTN =>
                    if Device_State /= Device_Prev_State or btnDeBnc(4) = '1' then
                        Device_Prev_State <= Device_State;
                        uartState <= LD_BTN_STR;
                    end if;

                -- Safe fallback
                when others =>
                    uartState <= RST_REG;
            end case;
        end if;
    end if;
end process;

-- UART output assignment
UART_TXD <= uartTX;

----------------------------------------------------------
------              Button Control                 -------
----------------------------------------------------------
Inst_btn_debounce: debouncer --Debounces btn signals
    generic map(
        DEBNC_CLOCKS => (2**16),
        PORT_WIDTH => 5)
    port map(
		SIGNAL_I => BTN,
		CLK_I => CLK,
		SIGNAL_O => btnDeBnc);

btn_reg_process : process (CLK) --Registers the debounced button signals, for edge detection.
begin
	if (rising_edge(CLK)) then
		btnReg <= btnDeBnc(3 downto 0);
	end if;
end process;

--btnDetect goes high for a single clock cycle when a btn press is
--detected. This triggers a UART message to begin being sent.
btnDetect <= '1' when ((btnReg(0)='0' and btnDeBnc(0)='1') or (btnReg(1)='0' and btnDeBnc(1)='1') or (btnReg(2)='0' and btnDeBnc(2)='1') or (btnReg(3)='0' and btnDeBnc(3)='1')  ) else '0';

end Behavioral;