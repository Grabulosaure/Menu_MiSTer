--------------------------------------------------------------------------------
-- EDID
--------------------------------------------------------------------------------
-- I2C <-> async serial port
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY edid IS
  GENERIC (
    SYSFREQ  : natural := 50_000_000;
    BAUDRATE : natural := 115200
    );
  PORT (
    -- UART
    rxd    : IN  std_logic;
    txd    : OUT std_logic;
    
    -- IIC
    scl    : OUT std_logic;
    sda    : INOUT std_logic;
    
    done   : IN  std_logic;
    
    clk    : IN  std_logic
    );
END ENTITY edid;

ARCHITECTURE rtl OF edid IS
  
  ------------------------------------------------
  -- I2C interface
  CONSTANT CLKDIV : natural := SYSFREQ / (400_000*8)+1;
  SIGNAL iic_clkcpt : natural RANGE 0 TO CLKDIV;
    
  SIGNAL iic_run,iic_done,iic_tick,iic_start : std_logic;
  SIGNAL iic_wr : std_logic;
  SIGNAL iic_pos : natural RANGE 0 TO 40*8;
  SIGNAL ii_sda,ii_scl : std_logic;
  SIGNAL iic_shift : unsigned(0 TO 38);
  SIGNAL iic_dr,iic_dw : unsigned(7 DOWNTO 0);
  SIGNAL iic_a  : unsigned(6 DOWNTO 0);
  SIGNAL iic_ra : unsigned(7 DOWNTO 0);
  
  -- Async port clock synth
  SIGNAL art_sync : std_logic;

  FUNCTION gcd (  -- Euclide
    CONSTANT a  : natural;
    CONSTANT b  : natural) RETURN natural IS
  BEGIN
    IF b/=0 THEN   RETURN gcd(b,a MOD b);
    ELSE RETURN a;
    END IF;
  END FUNCTION gcd;
  CONSTANT MUL : natural := BAUDRATE*16/gcd(SYSFREQ,BAUDRATE*16);
  CONSTANT DIV : natural :=     SYSFREQ/gcd(SYSFREQ,BAUDRATE*16);
  SIGNAL acc : integer RANGE -MUL TO DIV-MUL :=0;
  SIGNAL atick,sync16 : std_logic;
  SIGNAL cpt16  : natural RANGE 0 TO 15;

  -- Async port TX
  SIGNAL tx_req,tx_ack,tx_trans : std_logic;
  SIGNAL tx_buf : unsigned(9 DOWNTO 0);
  SIGNAL tx_cpt : natural RANGE 0 TO 11;
  SIGNAL tx_data : unsigned(7 DOWNTO 0);

  -- Async port RX
  SIGNAL rx_brx,rx_trans,rx_req : std_logic;
  SIGNAL rxd_sync,rxd_sync2 : std_logic;
  SIGNAL rx_phase : natural RANGE 0 TO 15;
  SIGNAL rx_cpt   : natural RANGE 0 TO 11;
  SIGNAL rx_rad   : unsigned(8 DOWNTO 0);
  SIGNAL rx_data,rx_cmd  : unsigned(7 DOWNTO 0);
        
  TYPE enum_state IS (sIDLE,sADRS,sREG,sWRITE,sWAIT);
  SIGNAL state : enum_state;
  
  CONSTANT CHAR_R : unsigned(7 DOWNTO 0) :=x"52";
  CONSTANT CHAR_W : unsigned(7 DOWNTO 0) :=x"57";
BEGIN
    
  ----------------------------------------------------------
  -- I2C INTERFACE
  IIC: PROCESS(clk) IS
  BEGIN
    IF rising_edge(clk) THEN
      IF iic_clkcpt<CLKDIV THEN
        iic_clkcpt<=iic_clkcpt+1;
        iic_tick<='0';
      ELSE
        iic_clkcpt<=0;
        iic_tick<='1';
      END IF;

      iic_done<='0';
      
      IF done='0' THEN
        iic_run<='0';
      ELSE
        ------------------------------------
        IF iic_tick='1' THEN
          IF iic_start='1' THEN
            iic_pos<=0;
            iic_run<='1';
            IF iic_wr='0' THEN
              iic_shift<='0' & iic_a & '0' & '1' & -- START ADRS WR <ACK>
                          iic_ra & '1' & -- REG <ACK>
                          '0' & iic_a & '1' & '1' & -- START ADRS RD <ACK>
                          x"FF" & '1' & '0'; -- data NACK STOP
            ELSE
              iic_shift<='0' & iic_a & '0' & '1' & -- START ADRS WR <ACK>
                          iic_ra & '1' & -- REG <ACK>
                          iic_dw & '1' & '0' & --DW <ACK> STOP
                          x"FF" & '1' & '1';
            END IF;
            ii_scl<='1';
            ii_sda<='1';
          ELSIF iic_run='1' THEN
            IF (iic_pos=8*39-1 AND iic_wr='0') OR
               (iic_pos=8*29-1 AND iic_wr='1') THEN
              iic_run<='0';
              iic_done<='1';
            END IF;
            iic_pos<=iic_pos+1;
            IF iic_pos MOD 8 =7 THEN
              iic_shift<=iic_shift(1 TO iic_shift'right) & '1';
            END IF;

            ii_sda<=iic_shift(0);
            IF iic_pos<2 THEN
              ii_sda<='1';
            ELSIF iic_pos/8=19 AND iic_pos MOD 8<4 AND iic_wr='0' THEN
              ii_sda<='1';
            ELSIF (iic_pos>38*8+5 AND iic_wr='0') OR
                  (iic_pos>28*8+5 AND iic_wr='1') THEN
              ii_sda<='1';
            END IF;
            
            ii_scl<='0';
            IF iic_pos<6 OR iic_pos>38*8+1 OR (iic_pos>28*8+1 AND iic_wr='1')
            THEN
              ii_scl<='1';
            ELSIF iic_pos MOD 8>1 AND iic_pos MOD 8 <6 THEN
              ii_scl<='1';
            END IF;
            IF iic_pos MOD 8=5 AND iic_pos/8<37 THEN
              iic_dr<=iic_dr(6 DOWNTO 0) & sda;
            END IF;
          END IF;
        END IF;
      END IF;
    END IF;
  END PROCESS;
  
  scl<='0' WHEN ii_scl='0' ELSE 'Z';
  sda<='0' WHEN ii_sda='0' ELSE 'Z';
  
  ----------------------------------------------------------
  -- Asynchronous port clock synth
  Synth:PROCESS (clk)
  BEGIN
    IF rising_edge(clk) THEN
      IF done='0' THEN atick<='0'; END IF;
      IF acc>0 THEN
        acc<=acc-MUL;
        atick<='0';
      ELSE
        acc<=acc+DIV-MUL;
        atick<='1';
      END IF;
    END IF;
  END PROCESS Synth;
  
  ClockGen:PROCESS (clk)
  BEGIN
    IF rising_edge(clk) THEN
      sync16<='0';
      IF atick='1' THEN
        IF cpt16/=15 THEN
          cpt16<=cpt16+1;
        ELSE
          cpt16<=0;
          sync16<='1';
        END IF;
      END IF;
    END IF;
  END PROCESS ClockGen;
  
  ----------------------------------------------------------
  Emit:PROCESS (clk)
    VARIABLE pop_v : std_logic;
  BEGIN
    IF rising_edge(clk) THEN
      IF done='0' THEN
        tx_cpt<=0;
        tx_trans<='0';
        tx_buf<="1111111111";
        txd<='1';
      ELSE
        ---------------------------------------------
        IF sync16='1' THEN
          IF tx_trans='0' THEN
            txd<='1';
          ELSE
            tx_buf<='1' & tx_buf(9 DOWNTO 1);
            txd<=tx_buf(0);
          END IF;
          
          IF tx_cpt<9 THEN
            tx_cpt<=tx_cpt+1;
          ELSE
            tx_trans<='0';
          END IF;
        END IF;
        IF tx_req='1' AND tx_trans='0' THEN
          tx_trans<='1';
          tx_cpt<=0;
          tx_buf<='1' & tx_data & '0';
          tx_ack<='1';
        END IF;
      END IF;
      ---------------------------------------------
    END IF;
  END PROCESS Emit;
  
  -------------------------------------------------
  Recept: PROCESS (clk)
    VARIABLE push_v : std_logic;
  BEGIN
    IF rising_edge(clk) THEN
      IF done = '0' THEN
        rx_trans<='0';
        rx_phase<=0;
        rx_cpt<=0;
        rx_rad<=(OTHERS =>'0');
        rx_brx<='0';
      ELSE
        rx_req<='0';
        rxd_sync<=rxd;
        rxd_sync2<=rxd_sync;
        IF atick='1' THEN
          IF rx_trans='0' THEN
            IF rxd_sync2='0' THEN
              rx_trans<='1';
              rx_phase<=cpt16;
              rx_cpt<=0;
            END IF;
          ELSE
            IF rx_phase=(cpt16 + 8) MOD 16 THEN
              rx_rad<=rxd_sync2 & rx_rad(8 DOWNTO 1);
              IF rx_cpt/=9 THEN
                rx_cpt<=rx_cpt+1;
              ELSE
                IF rxd_sync2='1' THEN
                  rx_req<=NOT rx_brx;
                  rx_data<=rx_rad(8 DOWNTO 1);
                  rx_trans<='0';
                  rx_brx<='0';
                ELSE
                  rx_brx<='1';
                END IF;
              END IF;
            END IF;
          END IF;
        END IF;
      END IF;
    END IF;
  END PROCESS Recept;
      
  ----------------------------------------------------------
  -- FSM
  FSM:PROCESS(clk) IS
  BEGIN
    IF rising_edge(clk) THEN
      tx_req<='0';
      
      IF done='0' THEN
        state<=sIDLE;
        iic_start<='0';
      ELSE
        -- 'W' <ADRS> <REG> <DATA>
        -- 'R' <ADRS> <REG> => <DATA>
        CASE state IS
          WHEN sIDLE =>
            rx_cmd<=rx_data;
            IF rx_req='1' AND (rx_data=CHAR_W OR rx_data=CHAR_R) THEN
              state<=sADRS;
            END IF;
            
          WHEN sADRS =>
            iic_a<=rx_data(7 DOWNTO 1);
            IF rx_req='1' THEN
              state<=sREG;
            END IF;

          WHEN sREG =>
            iic_ra<=rx_data;
            IF rx_req='1' AND rx_cmd=CHAR_W THEN
              state<=sWRITE;
            ELSIF rx_req='1' THEN
              iic_start<='1';
              iic_wr<='0';
              state<=sWAIT;
            END IF;
            
          WHEN sWRITE =>
            iic_dw<=rx_data;
            iic_wr<='1';
            IF rx_req='1' THEN
              iic_start<='1';
              state<=sWAIT;
            END IF;
            
          WHEN sWAIT =>
            tx_data<=iic_dr;
            IF iic_run='1' THEN
              iic_start<='0';
            END IF;
            IF iic_done='1' THEN
              IF rx_cmd=CHAR_R THEN
                tx_req<='1';
              END IF;
              state<=sIDLE;
            END IF;
        END CASE;
      END IF;
    END IF;
  END PROCESS FSM;
      
End ARCHITECTURE rtl;

