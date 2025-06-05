library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity fmas is
    Port (
        clk      : in  STD_LOGIC;   -- 50MHz clock
        trig     : out STD_LOGIC;    -- HC-SR04 Trigger
        echo     : in  STD_LOGIC;    -- HC-SR04 Echo
        relay    : out STD_LOGIC     -- Relay output (PIN_92)
    );
end fmas;

architecture Behavioral of fmas is
    -- Constants for 50MHz operation
    constant CLK_FREQ      : integer := 50_000_000;
    constant SOUND_SPEED   : integer := 34300;  -- cm/s
    
    constant DIST_ON       : integer := 35;
    constant DIST_OFF      : integer := 36;
    
    -- Timing constants
    constant TRIG_CYCLES   : integer := 500;       -- 10µs trigger
    constant MEASURE_DELAY : integer := 1_500_000; -- 30ms between measurements
    constant ECHO_TIMEOUT  : integer := 750_000;   -- 15ms max echo timeout
    constant RELAY_TIME    : integer := 250_000_000; -- 5s hold time
    
    -- Debouncing
    constant NUM_SAMPLES   : integer := 5;      -- Consistent readings required
    constant SAMPLE_DEV    : integer := 1;      -- Max deviation between samples (cm)
    
    -- State machine
    type state_type is (IDLE, TRIGGER, WAIT_ECHO, MEASURE, VERIFY);
    signal state : state_type := IDLE;
    
    -- Synchronized echo input
    signal echo_sync      : std_logic_vector(2 downto 0) := "000";
    
    -- Measurement system
    signal counter       : integer range 0 to MEASURE_DELAY := 0;
    signal echo_timer    : integer range 0 to ECHO_TIMEOUT := 0;
    signal distance_cm   : integer range 0 to 500 := 0;
    
    -- Precision detection system
    type sample_array is array (0 to NUM_SAMPLES-1) of integer range 0 to 500;
    signal samples       : sample_array := (others => 0);
    signal sample_index  : integer range 0 to NUM_SAMPLES-1 := 0;
    
    signal relay_reg     : std_logic := '0';
    signal hold_timer    : integer range 0 to RELAY_TIME := 0;
    signal object_detected : std_logic := '0';
   
    signal power_on_reset : std_logic := '1';
    signal reset_counter : integer range 0 to 1_000_000 := 0;  -- 20ms reset at 50MHz

begin
process(clk)
begin
    if rising_edge(clk) then
        if reset_counter < 1_000_000 then
            reset_counter <= reset_counter + 1;
            power_on_reset <= '1';
        else
            power_on_reset <= '0';
        end if;
    end if;
end process;

process(clk)
begin
    if rising_edge(clk) then
        echo_sync <= echo_sync(1 downto 0) & echo;
    end if;
end process;

process(clk)
    variable sample_avg : integer range 0 to 500;
    variable consistent : boolean;
begin
    if rising_edge(clk) then
        if power_on_reset = '1' then
            -- Initialize all signals (like JTAG mode)
            relay_reg <= '0';
            object_detected <= '0';
            hold_timer <= 0;
            state <= IDLE;
            counter <= 0;
            echo_timer <= 0;
            samples <= (others => 0);
        else
            trig <= '0';  -- Default trigger off
            
            case state is
                when IDLE =>
                    if object_detected = '1' then
                        -- Hold relay ON for RELAY_TIME
                        if hold_timer < RELAY_TIME then
                            hold_timer <= hold_timer + 1;
                        else
                            hold_timer <= 0;
                            object_detected <= '0';
                            relay_reg <= '0';
                        end if;
                    elsif counter < MEASURE_DELAY then
                        counter <= counter + 1;
                    else
                        counter <= 0;
                        state <= TRIGGER;
                    end if;
                    
                when TRIGGER =>
                    trig <= '1';
                    if counter < TRIG_CYCLES then
                        counter <= counter + 1;
                    else
                        counter <= 0;
                        state <= WAIT_ECHO;
                    end if;
                    
                when WAIT_ECHO =>
                    if echo_sync(2) = '1' then
                        echo_timer <= 0;
                        state <= MEASURE;
                    elsif counter < ECHO_TIMEOUT then
                        counter <= counter + 1;
                    else
                        state <= IDLE;  -- Timeout
                    end if;
                    
                when MEASURE =>
                    if echo_sync(2) = '1' then
                        if echo_timer < ECHO_TIMEOUT then
                            echo_timer <= echo_timer + 1;
                        else
                            state <= IDLE;  -- Timeout
                        end if;
                    else
                        distance_cm <= (echo_timer * SOUND_SPEED + CLK_FREQ) /(2*CLK_FREQ);
                        state <= VERIFY;
                    end if;
                    
                when VERIFY =>
                    samples(sample_index) <= distance_cm;
                    sample_index <= (sample_index + 1) mod NUM_SAMPLES;
                    consistent := true;
                    sample_avg := (samples(0) + samples(1) + samples(2) + 
                                  samples(3) + samples(4)) / NUM_SAMPLES;
                    
                    for i in 0 to NUM_SAMPLES-1 loop
                        if abs(samples(i) - sample_avg) > SAMPLE_DEV then
                            consistent := false;
                        end if;
                    end loop;
                    
                    if consistent then
                        if sample_avg <= DIST_ON and object_detected = '0' then
                            relay_reg <= '1';
                            object_detected <= '1';
                            hold_timer <= 0;
                        elsif sample_avg >= DIST_OFF and object_detected = '1' then
                            relay_reg <= '0';
                            object_detected <= '0';
                        end if;
                    end if;
                    
                    state <= IDLE;  
                when others =>
                    state <= IDLE;
            end case;
        end if;  
        relay <= relay_reg;
    end if;
end process;

end Behavioral;
