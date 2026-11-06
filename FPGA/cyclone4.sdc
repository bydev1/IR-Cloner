# ==============================================================================
# cyclone4.sdc -- Synopsys Design Constraints for Universal IR Transceiver
# Target: Intel Cyclone IV E (EP4CE6E22C8N)
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Master Clock Definition
# ------------------------------------------------------------------------------
# 50 MHz On-board Oscillator (Period = 20.000 ns)
create_clock -name clk -period 20.000 [get_ports {clk}]

# Automatically calculate and apply clock jitter and clock network skew margin
derive_clock_uncertainty

# ------------------------------------------------------------------------------
# 2. Input False Paths (Asynchronous / Oversampled Inputs)
# ------------------------------------------------------------------------------
# rst_n_key: Asynchronous human push-button (handled by internal 2-FF sync)
set_false_path -from [get_ports {rst_n_key}]

# SPI Bus Inputs: Master clock is NOT used as an FPGA clock. 
# SCK, CS_N, and MOSI are treated as asynchronous data inputs and oversampled.
set_false_path -from [get_ports {spi_sck}]
set_false_path -from [get_ports {spi_cs_n}]
set_false_path -from [get_ports {spi_mosi}]

# IR Receiver: Asynchronous 38 kHz baseband envelope (handled by internal sync)
set_false_path -from [get_ports {ir_rx_in}]

# ------------------------------------------------------------------------------
# 3. Output False Paths (Asynchronous / Slow Outputs)
# ------------------------------------------------------------------------------
# Telemetry LEDs: Human-readable status indicators
set_false_path -to [get_ports {led[*]}]

# SPI MISO: Driven asynchronously relative to the 50 MHz clock
set_false_path -to [get_ports {spi_miso}]

# IR Transmitter: Slow 38 kHz carrier frequency output
set_false_path -to [get_ports {ir_tx_out}]

# ==============================================================================
# ARCHITECTURE NOTE:
# - No create_generated_clock for 1 us, 100 us, and 38 kHz tick signals.
#   They are strictly implemented as Synchronous Clock Enables.
# - Recovery/Removal of the internal reset network is fully analyzed 
#   by TimeQuest automatically post-synchronizer.
# ==============================================================================