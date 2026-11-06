//==============================================================================
// ir_core.v -- IR capture / replay core ("Universal IR Cloner")
//
// Target : Intel MAX 10 10M50DAF484C7G (Terasic DE10-Lite), clk = 50 MHz
// Role   : RX (KY-022 raw timing capture), TX (38 kHz gated replay),
//          1024 x 16 pulse buffer, status flags.  The SPI slave is a separate
//          module (spi_slave.v) and talks to this core through the command
//          pulses and the buffer bus below.  Single clock domain (clk).
//
// LOCKED SPEC SUMMARY
//   * Buffer [Mark, Space, Mark, ...], uint16 microseconds, DEPTH = 1024.
//   * RX: IR_RX is active-low (light = 0).  It is inverted internally
//     (Mark = 1).  2-FF synchronizer, then a symmetric glitch filter
//     (GLITCH_US).  First Mark = falling edge of the pin (Space -> Mark).
//   * RX ends when a Space lasts EOF_TIMEOUT_US: the trailing Space is NOT
//     stored (rx_len is odd).  Overflow (rx_overflow + rx_done) when the
//     buffer is full, or when a Mark reaches 65,536 us (that Mark is not
//     stored).  Width counter is 17 bits so bit 16 detects the overrun.
//   * RX is disabled while tx_busy and for HOLDOFF_MS afterwards.
//   * TX: one-shot, entry 0 = Mark.  Entry value 0 is clamped to 1 us.
//     Odd length ends after the last Mark, even length after the last Space.
//     Durations are timed in clock cycles (no tick quantization, no drift).
//   * Carrier: toggles every CARRIER_HALF clocks (658 -> ~37.99 kHz).
//     The phase restarts at the start of every Mark (first cycle is whole)
//     and the gate closes only while the carrier is low (no runt pulses).
//     The visible Mark/Space boundary can therefore move by up to
//     +/- CARRIER_HALF clocks (~13 us); Mark + Space is exact, so there is
//     no drift over a frame (verified in simulation: 600/500 us -> 592/508).
//     ir_led is registered: ir_led <= carrier & gate.
//   * SPI buffer access (buf_*) is allowed only when RX is IDLE/DONE and
//     TX is idle (buf_ok = 1).  Otherwise reads return 0x0000 and writes are
//     ignored.  An accepted write clears rx_done / rx_overflow (rx_len is
//     left unchanged).
//
// COMMAND PULSES (from spi_slave.v, one clk cycle each, issued on CS_N rising
// edge after a complete, valid frame):
//   cmd_arm_rx    : clears flags + rx_len, waits for first Mark.
//                   Ignored while tx_busy.  Restarts a capture in progress.
//   cmd_set_len   : tx_len <= min(cmd_set_len_value, DEPTH).  Ignored while
//                   tx_busy.
//   cmd_start_tx  : one-shot replay.  Ignored while RX is ARMED/MARK/SPACE,
//                   while tx_busy, or when tx_len = 0.
//
// BUFFER BUS (spi_slave.v side):
//   buf_addr presented  -> buf_rdata valid ONE clock later (RAM latency).
//   buf_we pulse (1 clk) with buf_addr / buf_wdata stable writes one word.
//   Pointer saturation at index DEPTH-1 is done in spi_slave.v.
//
// NOTES
//   * AW / LW are derived parameters: do not override them.  DEPTH must be a
//     power of two.  HOLDOFF_MS must be <= 65.  GLITCH_US must be < 65536.
//   * A Space shorter than CARRIER_HALF clocks (~13 us) written by the host
//     can merge with the previous Mark (gate still closing).  Real captured
//     frames never contain such a Space (glitch filter).
//   * Constraints (SDC): create_clock 50 MHz on clk; set_false_path from the
//     input pins (rst_n, ir_rx_n and the SPI pins) to the synchronizers.
//==============================================================================
module ir_core #(
    parameter CLK_HZ         = 50_000_000,
    parameter CARRIER_HALF   = 658,
    parameter EOF_TIMEOUT_US = 35000,
    parameter DEPTH          = 1024,
    parameter GLITCH_US      = 100,
    parameter HOLDOFF_MS     = 2,
    // derived -- do not override
    parameter AW             = $clog2(DEPTH),
    parameter LW             = $clog2(DEPTH) + 1
)(
    input  wire          clk,
    input  wire          rst_n,              // async, active-low (KEY0)
    input  wire          ir_rx_n,            // KY-022 raw pin, active-low
    output wire          ir_led,             // active-high -> 2N2222A base

    // command pulses from spi_slave
    input  wire          cmd_arm_rx,
    input  wire          cmd_start_tx,
    input  wire          cmd_set_len,
    input  wire [15:0]   cmd_set_len_value,

    // status
    output wire [LW-1:0] rx_len,             // pulses captured (0..DEPTH)
    output wire          rx_done,
    output wire          rx_overflow,
    output wire          tx_busy,

    // buffer bus for spi_slave
    output wire          buf_ok,
    input  wire          buf_we,
    input  wire [AW-1:0] buf_addr,
    input  wire [15:0]   buf_wdata,
    output wire [15:0]   buf_rdata
);

    //--------------------------------------------------------------------------
    // Constants
    //--------------------------------------------------------------------------
    localparam TICK_DIV   = CLK_HZ / 1_000_000;   // clocks per microsecond
    localparam HOLD_TICKS = HOLDOFF_MS * 1000;    // microseconds

    localparam [2:0] R_IDLE  = 3'd0,
                     R_ARMED = 3'd1,
                     R_MARK  = 3'd2,
                     R_SPACE = 3'd3,
                     R_DONE  = 3'd4;

    localparam [1:0] T_IDLE = 2'd0,
                     T_ADDR = 2'd1,
                     T_LOAD = 2'd2,
                     T_RUN  = 2'd3;

    //--------------------------------------------------------------------------
    // Declarations
    //--------------------------------------------------------------------------
    reg  [1:0]    rst_sync;
    reg  [1:0]    rx_sync;
    reg  [15:0]   tick_cnt;
    reg  [15:0]   hold_cnt;
    reg           filt, filt_d;
    reg  [15:0]   gcnt;

    reg  [2:0]    rx_state;
    reg  [LW-1:0] rx_len_r;
    reg  [16:0]   rx_cnt;
    reg           rx_done_r, rx_ovf_r;
    reg           rx_wr;
    reg  [AW-1:0] rx_waddr;
    reg  [15:0]   rx_wdata;

    reg  [1:0]    tx_state;
    reg           tx_busy_r;
    reg  [LW-1:0] tx_len_r, tx_idx;
    reg           tx_is_mark;
    reg  [21:0]   tx_cnt;

    reg           gate_req, gate, carrier;
    reg  [15:0]   car_cnt;
    reg           ir_led_r;

    //--------------------------------------------------------------------------
    // 0. Reset: asynchronous assert, synchronous release
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end
    wire rst_s_n = rst_sync[1];

    //--------------------------------------------------------------------------
    // 1. 1 us time base (count 0 .. TICK_DIV-1)
    //--------------------------------------------------------------------------
    wire tick = (tick_cnt == TICK_DIV - 1);

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n)    tick_cnt <= 16'd0;
        else if (tick)   tick_cnt <= 16'd0;
        else             tick_cnt <= tick_cnt + 16'd1;
    end

    //--------------------------------------------------------------------------
    // 2. RX input: 2-FF synchronizer, inversion, symmetric glitch filter
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) rx_sync <= 2'b11;               // idle = pin high
        else          rx_sync <= {rx_sync[0], ir_rx_n};
    end
    wire mark_s = ~rx_sync[1];                        // 1 = Mark (light seen)

    // RX is disabled while transmitting and during the hold-off afterwards
    wire rx_enable = !tx_busy_r && (hold_cnt == 16'd0);

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            hold_cnt <= 16'd0;
        end else if (tx_busy_r) begin
            hold_cnt <= HOLD_TICKS[15:0];
        end else if (tick && (hold_cnt != 16'd0)) begin
            hold_cnt <= hold_cnt - 16'd1;
        end
    end

    // The filtered level changes only after the raw level has been stable
    // for GLITCH_US microseconds, in BOTH directions, so both edges are
    // delayed equally and pulse widths are preserved.
    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            filt <= 1'b0;
            gcnt <= 16'd0;
        end else if (!rx_enable) begin
            filt <= 1'b0;
            gcnt <= 16'd0;
        end else if (GLITCH_US == 0) begin
            filt <= mark_s;                           // bypass
        end else if (mark_s == filt) begin
            gcnt <= 16'd0;
        end else if (tick) begin
            if (gcnt == GLITCH_US - 1) begin
                filt <= mark_s;
                gcnt <= 16'd0;
            end else begin
                gcnt <= gcnt + 16'd1;
            end
        end
    end

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) filt_d <= 1'b0;
        else          filt_d <= filt;
    end

    wire mark_start =  filt & ~filt_d;                // pin falling edge
    wire mark_end   = ~filt &  filt_d;                // pin rising edge

    //--------------------------------------------------------------------------
    // 3. Pulse buffer: synchronous single-port RAM (1 clock read latency)
    //--------------------------------------------------------------------------
    wire rx_active = (rx_state == R_ARMED) || (rx_state == R_MARK) ||
                     (rx_state == R_SPACE);

    // SPI may touch the buffer only when RX and TX are both quiet
    wire buf_ok_w = !tx_busy_r && !rx_active && !rx_wr;
    wire spi_wr   = buf_we && buf_ok_w;

    // Access mux (the three owners are mutually exclusive by design):
    //   RX store pulse > TX playback > SPI
    wire [AW-1:0] ram_addr  = rx_wr      ? rx_waddr :
                              tx_busy_r  ? tx_idx[AW-1:0] : buf_addr;
    wire [15:0]   ram_wdata = rx_wr      ? rx_wdata : buf_wdata;
    wire          ram_we    = rx_wr | spi_wr;

    (* ramstyle = "M9K, no_rw_check" *) reg [15:0] mem [0:DEPTH-1];
    reg [AW-1:0] ram_addr_q;
    integer k;
    initial begin
        for (k = 0; k < DEPTH; k = k + 1) mem[k] = 16'd0;
    end

    always @(posedge clk) begin
        if (ram_we) mem[ram_addr] <= ram_wdata;
        ram_addr_q <= ram_addr;                       // registered read address
    end
    wire [15:0] ram_q = mem[ram_addr_q];

    assign buf_ok    = buf_ok_w;
    assign buf_rdata = buf_ok_w ? ram_q : 16'h0000;

    //--------------------------------------------------------------------------
    // 4. RX state machine
    //--------------------------------------------------------------------------
    wire [16:0] rx_cnt_nxt   = rx_cnt + 17'd1;
    wire        rx_slot_last = (rx_len_r == DEPTH - 1);   // this store fills it

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            rx_state  <= R_IDLE;
            rx_len_r  <= {LW{1'b0}};
            rx_cnt    <= 17'd0;
            rx_done_r <= 1'b0;
            rx_ovf_r  <= 1'b0;
            rx_wr     <= 1'b0;
            rx_waddr  <= {AW{1'b0}};
            rx_wdata  <= 16'd0;
        end else begin
            rx_wr <= 1'b0;                                // default: no store

            if (cmd_arm_rx && !tx_busy_r) begin
                // (re)arm: clear everything, wait for the first Mark
                rx_state  <= R_ARMED;
                rx_len_r  <= {LW{1'b0}};
                rx_cnt    <= 17'd0;
                rx_done_r <= 1'b0;
                rx_ovf_r  <= 1'b0;
            end else begin
                case (rx_state)
                    R_IDLE: ;

                    R_DONE: begin
                        // buffer overwritten by the host -> capture invalid
                        if (spi_wr) begin
                            rx_state  <= R_IDLE;
                            rx_done_r <= 1'b0;
                            rx_ovf_r  <= 1'b0;
                        end
                    end

                    R_ARMED: begin
                        if (mark_start) begin
                            rx_state <= R_MARK;
                            rx_cnt   <= 17'd0;
                        end
                    end

                    R_MARK: begin
                        if (mark_end) begin
                            // store Mark width
                            rx_wr    <= 1'b1;
                            rx_waddr <= rx_len_r[AW-1:0];
                            rx_wdata <= rx_cnt[15:0];
                            rx_len_r <= rx_len_r + 1'b1;
                            rx_cnt   <= 17'd0;
                            if (rx_slot_last) begin
                                rx_ovf_r  <= 1'b1;
                                rx_done_r <= 1'b1;
                                rx_state  <= R_DONE;
                            end else begin
                                rx_state  <= R_SPACE;
                            end
                        end else if (tick) begin
                            if (rx_cnt_nxt[16]) begin
                                // Mark >= 65,536 us: not stored, stop
                                rx_ovf_r  <= 1'b1;
                                rx_done_r <= 1'b1;
                                rx_state  <= R_DONE;
                            end else begin
                                rx_cnt <= rx_cnt_nxt;
                            end
                        end
                    end

                    R_SPACE: begin
                        if (mark_start) begin
                            // store Space width
                            rx_wr    <= 1'b1;
                            rx_waddr <= rx_len_r[AW-1:0];
                            rx_wdata <= rx_cnt[15:0];
                            rx_len_r <= rx_len_r + 1'b1;
                            rx_cnt   <= 17'd0;
                            if (rx_slot_last) begin
                                rx_ovf_r  <= 1'b1;
                                rx_done_r <= 1'b1;
                                rx_state  <= R_DONE;
                            end else begin
                                rx_state  <= R_MARK;
                            end
                        end else if (tick) begin
                            if (rx_cnt_nxt >= EOF_TIMEOUT_US) begin
                                // end of frame: trailing Space is dropped
                                rx_done_r <= 1'b1;
                                rx_state  <= R_DONE;
                            end else begin
                                rx_cnt <= rx_cnt_nxt;
                            end
                        end
                    end

                    default: rx_state <= R_IDLE;
                endcase
            end
        end
    end

    //--------------------------------------------------------------------------
    // 5. TX state machine (durations counted in clock cycles)
    //
    //   T_IDLE -> T_ADDR (present address) -> T_LOAD (RAM data valid, load
    //   counter, set level) -> T_RUN (count down) -> T_ADDR ... -> T_IDLE
    //
    //   Each segment's level (gate_req) is held for (RUN cycles + ADDR + LOAD)
    //   cycles, so the counter is loaded with dur*TICK_DIV - 2 to make every
    //   segment exactly dur microseconds long.
    //--------------------------------------------------------------------------
    wire [15:0] tx_dur     = (ram_q == 16'd0) ? 16'd1 : ram_q;   // clamp 0 -> 1
    wire [21:0] tx_dur_clk = tx_dur * TICK_DIV;

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            tx_state   <= T_IDLE;
            tx_busy_r  <= 1'b0;
            tx_len_r   <= {LW{1'b0}};
            tx_idx     <= {LW{1'b0}};
            tx_is_mark <= 1'b1;
            tx_cnt     <= 22'd0;
            gate_req   <= 1'b0;
        end else begin
            // SET_LEN: clamp to DEPTH, ignored while transmitting
            if (cmd_set_len && !tx_busy_r) begin
                if (cmd_set_len_value > DEPTH) tx_len_r <= DEPTH;
                else                           tx_len_r <= cmd_set_len_value[LW-1:0];
            end

            case (tx_state)
                T_IDLE: begin
                    gate_req <= 1'b0;
                    if (cmd_start_tx && !rx_active && (tx_len_r != {LW{1'b0}})) begin
                        tx_busy_r  <= 1'b1;
                        tx_idx     <= {LW{1'b0}};
                        tx_is_mark <= 1'b1;
                        tx_state   <= T_ADDR;
                    end
                end

                T_ADDR: begin
                    // ram_addr = tx_idx while tx_busy_r = 1
                    tx_state <= T_LOAD;
                end

                T_LOAD: begin
                    tx_cnt   <= tx_dur_clk - 22'd2;
                    gate_req <= tx_is_mark;
                    tx_state <= T_RUN;
                end

                T_RUN: begin
                    if (tx_cnt <= 22'd1) begin
                        if ((tx_idx + 1'b1) >= tx_len_r) begin
                            // last entry finished
                            tx_busy_r <= 1'b0;
                            gate_req  <= 1'b0;
                            tx_state  <= T_IDLE;
                        end else begin
                            tx_idx     <= tx_idx + 1'b1;
                            tx_is_mark <= ~tx_is_mark;
                            tx_state   <= T_ADDR;
                        end
                    end else begin
                        tx_cnt <= tx_cnt - 22'd1;
                    end
                end

                default: tx_state <= T_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // 6. 38 kHz carrier + register gating
    //--------------------------------------------------------------------------
    // Phase restarts when a Mark opens the gate; gate closes only while the
    // carrier is low, so every output pulse is a whole carrier half-period.
    wire car_restart = gate_req & ~gate;

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            carrier  <= 1'b0;
            car_cnt  <= 16'd0;
            gate     <= 1'b0;
            ir_led_r <= 1'b0;
        end else begin
            if (car_restart) begin
                carrier <= 1'b1;
                car_cnt <= 16'd0;
            end else if (car_cnt == CARRIER_HALF - 1) begin
                car_cnt <= 16'd0;
                carrier <= ~carrier;
            end else begin
                car_cnt <= car_cnt + 16'd1;
            end

            if (gate_req)        gate <= 1'b1;
            else if (!carrier)   gate <= 1'b0;

            ir_led_r <= carrier & gate;               // registered output
        end
    end

    //--------------------------------------------------------------------------
    // 7. Outputs
    //--------------------------------------------------------------------------
    assign ir_led      = ir_led_r;
    assign rx_len      = rx_len_r;
    assign rx_done     = rx_done_r;
    assign rx_overflow = rx_ovf_r;
    assign tx_busy     = tx_busy_r;

endmodule
