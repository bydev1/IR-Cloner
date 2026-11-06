//==============================================================================
// spi_slave.v -- SPI Mode 0 slave for the "Universal IR Cloner"
//
// Talks to ir_core.v (unchanged) through its command pulses and buffer bus.
// Everything runs on posedge clk (50 MHz).  spi_sck is NEVER used as a clock:
// SCK / CS_N / MOSI are 3-FF synchronized, glitch-filtered and turned into
// one-clock edge pulses (sck_rising, sck_falling, cs_falling, cs_rising).
//
// PROTOCOL  (CS_N low for the whole frame, MSB first, no dummy byte,
//            first byte = command; MISO is 0x00 during the command byte)
//   0x01 STATUS    : next byte  = {5'b0, rx_overflow, tx_busy, rx_done}
//   0x02 GET_LEN   : next 2 bytes = {5'b0, rx_len[10:8]}, rx_len[7:0]
//   0x03 READ_BUF  : stream of 16-bit words (high byte first) from index 0;
//                    index > DEPTH-1 or access blocked -> 0x0000
//   0x04 SET_LEN   : 2 bytes (high, low) -> clamped to DEPTH, sent to ir_core
//   0x05 WRITE_BUF : stream of 16-bit words (high byte first) from index 0;
//                    index > DEPTH-1, blocked access, half word -> ignored
//   0x06 START_TX  : one-shot replay
//   0x07 ARM_RX    : clear flags / length, wait for next frame
//   other          : ignored, MISO = 0x00
//
//   * Buffer pointer resets to 0 at every CS_N falling edge and advances one
//     word per 2 bytes.  It saturates at DEPTH (no wrap-around).
//   * STATUS, GET_LEN and the "buffer access allowed" flag (buf_ok) are
//     snapshotted when the command byte completes.
//   * ARM_RX, START_TX and SET_LEN take effect when CS_N rises, and only if
//     the frame was complete: bit count a multiple of 8 and at least
//     1 byte (ARM_RX / START_TX) or 3 bytes (SET_LEN).  Bytes beyond the
//     minimum are ignored.  A frame ended early or with a stray SCK pulse is
//     discarded without side effects.
//   * READ_BUF / WRITE_BUF act immediately (no commit step).  Words past
//     rx_len are stale data -- read only what GET_LEN reports.
//
// MODE 0 TIMING
//   MOSI is sampled at sck_rising.  MISO changes only at sck_falling.  The
//   first response byte is loaded at the falling edge that follows the 8th
//   rising edge, i.e. about one half SCK period (>= 2 us = 100 clocks) after
//   the command is known, so RAM latency is never an issue.  Pin-to-pulse
//   latency is 4 clocks (80 ns).
//
// PORT MAP TO ir_core.v
//   cmd_arm_rx, cmd_start_tx, cmd_set_len, cmd_set_len_value -> same names
//   buf_we, buf_addr, buf_wdata -> same names
//   rx_len, rx_done, rx_overflow, tx_busy, buf_ok, buf_rdata <- same names
//   tx_len is an informational copy of the last committed (clamped) SET_LEN
//   value; ir_core is the authority on whether it was accepted (it ignores
//   SET_LEN while tx_busy).
//
// TOP-LEVEL NOTES
//   * spi_miso must go straight to its pin (tri-state output).
//   * Enable the weak pull-up on spi_cs_n in Quartus so a floating line while
//     the ESP32 boots reads as "not selected".
//   * rst_n may be the raw KEY0; it is synchronized here.
//==============================================================================
module spi_slave #(
    parameter AW = 10,                 // buffer address width (DEPTH = 2**AW)
    parameter LW = AW + 1              // derived -- do not override
)(
    input  wire          clk,          // 50 MHz system clock
    input  wire          rst_n,        // async assert, synchronized release

    // SPI pins (ESP32-S3 = master)
    input  wire          spi_sck,
    input  wire          spi_cs_n,
    input  wire          spi_mosi,
    output wire          spi_miso,     // 1'bz while CS_N is high

    // command pulses to ir_core (1 clk each)
    output reg           cmd_arm_rx,
    output reg           cmd_start_tx,
    output reg           cmd_set_len,
    output reg  [15:0]   cmd_set_len_value,
    output reg  [LW-1:0] tx_len,

    // status from ir_core
    input  wire [LW-1:0] rx_len,
    input  wire          rx_done,
    input  wire          tx_busy,
    input  wire          rx_overflow,

    // buffer bus to ir_core (data valid 1 clock after address)
    input  wire          buf_ok,
    output reg           buf_we,
    output reg  [AW-1:0] buf_addr,
    output reg  [15:0]   buf_wdata,
    input  wire [15:0]   buf_rdata
);

    localparam [7:0] CMD_STATUS    = 8'h01,
                     CMD_GET_LEN   = 8'h02,
                     CMD_READ_BUF  = 8'h03,
                     CMD_SET_LEN   = 8'h04,
                     CMD_WRITE_BUF = 8'h05,
                     CMD_START_TX  = 8'h06,
                     CMD_ARM_RX    = 8'h07;

    localparam [15:0]   DEPTH    = (16'd1 << AW);
    localparam [LW-1:0] DEPTH_LW = (1 << AW);          // DEPTH at LW bits

    //--------------------------------------------------------------------------
    // Reset: asynchronous assert, synchronous release
    //--------------------------------------------------------------------------
    reg [1:0] rst_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end
    wire rst_s_n = rst_sync[1];

    //--------------------------------------------------------------------------
    // Input synchronizers (3 FF) + 2-sample glitch filter + edge pulses
    //   sr[0] = 1st FF, sr[1] = 2nd FF, sr[2] = 3rd FF.  The filtered value
    //   follows sr[2] only when sr[1] == sr[2], so pulses shorter than about
    //   one clock period are rejected.
    //--------------------------------------------------------------------------
    reg [2:0] sck_sr, cs_sr, mosi_sr;
    reg       sck_f, cs_f, mosi_f;          // filtered levels
    reg       sck_d, cs_d;                  // previous filtered levels

    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            sck_sr  <= 3'b000;  cs_sr  <= 3'b111;  mosi_sr <= 3'b000;
            sck_f   <= 1'b0;    cs_f   <= 1'b1;    mosi_f  <= 1'b0;
            sck_d   <= 1'b0;    cs_d   <= 1'b1;
        end else begin
            sck_sr  <= {sck_sr[1:0],  spi_sck};
            cs_sr   <= {cs_sr[1:0],   spi_cs_n};
            mosi_sr <= {mosi_sr[1:0], spi_mosi};

            if (sck_sr[1]  == sck_sr[2])  sck_f  <= sck_sr[2];
            if (cs_sr[1]   == cs_sr[2])   cs_f   <= cs_sr[2];
            if (mosi_sr[1] == mosi_sr[2]) mosi_f <= mosi_sr[2];

            sck_d <= sck_f;
            cs_d  <= cs_f;
        end
    end

    wire sck_rising  =  sck_f & ~sck_d;
    wire sck_falling = ~sck_f &  sck_d;
    wire cs_falling  = ~cs_f  &  cs_d;
    wire cs_rising   =  cs_f  & ~cs_d;

    //--------------------------------------------------------------------------
    // Frame state
    //--------------------------------------------------------------------------
    reg  [2:0]    bitc;                     // rising edges within current byte
    reg  [11:0]   byte_cnt;                 // complete bytes (command included)
    reg  [7:0]    rx_shift;
    reg  [7:0]    cmd;
    reg  [7:0]    tx_shift;                 // MISO shift register
    reg  [11:0]   rsp_idx;                  // response byte index
    reg  [AW:0]   ptr;                      // word pointer, 0..DEPTH (saturates)
    reg  [7:0]    wr_hi;                    // high byte of the word being written
    reg  [15:0]   len_val;                  // SET_LEN value being received
    reg  [7:0]    status_snap;
    reg  [LW-1:0] len_snap;
    reg           bufok_snap;

    wire [7:0]    byte_now = {rx_shift[6:0], mosi_f};
    wire [LW-1:0] len_clamped = (len_val > DEPTH) ? DEPTH_LW : len_val[LW-1:0];
    wire          rd_ok      = bufok_snap && !ptr[AW];

    // response byte selected for the current boundary
    reg [7:0] rsp_byte;
    always @* begin
        case (cmd)
            CMD_STATUS:   rsp_byte = (rsp_idx == 12'd0) ? status_snap : 8'h00;
            CMD_GET_LEN:  rsp_byte = (rsp_idx == 12'd0) ? {{(8-(LW-8)){1'b0}}, len_snap[LW-1:8]} :
                                     (rsp_idx == 12'd1) ? len_snap[7:0] : 8'h00;
            CMD_READ_BUF: rsp_byte = rd_ok ? (rsp_idx[0] ? buf_rdata[7:0] : buf_rdata[15:8])
                                           : 8'h00;
            default:      rsp_byte = 8'h00;
        endcase
    end

    // MISO: high-Z whenever the (filtered) CS_N is high
    assign spi_miso = cs_f ? 1'bz : tx_shift[7];

    //--------------------------------------------------------------------------
    // Main frame logic
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            cmd_arm_rx        <= 1'b0;
            cmd_start_tx      <= 1'b0;
            cmd_set_len       <= 1'b0;
            cmd_set_len_value <= 16'd0;
            tx_len            <= {LW{1'b0}};
            buf_we            <= 1'b0;
            buf_addr          <= {AW{1'b0}};
            buf_wdata         <= 16'd0;
            bitc              <= 3'd0;
            byte_cnt          <= 12'd0;
            rx_shift          <= 8'd0;
            cmd               <= 8'd0;
            tx_shift          <= 8'd0;
            rsp_idx           <= 12'd0;
            ptr               <= {(AW+1){1'b0}};
            wr_hi             <= 8'd0;
            len_val           <= 16'd0;
            status_snap       <= 8'd0;
            len_snap          <= {LW{1'b0}};
            bufok_snap        <= 1'b0;
        end else begin
            // defaults: pulses last one clock; address always follows pointer
            cmd_arm_rx   <= 1'b0;
            cmd_start_tx <= 1'b0;
            cmd_set_len  <= 1'b0;
            buf_we       <= 1'b0;
            buf_addr     <= ptr[AW-1:0];

            if (cs_falling) begin
                // new frame: clear all frame state, drive 0 on MISO
                bitc     <= 3'd0;
                byte_cnt <= 12'd0;
                rx_shift <= 8'd0;
                cmd      <= 8'd0;
                tx_shift <= 8'd0;
                rsp_idx  <= 12'd0;
                ptr      <= {(AW+1){1'b0}};
                wr_hi    <= 8'd0;
                len_val  <= 16'd0;

            end else if (cs_rising) begin
                tx_shift <= 8'd0;          // no stale MISO bit at the next CS_N fall
                // commit: only complete frames (bit count multiple of 8)
                if (bitc == 3'd0) begin
                    case (cmd)
                        CMD_ARM_RX:   if (byte_cnt >= 12'd1) cmd_arm_rx   <= 1'b1;
                        CMD_START_TX: if (byte_cnt >= 12'd1) cmd_start_tx <= 1'b1;
                        CMD_SET_LEN:  if (byte_cnt >= 12'd3) begin
                                          tx_len            <= len_clamped;
                                          cmd_set_len_value <= {{(16-LW){1'b0}}, len_clamped};
                                          cmd_set_len       <= 1'b1;
                                      end
                        default: ;
                    endcase
                end

            end else if (!cs_f) begin
                if (sck_rising) begin
                    //----------------------------------------------------------
                    // sample MOSI
                    //----------------------------------------------------------
                    rx_shift <= byte_now;
                    bitc     <= bitc + 3'd1;
                    if (bitc == 3'd7) begin
                        if (byte_cnt != 12'hFFF) byte_cnt <= byte_cnt + 12'd1;
                        if (byte_cnt == 12'd0) begin
                            // command byte complete: decode + snapshots
                            cmd         <= byte_now;
                            status_snap <= {5'b00000, rx_overflow, tx_busy, rx_done};
                            len_snap    <= rx_len;
                            bufok_snap  <= buf_ok;
                        end else begin
                            case (cmd)
                                CMD_SET_LEN: begin
                                    if (byte_cnt == 12'd1) len_val[15:8] <= byte_now;
                                    if (byte_cnt == 12'd2) len_val[7:0]  <= byte_now;
                                end
                                CMD_WRITE_BUF: begin
                                    if (byte_cnt[0]) begin
                                        wr_hi <= byte_now;             // high byte
                                    end else begin                     // low byte
                                        if (bufok_snap && !ptr[AW]) begin
                                            buf_we    <= 1'b1;
                                            buf_wdata <= {wr_hi, byte_now};
                                        end
                                        if (!ptr[AW]) ptr <= ptr + 1'b1;
                                    end
                                end
                                default: ;
                            endcase
                        end
                    end

                end else if (sck_falling) begin
                    //----------------------------------------------------------
                    // drive MISO
                    //----------------------------------------------------------
                    if (bitc == 3'd0 && byte_cnt != 12'd0) begin
                        tx_shift <= rsp_byte;                          // next byte
                        if (rsp_idx != 12'hFFF) rsp_idx <= rsp_idx + 12'd1;
                        if (cmd == CMD_READ_BUF && rsp_idx[0] && !ptr[AW])
                            ptr <= ptr + 1'b1;                         // word done
                    end else begin
                        tx_shift <= {tx_shift[6:0], 1'b0};
                    end
                end
            end
        end
    end

endmodule
