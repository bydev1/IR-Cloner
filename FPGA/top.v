//==============================================================================
// top.v -- Top Level for Universal IR Transceiver (Target: Cyclone IV EP4CE6)
//
// Integrated: ir_core.v + spi_slave.v
// Features  : Power-on Reset, IR_TX Watchdog Fail-Safe, 4-LED Telemetry
//==============================================================================
module top #(
    parameter CLK_HZ     = 50_000_000,
    parameter POR_BITS   = 16,          // 2^16 clocks = ~1.31 ms at 50 MHz
    parameter WD_MS      = 500,         // IR_TX_OUT fail-safe cutoff (500 ms)
    parameter HB_HALF_MS = 500          // Heartbeat 1 Hz (500 ms on / 500 ms off)
)(
    input  wire       clk,             // 50 MHz On-board Oscillator
    input  wire       rst_n_key,       // On-board Pushbutton (Active-Low)

    // SPI Bus (Connected to ESP32-S3)
    input  wire       spi_sck,
    input  wire       spi_cs_n,
    input  wire       spi_mosi,
    output wire       spi_miso,        // Tri-state output

    // IR Physical Interface
    input  wire       ir_rx_in,        // From KY-022 (Active-Low, do NOT invert)
    output wire       ir_tx_out,       // Active-High -> 2N2222A Base via Rb=390 ohm

    // On-board Telemetry (4x LEDs, Active-Low)
    output wire [3:0] led
);

    localparam integer MS_CLKS = CLK_HZ / 1000;
    localparam integer WD_CLKS = WD_MS * MS_CLKS;
    localparam integer HB_CLKS = HB_HALF_MS * MS_CLKS;
    localparam integer WD_W    = $clog2(WD_CLKS + 1);
    localparam integer HB_W    = $clog2(HB_CLKS + 1);

    //--------------------------------------------------------------------------
    // 1. Power-On Reset & Global Reset Synchronizer
    //--------------------------------------------------------------------------
    reg [POR_BITS-1:0] por_cnt  = {POR_BITS{1'b0}};
    reg                por_done = 1'b0;

    always @(posedge clk) begin
        if (!por_done) begin
            por_cnt <= por_cnt + 1'b1;
            if (&por_cnt) por_done <= 1'b1;
        end
    end

    wire rst_n_int = rst_n_key & por_done;

    reg [1:0] rst_top_sr;
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) rst_top_sr <= 2'b00;
        else            rst_top_sr <= {rst_top_sr[0], 1'b1};
    end
    wire rst_top_n = rst_top_sr[1];

    //--------------------------------------------------------------------------
    // 2. Core & SPI Slave Instantiations
    //--------------------------------------------------------------------------
    wire        cmd_arm_rx, cmd_start_tx, cmd_set_len;
    wire [15:0] cmd_set_len_value;
    wire [10:0] rx_len;
    wire        rx_done, rx_overflow, tx_busy;
    wire        buf_ok, buf_we;
    wire [9:0]  buf_addr;
    wire [15:0] buf_wdata, buf_rdata;
    wire        ir_led;

    ir_core u_core (
        .clk               (clk),
        .rst_n             (rst_n_int),
        .ir_rx_n           (ir_rx_in),
        .ir_led            (ir_led),
        .cmd_arm_rx        (cmd_arm_rx),
        .cmd_start_tx      (cmd_start_tx),
        .cmd_set_len       (cmd_set_len),
        .cmd_set_len_value (cmd_set_len_value),
        .rx_len            (rx_len),
        .rx_done           (rx_done),
        .rx_overflow       (rx_overflow),
        .tx_busy           (tx_busy),
        .buf_ok            (buf_ok),
        .buf_we            (buf_we),
        .buf_addr          (buf_addr),
        .buf_wdata         (buf_wdata),
        .buf_rdata         (buf_rdata)
    );

    spi_slave u_spi (
        .clk               (clk),
        .rst_n             (rst_n_int),
        .spi_sck           (spi_sck),
        .spi_cs_n          (spi_cs_n),
        .spi_mosi          (spi_mosi),
        .spi_miso          (spi_miso),
        .cmd_arm_rx        (cmd_arm_rx),
        .cmd_start_tx      (cmd_start_tx),
        .cmd_set_len       (cmd_set_len),
        .cmd_set_len_value (cmd_set_len_value),
        .tx_len            (),                 // Unconnected
        .rx_len            (rx_len),
        .rx_done           (rx_done),
        .tx_busy           (tx_busy),
        .rx_overflow       (rx_overflow),
        .buf_ok            (buf_ok),
        .buf_we            (buf_we),
        .buf_addr          (buf_addr),
        .buf_wdata         (buf_wdata),
        .buf_rdata         (buf_rdata)
    );

    //--------------------------------------------------------------------------
    // 3. IR_TX Fail-Safe Watchdog (500 ms limit)
    //--------------------------------------------------------------------------
    reg [WD_W-1:0] wd_cnt;
    reg            wd_trip;
    reg            ir_out_r;

    always @(posedge clk or negedge rst_top_n) begin
        if (!rst_top_n) begin
            wd_cnt  <= {WD_W{1'b0}};
            wd_trip <= 1'b0;
        end else if (!tx_busy) begin
            wd_cnt  <= {WD_W{1'b0}};
            wd_trip <= 1'b0;
        end else if (!wd_trip) begin
            if (wd_cnt == WD_CLKS - 1) wd_trip <= 1'b1;
            else                       wd_cnt  <= wd_cnt + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_top_n) begin
        if (!rst_top_n) ir_out_r <= 1'b0;
        else            ir_out_r <= ir_led & ~wd_trip;
    end
    assign ir_tx_out = ir_out_r;

    //--------------------------------------------------------------------------
    // 4. Telemetry: 4 LEDs (Active-Low: 0 = Light Up)
    //    LED[0]: rx_done (Capture success)
    //    LED[1]: tx_busy (Replay in progress)
    //    LED[2]: SPI transfer active (~spi_cs_n)
    //    LED[3]: Heartbeat 1 Hz (FPGA Alive)
    //--------------------------------------------------------------------------
    reg [1:0] cs_sr;
    always @(posedge clk or negedge rst_top_n) begin
        if (!rst_top_n) cs_sr <= 2'b11;
        else            cs_sr <= {cs_sr[0], spi_cs_n};
    end
    wire spi_active = ~cs_sr[1];

    reg [HB_W-1:0] hb_cnt;
    reg            hb;
    always @(posedge clk or negedge rst_top_n) begin
        if (!rst_top_n) begin
            hb_cnt <= {HB_W{1'b0}};
            hb     <= 1'b0;
        end else if (hb_cnt == HB_CLKS - 1) begin
            hb_cnt <= {HB_W{1'b0}};
            hb     <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    // Invert outputs because Cyclone IV board LEDs are typically Active-Low
    assign led[0] = ~rx_done;
    assign led[1] = ~tx_busy;
    assign led[2] = ~spi_active;
    assign led[3] = ~hb;

endmodule
