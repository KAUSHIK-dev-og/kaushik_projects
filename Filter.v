// -----------------------------------------------------------------------
// activity_detector
// Watches the incoming sample stream and flags "real signal" vs "quiet"
// by comparing the change between consecutive VALID samples to a threshold.
//
// FIX vs original: the original registered `diff` before comparing it,
// which added a redundant extra clock of latency (abs_diff was compared
// one cycle later than it needed to be). Here abs_diff is combinational
// off of prev_sample/sample_in, so compute_enable reflects the new
// sample with exactly one cycle of latency (the minimum possible, since
// prev_sample must be registered).
// -----------------------------------------------------------------------
module activity_detector #(
    parameter DATA_WIDTH = 16,
    parameter THRESHOLD  = 16'd20
)(
    input                          clk,
    input                          rst,
    input                          sample_valid,   // pulses when sample_in is a new sample
    input  signed [DATA_WIDTH-1:0] sample_in,
    output reg                     compute_enable  // 1 = this sample is "active", worth filtering
);

    reg signed [DATA_WIDTH-1:0] prev_sample;
    wire signed [DATA_WIDTH:0]  diff;
    wire        [DATA_WIDTH:0]  abs_diff;

    assign diff     = sample_in - prev_sample;
    assign abs_diff = diff[DATA_WIDTH] ? (~diff + 1'b1) : diff; // two's complement abs()

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prev_sample    <= 0;
            compute_enable <= 1'b0;
        end
        else if (sample_valid) begin
            prev_sample    <= sample_in;
            compute_enable <= (abs_diff >= THRESHOLD);
        end
    end

endmodule// -----------------------------------------------------------------------
// sample_delay_line
// Holds the last TAPS input samples (the FIR's sliding window).
//
// BUG FIXED: the original only shifted in a new sample when
// compute_enable was high. That means during "quiet" (gated) periods,
// incoming samples were silently DROPPED instead of buffered. The next
// time activity resumed, the delay line would contain stale samples
// from before the gap instead of the true last-8 samples -- so the
// very first few filtered outputs after a quiet period would be wrong.
//
// The fix: the delay line always shifts on every valid sample,
// regardless of compute_enable. Only the expensive bit-serial DA
// compute engine (FSM + LUT + accumulator) is clock-gated -- that is
// where almost all of the power actually goes (16 cycles of activity
// per output vs. 1 register update here), so gating just the compute
// engine preserves the power-saving novelty while keeping the filter
// mathematically correct.
// -----------------------------------------------------------------------
module sample_delay_line #(
    parameter DATA_WIDTH = 16,
    parameter TAPS       = 8
)(
    input                          clk,
    input                          rst,
    input                          sample_valid,
    input  signed [DATA_WIDTH-1:0] sample_in,

    output signed [DATA_WIDTH-1:0] tap0,
    output signed [DATA_WIDTH-1:0] tap1,
    output signed [DATA_WIDTH-1:0] tap2,
    output signed [DATA_WIDTH-1:0] tap3,
    output signed [DATA_WIDTH-1:0] tap4,
    output signed [DATA_WIDTH-1:0] tap5,
    output signed [DATA_WIDTH-1:0] tap6,
    output signed [DATA_WIDTH-1:0] tap7
);

    reg signed [DATA_WIDTH-1:0] shift_reg [0:TAPS-1];
    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < TAPS; i = i + 1)
                shift_reg[i] <= 0;
        end
        else if (sample_valid) begin
            shift_reg[7] <= shift_reg[6];
            shift_reg[6] <= shift_reg[5];
            shift_reg[5] <= shift_reg[4];
            shift_reg[4] <= shift_reg[3];
            shift_reg[3] <= shift_reg[2];
            shift_reg[2] <= shift_reg[1];
            shift_reg[1] <= shift_reg[0];
            shift_reg[0] <= sample_in;
        end
    end

    assign tap0 = shift_reg[0];
    assign tap1 = shift_reg[1];
    assign tap2 = shift_reg[2];
    assign tap3 = shift_reg[3];
    assign tap4 = shift_reg[4];
    assign tap5 = shift_reg[5];
    assign tap6 = shift_reg[6];
    assign tap7 = shift_reg[7];

endmodule// -----------------------------------------------------------------------
// da_fir_top
// Top level: activity-gated, multiplierless (Distributed Arithmetic)
// 8-tap FIR filter.
//
// CHANGES vs original top level:
//   - added `sample_valid` input: tells the design "sample_in is a new
//     sample this cycle." This decouples the ADC/sample rate from the
//     system clock, which the bit-serial DA engine needs anyway (it
//     takes DATA_WIDTH clock cycles to fully process one sample).
//   - sample_delay_line and activity_detector now run off sample_valid,
//     not compute_enable, so incoming samples are never silently
//     dropped during quiet (gated) periods (see sample_delay_line.v).
//   - control_fsm/shift_accumulator renamed bit_done -> bit_valid and
//     added is_last_bit, fixing the dropped-LSB and missing-sign-bit
//     bugs (see control_fsm.v / shift_accumulator.v for details).
// -----------------------------------------------------------------------
module da_fir_top #(
    parameter DATA_WIDTH = 16,
    parameter ACC_WIDTH  = 32,
    parameter THRESHOLD  = 16'd20
)(
    input                          clk,
    input                          rst,
    input                          sample_valid,   // pulse: sample_in is new this cycle
    input  signed [DATA_WIDTH-1:0] sample_in,

    output                         output_valid,   // pulse: filter_out is fresh
    output signed [ACC_WIDTH-1:0]  filter_out
);

    //--------------------------------------------------
    // Internal Signals
    //--------------------------------------------------
    wire compute_enable;

    wire signed [DATA_WIDTH-1:0] tap0, tap1, tap2, tap3, tap4, tap5, tap6, tap7;

    wire [7:0]                   lut_address;
    wire signed [ACC_WIDTH-1:0]  lut_data;

    wire        bit_valid;
    wire        is_last_bit;
    wire [4:0]  bit_index;

    wire signed [ACC_WIDTH-1:0] accumulator;

    //--------------------------------------------------
    // Activity Detector
    //--------------------------------------------------
    activity_detector #(
        .DATA_WIDTH(DATA_WIDTH),
        .THRESHOLD(THRESHOLD)
    ) ACTIVITY (
        .clk(clk),
        .rst(rst),
        .sample_valid(sample_valid),
        .sample_in(sample_in),
        .compute_enable(compute_enable)
    );

    //--------------------------------------------------
    // Sample Delay Line (always captures every valid sample)
    //--------------------------------------------------
    sample_delay_line #(
        .DATA_WIDTH(DATA_WIDTH),
        .TAPS(8)
    ) DELAY (
        .clk(clk),
        .rst(rst),
        .sample_valid(sample_valid),
        .sample_in(sample_in),
        .tap0(tap0), .tap1(tap1), .tap2(tap2), .tap3(tap3),
        .tap4(tap4), .tap5(tap5), .tap6(tap6), .tap7(tap7)
    );

    //--------------------------------------------------
    // Current Bit-Plane Address (one bit from each tap)
    //--------------------------------------------------
    assign lut_address = { tap7[bit_index], tap6[bit_index], tap5[bit_index], tap4[bit_index],
                            tap3[bit_index], tap2[bit_index], tap1[bit_index], tap0[bit_index] };

    //--------------------------------------------------
    // DA LUT
    //--------------------------------------------------
    da_lut #(
        .DATA_WIDTH(DATA_WIDTH),
        .LUT_WIDTH(ACC_WIDTH)
    ) LUT (
        .address(lut_address),
        .lut_out(lut_data)
    );

    //--------------------------------------------------
    // Control FSM (activity-gated: only starts on compute_enable)
    //--------------------------------------------------
    control_fsm #(
        .BIT_WIDTH(DATA_WIDTH)
    ) FSM (
        .clk(clk),
        .rst(rst),
        .compute_enable(compute_enable),
        .bit_index(bit_index),
        .bit_valid(bit_valid),
        .is_last_bit(is_last_bit),
        .output_valid(output_valid)
    );

    //--------------------------------------------------
    // Shift Accumulator
    //--------------------------------------------------
    shift_accumulator #(
        .ACC_WIDTH(ACC_WIDTH)
    ) ACC (
        .clk(clk),
        .rst(rst),
        .bit_valid(bit_valid),
        .is_last_bit(is_last_bit),
        .lut_data(lut_data),
        .accumulator(accumulator)
    );

    //--------------------------------------------------
    // Output Register
    //--------------------------------------------------
    output_register #(
        .OUT_WIDTH(ACC_WIDTH)
    ) OUTREG (
        .clk(clk),
        .rst(rst),
        .output_valid(output_valid),
        .accumulator(accumulator),
        .filter_out(filter_out)
    );

endmodule// -----------------------------------------------------------------------
// da_lut
// Precomputed Distributed-Arithmetic lookup table.
//
// FIX: the original file only had entries for addresses 0-7 filled in
// (everything else, including these 8, was left as placeholder 0) --
// so the filter could never produce real output no matter how correct
// the rest of the datapath was. This version has ALL 256 entries
// (2^8, one for every combination of the 8 taps' current bit) filled
// with real precomputed sums.
//
// Coefficients used (example 8-tap low-pass FIR, generated with
// scipy.signal.firwin(8, 0.25), quantized to Q1.14 fixed point,
// i.e. multiplied by 2^14 and rounded):
//   h0..h7 = 58, 624, 2638, 4871, 4871, 2638, 624, 58
//
// LUT[address] = sum_i  address_bit_i * h_i   for i = 0..7
// (address_bit_i is tap_i's current bit, per da_fir_top's
// lut_address construction: {tap7,...,tap1,tap0})
//
// To regenerate this table for YOUR OWN coefficients, see the Python
// snippet in README_DA_FIR_fixes.md included alongside this file.
// -----------------------------------------------------------------------
module da_lut #(
    parameter DATA_WIDTH = 16,
    parameter LUT_WIDTH  = 32
)(
    input      [7:0]                address,
    output reg signed [LUT_WIDTH-1:0] lut_out
);

    always @(*) begin
        case (address)
        8'd  0: lut_out = 32'sd0;
        8'd  1: lut_out = 32'sd58;
        8'd  2: lut_out = 32'sd624;
        8'd  3: lut_out = 32'sd682;
        8'd  4: lut_out = 32'sd2638;
        8'd  5: lut_out = 32'sd2696;
        8'd  6: lut_out = 32'sd3262;
        8'd  7: lut_out = 32'sd3320;
        8'd  8: lut_out = 32'sd4871;
        8'd  9: lut_out = 32'sd4929;
        8'd 10: lut_out = 32'sd5495;
        8'd 11: lut_out = 32'sd5553;
        8'd 12: lut_out = 32'sd7509;
        8'd 13: lut_out = 32'sd7567;
        8'd 14: lut_out = 32'sd8133;
        8'd 15: lut_out = 32'sd8191;
        8'd 16: lut_out = 32'sd4871;
        8'd 17: lut_out = 32'sd4929;
        8'd 18: lut_out = 32'sd5495;
        8'd 19: lut_out = 32'sd5553;
        8'd 20: lut_out = 32'sd7509;
        8'd 21: lut_out = 32'sd7567;
        8'd 22: lut_out = 32'sd8133;
        8'd 23: lut_out = 32'sd8191;
        8'd 24: lut_out = 32'sd9742;
        8'd 25: lut_out = 32'sd9800;
        8'd 26: lut_out = 32'sd10366;
        8'd 27: lut_out = 32'sd10424;
        8'd 28: lut_out = 32'sd12380;
        8'd 29: lut_out = 32'sd12438;
        8'd 30: lut_out = 32'sd13004;
        8'd 31: lut_out = 32'sd13062;
        8'd 32: lut_out = 32'sd2638;
        8'd 33: lut_out = 32'sd2696;
        8'd 34: lut_out = 32'sd3262;
        8'd 35: lut_out = 32'sd3320;
        8'd 36: lut_out = 32'sd5276;
        8'd 37: lut_out = 32'sd5334;
        8'd 38: lut_out = 32'sd5900;
        8'd 39: lut_out = 32'sd5958;
        8'd 40: lut_out = 32'sd7509;
        8'd 41: lut_out = 32'sd7567;
        8'd 42: lut_out = 32'sd8133;
        8'd 43: lut_out = 32'sd8191;
        8'd 44: lut_out = 32'sd10147;
        8'd 45: lut_out = 32'sd10205;
        8'd 46: lut_out = 32'sd10771;
        8'd 47: lut_out = 32'sd10829;
        8'd 48: lut_out = 32'sd7509;
        8'd 49: lut_out = 32'sd7567;
        8'd 50: lut_out = 32'sd8133;
        8'd 51: lut_out = 32'sd8191;
        8'd 52: lut_out = 32'sd10147;
        8'd 53: lut_out = 32'sd10205;
        8'd 54: lut_out = 32'sd10771;
        8'd 55: lut_out = 32'sd10829;
        8'd 56: lut_out = 32'sd12380;
        8'd 57: lut_out = 32'sd12438;
        8'd 58: lut_out = 32'sd13004;
        8'd 59: lut_out = 32'sd13062;
        8'd 60: lut_out = 32'sd15018;
        8'd 61: lut_out = 32'sd15076;
        8'd 62: lut_out = 32'sd15642;
        8'd 63: lut_out = 32'sd15700;
        8'd 64: lut_out = 32'sd624;
        8'd 65: lut_out = 32'sd682;
        8'd 66: lut_out = 32'sd1248;
        8'd 67: lut_out = 32'sd1306;
        8'd 68: lut_out = 32'sd3262;
        8'd 69: lut_out = 32'sd3320;
        8'd 70: lut_out = 32'sd3886;
        8'd 71: lut_out = 32'sd3944;
        8'd 72: lut_out = 32'sd5495;
        8'd 73: lut_out = 32'sd5553;
        8'd 74: lut_out = 32'sd6119;
        8'd 75: lut_out = 32'sd6177;
        8'd 76: lut_out = 32'sd8133;
        8'd 77: lut_out = 32'sd8191;
        8'd 78: lut_out = 32'sd8757;
        8'd 79: lut_out = 32'sd8815;
        8'd 80: lut_out = 32'sd5495;
        8'd 81: lut_out = 32'sd5553;
        8'd 82: lut_out = 32'sd6119;
        8'd 83: lut_out = 32'sd6177;
        8'd 84: lut_out = 32'sd8133;
        8'd 85: lut_out = 32'sd8191;
        8'd 86: lut_out = 32'sd8757;
        8'd 87: lut_out = 32'sd8815;
        8'd 88: lut_out = 32'sd10366;
        8'd 89: lut_out = 32'sd10424;
        8'd 90: lut_out = 32'sd10990;
        8'd 91: lut_out = 32'sd11048;
        8'd 92: lut_out = 32'sd13004;
        8'd 93: lut_out = 32'sd13062;
        8'd 94: lut_out = 32'sd13628;
        8'd 95: lut_out = 32'sd13686;
        8'd 96: lut_out = 32'sd3262;
        8'd 97: lut_out = 32'sd3320;
        8'd 98: lut_out = 32'sd3886;
        8'd 99: lut_out = 32'sd3944;
        8'd100: lut_out = 32'sd5900;
        8'd101: lut_out = 32'sd5958;
        8'd102: lut_out = 32'sd6524;
        8'd103: lut_out = 32'sd6582;
        8'd104: lut_out = 32'sd8133;
        8'd105: lut_out = 32'sd8191;
        8'd106: lut_out = 32'sd8757;
        8'd107: lut_out = 32'sd8815;
        8'd108: lut_out = 32'sd10771;
        8'd109: lut_out = 32'sd10829;
        8'd110: lut_out = 32'sd11395;
        8'd111: lut_out = 32'sd11453;
        8'd112: lut_out = 32'sd8133;
        8'd113: lut_out = 32'sd8191;
        8'd114: lut_out = 32'sd8757;
        8'd115: lut_out = 32'sd8815;
        8'd116: lut_out = 32'sd10771;
        8'd117: lut_out = 32'sd10829;
        8'd118: lut_out = 32'sd11395;
        8'd119: lut_out = 32'sd11453;
        8'd120: lut_out = 32'sd13004;
        8'd121: lut_out = 32'sd13062;
        8'd122: lut_out = 32'sd13628;
        8'd123: lut_out = 32'sd13686;
        8'd124: lut_out = 32'sd15642;
        8'd125: lut_out = 32'sd15700;
        8'd126: lut_out = 32'sd16266;
        8'd127: lut_out = 32'sd16324;
        8'd128: lut_out = 32'sd58;
        8'd129: lut_out = 32'sd116;
        8'd130: lut_out = 32'sd682;
        8'd131: lut_out = 32'sd740;
        8'd132: lut_out = 32'sd2696;
        8'd133: lut_out = 32'sd2754;
        8'd134: lut_out = 32'sd3320;
        8'd135: lut_out = 32'sd3378;
        8'd136: lut_out = 32'sd4929;
        8'd137: lut_out = 32'sd4987;
        8'd138: lut_out = 32'sd5553;
        8'd139: lut_out = 32'sd5611;
        8'd140: lut_out = 32'sd7567;
        8'd141: lut_out = 32'sd7625;
        8'd142: lut_out = 32'sd8191;
        8'd143: lut_out = 32'sd8249;
        8'd144: lut_out = 32'sd4929;
        8'd145: lut_out = 32'sd4987;
        8'd146: lut_out = 32'sd5553;
        8'd147: lut_out = 32'sd5611;
        8'd148: lut_out = 32'sd7567;
        8'd149: lut_out = 32'sd7625;
        8'd150: lut_out = 32'sd8191;
        8'd151: lut_out = 32'sd8249;
        8'd152: lut_out = 32'sd9800;
        8'd153: lut_out = 32'sd9858;
        8'd154: lut_out = 32'sd10424;
        8'd155: lut_out = 32'sd10482;
        8'd156: lut_out = 32'sd12438;
        8'd157: lut_out = 32'sd12496;
        8'd158: lut_out = 32'sd13062;
        8'd159: lut_out = 32'sd13120;
        8'd160: lut_out = 32'sd2696;
        8'd161: lut_out = 32'sd2754;
        8'd162: lut_out = 32'sd3320;
        8'd163: lut_out = 32'sd3378;
        8'd164: lut_out = 32'sd5334;
        8'd165: lut_out = 32'sd5392;
        8'd166: lut_out = 32'sd5958;
        8'd167: lut_out = 32'sd6016;
        8'd168: lut_out = 32'sd7567;
        8'd169: lut_out = 32'sd7625;
        8'd170: lut_out = 32'sd8191;
        8'd171: lut_out = 32'sd8249;
        8'd172: lut_out = 32'sd10205;
        8'd173: lut_out = 32'sd10263;
        8'd174: lut_out = 32'sd10829;
        8'd175: lut_out = 32'sd10887;
        8'd176: lut_out = 32'sd7567;
        8'd177: lut_out = 32'sd7625;
        8'd178: lut_out = 32'sd8191;
        8'd179: lut_out = 32'sd8249;
        8'd180: lut_out = 32'sd10205;
        8'd181: lut_out = 32'sd10263;
        8'd182: lut_out = 32'sd10829;
        8'd183: lut_out = 32'sd10887;
        8'd184: lut_out = 32'sd12438;
        8'd185: lut_out = 32'sd12496;
        8'd186: lut_out = 32'sd13062;
        8'd187: lut_out = 32'sd13120;
        8'd188: lut_out = 32'sd15076;
        8'd189: lut_out = 32'sd15134;
        8'd190: lut_out = 32'sd15700;
        8'd191: lut_out = 32'sd15758;
        8'd192: lut_out = 32'sd682;
        8'd193: lut_out = 32'sd740;
        8'd194: lut_out = 32'sd1306;
        8'd195: lut_out = 32'sd1364;
        8'd196: lut_out = 32'sd3320;
        8'd197: lut_out = 32'sd3378;
        8'd198: lut_out = 32'sd3944;
        8'd199: lut_out = 32'sd4002;
        8'd200: lut_out = 32'sd5553;
        8'd201: lut_out = 32'sd5611;
        8'd202: lut_out = 32'sd6177;
        8'd203: lut_out = 32'sd6235;
        8'd204: lut_out = 32'sd8191;
        8'd205: lut_out = 32'sd8249;
        8'd206: lut_out = 32'sd8815;
        8'd207: lut_out = 32'sd8873;
        8'd208: lut_out = 32'sd5553;
        8'd209: lut_out = 32'sd5611;
        8'd210: lut_out = 32'sd6177;
        8'd211: lut_out = 32'sd6235;
        8'd212: lut_out = 32'sd8191;
        8'd213: lut_out = 32'sd8249;
        8'd214: lut_out = 32'sd8815;
        8'd215: lut_out = 32'sd8873;
        8'd216: lut_out = 32'sd10424;
        8'd217: lut_out = 32'sd10482;
        8'd218: lut_out = 32'sd11048;
        8'd219: lut_out = 32'sd11106;
        8'd220: lut_out = 32'sd13062;
        8'd221: lut_out = 32'sd13120;
        8'd222: lut_out = 32'sd13686;
        8'd223: lut_out = 32'sd13744;
        8'd224: lut_out = 32'sd3320;
        8'd225: lut_out = 32'sd3378;
        8'd226: lut_out = 32'sd3944;
        8'd227: lut_out = 32'sd4002;
        8'd228: lut_out = 32'sd5958;
        8'd229: lut_out = 32'sd6016;
        8'd230: lut_out = 32'sd6582;
        8'd231: lut_out = 32'sd6640;
        8'd232: lut_out = 32'sd8191;
        8'd233: lut_out = 32'sd8249;
        8'd234: lut_out = 32'sd8815;
        8'd235: lut_out = 32'sd8873;
        8'd236: lut_out = 32'sd10829;
        8'd237: lut_out = 32'sd10887;
        8'd238: lut_out = 32'sd11453;
        8'd239: lut_out = 32'sd11511;
        8'd240: lut_out = 32'sd8191;
        8'd241: lut_out = 32'sd8249;
        8'd242: lut_out = 32'sd8815;
        8'd243: lut_out = 32'sd8873;
        8'd244: lut_out = 32'sd10829;
        8'd245: lut_out = 32'sd10887;
        8'd246: lut_out = 32'sd11453;
        8'd247: lut_out = 32'sd11511;
        8'd248: lut_out = 32'sd13062;
        8'd249: lut_out = 32'sd13120;
        8'd250: lut_out = 32'sd13686;
        8'd251: lut_out = 32'sd13744;
        8'd252: lut_out = 32'sd15700;
        8'd253: lut_out = 32'sd15758;
        8'd254: lut_out = 32'sd16324;
        8'd255: lut_out = 32'sd16382;
            default: lut_out = {LUT_WIDTH{1'b0}};
        endcase
    end

endmodule// -----------------------------------------------------------------------
// control_fsm
// Sequences the bit-serial DA computation: walks bit_index from 0 (LSB)
// to BIT_WIDTH-1 (sign bit), tells the accumulator when to add/subtract,
// and flags is_last_bit so the accumulator knows to subtract (two's
// complement sign weight) instead of add on the final bit.
//
// BUG FIXED: in the original, `bit_done` was a REGISTERED signal that
// lagged bit_index by one extra cycle relative to when it actually went
// high after leaving IDLE. That meant the very first bit processed
// (bit_index == 0, the LSB) was addressed in the LUT but bit_done was
// still 0 during that cycle -- so shift_accumulator never added it in.
// Every single filtered output was silently missing its LSB
// contribution. Renaming to `bit_valid` and asserting it in the SAME
// cycle bit_index becomes valid fixes this.
// -----------------------------------------------------------------------
module control_fsm #(
    parameter BIT_WIDTH = 16
)(
    input             clk,
    input             rst,
    input             compute_enable,

    output reg [4:0]  bit_index,
    output reg        bit_valid,     // bit_index/lut_address valid THIS cycle -> accumulate now
    output reg        is_last_bit,   // this cycle's bit is the sign bit -> subtract, not add
    output reg        output_valid   // pulses for 1 cycle once the sample is fully computed
);

    localparam IDLE    = 1'b0;
    localparam COMPUTE = 1'b1;

    reg state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= IDLE;
            bit_index    <= 5'd0;
            bit_valid    <= 1'b0;
            is_last_bit  <= 1'b0;
            output_valid <= 1'b0;
        end
        else begin
            output_valid <= 1'b0; // default: only pulses in the branch below

            case (state)

                IDLE: begin
                    bit_valid   <= 1'b0;
                    is_last_bit <= 1'b0;
                    if (compute_enable) begin
                        bit_index   <= 5'd0;
                        bit_valid   <= 1'b1;                       // bit 0 valid immediately
                        is_last_bit <= (BIT_WIDTH == 1);
                        state       <= COMPUTE;
                    end
                end

                COMPUTE: begin
                    if (is_last_bit) begin
                        // this cycle processed the sign bit; sample is done
                        bit_valid    <= 1'b0;
                        is_last_bit  <= 1'b0;
                        output_valid <= 1'b1;
                        state        <= IDLE;
                    end
                    else begin
                        bit_index   <= bit_index + 1'b1;
                        bit_valid   <= 1'b1;
                        is_last_bit <= (bit_index + 1'b1 == BIT_WIDTH - 1);
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule// -----------------------------------------------------------------------
// shift_accumulator
// Classic LSB-first bit-serial DA recurrence:
//     acc <= (acc >>> 1) + LUT[bit_k]      for k = 0 .. N-2
//     acc <= (acc >>> 1) - LUT[bit_{N-1}]  for the final sign bit (k = N-1)
//
// BUG FIXED: the original ALWAYS added, never subtracted. Because each
// tap is a two's-complement signed number, its MSB (sign bit) carries a
// NEGATIVE weight (-2^(N-1)), not a positive one. Always adding meant
// every negative-going sample contribution was computed with the wrong
// sign, so the filter output would only be correct for one specific
// combination of sample signs -- effectively broken for real signals.
// -----------------------------------------------------------------------
module shift_accumulator #(
    parameter ACC_WIDTH = 32
)(
    input                          clk,
    input                          rst,
    input                          bit_valid,
    input                          is_last_bit,
    input  signed [ACC_WIDTH-1:0]  lut_data,

    output reg signed [ACC_WIDTH-1:0] accumulator
);

    always @(posedge clk or posedge rst) begin
        if (rst)
            accumulator <= 0;
        else if (bit_valid) begin
            if (is_last_bit)
                accumulator <= (accumulator >>> 1) - lut_data;
            else
                accumulator <= (accumulator >>> 1) + lut_data;
        end
    end

endmodule
module output_register #(
    parameter OUT_WIDTH = 32
)(
    input                            clk,
    input                            rst,
    input                            output_valid,
    input  signed [OUT_WIDTH-1:0]    accumulator,

    output reg signed [OUT_WIDTH-1:0] filter_out
);

    always @(posedge clk or posedge rst) begin
        if (rst)
            filter_out <= 0;
        else if (output_valid)
            filter_out <= accumulator;
    end

endmodule
