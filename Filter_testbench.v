`timescale 1ns/1ps

module testbench;

    localparam DATA_WIDTH = 16;
    localparam ACC_WIDTH  = 32;
    localparam N_SAMPLES  = 28;
    localparam N_EXPECTED = 5;

    reg                          clk;
    reg                          rst;
    reg                          sample_valid;
    reg  signed [DATA_WIDTH-1:0] sample_in;
    wire                         output_valid;
    wire signed [ACC_WIDTH-1:0]  filter_out;

    integer sample_idx;
    integer exp_idx;
    integer errors;

    reg signed [DATA_WIDTH-1:0] samples [0:N_SAMPLES-1];
    reg signed [ACC_WIDTH-1:0]  expected_out [0:N_EXPECTED-1];

    da_fir_top #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH),
        .THRESHOLD(16'd20)
    ) DUT (
        .clk(clk),
        .rst(rst),
        .sample_valid(sample_valid),
        .sample_in(sample_in),
        .output_valid(output_valid),
        .filter_out(filter_out)
    );

    // 10 ns clock period
    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        samples[0]  = 0;
                samples[1]  = 3;
        samples[2]  = 4;
        samples[3]  = 0;
        samples[4]  = -5;
        samples[5]  = -5;
        samples[6]  = -4;
        samples[7]  = 2;
        samples[8]  = 1;
        samples[9]  = 4;
        samples[10]  = -3;
        samples[11]  = -1;
        samples[12]  = 500;
        samples[13]  = 2000;
        samples[14]  = 6000;
        samples[15]  = 8000;
        samples[16]  = 8000;
        samples[17]  = 8000;
        samples[18]  = 8000;
        samples[19]  = 8000;
        samples[20]  = 0;
        samples[21]  = -3;
        samples[22]  = -1;
        samples[23]  = 5;
        samples[24]  = -3;
        samples[25]  = -1;
        samples[26]  = 2;
        samples[27]  = 2;

                expected_out[0] = 1;
        expected_out[1] = 13;
        expected_out[2] = 88;
        expected_out[3] = 363;
        expected_out[4] = 3936;
    end

    // Checker: every time output_valid pulses, compare against the next
    // expected value in order.
    always @(posedge clk) begin
        if (output_valid) begin
            if (exp_idx < N_EXPECTED) begin
                if (filter_out === expected_out[exp_idx]) begin
                    $display("[PASS] output #%0d  filter_out=%0d (matches expected)", exp_idx, filter_out);
                end else begin
                    $display("[FAIL] output #%0d  filter_out=%0d  expected=%0d", exp_idx, filter_out, expected_out[exp_idx]);
                    errors = errors + 1;
                end
                exp_idx = exp_idx + 1;
            end else begin
                $display("[INFO] extra output_valid pulse, filter_out=%0d (no expected value queued)", filter_out);
            end
        end
    end

    initial begin
        rst          = 1;
        sample_valid = 0;
        sample_in    = 0;
        sample_idx   = 0;
        exp_idx      = 0;
        errors       = 0;

        repeat (3) @(posedge clk);
        rst = 0;

        // Feed one sample every 20 clock cycles -- plenty of margin for
        // a 16-cycle DA computation (BIT_WIDTH=16) to finish before the
        // next sample arrives.
        for (sample_idx = 0; sample_idx < N_SAMPLES; sample_idx = sample_idx + 1) begin
            @(posedge clk);
            sample_in    = samples[sample_idx];
            sample_valid = 1;
            @(posedge clk);
            sample_valid = 0;
            repeat (18) @(posedge clk);
        end

        repeat (20) @(posedge clk);

        if (errors == 0)
            $display("\n=== ALL %0d CHECKS PASSED ===", N_EXPECTED);
        else
            $display("\n=== %0d CHECK(S) FAILED ===", errors);

        $finish;
    end

    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, testbench);
    end

endmodule
