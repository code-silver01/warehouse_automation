
`timescale 1ns / 1ps

module traffic_controller (
    input  wire        clk,
    input  wire        reset,

    // Current positions of vehicles 0, 1 and 2
    input  wire [14:0] vehicle_positions,

    // One request bit per vehicle
    input  wire [2:0]  move_req,

    // Proposed next positions
    input  wire [14:0] next_positions,

    // Movement permission and denial indicators
    output reg  [2:0]  move_grant,
    output wire [2:0]  conflict
);

    // Round-robin starting vehicle:
    // 0 -> 1 -> 2 -> 0
    reg [1:0] rr_ptr;

    reg [2:0] grant_temp;

    reg [4:0] current_cell;
    reg [4:0] target_cell;

    reg [4:0] reserved_cells [0:2];

    reg occupied;
    reg already_reserved;

    integer i;
    integer j;
    integer idx;
    integer reserved_count;
    integer last_granted;

    // A requested move that is not granted is a conflict.
    assign conflict = move_req & (~move_grant);

    // ---------------------------------------------------------
    // COMBINATIONAL ARBITRATION
    // ---------------------------------------------------------
    always @(*) begin

        grant_temp = 3'b000;

        reserved_count = 0;
        last_granted = -1;

        current_cell = 5'd0;
        target_cell  = 5'd0;

        occupied = 1'b0;
        already_reserved = 1'b0;

        for (i = 0; i < 3; i = i + 1)
            reserved_cells[i] = 5'd31;

        // Examine vehicles in round-robin order.
        for (i = 0; i < 3; i = i + 1) begin

            idx = (rr_ptr + i) % 3;

            if (move_req[idx]) begin

                current_cell =
                    vehicle_positions[(idx * 5) +: 5];

                target_cell =
                    next_positions[(idx * 5) +: 5];

                occupied = 1'b0;
                already_reserved = 1'b0;

                // A vehicle cannot request a no-op move.
                if (target_cell == current_cell)
                    occupied = 1'b1;

                // Check whether another vehicle currently
                // occupies the requested destination.
                for (j = 0; j < 3; j = j + 1) begin

                    if (j != idx) begin

                        if (target_cell ==
                            vehicle_positions[(j * 5) +: 5])
                            occupied = 1'b1;

                    end
                end

                // Check whether an earlier vehicle in this
                // arbitration cycle has reserved this cell.
                for (j = 0; j < 3; j = j + 1) begin

                    if (j < reserved_count) begin

                        if (target_cell == reserved_cells[j])
                            already_reserved = 1'b1;

                    end
                end

                // Grant only if destination is safe and free.
                if (!occupied && !already_reserved) begin

                    grant_temp[idx] = 1'b1;

                    reserved_cells[reserved_count] = target_cell;
                    reserved_count = reserved_count + 1;

                    last_granted = idx;

                end

            end

        end

    end

    // ---------------------------------------------------------
    // SEQUENTIAL ROUND-ROBIN POINTER UPDATE
    // ---------------------------------------------------------
    always @(posedge clk or posedge reset) begin

        if (reset) begin

            rr_ptr <= 2'd0;
            move_grant <= 3'b000;

        end

        else begin

            move_grant <= grant_temp;

            // Next arbitration starts after the last vehicle
            // that received a grant.
            if (last_granted == 0)
                rr_ptr <= 2'd1;

            else if (last_granted == 1)
                rr_ptr <= 2'd2;

            else if (last_granted == 2)
                rr_ptr <= 2'd0;

        end

    end

endmodule
