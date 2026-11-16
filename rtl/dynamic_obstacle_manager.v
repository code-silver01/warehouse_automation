
module dynamic_obstacle_manager (
    input wire clk,
    input wire reset,

    // Obstacle update interface
    input wire obstacle_update,
    input wire [4:0] obstacle_cell,
    input wire obstacle_blocked,

    // Three vehicle routes
    input wire [374:0] all_paths,

    // Route lengths and current route indices
    input wire [14:0] route_lengths,
    input wire [14:0] route_indices,

    // Outputs
    output reg [24:0] blocked_mask,
    output reg [2:0] reroute_required
);

    integer v;
    integer k;

    reg [4:0] current_cell;

    // Update obstacle map
    always @(posedge clk) begin

        if (reset) begin
            blocked_mask <= 25'b0;
        end

        else if (obstacle_update && obstacle_cell <= 5'd24) begin

            if (obstacle_blocked)
                blocked_mask[obstacle_cell] <= 1'b1;

            else
                blocked_mask[obstacle_cell] <= 1'b0;
        end
    end

    // Check remaining routes for blocked cells
    always @(*) begin

        reroute_required = 3'b000;
        current_cell = 5'd0;

        for (v = 0; v < 3; v = v + 1) begin

            for (k = 0; k < 25; k = k + 1) begin

                if ((k > route_indices[(v * 5) +: 5]) &&
                    (k < route_lengths[(v * 5) +: 5])) begin

                    current_cell =
                        all_paths[(v * 125) + (k * 5) +: 5];

                    if (current_cell <= 5'd24) begin

                        if (blocked_mask[current_cell])
                            reroute_required[v] = 1'b1;

                    end
                end
            end
        end
    end

endmodule
