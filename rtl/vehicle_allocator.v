
module vehicle_allocator (
    input wire clk,
    input wire reset,

    input wire order_valid,
    input wire [4:0] pickup_cell,
    input wire [4:0] destination_cell,
    input wire [1:0] order_priority,

    // Each vehicle position occupies 5 bits:
    // [4:0]   = Vehicle 0
    // [9:5]   = Vehicle 1
    // [14:10] = Vehicle 2
    input wire [14:0] vehicle_positions,

    // 1 = busy, 0 = available
    input wire [2:0] vehicle_busy,

    output wire order_ready,

    output reg alloc_valid,
    output reg [1:0] alloc_vehicle,
    output reg [4:0] alloc_pickup,
    output reg [4:0] alloc_destination,
    output reg [1:0] alloc_priority
);

    localparam IDLE  = 1'b0;
    localparam ISSUE = 1'b1;

    reg state;

    // Round-robin starting vehicle for tie-breaking
    reg [1:0] round_robin_ptr;

    reg candidate_found;
    reg [1:0] candidate_id;

    integer best_distance;
    integer current_distance;
    integer offset;
    integer candidate_index;

    // -----------------------------------------
    // Manhattan distance between two grid cells
    // -----------------------------------------

    function integer manhattan_distance;
        input [4:0] cell_a;
        input [4:0] cell_b;

        integer row_a;
        integer col_a;
        integer row_b;
        integer col_b;
        integer row_diff;
        integer col_diff;

        begin
            row_a = cell_a / 5;
            col_a = cell_a % 5;

            row_b = cell_b / 5;
            col_b = cell_b % 5;

            row_diff = row_a - row_b;
            col_diff = col_a - col_b;

            if (row_diff < 0)
                row_diff = -row_diff;

            if (col_diff < 0)
                col_diff = -col_diff;

            manhattan_distance = row_diff + col_diff;
        end
    endfunction

    // -----------------------------------------
    // Select vehicle position using its ID
    // -----------------------------------------

    function [4:0] get_vehicle_position;
        input [1:0] vehicle_id;

        begin
            case (vehicle_id)

                2'd0:
                    get_vehicle_position = vehicle_positions[4:0];

                2'd1:
                    get_vehicle_position = vehicle_positions[9:5];

                2'd2:
                    get_vehicle_position = vehicle_positions[14:10];

                default:
                    get_vehicle_position = 5'd0;

            endcase
        end
    endfunction

    // -----------------------------------------
    // Combinational nearest-vehicle selection
    // -----------------------------------------

    always @(*) begin

        candidate_found = 1'b0;
        candidate_id = 2'd0;

        best_distance = 1000;
        current_distance = 1000;

        candidate_index = 0;

        for (offset = 0; offset < 3; offset = offset + 1) begin

            candidate_index = round_robin_ptr + offset;

            if (candidate_index >= 3)
                candidate_index = candidate_index - 3;

            if (!vehicle_busy[candidate_index]) begin

                current_distance = manhattan_distance(
                    get_vehicle_position(candidate_index[1:0]),
                    pickup_cell
                );

                if (current_distance < best_distance) begin

                    best_distance = current_distance;
                    candidate_id = candidate_index[1:0];
                    candidate_found = 1'b1;

                end

            end

        end

    end

    // -----------------------------------------
    // Order handshake
    // -----------------------------------------

    assign order_ready =
        (state == IDLE) && candidate_found;

    // -----------------------------------------
    // Allocation FSM
    // -----------------------------------------

    always @(posedge clk or posedge reset) begin

        if (reset) begin

            state <= IDLE;

            round_robin_ptr <= 2'd0;

            alloc_valid <= 1'b0;
            alloc_vehicle <= 2'd0;

            alloc_pickup <= 5'd0;
            alloc_destination <= 5'd0;
            alloc_priority <= 2'd0;

        end

        else begin

            // Default: allocation-valid is a one-cycle pulse
            alloc_valid <= 1'b0;

            case (state)

                // ---------------------------------
                // IDLE: Wait for an available vehicle
                // ---------------------------------

                IDLE: begin

                    if (order_valid && candidate_found) begin

                        alloc_vehicle <= candidate_id;

                        alloc_pickup <= pickup_cell;
                        alloc_destination <= destination_cell;
                        alloc_priority <= order_priority;

                        // Advance round-robin pointer
                        if (candidate_id == 2'd2)
                            round_robin_ptr <= 2'd0;
                        else
                            round_robin_ptr <= candidate_id + 1'b1;

                        state <= ISSUE;

                    end

                end

                // ---------------------------------
                // ISSUE: Notify the selected vehicle
                // ---------------------------------

                ISSUE: begin

                    alloc_valid <= 1'b1;

                    state <= IDLE;

                end

                default: begin

                    state <= IDLE;

                end

            endcase

        end

    end

endmodule
