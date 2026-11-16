
module vehicle_movement_controller (
    input  wire         clk,
    input  wire         reset,
    input  wire [14:0]  initial_positions,
    input  wire [2:0]   route_load,
    input  wire         pause,
    input  wire [14:0]  path_lengths,
    input  wire [374:0] all_paths,

    output wire [14:0]  vehicle_positions,
    output wire [2:0]   vehicle_done,

    output reg  [2:0]   move_req,
    output wire [2:0]   move_grant,
    output wire [2:0]   conflict,

    output wire [374:0] stored_paths,
    output wire [14:0]  stored_lengths,
    output wire [14:0]  route_indices
);

    reg [4:0] pos0, pos1, pos2;
    reg [4:0] index0, index1, index2;
    reg [4:0] len0, len1, len2;

    reg [124:0] path0, path1, path2;
    reg [2:0] route_active;

    reg [4:0] next0, next1, next2;
    reg cooldown;

    wire [14:0] next_positions;

    assign vehicle_positions = {pos2, pos1, pos0};
    assign next_positions = {next2, next1, next0};

    assign stored_paths = {path2, path1, path0};
    assign stored_lengths = {len2, len1, len0};
    assign route_indices = {index2, index1, index0};

    assign vehicle_done[0] =
        route_active[0] && (len0 != 0) &&
        (index0 >= len0 - 1'b1);

    assign vehicle_done[1] =
        route_active[1] && (len1 != 0) &&
        (index1 >= len1 - 1'b1);

    assign vehicle_done[2] =
        route_active[2] && (len2 != 0) &&
        (index2 >= len2 - 1'b1);

    // Suppress movement requests while a route is being replanned.
    always @(*) begin
        move_req = 3'b000;

        next0 = pos0;
        next1 = pos1;
        next2 = pos2;

        if (!cooldown && !pause) begin

            if (route_active[0] &&
                (len0 > 1) &&
                (index0 < len0 - 1'b1)) begin

                move_req[0] = 1'b1;
                next0 = path0[(index0 + 1'b1) * 5 +: 5];
            end

            if (route_active[1] &&
                (len1 > 1) &&
                (index1 < len1 - 1'b1)) begin

                move_req[1] = 1'b1;
                next1 = path1[(index1 + 1'b1) * 5 +: 5];
            end

            if (route_active[2] &&
                (len2 > 1) &&
                (index2 < len2 - 1'b1)) begin

                move_req[2] = 1'b1;
                next2 = path2[(index2 + 1'b1) * 5 +: 5];
            end
        end
    end

    traffic_controller traffic_inst (
        .clk(clk),
        .reset(reset),

        .vehicle_positions(vehicle_positions),
        .move_req(move_req),
        .next_positions(next_positions),

        .move_grant(move_grant),
        .conflict(conflict)
    );

    always @(posedge clk or posedge reset) begin

        if (reset) begin

            pos0 <= initial_positions[4:0];
            pos1 <= initial_positions[9:5];
            pos2 <= initial_positions[14:10];

            index0 <= 5'd0;
            index1 <= 5'd0;
            index2 <= 5'd0;

            len0 <= 5'd0;
            len1 <= 5'd0;
            len2 <= 5'd0;

            path0 <= 125'd0;
            path1 <= 125'd0;
            path2 <= 125'd0;

            route_active <= 3'b000;
            cooldown <= 1'b0;

        end else begin

            // Route loading has highest priority.
            if (|route_load) begin

                cooldown <= 1'b1;

                if (route_load[0]) begin
                    path0 <= all_paths[124:0];
                    len0 <= path_lengths[4:0];
                    index0 <= 5'd0;
                    route_active[0] <= (path_lengths[4:0] != 0);

                    if (path_lengths[4:0] != 0)
                        pos0 <= all_paths[4:0];
                end

                if (route_load[1]) begin
                    path1 <= all_paths[249:125];
                    len1 <= path_lengths[9:5];
                    index1 <= 5'd0;
                    route_active[1] <= (path_lengths[9:5] != 0);

                    if (path_lengths[9:5] != 0)
                        pos1 <= all_paths[129:125];
                end

                if (route_load[2]) begin
                    path2 <= all_paths[374:250];
                    len2 <= path_lengths[14:10];
                    index2 <= 5'd0;
                    route_active[2] <= (path_lengths[14:10] != 0);

                    if (path_lengths[14:10] != 0)
                        pos2 <= all_paths[254:250];
                end

            end else if (pause) begin

                // Ignore any grant registered before pause was asserted.
                cooldown <= 1'b1;

            end else if (cooldown) begin

                // Wait for the traffic controller to clear its old grant.
                if (move_grant == 3'b000)
                    cooldown <= 1'b0;

            end else if (|(move_grant & move_req)) begin

                if (move_grant[0] && move_req[0]) begin
                    pos0 <= next0;
                    index0 <= index0 + 1'b1;
                end

                if (move_grant[1] && move_req[1]) begin
                    pos1 <= next1;
                    index1 <= index1 + 1'b1;
                end

                if (move_grant[2] && move_req[2]) begin
                    pos2 <= next2;
                    index2 <= index2 + 1'b1;
                end

                cooldown <= 1'b1;

            end
        end
    end

endmodule
