
module occupancy_manager (
    input wire [14:0] vehicle_positions,

    output reg [24:0] occupancy_mask,
    output reg        collision_detected,
    output reg [2:0]  invalid_position
);

    integer i;
    integer j;

    reg [4:0] current_position;

    always @(*) begin

        occupancy_mask = 25'b0;
        collision_detected = 1'b0;
        invalid_position = 3'b000;

        // Generate occupancy map
        for (i = 0; i < 3; i = i + 1) begin

            current_position = vehicle_positions[(i * 5) +: 5];

            if (current_position <= 5'd24) begin

                if (occupancy_mask[current_position] == 1'b1)
                    collision_detected = 1'b1;

                occupancy_mask[current_position] = 1'b1;
            end
            else begin
                invalid_position[i] = 1'b1;
            end
        end
    end

endmodule
