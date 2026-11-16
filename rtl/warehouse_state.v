//=====================================================================
// warehouse_state.v
// Holds a 25-bit blocked mask and a 25-bit occupied mask.
// One cell can be updated per clock cycle per mask.
//=====================================================================
module warehouse_state (
    input  wire        clk,
    input  wire        reset,

    // Blocked-cell update interface (sensors / manual)
    input  wire        set_blocked_valid,
    input  wire [4:0]  set_blocked_cell,
    input  wire        set_blocked_value,   // 1 = block, 0 = unblock

    // Occupied-cell update interface (traffic controller later)
    input  wire        set_occupied_valid,
    input  wire [4:0]  set_occupied_cell,
    input  wire        set_occupied_value,

    // Bulk clear
    input  wire        clear_all,

    output reg  [24:0] blocked_mask,
    output reg  [24:0] occupied_mask
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            blocked_mask  <= 25'd0;
            occupied_mask <= 25'd0;
        end
        else if (clear_all) begin
            blocked_mask  <= 25'd0;
            occupied_mask <= 25'd0;
        end
        else begin
            if (set_blocked_valid && (set_blocked_cell < 5'd25))
                blocked_mask[set_blocked_cell] <= set_blocked_value;

            if (set_occupied_valid && (set_occupied_cell < 5'd25))
                occupied_mask[set_occupied_cell] <= set_occupied_value;
        end
    end

endmodule