
module warehouse_fpga_top (
    input  wire        clk,
    input  wire        reset,

    // Order interface
    input  wire        order_push_valid,
    input  wire [1:0]  order_push_priority,
    input  wire [4:0]  order_push_pickup,
    input  wire [4:0]  order_push_destination,
    output wire        order_push_ready,
    output wire [2:0]  queue_count_out,

    // Initial vehicle positions
    input  wire [14:0] initial_positions,

    // Obstacle interface
    input  wire        set_blocked_valid,
    input  wire [4:0]  set_blocked_cell,
    input  wire        set_blocked_value,

    // Vehicle status
    output wire [14:0] vehicle_positions,
    output wire [2:0]  vehicle_done,
    output wire [2:0]  vehicle_busy,
    output wire        collision_detected,

    // Allocation status
    output wire        alloc_valid,
    output wire [1:0]  alloc_vehicle,
    output wire [4:0]  alloc_pickup,
    output wire [4:0]  alloc_destination,
    output wire [1:0]  alloc_priority,

    // Route status (compact)
    output wire        route_valid,
    output wire [1:0]  route_vehicle,
    output wire        route_leg,
    output wire [4:0]  route_start_cell,
    output wire [4:0]  route_goal_cell,
    output wire [4:0]  route_path_length,
    output wire        route_success,

    // Traffic status
    output wire [2:0]  move_req,
    output wire [2:0]  move_grant,
    output wire [2:0]  movement_conflict,
    output wire [2:0]  reroute_required
);

    // Internal diagnostic signals.
    // These remain inside the FPGA design and do not consume I/O pins.
    wire [124:0] route_path_cells;
    wire [24:0]  occupied_mask_out;
    wire [24:0]  blocked_mask_out;

    warehouse_top u_warehouse (
        .clk(clk),
        .reset(reset),

        .order_push_valid(order_push_valid),
        .order_push_priority(order_push_priority),
        .order_push_pickup(order_push_pickup),
        .order_push_destination(order_push_destination),
        .order_push_ready(order_push_ready),
        .queue_count_out(queue_count_out),

        .initial_positions(initial_positions),

        .set_blocked_valid(set_blocked_valid),
        .set_blocked_cell(set_blocked_cell),
        .set_blocked_value(set_blocked_value),

        .vehicle_positions(vehicle_positions),
        .vehicle_done(vehicle_done),
        .vehicle_busy(vehicle_busy),
        .occupied_mask_out(occupied_mask_out),
        .collision_detected(collision_detected),

        .alloc_valid(alloc_valid),
        .alloc_vehicle(alloc_vehicle),
        .alloc_pickup(alloc_pickup),
        .alloc_destination(alloc_destination),
        .alloc_priority(alloc_priority),

        .route_valid(route_valid),
        .route_vehicle(route_vehicle),
        .route_leg(route_leg),
        .route_start_cell(route_start_cell),
        .route_goal_cell(route_goal_cell),
        .route_path_cells(route_path_cells),
        .route_path_length(route_path_length),
        .route_success(route_success),

        .move_req(move_req),
        .move_grant(move_grant),
        .movement_conflict(movement_conflict),

        .blocked_mask_out(blocked_mask_out),
        .reroute_required(reroute_required)
    );

endmodule
