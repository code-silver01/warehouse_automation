
module warehouse_top (
    input  wire        clk,
    input  wire        reset,

    // Dashboard order input.
    input  wire        order_push_valid,
    input  wire [1:0]  order_push_priority,
    input  wire [4:0]  order_push_pickup,
    input  wire [4:0]  order_push_destination,
    output wire        order_push_ready,
    output wire [2:0]  queue_count_out,

    // Initial positions: {vehicle2, vehicle1, vehicle0}
    input  wire [14:0] initial_positions,

    // Obstacle update.
    input  wire        set_blocked_valid,
    input  wire [4:0]  set_blocked_cell,
    input  wire        set_blocked_value,

    // Vehicle and occupancy status.
    output wire [14:0] vehicle_positions,
    output wire [2:0]  vehicle_done,
    output reg  [2:0]  vehicle_busy,
    output reg  [24:0] occupied_mask_out,
    output reg         collision_detected,

    // Allocation status.
    output wire        alloc_valid,
    output wire [1:0]  alloc_vehicle,
    output wire [4:0]  alloc_pickup,
    output wire [4:0]  alloc_destination,
    output wire [1:0]  alloc_priority,

    // Route status.
    output reg         route_valid,
    output reg  [1:0]  route_vehicle,
    output reg         route_leg,
    output reg  [4:0]  route_start_cell,
    output reg  [4:0]  route_goal_cell,
    output reg  [124:0] route_path_cells,
    output reg  [4:0]  route_path_length,
    output reg         route_success,

    // Traffic status.
    output wire [2:0]  move_req,
    output wire [2:0]  move_grant,
    output wire [2:0]  movement_conflict,

    output wire [24:0] blocked_mask_out,
    output wire [2:0]  reroute_required
);

    localparam [3:0]
        S_IDLE       = 4'd0,
        S_ALLOC      = 4'd1,
        S_ALLOC_WAIT = 4'd2,
        S_PREPARE    = 4'd3,
        S_LAUNCH     = 4'd4,
        S_PLAN_WAIT  = 4'd5,
        S_LOAD       = 4'd6,
        S_MOVE       = 4'd7;

    reg [3:0] state;

    reg phase;

    reg [1:0] lat_vehicle;
    reg [4:0] lat_pickup;
    reg [4:0] lat_destination;

    wire om_push_ready;
    wire om_head_valid;

    wire [1:0] om_head_priority;
    wire [4:0] om_head_pickup;
    wire [4:0] om_head_destination;

    wire om_pop_valid;
    wire [2:0] om_count;

    wire om_pop_ready;

    // FIX: Remove the queue head when allocation is accepted.
    assign om_pop_ready =
        (state == S_ALLOC_WAIT) && alloc_valid;

    order_manager u_order_manager (
        .clk(clk),
        .reset(reset),

        .push_valid(order_push_valid),
        .push_priority(order_push_priority),
        .push_pickup(order_push_pickup),
        .push_destination(order_push_destination),
        .push_ready(om_push_ready),

        .head_valid(om_head_valid),
        .head_priority(om_head_priority),
        .head_pickup(om_head_pickup),
        .head_destination(om_head_destination),

        .pop_ready(om_pop_ready),
        .pop_valid(om_pop_valid),

        .count_out(om_count)
    );

    assign order_push_ready = om_push_ready;
    assign queue_count_out = om_count;

    // -----------------------------------------------------
    // VEHICLE ALLOCATOR
    // -----------------------------------------------------

    wire alloc_order_ready;

    wire [1:0] alloc_vehicle_int;
    wire [4:0] alloc_pickup_int;
    wire [4:0] alloc_destination_int;
    wire [1:0] alloc_priority_int;

    wire order_valid_to_allocator;

    assign order_valid_to_allocator =
        (state == S_ALLOC) && om_head_valid;

    vehicle_allocator u_allocator (
        .clk(clk),
        .reset(reset),

        .order_valid(order_valid_to_allocator),
        .pickup_cell(om_head_pickup),
        .destination_cell(om_head_destination),
        .order_priority(om_head_priority),

        .vehicle_positions(vehicle_positions),
        .vehicle_busy(vehicle_busy),

        .order_ready(alloc_order_ready),

        .alloc_valid(alloc_valid),
        .alloc_vehicle(alloc_vehicle_int),
        .alloc_pickup(alloc_pickup_int),
        .alloc_destination(alloc_destination_int),
        .alloc_priority(alloc_priority_int)
    );

    assign alloc_vehicle = alloc_vehicle_int;
    assign alloc_pickup = alloc_pickup_int;
    assign alloc_destination = alloc_destination_int;
    assign alloc_priority = alloc_priority_int;

    // -----------------------------------------------------
    // BFS ROUTE PLANNER
    // -----------------------------------------------------

    reg planner_start;
    reg [4:0] planner_start_cell;
    reg [4:0] planner_goal_cell;

    wire planner_busy;
    wire planner_done;
    wire planner_success;

    wire [124:0] planner_path_cells;
    wire [4:0] planner_path_length;

    route_planner u_route_planner (
        .clk(clk),
        .reset(reset),

        .start(planner_start),
        .blocked(blocked_mask_out),

        .start_cell(planner_start_cell),
        .goal_cell(planner_goal_cell),

        .busy(planner_busy),
        .done(planner_done),
        .success(planner_success),

        .path_cells(planner_path_cells),
        .path_length(planner_path_length)
    );

    // -----------------------------------------------------
    // MOVEMENT CONTROLLER INTERFACE
    // -----------------------------------------------------

    reg [2:0] movement_route_load;
    reg [14:0] movement_load_lengths;
    reg [374:0] movement_load_paths;

    wire [374:0] stored_paths;
    wire [14:0] stored_lengths;
    wire [14:0] route_indices;

    // -----------------------------------------------------
    // IMMEDIATE OBSTACLE-ON-ROUTE PAUSE
    // -----------------------------------------------------

    reg pending_obstacle_on_route;
    integer reroute_scan_idx;
    wire movement_pause;

    always @(*) begin

        pending_obstacle_on_route = 1'b0;

        if (set_blocked_valid &&
            set_blocked_value &&
            (lat_vehicle < 2'd3)) begin

            for (reroute_scan_idx = 0;
                 reroute_scan_idx < 25;
                 reroute_scan_idx = reroute_scan_idx + 1) begin

                if ((reroute_scan_idx >
                     route_indices[(lat_vehicle * 5) +: 5]) &&

                    (reroute_scan_idx <
                     stored_lengths[(lat_vehicle * 5) +: 5]) &&

                    (stored_paths[(lat_vehicle * 125) +
                     (reroute_scan_idx * 5) +: 5] ==
                     set_blocked_cell)) begin

                    pending_obstacle_on_route = 1'b1;
                end
            end
        end
    end

    assign movement_pause =
        pending_obstacle_on_route ||
        ((lat_vehicle < 2'd3) &&
         reroute_required[lat_vehicle]);

    vehicle_movement_controller u_movement (
        .clk(clk),
        .reset(reset),

        .initial_positions(initial_positions),

        .route_load(movement_route_load),
        .pause(movement_pause),
        .path_lengths(movement_load_lengths),
        .all_paths(movement_load_paths),

        .vehicle_positions(vehicle_positions),
        .vehicle_done(vehicle_done),

        .move_req(move_req),
        .move_grant(move_grant),
        .conflict(movement_conflict),

        .stored_paths(stored_paths),
        .stored_lengths(stored_lengths),
        .route_indices(route_indices)
    );

    // -----------------------------------------------------
    // DYNAMIC OBSTACLE MANAGER
    // -----------------------------------------------------

    dynamic_obstacle_manager u_obstacles (
        .clk(clk),
        .reset(reset),

        .obstacle_update(set_blocked_valid),
        .obstacle_cell(set_blocked_cell),
        .obstacle_blocked(set_blocked_value),

        .all_paths(stored_paths),
        .route_lengths(stored_lengths),
        .route_indices(route_indices),

        .blocked_mask(blocked_mask_out),
        .reroute_required(reroute_required)
    );

    // -----------------------------------------------------
    // ROUTE LOAD MULTIPLEXER
    // -----------------------------------------------------

    always @(*) begin

        movement_route_load = 3'b000;
        movement_load_lengths = 15'd0;
        movement_load_paths = 375'd0;

        if (state == S_LOAD) begin

            case (lat_vehicle)

                2'd0: begin
                    movement_route_load = 3'b001;
                    movement_load_lengths[4:0] =
                        planner_path_length;
                    movement_load_paths[124:0] =
                        planner_path_cells;
                end

                2'd1: begin
                    movement_route_load = 3'b010;
                    movement_load_lengths[9:5] =
                        planner_path_length;
                    movement_load_paths[249:125] =
                        planner_path_cells;
                end

                2'd2: begin
                    movement_route_load = 3'b100;
                    movement_load_lengths[14:10] =
                        planner_path_length;
                    movement_load_paths[374:250] =
                        planner_path_cells;
                end

                default: begin
                    movement_route_load = 3'b000;
                end

            endcase
        end
    end

    // -----------------------------------------------------
    // OCCUPANCY MAP AND COLLISION DETECTION
    // -----------------------------------------------------

    always @(*) begin

        occupied_mask_out = 25'd0;
        collision_detected = 1'b0;

        if (vehicle_positions[4:0] < 25)
            occupied_mask_out[vehicle_positions[4:0]] = 1'b1;

        if (vehicle_positions[9:5] < 25) begin

            if (occupied_mask_out[vehicle_positions[9:5]])
                collision_detected = 1'b1;

            occupied_mask_out[vehicle_positions[9:5]] = 1'b1;
        end

        if (vehicle_positions[14:10] < 25) begin

            if (occupied_mask_out[vehicle_positions[14:10]])
                collision_detected = 1'b1;

            occupied_mask_out[vehicle_positions[14:10]] = 1'b1;
        end
    end

    // -----------------------------------------------------
    // MAIN WAREHOUSE COORDINATION FSM
    // -----------------------------------------------------

    always @(posedge clk or posedge reset) begin

        if (reset) begin

            state <= S_IDLE;

            phase <= 1'b0;

            lat_vehicle <= 2'd0;
            lat_pickup <= 5'd0;
            lat_destination <= 5'd0;

            vehicle_busy <= 3'b000;

            planner_start <= 1'b0;
            planner_start_cell <= 5'd0;
            planner_goal_cell <= 5'd0;

            route_valid <= 1'b0;
            route_vehicle <= 2'd0;
            route_leg <= 1'b0;
            route_start_cell <= 5'd0;
            route_goal_cell <= 5'd0;
            route_path_cells <= 125'd0;
            route_path_length <= 5'd0;
            route_success <= 1'b0;

        end else begin

            planner_start <= 1'b0;
            route_valid <= 1'b0;

            case (state)

                // Wait for an order.
                S_IDLE: begin
                    if (om_head_valid)
                        state <= S_ALLOC;
                end

                // Request an available vehicle.
                S_ALLOC: begin
                    if (alloc_order_ready)
                        state <= S_ALLOC_WAIT;
                end

                // Wait for allocator's registered result.
                // Queue entry is popped on this same acceptance cycle.
                S_ALLOC_WAIT: begin

                    if (alloc_valid) begin

                        lat_vehicle <= alloc_vehicle_int;
                        lat_pickup <= alloc_pickup_int;
                        lat_destination <= alloc_destination_int;

                        vehicle_busy[alloc_vehicle_int] <= 1'b1;

                        phase <= 1'b0;

                        state <= S_PREPARE;
                    end
                end

                // Prepare pickup or delivery route.
                S_PREPARE: begin

                    case (lat_vehicle)

                        2'd0:
                            planner_start_cell <=
                                vehicle_positions[4:0];

                        2'd1:
                            planner_start_cell <=
                                vehicle_positions[9:5];

                        2'd2:
                            planner_start_cell <=
                                vehicle_positions[14:10];

                        default:
                            planner_start_cell <= 5'd0;

                    endcase

                    if (phase == 1'b0)
                        planner_goal_cell <= lat_pickup;
                    else
                        planner_goal_cell <= lat_destination;

                    state <= S_LAUNCH;
                end

                // Start BFS.
                S_LAUNCH: begin
                    planner_start <= 1'b1;
                    state <= S_PLAN_WAIT;
                end

                // Wait for BFS completion.
                S_PLAN_WAIT: begin

                    if (planner_done) begin

                        if (planner_success) begin
                            state <= S_LOAD;
                        end else begin

                            // Release vehicle if route cannot be planned.
                            vehicle_busy[lat_vehicle] <= 1'b0;

                            // FIX: Order was already removed at allocation.
                            state <= S_IDLE;
                        end

                    end
                end

                // Load generated route.
                S_LOAD: begin

                    route_valid <= 1'b1;
                    route_vehicle <= lat_vehicle;
                    route_leg <= phase;

                    route_start_cell <= planner_start_cell;
                    route_goal_cell <= planner_goal_cell;

                    route_path_cells <= planner_path_cells;
                    route_path_length <= planner_path_length;
                    route_success <= planner_success;

                    state <= S_MOVE;
                end

                // Wait for movement completion or rerouting.
                S_MOVE: begin

                    if (reroute_required[lat_vehicle]) begin

                        // Replan from the vehicle's actual position.
                        state <= S_PREPARE;

                    end else if (vehicle_done[lat_vehicle]) begin

                        if (phase == 1'b0) begin

                            // Pickup reached: plan delivery leg.
                            phase <= 1'b1;
                            state <= S_PREPARE;

                        end else begin

                            // Delivery completed.
                            vehicle_busy[lat_vehicle] <= 1'b0;

                            // FIX: Queue entry was already popped.
                            state <= S_IDLE;

                        end
                    end
                end

                default: begin
                    state <= S_IDLE;
                end

            endcase
        end
    end

endmodule
