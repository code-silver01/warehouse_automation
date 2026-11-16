module route_planner (
    input wire clk,
    input wire reset,
    input wire start,

    input wire [24:0] blocked,
    input wire [4:0] start_cell,
    input wire [4:0] goal_cell,

    output reg busy,
    output reg done,
    output reg success,

    output reg [124:0] path_cells,
    output reg [4:0] path_length
);

    localparam IDLE    = 3'd0;
    localparam POP     = 3'd1;
    localparam EXPAND  = 3'd2;
    localparam TRACE   = 3'd3;
    localparam REVERSE = 3'd4;

    reg [2:0] state;

    reg [4:0] queue [0:24];
    reg [5:0] parent [0:24];
    reg [4:0] reverse_path [0:24];

    reg [24:0] visited;

    reg [5:0] head;
    reg [5:0] tail;

    reg [4:0] current_cell;
    reg [1:0] direction;

    reg [4:0] trace_cell;
    reg [5:0] reverse_count;

    reg [5:0] neighbor_cell;

    integer i;

    function [5:0] get_neighbor;
        input [4:0] cell_idx;
        input [1:0] dir;

        integer row_idx;
        integer col_idx;

        begin
            row_idx = cell_idx / 5;
            col_idx = cell_idx % 5;

            case (dir)
                2'd0: get_neighbor = (row_idx == 0) ? 6'd63 : cell_idx - 5;
                2'd1: get_neighbor = (row_idx == 4) ? 6'd63 : cell_idx + 5;
                2'd2: get_neighbor = (col_idx == 0) ? 6'd63 : cell_idx - 1;
                2'd3: get_neighbor = (col_idx == 4) ? 6'd63 : cell_idx + 1;
                default: get_neighbor = 6'd63;
            endcase
        end
    endfunction

    always @(*) begin
        neighbor_cell = get_neighbor(current_cell, direction);
    end

    always @(posedge clk or posedge reset) begin

        if (reset) begin
            state         <= IDLE;
            busy          <= 1'b0;
            done          <= 1'b0;
            success       <= 1'b0;
            head          <= 6'd0;
            tail          <= 6'd0;
            current_cell  <= 5'd0;
            direction     <= 2'd0;
            trace_cell    <= 5'd0;
            reverse_count <= 6'd0;
            visited       <= 25'd0;
            path_cells    <= 125'd0;
            path_length   <= 5'd0;
            for (i = 0; i < 25; i = i + 1) begin
                queue[i]        <= 5'd0;
                parent[i]       <= 6'd63;
                reverse_path[i] <= 5'd0;
            end
        end

        else begin
            case (state)

                // ------------------------------------
                // IDLE
                // ------------------------------------
                IDLE: begin
                    // Make done a 1-cycle pulse, not a sticky level
                    done <= 1'b0;

                    if (start) begin
                        success     <= 1'b0;
                        busy        <= 1'b1;
                        path_cells  <= 125'd0;
                        path_length <= 5'd0;
                        head        <= 6'd0;
                        tail        <= 6'd0;
                        visited     <= 25'd0;

                        if ((start_cell >= 5'd25) ||
                            (goal_cell >= 5'd25) ||
                            blocked[start_cell] ||
                            blocked[goal_cell]) begin
                            busy    <= 1'b0;
                            done    <= 1'b1;
                            success <= 1'b0;
                        end else begin
                            queue[0]            <= start_cell;
                            visited[start_cell] <= 1'b1;
                            parent[start_cell]  <= 6'd63;
                            tail                <= 6'd1;
                            state               <= POP;
                        end
                    end
                end

                // ------------------------------------
                // POP
                // ------------------------------------
                POP: begin
                    if (head < tail) begin
                        current_cell <= queue[head];
                        head <= head + 1'b1;

                        if (queue[head] == goal_cell) begin
                            trace_cell      <= goal_cell;
                            reverse_path[0] <= goal_cell;
                            reverse_count   <= 6'd1;
                            state <= TRACE;
                        end else begin
                            direction <= 2'd0;
                            state <= EXPAND;
                        end
                    end else begin
                        busy    <= 1'b0;
                        done    <= 1'b1;
                        success <= 1'b0;
                        state   <= IDLE;
                    end
                end

                // ------------------------------------
                // EXPAND
                // ------------------------------------
                EXPAND: begin
                    if (neighbor_cell < 6'd25) begin
                        if (!visited[neighbor_cell] && !blocked[neighbor_cell]) begin
                            visited[neighbor_cell] <= 1'b1;
                            parent[neighbor_cell] <= current_cell;
                            if (tail < 6'd25) begin
                                queue[tail] <= neighbor_cell[4:0];
                                tail <= tail + 1'b1;
                            end
                        end
                    end
                    if (direction == 2'd3)
                        state <= POP;
                    else
                        direction <= direction + 1'b1;
                end

                // ------------------------------------
                // TRACE
                // ------------------------------------
                TRACE: begin
                    if (trace_cell == start_cell) begin
                        state <= REVERSE;
                    end else if (parent[trace_cell] < 6'd25) begin
                        trace_cell <= parent[trace_cell];
                        reverse_path[reverse_count] <= parent[trace_cell][4:0];
                        reverse_count <= reverse_count + 1'b1;
                    end else begin
                        busy    <= 1'b0;
                        done    <= 1'b1;
                        success <= 1'b0;
                        state   <= IDLE;
                    end
                end

                // ------------------------------------
                // REVERSE
                // ------------------------------------
                REVERSE: begin
                    for (i = 0; i < 25; i = i + 1) begin
                        if (i < reverse_count)
                            path_cells[(i * 5) +: 5] <= reverse_path[reverse_count - 1 - i];
                    end
                    path_length <= reverse_count[4:0];
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    success <= 1'b1;
                    state   <= IDLE;
                end

                default: begin
                    state <= IDLE;
                    busy  <= 1'b0;
                end
            endcase
        end
    end

endmodule