module order_manager (
    input wire clk,
    input wire reset,

input wire push_valid,
    input wire [1:0] push_priority,
    input wire [4:0] push_pickup,
    input wire [4:0] push_destination,
    output wire push_ready,

output wire head_valid,
    output wire [1:0] head_priority,
    output wire [4:0] head_pickup,
    output wire [4:0] head_destination,

input wire pop_ready,
    output reg pop_valid,

output wire [2:0] count_out
);

    localparam DEPTH = 4;

reg [1:0] prio_q [0:DEPTH-1];
    reg [4:0] pick_q [0:DEPTH-1];
    reg [4:0] dest_q [0:DEPTH-1];

reg [1:0] prio_next [0:DEPTH-1];
    reg [4:0] pick_next [0:DEPTH-1];
    reg [4:0] dest_next [0:DEPTH-1];

reg [2:0] count;
    reg [2:0] count_next;

integer a;
    integer b;
    integer c;
    integer insert_pos;

wire pop_fire;
    wire push_fire;

assign head_valid = (count != 0);
    assign head_priority = prio_q[0];
    assign head_pickup = pick_q[0];
    assign head_destination = dest_q[0];

    assign count_out = count;

assign push_ready =
        (count < DEPTH) || ((count != 0) && pop_ready);

assign pop_fire = pop_ready && (count != 0);
    assign push_fire = push_valid && push_ready;

    always @(*) begin

count_next = count;
        insert_pos = 0;

for (a = 0; a < DEPTH; a = a + 1) begin
            prio_next[a] = prio_q[a];
            pick_next[a] = pick_q[a];
            dest_next[a] = dest_q[a];
        end

// Pop first.
        if (pop_fire) begin

for (a = 0; a < DEPTH-1; a = a + 1) begin
                prio_next[a] = prio_next[a+1];
                pick_next[a] = pick_next[a+1];
                dest_next[a] = dest_next[a+1];
            end

prio_next[DEPTH-1] = 2'd0;
            pick_next[DEPTH-1] = 5'd0;
            dest_next[DEPTH-1] = 5'd0;

count_next = count_next - 1'b1;
        end

// Then insert the new order.
        if (push_fire) begin

            insert_pos = count_next;

for (b = 0; b < DEPTH; b = b + 1) begin
                if ((b < count_next) &&
                    (insert_pos == count_next) &&
                    (prio_next[b] < push_priority)) begin
                    insert_pos = b;
                end
            end

for (a = DEPTH-1; a > 0; a = a - 1) begin
                if (a > insert_pos) begin
                    prio_next[a] = prio_next[a-1];
                    pick_next[a] = pick_next[a-1];
                    dest_next[a] = dest_next[a-1];
                end
            end

prio_next[insert_pos] = push_priority;
            pick_next[insert_pos] = push_pickup;
            dest_next[insert_pos] = push_destination;

count_next = count_next + 1'b1;
        end
    end

    always @(posedge clk or posedge reset) begin

        if (reset) begin

count <= 3'd0;
            pop_valid <= 1'b0;

for (c = 0; c < DEPTH; c = c + 1) begin
                prio_q[c] <= 2'd0;
                pick_q[c] <= 5'd0;
                dest_q[c] <= 5'd0;
            end

        end else begin

count <= count_next;
            pop_valid <= pop_fire;

for (c = 0; c < DEPTH; c = c + 1) begin
                prio_q[c] <= prio_next[c];
                pick_q[c] <= pick_next[c];
                dest_q[c] <= dest_next[c];
            end

end
    end

endmodule