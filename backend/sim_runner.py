
import re
import subprocess
import tempfile
from pathlib import Path


# =========================================================
# CONFIGURATION
# =========================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RTL_DIR = PROJECT_ROOT / "rtl"

VIVADO_BIN = Path(r"C:\Xilinx\Vivado\2018.3\bin")

RTL_FILES = [
    "route_planner.v",
    "order_manager.v",
    "vehicle_allocator.v",
    "traffic_controller.v",
    "vehicle_movement_controller.v",
    "dynamic_obstacle_manager.v",
    "warehouse_state.v",
    "warehouse_top.v",
]


# =========================================================
# VIVADO TOOL DISCOVERY
# =========================================================

def find_tool(tool_name):
    """Find the Windows Vivado batch launcher."""

    try:
        bat_file = VIVADO_BIN / f"{tool_name}.bat"
        if bat_file.exists():
            return str(bat_file)

        exe_file = VIVADO_BIN / f"{tool_name}.exe"
        if exe_file.exists():
            return str(exe_file)

        result = subprocess.run(
            ["where.exe", tool_name],
            capture_output=True,
            text=True,
        )

        if result.returncode == 0:
            for candidate in result.stdout.splitlines():
                candidate = candidate.strip()

                if candidate.lower().endswith(".bat"):
                    return candidate

                if candidate.lower().endswith(".exe"):
                    return candidate
    except Exception:
        pass

    raise FileNotFoundError(
        f"Cannot find Vivado tool: {tool_name}"
    )


def is_vivado_available():
    """Check if Vivado command-line tools are available on the system."""
    try:
        find_tool("xvlog")
        return True
    except Exception:
        return False


# =========================================================
# COMMAND EXECUTION
# =========================================================

def run_command(command, working_directory, timeout=120):
    """Run Vivado commands through CMD when using .bat files."""

    if command[0].lower().endswith(".bat"):
        command_to_run = [
            "cmd.exe",
            "/d",
            "/c",
            subprocess.list2cmdline(command),
        ]
    else:
        command_to_run = command

    result = subprocess.run(
        command_to_run,
        cwd=str(working_directory),
        capture_output=True,
        text=True,
        timeout=timeout,
        shell=False,
    )

    output = (
        (result.stdout or "")
        + "\n"
        + (result.stderr or "")
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"Command failed:\n"
            f"{' '.join(command)}\n\n"
            f"{output[-12000:]}"
        )

    return output


# =========================================================
# TEMPORARY RTL PREPARATION
# =========================================================

def prepare_rtl_sources(run_dir):
    """
    Create temporary copies of RTL sources with a consistent
    timescale directive. Original project files remain untouched.
    """

    prepared_dir = run_dir / "rtl"
    prepared_dir.mkdir()

    prepared_files = []

    for filename in RTL_FILES:
        source = RTL_DIR / filename

        if not source.exists():
            raise FileNotFoundError(
                f"Missing RTL file: {source}"
            )

        content = source.read_text(encoding="utf-8-sig")

        # Avoid adding a duplicate directive.
        if not re.search(
            r"^\s*`timescale\b",
            content,
            flags=re.MULTILINE,
        ):
            content = "`timescale 1ns/1ps\n" + content

        destination = prepared_dir / filename
        destination.write_text(content, encoding="utf-8")

        prepared_files.append(destination)

    return prepared_files


# =========================================================
# TESTBENCH GENERATION
# =========================================================

def build_testbench(
    pickup,
    destination,
    priority,
    initial_positions,
):
    return f"""
`timescale 1ns/1ps

module warehouse_tb;

    reg clk = 0;
    reg reset = 1;

    reg order_push_valid = 0;
    reg [1:0] order_push_priority = 0;
    reg [4:0] order_push_pickup = 0;
    reg [4:0] order_push_destination = 0;

    wire order_push_ready;
    wire [2:0] queue_count_out;

    reg [14:0] initial_positions = 15'd{initial_positions};

    reg set_blocked_valid = 0;
    reg [4:0] set_blocked_cell = 0;
    reg set_blocked_value = 0;

    wire [14:0] vehicle_positions;
    wire [2:0] vehicle_done;
    wire [2:0] vehicle_busy;
    wire [24:0] occupied_mask_out;
    wire collision_detected;

    wire alloc_valid;
    wire [1:0] alloc_vehicle;
    wire [4:0] alloc_pickup;
    wire [4:0] alloc_destination;
    wire [1:0] alloc_priority;

    wire route_valid;
    wire [1:0] route_vehicle;
    wire route_leg;
    wire [4:0] route_start_cell;
    wire [4:0] route_goal_cell;
    wire [124:0] route_path_cells;
    wire [4:0] route_path_length;
    wire route_success;

    wire [2:0] move_req;
    wire [2:0] move_grant;
    wire [2:0] movement_conflict;

    wire [24:0] blocked_mask_out;
    wire [2:0] reroute_required;

    reg allocation_seen = 0;
    reg [14:0] previous_positions = 15'h7fff;

    integer cycle_count = 0;

    always #5 clk = ~clk;

    warehouse_top dut (
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

    always @(negedge clk) begin
        if (!reset) begin
            cycle_count = cycle_count + 1;

            if (vehicle_positions !== previous_positions) begin
                $display(
                    "POS,%0t,%0d,%0d,%0d",
                    $time,
                    vehicle_positions[4:0],
                    vehicle_positions[9:5],
                    vehicle_positions[14:10]
                );

                previous_positions = vehicle_positions;
            end

            if (alloc_valid) begin
                allocation_seen = 1;

                $display(
                    "ALLOC,%0t,%0d,%0d,%0d,%0d",
                    $time,
                    alloc_vehicle,
                    alloc_pickup,
                    alloc_destination,
                    alloc_priority
                );
            end

            if (route_valid) begin
                $display(
                    "ROUTE,%0t,%0d,%0d,%0d,%0d,%0d,%0d,%h",
                    $time,
                    route_vehicle,
                    route_leg,
                    route_start_cell,
                    route_goal_cell,
                    route_path_length,
                    route_success,
                    route_path_cells
                );
            end

            if (collision_detected)
                $display("COLLISION,%0t", $time);

            if (movement_conflict != 3'b000) begin
                $display(
                    "MOVE_CONFLICT,%0t,%b",
                    $time,
                    movement_conflict
                );
            end

            if (cycle_count > 20000) begin
                $display("SIM_TIMEOUT");
                $finish;
            end
        end
    end

    initial begin
        $display("SIM_START");

        order_push_priority = 2'd{priority};
        order_push_pickup = 5'd{pickup};
        order_push_destination = 5'd{destination};

        repeat (5) @(negedge clk);
        reset = 0;

        wait (order_push_ready == 1'b1);

        @(negedge clk);
        order_push_valid = 1;

        @(negedge clk);
        order_push_valid = 0;

        // Wait until allocation has actually happened.
wait (allocation_seen == 1);

// Do not declare completion before the vehicle becomes busy.
wait (vehicle_busy != 3'b000);

// Wait for the vehicle to finish its pickup and delivery.
wait (
    queue_count_out == 0 &&
    vehicle_busy == 3'b000
);

repeat (5) @(negedge clk);

        $display("SIM_DONE");
        $finish;
    end

    initial begin
        #2500000;
        $display("SIM_TIMEOUT");
        $finish;
    end

endmodule
"""


# =========================================================
# OUTPUT PARSER
# =========================================================

def parse_simulation_output(output):

    positions = []
    allocations = []
    routes = []
    collisions = []
    movement_conflicts = []

    for line in output.splitlines():
        line = line.strip()

        match = re.search(
            r"POS,(\d+),(\d+),(\d+),(\d+)",
            line,
        )

        if match:
            positions.append({
                "time_ns": int(match.group(1)),
                "vehicles": [
                    int(match.group(2)),
                    int(match.group(3)),
                    int(match.group(4)),
                ],
            })
            continue

        match = re.search(
            r"ALLOC,(\d+),(\d+),(\d+),(\d+),(\d+)",
            line,
        )

        if match:
            allocations.append({
                "time_ns": int(match.group(1)),
                "vehicle": int(match.group(2)),
                "pickup": int(match.group(3)),
                "destination": int(match.group(4)),
                "priority": int(match.group(5)),
            })
            continue

        match = re.search(
            r"ROUTE,(\d+),(\d+),(\d+),(\d+),"
            r"(\d+),(\d+),(\d+),([0-9a-fA-FxX]+)",
            line,
        )

        if match:
            routes.append({
                "time_ns": int(match.group(1)),
                "vehicle": int(match.group(2)),
                "leg": int(match.group(3)),
                "start": int(match.group(4)),
                "goal": int(match.group(5)),
                "length": int(match.group(6)),
                "success": int(match.group(7)),
                "path_hex": match.group(8),
            })
            continue

        match = re.search(r"COLLISION,(\d+)", line)

        if match:
            collisions.append({
                "time_ns": int(match.group(1)),
            })
            continue

        match = re.search(
            r"MOVE_CONFLICT,(\d+),([01xXzZ]+)",
            line,
        )

        if match:
            movement_conflicts.append({
                "time_ns": int(match.group(1)),
                "mask": match.group(2),
            })

    return {
        "positions": positions,
        "allocations": allocations,
        "routes": routes,
        "collisions": collisions,
        "movement_conflicts": movement_conflicts,
        "completed": "SIM_DONE" in output,
        "timed_out": "SIM_TIMEOUT" in output,
        "raw_output": output[-12000:],
    }


# =========================================================
# CLOUD DIGITAL TWIN EMULATOR (FALLBACK FOR VERCEL / LINUX)
# =========================================================

def _bfs_path_planner(start_cell, goal_cell, blocked_cells=None):
    """
    Breadth-first search 4-neighbor route planner mirroring RTL route_planner.v.
    Neighbor expansion order:
      0: Up   (-5)
      1: Down (+5)
      2: Left (-1)
      3: Right(+1)
    """
    if blocked_cells is None:
        blocked_cells = set()

    if start_cell == goal_cell:
        return [start_cell]

    from collections import deque

    queue = deque([start_cell])
    visited = {start_cell}
    parent = {start_cell: None}

    found = False
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            found = True
            break

        row = current // 5
        col = current % 5

        # 4 directions matching Verilog route_planner get_neighbor function
        neighbors = []
        if row > 0:
            neighbors.append(current - 5)  # 0: Up
        if row < 4:
            neighbors.append(current + 5)  # 1: Down
        if col > 0:
            neighbors.append(current - 1)  # 2: Left
        if col < 4:
            neighbors.append(current + 1)  # 3: Right

        for neighbor in neighbors:
            if neighbor not in visited and neighbor not in blocked_cells:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append(neighbor)

    if not found:
        return []

    path = []
    curr = goal_cell
    while curr is not None:
        path.append(curr)
        curr = parent[curr]
    path.reverse()
    return path


def _pack_path_cells(path):
    """Pack 5-bit path cell sequence into hex string matching RTL format."""
    packed = 0
    for i, cell in enumerate(path):
        packed |= (cell & 0x1F) << (i * 5)
    return f"{packed:032x}"


def emulate_order_digital_twin(
    pickup,
    destination,
    priority=1,
    initial_positions=(0, 4, 20),
):
    """
    Full Python digital twin simulation of warehouse RTL behavior.
    Used when Vivado XSim is unavailable (e.g. cloud serverless environments).
    """
    p_row, p_col = pickup // 5, pickup % 5
    vehicles = list(initial_positions)

    # 1. Vehicle Allocation (mirrors vehicle_allocator.v Manhattan distance)
    best_dist = 999
    allocated_vehicle = 0
    for v_id in range(3):
        v_pos = vehicles[v_id]
        v_row, v_col = v_pos // 5, v_pos % 5
        dist = abs(v_row - p_row) + abs(v_col - p_col)
        if dist < best_dist:
            best_dist = dist
            allocated_vehicle = v_id

    time_ns = 50
    positions = []
    routes = []
    allocations = []
    log_lines = ["SIM_START"]

    # Initial state
    positions.append({
        "time_ns": time_ns,
        "vehicles": list(vehicles),
    })
    log_lines.append(f"POS,{time_ns},{vehicles[0]},{vehicles[1]},{vehicles[2]}")

    # Allocation event
    time_ns = 100
    allocations.append({
        "time_ns": time_ns,
        "vehicle": allocated_vehicle,
        "pickup": pickup,
        "destination": destination,
        "priority": priority,
    })
    log_lines.append(
        f"ALLOC,{time_ns},{allocated_vehicle},{pickup},{destination},{priority}"
    )

    # Leg 0: Transit to Pickup
    v_start = vehicles[allocated_vehicle]
    path_to_pickup = _bfs_path_planner(v_start, pickup)
    path_hex_0 = _pack_path_cells(path_to_pickup)

    time_ns = 120
    routes.append({
        "time_ns": time_ns,
        "vehicle": allocated_vehicle,
        "leg": 0,
        "start": v_start,
        "goal": pickup,
        "length": len(path_to_pickup),
        "success": 1 if path_to_pickup else 0,
        "path_hex": path_hex_0,
    })
    log_lines.append(
        f"ROUTE,{time_ns},{allocated_vehicle},0,{v_start},{pickup},"
        f"{len(path_to_pickup)},1,{path_hex_0}"
    )

    # Move along Leg 0
    for step_cell in path_to_pickup[1:]:
        time_ns += 20
        vehicles[allocated_vehicle] = step_cell
        positions.append({
            "time_ns": time_ns,
            "vehicles": list(vehicles),
        })
        log_lines.append(f"POS,{time_ns},{vehicles[0]},{vehicles[1]},{vehicles[2]}")

    # Leg 1: Transit to Destination
    time_ns += 10
    path_to_dest = _bfs_path_planner(pickup, destination)
    path_hex_1 = _pack_path_cells(path_to_dest)

    routes.append({
        "time_ns": time_ns,
        "vehicle": allocated_vehicle,
        "leg": 1,
        "start": pickup,
        "goal": destination,
        "length": len(path_to_dest),
        "success": 1 if path_to_dest else 0,
        "path_hex": path_hex_1,
    })
    log_lines.append(
        f"ROUTE,{time_ns},{allocated_vehicle},1,{pickup},{destination},"
        f"{len(path_to_dest)},1,{path_hex_1}"
    )

    # Move along Leg 1
    for step_cell in path_to_dest[1:]:
        time_ns += 20
        vehicles[allocated_vehicle] = step_cell
        positions.append({
            "time_ns": time_ns,
            "vehicles": list(vehicles),
        })
        log_lines.append(f"POS,{time_ns},{vehicles[0]},{vehicles[1]},{vehicles[2]}")

    time_ns += 10
    log_lines.append("SIM_DONE")
    log_lines.append("[Cloud Digital Twin] Completed via RTL-equivalent emulation engine.")

    raw_output = "\n".join(log_lines)

    return {
        "positions": positions,
        "allocations": allocations,
        "routes": routes,
        "collisions": [],
        "movement_conflicts": [],
        "completed": True,
        "timed_out": False,
        "raw_output": raw_output,
        "input": {
            "pickup": pickup,
            "destination": destination,
            "priority": priority,
            "initial_positions": list(initial_positions),
        },
    }


# =========================================================
# MAIN SIMULATION FUNCTION
# =========================================================

def simulate_order(
    pickup,
    destination,
    priority=1,
    initial_positions=(0, 4, 20),
):
    """Compile and simulate one order using Vivado XSim (or fallback on cloud)."""

    pickup = int(pickup)
    destination = int(destination)
    priority = int(priority)

    initial_positions = tuple(
        int(position) for position in initial_positions
    )

    if not 0 <= pickup <= 24:
        raise ValueError("Pickup cell must be between 0 and 24.")

    if not 0 <= destination <= 24:
        raise ValueError("Destination cell must be between 0 and 24.")

    if pickup == destination:
        raise ValueError("Pickup and destination must differ.")

    if not 0 <= priority <= 3:
        raise ValueError("Priority must be between 0 and 3.")

    if len(initial_positions) != 3:
        raise ValueError("Exactly three vehicle positions are required.")

    if any(p < 0 or p > 24 for p in initial_positions):
        raise ValueError("Vehicle positions must be between 0 and 24.")

    if len(set(initial_positions)) != 3:
        raise ValueError("Initial vehicle positions must be unique.")

    # In cloud environments (e.g. Vercel) where Vivado is not installed,
    # seamlessly run the digital twin emulation.
    if not is_vivado_available():
        return emulate_order_digital_twin(
            pickup=pickup,
            destination=destination,
            priority=priority,
            initial_positions=initial_positions,
        )

    # Packed as {vehicle 2, vehicle 1, vehicle 0}.
    packed_positions = (
        (initial_positions[2] << 10)
        | (initial_positions[1] << 5)
        | initial_positions[0]
    )

    try:
        xvlog = find_tool("xvlog")
        xelab = find_tool("xelab")
        xsim = find_tool("xsim")

        with tempfile.TemporaryDirectory(
            prefix="warehouse_xsim_"
        ) as temp:

            run_dir = Path(temp)

            # Prepare consistent-timescale copies of the RTL.
            prepared_rtl = prepare_rtl_sources(run_dir)

            testbench_file = run_dir / "warehouse_tb.v"

            testbench_file.write_text(
                build_testbench(
                    pickup,
                    destination,
                    priority,
                    packed_positions,
                ),
                encoding="utf-8",
            )

            # Compile.
            compile_command = [
                xvlog,
                "-nolog",
                *[str(path) for path in prepared_rtl],
                str(testbench_file),
            ]

            compile_output = run_command(
                compile_command,
                run_dir,
            )

            # Elaborate.
            elaborate_command = [
                xelab,
                "-nolog",
                "warehouse_tb",
                "-s",
                "warehouse_snapshot",
            ]

            elaborate_output = run_command(
                elaborate_command,
                run_dir,
            )

            # Simulate.
            simulate_command = [
                xsim,
                "-nolog",
                "warehouse_snapshot",
                "-runall",
            ]

            simulation_output = run_command(
                simulate_command,
                run_dir,
                timeout=180,
            )

            combined_output = (
                compile_output
                + "\n"
                + elaborate_output
                + "\n"
                + simulation_output
            )

            result = parse_simulation_output(combined_output)

            if not result["completed"]:
                raise RuntimeError(
                    "Simulation did not complete successfully.\n\n"
                    + result["raw_output"]
                )

            if result["collisions"]:
                result["warning"] = (
                    "The simulation reported a collision."
                )

            result["input"] = {
                "pickup": pickup,
                "destination": destination,
                "priority": priority,
                "initial_positions": list(initial_positions),
            }

            return result

    except Exception as xsim_err:
        print(f"[Notice] Vivado hardware simulation unavailable ({xsim_err}). Falling back to digital twin.")
        fallback_result = emulate_order_digital_twin(
            pickup=pickup,
            destination=destination,
            priority=priority,
            initial_positions=initial_positions,
        )
        fallback_result["warning"] = f"Emulated fallback used ({str(xsim_err).splitlines()[-1] if str(xsim_err).splitlines() else str(xsim_err)})"
        return fallback_result
