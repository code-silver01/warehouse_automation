
from pathlib import Path
from flask import Flask, render_template, request, jsonify
from backend.sim_runner import simulate_order
import traceback

BASE_DIR = Path(__file__).resolve().parent.parent

app = Flask(
    __name__,
    template_folder=str(BASE_DIR / "templates"),
    static_folder=str(BASE_DIR / "static")
)

GRID_SIZE = 5
VALID_CELLS = set(range(GRID_SIZE * GRID_SIZE))


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/simulate", methods=["POST"])
def run_simulation():
    try:
        data = request.get_json(silent=True) or {}

        pickup = int(data.get("pickup", -1))
        destination = int(data.get("destination", -1))
        priority = int(data.get("priority", 1))

        if pickup not in VALID_CELLS:
            return jsonify({
                "success": False,
                "error": "Pickup must be between 0 and 24."
            }), 400

        if destination not in VALID_CELLS:
            return jsonify({
                "success": False,
                "error": "Destination must be between 0 and 24."
            }), 400

        if pickup == destination:
            return jsonify({
                "success": False,
                "error": "Pickup and destination must differ."
            }), 400

        if priority not in (0, 1, 2, 3):
            return jsonify({
                "success": False,
                "error": "Priority must be between 0 and 3."
            }), 400

        result = simulate_order(
            pickup=pickup,
            destination=destination,
            priority=priority
        )

        return jsonify({
            "success": True,
            "completed": result.get("completed", False),
            "pickup": pickup,
            "destination": destination,
            "priority": priority,
            "allocations": result.get("allocations", []),
            "routes": result.get("routes", []),
            "positions": result.get("positions", []),
            "collisions": result.get("collisions", []),
            "move_conflicts": result.get("move_conflicts", []),
            "raw_output": result.get("raw_output", "")
        })

    except ValueError as error:
        return jsonify({
            "success": False,
            "error": str(error)
        }), 400

    except Exception as error:
        traceback.print_exc()

        return jsonify({
            "success": False,
            "error": "Simulation failed: " + str(error)
        }), 500


@app.route("/api/health")
def health():
    return jsonify({
        "status": "online",
        "simulator": "Vivado XSim 2018.3",
        "grid": "5x5",
        "vehicles": 3
    })


if __name__ == "__main__":
    app.run(
        host="127.0.0.1",
        port=5000,
        debug=True,
        use_reloader=False
    )
