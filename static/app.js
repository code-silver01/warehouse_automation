
const grid = document.getElementById("grid");
const pickupSelect = document.getElementById("pickup");
const destinationSelect = document.getElementById("destination");
const runButton = document.getElementById("runButton");

let animationTimer = null;
let currentSimulation = null;

// --------------------------------------------------
// INITIALIZE 5x5 GRID
// --------------------------------------------------

function initializeGrid() {
    grid.innerHTML = "";

    for (let cell = 0; cell < 25; cell++) {
        const option1 = document.createElement("option");
        option1.value = cell;
        option1.textContent = `Cell ${cell}`;
        pickupSelect.appendChild(option1);

        const option2 = document.createElement("option");
        option2.value = cell;
        option2.textContent = `Cell ${cell}`;
        destinationSelect.appendChild(option2);

        const element = document.createElement("div");
        element.className = "cell";
        element.id = `cell-${cell}`;

        element.innerHTML = `
            <span>${cell}</span>
            <small>R${Math.floor(cell / 5)} C${cell % 5}</small>
        `;

        grid.appendChild(element);
    }

    pickupSelect.value = 6;
    destinationSelect.value = 24;
}

function logMessage(message) {
    const log = document.getElementById("simulationLog");
    const timestamp = new Date().toLocaleTimeString();

    log.textContent += `\n[${timestamp}] ${message}`;
    log.scrollTop = log.scrollHeight;
}

function setMessage(message) {
    document.getElementById("message").textContent = message;
}

// --------------------------------------------------
// GRID DISPLAY
// --------------------------------------------------

function renderGrid(positions = [0, 4, 20], pickup = null,
    destination = null, routeCells = []) {

    for (let cell = 0; cell < 25; cell++) {
        const element = document.getElementById(`cell-${cell}`);

        element.className = "cell";

        if (routeCells.includes(cell)) {
            element.classList.add("route");
        }

        if (cell === pickup) {
            element.classList.add("pickup");
        }

        if (cell === destination) {
            element.classList.add("destination");
        }

        element.innerHTML = `
            <span>${cell}</span>
            <small>R${Math.floor(cell / 5)} C${cell % 5}</small>
        `;

        positions.forEach((position, vehicle) => {
            if (position === cell) {
                const marker = document.createElement("span");
                marker.className = `vehicle-marker marker-${vehicle}`;
                marker.textContent = `V${vehicle}`;
                element.appendChild(marker);
            }
        });
    }
}

// --------------------------------------------------
// ROUTE DECODING
// --------------------------------------------------

function decodeRoute(pathHex, length) {
    const packed = BigInt("0x" + pathHex);
    const path = [];

    for (let i = 0; i < length; i++) {
        const cell = Number((packed >> BigInt(i * 5)) & 31n);
        path.push(cell);
    }

    return path;
}

// --------------------------------------------------
// SIMULATION ANIMATION
// --------------------------------------------------

function animateSimulation(result) {
    if (animationTimer) {
        clearInterval(animationTimer);
        animationTimer = null;
    }

    const samples = result.positions || [];

    if (samples.length === 0) {
        logMessage("No position samples returned by XSim.");
        return;
    }

    const pickup = result.pickup;
    const destination = result.destination;

    let frame = 0;

    const allRouteCells = [];

    (result.routes || []).forEach(route => {
        if (route.success) {
            const decoded = decodeRoute(route.path_hex, route.length);
            decoded.forEach(cell => {
                if (!allRouteCells.includes(cell)) {
                    allRouteCells.push(cell);
                }
            });
        }
    });

    function showFrame() {
        if (frame >= samples.length) {
            clearInterval(animationTimer);
            animationTimer = null;

            const last = samples[samples.length - 1].vehicles;

            renderGrid(last, pickup, destination, allRouteCells);

            logMessage("Vehicle animation completed.");
            setMessage(
                result.completed
                    ? "Simulation completed successfully."
                    : "Simulation ended without confirmed completion."
            );

            return;
        }

        const sample = samples[frame];

        renderGrid(
            sample.vehicles,
            pickup,
            destination,
            allRouteCells
        );

        document.getElementById("vehiclePositions").innerHTML =
            sample.vehicles.map((position, index) => `
                <div>
                    <span class="vehicle-marker marker-${index}">
                        V${index}
                    </span>
                    Vehicle ${index}: Cell ${position}
                </div>
            `).join("");

        document.getElementById("finalPosition").textContent =
            sample.vehicles[0];

        frame++;
    }

    showFrame();

    animationTimer = setInterval(showFrame, 450);
}

// --------------------------------------------------
// ANALYTICS
// --------------------------------------------------

function displayResults(result) {
    const allocations = result.allocations || [];
    const routes = result.routes || [];
    const collisions = result.collisions || [];

    document.getElementById("allocatedVehicle").textContent =
        allocations.length
            ? `V${allocations[0].vehicle}`
            : "--";

    document.getElementById("routeCount").textContent =
        routes.length;

    document.getElementById("collisionCount").textContent =
        collisions.length;

    document.getElementById("routeInfo").innerHTML =
        routes.length
            ? routes.map(route => {
                const path = decodeRoute(
                    route.path_hex,
                    route.length
                );

                return `
                    <div class="route-item">
                        <strong>Vehicle ${route.vehicle}</strong><br>
                        ${route.leg === 0 ? "Pickup route" : "Delivery route"}<br>
                        ${route.start} → ${route.goal}<br>
                        Path: ${path.join(" → ")}<br>
                        Length: ${route.length} cells<br>
                        Status: ${route.success ? "Successful" : "Failed"}
                    </div>
                `;
            }).join("")
            : "<p>No routes returned.</p>";

    if (result.completed) {
        logMessage("Simulation completed.");
    } else {
        logMessage("Simulation did not confirm completion.");
    }

    logMessage(`Allocated vehicles: ${allocations.length}`);
    logMessage(`Routes generated: ${routes.length}`);
    logMessage(`Collisions detected: ${collisions.length}`);

    if (collisions.length > 0) {
        logMessage("WARNING: Collision events detected.");
    }
}

// --------------------------------------------------
// RUN ACTUAL VERILOG SIMULATION
// --------------------------------------------------

async function runSimulation() {
    const pickup = Number(pickupSelect.value);
    const destination = Number(destinationSelect.value);
    const priority = Number(
        document.getElementById("priority").value
    );

    if (pickup === destination) {
        setMessage("Pickup and destination must be different.");
        return;
    }

    if (animationTimer) {
        clearInterval(animationTimer);
        animationTimer = null;
    }

    runButton.disabled = true;
    runButton.textContent = "RUNNING XSIM...";

    setMessage("Compiling and running Verilog simulation...");
    document.getElementById("simulationLog").textContent = "";

    renderGrid([0, 4, 20], pickup, destination);

    document.getElementById("allocatedVehicle").textContent = "--";
    document.getElementById("routeCount").textContent = "0";
    document.getElementById("finalPosition").textContent = "--";
    document.getElementById("collisionCount").textContent = "0";
    document.getElementById("routeInfo").innerHTML =
        "<p>Waiting for XSim...</p>";

    logMessage(`Order submitted: ${pickup} → ${destination}`);
    logMessage(`Priority: ${priority}`);
    logMessage("Starting Vivado XSim...");

    try {
        const response = await fetch("/api/simulate", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                pickup: pickup,
                destination: destination,
                priority: priority
            })
        });

        const result = await response.json();

        if (!response.ok || !result.success) {
            throw new Error(result.error || "Simulation failed.");
        }

        currentSimulation = result;

        displayResults(result);

        logMessage("XSim results received by dashboard.");

        animateSimulation(result);

    } catch (error) {
        console.error(error);

        setMessage("Simulation failed: " + error.message);
        logMessage("ERROR: " + error.message);

    } finally {
        runButton.disabled = false;
        runButton.textContent = "RUN SIMULATION";
    }
}

// --------------------------------------------------
// INITIALIZATION
// --------------------------------------------------

initializeGrid();
renderGrid();

fetch("/api/health")
    .then(response => response.json())
    .then(data => {
        logMessage(
            `Backend connected: ${data.simulator}`
        );
        logMessage(`Warehouse grid: ${data.grid}`);
        logMessage(`Virtual vehicles: ${data.vehicles}`);
    })
    .catch(() => {
        setMessage("Cannot connect to Flask backend.");
    });
