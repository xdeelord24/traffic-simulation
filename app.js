const canvas = document.getElementById('sim-canvas');
const ctx = canvas.getContext('2d');

if (!ctx) {
  throw new Error('2D canvas context is required.');
}

const ui = {
  modeButtons: Array.from(document.querySelectorAll('.mode-btn')),
  toggleRun: document.getElementById('toggle-run'),
  stepBtn: document.getElementById('step-btn'),
  clearTraffic: document.getElementById('clear-traffic'),
  clearMap: document.getElementById('clear-map'),
  sampleGrid: document.getElementById('sample-grid'),
  spawnMode: document.getElementById('spawn-mode'),
  spawnRate: document.getElementById('spawn-rate'),
  spawnRateValue: document.getElementById('spawn-rate-value'),
  autoSpawn: document.getElementById('auto-spawn'),
  stats: document.getElementById('stats'),
  inspector: document.getElementById('inspector'),
  weightTable: document.getElementById('weight-table'),
  stageWrap: document.getElementById('stage-wrap'),
  appShell: document.getElementById('app-shell'),
};

const INTERSECTION_RADIUS = 16;
const ROAD_WIDTH = 16;
const LANE_OFFSET = 5;
const LANE_WIDTH = 6; // Spacing between lane centers (vehicle width = 6)
const ROAD_LANE_SPACING = 11; // Road width growth per extra lane (drawing)
const VEHICLE_LENGTH = 12;
const SAFE_GAP = 11;
const STOP_BUFFER = 17;
const ACCELERATION = 42;
const DECELERATION = 64;
const INTERSECTION_RESERVE_DISTANCE = 28;
const INTERSECTION_HOLD_SPEED = 8;
const INTERSECTION_RELEASE_DISTANCE = VEHICLE_LENGTH + SAFE_GAP;
const COLLISION_DISTANCE = 4;
const PRE_INTERSECTION_PROGRESS = 0.999;
const AGENT_INTERVAL_SECONDS = 1.0;
const FIXED_DT = 1 / 60;
const APPROACH_ORDER = ['N', 'E', 'S', 'W'];
const ADAPTIVE_QUEUE_WEIGHT = 2.8;
const ADAPTIVE_FLOW_WEIGHT = 1.2;
const ADAPTIVE_WAIT_AGE_WEIGHT = 0.5;
const ADAPTIVE_SWITCH_MARGIN = 0.5;
const ADAPTIVE_SWITCH_MARGIN_RATIO = 1.25;
const ADAPTIVE_DRY_LANE_THRESHOLD = 0.3;
const ADAPTIVE_STARVATION_SECONDS = 6;
const ADAPTIVE_COORDINATION_BONUS = 0.8;

const palette = ['#8ecae6', '#ffb703', '#fb8500', '#90be6d', '#f9844a'];

const state = {
  mode: 'add-intersection',
  intersections: [],
  roads: [],
  vehicles: [],
  running: true,
  autoSpawn: true,
  spawnMode: 'weighted',
  spawnRatePerMinute: 26,
  simulationTime: 0,
  spawnAccumulator: 0,
  selectedType: null,
  selectedId: null,
  connectStartId: null,
  dragIntersectionId: null,
  nextIntersectionId: 1,
  nextRoadId: 1,
  nextVehicleId: 1,
  lastQueueMetrics: new Map(),
  lastFlowMetrics: new Map(),
  lastOverlapPairs: 0,
  collisionWarningTicks: 0,
  trafficLightAgent: null,
  rlTrafficLightAgent: null,
  pendingAgentDecision: false,
  lastAgentDecisionAt: 0,
  vehiclesCompleted: 0,
  phaseChangeCount: 0,
  backtestSeed: null,
  currentScenarioId: null,
};

function seededRandom() {
  if (state.backtestSeed == null) return Math.random();
  let x = state.backtestSeed;
  x ^= x << 13;
  x ^= x >>> 17;
  x ^= x << 5;
  state.backtestSeed = (x >>> 0) || 1;
  return (x >>> 0) / 4294967296;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function round(value, digits = 2) {
  const scale = 10 ** digits;
  return Math.round(value * scale) / scale;
}

function distance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function normalize(vec) {
  const len = Math.hypot(vec.x, vec.y) || 1;
  return { x: vec.x / len, y: vec.y / len };
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function idNumber(id) {
  if (!id) return 0;
  const numeric = Number(String(id).replace(/[^0-9]/g, ''));
  return Number.isFinite(numeric) ? numeric : 0;
}

function getIntersectionById(id) {
  return state.intersections.find((item) => item.id === id) || null;
}

function getRoadById(id) {
  return state.roads.find((item) => item.id === id) || null;
}

function segmentOrientation(a, b) {
  return Math.abs(b.x - a.x) >= Math.abs(b.y - a.y) ? 'EW' : 'NS';
}

function laneOffsetForDirection(startId, endId, laneIndex = 0, lanes = 1) {
  // For 1 lane per direction: single offset. For 2+ lanes: offset per lane within lane group.
  if (lanes <= 1) return LANE_OFFSET;
  const base = 4;
  return base + laneIndex * LANE_WIDTH;
}

function getRoadDrawWidth(lanes) {
  const n = lanes ?? 1;
  if (n <= 1) return ROAD_WIDTH;
  return ROAD_WIDTH + (n - 1) * ROAD_LANE_SPACING;
}

function makeEmptyQueue() {
  return { N: 0, E: 0, S: 0, W: 0 };
}

function laneKey(fromId, toId) {
  return `${fromId}->${toId}`;
}

function ensureSignalWaitAges(signal) {
  if (!signal) return;
  if (!signal.waitAges || typeof signal.waitAges !== 'object') {
    signal.waitAges = makeEmptyQueue();
  }
  for (const approach of APPROACH_ORDER) {
    const value = Number(signal.waitAges[approach]);
    signal.waitAges[approach] = Number.isFinite(value) ? Math.max(0, value) : 0;
  }
}

function chooseBestApproach(availableApproaches, scores) {
  let best = null;
  for (const approach of availableApproaches) {
    if (!best || (scores[approach] || 0) > (scores[best] || 0)) {
      best = approach;
    }
  }
  return best;
}

function getApproachKey(start, end) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  if (Math.abs(dx) >= Math.abs(dy)) {
    return dx >= 0 ? 'W' : 'E';
  }
  return dy >= 0 ? 'N' : 'S';
}

function getIntersectionApproaches(intersectionId) {
  const intersection = getIntersectionById(intersectionId);
  if (!intersection) return [];

  const approaches = new Set();
  for (const road of state.roads) {
    let neighborId = null;
    if (road.from === intersectionId) {
      neighborId = road.to;
    } else if (road.to === intersectionId) {
      neighborId = road.from;
    }
    if (!neighborId) continue;
    const neighbor = getIntersectionById(neighborId);
    if (!neighbor) continue;
    approaches.add(getApproachKey(neighbor, intersection));
  }

  return APPROACH_ORDER.filter((approach) => approaches.has(approach));
}

function resolveApproach(intersection, value, queue = null) {
  if (!intersection) return null;
  const available = getIntersectionApproaches(intersection.id);
  if (available.length === 0) return null;

  if (APPROACH_ORDER.includes(value) && available.includes(value)) {
    return value;
  }

  const aliasMap = {
    NS: ['N', 'S'],
    EW: ['E', 'W'],
  };
  const aliasApproaches = aliasMap[value];
  if (aliasApproaches) {
    const candidates = aliasApproaches.filter((approach) => available.includes(approach));
    if (candidates.length === 0) return null;
    if (!queue) return candidates[0];
    candidates.sort((a, b) => (queue[b] || 0) - (queue[a] || 0));
    return candidates[0];
  }

  return null;
}

function makeIntersection(x, y) {
  return {
    id: `I${state.nextIntersectionId++}`,
    x,
    y,
    entryWeight: 1,
    exitWeight: 1,
    signal: {
      phase: 'N',
      elapsed: 0,
      mode: 'adaptive',
      fixedGreen: 8,
      minGreen: 3,
      maxGreen: 12,
      waitAges: makeEmptyQueue(),
    },
  };
}

function makeRoad(from, to) {
  return {
    id: `R${state.nextRoadId++}`,
    from,
    to,
    speedLimit: 52,
    lanes: 2, // 1–8 = lanes per direction (2 = 4 lanes total, default)
  };
}

function makeVehicle(route) {
  return {
    id: `V${state.nextVehicleId++}`,
    route,
    segmentIndex: 0,
    progress: 0,
    speed: 0,
    laneIndex: 0,
    color: palette[(state.nextVehicleId - 1) % palette.length],
  };
}

function roadExists(a, b) {
  return state.roads.some(
    (road) =>
      (road.from === a && road.to === b) ||
      (road.from === b && road.to === a)
  );
}

function addIntersection(x, y, shouldSelect = true) {
  const intersection = makeIntersection(x, y);
  state.intersections.push(intersection);
  if (shouldSelect) {
    selectObject('intersection', intersection.id);
  }
  renderWeightTable();
  return intersection;
}

function addRoad(a, b, shouldSelect = true) {
  if (!a || !b || a === b || roadExists(a, b)) {
    return null;
  }
  const road = makeRoad(a, b);
  state.roads.push(road);
  if (shouldSelect) {
    selectObject('road', road.id);
  }
  return road;
}

function clearVehicles() {
  state.vehicles = [];
  state.lastOverlapPairs = 0;
  state.collisionWarningTicks = 0;
}

function clearMap() {
  state.intersections = [];
  state.roads = [];
  state.vehicles = [];
  state.selectedType = null;
  state.selectedId = null;
  state.connectStartId = null;
  state.spawnAccumulator = 0;
  state.nextIntersectionId = 1;
  state.nextRoadId = 1;
  state.nextVehicleId = 1;
  state.lastQueueMetrics = new Map();
  state.lastFlowMetrics = new Map();
  state.lastOverlapPairs = 0;
  state.collisionWarningTicks = 0;
  state.vehiclesCompleted = 0;
  state.phaseChangeCount = 0;
  state.currentScenarioId = null;
  renderWeightTable();
  renderInspector();
}

function getPointerPosition(event) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
}

function findIntersectionAt(x, y) {
  return (
    state.intersections.find((node) => distance(node, { x, y }) <= INTERSECTION_RADIUS + 6) ||
    null
  );
}

function pointToSegmentDistance(point, a, b) {
  const vx = b.x - a.x;
  const vy = b.y - a.y;
  const wx = point.x - a.x;
  const wy = point.y - a.y;
  const lenSq = vx * vx + vy * vy;

  if (lenSq === 0) {
    return Math.hypot(point.x - a.x, point.y - a.y);
  }

  const t = clamp((wx * vx + wy * vy) / lenSq, 0, 1);
  const px = a.x + t * vx;
  const py = a.y + t * vy;
  return Math.hypot(point.x - px, point.y - py);
}

function findRoadAt(x, y) {
  let hit = null;
  let bestDistance = Infinity;
  for (const road of state.roads) {
    const a = getIntersectionById(road.from);
    const b = getIntersectionById(road.to);
    if (!a || !b) continue;

    const lanes = road.lanes ?? 2;
    const roadWidth = getRoadDrawWidth(lanes);
    const d = pointToSegmentDistance({ x, y }, a, b);
    if (d < bestDistance && d <= roadWidth * 0.7) {
      bestDistance = d;
      hit = road;
    }
  }
  return hit;
}

function selectObject(type, id) {
  state.selectedType = type;
  state.selectedId = id;
  renderInspector();
}

function removeIntersection(id) {
  state.intersections = state.intersections.filter((node) => node.id !== id);
  state.roads = state.roads.filter((road) => road.from !== id && road.to !== id);
  state.vehicles = state.vehicles.filter((vehicle) => !vehicle.route.includes(id));
  if (state.connectStartId === id) {
    state.connectStartId = null;
  }
  if (state.selectedType === 'intersection' && state.selectedId === id) {
    state.selectedType = null;
    state.selectedId = null;
  }
  renderWeightTable();
  renderInspector();
}

function removeRoad(id) {
  const road = getRoadById(id);
  if (!road) return;

  state.roads = state.roads.filter((item) => item.id !== id);
  state.vehicles = state.vehicles.filter((vehicle) => {
    for (let i = vehicle.segmentIndex; i < vehicle.route.length - 1; i += 1) {
      const from = vehicle.route[i];
      const to = vehicle.route[i + 1];
      if (
        (road.from === from && road.to === to) ||
        (road.from === to && road.to === from)
      ) {
        return false;
      }
    }
    return true;
  });

  if (state.selectedType === 'road' && state.selectedId === id) {
    state.selectedType = null;
    state.selectedId = null;
  }

  renderInspector();
}

function redistributeVehiclesOnRoad(road) {
  const lanes = road.lanes ?? 2;
  if (lanes <= 1) return;

  const dir1 = state.vehicles.filter((v) => {
    const seg = getVehicleSegment(v);
    return seg && seg.startId === road.from && seg.endId === road.to;
  });
  const dir2 = state.vehicles.filter((v) => {
    const seg = getVehicleSegment(v);
    return seg && seg.startId === road.to && seg.endId === road.from;
  });

  for (const vehicles of [dir1, dir2]) {
    vehicles.sort((a, b) => a.progress - b.progress);
    vehicles.forEach((v, i) => {
      v.laneIndex = i % lanes;
    });
  }
}

function setMode(mode) {
  state.mode = mode;
  if (mode !== 'connect-roads') {
    state.connectStartId = null;
  }
  updateModeButtons();
}

function updateModeButtons() {
  for (const button of ui.modeButtons) {
    const isActive = button.dataset.mode === state.mode;
    button.classList.toggle('active', isActive);
  }
}

function updateRunButton() {
  if (ui.toggleRun) ui.toggleRun.textContent = state.running ? 'Pause' : 'Resume';
}

function updateSpawnControls() {
  if (ui.spawnMode) ui.spawnMode.value = state.spawnMode;
  if (ui.spawnRate) ui.spawnRate.value = String(state.spawnRatePerMinute);
  if (ui.autoSpawn) ui.autoSpawn.checked = state.autoSpawn;
  if (ui.spawnRateValue) ui.spawnRateValue.textContent = `${Math.round(state.spawnRatePerMinute)}/min`;
}

function renderWeightTable() {
  if (!ui.weightTable) return;
  if (state.intersections.length === 0) {
    ui.weightTable.innerHTML = '<div class="tiny">No intersections yet.</div>';
    return;
  }

  const rows = state.intersections
    .map(
      (node) => `
      <tr>
        <td>${node.id}</td>
        <td>
          <input type="number" min="0" step="0.1" data-weight-id="${node.id}" data-weight-field="entry" value="${node.entryWeight.toFixed(1)}" />
        </td>
        <td>
          <input type="number" min="0" step="0.1" data-weight-id="${node.id}" data-weight-field="exit" value="${node.exitWeight.toFixed(1)}" />
        </td>
      </tr>
    `
    )
    .join('');

  ui.weightTable.innerHTML = `
    <table class="weights-table">
      <thead>
        <tr>
          <th>ID</th>
          <th>Entry Weight</th>
          <th>Exit Weight</th>
        </tr>
      </thead>
      <tbody>${rows}</tbody>
    </table>
  `;
}

function renderInspector() {
  if (!ui.inspector) return;
  if (!state.selectedType || !state.selectedId) {
    ui.inspector.innerHTML = 'Select an intersection or road.';
    return;
  }

  if (state.selectedType === 'intersection') {
    const node = getIntersectionById(state.selectedId);
    if (!node) {
      ui.inspector.innerHTML = 'Select an intersection or road.';
      return;
    }

    const hasTrafficLight = node.signal.mode !== 'none';
    const approachNames = {
      N: 'North inbound',
      E: 'East inbound',
      S: 'South inbound',
      W: 'West inbound',
    };
    const availableApproaches = getIntersectionApproaches(node.id);
    if (hasTrafficLight && availableApproaches.length > 0 && !availableApproaches.includes(node.signal.phase)) {
      node.signal.phase = availableApproaches[0];
      node.signal.elapsed = 0;
    }
    const phaseOptions = availableApproaches.length
      ? availableApproaches
          .map(
            (approach) =>
              `<option value="${approach}" ${node.signal.phase === approach ? 'selected' : ''}>${approachNames[approach]}</option>`
          )
          .join('')
      : '<option value="N">No inbound roads</option>';

    ui.inspector.innerHTML = `
      <div class="stack">
        <strong>${node.id}</strong>
        <div class="inline">
          <label>
            Entry weight
            <input data-inspector="entryWeight" type="number" min="0" step="0.1" value="${node.entryWeight.toFixed(1)}" />
          </label>
          <label>
            Exit weight
            <input data-inspector="exitWeight" type="number" min="0" step="0.1" value="${node.exitWeight.toFixed(1)}" />
          </label>
        </div>
        <label>
          Signal mode
          <select data-inspector="signalMode">
            <option value="none" ${node.signal.mode === 'none' ? 'selected' : ''}>No traffic light (free flow)</option>
            <option value="adaptive" ${node.signal.mode === 'adaptive' ? 'selected' : ''}>Adaptive AI (pressure)</option>
            <option value="rl" ${node.signal.mode === 'rl' ? 'selected' : ''}>RL Agent (trained)</option>
            <option value="fixed" ${node.signal.mode === 'fixed' ? 'selected' : ''}>Static (fixed cycle, typical)</option>
            <option value="external" ${node.signal.mode === 'external' ? 'selected' : ''}>External AI controlled</option>
          </select>
        </label>
        ${hasTrafficLight
          ? `
        <label>
          Current green road
          <select data-inspector="phaseDirection">
            ${phaseOptions}
          </select>
        </label>
        <div class="inline">
          <label>
            Min green (s)
            <input data-inspector="minGreen" type="number" min="1" step="1" value="${Math.round(node.signal.minGreen)}" />
          </label>
          <label>
            Max green (s)
            <input data-inspector="maxGreen" type="number" min="2" step="1" value="${Math.round(node.signal.maxGreen)}" />
          </label>
        </div>
        <label>
          Fixed green duration (s)
          <input data-inspector="fixedGreen" type="number" min="2" step="1" value="${Math.round(node.signal.fixedGreen)}" />
        </label>
        `
          : `<span class="tiny">No signal control here. Vehicles will pass through this intersection.</span>`}
        <div class="button-row">
          ${hasTrafficLight ? '<button data-inspector-action="switch-phase">Switch Phase Now</button>' : ''}
          <button data-inspector-action="delete-intersection" class="danger">Delete Intersection</button>
        </div>
        <span class="tiny">Position: (${Math.round(node.x)}, ${Math.round(node.y)})</span>
      </div>
    `;
    return;
  }

  if (state.selectedType === 'road') {
    const road = getRoadById(state.selectedId);
    if (!road) {
      ui.inspector.innerHTML = 'Select an intersection or road.';
      return;
    }

    const lanes = road.lanes ?? 2;
    ui.inspector.innerHTML = `
      <div class="stack">
        <strong>${road.id}</strong>
        <span>${road.from} <-> ${road.to}</span>
        <label>
          Speed limit (px/s)
          <input data-inspector="speedLimit" type="number" min="10" max="160" step="1" value="${Math.round(road.speedLimit)}" />
        </label>
        <label>
          Lanes (road widening)
          <select data-inspector="lanes">
            <option value="1" ${lanes === 1 ? 'selected' : ''}>2 lanes (1 per direction)</option>
            <option value="2" ${lanes === 2 ? 'selected' : ''}>4 lanes (2 per direction)</option>
            <option value="3" ${lanes === 3 ? 'selected' : ''}>6 lanes (3 per direction)</option>
            <option value="4" ${lanes === 4 ? 'selected' : ''}>8 lanes (4 per direction)</option>
            <option value="5" ${lanes === 5 ? 'selected' : ''}>10 lanes (5 per direction)</option>
            <option value="6" ${lanes === 6 ? 'selected' : ''}>12 lanes (6 per direction)</option>
            <option value="7" ${lanes === 7 ? 'selected' : ''}>14 lanes (7 per direction)</option>
            <option value="8" ${lanes === 8 ? 'selected' : ''}>16 lanes (8 per direction)</option>
          </select>
        </label>
        <button data-inspector-action="delete-road" class="danger">Delete Road</button>
      </div>
    `;
  }
}

function togglePhase(intersection, nextPhase = null, queue = null) {
  if (!intersection || intersection.signal.mode === 'none') return;

  const available = getIntersectionApproaches(intersection.id);
  if (available.length === 0) return;
  ensureSignalWaitAges(intersection.signal);

  let resolved = resolveApproach(intersection, nextPhase, queue);
  if (!resolved) {
    const currentIndex = available.indexOf(intersection.signal.phase);
    if (currentIndex === -1) {
      resolved = available[0];
    } else {
      resolved = available[(currentIndex + 1) % available.length];
    }
  }

  const previousPhase = intersection.signal.phase;
  intersection.signal.phase = resolved;
  intersection.signal.elapsed = 0;
  intersection.signal.waitAges[resolved] = 0;
  if (resolved !== previousPhase) {
    state.phaseChangeCount += 1;
  }
}

function handleCanvasPointerDown(event) {
  const pointer = getPointerPosition(event);
  const hitIntersection = findIntersectionAt(pointer.x, pointer.y);

  if (state.mode === 'add-intersection') {
    if (hitIntersection) {
      selectObject('intersection', hitIntersection.id);
      return;
    }

    const node = addIntersection(pointer.x, pointer.y, true);
    selectObject('intersection', node.id);
    return;
  }

  if (state.mode === 'connect-roads') {
    if (!hitIntersection) {
      return;
    }

    if (!state.connectStartId) {
      state.connectStartId = hitIntersection.id;
      selectObject('intersection', hitIntersection.id);
      return;
    }

    if (state.connectStartId === hitIntersection.id) {
      state.connectStartId = null;
      return;
    }

    const created = addRoad(state.connectStartId, hitIntersection.id, true);
    state.connectStartId = hitIntersection.id;
    if (created) {
      selectObject('road', created.id);
    }
    return;
  }

  if (state.mode === 'select') {
    if (hitIntersection) {
      state.dragIntersectionId = hitIntersection.id;
      selectObject('intersection', hitIntersection.id);
      return;
    }

    const hitRoad = findRoadAt(pointer.x, pointer.y);
    if (hitRoad) {
      selectObject('road', hitRoad.id);
      return;
    }

    state.selectedType = null;
    state.selectedId = null;
    renderInspector();
  }
}

function handleCanvasPointerMove(event) {
  if (!state.dragIntersectionId) return;
  const pointer = getPointerPosition(event);
  const node = getIntersectionById(state.dragIntersectionId);
  if (!node) return;

  node.x = clamp(pointer.x, 24, canvas.width - 24);
  node.y = clamp(pointer.y, 24, canvas.height - 24);
}

function handleCanvasPointerUp() {
  state.dragIntersectionId = null;
}

function buildNeighborGraph() {
  const graph = new Map();
  for (const node of state.intersections) {
    graph.set(node.id, []);
  }

  for (const road of state.roads) {
    const a = getIntersectionById(road.from);
    const b = getIntersectionById(road.to);
    if (!a || !b) continue;

    const cost = distance(a, b);
    graph.get(road.from)?.push({ id: road.to, cost });
    graph.get(road.to)?.push({ id: road.from, cost });
  }

  return graph;
}

function shortestPath(startId, endId, previousNodeId = null) {
  if (startId === endId) {
    return [startId];
  }

  const graph = buildNeighborGraph();
  if (!graph.has(startId) || !graph.has(endId)) {
    return null;
  }

  // State: (nodeId, prevNodeId) to prevent immediate U-turns.
  // previousNodeId is optionally provided for cases where the route is prefixed
  // by an already-traversed edge (boundary spawn entry).
  const stateKey = (node, prev) => `${node}:${prev ?? ''}`;
  const dist = new Map();
  const prev = new Map(); // prev[stateKey] = { nodeId, prevNodeId }
  const unvisited = new Set();
  const startState = stateKey(startId, previousNodeId);
  dist.set(startState, 0);
  prev.set(startState, null);
  unvisited.add(startState);

  while (unvisited.size > 0) {
    let bestState = null;
    let best = Number.POSITIVE_INFINITY;

    for (const key of unvisited) {
      const value = dist.get(key) ?? Number.POSITIVE_INFINITY;
      if (value < best) {
        best = value;
        bestState = key;
      }
    }

    if (!bestState || best === Number.POSITIVE_INFINITY) {
      break;
    }

    unvisited.delete(bestState);
    const [currentId, prevId] = bestState.split(':');
    const prevNodeId = prevId || null;

    if (currentId === endId) {
      const path = [];
      let s = bestState;
      while (s) {
        const [n] = s.split(':');
        path.unshift(n);
        s = prev.get(s);
      }
      return path;
    }

    const neighbors = graph.get(currentId) || [];
    for (const neighbor of neighbors) {
      if (neighbor.id === prevNodeId) continue; // No U-turn: don't go back to previous node
      const nextState = stateKey(neighbor.id, currentId);
      const alt = best + neighbor.cost;
      if (alt < (dist.get(nextState) ?? Number.POSITIVE_INFINITY)) {
        dist.set(nextState, alt);
        prev.set(nextState, bestState);
        unvisited.add(nextState);
      }
    }
  }

  return null;
}

function pickRandomItem(array) {
  if (array.length === 0) return null;
  return array[Math.floor(seededRandom() * array.length)] || null;
}

function pickByWeight(items, weightAccessor) {
  if (items.length === 0) return null;

  const weights = items.map((item) => Math.max(0, weightAccessor(item)));
  const total = weights.reduce((sum, value) => sum + value, 0);

  if (total <= 0) {
    return pickRandomItem(items);
  }

  let roll = seededRandom() * total;
  for (let i = 0; i < items.length; i += 1) {
    roll -= weights[i];
    if (roll <= 0) {
      return items[i];
    }
  }

  return items[items.length - 1];
}

function buildAdjacencyMap() {
  const adjacency = new Map();
  for (const node of state.intersections) {
    adjacency.set(node.id, []);
  }

  for (const road of state.roads) {
    adjacency.get(road.from)?.push(road.to);
    adjacency.get(road.to)?.push(road.from);
  }

  return adjacency;
}

function buildBoundarySpawnPorts() {
  if (state.intersections.length < 2) {
    return { entryPorts: [], boundaryIds: [] };
  }

  const xs = state.intersections.map((node) => node.x);
  const ys = state.intersections.map((node) => node.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const marginX = Math.max(8, (maxX - minX) * 0.08);
  const marginY = Math.max(8, (maxY - minY) * 0.08);

  const center = {
    x: state.intersections.reduce((sum, node) => sum + node.x, 0) / state.intersections.length,
    y: state.intersections.reduce((sum, node) => sum + node.y, 0) / state.intersections.length,
  };

  const adjacency = buildAdjacencyMap();
  const entryPorts = [];
  const boundaryIds = [];

  for (const node of state.intersections) {
    const onBoundary =
      node.x <= minX + marginX ||
      node.x >= maxX - marginX ||
      node.y <= minY + marginY ||
      node.y >= maxY - marginY;

    if (!onBoundary) continue;
    boundaryIds.push(node.id);

    const neighbors = adjacency.get(node.id) || [];
    if (neighbors.length === 0) continue;

    const nodeDistance = distance(node, center);
    let bestNeighborId = null;
    let bestInwardGain = Number.NEGATIVE_INFINITY;

    for (const neighborId of neighbors) {
      const neighbor = getIntersectionById(neighborId);
      if (!neighbor) continue;
      const inwardGain = nodeDistance - distance(neighbor, center);
      if (inwardGain > bestInwardGain) {
        bestInwardGain = inwardGain;
        bestNeighborId = neighborId;
      }
    }

    if (bestNeighborId) {
      entryPorts.push({
        entryId: node.id,
        nextId: bestNeighborId,
      });
    }
  }

  return { entryPorts, boundaryIds };
}

function findRoadBetween(a, b) {
  return (
    state.roads.find(
      (road) =>
        (road.from === a && road.to === b) ||
        (road.from === b && road.to === a)
    ) || null
  );
}

function isSpawnLaneBlocked(route) {
  if (!Array.isArray(route) || route.length < 2) return true;

  const start = route[0];
  const next = route[1];
  const startNode = getIntersectionById(start);
  const nextNode = getIntersectionById(next);
  const laneLength =
    startNode && nextNode ? Math.max(1, distance(startNode, nextNode)) : 120;
  const minSpawnProgress = (VEHICLE_LENGTH + SAFE_GAP) / laneLength;

  return state.vehicles.some(
    (vehicle) =>
      vehicle.route[vehicle.segmentIndex] === start &&
      vehicle.route[vehicle.segmentIndex + 1] === next &&
      vehicle.progress <= minSpawnProgress
  );
}

function spawnVehicle() {
  if (state.intersections.length < 2 || state.roads.length === 0) {
    return false;
  }

  let route = null;
  const { entryPorts, boundaryIds } = buildBoundarySpawnPorts();

  if (entryPorts.length > 0 && boundaryIds.length > 1) {
    let entryPort = null;
    let exitBoundaryId = null;

    if (state.spawnMode === 'random') {
      entryPort = pickRandomItem(entryPorts);
      const exitCandidates = boundaryIds.filter((id) => id !== entryPort?.entryId);
      exitBoundaryId = pickRandomItem(exitCandidates);
    } else {
      entryPort = pickByWeight(
        entryPorts,
        (port) => getIntersectionById(port.entryId)?.entryWeight ?? 1
      );
      const exitCandidates = boundaryIds.filter((id) => id !== entryPort?.entryId);
      exitBoundaryId = pickByWeight(
        exitCandidates,
        (id) => getIntersectionById(id)?.exitWeight ?? 1
      );
    }

    if (entryPort && exitBoundaryId) {
      const innerRoute = shortestPath(
        entryPort.nextId,
        exitBoundaryId,
        entryPort.entryId
      );
      if (innerRoute && innerRoute.length >= 1) {
        route = [entryPort.entryId, ...innerRoute];
      }
    }
  }

  // Fallback for tiny/custom maps where boundary ports cannot be inferred.
  if (!route) {
    let origin = null;
    let destination = null;

    if (state.spawnMode === 'random') {
      origin = pickRandomItem(state.intersections);
      const candidates = state.intersections.filter((node) => node.id !== origin?.id);
      destination = pickRandomItem(candidates);
    } else {
      origin = pickByWeight(state.intersections, (node) => node.entryWeight);
      const candidates = state.intersections.filter((node) => node.id !== origin?.id);
      destination = pickByWeight(candidates, (node) => node.exitWeight);
    }

    if (!origin || !destination) {
      return false;
    }

    route = shortestPath(origin.id, destination.id);
  }

  if (!route || route.length < 2 || isSpawnLaneBlocked(route)) {
    return false;
  }

  const vehicle = makeVehicle(route);
  const firstSegment = getVehicleSegment(vehicle);
  if (firstSegment) {
    const startProgress = INTERSECTION_RELEASE_DISTANCE / firstSegment.length;
    vehicle.progress = clamp(startProgress, 0, 0.8);
    const road = firstSegment.road;
    const lanes = road?.lanes ?? 1;
    if (lanes > 1) {
      const picked = pickLaneWithSpace(
        firstSegment.startId,
        firstSegment.endId,
        vehicle.id,
        startProgress + (VEHICLE_LENGTH + SAFE_GAP) / firstSegment.length,
        road
      );
      vehicle.laneIndex = picked >= 0 ? picked : 0;
    } else {
      vehicle.laneIndex = 0;
    }
  }
  state.vehicles.push(vehicle);
  return true;
}

function getVehicleSegment(vehicle) {
  const startId = vehicle.route[vehicle.segmentIndex];
  const endId = vehicle.route[vehicle.segmentIndex + 1];
  if (!startId || !endId) {
    return null;
  }

  const start = getIntersectionById(startId);
  const end = getIntersectionById(endId);
  const road = findRoadBetween(startId, endId);

  if (!start || !end || !road) {
    return null;
  }

  return {
    startId,
    endId,
    start,
    end,
    road,
    length: Math.max(1, distance(start, end)),
  };
}

function getVehicleWorldPose(vehicle) {
  const segment = getVehicleSegment(vehicle);
  if (!segment) return null;

  const t = clamp(vehicle.progress, 0, 1);
  const baseX = lerp(segment.start.x, segment.end.x, t);
  const baseY = lerp(segment.start.y, segment.end.y, t);
  const dir = normalize({
    x: segment.end.x - segment.start.x,
    y: segment.end.y - segment.start.y,
  });
  const perp = { x: -dir.y, y: dir.x };
  const lanes = segment.road?.lanes ?? 1;
  const laneIndex = Math.min(vehicle.laneIndex ?? 0, Math.max(0, lanes - 1));
  const laneOffset = laneOffsetForDirection(segment.startId, segment.endId, laneIndex, lanes);

  return {
    x: baseX + perp.x * laneOffset,
    y: baseY + perp.y * laneOffset,
    heading: Math.atan2(dir.y, dir.x),
    segment,
  };
}

function buildIntersectionReservations() {
  const reservations = new Map();

  function reserveClosest(intersectionId, vehicleId, distanceFromCenter) {
    const existing = reservations.get(intersectionId);
    if (!existing || distanceFromCenter < existing.distanceFromCenter) {
      reservations.set(intersectionId, {
        vehicleId,
        distanceFromCenter,
      });
    }
  }

  for (const vehicle of state.vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;

    const remainingDistance = segment.length * (1 - vehicle.progress);
    // Only vehicles with green can reserve the intersection they're approaching.
    // Otherwise a stopped vehicle at red blocks vehicles with green from entering.
    if (remainingDistance <= INTERSECTION_RESERVE_DISTANCE && isLaneGreen(segment)) {
      reserveClosest(segment.endId, vehicle.id, remainingDistance);
    }

    const distanceFromStart = segment.length * vehicle.progress;
    if (distanceFromStart <= INTERSECTION_RELEASE_DISTANCE) {
      reserveClosest(segment.startId, vehicle.id, distanceFromStart);
    }
  }

  const ownerMap = new Map();
  for (const [intersectionId, data] of reservations.entries()) {
    ownerMap.set(intersectionId, data.vehicleId);
  }
  return ownerMap;
}

function hasLaneEntrySpace(startId, endId, vehicleId, requiredFrontProgress, laneIndex = 0) {
  const road = findRoadBetween(startId, endId);
  const lanes = road?.lanes ?? 1;
  const checkLane = Math.min(laneIndex ?? 0, Math.max(0, lanes - 1));

  for (const other of state.vehicles) {
    if (other.id === vehicleId) continue;
    const otherSegment = getVehicleSegment(other);
    if (!otherSegment) continue;
    if (otherSegment.startId !== startId || otherSegment.endId !== endId) continue;
    if (lanes > 1) {
      const otherLane = Math.min(other.laneIndex ?? 0, Math.max(0, lanes - 1));
      if (otherLane !== checkLane) continue;
    }
    if (other.progress < requiredFrontProgress) {
      return false;
    }
  }
  return true;
}

function pickLaneWithSpace(startId, endId, vehicleId, requiredFrontProgress, road) {
  const lanes = road?.lanes ?? 1;
  for (let i = 0; i < lanes; i += 1) {
    if (hasLaneEntrySpace(startId, endId, vehicleId, requiredFrontProgress, i)) {
      return i;
    }
  }
  return -1;
}

function enforceLaneSeparation(vehicles) {
  const lanes = new Map();

  for (const vehicle of vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;
    const roadLanes = segment.road?.lanes ?? 1;
    const laneIndex = Math.min(vehicle.laneIndex ?? 0, Math.max(0, roadLanes - 1));
    const key = `${segment.startId}->${segment.endId}:${laneIndex}`;
    if (!lanes.has(key)) {
      lanes.set(key, []);
    }
    lanes.get(key).push({ vehicle, segment });
  }

  for (const laneEntries of lanes.values()) {
    laneEntries.sort((a, b) => b.vehicle.progress - a.vehicle.progress);
    for (let i = 1; i < laneEntries.length; i += 1) {
      const lead = laneEntries[i - 1];
      const follower = laneEntries[i];
      const minProgressGap =
        (VEHICLE_LENGTH + SAFE_GAP) / Math.max(1, follower.segment.length);
      const maxFollowerProgress = lead.vehicle.progress - minProgressGap;

      if (follower.vehicle.progress > maxFollowerProgress) {
        follower.vehicle.progress = Math.max(0, maxFollowerProgress);
        follower.vehicle.speed = Math.min(follower.vehicle.speed, lead.vehicle.speed);
      }
    }
  }
}

function countVehicleOverlaps() {
  const poses = [];
  for (const vehicle of state.vehicles) {
    const pose = getVehicleWorldPose(vehicle);
    if (pose) {
      poses.push(pose);
    }
  }

  let overlaps = 0;
  for (let i = 0; i < poses.length; i += 1) {
    for (let j = i + 1; j < poses.length; j += 1) {
      const dx = poses[i].x - poses[j].x;
      const dy = poses[i].y - poses[j].y;
      if (Math.hypot(dx, dy) < COLLISION_DISTANCE) {
        overlaps += 1;
      }
    }
  }

  return overlaps;
}

function buildLaneGapMap() {
  const byLane = new Map();

  for (const vehicle of state.vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;
    const lanes = segment.road?.lanes ?? 1;
    const laneIndex = Math.min(vehicle.laneIndex ?? 0, Math.max(0, lanes - 1));
    const key = `${segment.startId}->${segment.endId}:${laneIndex}`;
    if (!byLane.has(key)) {
      byLane.set(key, []);
    }
    byLane.get(key).push({
      vehicle,
      progress: vehicle.progress,
      length: segment.length,
    });
  }

  const gapMap = new Map();

  for (const laneList of byLane.values()) {
    laneList.sort((a, b) => b.progress - a.progress);
    for (let i = 1; i < laneList.length; i += 1) {
      const front = laneList[i - 1];
      const follower = laneList[i];
      const gap = (front.progress - follower.progress) * follower.length - VEHICLE_LENGTH;
      gapMap.set(follower.vehicle.id, Math.max(0, gap));
    }
  }

  return gapMap;
}

function isLaneGreen(segment) {
  const target = getIntersectionById(segment.endId);
  if (!target) return true;
  if (target.signal.mode === 'none') return true;
  const approach = getApproachKey(segment.start, segment.end);
  return target.signal.phase === approach;
}

function computeQueueMetrics() {
  const metrics = new Map();
  for (const node of state.intersections) {
    metrics.set(node.id, makeEmptyQueue());
  }

  for (const vehicle of state.vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;

    if (vehicle.progress < 0.64 || vehicle.speed > 20) {
      continue;
    }

    const queue = metrics.get(segment.endId);
    if (!queue) continue;

    const approach = getApproachKey(segment.start, segment.end);
    queue[approach] += 1;
  }

  return metrics;
}

function buildLaneLoadMap() {
  const loads = new Map();
  for (const vehicle of state.vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;
    const key = laneKey(segment.startId, segment.endId);
    loads.set(key, (loads.get(key) || 0) + 1);
  }
  return loads;
}

function computeFlowMetrics(laneLoads) {
  const metrics = new Map();
  for (const node of state.intersections) {
    metrics.set(node.id, makeEmptyQueue());
  }

  for (const vehicle of state.vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;

    const flow = metrics.get(segment.endId);
    if (!flow) continue;

    const approach = getApproachKey(segment.start, segment.end);
    const proximity = clamp((vehicle.progress - 0.15) / 0.85, 0, 1);
    const lowSpeedBonus = vehicle.speed < 8 ? 0.45 : vehicle.speed < 18 ? 0.2 : 0;
    let contribution = 0.2 + proximity * 1.05 + lowSpeedBonus;

    const nextEndId = vehicle.route[vehicle.segmentIndex + 2];
    if (nextEndId) {
      const downStart = getIntersectionById(segment.endId);
      const downEnd = getIntersectionById(nextEndId);
      if (downStart && downEnd) {
        const downstreamKey = laneKey(downStart.id, downEnd.id);
        const downstreamLoad = laneLoads.get(downstreamKey) || 0;
        const downstreamLength = Math.max(1, distance(downStart, downEnd));
        const downstreamCapacity = Math.max(2, downstreamLength / (VEHICLE_LENGTH + SAFE_GAP));
        const occupancy = clamp(downstreamLoad / downstreamCapacity, 0, 2);
        const releaseFactor = clamp(1.2 - occupancy * 0.65, 0.45, 1.2);
        contribution *= releaseFactor;
      }
    }

    flow[approach] += contribution;
  }

  return metrics;
}

function computeApproachScores(signal, queue, flow, availableApproaches) {
  ensureSignalWaitAges(signal);
  const scores = makeEmptyQueue();
  for (const approach of availableApproaches) {
    scores[approach] =
      (queue[approach] || 0) * ADAPTIVE_QUEUE_WEIGHT +
      (flow[approach] || 0) * ADAPTIVE_FLOW_WEIGHT +
      (signal.waitAges[approach] || 0) * ADAPTIVE_WAIT_AGE_WEIGHT;
  }
  return scores;
}

function getCoordinationBonuses(node) {
  const out = { N: 0, E: 0, S: 0, W: 0 };
  const scenario = state.currentScenarioId;
  if (scenario !== 'ring' && scenario !== 'linear') return out;

  for (const road of state.roads) {
    if (road.to !== node.id) continue;
    const neighbor = getIntersectionById(road.from);
    if (!neighbor) continue;

    const approachFromNeighbor = getApproachKey(neighbor, node);
    const neighborPhase = neighbor.signal?.phase;
    if (!neighborPhase) continue;

    const prevRoad = state.roads.find(
      (r) =>
        (r.from === road.from && r.to !== node.id) ||
        (r.to === road.from && r.from !== node.id)
    );
    if (!prevRoad) continue;
    const prevId = prevRoad.from === road.from ? prevRoad.to : prevRoad.from;
    const prevNode = getIntersectionById(prevId);
    if (!prevNode) continue;

    const approachAtNeighborForUs = getApproachKey(prevNode, neighbor);
    if (neighborPhase === approachAtNeighborForUs) {
      out[approachFromNeighbor] = ADAPTIVE_COORDINATION_BONUS;
    }
  }
  return out;
}

function applyCoordinationBonus(node, scores, availableApproaches) {
  const bonuses = getCoordinationBonuses(node);
  for (const approach of availableApproaches) {
    const b = bonuses[approach] || 0;
    if (b > 0) scores[approach] = (scores[approach] || 0) + b;
  }
}

function updateWaitAges(signal, queue, flow, availableApproaches, dt) {
  ensureSignalWaitAges(signal);
  for (const approach of APPROACH_ORDER) {
    if (!availableApproaches.includes(approach)) {
      signal.waitAges[approach] = 0;
      continue;
    }

    const demand = (queue[approach] || 0) + (flow[approach] || 0) * 0.45;
    if (approach === signal.phase) {
      signal.waitAges[approach] = Math.max(0, signal.waitAges[approach] - dt * 2.5);
    } else if (demand > 0.3) {
      signal.waitAges[approach] += dt;
    } else {
      signal.waitAges[approach] = Math.max(0, signal.waitAges[approach] - dt * 0.4);
    }
  }
}

function maybeAskTrafficAgent() {
  const needsExternalAgent =
    typeof state.trafficLightAgent === 'function' &&
    state.intersections.some((node) => node.signal.mode === 'external');
  const needsRlAgent =
    typeof state.rlTrafficLightAgent === 'function' &&
    state.intersections.some((node) => node.signal.mode === 'rl');

  if (!needsExternalAgent && !needsRlAgent) {
    return;
  }

  if (state.pendingAgentDecision) {
    return;
  }

  if (state.simulationTime - state.lastAgentDecisionAt < AGENT_INTERVAL_SECONDS) {
    return;
  }

  state.lastAgentDecisionAt = state.simulationTime;
  state.pendingAgentDecision = true;

  const snapshot = buildTrafficSnapshot();
  const agentCalls = [];
  if (needsExternalAgent) {
    agentCalls.push(
      Promise.resolve(state.trafficLightAgent(snapshot)).then((decisions) => {
        applyTrafficLightDecisions(decisions, 'external');
      })
    );
  }
  if (needsRlAgent) {
    agentCalls.push(
      Promise.resolve(state.rlTrafficLightAgent(snapshot)).then((decisions) => {
        applyTrafficLightDecisions(decisions, 'rl');
      })
    );
  }

  Promise.all(agentCalls)
    .catch((error) => {
      console.error('Traffic agent error:', error);
    })
    .finally(() => {
      state.pendingAgentDecision = false;
    });
}

function applyTrafficLightDecisions(decisions, requiredMode = null) {
  if (!decisions) return;

  if (Array.isArray(decisions)) {
    for (const entry of decisions) {
      if (!entry || !entry.id) continue;
      const node = getIntersectionById(entry.id);
      if (!node) continue;
      if (requiredMode && node.signal.mode !== requiredMode) continue;
      const queue = state.lastQueueMetrics.get(node.id) || makeEmptyQueue();
      if (typeof entry.phase === 'string' || typeof entry.approach === 'string') {
        togglePhase(node, entry.approach || entry.phase, queue);
      } else if (entry.switch === true) {
        togglePhase(node, null, queue);
      }
    }
    return;
  }

  if (typeof decisions !== 'object') {
    return;
  }

  if (decisions.phases && typeof decisions.phases === 'object') {
    for (const [id, phase] of Object.entries(decisions.phases)) {
      const node = getIntersectionById(id);
      if (!node) continue;
      if (requiredMode && node.signal.mode !== requiredMode) continue;
      if (typeof phase === 'string') {
        const queue = state.lastQueueMetrics.get(node.id) || makeEmptyQueue();
        togglePhase(node, phase, queue);
      }
    }
  }

  if (Array.isArray(decisions.switch)) {
    for (const id of decisions.switch) {
      const node = getIntersectionById(id);
      if (!node) continue;
      if (requiredMode && node.signal.mode !== requiredMode) continue;
      const queue = state.lastQueueMetrics.get(node.id) || makeEmptyQueue();
      togglePhase(node, null, queue);
    }
  }
}

function updateSignals(dt, queueMetrics) {
  const laneLoads = buildLaneLoadMap();
  const flowMetrics = computeFlowMetrics(laneLoads);
  state.lastFlowMetrics = flowMetrics;

  for (const node of state.intersections) {
    const signal = node.signal;
    const availableApproaches = getIntersectionApproaches(node.id);
    if (availableApproaches.length === 0) {
      signal.elapsed = 0;
      ensureSignalWaitAges(signal);
      for (const approach of APPROACH_ORDER) {
        signal.waitAges[approach] = 0;
      }
      continue;
    }

    if (signal.mode === 'none') {
      signal.elapsed = 0;
      ensureSignalWaitAges(signal);
      for (const approach of APPROACH_ORDER) {
        signal.waitAges[approach] = 0;
      }
      continue;
    }

    ensureSignalWaitAges(signal);
    if (!availableApproaches.includes(signal.phase)) {
      signal.phase = availableApproaches[0];
      signal.elapsed = 0;
    }

    signal.elapsed += dt;
    const queue = queueMetrics.get(node.id) || makeEmptyQueue();
    const flow = flowMetrics.get(node.id) || makeEmptyQueue();
    updateWaitAges(signal, queue, flow, availableApproaches, dt);
    const scores = computeApproachScores(signal, queue, flow, availableApproaches);
    applyCoordinationBonus(node, scores, availableApproaches);
    const bestApproach = chooseBestApproach(availableApproaches, scores);
    const currentScore = scores[signal.phase] || 0;
    const bestScore = bestApproach ? scores[bestApproach] || 0 : 0;
    const starvationApproach = chooseBestApproach(availableApproaches, signal.waitAges);
    const starvationForced =
      starvationApproach &&
      starvationApproach !== signal.phase &&
      (signal.waitAges[starvationApproach] || 0) >= ADAPTIVE_STARVATION_SECONDS;

    let downstreamOccupancy = 0;
    const exitRoads = state.roads.filter((r) => r.from === node.id);
    if (exitRoads.length > 0) {
      let sum = 0;
      for (const r of exitRoads) {
        const start = getIntersectionById(r.from);
        const end = getIntersectionById(r.to);
        const len = start && end ? distance(start, end) : 0;
        const cap = Math.max(2, len / (VEHICLE_LENGTH + SAFE_GAP));
        const load = laneLoads.get(laneKey(r.from, r.to)) || 0;
        sum += clamp(load / cap, 0, 2);
      }
      downstreamOccupancy = sum / exitRoads.length;
    }
    const downstreamFull = downstreamOccupancy > 0.85;
    const switchMarginMult = downstreamFull ? 1.4 : 1;
    const marginAbs = ADAPTIVE_SWITCH_MARGIN * switchMarginMult;
    const marginRatio = Math.pow(ADAPTIVE_SWITCH_MARGIN_RATIO, switchMarginMult);

    const currentDemand = (queue[signal.phase] || 0) + (flow[signal.phase] || 0) * 0.5;
    const betterDemandAbs =
      bestApproach &&
      bestApproach !== signal.phase &&
      bestScore >= currentScore + marginAbs;
    const betterDemandRatio =
      bestApproach &&
      bestApproach !== signal.phase &&
      currentScore > 0.1 &&
      bestScore >= currentScore * marginRatio;
    const betterDemand = betterDemandAbs || betterDemandRatio;
    const alternateDemand =
      bestApproach &&
      bestApproach !== signal.phase &&
      bestScore > 0.2;

    if (signal.mode === 'fixed') {
      if (signal.elapsed >= signal.fixedGreen) {
        togglePhase(node, null, queue);
      }
      continue;
    }

    if (signal.mode === 'adaptive' || (signal.mode === 'rl' && typeof state.rlTrafficLightAgent !== 'function')) {
      if (signal.elapsed >= signal.maxGreen) {
        const forced = starvationForced ? starvationApproach : alternateDemand ? bestApproach : null;
        togglePhase(node, forced, queue);
        continue;
      }

      const shouldSwitch =
        signal.elapsed >= signal.minGreen &&
        (starvationForced ||
          betterDemand ||
          (currentDemand <= ADAPTIVE_DRY_LANE_THRESHOLD && alternateDemand));

      if (shouldSwitch) {
        const next = starvationForced ? starvationApproach : bestApproach;
        if (next) {
          togglePhase(node, next, queue);
        }
      }
      continue;
    }

    if ((signal.mode === 'external' || signal.mode === 'rl') && signal.elapsed >= signal.maxGreen * 2) {
      // Safety fallback when an external or RL agent is silent.
      const fallback = starvationForced ? starvationApproach : alternateDemand ? bestApproach : null;
      togglePhase(node, fallback, queue);
    }
  }
}

function updateVehicles(dt) {
  const gapMap = buildLaneGapMap();
  const intersectionReservations = buildIntersectionReservations();
  const survivors = [];

  for (const vehicle of state.vehicles) {
    const tickStartProgress = vehicle.progress;
    let segment = getVehicleSegment(vehicle);
    if (!segment) {
      continue;
    }

    let targetSpeed = segment.road.speedLimit;
    const gap = gapMap.get(vehicle.id);

    if (typeof gap === 'number') {
      targetSpeed = Math.min(targetSpeed, Math.max(0, gap * 2.3));
    }

    const remainingDistance = segment.length * (1 - vehicle.progress);
    const laneGreen = isLaneGreen(segment);
    const reservationOwner = intersectionReservations.get(segment.endId);
    const approachingIntersection = remainingDistance <= INTERSECTION_RESERVE_DISTANCE;
    const intersectionBlocked =
      approachingIntersection &&
      reservationOwner &&
      reservationOwner !== vehicle.id;
    const canProceed = laneGreen && !intersectionBlocked;

    if (!canProceed) {
      const stopDistance = Math.max(0, remainingDistance - STOP_BUFFER);
      if (stopDistance < 56) {
        targetSpeed = Math.min(targetSpeed, stopDistance * 2.2);
      }
    }

    const accelLimit = targetSpeed >= vehicle.speed ? ACCELERATION : DECELERATION;
    const diff = targetSpeed - vehicle.speed;
    const maxChange = accelLimit * dt;
    if (Math.abs(diff) > maxChange) {
      vehicle.speed += Math.sign(diff) * maxChange;
    } else {
      vehicle.speed = targetSpeed;
    }

    let moveDistance = Math.max(0, vehicle.speed * dt);

    if (!canProceed) {
      const stopDistance = Math.max(0, remainingDistance - STOP_BUFFER);
      moveDistance = Math.min(moveDistance, stopDistance);
    }

    if (typeof gap === 'number') {
      moveDistance = Math.min(moveDistance, Math.max(0, gap - SAFE_GAP));
    }

    vehicle.progress += moveDistance / segment.length;

    let arrived = false;
    let haltedAtIntersection = false;

    while (vehicle.progress >= 1) {
      const crossingIntersectionId = segment.endId;
      const crossingOwner = intersectionReservations.get(crossingIntersectionId);
      const crossingGreen = isLaneGreen(segment);
      const canEnterIntersection =
        crossingGreen &&
        (!crossingOwner || crossingOwner === vehicle.id);

      if (!canEnterIntersection) {
        const holdProgress = clamp(
          Math.max(tickStartProgress, PRE_INTERSECTION_PROGRESS),
          0,
          0.9999
        );
        vehicle.progress = Math.min(vehicle.progress, holdProgress);
        vehicle.speed = Math.min(vehicle.speed, INTERSECTION_HOLD_SPEED);
        haltedAtIntersection = true;
        break;
      }

      const overflow = (vehicle.progress - 1) * segment.length;
      const nextStartId = crossingIntersectionId;
      const nextEndId = vehicle.route[vehicle.segmentIndex + 2];
      const nextRoad = nextEndId ? findRoadBetween(nextStartId, nextEndId) : null;
      if (nextEndId && nextRoad) {
        const nextStart = getIntersectionById(nextStartId);
        const nextEnd = getIntersectionById(nextEndId);
        if (!nextStart || !nextEnd) {
          arrived = true;
          break;
        }

        const nextLength = Math.max(1, distance(nextStart, nextEnd));
        const minEntryProgress = INTERSECTION_RELEASE_DISTANCE / nextLength;
        const projectedProgress = clamp(
          Math.max(overflow / nextLength, minEntryProgress),
          0,
          0.98
        );
        const requiredFrontProgress =
          projectedProgress + (VEHICLE_LENGTH + SAFE_GAP) / nextLength;
        const pickedLane = pickLaneWithSpace(
          nextStartId,
          nextEndId,
          vehicle.id,
          requiredFrontProgress,
          nextRoad
        );
        if (pickedLane < 0) {
          const holdProgress = clamp(
            Math.max(tickStartProgress, PRE_INTERSECTION_PROGRESS),
            0,
            0.9999
          );
          vehicle.progress = Math.min(vehicle.progress, holdProgress);
          vehicle.speed = Math.min(vehicle.speed, INTERSECTION_HOLD_SPEED);
          haltedAtIntersection = true;
          break;
        }
        vehicle.laneIndex = pickedLane;
      }

      // Reserve this intersection for this tick so only one vehicle crosses at a time.
      intersectionReservations.set(crossingIntersectionId, vehicle.id);
      vehicle.segmentIndex += 1;

      if (vehicle.segmentIndex >= vehicle.route.length - 1) {
        arrived = true;
        break;
      }

      segment = getVehicleSegment(vehicle);
      if (!segment) {
        arrived = true;
        break;
      }

      const minExitProgress = INTERSECTION_RELEASE_DISTANCE / segment.length;
      vehicle.progress = clamp(
        Math.max(overflow / segment.length, minExitProgress),
        0,
        0.98
      );
    }

    if (haltedAtIntersection && vehicle.speed < 0.2) {
      vehicle.speed = 0;
    }

    if (arrived) {
      state.vehiclesCompleted += 1;
    } else {
      survivors.push(vehicle);
    }
  }

  state.vehicles = survivors;
  enforceLaneSeparation(state.vehicles);
  state.lastOverlapPairs = countVehicleOverlaps();
  if (state.lastOverlapPairs > 0) {
    state.collisionWarningTicks += 1;
  }
}

function simulate(dt) {
  state.simulationTime += dt;

  if (state.autoSpawn && state.spawnRatePerMinute > 0) {
    state.spawnAccumulator += (state.spawnRatePerMinute / 60) * dt;
    while (state.spawnAccumulator >= 1) {
      spawnVehicle();
      state.spawnAccumulator -= 1;
    }
  }

  state.lastQueueMetrics = computeQueueMetrics();
  updateSignals(dt, state.lastQueueMetrics);
  maybeAskTrafficAgent();
  updateVehicles(dt);
}

function stepSimulation(seconds) {
  let remaining = Math.max(0, seconds);
  while (remaining > 0) {
    const dt = Math.min(FIXED_DT, remaining);
    simulate(dt);
    remaining -= dt;
  }
}

function drawBackground() {
  const gradient = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
  gradient.addColorStop(0, '#2d4f6a');
  gradient.addColorStop(0.5, '#1d3248');
  gradient.addColorStop(1, '#102033');
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = 'rgba(210, 228, 238, 0.06)';
  ctx.lineWidth = 1;
  const step = 42;
  for (let x = 0; x <= canvas.width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 0; y <= canvas.height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }
}

function drawRoads() {
  for (const road of state.roads) {
    const start = getIntersectionById(road.from);
    const end = getIntersectionById(road.to);
    if (!start || !end) continue;

    const selected = state.selectedType === 'road' && state.selectedId === road.id;
    const lanes = road.lanes ?? 2;
    const roadWidth = getRoadDrawWidth(lanes);

    ctx.lineCap = 'round';
    ctx.strokeStyle = selected ? '#ffd166' : '#6f8095';
    ctx.lineWidth = selected ? roadWidth + 3 : roadWidth;
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();

    ctx.strokeStyle = 'rgba(13, 24, 35, 0.55)';
    ctx.lineWidth = 2;
    ctx.setLineDash([12, 8]);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
    ctx.setLineDash([]);

    const midX = (start.x + end.x) / 2;
    const midY = (start.y + end.y) / 2;
    ctx.fillStyle = 'rgba(12, 18, 25, 0.72)';
    ctx.fillRect(midX - 24, midY - 11, 48, 14);
    ctx.fillStyle = '#d9e6ef';
    ctx.font = '11px "Space Grotesk", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${Math.round(road.speedLimit)}`, midX, midY - 4);
    if (lanes > 1) {
      ctx.font = '9px "Space Grotesk", sans-serif';
      ctx.fillStyle = 'rgba(217, 230, 239, 0.85)';
      ctx.fillText(`${lanes * 2} lanes`, midX, midY + 4);
    }
  }
}

function drawIntersection(node) {
  const queue = state.lastQueueMetrics.get(node.id) || makeEmptyQueue();
  const congestion = queue.N + queue.E + queue.S + queue.W;
  const freeFlow = node.signal.mode === 'none';

  const selected = state.selectedType === 'intersection' && state.selectedId === node.id;
  const anchor = state.connectStartId === node.id;

  ctx.beginPath();
  ctx.arc(node.x, node.y, INTERSECTION_RADIUS + (selected ? 5 : 0), 0, Math.PI * 2);
  ctx.fillStyle = selected
    ? 'rgba(255, 209, 102, 0.32)'
    : anchor
    ? 'rgba(251, 133, 0, 0.32)'
    : 'rgba(15, 31, 46, 0.58)';
  ctx.fill();

  ctx.beginPath();
  ctx.arc(node.x, node.y, INTERSECTION_RADIUS, 0, Math.PI * 2);
  ctx.fillStyle = freeFlow
    ? '#30739f'
    : congestion >= 6
    ? '#cb6e2b'
    : congestion >= 3
    ? '#2a9d8f'
    : '#30739f';
  ctx.fill();

  if (freeFlow) {
    ctx.beginPath();
    ctx.arc(node.x, node.y, 3.2, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(229, 239, 247, 0.9)';
    ctx.fill();
  } else {
    const available = getIntersectionApproaches(node.id);
    const signals = {
      N: { x: -2, y: -13, w: 4, h: 9 },
      E: { x: 4, y: -2, w: 9, h: 4 },
      S: { x: -2, y: 4, w: 4, h: 9 },
      W: { x: -13, y: -2, w: 9, h: 4 },
    };

    for (const approach of APPROACH_ORDER) {
      if (!available.includes(approach)) continue;
      const sig = signals[approach];
      ctx.fillStyle = node.signal.phase === approach ? '#74d48b' : '#f05d5e';
      ctx.fillRect(node.x + sig.x, node.y + sig.y, sig.w, sig.h);
    }
  }

  ctx.fillStyle = '#f2f7fb';
  ctx.font = '11px "Space Grotesk", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.fillText(node.id, node.x, node.y + INTERSECTION_RADIUS + 4);
}

function drawIntersections() {
  for (const node of state.intersections) {
    drawIntersection(node);
  }
}

function drawVehicle(vehicle) {
  const pose = getVehicleWorldPose(vehicle);
  if (!pose) return;

  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.heading);

  ctx.fillStyle = 'rgba(9, 15, 23, 0.44)';
  ctx.fillRect(-VEHICLE_LENGTH * 0.5 - 1, -4, VEHICLE_LENGTH + 2, 8);

  ctx.fillStyle = vehicle.color;
  ctx.fillRect(-VEHICLE_LENGTH * 0.5, -3, VEHICLE_LENGTH, 6);

  ctx.restore();
}

function drawVehicles() {
  for (const vehicle of state.vehicles) {
    drawVehicle(vehicle);
  }
}

function drawHud() {
  const modeLabel = {
    'add-intersection': 'Add Intersection',
    'connect-roads': 'Connect Roads',
    select: 'Select / Move',
  }[state.mode];

  ctx.fillStyle = 'rgba(6, 13, 20, 0.52)';
  ctx.fillRect(12, 12, 360, 60);

  ctx.fillStyle = '#e7f2fb';
  ctx.font = '15px "Space Grotesk", sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText(`Mode: ${modeLabel}`, 20, 22);
  ctx.fillText(
    `Traffic: ${state.spawnMode} @ ${Math.round(state.spawnRatePerMinute)}/min`,
    20,
    44
  );

  if (state.mode === 'connect-roads' && state.connectStartId) {
    ctx.fillStyle = 'rgba(251, 133, 0, 0.65)';
    ctx.fillRect(12, 80, 390, 28);
    ctx.fillStyle = '#fff6ea';
    ctx.fillText(`Connect from ${state.connectStartId}: click another intersection`, 18, 88);
  }
}

function render() {
  drawBackground();
  drawRoads();
  drawVehicles();
  drawIntersections();
  drawHud();
  updateStats();
}

function getAverageSpeed() {
  if (state.vehicles.length === 0) return 0;
  const total = state.vehicles.reduce((sum, vehicle) => sum + vehicle.speed, 0);
  return total / state.vehicles.length;
}

function updateStats() {
  let totalQueue = 0;
  let hotSpot = null;
  let hotSpotLoad = -1;

  for (const [id, queue] of state.lastQueueMetrics.entries()) {
    const load = queue.N + queue.E + queue.S + queue.W;
    totalQueue += load;
    if (load > hotSpotLoad) {
      hotSpotLoad = load;
      hotSpot = id;
    }
  }

  const lines = [
    `sim_time_s: ${round(state.simulationTime, 1)}`,
    `running: ${state.running}`,
    `mode: ${state.mode}`,
    `intersections: ${state.intersections.length}`,
    `roads: ${state.roads.length}`,
    `vehicles: ${state.vehicles.length}`,
    `vehicles_completed: ${state.vehiclesCompleted}`,
    `avg_speed_px_s: ${round(getAverageSpeed(), 1)}`,
    `queued_vehicles: ${totalQueue}`,
    `largest_queue_at: ${hotSpot || 'n/a'}`,
    `overlap_pairs: ${state.lastOverlapPairs}`,
    `collision_warning_ticks: ${state.collisionWarningTicks}`,
  ];

  if (ui.stats) ui.stats.textContent = lines.join('\n');
}

function buildTrafficSnapshot() {
  const intersections = state.intersections.map((node) => {
    const queue = state.lastQueueMetrics.get(node.id) || makeEmptyQueue();
    const flow = state.lastFlowMetrics.get(node.id) || makeEmptyQueue();
    ensureSignalWaitAges(node.signal);
    return {
      id: node.id,
      x: round(node.x, 1),
      y: round(node.y, 1),
      entry_weight: round(node.entryWeight, 2),
      exit_weight: round(node.exitWeight, 2),
      signal: {
        phase: node.signal.phase,
        mode: node.signal.mode,
        available_approaches: getIntersectionApproaches(node.id),
        elapsed_s: round(node.signal.elapsed, 2),
        min_green_s: node.signal.minGreen,
        max_green_s: node.signal.maxGreen,
        wait_ages_s: {
          N: round(node.signal.waitAges.N || 0, 2),
          E: round(node.signal.waitAges.E || 0, 2),
          S: round(node.signal.waitAges.S || 0, 2),
          W: round(node.signal.waitAges.W || 0, 2),
        },
      },
      queue: {
        N: round(queue.N || 0, 2),
        E: round(queue.E || 0, 2),
        S: round(queue.S || 0, 2),
        W: round(queue.W || 0, 2),
      },
      approach_flow: {
        N: round(flow.N || 0, 2),
        E: round(flow.E || 0, 2),
        S: round(flow.S || 0, 2),
        W: round(flow.W || 0, 2),
      },
      coordination_bonus: getCoordinationBonuses(node),
      congestion: queue.N + queue.E + queue.S + queue.W,
    };
  });

  const laneLoads = buildLaneLoadMap();
  const roads = state.roads.map((road) => {
    const start = getIntersectionById(road.from);
    const end = getIntersectionById(road.to);
    const length = start && end ? distance(start, end) : 0;
    const capacity = Math.max(2, length / (VEHICLE_LENGTH + SAFE_GAP));
    const loadFwd = laneLoads.get(laneKey(road.from, road.to)) || 0;
    const loadRev = laneLoads.get(laneKey(road.to, road.from)) || 0;
    const occupancyFwd = round(clamp(loadFwd / capacity, 0, 2), 2);
    const occupancyRev = round(clamp(loadRev / capacity, 0, 2), 2);
    return {
      id: road.id,
      from: road.from,
      to: road.to,
      length: round(length, 1),
      speed_limit: round(road.speedLimit, 1),
      lanes: road.lanes ?? 2,
      occupancy_fwd: occupancyFwd,
      occupancy_rev: occupancyRev,
    };
  });

  const vehicles = state.vehicles.slice(0, 120).map((vehicle) => {
    const segment = getVehicleSegment(vehicle);
    return {
      id: vehicle.id,
      lane_from: segment?.startId || null,
      lane_to: segment?.endId || null,
      progress: round(vehicle.progress, 3),
      speed: round(vehicle.speed, 2),
      route_end: vehicle.route[vehicle.route.length - 1] || null,
    };
  });

  return {
    coordinate_system: 'origin=(top-left), +x=right, +y=down, units=canvas pixels',
    mode: state.mode,
    running: state.running,
    time_s: round(state.simulationTime, 2),
    spawn: {
      mode: state.spawnMode,
      rate_per_min: state.spawnRatePerMinute,
      auto: state.autoSpawn,
    },
    selection: {
      type: state.selectedType,
      id: state.selectedId,
      connect_start: state.connectStartId,
    },
    totals: {
      intersections: state.intersections.length,
      roads: state.roads.length,
      vehicles: state.vehicles.length,
      overlap_pairs: state.lastOverlapPairs,
      collision_warning_ticks: state.collisionWarningTicks,
    },
    intersections,
    roads,
    vehicles,
  };
}

function renderGameToText() {
  return JSON.stringify(buildTrafficSnapshot());
}

function advanceTime(ms) {
  const seconds = Math.max(0, ms / 1000);
  stepSimulation(seconds);
  render();
}

function resizeCanvas() {
  const bounds = ui.stageWrap.getBoundingClientRect();
  const width = Math.max(720, Math.floor(bounds.width - 2));
  const height = Math.max(520, Math.floor(bounds.height - 2));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function buildSampleGrid() {
  const centerX = canvas.width * 0.5;
  const centerY = canvas.height * 0.52;
  const dx = Math.min(180, canvas.width * 0.18);
  const dy = Math.min(150, canvas.height * 0.16);

  const ids = [];
  ids.push(addIntersection(centerX - dx, centerY - dy, false).id);
  ids.push(addIntersection(centerX, centerY - dy, false).id);
  ids.push(addIntersection(centerX + dx, centerY - dy, false).id);
  ids.push(addIntersection(centerX - dx, centerY, false).id);
  ids.push(addIntersection(centerX, centerY, false).id);
  ids.push(addIntersection(centerX + dx, centerY, false).id);
  ids.push(addIntersection(centerX - dx, centerY + dy, false).id);
  ids.push(addIntersection(centerX, centerY + dy, false).id);
  ids.push(addIntersection(centerX + dx, centerY + dy, false).id);

  const links = [
    [0, 1], [1, 2],
    [3, 4], [4, 5],
    [6, 7], [7, 8],
    [0, 3], [3, 6],
    [1, 4], [4, 7],
    [2, 5], [5, 8],
  ];

  for (const [a, b] of links) {
    addRoad(ids[a], ids[b], false);
  }

  const center = getIntersectionById(ids[4]);
  if (center) {
    center.entryWeight = 3;
    center.exitWeight = 2;
  }

  renderWeightTable();
  renderInspector();
}

function buildScenario(scenarioId) {
  clearMap();
  const cx = canvas.width * 0.5;
  const cy = canvas.height * 0.52;
  const d = Math.min(140, canvas.width * 0.14);

  const add = (x, y) => addIntersection(x, y, false).id;
  const link = (a, b) => addRoad(a, b, false);

  if (scenarioId === 'grid3x3') {
    const ids = [
      add(cx - d, cy - d), add(cx, cy - d), add(cx + d, cy - d),
      add(cx - d, cy), add(cx, cy), add(cx + d, cy),
      add(cx - d, cy + d), add(cx, cy + d), add(cx + d, cy + d),
    ];
    [[0, 1], [1, 2], [3, 4], [4, 5], [6, 7], [7, 8], [0, 3], [3, 6], [1, 4], [4, 7], [2, 5], [5, 8]].forEach(([a, b]) => link(ids[a], ids[b]));
    const c = getIntersectionById(ids[4]);
    if (c) { c.entryWeight = 3; c.exitWeight = 2; }
  } else if (scenarioId === 'grid4x4') {
    const ids = [];
    for (let row = 0; row < 4; row++) {
      for (let col = 0; col < 4; col++) {
        ids.push(add(cx - 1.5 * d + col * d, cy - 1.5 * d + row * d));
      }
    }
    for (let i = 0; i < 16; i++) {
      if (i % 4 < 3) link(ids[i], ids[i + 1]);
      if (i < 12) link(ids[i], ids[i + 4]);
    }
  } else if (scenarioId === 'linear') {
    const ids = [add(cx - 2 * d, cy), add(cx - d, cy), add(cx, cy), add(cx + d, cy), add(cx + 2 * d, cy)];
    for (let i = 0; i < 4; i++) link(ids[i], ids[i + 1]);
    const c = getIntersectionById(ids[2]);
    if (c) { c.entryWeight = 2; c.exitWeight = 2; }
  } else if (scenarioId === 'cross') {
    const c = add(cx, cy);
    const n = add(cx, cy - d), s = add(cx, cy + d), e = add(cx + d, cy), w = add(cx - d, cy);
    link(c, n); link(c, s); link(c, e); link(c, w);
    const cNode = getIntersectionById(c);
    if (cNode) { cNode.entryWeight = 3; cNode.exitWeight = 2; }
  } else if (scenarioId === 'star') {
    const c = add(cx, cy);
    const n = add(cx, cy - d), s = add(cx, cy + d), e = add(cx + d, cy), w = add(cx - d, cy);
    link(c, n); link(c, s); link(c, e); link(c, w);
    const cNode = getIntersectionById(c);
    if (cNode) { cNode.entryWeight = 2; cNode.exitWeight = 2; }
  } else if (scenarioId === 'diamond') {
    const t = add(cx, cy - d), b = add(cx, cy + d), l = add(cx - d, cy), r = add(cx + d, cy), c = add(cx, cy);
    link(c, t); link(c, b); link(c, l); link(c, r);
    const cNode = getIntersectionById(c);
    if (cNode) { cNode.entryWeight = 2; cNode.exitWeight = 2; }
  } else if (scenarioId === 'bottleneck') {
    const left = [add(cx - 2 * d, cy - d), add(cx - 2 * d, cy), add(cx - 2 * d, cy + d)];
    const right = [add(cx + 2 * d, cy - d), add(cx + 2 * d, cy), add(cx + 2 * d, cy + d)];
    const mid = add(cx, cy);
    link(left[0], mid); link(left[1], mid); link(left[2], mid);
    link(mid, right[0]); link(mid, right[1]); link(mid, right[2]);
    const cNode = getIntersectionById(mid);
    if (cNode) { cNode.entryWeight = 3; cNode.exitWeight = 2; }
  } else if (scenarioId === 'ring') {
    const ids = [
      add(cx, cy - d), add(cx + d, cy - d * 0.5), add(cx + d, cy + d * 0.5),
      add(cx, cy + d), add(cx - d, cy + d * 0.5), add(cx - d, cy - d * 0.5),
    ];
    for (let i = 0; i < 6; i++) link(ids[i], ids[(i + 1) % 6]);
  } else {
    buildSampleGrid();
  }

  state.currentScenarioId = scenarioId || null;
  state.vehiclesCompleted = 0;
  state.simulationTime = 0;
  state.spawnAccumulator = 0;
  state.lastQueueMetrics = new Map();
  state.lastFlowMetrics = new Map();
  state.pendingAgentDecision = false;
  state.lastAgentDecisionAt = 0;
  renderWeightTable();
  renderInspector();
}

function ensureStarterScenario() {
  if (state.intersections.length > 0) {
    return;
  }
  buildSampleGrid();
}

function toggleFullscreen() {
  if (!document.fullscreenElement) {
    ui.appShell.requestFullscreen?.();
  } else {
    document.exitFullscreen?.();
  }
}

function attachUiEvents() {
  ui.modeButtons.forEach((button) => {
    button.addEventListener('click', () => {
      setMode(button.dataset.mode);
    });
  });

  ui.toggleRun?.addEventListener('click', () => {
    state.running = !state.running;
    updateRunButton();
  });

  ui.stepBtn?.addEventListener('click', () => {
    stepSimulation(0.5);
    render();
  });

  ui.clearTraffic?.addEventListener('click', () => {
    clearVehicles();
  });

  ui.clearMap?.addEventListener('click', () => {
    clearMap();
  });

  ui.sampleGrid?.addEventListener('click', () => {
    buildSampleGrid();
  });

  ui.spawnMode?.addEventListener('change', (event) => {
    state.spawnMode = event.target.value === 'random' ? 'random' : 'weighted';
    updateSpawnControls();
  });

  ui.spawnRate?.addEventListener('input', (event) => {
    state.spawnRatePerMinute = clamp(Number(event.target.value) || 0, 0, 120);
    updateSpawnControls();
  });

  ui.autoSpawn?.addEventListener('change', (event) => {
    state.autoSpawn = Boolean(event.target.checked);
    updateSpawnControls();
  });

  ui.weightTable?.addEventListener('input', (event) => {
    const target = event.target;
    const id = target.dataset.weightId;
    const field = target.dataset.weightField;
    if (!id || !field) return;

    const node = getIntersectionById(id);
    if (!node) return;

    const value = Math.max(0, Number(target.value) || 0);
    if (field === 'entry') {
      node.entryWeight = value;
    } else if (field === 'exit') {
      node.exitWeight = value;
    }

    if (state.selectedType === 'intersection' && state.selectedId === node.id) {
      renderInspector();
    }
  });

  ui.inspector?.addEventListener('input', (event) => {
    const key = event.target.dataset.inspector;
    if (!key) return;

    if (state.selectedType === 'intersection') {
      const node = getIntersectionById(state.selectedId);
      if (!node) return;

      if (key === 'entryWeight') {
        node.entryWeight = Math.max(0, Number(event.target.value) || 0);
        renderWeightTable();
      }
      if (key === 'exitWeight') {
        node.exitWeight = Math.max(0, Number(event.target.value) || 0);
        renderWeightTable();
      }
      if (key === 'signalMode') {
        const nextMode = String(event.target.value || 'none');
        node.signal.mode = ['none', 'adaptive', 'fixed', 'external', 'rl'].includes(nextMode)
          ? nextMode
          : 'none';
        ensureSignalWaitAges(node.signal);
        if (node.signal.mode === 'none') {
          node.signal.elapsed = 0;
          for (const approach of APPROACH_ORDER) {
            node.signal.waitAges[approach] = 0;
          }
        } else {
          const available = getIntersectionApproaches(node.id);
          if (available.length > 0 && !available.includes(node.signal.phase)) {
            node.signal.phase = available[0];
          }
        }
        renderInspector();
      }
      if (key === 'phaseDirection') {
        const queue = state.lastQueueMetrics.get(node.id) || makeEmptyQueue();
        togglePhase(node, String(event.target.value || ''), queue);
        renderInspector();
      }
      if (key === 'minGreen') {
        node.signal.minGreen = clamp(Number(event.target.value) || 1, 1, 60);
      }
      if (key === 'maxGreen') {
        node.signal.maxGreen = clamp(Number(event.target.value) || 2, 2, 90);
      }
      if (key === 'fixedGreen') {
        node.signal.fixedGreen = clamp(Number(event.target.value) || 2, 2, 90);
      }
    }

    if (state.selectedType === 'road') {
      const road = getRoadById(state.selectedId);
      if (!road) return;
      if (key === 'speedLimit') {
        road.speedLimit = clamp(Number(event.target.value) || 30, 10, 160);
      }
      if (key === 'lanes') {
        const newLanes = clamp(Number(event.target.value) || 2, 1, 8);
        const oldLanes = road.lanes ?? 2;
        road.lanes = newLanes;
        if (newLanes > 1 && newLanes !== oldLanes) {
          redistributeVehiclesOnRoad(road);
        }
      }
    }
  });

  ui.inspector?.addEventListener('click', (event) => {
    const action = event.target.dataset.inspectorAction;
    if (!action) return;

    if (action === 'switch-phase' && state.selectedType === 'intersection') {
      const node = getIntersectionById(state.selectedId);
      if (node) {
        togglePhase(node);
        renderInspector();
      }
      return;
    }

    if (action === 'delete-intersection' && state.selectedType === 'intersection') {
      removeIntersection(state.selectedId);
      return;
    }

    if (action === 'delete-road' && state.selectedType === 'road') {
      removeRoad(state.selectedId);
    }
  });

  canvas.addEventListener('mousedown', handleCanvasPointerDown);
  canvas.addEventListener('mousemove', handleCanvasPointerMove);
  canvas.addEventListener('mouseup', handleCanvasPointerUp);
  canvas.addEventListener('mouseleave', handleCanvasPointerUp);
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());

  window.addEventListener('resize', () => {
    resizeCanvas();
    render();
  });

  document.addEventListener('fullscreenchange', () => {
    resizeCanvas();
    render();
  });

  document.addEventListener('keydown', (event) => {
    const key = event.key.toLowerCase();

    if (key === 'f') {
      event.preventDefault();
      toggleFullscreen();
      return;
    }

    if (key === 'a') {
      setMode('add-intersection');
      return;
    }

    if (key === 'b') {
      setMode('connect-roads');
      return;
    }

    if (event.key === 'Enter') {
      setMode('select');
      return;
    }

    if (event.code === 'Space') {
      event.preventDefault();
      state.running = !state.running;
      updateRunButton();
    }
  });
}

let previousTimestamp = performance.now();

function frame(timestamp) {
  const dt = Math.min(0.05, (timestamp - previousTimestamp) / 1000);
  previousTimestamp = timestamp;

  if (state.running) {
    stepSimulation(dt);
  }

  render();
  requestAnimationFrame(frame);
}

window.render_game_to_text = renderGameToText;
window.advanceTime = advanceTime;
window.getTrafficSnapshot = () => buildTrafficSnapshot();
window.applyTrafficLightDecisions = (decisions) => applyTrafficLightDecisions(decisions);
window.setTrafficLightAgent = (agentFn) => {
  state.trafficLightAgent = typeof agentFn === 'function' ? agentFn : null;
};
window.setRlTrafficLightAgent = (agentFn) => {
  state.rlTrafficLightAgent = typeof agentFn === 'function' ? agentFn : null;
};
window.stepSimulation = stepSimulation;
window.clearMap = clearMap;
window.buildSampleGrid = buildSampleGrid;
window.buildScenario = buildScenario;
window.getRewardMetrics = () => {
  let totalQueue = 0;
  for (const q of state.lastQueueMetrics.values()) {
    totalQueue += (q.N || 0) + (q.E || 0) + (q.S || 0) + (q.W || 0);
  }
  return {
    vehiclesCompleted: state.vehiclesCompleted,
    totalQueue,
    phaseChanges: state.phaseChangeCount,
    vehiclesActive: state.vehicles.length,
    time_s: state.simulationTime,
  };
};
window.resetPhaseChangeCounter = () => {
  state.phaseChangeCount = 0;
};
window.getBacktestMetrics = () => {
  let totalQueue = 0;
  for (const q of state.lastQueueMetrics.values()) {
    totalQueue += (q.N || 0) + (q.E || 0) + (q.S || 0) + (q.W || 0);
  }
  return {
    vehiclesCompleted: state.vehiclesCompleted,
    vehiclesActive: state.vehicles.length,
    totalQueue,
    time_s: state.simulationTime,
    overlapPairs: state.lastOverlapPairs,
  };
};
window.setSpawnRate = (rate) => {
  state.spawnRatePerMinute = Math.max(0, rate);
  if (ui.spawnRate) ui.spawnRate.value = String(state.spawnRatePerMinute);
  if (ui.spawnRateValue) ui.spawnRateValue.textContent = `${Math.round(state.spawnRatePerMinute)}/min`;
};
window.setBacktestSeed = (seed) => {
  state.backtestSeed = typeof seed === 'number' && seed !== 0 ? (seed >>> 0) || 1 : 1;
};
window.clearBacktestSeed = () => {
  state.backtestSeed = null;
};
window.setAllSignalModes = (mode) => {
  for (const node of state.intersections) {
    node.signal.mode = ['none', 'adaptive', 'fixed', 'external', 'rl'].includes(mode) ? mode : 'adaptive';
    if (node.signal.mode === 'none') node.signal.elapsed = 0;
  }
};
window.setFixedGreenForAll = (seconds) => {
  const s = clamp(Number(seconds) || 8, 2, 90);
  for (const node of state.intersections) {
    node.signal.fixedGreen = s;
  }
};
window.buildScenario = buildScenario;
window.resetBacktest = () => buildScenario('grid3x3');

function init() {
  resizeCanvas();
  attachUiEvents();
  ensureStarterScenario();
  updateModeButtons();
  updateSpawnControls();
  updateRunButton();
  renderWeightTable();
  renderInspector();
  render();
  requestAnimationFrame(frame);
}

init();
