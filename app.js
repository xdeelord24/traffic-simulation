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
  scenarioPreset: document.getElementById('scenario-preset'),
  scenarioLanes: document.getElementById('scenario-lanes'),
  spawnMode: document.getElementById('spawn-mode'),
  spawnRate: document.getElementById('spawn-rate'),
  spawnRateValue: document.getElementById('spawn-rate-value'),
  autoSpawn: document.getElementById('auto-spawn'),
  globalSignalMode: document.getElementById('global-signal-mode'),
  stats: document.getElementById('stats'),
  inspector: document.getElementById('inspector'),
  weightTable: document.getElementById('weight-table'),
  stageWrap: document.getElementById('stage-wrap'),
  appShell: document.getElementById('app-shell'),
};

const INTERSECTION_RADIUS = 20;
const ROAD_WIDTH = 16;
const LANE_OFFSET = 5;
const LANE_WIDTH = 9; // Center spacing between adjacent lanes per direction.
const ROAD_LANE_SPACING = 18; // Visual road width growth per extra lane per direction.
const DEFAULT_VEHICLE_LENGTH = 12;
const DEFAULT_VEHICLE_WIDTH = 6;
const DEFAULT_SAFE_GAP = 11;
const VEHICLE_PROFILES = [
  {
    id: 'motorcycle',
    label: 'Motorcycle',
    length: 8,
    width: 4,
    safeGap: 8,
    speedFactor: 1.2,
    accelerationFactor: 1.2,
    brakingFactor: 0.95,
    spawnWeight: 1.4,
    color: '#7bdff2',
  },
  {
    id: 'compact',
    label: 'Compact',
    length: 10,
    width: 5,
    safeGap: 9,
    speedFactor: 1.08,
    accelerationFactor: 1.05,
    brakingFactor: 1,
    spawnWeight: 1.2,
    color: '#8ecae6',
  },
  {
    id: 'sedan',
    label: 'Sedan',
    length: 12,
    width: 6,
    safeGap: 11,
    speedFactor: 1,
    accelerationFactor: 1,
    brakingFactor: 1,
    spawnWeight: 1.9,
    color: '#ffb703',
  },
  {
    id: 'van',
    label: 'Van',
    length: 14,
    width: 7,
    safeGap: 12,
    speedFactor: 0.9,
    accelerationFactor: 0.9,
    brakingFactor: 1.05,
    spawnWeight: 1,
    color: '#90be6d',
  },
  {
    id: 'truck',
    label: 'Truck',
    length: 17,
    width: 8,
    safeGap: 15,
    speedFactor: 0.76,
    accelerationFactor: 0.72,
    brakingFactor: 1.12,
    spawnWeight: 0.8,
    color: '#f9844a',
  },
];
const AVERAGE_VEHICLE_SPACING =
  VEHICLE_PROFILES.reduce(
    (sum, profile) => sum + (profile.length + profile.safeGap) * (profile.spawnWeight ?? 1),
    0
  ) /
  VEHICLE_PROFILES.reduce((sum, profile) => sum + (profile.spawnWeight ?? 1), 0);
const STOP_BUFFER = 17;
const ACCELERATION = 42;
const DECELERATION = 64;
const INTERSECTION_RESERVE_DISTANCE = 28;
const INTERSECTION_HOLD_SPEED = 8;
const INTERSECTION_EXIT_HOLD_SPEED = 16;
const COLLISION_DISTANCE = 4;
const PRE_INTERSECTION_PROGRESS = 0.999;
const AGENT_INTERVAL_SECONDS = 1.0;
const FIXED_DT = 1 / 60;
const APPROACH_ORDER = ['N', 'E', 'S', 'W'];
const TURN_BLEND_MIN_PROGRESS = 0.72;
const TURN_CONTROL_MIN_DISTANCE = 12;
const TURN_CONTROL_MAX_DISTANCE = 36;
const MAX_LANE_CHANGE_PER_INTERSECTION = 0;
const MAX_STRAIGHT_LANE_CHANGE_PER_INTERSECTION = 1;
const ADAPTIVE_QUEUE_WEIGHT = 2.8;
const ADAPTIVE_FLOW_WEIGHT = 1.2;
const ADAPTIVE_WAIT_AGE_WEIGHT = 0.5;
const ADAPTIVE_SWITCH_MARGIN = 0.5;
const ADAPTIVE_SWITCH_MARGIN_RATIO = 1.25;
const ADAPTIVE_DRY_LANE_THRESHOLD = 0.3;
const ADAPTIVE_STARVATION_SECONDS = 6;
const ADAPTIVE_COORDINATION_BONUS = 0.8;
const SIGNAL_YELLOW_DURATION = 1.4;
const SIGNAL_ALL_RED_DURATION = 0.7;
const YELLOW_CLEARANCE_DISTANCE = STOP_BUFFER + 6;
const STUCK_TICKS_RELAXED_ENTRY = 15;
const SIGNAL_MODE_VALUES = ['none', 'adaptive', 'fixed', 'external', 'rl'];
const MIN_VIEW_ZOOM = 0.45;
const MAX_VIEW_ZOOM = 2.8;
const ZOOM_SENSITIVITY = 0.0015;

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
  defaultRoadLanes: 2,
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
  view: { x: 0, y: 0, zoom: 1 },
  dragPointerMode: null,
  dragPanLastScreen: null,
  dragPanMoved: false,
  dragPanAllowsDeselect: false,
  overlapCheckCounter: 0,
  renderEnabled: true,
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

function getVehicleProfile(typeId) {
  return VEHICLE_PROFILES.find((profile) => profile.id === typeId) || VEHICLE_PROFILES[2];
}

function resolveVehicleProfile(profile) {
  if (!profile) return pickVehicleProfile();
  if (typeof profile === 'string') return getVehicleProfile(profile);
  return getVehicleProfile(profile.id);
}

function pickVehicleProfile() {
  return pickByWeight(VEHICLE_PROFILES, (profile) => profile.spawnWeight ?? 1) || VEHICLE_PROFILES[0];
}

function getVehicleLength(vehicle) {
  const value = Number(vehicle?.length);
  return Number.isFinite(value) && value > 0 ? value : DEFAULT_VEHICLE_LENGTH;
}

function getVehicleWidth(vehicle) {
  const value = Number(vehicle?.width);
  return Number.isFinite(value) && value > 0 ? value : DEFAULT_VEHICLE_WIDTH;
}

function getVehicleSafeGap(vehicle) {
  const value = Number(vehicle?.safeGap);
  return Number.isFinite(value) && value >= 0 ? value : DEFAULT_SAFE_GAP;
}

function getVehicleReleaseDistance(vehicle) {
  return getVehicleLength(vehicle) + getVehicleSafeGap(vehicle);
}

function getVehicleBodyClearance(a, b) {
  return getVehicleLength(a) * 0.5 + getVehicleLength(b) * 0.5;
}

function getVehicleSpacingDistance(lead, follower) {
  return getVehicleBodyClearance(lead, follower) + Math.max(getVehicleSafeGap(lead), getVehicleSafeGap(follower));
}

function getVehicleTargetRoadSpeed(vehicle, road) {
  const speedFactor = Number(vehicle?.speedFactor);
  const scale = Number.isFinite(speedFactor) && speedFactor > 0 ? speedFactor : 1;
  return (road?.speedLimit ?? 0) * scale;
}

function getVehicleCarryThroughSpeed(vehicle, fromRoad, toRoad) {
  const fromTargetSpeed = getVehicleTargetRoadSpeed(vehicle, fromRoad);
  const toTargetSpeed = getVehicleTargetRoadSpeed(vehicle, toRoad);
  if (toTargetSpeed <= 0) {
    return fromTargetSpeed;
  }
  return Math.min(fromTargetSpeed, toTargetSpeed);
}

function getIntersectionExitProgress(segment, vehicle) {
  if (!segment) return 0.3;
  const clearanceDistance = Math.max(
    getVehicleReleaseDistance(vehicle),
    getIntersectionControlRadius(segment) + getVehicleLength(vehicle) * 0.5 + 2
  );
  return clamp(
    clearanceDistance / Math.max(1, segment.length),
    0.08,
    0.42
  );
}

function getIntersectionReleaseDistance(segment, vehicle) {
  if (!segment) return getVehicleReleaseDistance(vehicle);
  return Math.max(
    getVehicleReleaseDistance(vehicle),
    segment.length * getIntersectionExitProgress(segment, vehicle)
  );
}

function buildVehicleMixSummary() {
  const counts = new Map();
  for (const vehicle of state.vehicles) {
    const profile = getVehicleProfile(vehicle.type);
    counts.set(profile.label, (counts.get(profile.label) || 0) + 1);
  }
  return [...counts.entries()]
    .sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0]))
    .map(([label, count]) => `${label}:${count}`)
    .join(', ');
}

function getVehicleNextSegment(vehicle) {
  if (!vehicle) return null;
  const nextStartId = vehicle.route[vehicle.segmentIndex + 1];
  const nextEndId = vehicle.route[vehicle.segmentIndex + 2];
  if (!nextStartId || !nextEndId) return null;
  return getSegmentFromIds(nextStartId, nextEndId);
}

function getVehicleTurnIntent(vehicle) {
  return getVehicleTurnIntentForRouteIndex(vehicle, vehicle?.segmentIndex ?? 0);
}

function getVehicleTurnIntentForRouteIndex(vehicle, routeSegmentIndex = 0) {
  if (!vehicle) return 'straight';
  const startId = vehicle.route?.[routeSegmentIndex];
  const midId = vehicle.route?.[routeSegmentIndex + 1];
  const endId = vehicle.route?.[routeSegmentIndex + 2];
  if (!startId || !midId || !endId) return 'straight';

  const segment = getSegmentFromIds(startId, midId);
  const nextSegment = getSegmentFromIds(midId, endId);
  if (!segment || !nextSegment) return 'straight';

  const currentDir = normalize({
    x: segment.end.x - segment.start.x,
    y: segment.end.y - segment.start.y,
  });
  const nextDir = normalize({
    x: nextSegment.end.x - nextSegment.start.x,
    y: nextSegment.end.y - nextSegment.start.y,
  });
  const cross = currentDir.x * nextDir.y - currentDir.y * nextDir.x;
  const dotValue = dot(currentDir, nextDir);

  if (Math.abs(cross) < 0.25 && dotValue > 0.3) {
    return 'straight';
  }

  return cross > 0 ? 'right' : 'left';
}

function getTurnLanePreference(turnIntent, lanes) {
  const laneCount = Math.max(1, lanes || 1);
  if (laneCount <= 1) return 0;
  if (turnIntent === 'left') return 0;
  if (turnIntent === 'right') return laneCount - 1;
  return null;
}

function getPathDirection(fromId, toId) {
  const from = getIntersectionById(fromId);
  const to = getIntersectionById(toId);
  if (!from || !to) return null;
  return normalize({
    x: to.x - from.x,
    y: to.y - from.y,
  });
}

function isStraightPathTransition(prevNodeId, currentId, nextId) {
  if (!prevNodeId || !currentId || !nextId) return true;
  const incoming = getPathDirection(prevNodeId, currentId);
  const outgoing = getPathDirection(currentId, nextId);
  if (!incoming || !outgoing) return true;
  const cross = incoming.x * outgoing.y - incoming.y * outgoing.x;
  const dotValue = dot(incoming, outgoing);
  return Math.abs(cross) < 0.25 && dotValue > 0.3;
}

function getPathHeadingPenalty(currentId, nextId, endId) {
  const travel = getPathDirection(currentId, nextId);
  const towardEnd = getPathDirection(currentId, endId);
  if (!travel || !towardEnd) return 0;
  return 1 - clamp(dot(travel, towardEnd), -1, 1);
}

function createVehicleLaneDiscipline() {
  return {
    style: 'normal',
    driftOffsetPx: 0,
  };
}

function getVehicleLaneDrift(vehicle) {
  return 0;
}

function getVehicleTurnLanePreference(vehicle, routeSegmentIndex, lanes) {
  return getTurnLanePreference(getVehicleTurnIntentForRouteIndex(vehicle, routeSegmentIndex), lanes);
}

function getLaneShiftLimitForSegment(vehicle, routeSegmentIndex, lanes, fallback = null) {
  const turnLane = getVehicleTurnLanePreference(vehicle, routeSegmentIndex, lanes);
  if (turnLane == null) return fallback;
  return Math.max(1, lanes - 1);
}

function getIntersectionLaneShiftLimit(vehicle, routeSegmentIndex, lanes) {
  const laneCount = Math.max(1, lanes || 1);
  if (laneCount <= 1) return 0;
  const straightFallback = Math.min(MAX_STRAIGHT_LANE_CHANGE_PER_INTERSECTION, laneCount - 1);
  return (
    getLaneShiftLimitForSegment(vehicle, routeSegmentIndex, laneCount, straightFallback) ??
    straightFallback
  );
}

function getLanePreferenceScore(laneIndex, lanes, preferredLaneIndex, routeLaneIndex, vehicle) {
  let score = 0;
  if (preferredLaneIndex != null) {
    score -= Math.abs(laneIndex - preferredLaneIndex) * 10;
  }
  if (routeLaneIndex != null) {
    score -= Math.abs(laneIndex - routeLaneIndex) * 30;
    if (laneIndex === routeLaneIndex) {
      score += 24;
    }
  }
  const looseDriver = vehicle?.laneStyle === 'loose';
  if (looseDriver && routeLaneIndex != null && Math.abs(laneIndex - routeLaneIndex) === 1) {
    score += 3;
  }
  return score;
}

function getVehicleBrakeLightState(vehicle) {
  return Boolean(vehicle?.brakeLightOn);
}

function normalizeSignalMode(mode, fallback = 'adaptive') {
  const value = String(mode || '');
  return SIGNAL_MODE_VALUES.includes(value) ? value : fallback;
}

function normalizeRoadLanes(value, fallback = state.defaultRoadLanes ?? 2) {
  return clamp(Number(value) || fallback || 2, 1, 8);
}

function normalizeScenarioPreset(value) {
  return [
    'sample-grid',
    'grid3x3',
    'grid4x4',
    'linear',
    'cross',
    'star',
    'diamond',
    'bottleneck',
    'ring',
  ].includes(String(value || ''))
    ? String(value)
    : 'sample-grid';
}

function getScenarioPresetSelection() {
  return normalizeScenarioPreset(state.currentScenarioId || 'sample-grid');
}

function updateScenarioPresetControl() {
  if (!ui.scenarioPreset) return;
  ui.scenarioPreset.value = getScenarioPresetSelection();
}

function getScenarioLaneSelection() {
  if (state.roads.length === 0) {
    return String(normalizeRoadLanes(state.defaultRoadLanes, 2));
  }
  const first = String(normalizeRoadLanes(state.roads[0]?.lanes, state.defaultRoadLanes));
  for (let i = 1; i < state.roads.length; i += 1) {
    if (String(normalizeRoadLanes(state.roads[i]?.lanes, state.defaultRoadLanes)) !== first) {
      return 'mixed';
    }
  }
  return first;
}

function updateScenarioLaneControl() {
  if (!ui.scenarioLanes) return;
  ui.scenarioLanes.value = getScenarioLaneSelection();
}

function loadScenarioPreset(preset) {
  const nextPreset = normalizeScenarioPreset(preset);
  if (nextPreset === 'sample-grid') {
    clearMap();
    buildSampleGrid();
    return;
  }
  buildScenario(nextPreset);
}

function setIntersectionSignalMode(node, nextMode) {
  if (!node?.signal) return;

  node.signal.mode = normalizeSignalMode(nextMode, 'none');
  ensureSignalWaitAges(node.signal);
  ensureSignalCycleState(node.signal);
  node.signal.elapsed = 0;
  node.signal.color = 'green';
  node.signal.pendingPhase = null;

  if (node.signal.mode === 'none') {
    for (const approach of APPROACH_ORDER) {
      node.signal.waitAges[approach] = 0;
    }
    return;
  }

  const available = getIntersectionApproaches(node.id);
  if (available.length > 0 && !available.includes(node.signal.phase)) {
    node.signal.phase = available[0];
  }
}

function getGlobalSignalModeSelection() {
  if (state.intersections.length === 0) return 'adaptive';
  const firstMode = normalizeSignalMode(state.intersections[0]?.signal?.mode, 'adaptive');
  for (let i = 1; i < state.intersections.length; i += 1) {
    const mode = normalizeSignalMode(state.intersections[i]?.signal?.mode, 'adaptive');
    if (mode !== firstMode) return 'mixed';
  }
  return firstMode;
}

function updateGlobalSignalModeControl() {
  if (!ui.globalSignalMode) return;
  ui.globalSignalMode.value = getGlobalSignalModeSelection();
}

function resetView() {
  state.view.x = 0;
  state.view.y = 0;
  state.view.zoom = 1;
}

function screenToWorld(x, y) {
  return {
    x: (x - state.view.x) / state.view.zoom,
    y: (y - state.view.y) / state.view.zoom,
  };
}

function distance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function normalize(vec) {
  const len = Math.hypot(vec.x, vec.y) || 1;
  return { x: vec.x / len, y: vec.y / len };
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y;
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function cubicBezierPoint(p0, p1, p2, p3, t) {
  const inv = 1 - t;
  const inv2 = inv * inv;
  const t2 = t * t;
  return {
    x: p0.x * inv2 * inv + 3 * p1.x * inv2 * t + 3 * p2.x * inv * t2 + p3.x * t2 * t,
    y: p0.y * inv2 * inv + 3 * p1.y * inv2 * t + 3 * p2.y * inv * t2 + p3.y * t2 * t,
  };
}

function cubicBezierTangent(p0, p1, p2, p3, t) {
  const inv = 1 - t;
  const a = 3 * inv * inv;
  const b = 6 * inv * t;
  const c = 3 * t * t;
  return {
    x: a * (p1.x - p0.x) + b * (p2.x - p1.x) + c * (p3.x - p2.x),
    y: a * (p1.y - p0.y) + b * (p2.y - p1.y) + c * (p3.y - p2.y),
  };
}

function applyIntersectionHoldSpeed(vehicle, dt) {
  if (!vehicle) return;
  const target = INTERSECTION_HOLD_SPEED;
  if (vehicle.speed <= target) return;
  const decelStep = DECELERATION * Math.max(dt, FIXED_DT);
  vehicle.speed = Math.max(target, vehicle.speed - decelStep);
}

function getVehicleStopDistance(vehicle, segment = null) {
  return Math.max(
    STOP_BUFFER,
    getIntersectionControlRadius(segment) + getVehicleLength(vehicle) * 0.5 + 4
  );
}

function getVehicleReserveDistance(vehicle, segment = null) {
  return Math.max(INTERSECTION_RESERVE_DISTANCE, getVehicleStopDistance(vehicle, segment) + 4);
}

function getVehicleCommitDistance(vehicle, segment = null) {
  return Math.max(
    STOP_BUFFER - 4,
    getIntersectionControlRadius(segment) + getVehicleLength(vehicle) * 0.5
  );
}

function getStopLineProgress(segment, vehicle = null) {
  if (!segment) return PRE_INTERSECTION_PROGRESS;
  return clamp(1 - getVehicleStopDistance(vehicle, segment) / Math.max(1, segment.length), 0, 0.985);
}

function getCommitProgress(segment, vehicle = null) {
  if (!segment) return PRE_INTERSECTION_PROGRESS;
  return clamp(1 - getVehicleCommitDistance(vehicle, segment) / Math.max(1, segment.length), 0, 0.99);
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

function getIntersectionControlRadius(segment = null) {
  const lanes = segment?.road?.lanes ?? 1;
  return Math.max(INTERSECTION_RADIUS, getRoadDrawWidth(lanes) * 0.5);
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

function ensureSignalCycleState(signal) {
  if (!signal) return;
  if (signal.color !== 'green' && signal.color !== 'yellow' && signal.color !== 'red') {
    signal.color = 'green';
  }
  if (typeof signal.pendingPhase !== 'string') {
    signal.pendingPhase = null;
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
      color: 'green',
      pendingPhase: null,
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
    lanes: normalizeRoadLanes(state.defaultRoadLanes, 2), // 1–8 = lanes per direction
  };
}

function makeVehicle(route, profile = null) {
  const variant = resolveVehicleProfile(profile);
  const laneDiscipline = createVehicleLaneDiscipline();
  return {
    id: `V${state.nextVehicleId++}`,
    route,
    segmentIndex: 0,
    progress: 0,
    speed: 0,
    laneIndex: 0,
    type: variant.id,
    label: variant.label,
    length: variant.length,
    width: variant.width,
    safeGap: variant.safeGap,
    speedFactor: variant.speedFactor,
    accelerationFactor: variant.accelerationFactor,
    brakingFactor: variant.brakingFactor,
    color: variant.color,
    laneStyle: laneDiscipline.style,
    laneDriftOffsetPx: laneDiscipline.driftOffsetPx,
    brakeLightOn: false,
    intersectionCarrySpeed: 0,
    intersectionCarryUntilProgress: 0,
    plannedNextLaneIndex: null,
    stuckAtIntersectionTicks: 0,
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
  state.overlapCheckCounter = 0;
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
  state.overlapCheckCounter = 0;
  state.vehiclesCompleted = 0;
  state.phaseChangeCount = 0;
  state.currentScenarioId = null;
  resetView();
  updateScenarioPresetControl();
  updateScenarioLaneControl();
  renderWeightTable();
  renderInspector();
}

function getCanvasPointerPosition(event) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
}

function getPointerPosition(event) {
  const screen = getCanvasPointerPosition(event);
  return screenToWorld(screen.x, screen.y);
}

function getVisibleWorldBounds() {
  const topLeft = screenToWorld(0, 0);
  const bottomRight = screenToWorld(canvas.width, canvas.height);
  return {
    minX: Math.min(topLeft.x, bottomRight.x),
    maxX: Math.max(topLeft.x, bottomRight.x),
    minY: Math.min(topLeft.y, bottomRight.y),
    maxY: Math.max(topLeft.y, bottomRight.y),
  };
}

function getMapBounds() {
  if (state.intersections.length === 0) {
    return null;
  }

  const xs = state.intersections.map((node) => node.x);
  const ys = state.intersections.map((node) => node.y);
  return {
    minX: Math.min(...xs),
    maxX: Math.max(...xs),
    minY: Math.min(...ys),
    maxY: Math.max(...ys),
  };
}

function focusViewOnMap(padding = 110) {
  const bounds = getMapBounds();
  if (!bounds) {
    resetView();
    return;
  }

  const contentWidth = Math.max(140, bounds.maxX - bounds.minX + INTERSECTION_RADIUS * 4);
  const contentHeight = Math.max(140, bounds.maxY - bounds.minY + INTERSECTION_RADIUS * 4);
  const availableWidth = Math.max(220, canvas.width - padding * 2);
  const availableHeight = Math.max(220, canvas.height - padding * 2);
  const nextZoom = clamp(
    Math.min(1.25, availableWidth / contentWidth, availableHeight / contentHeight),
    MIN_VIEW_ZOOM,
    MAX_VIEW_ZOOM
  );
  const centerX = (bounds.minX + bounds.maxX) * 0.5;
  const centerY = (bounds.minY + bounds.maxY) * 0.5;

  state.view.zoom = nextZoom;
  state.view.x = canvas.width * 0.5 - centerX * nextZoom;
  state.view.y = canvas.height * 0.52 - centerY * nextZoom;
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

function setAllRoadLanes(lanes) {
  const nextLanes = normalizeRoadLanes(lanes, 2);
  state.defaultRoadLanes = nextLanes;
  for (const road of state.roads) {
    const oldLanes = road.lanes ?? nextLanes;
    road.lanes = nextLanes;
    if (nextLanes > 1 && nextLanes !== oldLanes) {
      redistributeVehiclesOnRoad(road);
    }
  }
  updateScenarioLaneControl();
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
  updateGlobalSignalModeControl();
  updateScenarioPresetControl();
  updateScenarioLaneControl();
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
  ensureSignalCycleState(intersection.signal);

  let resolved = resolveApproach(intersection, nextPhase, queue);
  if (!resolved) {
    const currentIndex = available.indexOf(intersection.signal.phase);
    if (currentIndex === -1) {
      resolved = available[0];
    } else {
      resolved = available[(currentIndex + 1) % available.length];
    }
  }

  if (!resolved) return;

  if (intersection.signal.color === 'yellow') {
    intersection.signal.pendingPhase = resolved;
    return;
  }

  if (resolved === intersection.signal.phase) {
    return;
  }

  intersection.signal.pendingPhase = resolved;
  intersection.signal.color = 'yellow';
  intersection.signal.elapsed = 0;
}

function handleCanvasPointerDown(event) {
  const screen = getCanvasPointerPosition(event);
  const pointer = getPointerPosition(event);
  const hitIntersection = findIntersectionAt(pointer.x, pointer.y);
  const hitRoad = hitIntersection ? null : findRoadAt(pointer.x, pointer.y);
  const isPanButton = event.button === 1 || event.button === 2;

  if (isPanButton || (state.mode === 'select' && event.button === 0 && !hitIntersection && !hitRoad)) {
    state.dragPointerMode = 'pan';
    state.dragPanLastScreen = screen;
    state.dragPanMoved = false;
    state.dragPanAllowsDeselect = !isPanButton && state.mode === 'select';
    return;
  }

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
      state.dragPointerMode = 'intersection';
      state.dragIntersectionId = hitIntersection.id;
      selectObject('intersection', hitIntersection.id);
      return;
    }

    if (hitRoad) {
      selectObject('road', hitRoad.id);
      return;
    }
  }
}

function handleCanvasPointerMove(event) {
  if (state.dragPointerMode === 'pan' && state.dragPanLastScreen) {
    const screen = getCanvasPointerPosition(event);
    const dx = screen.x - state.dragPanLastScreen.x;
    const dy = screen.y - state.dragPanLastScreen.y;
    if (Math.abs(dx) > 0.01 || Math.abs(dy) > 0.01) {
      state.dragPanMoved = true;
    }
    state.view.x += dx;
    state.view.y += dy;
    state.dragPanLastScreen = screen;
    return;
  }

  if (state.dragPointerMode !== 'intersection' || !state.dragIntersectionId) return;
  const pointer = getPointerPosition(event);
  const node = getIntersectionById(state.dragIntersectionId);
  if (!node) return;

  node.x = pointer.x;
  node.y = pointer.y;
}

function handleCanvasPointerUp() {
  if (state.dragPointerMode === 'pan' && !state.dragPanMoved && state.dragPanAllowsDeselect) {
    state.selectedType = null;
    state.selectedId = null;
    renderInspector();
  }
  state.dragPointerMode = null;
  state.dragIntersectionId = null;
  state.dragPanLastScreen = null;
  state.dragPanMoved = false;
  state.dragPanAllowsDeselect = false;
}

function handleCanvasWheel(event) {
  event.preventDefault();
  const screen = getCanvasPointerPosition(event);
  const worldBefore = screenToWorld(screen.x, screen.y);
  const zoomFactor = Math.exp(-event.deltaY * ZOOM_SENSITIVITY);
  const nextZoom = clamp(state.view.zoom * zoomFactor, MIN_VIEW_ZOOM, MAX_VIEW_ZOOM);
  if (Math.abs(nextZoom - state.view.zoom) < 0.0001) {
    return;
  }

  state.view.zoom = nextZoom;
  state.view.x = screen.x - worldBefore.x * nextZoom;
  state.view.y = screen.y - worldBefore.y * nextZoom;
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
  const turns = new Map();
  const headingPenalty = new Map();
  const prev = new Map(); // prev[stateKey] = { nodeId, prevNodeId }
  const unvisited = new Set();
  const startState = stateKey(startId, previousNodeId);
  dist.set(startState, 0);
  turns.set(startState, 0);
  headingPenalty.set(startState, 0);
  prev.set(startState, null);
  unvisited.add(startState);

  function getCostTuple(key) {
    return {
      distance: dist.get(key) ?? Number.POSITIVE_INFINITY,
      turns: turns.get(key) ?? Number.POSITIVE_INFINITY,
      heading: headingPenalty.get(key) ?? Number.POSITIVE_INFINITY,
    };
  }

  function isBetterCost(nextCost, existingCost) {
    if (nextCost.distance < existingCost.distance - 1e-6) return true;
    if (nextCost.distance > existingCost.distance + 1e-6) return false;
    if (nextCost.turns < existingCost.turns) return true;
    if (nextCost.turns > existingCost.turns) return false;
    return nextCost.heading < existingCost.heading - 1e-6;
  }

  while (unvisited.size > 0) {
    let bestState = null;
    let bestCost = {
      distance: Number.POSITIVE_INFINITY,
      turns: Number.POSITIVE_INFINITY,
      heading: Number.POSITIVE_INFINITY,
    };

    for (const key of unvisited) {
      const value = getCostTuple(key);
      if (isBetterCost(value, bestCost)) {
        bestCost = value;
        bestState = key;
      }
    }

    if (!bestState || bestCost.distance === Number.POSITIVE_INFINITY) {
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
      const nextCost = {
        distance: bestCost.distance + neighbor.cost,
        turns: bestCost.turns + (isStraightPathTransition(prevNodeId, currentId, neighbor.id) ? 0 : 1),
        heading:
          bestCost.heading + getPathHeadingPenalty(currentId, neighbor.id, endId),
      };
      if (isBetterCost(nextCost, getCostTuple(nextState))) {
        dist.set(nextState, nextCost.distance);
        turns.set(nextState, nextCost.turns);
        headingPenalty.set(nextState, nextCost.heading);
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

function buildSpawnCandidateVehicle(route, vehicleProfile = null) {
  const variant = resolveVehicleProfile(vehicleProfile);
  return {
    id: '__spawn__',
    route,
    segmentIndex: 0,
    progress: 0,
    speed: 0,
    laneIndex: 0,
    type: variant.id,
    label: variant.label,
    length: variant.length,
    width: variant.width,
    safeGap: variant.safeGap,
    speedFactor: variant.speedFactor,
    accelerationFactor: variant.accelerationFactor,
    brakingFactor: variant.brakingFactor,
    color: variant.color,
    laneStyle: 'normal',
    laneDriftOffsetPx: 0,
    brakeLightOn: false,
    intersectionCarrySpeed: 0,
    intersectionCarryUntilProgress: 0,
  };
}

function planSpawnPlacement(route, vehicleProfile = null) {
  if (!Array.isArray(route) || route.length < 2) return null;

  const candidate = buildSpawnCandidateVehicle(route, vehicleProfile);
  const firstSegment = getSegmentFromIds(route[0], route[1]);
  if (!firstSegment) return null;

  const startProgress = clamp(
    getVehicleReleaseDistance(candidate) / Math.max(1, firstSegment.length),
    0,
    0.8
  );
  const laneIndex = pickLaneWithSpace(
    firstSegment.startId,
    firstSegment.endId,
    candidate.id,
    startProgress,
    firstSegment.road,
    null,
    null,
    candidate,
    0
  );
  if (laneIndex < 0) {
    return null;
  }

  return {
    progress: startProgress,
    laneIndex,
  };
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

  const vehicleProfile = pickVehicleProfile();
  const spawnPlacement = planSpawnPlacement(route, vehicleProfile);
  if (!route || route.length < 2 || !spawnPlacement) {
    return false;
  }

  const vehicle = makeVehicle(route, vehicleProfile);
  vehicle.progress = spawnPlacement.progress;
  vehicle.laneIndex = spawnPlacement.laneIndex;
  state.vehicles.push(vehicle);
  return true;
}

function getSegmentFromIds(startId, endId) {
  if (!startId || !endId) return null;
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

function getVehicleSegment(vehicle) {
  const startId = vehicle.route[vehicle.segmentIndex];
  const endId = vehicle.route[vehicle.segmentIndex + 1];
  return getSegmentFromIds(startId, endId);
}

function getVehicleNextLanePlanContext(vehicle, segment = getVehicleSegment(vehicle)) {
  if (!vehicle || !segment) return null;
  const nextStartId = segment.endId;
  const nextEndId = vehicle.route[vehicle.segmentIndex + 2];
  if (!nextEndId) {
    return null;
  }

  const nextRoad = findRoadBetween(nextStartId, nextEndId);
  const nextStart = getIntersectionById(nextStartId);
  const nextEnd = getIntersectionById(nextEndId);
  if (!nextRoad || !nextStart || !nextEnd) {
    return null;
  }

  return {
    nextStartId,
    nextEndId,
    nextRoad,
    nextLength: Math.max(1, distance(nextStart, nextEnd)),
  };
}

function canUseVehicleNextLane(vehicle, laneIndex, segment = getVehicleSegment(vehicle), entryProgress = null) {
  const context = getVehicleNextLanePlanContext(vehicle, segment);
  if (!context || laneIndex == null || laneIndex < 0) return false;
  const minEntryProgress =
    getIntersectionReleaseDistance({ length: context.nextLength, road: context.nextRoad }, vehicle) /
    context.nextLength;
  const resolvedEntryProgress = clamp(
    Math.max(entryProgress ?? minEntryProgress, minEntryProgress),
    0,
    0.98
  );
  const checkLane = clamp(laneIndex, 0, Math.max(0, (context.nextRoad?.lanes ?? 1) - 1));
  const entryMetrics = getLaneEntryMetrics(
    context.nextStartId,
    context.nextEndId,
    vehicle.id,
    resolvedEntryProgress,
    checkLane,
    context.nextRoad,
    vehicle
  );
  return entryMetrics.clear && entryMetrics.frontGap >= entryMetrics.requiredFrontGap;
}

function pickVehicleNextLane(vehicle, segment = getVehicleSegment(vehicle), entryProgress = null) {
  const context = getVehicleNextLanePlanContext(vehicle, segment);
  if (!context) return -1;
  const minEntryProgress =
    getIntersectionReleaseDistance({ length: context.nextLength, road: context.nextRoad }, vehicle) /
    context.nextLength;
  const resolvedEntryProgress = clamp(
    Math.max(entryProgress ?? minEntryProgress, minEntryProgress),
    0,
    0.98
  );
  const nextLanes = context.nextRoad?.lanes ?? 1;
  const shiftLimit = getIntersectionLaneShiftLimit(vehicle, vehicle.segmentIndex + 1, nextLanes);
  const result = pickLaneWithSpace(
    context.nextStartId,
    context.nextEndId,
    vehicle.id,
    resolvedEntryProgress,
    context.nextRoad,
    vehicle.laneIndex ?? 0,
    shiftLimit,
    vehicle,
    vehicle.segmentIndex + 1
  );
  if (result >= 0 || nextLanes <= 1) return result;
  // All lanes within the normal shift limit are full — retry allowing any lane.
  return pickLaneWithSpace(
    context.nextStartId,
    context.nextEndId,
    vehicle.id,
    resolvedEntryProgress,
    context.nextRoad,
    vehicle.laneIndex ?? 0,
    Math.max(0, nextLanes - 1),
    vehicle,
    vehicle.segmentIndex + 1
  );
}

function resolveVehicleNextLane(vehicle, segment = getVehicleSegment(vehicle), entryProgress = null) {
  if (canUseVehicleNextLane(vehicle, vehicle?.plannedNextLaneIndex, segment, entryProgress)) {
    return vehicle.plannedNextLaneIndex;
  }
  return pickVehicleNextLane(vehicle, segment, entryProgress);
}

function refreshPlannedNextLanes(vehicles = state.vehicles) {
  for (const vehicle of vehicles) {
    const segment = getVehicleSegment(vehicle);
    const plannedLane = pickVehicleNextLane(vehicle, segment);
    vehicle.plannedNextLaneIndex = plannedLane >= 0 ? plannedLane : null;
  }
}

function getLanePoseOnSegment(segment, laneIndex, progress, vehicle = null) {
  const t = clamp(progress, 0, 1);
  const baseX = lerp(segment.start.x, segment.end.x, t);
  const baseY = lerp(segment.start.y, segment.end.y, t);
  const dir = normalize({
    x: segment.end.x - segment.start.x,
    y: segment.end.y - segment.start.y,
  });
  const perp = { x: -dir.y, y: dir.x };
  const lanes = segment.road?.lanes ?? 1;
  const lane = Math.min(Math.max(0, laneIndex || 0), Math.max(0, lanes - 1));
  const laneOffset =
    laneOffsetForDirection(segment.startId, segment.endId, lane, lanes) + getVehicleLaneDrift(vehicle);
  return {
    x: baseX + perp.x * laneOffset,
    y: baseY + perp.y * laneOffset,
    heading: Math.atan2(dir.y, dir.x),
    dir,
  };
}

function getVehicleWorldPose(vehicle) {
  const segment = getVehicleSegment(vehicle);
  if (!segment) return null;
  const currentLaneIndex = vehicle.laneIndex ?? 0;
  const linearPose = getLanePoseOnSegment(segment, currentLaneIndex, vehicle.progress, vehicle);

  const nextStartId = vehicle.route[vehicle.segmentIndex + 1];
  const nextEndId = vehicle.route[vehicle.segmentIndex + 2];
  if (!nextStartId || !nextEndId) {
    return { ...linearPose, segment };
  }

  const nextSegment = getSegmentFromIds(nextStartId, nextEndId);
  if (!nextSegment) {
    return { ...linearPose, segment };
  }

  const stopLineProgress = getStopLineProgress(segment, vehicle);
  const turnStartProgress = clamp(
    Math.max(
      1 - getIntersectionReleaseDistance(segment, vehicle) / Math.max(1, segment.length),
      stopLineProgress + 0.03
    ),
    TURN_BLEND_MIN_PROGRESS,
    0.97
  );
  const remainingDistance = segment.length * (1 - clamp(vehicle.progress, 0, 1));
  const closeToStopLine = vehicle.progress <= stopLineProgress + 0.008;
  const canBlendThroughIntersection =
    !closeToStopLine || canProceedThroughSignal(segment, remainingDistance, vehicle);
  if (vehicle.progress < turnStartProgress) {
    return { ...linearPose, segment };
  }
  if (!canBlendThroughIntersection) {
    return { ...linearPose, segment };
  }

  const blend = clamp((vehicle.progress - turnStartProgress) / (1 - turnStartProgress), 0, 1);
  const nextLaneIndex = vehicle.plannedNextLaneIndex ?? currentLaneIndex;
  const straightThrough =
    isStraightPathTransition(segment.startId, segment.endId, nextSegment.endId) &&
    currentLaneIndex === nextLaneIndex;
  const turnExitProgress = getIntersectionExitProgress(nextSegment, vehicle);
  const p0Pose = getLanePoseOnSegment(segment, currentLaneIndex, turnStartProgress, vehicle);
  const p3Pose = getLanePoseOnSegment(nextSegment, nextLaneIndex, turnExitProgress, vehicle);
  if (straightThrough) {
    return {
      x: lerp(p0Pose.x, p3Pose.x, blend),
      y: lerp(p0Pose.y, p3Pose.y, blend),
      heading: linearPose.heading,
      dir: linearPose.dir,
      segment,
    };
  }
  const chord = Math.hypot(p3Pose.x - p0Pose.x, p3Pose.y - p0Pose.y);
  const controlCap = Math.max(8, Math.min(TURN_CONTROL_MAX_DISTANCE, chord * 0.48));

  const inControlDist = clamp(
    segment.length * 0.18,
    TURN_CONTROL_MIN_DISTANCE,
    TURN_CONTROL_MAX_DISTANCE
  );
  const outControlDist = clamp(
    nextSegment.length * 0.18,
    TURN_CONTROL_MIN_DISTANCE,
    TURN_CONTROL_MAX_DISTANCE
  );
  const inControl = Math.min(inControlDist, controlCap);
  const outControl = Math.min(outControlDist, controlCap);
  const p1 = {
    x: p0Pose.x + p0Pose.dir.x * inControl,
    y: p0Pose.y + p0Pose.dir.y * inControl,
  };
  const p2 = {
    x: p3Pose.x - p3Pose.dir.x * outControl,
    y: p3Pose.y - p3Pose.dir.y * outControl,
  };

  const curvedPoint = cubicBezierPoint(p0Pose, p1, p2, p3Pose, blend);
  const tangent = cubicBezierTangent(p0Pose, p1, p2, p3Pose, blend);
  const tangentLen = Math.hypot(tangent.x, tangent.y);
  const heading =
    tangentLen > 0.001 ? Math.atan2(tangent.y, tangent.x) : p3Pose.heading;

  return {
    x: curvedPoint.x,
    y: curvedPoint.y,
    heading,
    segment,
  };
}

function rerouteVehicleFromNode(vehicle, currentNodeId, previousNodeId = null) {
  if (!vehicle || !currentNodeId) return false;
  const destinationId = vehicle.route[vehicle.route.length - 1];
  if (!destinationId) return false;
  if (currentNodeId === destinationId) {
    vehicle.route = [currentNodeId];
    vehicle.segmentIndex = 0;
    vehicle.progress = 0;
    vehicle.speed = 0;
    return true;
  }

  const newRoute = shortestPath(currentNodeId, destinationId, previousNodeId);
  if (!newRoute || newRoute.length < 2) {
    return false;
  }

  vehicle.route = newRoute;
  vehicle.segmentIndex = 0;
  vehicle.progress = 0;
  return true;
}

function canVehicleClearIntersection(vehicle, segment = getVehicleSegment(vehicle)) {
  if (!vehicle || !segment) return false;
  const context = getVehicleNextLanePlanContext(vehicle, segment);
  if (!context) {
    return true;
  }

  const minEntryProgress =
    getIntersectionReleaseDistance({ length: context.nextLength, road: context.nextRoad }, vehicle) /
    context.nextLength;
  let pickedLane = resolveVehicleNextLane(vehicle, segment, minEntryProgress);
  if (pickedLane < 0 && (vehicle.stuckAtIntersectionTicks ?? 0) >= STUCK_TICKS_RELAXED_ENTRY) {
    pickedLane = pickLaneRelaxed(
      context.nextStartId, context.nextEndId, vehicle.id,
      minEntryProgress, context.nextRoad, vehicle
    );
  }
  if (pickedLane < 0) {
    return false;
  }
  const entryMetrics = getLaneEntryMetrics(
    context.nextStartId,
    context.nextEndId,
    vehicle.id,
    minEntryProgress,
    pickedLane,
    context.nextRoad,
    vehicle
  );
  const requiredGap = (vehicle.stuckAtIntersectionTicks ?? 0) >= STUCK_TICKS_RELAXED_ENTRY
    ? getVehicleBodyClearance(vehicle, vehicle) + 2
    : getIntersectionReleaseDistance({ length: context.nextLength }, vehicle);
  return entryMetrics.frontGap >= requiredGap;
}

function buildIntersectionReservations(dt = FIXED_DT) {
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
    const reserveLookahead =
      Math.max(vehicle.speed, getVehicleTargetRoadSpeed(vehicle, segment.road)) * Math.max(dt, FIXED_DT) + 1;
    const physicallyCommitted = remainingDistance <= getVehicleCommitDistance(vehicle, segment);
    // Only physically committed (past stop line) or able-to-clear vehicles get reservation.
    // Vehicles at the stop line that cannot find a lane must NOT reserve — they would block others.
    const canClear = physicallyCommitted || canVehicleClearIntersection(vehicle, segment);
    const stuckTicks = vehicle.stuckAtIntersectionTicks ?? 0;
    const waitingLongEnough = stuckTicks >= STUCK_TICKS_RELAXED_ENTRY;
    const nextEndId = vehicle.route[vehicle.segmentIndex + 2];
    const nextRoad = nextEndId ? findRoadBetween(segment.endId, nextEndId) : null;
    const nextStart = getIntersectionById(segment.endId);
    const nextEnd = getIntersectionById(nextEndId);
    const nextLength = nextStart && nextEnd ? Math.max(1, distance(nextStart, nextEnd)) : 1;
    const minNextEntryProgress =
      nextRoad
        ? getIntersectionReleaseDistance({ length: nextLength, road: nextRoad }, vehicle) / nextLength
        : 0;
    const relaxedLaneOk =
      waitingLongEnough &&
      nextEndId &&
      nextRoad &&
      pickLaneRelaxed(
        segment.endId,
        nextEndId,
        vehicle.id,
        minNextEntryProgress,
        nextRoad,
        vehicle
      ) >= 0;
    const allowReservation =
      remainingDistance <= getVehicleReserveDistance(vehicle, segment) + reserveLookahead &&
      isLaneGreen(segment, remainingDistance, vehicle) &&
      (canClear || relaxedLaneOk);
    if (allowReservation) {
      const distForReservation = canClear
        ? remainingDistance
        : remainingDistance - (waitingLongEnough ? 200 : 0);
      reserveClosest(segment.endId, vehicle.id, Math.max(1, distForReservation));
    }

    const distanceFromStart = segment.length * vehicle.progress;
    const exitReserveDistance = getIntersectionReleaseDistance(segment, vehicle);
    const stuckAtStartOfSegment =
      vehicle.speed < 2 && vehicle.progress < 0.2;
    if (distanceFromStart <= exitReserveDistance && !stuckAtStartOfSegment) {
      reserveClosest(segment.startId, vehicle.id, distanceFromStart);
    }
  }

  const ownerMap = new Map();
  for (const [intersectionId, data] of reservations.entries()) {
    ownerMap.set(intersectionId, data.vehicleId);
  }
  return ownerMap;
}

function isIntersectionClearForPhaseSwitch(intersectionId) {
  for (const vehicle of state.vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;

    if (segment.endId === intersectionId) {
      const remainingDistance = segment.length * (1 - vehicle.progress);
      if (remainingDistance <= getVehicleCommitDistance(vehicle, segment) + 1) {
        return false;
      }
    }

    if (segment.startId === intersectionId) {
      const distanceFromStart = segment.length * vehicle.progress;
      if (distanceFromStart <= getIntersectionReleaseDistance(segment, vehicle) + 1) {
        return false;
      }
    }
  }
  return true;
}

function ignoresReservationForReverseLane(segment, reservationOwnerId) {
  if (!segment || !reservationOwnerId) return false;
  const owner = state.vehicles.find((vehicle) => vehicle.id === reservationOwnerId);
  if (!owner) return false;
  const ownerSegment = getVehicleSegment(owner);
  if (!ownerSegment) return false;

  const justExitedSameIntersection =
    ownerSegment.startId === segment.endId &&
    ownerSegment.endId === segment.startId &&
    owner.progress * ownerSegment.length <= getVehicleReleaseDistance(owner);

  return justExitedSameIntersection;
}

function getLaneEntryMetrics(
  startId,
  endId,
  vehicleId,
  entryProgress,
  laneIndex = 0,
  road = null,
  candidateVehicle = null
) {
  const resolvedRoad = road || findRoadBetween(startId, endId);
  const lanes = resolvedRoad?.lanes ?? 1;
  const checkLane = Math.min(laneIndex ?? 0, Math.max(0, lanes - 1));
  const start = getIntersectionById(startId);
  const end = getIntersectionById(endId);
  const segmentLength = start && end ? Math.max(1, distance(start, end)) : 1;
  const candidate = candidateVehicle || {
    length: DEFAULT_VEHICLE_LENGTH,
    width: DEFAULT_VEHICLE_WIDTH,
    safeGap: DEFAULT_SAFE_GAP,
  };
  let clear = true;
  let laneLoad = 0;
  let vehiclesAhead = 0;
  let frontGap = segmentLength * (1 - clamp(entryProgress, 0, 1));
  const requiredFrontGap = getIntersectionReleaseDistance({ length: segmentLength }, candidate);

  for (const other of state.vehicles) {
    if (other.id === vehicleId) continue;
    const otherSegment = getVehicleSegment(other);
    if (!otherSegment) continue;
    if (otherSegment.startId !== startId || otherSegment.endId !== endId) continue;
    if (lanes > 1) {
      const otherLane = Math.min(other.laneIndex ?? 0, Math.max(0, lanes - 1));
      if (otherLane !== checkLane) continue;
    }
    laneLoad += 1;
    const centerDelta = (other.progress - entryProgress) * segmentLength;
    const centerDistance = Math.abs(centerDelta);
    if (centerDistance < getVehicleSpacingDistance(other, candidate)) {
      clear = false;
    }
    if (centerDelta >= 0) {
      vehiclesAhead += 1;
      const bodyGap = Math.max(0, centerDelta - getVehicleBodyClearance(other, candidate));
      frontGap = Math.min(frontGap, bodyGap);
    }
  }

  return {
    clear,
    laneLoad,
    vehiclesAhead,
    frontGap,
    requiredFrontGap,
  };
}

function hasLaneEntrySpace(
  startId,
  endId,
  vehicleId,
  entryProgress,
  laneIndex = 0,
  road = null,
  candidateVehicle = null
) {
  const metrics = getLaneEntryMetrics(
    startId,
    endId,
    vehicleId,
    entryProgress,
    laneIndex,
    road,
    candidateVehicle
  );
  return metrics.clear && metrics.frontGap >= metrics.requiredFrontGap;
}

function pickLaneWithSpace(
  startId,
  endId,
  vehicleId,
  entryProgress,
  road,
  preferredLaneIndex = null,
  maxLaneShift = null,
  candidateVehicle = null,
  routeSegmentIndex = null
) {
  const lanes = road?.lanes ?? 1;
  if (lanes <= 1) {
    return hasLaneEntrySpace(startId, endId, vehicleId, entryProgress, 0, road, candidateVehicle)
      ? 0
      : -1;
  }

  const preferred =
    preferredLaneIndex == null
      ? null
      : clamp(preferredLaneIndex, 0, Math.max(0, lanes - 1));
  const routeLaneIndex =
    candidateVehicle == null ? null : getVehicleTurnLanePreference(candidateVehicle, routeSegmentIndex, lanes);
  if (routeLaneIndex != null) {
    if (preferred != null && maxLaneShift != null && Math.abs(routeLaneIndex - preferred) > maxLaneShift) {
      return -1;
    }
    if (
      hasLaneEntrySpace(
        startId,
        endId,
        vehicleId,
        entryProgress,
        routeLaneIndex,
        road,
        candidateVehicle
      )
    ) {
      return routeLaneIndex;
    }
    // Preferred turn lane is full — fall through to try adjacent lanes
    // so the vehicle doesn't freeze the entire lane behind it.
  }
  let bestLane = -1;
  let bestScore = Number.NEGATIVE_INFINITY;

  for (let i = 0; i < lanes; i += 1) {
    if (preferred != null && maxLaneShift != null && Math.abs(i - preferred) > maxLaneShift) {
      continue;
    }
    if (routeLaneIndex != null && maxLaneShift != null && Math.abs(i - routeLaneIndex) > maxLaneShift) {
      continue;
    }
    const metrics = getLaneEntryMetrics(
      startId,
      endId,
      vehicleId,
      entryProgress,
      i,
      road,
      candidateVehicle
    );
    if (!metrics.clear || metrics.frontGap < metrics.requiredFrontGap) {
      continue;
    }
    const lanePreferenceScore = getLanePreferenceScore(
      i,
      lanes,
      preferred,
      routeLaneIndex,
      candidateVehicle
    );
    const score = metrics.frontGap * 1.8 - metrics.vehiclesAhead * 22 - metrics.laneLoad * 10 + lanePreferenceScore;
    if (
      score > bestScore ||
      (score === bestScore &&
        routeLaneIndex != null &&
        bestLane >= 0 &&
        Math.abs(i - routeLaneIndex) < Math.abs(bestLane - routeLaneIndex)) ||
      (score === bestScore &&
        routeLaneIndex == null &&
        preferred != null &&
        bestLane >= 0 &&
        Math.abs(i - preferred) < Math.abs(bestLane - preferred))
    ) {
      bestScore = score;
      bestLane = i;
    }
  }
  return bestLane;
}

function pickLaneRelaxed(startId, endId, vehicleId, entryProgress, road, candidateVehicle) {
  const lanes = road?.lanes ?? 1;
  const start = getIntersectionById(startId);
  const end = getIntersectionById(endId);
  const segmentLength = start && end ? Math.max(1, distance(start, end)) : 1;
  let bestLane = -1;
  let bestGap = -1;

  for (let i = 0; i < lanes; i += 1) {
    let minGap = segmentLength;
    let blocked = false;
    for (const other of state.vehicles) {
      if (other.id === vehicleId) continue;
      const otherSegment = getVehicleSegment(other);
      if (!otherSegment) continue;
      if (otherSegment.startId !== startId || otherSegment.endId !== endId) continue;
      if (lanes > 1) {
        const otherLane = Math.min(other.laneIndex ?? 0, Math.max(0, lanes - 1));
        if (otherLane !== i) continue;
      }
      const centerDelta = (other.progress - entryProgress) * segmentLength;
      const bodyDist = Math.abs(centerDelta) - getVehicleBodyClearance(other, candidateVehicle);
      if (bodyDist < 1) {
        blocked = true;
        break;
      }
      if (centerDelta >= 0) {
        minGap = Math.min(minGap, bodyDist);
      }
    }
    if (!blocked && minGap > bestGap) {
      bestGap = minGap;
      bestLane = i;
    }
  }
  return bestLane;
}

function enforceLaneSeparation(vehicles, dt = FIXED_DT) {
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
        getVehicleSpacingDistance(lead.vehicle, follower.vehicle) /
        Math.max(1, follower.segment.length);
      const maxFollowerProgress = lead.vehicle.progress - minProgressGap;

      if (follower.vehicle.progress > maxFollowerProgress) {
        follower.vehicle.progress = Math.max(0, maxFollowerProgress);
        if (follower.vehicle.speed > lead.vehicle.speed) {
          const decelStep = DECELERATION * Math.max(dt, FIXED_DT);
          follower.vehicle.speed = Math.max(
            lead.vehicle.speed,
            follower.vehicle.speed - decelStep
          );
        }
      }
    }
  }
}

function sharedIntersectionClearance(segmentA, progressA, vehicleA, segmentB, progressB, vehicleB) {
  if (!segmentA || !segmentB) return false;

  const sharedIds = [
    segmentA.startId === segmentB.startId ? segmentA.startId : null,
    segmentA.startId === segmentB.endId ? segmentA.startId : null,
    segmentA.endId === segmentB.startId ? segmentA.endId : null,
    segmentA.endId === segmentB.endId ? segmentA.endId : null,
  ].filter(Boolean);
  const uniqueSharedIds = [...new Set(sharedIds)];

  if (uniqueSharedIds.length === 0) return false;
  const protectedRadius =
    INTERSECTION_RESERVE_DISTANCE + Math.max(getVehicleReleaseDistance(vehicleA), getVehicleReleaseDistance(vehicleB)) + 2;
  const distanceFromShared = (segment, progress, sharedId) => {
    if (segment.startId === sharedId) return segment.length * progress;
    if (segment.endId === sharedId) return segment.length * (1 - progress);
    return Number.POSITIVE_INFINITY;
  };

  return uniqueSharedIds.some((sharedId) => {
    const aDistance = distanceFromShared(segmentA, progressA, sharedId);
    const bDistance = distanceFromShared(segmentB, progressB, sharedId);
    return aDistance <= protectedRadius && bDistance <= protectedRadius;
  });
}

function rotateLocalPoint(point, pose) {
  const cos = Math.cos(pose?.heading ?? 0);
  const sin = Math.sin(pose?.heading ?? 0);
  return {
    x: pose.x + point.x * cos - point.y * sin,
    y: pose.y + point.x * sin + point.y * cos,
  };
}

function getVehicleCollisionPolygon(vehicle, pose) {
  const length = getVehicleLength(vehicle);
  const width = getVehicleWidth(vehicle);
  const localPoints = [
    { x: length * 0.5, y: 0 },
    { x: -length * 0.45, y: width * 0.5 },
    { x: -length * 0.45, y: -width * 0.5 },
  ];
  return localPoints.map((point) => rotateLocalPoint(point, pose));
}

function getPolygonAxes(points) {
  const axes = [];
  for (let i = 0; i < points.length; i += 1) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    const edge = { x: b.x - a.x, y: b.y - a.y };
    axes.push(normalize({ x: -edge.y, y: edge.x }));
  }
  return axes;
}

function getPolygonBounds(points) {
  let minX = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const point of points) {
    minX = Math.min(minX, point.x);
    maxX = Math.max(maxX, point.x);
    minY = Math.min(minY, point.y);
    maxY = Math.max(maxY, point.y);
  }
  return { minX, maxX, minY, maxY };
}

function doBoundsOverlap(boundsA, boundsB) {
  return !(
    boundsA.maxX < boundsB.minX ||
    boundsB.maxX < boundsA.minX ||
    boundsA.maxY < boundsB.minY ||
    boundsB.maxY < boundsA.minY
  );
}

function projectPolygon(points, axis) {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (const point of points) {
    const value = dot(point, axis);
    min = Math.min(min, value);
    max = Math.max(max, value);
  }
  return { min, max };
}

function doVehicleBodiesOverlap(poseA, poseB) {
  if (!poseA || !poseB) return false;
  const polyA = getVehicleCollisionPolygon(poseA.vehicle, poseA);
  const polyB = getVehicleCollisionPolygon(poseB.vehicle, poseB);
  const axes = [...getPolygonAxes(polyA), ...getPolygonAxes(polyB)];
  for (const axis of axes) {
    const a = projectPolygon(polyA, axis);
    const b = projectPolygon(polyB, axis);
    if (a.max <= b.min || b.max <= a.min) {
      return false;
    }
  }
  return true;
}

function isSerializedTurnHandoff(poseA, poseB) {
  if (!poseA?.segment || !poseB?.segment) return false;

  function matches(approachPose, exitPose) {
    if (approachPose.segment.endId !== exitPose.segment.startId) {
      return false;
    }
    const remainingToShared = approachPose.segment.length * (1 - approachPose.progress);
    const exitDistance = exitPose.segment.length * exitPose.progress;
    const releaseEnvelope = Math.max(
      getVehicleReleaseDistance(exitPose.vehicle) + INTERSECTION_RADIUS,
      exitPose.segment.length * 0.38
    );
    return (
      remainingToShared <= getVehicleReleaseDistance(approachPose.vehicle) + 2 &&
      exitDistance <= releaseEnvelope
    );
  }

  return matches(poseA, poseB) || matches(poseB, poseA);
}

function isSeparatedIntersectionApproachExitPair(poseA, poseB) {
  if (!poseA?.segment || !poseB?.segment) return false;

  function matches(approachPose, exitPose) {
    if (approachPose.segment.endId !== exitPose.segment.startId) {
      return false;
    }
    const stopLineProgress = getStopLineProgress(approachPose.segment, approachPose.vehicle);
    const exitClearProgress = clamp(
      getIntersectionExitProgress(exitPose.segment, exitPose.vehicle) + 0.05,
      0,
      0.35
    );
    return approachPose.progress <= stopLineProgress + 0.03 && exitPose.progress <= exitClearProgress;
  }

  return matches(poseA, poseB) || matches(poseB, poseA);
}

function isSeparatedParallelLanePair(poseA, poseB) {
  if (!poseA?.segment || !poseB?.segment) return false;
  return (
    poseA.segment.startId === poseB.segment.startId &&
    poseA.segment.endId === poseB.segment.endId &&
    (poseA.vehicle.laneIndex ?? 0) !== (poseB.vehicle.laneIndex ?? 0)
  );
}

function isSeparatedOpposingApproachPair(poseA, poseB) {
  if (!poseA?.segment || !poseB?.segment) return false;
  if (poseA.segment.endId !== poseB.segment.endId) return false;
  if (segmentOrientation(poseA.segment.start, poseA.segment.end) !== segmentOrientation(poseB.segment.start, poseB.segment.end)) {
    return false;
  }
  const dirA = normalize({
    x: poseA.segment.end.x - poseA.segment.start.x,
    y: poseA.segment.end.y - poseA.segment.start.y,
  });
  const dirB = normalize({
    x: poseB.segment.end.x - poseB.segment.start.x,
    y: poseB.segment.end.y - poseB.segment.start.y,
  });
  if (dot(dirA, dirB) > -0.92) {
    return false;
  }
  const remainingA = poseA.segment.length * (1 - poseA.progress);
  const remainingB = poseB.segment.length * (1 - poseB.progress);
  return (
    remainingA <= getVehicleStopDistance(poseA.vehicle, poseA.segment) + 4 &&
    remainingB <= getVehicleStopDistance(poseB.vehicle, poseB.segment) + 4
  );
}

function isQueuedCrossApproachPair(poseA, poseB) {
  if (!poseA?.segment || !poseB?.segment) return false;
  if (poseA.segment.endId !== poseB.segment.endId) return false;
  if (segmentOrientation(poseA.segment.start, poseA.segment.end) === segmentOrientation(poseB.segment.start, poseB.segment.end)) {
    return false;
  }
  const nearStopA = poseA.progress <= getStopLineProgress(poseA.segment, poseA.vehicle) + 0.05;
  const nearStopB = poseB.progress <= getStopLineProgress(poseB.segment, poseB.vehicle) + 0.05;
  if (!nearStopA && !nearStopB) {
    return false;
  }
  const committedA = poseA.progress >= getCommitProgress(poseA.segment, poseA.vehicle);
  const committedB = poseB.progress >= getCommitProgress(poseB.segment, poseB.vehicle);
  return !(committedA && committedB);
}

function isQueuedIntersectionTailPair(poseA, poseB) {
  if (!poseA?.segment || !poseB?.segment) return false;

  function matches(leadPose, tailPose) {
    if (
      leadPose.segment.startId !== tailPose.segment.startId ||
      leadPose.segment.endId !== tailPose.segment.endId
    ) {
      return false;
    }
    if ((leadPose.vehicle.laneIndex ?? 0) !== (tailPose.vehicle.laneIndex ?? 0)) {
      return false;
    }
    if (leadPose.progress <= tailPose.progress) {
      return false;
    }
    const stopLineProgress = getStopLineProgress(tailPose.segment, tailPose.vehicle);
    const centerGap = (leadPose.progress - tailPose.progress) * tailPose.segment.length;
    return (
      leadPose.progress >= PRE_INTERSECTION_PROGRESS - 0.01 &&
      tailPose.progress >= stopLineProgress + 0.03 &&
      centerGap >= getVehicleBodyClearance(leadPose.vehicle, tailPose.vehicle) + 4
    );
  }

  return matches(poseA, poseB) || matches(poseB, poseA);
}

function collectVehicleOverlaps(limit = Number.POSITIVE_INFINITY) {
  const poses = [];
  for (const vehicle of state.vehicles) {
    const pose = getVehicleWorldPose(vehicle);
    if (pose) {
      poses.push({
        ...pose,
        progress: vehicle.progress,
        vehicle,
      });
    }
  }

  const overlaps = [];
  for (let i = 0; i < poses.length; i += 1) {
    for (let j = i + 1; j < poses.length; j += 1) {
      if (
        doVehicleBodiesOverlap(poses[i], poses[j]) &&
        !isSerializedTurnHandoff(poses[i], poses[j]) &&
        !isSeparatedIntersectionApproachExitPair(poses[i], poses[j]) &&
        !isSeparatedParallelLanePair(poses[i], poses[j]) &&
        !isSeparatedOpposingApproachPair(poses[i], poses[j]) &&
        !isQueuedCrossApproachPair(poses[i], poses[j]) &&
        !isQueuedIntersectionTailPair(poses[i], poses[j])
      ) {
        overlaps.push({
          a: {
            id: poses[i].vehicle.id,
            lane_from: poses[i].segment.startId,
            lane_to: poses[i].segment.endId,
            lane_index: poses[i].vehicle.laneIndex ?? 0,
            progress: round(poses[i].progress, 3),
          },
          b: {
            id: poses[j].vehicle.id,
            lane_from: poses[j].segment.startId,
            lane_to: poses[j].segment.endId,
            lane_index: poses[j].vehicle.laneIndex ?? 0,
            progress: round(poses[j].progress, 3),
          },
        });
        if (overlaps.length >= limit) {
          return overlaps;
        }
      }
    }
  }

  return overlaps;
}

function countVehicleOverlaps() {
  return collectVehicleOverlaps().length;
}

function getOverlapCheckInterval(vehicleCount) {
  if (vehicleCount >= 160) return 12;
  if (vehicleCount >= 120) return 8;
  if (vehicleCount >= 80) return 6;
  if (vehicleCount >= 40) return 3;
  return 1;
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
      const gap =
        (front.progress - follower.progress) * follower.length -
        getVehicleBodyClearance(front.vehicle, follower.vehicle);
      gapMap.set(follower.vehicle.id, Math.max(0, gap));
    }
  }

  return gapMap;
}

function getApproachSignalColor(intersection, approach) {
  if (!intersection || !approach) return 'green';
  if (intersection.signal.mode === 'none') return 'green';
  ensureSignalCycleState(intersection.signal);
  if (intersection.signal.color === 'red') return 'red';
  if (intersection.signal.phase !== approach) return 'red';
  return intersection.signal.color === 'yellow' ? 'yellow' : 'green';
}

function canProceedThroughSignal(segment, remainingDistance = Number.POSITIVE_INFINITY, vehicle = null) {
  const target = getIntersectionById(segment.endId);
  if (!target) return true;
  if (target.signal.mode === 'none') return true;
  // Once the vehicle body has reached the intersection entry, it is committed to clear it.
  if (remainingDistance <= getVehicleCommitDistance(vehicle, segment)) return true;
  const approach = getApproachKey(segment.start, segment.end);
  const color = getApproachSignalColor(target, approach);
  if (color === 'green') return true;
  if (color === 'yellow') {
    return remainingDistance <= YELLOW_CLEARANCE_DISTANCE;
  }
  return false;
}

function isLaneGreen(segment, remainingDistance = Number.POSITIVE_INFINITY, vehicle = null) {
  return canProceedThroughSignal(segment, remainingDistance, vehicle);
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
        const downstreamCapacity = Math.max(2, downstreamLength / AVERAGE_VEHICLE_SPACING);
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
      ensureSignalCycleState(signal);
      signal.color = 'green';
      signal.pendingPhase = null;
      for (const approach of APPROACH_ORDER) {
        signal.waitAges[approach] = 0;
      }
      continue;
    }

    if (signal.mode === 'none') {
      signal.elapsed = 0;
      ensureSignalWaitAges(signal);
      ensureSignalCycleState(signal);
      signal.color = 'green';
      signal.pendingPhase = null;
      for (const approach of APPROACH_ORDER) {
        signal.waitAges[approach] = 0;
      }
      continue;
    }

    ensureSignalWaitAges(signal);
    ensureSignalCycleState(signal);
    if (!availableApproaches.includes(signal.phase)) {
      signal.phase = availableApproaches[0];
      signal.elapsed = 0;
      signal.color = 'green';
      signal.pendingPhase = null;
    }
    if (signal.pendingPhase && !availableApproaches.includes(signal.pendingPhase)) {
      signal.pendingPhase = null;
    }

    signal.elapsed += dt;
    const queue = queueMetrics.get(node.id) || makeEmptyQueue();
    const flow = flowMetrics.get(node.id) || makeEmptyQueue();
    updateWaitAges(signal, queue, flow, availableApproaches, dt);

    if (signal.color === 'yellow') {
      if (signal.elapsed >= SIGNAL_YELLOW_DURATION) {
        signal.color = 'red';
        signal.elapsed = 0;
      }
      continue;
    }

    if (signal.color === 'red') {
      if (signal.elapsed >= SIGNAL_ALL_RED_DURATION) {
        const nextPhase =
          resolveApproach(node, signal.pendingPhase, queue) ||
          signal.pendingPhase ||
          signal.phase;
        const previousPhase = signal.phase;
        signal.phase = nextPhase;
        signal.color = 'green';
        signal.pendingPhase = null;
        signal.elapsed = 0;
        signal.waitAges[nextPhase] = 0;
        if (nextPhase !== previousPhase) {
          state.phaseChangeCount += 1;
        }
      }
      continue;
    }

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
        const cap = Math.max(2, len / AVERAGE_VEHICLE_SPACING);
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

function tryMidSegmentLaneChanges(vehicles) {
  const laneMap = new Map();
  for (const vehicle of vehicles) {
    const segment = getVehicleSegment(vehicle);
    if (!segment) continue;
    const roadLanes = segment.road?.lanes ?? 1;
    if (roadLanes <= 1) continue;
    const laneIndex = Math.min(vehicle.laneIndex ?? 0, Math.max(0, roadLanes - 1));
    const key = `${segment.startId}->${segment.endId}:${laneIndex}`;
    if (!laneMap.has(key)) laneMap.set(key, []);
    laneMap.get(key).push({ vehicle, segment, laneIndex });
  }

  for (const laneEntries of laneMap.values()) {
    if (laneEntries.length < 2) continue;
    laneEntries.sort((a, b) => b.vehicle.progress - a.vehicle.progress);

    const leader = laneEntries[0];
    if (leader.vehicle.speed > 3) continue;
    const leaderStopProgress = getStopLineProgress(leader.segment, leader.vehicle);
    if (leader.vehicle.progress < leaderStopProgress - 0.05) continue;

    const roadLanes = leader.segment.road?.lanes ?? 1;

    for (let fi = 1; fi < laneEntries.length; fi += 1) {
      const follower = laneEntries[fi];
      if (follower.vehicle.speed > 5) continue;
      const gap =
        (leader.vehicle.progress - follower.vehicle.progress) * follower.segment.length -
        getVehicleBodyClearance(leader.vehicle, follower.vehicle);
      if (gap > getVehicleSafeGap(follower.vehicle) * 1.5) continue;

      const candidateLanes = [];
      for (let cl = 0; cl < roadLanes; cl += 1) {
        if (cl !== follower.laneIndex) candidateLanes.push(cl);
      }
      candidateLanes.sort((a, b) => Math.abs(a - follower.laneIndex) - Math.abs(b - follower.laneIndex));

      let bestLane = -1;
      let bestGap = -1;
      for (const targetLane of candidateLanes) {
        const targetKey = `${follower.segment.startId}->${follower.segment.endId}:${targetLane}`;
        const targetEntries = laneMap.get(targetKey) || [];
        let laneBlocked = false;
        let minForwardGap = follower.segment.length * (1 - follower.vehicle.progress);

        for (const other of targetEntries) {
          const dist = Math.abs(other.vehicle.progress - follower.vehicle.progress) * follower.segment.length;
          if (dist < getVehicleSpacingDistance(other.vehicle, follower.vehicle)) {
            laneBlocked = true;
            break;
          }
          const fwdDelta = (other.vehicle.progress - follower.vehicle.progress) * follower.segment.length;
          if (fwdDelta > 0) {
            const bodyGap = fwdDelta - getVehicleBodyClearance(other.vehicle, follower.vehicle);
            minForwardGap = Math.min(minForwardGap, bodyGap);
          }
        }
        if (!laneBlocked && minForwardGap > getVehicleSafeGap(follower.vehicle)) {
          if (minForwardGap > bestGap) {
            bestGap = minForwardGap;
            bestLane = targetLane;
          }
        }
      }

      if (bestLane >= 0) {
        const oldKey = `${follower.segment.startId}->${follower.segment.endId}:${follower.laneIndex}`;
        const newKey = `${follower.segment.startId}->${follower.segment.endId}:${bestLane}`;
        const oldList = laneMap.get(oldKey);
        if (oldList) {
          const idx = oldList.indexOf(follower);
          if (idx >= 0) oldList.splice(idx, 1);
        }
        follower.vehicle.laneIndex = bestLane;
        follower.laneIndex = bestLane;
        if (!laneMap.has(newKey)) laneMap.set(newKey, []);
        laneMap.get(newKey).push(follower);
      }
    }
  }
}

function updateVehicles(dt) {
  refreshPlannedNextLanes(state.vehicles);
  tryMidSegmentLaneChanges(state.vehicles);
  const gapMap = buildLaneGapMap();
  const intersectionReservations = buildIntersectionReservations(dt);
  const survivors = [];

  for (const vehicle of state.vehicles) {
    const tickStartProgress = vehicle.progress;
    let segment = getVehicleSegment(vehicle);
    if (!segment) {
      const currentNodeId = vehicle.route[vehicle.segmentIndex] || vehicle.route[vehicle.route.length - 1];
      const previousNodeId = vehicle.segmentIndex > 0 ? vehicle.route[vehicle.segmentIndex - 1] : null;
      const repaired = rerouteVehicleFromNode(vehicle, currentNodeId, previousNodeId);
      if (currentNodeId && currentNodeId === vehicle.route[vehicle.route.length - 1]) {
        state.vehiclesCompleted += 1;
        continue;
      }
      if (!repaired) {
        survivors.push(vehicle);
        continue;
      }
      segment = getVehicleSegment(vehicle);
      if (!segment) {
        survivors.push(vehicle);
        continue;
      }
    }

    const startingSpeed = vehicle.speed;
    let targetSpeed = getVehicleTargetRoadSpeed(vehicle, segment.road);
    const gap = gapMap.get(vehicle.id);
    const withinIntersectionCarryWindow =
      (vehicle.intersectionCarrySpeed ?? 0) > 0 &&
      vehicle.progress < (vehicle.intersectionCarryUntilProgress ?? 0);

    if (typeof gap === 'number' && !withinIntersectionCarryWindow) {
      targetSpeed = Math.min(targetSpeed, Math.max(0, gap * 2.3));
    }

    if (withinIntersectionCarryWindow) {
      const carrySpeed = Math.min(
        getVehicleTargetRoadSpeed(vehicle, segment.road),
        vehicle.intersectionCarrySpeed ?? 0
      );
      if (carrySpeed > 0) {
        targetSpeed = Math.max(targetSpeed, carrySpeed);
      }
    } else {
      vehicle.intersectionCarrySpeed = 0;
      vehicle.intersectionCarryUntilProgress = 0;
    }

    const remainingDistance = segment.length * (1 - vehicle.progress);
    const stopLineDistance = getVehicleStopDistance(vehicle, segment);
    const commitDistance = getVehicleCommitDistance(vehicle, segment);
    const committedToIntersection = remainingDistance <= commitDistance;
    const laneGreen = isLaneGreen(segment, remainingDistance, vehicle);
    const reservationOwner = intersectionReservations.get(segment.endId);
    const approachingIntersection =
      remainingDistance <= getVehicleReserveDistance(vehicle, segment) + Math.max(vehicle.speed, targetSpeed) * dt + 1;
    const reservationIgnored = ignoresReservationForReverseLane(segment, reservationOwner);
    const intersectionBlocked =
      approachingIntersection &&
      !committedToIntersection &&
      reservationOwner &&
      reservationOwner !== vehicle.id &&
      !reservationIgnored;
    const canProceed = (laneGreen || committedToIntersection) && !intersectionBlocked;
    let brakingForControl = false;

    if (!canProceed) {
      const stopDistance = Math.max(0, remainingDistance - stopLineDistance);
      if (stopDistance < 56) {
        targetSpeed = Math.min(targetSpeed, stopDistance * 2.2);
      }
      brakingForControl = true;
      if (laneGreen && intersectionBlocked && stopDistance < 30) {
        vehicle.stuckAtIntersectionTicks = (vehicle.stuckAtIntersectionTicks ?? 0) + 1;
      }
    } else if (remainingDistance > getVehicleReserveDistance(vehicle, segment) + 20) {
      vehicle.stuckAtIntersectionTicks = 0;
    }

    if (targetSpeed < startingSpeed - 0.35) {
      brakingForControl = true;
    }

    const accelLimit =
      targetSpeed >= vehicle.speed
        ? ACCELERATION * (vehicle.accelerationFactor ?? 1)
        : DECELERATION * (vehicle.brakingFactor ?? 1);
    const diff = targetSpeed - vehicle.speed;
    const maxChange = accelLimit * dt;
    if (Math.abs(diff) > maxChange) {
      vehicle.speed += Math.sign(diff) * maxChange;
    } else {
      vehicle.speed = targetSpeed;
    }

    let moveDistance = Math.max(0, vehicle.speed * dt);

    if (!canProceed) {
      const stopDistance = Math.max(0, remainingDistance - stopLineDistance);
      moveDistance = Math.min(moveDistance, stopDistance);
    }

    if (typeof gap === 'number' && !withinIntersectionCarryWindow) {
      moveDistance = Math.min(moveDistance, Math.max(0, gap - getVehicleSafeGap(vehicle)));
    }

    vehicle.progress += moveDistance / segment.length;

    let arrived = false;
    let haltedAtIntersection = false;

    while (vehicle.progress >= 1) {
      const crossingIntersectionId = segment.endId;
      const crossingOwner = intersectionReservations.get(crossingIntersectionId);
      const crossingIgnored = ignoresReservationForReverseLane(segment, crossingOwner);
      const committedCrossing = tickStartProgress >= getCommitProgress(segment, vehicle);
      const crossingGreen = isLaneGreen(segment, 0, vehicle);
      const crossingReservedForVehicle =
        !crossingOwner || crossingOwner === vehicle.id || crossingIgnored;
      const canEnterIntersection =
        crossingReservedForVehicle && (committedCrossing || crossingGreen);

      if (!canEnterIntersection) {
        if (crossingGreen && crossingOwner && crossingOwner !== vehicle.id) {
          vehicle.stuckAtIntersectionTicks = (vehicle.stuckAtIntersectionTicks ?? 0) + 1;
        }
        const holdProgress = clamp(
          Math.max(tickStartProgress, getStopLineProgress(segment, vehicle)),
          0,
          0.9999
        );
        vehicle.progress = Math.min(vehicle.progress, holdProgress);
        applyIntersectionHoldSpeed(vehicle, dt);
        haltedAtIntersection = true;
        break;
      }

      const overflow = (vehicle.progress - 1) * segment.length;
      const nextStartId = crossingIntersectionId;
      let nextEndId = vehicle.route[vehicle.segmentIndex + 2];
      let nextRoad = nextEndId ? findRoadBetween(nextStartId, nextEndId) : null;
      if (nextEndId && !nextRoad) {
        const repaired = rerouteVehicleFromNode(vehicle, nextStartId, segment.startId);
        if (!repaired) {
          const holdProgress = clamp(
            Math.max(tickStartProgress, getStopLineProgress(segment, vehicle)),
            0,
            0.9999
          );
          vehicle.progress = Math.min(vehicle.progress, holdProgress);
          applyIntersectionHoldSpeed(vehicle, dt);
          haltedAtIntersection = true;
          break;
        }
        segment = getVehicleSegment(vehicle);
        if (!segment) {
          arrived = vehicle.route[vehicle.route.length - 1] === nextStartId;
          break;
        }
        nextEndId = vehicle.route[vehicle.segmentIndex + 1];
        nextRoad = nextEndId ? findRoadBetween(nextStartId, nextEndId) : null;
      }

      const currentRoad = segment.road;

      if (nextEndId && nextRoad) {
        const nextStart = getIntersectionById(nextStartId);
        const nextEnd = getIntersectionById(nextEndId);
        if (!nextStart || !nextEnd) {
          arrived = true;
          break;
        }

        const nextLength = Math.max(1, distance(nextStart, nextEnd));
        const minEntryProgress =
          getIntersectionReleaseDistance({ length: nextLength, road: nextRoad }, vehicle) / nextLength;
        const projectedProgress = clamp(
          Math.max(overflow / nextLength, minEntryProgress),
          0,
          0.98
        );
        let pickedLane = resolveVehicleNextLane(vehicle, segment, projectedProgress);
        if (pickedLane < 0 && (vehicle.stuckAtIntersectionTicks ?? 0) >= STUCK_TICKS_RELAXED_ENTRY) {
          pickedLane = pickLaneRelaxed(nextStartId, nextEndId, vehicle.id, projectedProgress, nextRoad, vehicle);
        }
        if (pickedLane < 0) {
          vehicle.stuckAtIntersectionTicks = (vehicle.stuckAtIntersectionTicks ?? 0) + 1;
          const holdProgress = clamp(
            Math.max(tickStartProgress, getStopLineProgress(segment, vehicle)),
            0,
            0.9999
          );
          vehicle.progress = Math.min(vehicle.progress, holdProgress);
          applyIntersectionHoldSpeed(vehicle, dt);
          haltedAtIntersection = true;
          break;
        }
        vehicle.laneIndex = pickedLane;
        vehicle.stuckAtIntersectionTicks = 0;
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
      const nextPlannedLane = pickVehicleNextLane(vehicle, segment);
      vehicle.plannedNextLaneIndex = nextPlannedLane >= 0 ? nextPlannedLane : null;

      const minExitProgress = getIntersectionReleaseDistance(segment, vehicle) / segment.length;
      vehicle.progress = clamp(
        Math.max(overflow / segment.length, minExitProgress),
        0,
        0.98
      );
      const carryThroughSpeed = getVehicleCarryThroughSpeed(vehicle, currentRoad, segment.road);
      vehicle.speed = Math.max(carryThroughSpeed, Math.min(vehicle.speed, getVehicleTargetRoadSpeed(vehicle, segment.road)));
      vehicle.intersectionCarrySpeed = carryThroughSpeed;
      vehicle.intersectionCarryUntilProgress = Math.max(
        vehicle.progress,
        getIntersectionExitProgress(segment, vehicle)
      );
    }

    if (haltedAtIntersection && vehicle.speed < 0.2) {
      vehicle.speed = 0;
    }

    vehicle.brakeLightOn =
      brakingForControl ||
      haltedAtIntersection ||
      vehicle.speed < startingSpeed - 0.35 ||
      (vehicle.speed < 1.5 && targetSpeed < 4);

    if (arrived) {
      state.vehiclesCompleted += 1;
    } else {
      survivors.push(vehicle);
    }
  }

  state.vehicles = survivors;
  refreshPlannedNextLanes(state.vehicles);
  enforceLaneSeparation(state.vehicles, dt);
  const overlapCheckInterval = getOverlapCheckInterval(state.vehicles.length);
  state.overlapCheckCounter = (state.overlapCheckCounter + 1) % overlapCheckInterval;
  if (state.overlapCheckCounter === 0) {
    state.lastOverlapPairs = countVehicleOverlaps();
    if (state.lastOverlapPairs > 0) {
      state.collisionWarningTicks += 1;
    }
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
}

function drawGrid() {
  const bounds = getVisibleWorldBounds();
  ctx.strokeStyle = 'rgba(210, 228, 238, 0.06)';
  ctx.lineWidth = 1 / state.view.zoom;
  const step = 42;
  const startX = Math.floor(bounds.minX / step) * step;
  const endX = Math.ceil(bounds.maxX / step) * step;
  const startY = Math.floor(bounds.minY / step) * step;
  const endY = Math.ceil(bounds.maxY / step) * step;

  for (let x = startX; x <= endX; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, startY);
    ctx.lineTo(x, endY);
    ctx.stroke();
  }
  for (let y = startY; y <= endY; y += step) {
    ctx.beginPath();
    ctx.moveTo(startX, y);
    ctx.lineTo(endX, y);
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
      const signalColor = getApproachSignalColor(node, approach);
      ctx.fillStyle =
        signalColor === 'green'
          ? '#74d48b'
          : signalColor === 'yellow'
          ? '#f6c445'
          : '#f05d5e';
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

function traceVehicleTurnIndicator(turnIntent, length, width) {
  ctx.beginPath();
  if (turnIntent === 'left') {
    ctx.moveTo(-length * 0.14, 0);
    ctx.lineTo(-length * 0.01, 0);
    ctx.lineTo(length * 0.08, -width * 0.32);
    ctx.moveTo(length * 0.08, -width * 0.32);
    ctx.lineTo(-length * 0.02, -width * 0.34);
    ctx.moveTo(length * 0.08, -width * 0.32);
    ctx.lineTo(length * 0.02, -width * 0.12);
    return;
  }
  if (turnIntent === 'right') {
    ctx.moveTo(-length * 0.14, 0);
    ctx.lineTo(-length * 0.01, 0);
    ctx.lineTo(length * 0.08, width * 0.32);
    ctx.moveTo(length * 0.08, width * 0.32);
    ctx.lineTo(-length * 0.02, width * 0.34);
    ctx.moveTo(length * 0.08, width * 0.32);
    ctx.lineTo(length * 0.02, width * 0.12);
    return;
  }
  ctx.moveTo(-length * 0.12, 0);
  ctx.lineTo(length * 0.14, 0);
  ctx.moveTo(length * 0.14, 0);
  ctx.lineTo(length * 0.03, -width * 0.16);
  ctx.moveTo(length * 0.14, 0);
  ctx.lineTo(length * 0.03, width * 0.16);
}

function drawVehicle(vehicle) {
  const pose = getVehicleWorldPose(vehicle);
  if (!pose) return;
  const length = getVehicleLength(vehicle);
  const width = getVehicleWidth(vehicle);
  const turnIntent = getVehicleTurnIntent(vehicle);
  const brakeLightOn = getVehicleBrakeLightState(vehicle);

  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.heading);

  ctx.fillStyle = 'rgba(9, 15, 23, 0.44)';
  ctx.beginPath();
  ctx.moveTo(length * 0.54, 0);
  ctx.lineTo(-length * 0.52, width * 0.72);
  ctx.lineTo(-length * 0.7, 0);
  ctx.lineTo(-length * 0.52, -width * 0.72);
  ctx.closePath();
  ctx.fill();

  ctx.fillStyle = vehicle.color;
  ctx.beginPath();
  ctx.moveTo(length * 0.5, 0);
  ctx.lineTo(-length * 0.45, width * 0.5);
  ctx.lineTo(-length * 0.45, -width * 0.5);
  ctx.closePath();
  ctx.fill();

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.58)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(length * 0.1, 0);
  ctx.lineTo(-length * 0.28, 0);
  ctx.stroke();

  ctx.strokeStyle = 'rgba(8, 16, 24, 0.92)';
  ctx.lineWidth = 2.6;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  traceVehicleTurnIndicator(turnIntent, length, width);
  ctx.stroke();

  ctx.strokeStyle = turnIntent === 'straight' ? '#fff8c2' : '#ffd166';
  ctx.lineWidth = 1.25;
  traceVehicleTurnIndicator(turnIntent, length, width);
  ctx.stroke();

  const brakeGlow = brakeLightOn ? 'rgba(255, 82, 82, 0.95)' : 'rgba(120, 36, 36, 0.45)';
  const brakeCore = brakeLightOn ? '#ffd0d0' : '#7d3030';
  const brakeRadius = Math.max(1.35, width * 0.14);
  const rearX = -length * 0.36;
  const rearY = width * 0.24;

  ctx.fillStyle = brakeGlow;
  ctx.beginPath();
  ctx.arc(rearX, -rearY, brakeRadius, 0, Math.PI * 2);
  ctx.arc(rearX, rearY, brakeRadius, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = brakeCore;
  ctx.beginPath();
  ctx.arc(rearX, -rearY, brakeRadius * 0.45, 0, Math.PI * 2);
  ctx.arc(rearX, rearY, brakeRadius * 0.45, 0, Math.PI * 2);
  ctx.fill();

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
  ctx.fillRect(12, 12, 430, 82);

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
  ctx.fillText(
    `View: ${Math.round(state.view.zoom * 100)}% | Wheel: zoom | Drag empty space: pan`,
    20,
    66
  );

  if (state.mode === 'connect-roads' && state.connectStartId) {
    ctx.fillStyle = 'rgba(251, 133, 0, 0.65)';
    ctx.fillRect(12, 104, 390, 28);
    ctx.fillStyle = '#fff6ea';
    ctx.fillText(`Connect from ${state.connectStartId}: click another intersection`, 18, 112);
  }
}

function render() {
  drawBackground();
  ctx.save();
  ctx.translate(state.view.x, state.view.y);
  ctx.scale(state.view.zoom, state.view.zoom);
  drawGrid();
  drawRoads();
  drawIntersections();
  drawVehicles();
  ctx.restore();
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
    `vehicle_mix: ${buildVehicleMixSummary() || 'n/a'}`,
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
    ensureSignalCycleState(node.signal);
    return {
      id: node.id,
      x: round(node.x, 1),
      y: round(node.y, 1),
      entry_weight: round(node.entryWeight, 2),
      exit_weight: round(node.exitWeight, 2),
      signal: {
        phase: node.signal.phase,
        color: node.signal.color,
        pending_phase: node.signal.pendingPhase,
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
    const capacity = Math.max(2, length / AVERAGE_VEHICLE_SPACING);
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
      type: vehicle.type,
      label: vehicle.label,
      lane_style: vehicle.laneStyle || 'normal',
      lane_drift_px: round(getVehicleLaneDrift(vehicle), 2),
      turn_intent: getVehicleTurnIntent(vehicle),
      brake_light_on: getVehicleBrakeLightState(vehicle),
      lane_from: segment?.startId || null,
      lane_to: segment?.endId || null,
      lane_index: vehicle.laneIndex ?? 0,
      progress: round(vehicle.progress, 3),
      speed: round(vehicle.speed, 2),
      length: round(getVehicleLength(vehicle), 1),
      width: round(getVehicleWidth(vehicle), 1),
      route_end: vehicle.route[vehicle.route.length - 1] || null,
    };
  });

  return {
    coordinate_system: 'origin=(top-left), +x=right, +y=down, units=world pixels',
    mode: state.mode,
    running: state.running,
    time_s: round(state.simulationTime, 2),
    view: {
      offset_x: round(state.view.x, 2),
      offset_y: round(state.view.y, 2),
      zoom: round(state.view.zoom, 3),
    },
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
  const previousCenter =
    canvas.width > 0 && canvas.height > 0
      ? screenToWorld(canvas.width * 0.5, canvas.height * 0.5)
      : null;
  const bounds = ui.stageWrap.getBoundingClientRect();
  const width = Math.max(720, Math.floor(bounds.width - 2));
  const height = Math.max(520, Math.floor(bounds.height - 2));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    if (previousCenter) {
      state.view.x = canvas.width * 0.5 - previousCenter.x * state.view.zoom;
      state.view.y = canvas.height * 0.5 - previousCenter.y * state.view.zoom;
    }
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

  state.currentScenarioId = 'sample-grid';
  state.vehiclesCompleted = 0;
  state.simulationTime = 0;
  state.spawnAccumulator = 0;
  state.lastQueueMetrics = new Map();
  state.lastFlowMetrics = new Map();
  state.pendingAgentDecision = false;
  state.lastAgentDecisionAt = 0;
  focusViewOnMap();
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
    const n = add(cx, cy - d);
    const s = add(cx, cy + d);
    const e = add(cx + d, cy);
    const w = add(cx - d, cy);
    const ne = add(cx + d * 0.78, cy - d * 0.78);
    const nw = add(cx - d * 0.78, cy - d * 0.78);
    const se = add(cx + d * 0.78, cy + d * 0.78);
    const sw = add(cx - d * 0.78, cy + d * 0.78);
    link(c, n); link(c, s); link(c, e); link(c, w);
    link(c, ne); link(c, nw); link(c, se); link(c, sw);
    const cNode = getIntersectionById(c);
    if (cNode) { cNode.entryWeight = 3; cNode.exitWeight = 2; }
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
  focusViewOnMap();
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
    loadScenarioPreset('sample-grid');
  });

  ui.scenarioPreset?.addEventListener('change', (event) => {
    loadScenarioPreset(event.target.value);
    render();
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

  ui.scenarioLanes?.addEventListener('change', (event) => {
    const value = String(event.target.value || '');
    if (value === 'mixed') {
      updateScenarioLaneControl();
      return;
    }
    setAllRoadLanes(value);
    renderInspector();
    render();
  });

  ui.globalSignalMode?.addEventListener('change', (event) => {
    const nextMode = normalizeSignalMode(event.target.value, '');
    if (!nextMode) {
      updateGlobalSignalModeControl();
      return;
    }
    window.setAllSignalModes(nextMode);
    renderInspector();
    render();
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
        setIntersectionSignalMode(node, nextMode);
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
        state.defaultRoadLanes = newLanes;
        if (newLanes > 1 && newLanes !== oldLanes) {
          redistributeVehiclesOnRoad(road);
        }
        updateScenarioLaneControl();
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
  canvas.addEventListener('wheel', handleCanvasWheel, { passive: false });
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());
  window.addEventListener('mouseup', handleCanvasPointerUp);

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

  if (state.renderEnabled) {
    render();
  }
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
window.getOverlapDiagnostics = (limit = 10) => collectVehicleOverlaps(limit);
window.stepSimulation = stepSimulation;
window.clearMap = clearMap;
window.buildSampleGrid = buildSampleGrid;
window.buildScenario = buildScenario;
window.getRewardMetrics = () => {
  let totalQueue = 0;
  for (const q of state.lastQueueMetrics.values()) {
    totalQueue += (q.N || 0) + (q.E || 0) + (q.S || 0) + (q.W || 0);
  }
  const avgSpeed = getAverageSpeed();
  const stoppedVehicles = state.vehicles.reduce((sum, vehicle) => sum + (vehicle.speed < 6 ? 1 : 0), 0);
  return {
    vehiclesCompleted: state.vehiclesCompleted,
    totalQueue,
    phaseChanges: state.phaseChangeCount,
    vehiclesActive: state.vehicles.length,
    avgSpeed,
    stoppedVehicles,
    overlapPairs: state.lastOverlapPairs,
    collisionWarnings: state.collisionWarningTicks,
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
window.setRenderEnabled = (enabled, options = {}) => {
  state.renderEnabled = !!enabled;
  if (options.pauseSimulation) {
    state.savedRunning = state.running;
    state.running = false;
  } else if (options.pauseSimulation === false && state.savedRunning !== undefined) {
    state.running = state.savedRunning;
    state.savedRunning = undefined;
  }
};
window.setAllSignalModes = (mode) => {
  const nextMode = normalizeSignalMode(mode, 'adaptive');
  for (const node of state.intersections) {
    setIntersectionSignalMode(node, nextMode);
  }
  updateGlobalSignalModeControl();
};
window.setAllRoadLanes = (lanes) => {
  setAllRoadLanes(lanes);
  renderInspector();
  render();
};
window.loadScenarioPreset = (preset) => {
  loadScenarioPreset(preset);
  render();
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
  updateGlobalSignalModeControl();
  updateScenarioPresetControl();
  updateScenarioLaneControl();
  updateModeButtons();
  updateSpawnControls();
  updateRunButton();
  renderWeightTable();
  renderInspector();
  render();
  requestAnimationFrame(frame);
}

init();
