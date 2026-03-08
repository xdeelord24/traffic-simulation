const TF = globalThis.tf;

if (!TF) {
  throw new Error('TensorFlow.js must be loaded before rl-agent.js');
}

export const RL_ACTIONS = ['N', 'E', 'S', 'W'];
export const DEFAULT_MAX_INTERSECTIONS = 16;
export const FEATURES_PER_INTERSECTION = 22;

const QUEUE_SCALE = 12;
const FLOW_SCALE = 8;
const WAIT_AGE_SCALE = 12;
const COORD_SCALE = 1.5;
const ELAPSED_SCALE = 12;
const OCCUPANCY_SCALE = 1.5;

function numericId(id) {
  const numeric = Number(String(id || '').replace(/[^0-9]/g, ''));
  return Number.isFinite(numeric) ? numeric : 0;
}

function sortIntersections(intersections) {
  return [...intersections].sort((a, b) => numericId(a.id) - numericId(b.id));
}

function normalize(value, scale) {
  return Math.max(0, Math.min(2, (Number(value) || 0) / scale));
}

function oneHotPhase(phase) {
  return RL_ACTIONS.map((action) => (phase === action ? 1 : 0));
}

export function encodeState(snapshot, maxIntersections = DEFAULT_MAX_INTERSECTIONS, eligibleMode = null) {
  return prepareObservation(snapshot, maxIntersections, eligibleMode).observation;
}

export function computeRewardBreakdown(prevMetrics, currMetrics, phaseChanges = 0) {
  const completedDelta =
    Math.max(0, (currMetrics?.vehiclesCompleted || 0) - (prevMetrics?.vehiclesCompleted || 0));
  const totalQueue = currMetrics?.totalQueue || 0;
  const queueDelta = totalQueue - (prevMetrics?.totalQueue || 0);
  const speedDelta = (currMetrics?.avgSpeed || 0) - (prevMetrics?.avgSpeed || 0);
  const stoppedVehicles = currMetrics?.stoppedVehicles || 0;
  const overlapDelta = Math.max(
    0,
    (currMetrics?.overlapPairs || 0) - (prevMetrics?.overlapPairs || 0)
  );
  const collisionDelta = Math.max(
    0,
    (currMetrics?.collisionWarnings || 0) - (prevMetrics?.collisionWarnings || 0)
  );

  const terms = {
    completed: completedDelta * 2.4,
    speed: speedDelta * 0.08,
    queue: -totalQueue * 0.015,
    queueGrowth: -Math.max(0, queueDelta) * 0.05,
    stopped: -stoppedVehicles * 0.025,
    phaseSwitch: -phaseChanges * 0.18,
    overlap: -overlapDelta * 2.5,
    collision: -collisionDelta * 0.75,
  };
  const total =
    terms.completed +
    terms.speed +
    terms.queue +
    terms.queueGrowth +
    terms.stopped +
    terms.phaseSwitch +
    terms.overlap +
    terms.collision;

  return {
    total,
    terms,
    completedDelta,
    totalQueue,
    queueDelta,
    speedDelta,
    stoppedVehicles,
    phaseChanges,
    overlapDelta,
    collisionDelta,
  };
}

export function computeReward(prevMetrics, currMetrics, phaseChanges = 0) {
  return computeRewardBreakdown(prevMetrics, currMetrics, phaseChanges).total;
}

export function buildPressureFallbackDecisions(snapshot, eligibleMode = 'rl') {
  const decisions = [];
  const roads = snapshot.roads || [];
  for (const node of sortIntersections(snapshot.intersections || [])) {
    if (eligibleMode && node.signal?.mode !== eligibleMode) continue;
    const approaches = node.signal?.available_approaches || [];
    if (approaches.length === 0) continue;

    const queue = node.queue || {};
    const flow = node.approach_flow || {};
    const waitAges = node.signal?.wait_ages_s || {};
    const coord = node.coordination_bonus || {};
    const phase = node.signal?.phase || approaches[0];
    const elapsed = node.signal?.elapsed_s ?? 0;
    const minGreen = node.signal?.min_green_s ?? 3;
    const maxGreen = node.signal?.max_green_s ?? 12;

    let downstreamOccupancy = 0;
    const exitRoads = roads.filter((road) => road.from === node.id);
    if (exitRoads.length > 0) {
      downstreamOccupancy =
        exitRoads.reduce((sum, road) => sum + (road.occupancy_fwd ?? 0), 0) / exitRoads.length;
    }
    const marginMult = downstreamOccupancy > 0.85 ? 1.4 : 1;

    const scores = {};
    for (const approach of approaches) {
      scores[approach] =
        (queue[approach] || 0) * 2.8 +
        (flow[approach] || 0) * 1.2 +
        (waitAges[approach] || 0) * 0.5 +
        (coord[approach] || 0);
    }

    let bestApproach = approaches[0];
    for (const approach of approaches) {
      if ((scores[approach] || 0) > (scores[bestApproach] || 0)) {
        bestApproach = approach;
      }
    }

    let starvationApproach = approaches[0];
    for (const approach of approaches) {
      if ((waitAges[approach] || 0) > (waitAges[starvationApproach] || 0)) {
        starvationApproach = approach;
      }
    }

    const currentScore = scores[phase] || 0;
    const bestScore = scores[bestApproach] || 0;
    const currentDemand = (queue[phase] || 0) + (flow[phase] || 0) * 0.5;
    const starvationForced =
      starvationApproach !== phase && (waitAges[starvationApproach] || 0) >= 6;
    const betterDemandAbs =
      bestApproach !== phase && bestScore >= currentScore + 0.5 * marginMult;
    const betterDemandRatio =
      bestApproach !== phase &&
      currentScore > 0.1 &&
      bestScore >= currentScore * Math.pow(1.25, marginMult);
    const alternateDemand = bestApproach !== phase && bestScore > 0.2;

    const mustSwitch = elapsed >= maxGreen;
    const shouldSwitch =
      elapsed >= minGreen &&
      (starvationForced ||
        betterDemandAbs ||
        betterDemandRatio ||
        (currentDemand <= 0.3 && alternateDemand));

    if (mustSwitch || shouldSwitch) {
      decisions.push({
        id: node.id,
        phase: starvationForced ? starvationApproach : bestApproach,
      });
    }
  }
  return decisions;
}

function buildRoadMaps(snapshot) {
  const outgoing = new Map();
  for (const road of snapshot.roads || []) {
    if (!outgoing.has(road.from)) outgoing.set(road.from, []);
    outgoing.get(road.from).push(road);
  }
  return { outgoing };
}

function prepareObservation(snapshot, maxIntersections, eligibleMode = null) {
  const ordered = sortIntersections(snapshot.intersections || []).filter((node) =>
    eligibleMode ? node.signal?.mode === eligibleMode : true
  );
  const { outgoing } = buildRoadMaps(snapshot);

  const observation = new Float32Array(maxIntersections * FEATURES_PER_INTERSECTION);
  const actionMask = new Float32Array(maxIntersections * RL_ACTIONS.length);
  const activeMask = new Float32Array(maxIntersections);
  const limited = ordered.slice(0, maxIntersections);

  for (let i = 0; i < limited.length; i += 1) {
    const node = limited[i];
    const queue = node.queue || {};
    const flow = node.approach_flow || {};
    const waitAges = node.signal?.wait_ages_s || {};
    const coord = node.coordination_bonus || {};
    const phase = node.signal?.phase || null;
    const offset = i * FEATURES_PER_INTERSECTION;
    const phaseHot = oneHotPhase(phase);
    const exits = outgoing.get(node.id) || [];
    const meanOccupancy =
      exits.length > 0
        ? exits.reduce((sum, road) => sum + (road.occupancy_fwd ?? 0), 0) / exits.length
        : 0;
    const maxOccupancy =
      exits.length > 0
        ? exits.reduce((best, road) => Math.max(best, road.occupancy_fwd ?? 0), 0)
        : 0;
    const elapsedNorm = normalize(node.signal?.elapsed_s || 0, ELAPSED_SCALE);

    observation[offset] = normalize(queue.N, QUEUE_SCALE);
    observation[offset + 1] = normalize(queue.E, QUEUE_SCALE);
    observation[offset + 2] = normalize(queue.S, QUEUE_SCALE);
    observation[offset + 3] = normalize(queue.W, QUEUE_SCALE);
    observation[offset + 4] = normalize(flow.N, FLOW_SCALE);
    observation[offset + 5] = normalize(flow.E, FLOW_SCALE);
    observation[offset + 6] = normalize(flow.S, FLOW_SCALE);
    observation[offset + 7] = normalize(flow.W, FLOW_SCALE);
    observation[offset + 8] = normalize(waitAges.N, WAIT_AGE_SCALE);
    observation[offset + 9] = normalize(waitAges.E, WAIT_AGE_SCALE);
    observation[offset + 10] = normalize(waitAges.S, WAIT_AGE_SCALE);
    observation[offset + 11] = normalize(waitAges.W, WAIT_AGE_SCALE);
    observation[offset + 12] = phaseHot[0];
    observation[offset + 13] = phaseHot[1];
    observation[offset + 14] = phaseHot[2];
    observation[offset + 15] = phaseHot[3];
    observation[offset + 16] = normalize(coord.N, COORD_SCALE);
    observation[offset + 17] = normalize(coord.E, COORD_SCALE);
    observation[offset + 18] = normalize(coord.S, COORD_SCALE);
    observation[offset + 19] = normalize(coord.W, COORD_SCALE);
    observation[offset + 20] = elapsedNorm;
    observation[offset + 21] = Math.max(
      normalize(meanOccupancy, OCCUPANCY_SCALE),
      normalize(maxOccupancy, OCCUPANCY_SCALE)
    );

    const approaches = node.signal?.available_approaches || [];
    activeMask[i] = 1;
    for (let a = 0; a < RL_ACTIONS.length; a += 1) {
      actionMask[i * RL_ACTIONS.length + a] = approaches.includes(RL_ACTIONS[a]) ? 1 : 0;
    }
    if (!approaches.length) {
      actionMask[i * RL_ACTIONS.length] = 1;
      activeMask[i] = 0;
    }
  }

  for (let i = limited.length; i < maxIntersections; i += 1) {
    actionMask[i * RL_ACTIONS.length] = 1;
  }

  return {
    observation,
    actionMask,
    activeMask,
    intersections: limited,
  };
}

function maskedProbabilities(logits, mask) {
  let maxLogit = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < RL_ACTIONS.length; i += 1) {
    if (mask[i] > 0) {
      maxLogit = Math.max(maxLogit, logits[i]);
    }
  }
  if (!Number.isFinite(maxLogit)) {
    return [1, 0, 0, 0];
  }

  const probs = [0, 0, 0, 0];
  let sum = 0;
  for (let i = 0; i < RL_ACTIONS.length; i += 1) {
    if (mask[i] <= 0) continue;
    probs[i] = Math.exp(logits[i] - maxLogit);
    sum += probs[i];
  }
  if (sum <= 0) {
    return [1, 0, 0, 0];
  }
  for (let i = 0; i < probs.length; i += 1) {
    probs[i] /= sum;
  }
  return probs;
}

function sampleAction(probs, deterministic) {
  if (deterministic) {
    let best = 0;
    for (let i = 1; i < probs.length; i += 1) {
      if (probs[i] > probs[best]) best = i;
    }
    return best;
  }

  let roll = Math.random();
  for (let i = 0; i < probs.length; i += 1) {
    roll -= probs[i];
    if (roll <= 0) return i;
  }
  return probs.length - 1;
}

export function decodeActions(actionIndices, snapshot, options = {}) {
  const { maxIntersections = DEFAULT_MAX_INTERSECTIONS, eligibleMode = null } = options;
  const prepared = prepareObservation(snapshot, maxIntersections, eligibleMode);
  const decisions = [];

  for (let i = 0; i < prepared.intersections.length; i += 1) {
    const node = prepared.intersections[i];
    const approaches = node.signal?.available_approaches || [];
    if (!approaches.length) continue;
    const actionIndex = actionIndices[i] ?? 0;
    const chosen = RL_ACTIONS[actionIndex] || approaches[0];
    if (approaches.includes(chosen) && chosen !== node.signal?.phase) {
      decisions.push({ id: node.id, phase: chosen });
    }
  }

  return decisions;
}

export function buildTeacherActionIndices(snapshot, options = {}) {
  const {
    maxIntersections = DEFAULT_MAX_INTERSECTIONS,
    eligibleMode = 'rl',
    decisions = null,
  } = options;
  const prepared = prepareObservation(snapshot, maxIntersections, eligibleMode);
  const teacherDecisions = decisions || buildPressureFallbackDecisions(snapshot, eligibleMode);
  const teacherPhaseById = new Map();

  for (const node of prepared.intersections) {
    const approaches = node.signal?.available_approaches || [];
    const currentPhase = node.signal?.phase;
    teacherPhaseById.set(
      node.id,
      approaches.includes(currentPhase) ? currentPhase : approaches[0] || RL_ACTIONS[0]
    );
  }
  for (const decision of teacherDecisions) {
    if (decision?.id && typeof decision.phase === 'string') {
      teacherPhaseById.set(decision.id, decision.phase);
    }
  }

  const actionIndices = new Array(maxIntersections).fill(0);
  for (let i = 0; i < prepared.intersections.length; i += 1) {
    const node = prepared.intersections[i];
    const approaches = node.signal?.available_approaches || [];
    const phase = teacherPhaseById.get(node.id) || approaches[0] || RL_ACTIONS[0];
    const safePhase = approaches.includes(phase) ? phase : approaches[0] || RL_ACTIONS[0];
    const actionIndex = RL_ACTIONS.indexOf(safePhase);
    actionIndices[i] = actionIndex >= 0 ? actionIndex : 0;
  }

  return {
    actionIndices,
    prepared,
    decisions: teacherDecisions,
  };
}

function buildActorCriticModel(inputSize, outputSize) {
  const input = TF.input({ shape: [inputSize] });
  const hidden1 = TF.layers.dense({ units: 128, activation: 'relu' }).apply(input);
  const hidden2 = TF.layers.dense({ units: 64, activation: 'relu' }).apply(hidden1);
  const policy = TF.layers.dense({ units: outputSize }).apply(hidden2);
  const value = TF.layers.dense({ units: 1 }).apply(hidden2);
  return TF.model({ inputs: input, outputs: [policy, value] });
}

function storageUrl(storageKey) {
  return `indexeddb://traffic-rl-${storageKey}`;
}

export class RLAgent {
  constructor(options = {}) {
    this.maxIntersections = options.maxIntersections || DEFAULT_MAX_INTERSECTIONS;
    this.inputSize = this.maxIntersections * FEATURES_PER_INTERSECTION;
    this.outputSize = this.maxIntersections * RL_ACTIONS.length;
    this.gamma = options.gamma || 0.99;
    this.gaeLambda = options.gaeLambda || 0.95;
    this.clipRatio = options.clipRatio || 0.2;
    this.valueLossCoeff = options.valueLossCoeff || 0.5;
    this.entropyCoeff = options.entropyCoeff || 0.01;
    this.updateEpochs = options.updateEpochs || 4;
    this.learningRate = options.learningRate || 0.0003;
    this.model = buildActorCriticModel(this.inputSize, this.outputSize);
    this.optimizer = TF.train.adam(this.learningRate);
    this.rollout = [];
    this.imitationBuffer = [];
    this.episodeStart = 0;
  }

  beginEpisode() {
    this.episodeStart = this.rollout.length;
  }

  act(snapshot, options = {}) {
    const eligibleMode = options.eligibleMode ?? null;
    const deterministic = options.deterministic === true;
    const prepared = prepareObservation(snapshot, this.maxIntersections, eligibleMode);

    const obsTensor = TF.tensor2d([prepared.observation], [1, this.inputSize]);
    const [policyTensor, valueTensor] = this.model.predict(obsTensor);
    const policyData = policyTensor.dataSync();
    const value = valueTensor.dataSync()[0];
    obsTensor.dispose();
    policyTensor.dispose();
    valueTensor.dispose();

    const actionIndices = new Array(this.maxIntersections).fill(0);
    let logProb = 0;
    for (let i = 0; i < this.maxIntersections; i += 1) {
      if (prepared.activeMask[i] <= 0) continue;
      const logits = policyData.slice(i * RL_ACTIONS.length, (i + 1) * RL_ACTIONS.length);
      const mask = Array.from(
        prepared.actionMask.slice(i * RL_ACTIONS.length, (i + 1) * RL_ACTIONS.length)
      );
      const probs = maskedProbabilities(logits, mask);
      const actionIndex = sampleAction(probs, deterministic);
      actionIndices[i] = actionIndex;
      logProb += Math.log(Math.max(probs[actionIndex] || 1e-8, 1e-8));
    }

    const decisions = decodeActions(actionIndices, snapshot, {
      maxIntersections: this.maxIntersections,
      eligibleMode,
    });

    return {
      decisions,
      transition: {
        observation: Float32Array.from(prepared.observation),
        actionIndices,
        actionMask: Float32Array.from(prepared.actionMask),
        activeMask: Float32Array.from(prepared.activeMask),
        logProb,
        value,
      },
    };
  }

  addTransition(transition, reward, done = false) {
    this.rollout.push({
      ...transition,
      reward,
      done,
      advantage: 0,
      returnValue: 0,
    });
  }

  addImitationSample(transition, teacherActionIndices) {
    this.imitationBuffer.push({
      observation: Float32Array.from(transition.observation),
      actionMask: Float32Array.from(transition.actionMask),
      activeMask: Float32Array.from(transition.activeMask),
      teacherActionIndices: [...teacherActionIndices],
    });
  }

  finishEpisode(lastValue = 0) {
    let gae = 0;
    for (let i = this.rollout.length - 1; i >= this.episodeStart; i -= 1) {
      const item = this.rollout[i];
      const nextValue = i === this.rollout.length - 1 || item.done ? lastValue : this.rollout[i + 1].value;
      const nextNonTerminal = item.done ? 0 : 1;
      const delta = item.reward + this.gamma * nextValue * nextNonTerminal - item.value;
      gae = delta + this.gamma * this.gaeLambda * nextNonTerminal * gae;
      item.advantage = gae;
      item.returnValue = gae + item.value;
    }
    this.episodeStart = this.rollout.length;
  }

  async update() {
    if (!this.rollout.length) {
      return { policyLoss: 0, valueLoss: 0, entropy: 0, loss: 0 };
    }

    const batch = this.rollout.length;
    const observations = this.rollout.map((item) => Array.from(item.observation));
    const actions = this.rollout.map((item) => item.actionIndices);
    const actionMasks = this.rollout.map((item) =>
      Array.from(item.actionMask).reduce((rows, _, index, source) => {
        if (index % RL_ACTIONS.length === 0) {
          rows.push(source.slice(index, index + RL_ACTIONS.length));
        }
        return rows;
      }, [])
    );
    const activeMasks = this.rollout.map((item) => Array.from(item.activeMask));
    const oldLogProbs = this.rollout.map((item) => item.logProb);
    const returns = this.rollout.map((item) => item.returnValue);
    const advantages = this.rollout.map((item) => item.advantage);

    const advMean = advantages.reduce((sum, value) => sum + value, 0) / Math.max(1, advantages.length);
    const advVar =
      advantages.reduce((sum, value) => sum + (value - advMean) ** 2, 0) / Math.max(1, advantages.length);
    const advStd = Math.sqrt(advVar) || 1;
    const normalizedAdvantages = advantages.map((value) => (value - advMean) / advStd);

    const obsTensor = TF.tensor2d(observations, [batch, this.inputSize]);
    const actionTensor = TF.tensor2d(actions, [batch, this.maxIntersections], 'int32');
    const actionMaskTensor = TF.tensor3d(actionMasks, [batch, this.maxIntersections, RL_ACTIONS.length]);
    const activeMaskTensor = TF.tensor2d(activeMasks, [batch, this.maxIntersections]);
    const oldLogProbTensor = TF.tensor1d(oldLogProbs);
    const returnTensor = TF.tensor1d(returns);
    const advantageTensor = TF.tensor1d(normalizedAdvantages);

    let lastMetrics = { policyLoss: 0, valueLoss: 0, entropy: 0, loss: 0 };
    for (let epoch = 0; epoch < this.updateEpochs; epoch += 1) {
      this.optimizer.minimize(() => {
        const outputs = this.model.apply(obsTensor, { training: true });
        const policy = outputs[0];
        const value = outputs[1];
        const reshapedPolicy = policy.reshape([batch, this.maxIntersections, RL_ACTIONS.length]);
        const maskedLogits = reshapedPolicy.add(TF.sub(1, actionMaskTensor).mul(-1e9));
        const logProbs = TF.logSoftmax(maskedLogits, -1);
        const probs = TF.softmax(maskedLogits, -1);
        const actionOneHot = TF.oneHot(actionTensor, RL_ACTIONS.length).toFloat();
        const chosenLogProbs = logProbs.mul(actionOneHot).sum(-1);
        const jointLogProb = chosenLogProbs.mul(activeMaskTensor).sum(-1);
        const ratio = jointLogProb.sub(oldLogProbTensor).exp();
        const unclipped = ratio.mul(advantageTensor);
        const clipped = TF.clipByValue(ratio, 1 - this.clipRatio, 1 + this.clipRatio).mul(advantageTensor);
        const policyLoss = TF.minimum(unclipped, clipped).mean().neg();
        const values = value.reshape([batch]);
        const valueLoss = returnTensor.sub(values).square().mean();
        const entropy = probs.mul(logProbs).sum(-1).neg().mul(activeMaskTensor).sum(-1).mean();
        const loss = policyLoss.add(valueLoss.mul(this.valueLossCoeff)).sub(entropy.mul(this.entropyCoeff));

        lastMetrics = {
          policyLoss: Number(policyLoss.dataSync()[0]),
          valueLoss: Number(valueLoss.dataSync()[0]),
          entropy: Number(entropy.dataSync()[0]),
          loss: Number(loss.dataSync()[0]),
        };
        return loss;
      }, false);
      await TF.nextFrame();
    }

    obsTensor.dispose();
    actionTensor.dispose();
    actionMaskTensor.dispose();
    activeMaskTensor.dispose();
    oldLogProbTensor.dispose();
    returnTensor.dispose();
    advantageTensor.dispose();
    this.rollout = [];
    this.episodeStart = 0;
    return lastMetrics;
  }

  async updateImitation() {
    if (!this.imitationBuffer.length) {
      return { imitationLoss: 0, entropy: 0 };
    }

    const batch = this.imitationBuffer.length;
    const observations = this.imitationBuffer.map((item) => Array.from(item.observation));
    const teacherActions = this.imitationBuffer.map((item) => item.teacherActionIndices);
    const actionMasks = this.imitationBuffer.map((item) =>
      Array.from(item.actionMask).reduce((rows, _, index, source) => {
        if (index % RL_ACTIONS.length === 0) {
          rows.push(source.slice(index, index + RL_ACTIONS.length));
        }
        return rows;
      }, [])
    );
    const activeMasks = this.imitationBuffer.map((item) => Array.from(item.activeMask));

    const obsTensor = TF.tensor2d(observations, [batch, this.inputSize]);
    const teacherTensor = TF.tensor2d(teacherActions, [batch, this.maxIntersections], 'int32');
    const actionMaskTensor = TF.tensor3d(actionMasks, [batch, this.maxIntersections, RL_ACTIONS.length]);
    const activeMaskTensor = TF.tensor2d(activeMasks, [batch, this.maxIntersections]);

    let lastMetrics = { imitationLoss: 0, entropy: 0 };
    for (let epoch = 0; epoch < Math.max(2, this.updateEpochs); epoch += 1) {
      this.optimizer.minimize(() => {
        const outputs = this.model.apply(obsTensor, { training: true });
        const policy = outputs[0];
        const reshapedPolicy = policy.reshape([batch, this.maxIntersections, RL_ACTIONS.length]);
        const maskedLogits = reshapedPolicy.add(TF.sub(1, actionMaskTensor).mul(-1e9));
        const logProbs = TF.logSoftmax(maskedLogits, -1);
        const probs = TF.softmax(maskedLogits, -1);
        const teacherOneHot = TF.oneHot(teacherTensor, RL_ACTIONS.length).toFloat();
        const nll = logProbs.mul(teacherOneHot).sum(-1).neg();
        const imitationLoss = nll.mul(activeMaskTensor).sum(-1).mean();
        const entropy = probs.mul(logProbs).sum(-1).neg().mul(activeMaskTensor).sum(-1).mean();
        const loss = imitationLoss.sub(entropy.mul(this.entropyCoeff * 0.5));

        lastMetrics = {
          imitationLoss: Number(imitationLoss.dataSync()[0]),
          entropy: Number(entropy.dataSync()[0]),
        };
        return loss;
      }, false);
      await TF.nextFrame();
    }

    obsTensor.dispose();
    teacherTensor.dispose();
    actionMaskTensor.dispose();
    activeMaskTensor.dispose();
    this.imitationBuffer = [];
    return lastMetrics;
  }

  async save(storageKey = 'trainer') {
    await this.model.save(storageUrl(storageKey));
  }

  async load(storageKey = 'trainer') {
    const loaded = await TF.loadLayersModel(storageUrl(storageKey));
    if (this.model) {
      this.model.dispose();
    }
    this.model = loaded;
    return this;
  }
}

export async function attachRlRuntime(targetWindow = window, options = {}) {
  const storageKey = options.storageKey || 'main';
  const eligibleMode = options.eligibleMode || 'rl';
  const agent = new RLAgent();
  let loaded = false;

  try {
    await agent.load(storageKey);
    loaded = true;
  } catch (error) {
    loaded = false;
    console.info(`RL model not found for "${storageKey}", using heuristic fallback.`);
  }

  targetWindow.setRlTrafficLightAgent?.((snapshot) => {
    if (!loaded) {
      return buildPressureFallbackDecisions(snapshot, eligibleMode);
    }
    return agent.act(snapshot, { eligibleMode, deterministic: true }).decisions;
  });
  targetWindow.__trafficRlAgent = agent;
  targetWindow.__trafficRlLoaded = loaded;
  return { agent, loaded };
}
