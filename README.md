# Traffic Network Sandbox

Design intersections and roads, then simulate weighted or random traffic flow. Built for experimenting with AI-controlled traffic lights based on congestion.

## Features

- **Editor modes**: Add intersections, connect roads, select/move elements
- **Traffic simulation**: Weighted demand (entry/exit weights) or random demand
- **Traffic lights**: Free flow, adaptive (pressure-based), fixed cycle, or external AI
- **AI hooks**: `setTrafficLightAgent(fn)` to plug in custom traffic light logic
- **Backtest page**: Compare adaptive vs external AI vs static signals across scenarios

## Quick Start

1. Open `index.html` in a browser, or serve the project:

   ```bash
   npx serve .
   ```

2. Use **Add Sample Grid** to create a starter 3×3 network, or design your own.

3. Adjust spawn rate and mode, then run the simulation.

## Editor Modes

| Mode | Shortcut | Action |
|------|----------|--------|
| Add Intersection | `A` | Click to place intersections |
| Connect Roads | `B` | Click two intersections to link them |
| Select / Move | `Enter` | Click to select; drag intersections to move |

**Other shortcuts**: `Space` = pause/resume, `F` = fullscreen

## Traffic Options

- **Spawn mode**: Weighted (uses entry/exit weights) or Random
- **Spawn rate**: 0–120 vehicles/min
- **Auto spawn**: Toggle automatic vehicle generation

## Traffic Light Modes (per intersection)

- **No traffic light**: Free flow; vehicles pass through
- **Adaptive AI**: Built-in pressure-based phase selection (queue + flow + wait age)
- **Static**: Fixed green duration per phase
- **External AI**: Controlled via `setTrafficLightAgent`

## AI Integration

```javascript
// Provide a function that receives a snapshot and returns phase decisions
setTrafficLightAgent((snapshot) => {
  const decisions = snapshot.intersections.map((i) => ({
    id: i.id,
    phase: i.queue.N > i.queue.E ? 'N' : 'E',  // example
  }));
  return { phases: Object.fromEntries(decisions.map(d => [d.id, d.phase])) };
});
```

Snapshot includes per-intersection: `queue`, `approach_flow`, `signal.wait_ages_s`, `coordination_bonus`, and more.

## Project Structure

```
├── index.html      # Main sandbox UI
├── backtest.html   # Adaptive vs External vs Static comparison
├── app.js          # Simulation logic
├── styles.css      # Layout and controls
└── package.json    # Dependencies (Playwright for tests)
```

## License

ISC
