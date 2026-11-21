// Room-decomposed push-optimal Sokoban solver.
//
// Idea: every level of this competition is a one-cell-wide hallway with one
// to four rooms hanging off it (one door each, boxes and goals balanced per
// room, and a box pushed out of its door can never come back). So each room
// is solved on its own with a push-space A* search that is optimal in pushes,
// then the rooms are stitched together with plain player walks along the
// hallway. The push-optimal count of a level is the sum of its rooms'
// optima, so per-room optimality is the whole game.
//
// Per room the search state is (sorted box cells, player reachable-region
// representative). Successors are pushes; the player reaching the square
// behind a box is checked with one BFS per expanded state. The heuristic is
// the cheapest assignment of boxes to goals under static floor distances
// (admissible: one push moves one box by one square), and dead states are
// pruned with reverse-pull dead squares plus a frozen-box fixpoint. The
// search budget is a fixed node count, never a clock, so two runs of this
// program on the same input produce byte-identical output.
//
// The program reads every instance under MAKEFUN_INPUT_DIR (default /input)
// and writes MAKEFUN_OUTPUT_DIR/<id>/solution.txt, a string of u/d/l/r
// moves. It never reads the clock, randomness, or environment beyond the two
// directory variables above.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions: 0 = up, 1 = down, 2 = left, 3 = right. Opposite of d is d ^ 1.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];

// Deterministic work budget: maximum A* expansions per room (or per whole
// level in the fallback). No clock is ever consulted.
const MAX_EXPANSIONS = 5000000;

// ---------------------------------------------------------------------------
// Level parsing
// ---------------------------------------------------------------------------

interface Grid {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (cells past a short row count as walls)
  goal: Uint8Array; // 1 = goal
  boxes: number[]; // cell indices holding a box
  player: number; // cell index of the player
}

function parseLevel(text: string): Grid {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  return { w, h, wall, goal, boxes, player };
}

/** A transposed copy of a level (x and y swapped). Used when the hallway runs vertically. */
function transposeGrid(g: Grid): Grid {
  const w = g.h;
  const h = g.w;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < g.h; y++) {
    for (let x = 0; x < g.w; x++) {
      const src = y * g.w + x;
      const dst = x * w + y;
      if (!g.wall[src]) {
        wall[dst] = 0;
        if (g.goal[src]) goal[dst] = 1;
        if (g.boxes.includes(src)) boxes.push(dst);
        if (g.player === src) player = dst;
      }
    }
  }
  return { w, h, wall, goal, boxes, player };
}

/** Transpose a move string back (u<->l, d<->r). */
function transposeMoves(moves: string): string {
  let out = '';
  for (const c of moves) {
    if (c === 'u') out += 'l';
    else if (c === 'd') out += 'r';
    else if (c === 'l') out += 'u';
    else if (c === 'r') out += 'd';
    else out += c;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Plain player walks (no pushes): BFS over floor cells, boxes are obstacles.
// ---------------------------------------------------------------------------

function walkPath(g: Grid, from: number, to: number, boxAt: Uint8Array): string | null {
  if (from === to) return '';
  const size = g.w * g.h;
  const prevCell = new Int32Array(size).fill(-1);
  const prevDir = new Int8Array(size).fill(-1);
  const queue: number[] = [from];
  prevCell[from] = from;
  for (let head = 0; head < queue.length; head++) {
    const c = queue[head];
    const cx = c % g.w;
    const cy = (c / g.w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
      const n = ny * g.w + nx;
      if (g.wall[n] || boxAt[n] || prevCell[n] !== -1) continue;
      prevCell[n] = c;
      prevDir[n] = d;
      if (n === to) {
        let moves = '';
        let cur = to;
        while (cur !== from) {
          moves = CH[prevDir[cur]] + moves;
          cur = prevCell[cur];
        }
        return moves;
      }
      queue.push(n);
    }
  }
  return null;
}

/** Replay a move string on a level; returns the push count if it solves the level, else -1. */
function replayPushes(g: Grid, moves: string): number {
  const boxAt = new Uint8Array(g.w * g.h);
  for (const b of g.boxes) boxAt[b] = 1;
  let player = g.player;
  let pushes = 0;
  for (const c of moves) {
    const d = CH.indexOf(c);
    if (d < 0) return -1;
    const step = DY[d] * g.w + DX[d];
    const next = player + step;
    if (next < 0 || next >= g.w * g.h || g.wall[next]) return -1;
    if (boxAt[next]) {
      const beyond = next + step;
      if (beyond < 0 || beyond >= g.w * g.h || g.wall[beyond] || boxAt[beyond]) return -1;
      boxAt[next] = 0;
      boxAt[beyond] = 1;
      pushes++;
    }
    player = next;
  }
  for (let i = 0; i < g.w * g.h; i++) {
    if (boxAt[i] && !g.goal[i]) return -1;
  }
  return pushes;
}

// ---------------------------------------------------------------------------
// Decomposition: hallway row + one room per door
// ---------------------------------------------------------------------------

interface RoomSpec {
  door: number; // global cell of the door (room side of the hallway)
  entry: number; // global hallway cell the player stands on to step in
  cells: number[]; // global room cells (door included); boxes live here
  boxes: number[]; // global box cells in this room
  goals: number[]; // global goal cells in this room
}

function decompose(g: Grid): RoomSpec[] | null {
  const H = (g.player / g.w) | 0;
  const rooms: RoomSpec[] = [];
  const used = new Uint8Array(g.w * g.h);
  for (const y of [H - 1, H + 1]) {
    if (y < 0 || y >= g.h) continue;
    const above = y < H;
    for (let x = 0; x < g.w; x++) {
      const door = y * g.w + x;
      if (g.wall[door]) continue;
      const entry = H * g.w + x;
      if (g.wall[entry]) continue; // the door must open onto the hallway
      // Flood the room: floor cells on the room side of the hallway.
      const cells: number[] = [];
      const seen = new Uint8Array(g.w * g.h);
      const queue = [door];
      seen[door] = 1;
      for (let head = 0; head < queue.length; head++) {
        const c = queue[head];
        cells.push(c);
        const cx = c % g.w;
        const cy = (c / g.w) | 0;
        for (let d = 0; d < 4; d++) {
          const nx = cx + DX[d];
          const ny = cy + DY[d];
          if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
          if (above ? ny >= H : ny <= H) continue;
          const n = ny * g.w + nx;
          if (g.wall[n] || seen[n]) continue;
          seen[n] = 1;
          queue.push(n);
        }
      }
      const boxes = g.boxes.filter((b) => seen[b]);
      const goals: number[] = [];
      for (const c of cells) if (g.goal[c]) goals.push(c);
      if (boxes.length === 0) {
        if (goals.length > 0) return null; // goals with no boxes: not this shape
        continue; // empty closet off the hallway: ignore it
      }
      if (boxes.length !== goals.length) return null;
      for (const c of cells) {
        if (used[c]) return null; // rooms must not overlap
        used[c] = 1;
      }
      rooms.push({ door, entry, cells, boxes, goals });
    }
  }
  if (rooms.length === 0) return null;
  let coveredBoxes = 0;
  let coveredGoals = 0;
  for (const r of rooms) {
    coveredBoxes += r.boxes.length;
    coveredGoals += r.goals.length;
    if (r.cells.includes(g.player)) return null;
  }
  let totalGoals = 0;
  for (let i = 0; i < g.w * g.h; i++) if (g.goal[i]) totalGoals++;
  if (coveredBoxes !== g.boxes.length || coveredGoals !== totalGoals) return null;
  return rooms;
}

// ---------------------------------------------------------------------------
// Push-space A* over one box set.
// walkCells: global cells the player may stand on.
// boxCells:  global cells boxes may occupy (subset of walkCells).
// Returns the pushes as {box, dir} with box a global cell, or null.
// ---------------------------------------------------------------------------

interface PushStep {
  box: number; // global cell of the box before the push
  dir: number; // 0..3
}

function pushAStar(
  g: Grid,
  walkCells: number[],
  boxCells: number[],
  goals: number[],
  boxes: number[],
  playerStart: number,
): PushStep[] | null {
  const n = walkCells.length;
  const loc = new Map<number, number>();
  for (let i = 0; i < n; i++) loc.set(walkCells[i], i);
  const isBoxCell = new Uint8Array(n);
  for (const c of boxCells) isBoxCell[loc.get(c) as number] = 1;
  const isGoal = new Uint8Array(n);
  for (const c of goals) isGoal[loc.get(c) as number] = 1;

  // Neighbours within the walkable cells, per direction.
  const nbr = new Int32Array(n * 4).fill(-1);
  for (let i = 0; i < n; i++) {
    const cell = walkCells[i];
    const cx = cell % g.w;
    const cy = (cell / g.w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
      const j = loc.get(ny * g.w + nx);
      if (j !== undefined) nbr[i * 4 + d] = j;
    }
  }

  // Static box distances over box cells only (ignores other boxes/player).
  const dist = new Int16Array(n * n).fill(-1);
  for (let s = 0; s < n; s++) {
    if (!isBoxCell[s]) continue;
    dist[s * n + s] = 0;
    const queue = [s];
    for (let head = 0; head < queue.length; head++) {
      const c = queue[head];
      const dc = dist[s * n + c];
      for (let d = 0; d < 4; d++) {
        const m = nbr[c * 4 + d];
        if (m < 0 || !isBoxCell[m] || dist[s * n + m] !== -1) continue;
        dist[s * n + m] = dc + 1;
        queue.push(m);
      }
    }
  }

  // Dead squares: reverse-pull reachability from the goals. A box cell is
  // live when a box there can still be pushed onto a goal; a box on a
  // non-live cell can never reach a goal, so such states are pruned.
  const live = new Uint8Array(n);
  const liveQueue: number[] = [];
  for (let i = 0; i < n; i++) {
    if (isGoal[i]) {
      live[i] = 1;
      liveQueue.push(i);
    }
  }
  for (let head = 0; head < liveQueue.length; head++) {
    const c = liveQueue[head];
    for (let d = 0; d < 4; d++) {
      const p = nbr[c * 4 + (d ^ 1)]; // box at p pushed in direction d lands on c
      if (p < 0 || !isBoxCell[p] || live[p]) continue;
      const stand = nbr[p * 4 + (d ^ 1)]; // player stands here to push
      if (stand < 0) continue;
      live[p] = 1;
      liveQueue.push(p);
    }
  }

  const nb = boxes.length;
  const goalLocs = goals.map((c) => loc.get(c) as number);
  const boxLocs = boxes.map((c) => loc.get(c) as number);
  const startPlayer = loc.get(playerStart) as number;

  // Cheapest-assignment heuristic: with few boxes, try every ordering of the
  // goals and take the cheapest total static distance (admissible: one push
  // moves one box by one square). With many boxes the orderings explode, so
  // fall back to the sum of each box's cheapest distance to any goal, which
  // is still admissible (each box must at least reach its own nearest goal).
  // Either way an unreachable box (no finite distance) means a dead state.
  const perms: number[][] = [];
  if (nb <= 5) {
    const buildPerm = (cur: number[], rest: number[]): void => {
      if (rest.length === 0) {
        perms.push(cur);
        return;
      }
      for (let i = 0; i < rest.length; i++) {
        buildPerm([...cur, rest[i]], [...rest.slice(0, i), ...rest.slice(i + 1)]);
      }
    };
    buildPerm([], goalLocs.map((_, i) => i));
  }

  const heuristic = (bl: number[]): number => {
    if (perms.length > 0) {
      let best = Infinity;
      for (const perm of perms) {
        let sum = 0;
        for (let i = 0; i < nb; i++) {
          const d = dist[bl[i] * n + goalLocs[perm[i]]];
          if (d < 0) {
            sum = Infinity;
            break;
          }
          sum += d;
        }
        if (sum < best) best = sum;
      }
      return best;
    }
    let sum = 0;
    for (let i = 0; i < nb; i++) {
      let best = Infinity;
      for (let j = 0; j < nb; j++) {
        const d = dist[bl[i] * n + goalLocs[j]];
        if (d >= 0 && d < best) best = d;
      }
      if (best === Infinity) return Infinity;
      sum += best;
    }
    return sum;
  };

  // Player BFS over walk cells minus boxes: returns the visited set.
  const boxSetOf = (bl: number[]): Uint8Array => {
    const s = new Uint8Array(n);
    for (const b of bl) s[b] = 1;
    return s;
  };
  const reachable = (from: number, boxSet: Uint8Array): Uint8Array => {
    const seen = new Uint8Array(n);
    const queue = [from];
    seen[from] = 1;
    for (let head = 0; head < queue.length; head++) {
      const c = queue[head];
      for (let d = 0; d < 4; d++) {
        const m = nbr[c * 4 + d];
        if (m < 0 || seen[m] || boxSet[m]) continue;
        seen[m] = 1;
        queue.push(m);
      }
    }
    return seen;
  };
  const normalize = (from: number, boxSet: Uint8Array): number => {
    const seen = reachable(from, boxSet);
    for (let i = 0; i < n; i++) if (seen[i]) return i;
    return from;
  };

  // Frozen boxes: a box is frozen when every push direction is blocked by a
  // wall or by a box that is itself frozen (least fixpoint). A frozen box
  // off a goal can never move, so the state is dead.
  const frozenDead = (bl: number[]): boolean => {
    const frozen = new Uint8Array(n);
    let changed = true;
    while (changed) {
      changed = false;
      for (const b of bl) {
        if (frozen[b]) continue;
        let allBlocked = true;
        for (let d = 0; d < 4; d++) {
          const target = nbr[b * 4 + d];
          const stand = nbr[b * 4 + (d ^ 1)];
          const targetBlocked = target < 0 || !isBoxCell[target] || frozen[target] === 1;
          const standBlocked = stand < 0 || frozen[stand] === 1;
          if (!targetBlocked && !standBlocked) {
            allBlocked = false;
            break;
          }
        }
        if (allBlocked) {
          frozen[b] = 1;
          changed = true;
        }
      }
    }
    for (const b of bl) {
      if (frozen[b] && !isGoal[b]) return true;
    }
    return false;
  };

  // Pack a state (sorted box locals + player local) into one number.
  // Five base-256 slots: player, then boxes, padded with 255 (no cell index
  // can reach 255: levels are at most 15x17).
  const pack = (bl: number[], player: number): number => {
    let k = player;
    for (let i = bl.length - 1; i >= 0; i--) k = k * 256 + bl[i];
    for (let i = bl.length; i < 4; i++) k = k * 256 + 255;
    return k;
  };
  const unpack = (key: number): { bl: number[]; player: number } => {
    const slots: number[] = [];
    let t = key;
    for (let i = 0; i < 4; i++) {
      slots.push(t % 256);
      t = Math.floor(t / 256);
    }
    return { bl: slots.slice(4 - nb), player: t };
  };

  // Binary heap ordered by (f, h): smallest f first, ties prefer smaller h.
  const hf: number[] = [];
  const hg: number[] = [];
  const hk: number[] = [];
  const heapLess = (i: number, j: number): boolean => {
    if (hf[i] !== hf[j]) return hf[i] < hf[j];
    return hf[i] - hg[i] < hf[j] - hg[j];
  };
  const heapSwap = (i: number, j: number): void => {
    const tf = hf[i];
    hf[i] = hf[j];
    hf[j] = tf;
    const tg = hg[i];
    hg[i] = hg[j];
    hg[j] = tg;
    const tk = hk[i];
    hk[i] = hk[j];
    hk[j] = tk;
  };
  const heapPush = (f: number, gv: number, key: number): void => {
    hf.push(f);
    hg.push(gv);
    hk.push(key);
    let i = hf.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(i, p)) break;
      heapSwap(i, p);
      i = p;
    }
  };
  const heapPop = (): [number, number, number] => {
    const topF = hf[0];
    const topG = hg[0];
    const topK = hk[0];
    const lastF = hf.pop() as number;
    const lastG = hg.pop() as number;
    const lastK = hk.pop() as number;
    if (hf.length > 0) {
      hf[0] = lastF;
      hg[0] = lastG;
      hk[0] = lastK;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < hf.length && heapLess(l, m)) m = l;
        if (r < hf.length && heapLess(r, m)) m = r;
        if (m === i) break;
        heapSwap(i, m);
        i = m;
      }
    }
    return [topF, topG, topK];
  };

  const startBoxes = [...boxLocs].sort((a, b) => a - b);
  const startBoxSet = boxSetOf(startBoxes);
  const startPlayerNorm = normalize(startPlayer, startBoxSet);
  const startKey = pack(startBoxes, startPlayerNorm);
  const startH = heuristic(startBoxes);
  if (startH === Infinity) return null;

  const bestG = new Map<number, number>();
  const parent = new Map<number, { prev: number; box: number; dir: number }>();
  bestG.set(startKey, 0);
  heapPush(startH, 0, startKey);

  let expansions = 0;
  while (hf.length > 0) {
    const [, gv, key] = heapPop();
    if (gv !== bestG.get(key)) continue;
    const { bl, player } = unpack(key);
    if (bl.every((b) => isGoal[b])) {
      // Reconstruct the push list.
      const pushes: PushStep[] = [];
      let cur = key;
      while (parent.has(cur)) {
        const p = parent.get(cur) as { prev: number; box: number; dir: number };
        pushes.push({ box: walkCells[p.box], dir: p.dir });
        cur = p.prev;
      }
      pushes.reverse();
      return pushes;
    }
    expansions++;
    if (expansions > MAX_EXPANSIONS) return null;
    const boxSet = boxSetOf(bl);
    const reach = reachable(player, boxSet);
    for (let bi = 0; bi < nb; bi++) {
      const b = bl[bi];
      for (let d = 0; d < 4; d++) {
        const target = nbr[b * 4 + d];
        if (target < 0 || !isBoxCell[target] || boxSet[target]) continue;
        const stand = nbr[b * 4 + (d ^ 1)];
        if (stand < 0 || !reach[stand]) continue;
        const nextBoxes = [...bl];
        nextBoxes[bi] = target;
        nextBoxes.sort((x, y) => x - y);
        if (!live[target] && !isGoal[target]) continue;
        if (nextBoxes.some((x) => !live[x])) continue;
        if (frozenDead(nextBoxes)) continue;
        const h = heuristic(nextBoxes);
        if (h === Infinity) continue;
        const nextBoxSet = boxSetOf(nextBoxes);
        const nextPlayer = normalize(b, nextBoxSet);
        const nextKey = pack(nextBoxes, nextPlayer);
        const ng = gv + 1;
        if (bestG.has(nextKey) && (bestG.get(nextKey) as number) <= ng) continue;
        bestG.set(nextKey, ng);
        parent.set(nextKey, { prev: key, box: b, dir: d });
        heapPush(ng + h, ng, nextKey);
      }
    }
  }
  return null;
}

// ---------------------------------------------------------------------------
// Solving one level
// ---------------------------------------------------------------------------

function solveWithRooms(g: Grid, rooms: RoomSpec[]): string | null {
  // A room whose goal sits on its door traps the player inside once solved,
  // so such rooms go last (in practice no goal sits on a door).
  const doorBlocked = (r: RoomSpec): boolean => r.goals.includes(r.door);
  const ordered = [...rooms].sort((a, b) => (doorBlocked(a) ? 1 : 0) - (doorBlocked(b) ? 1 : 0));
  const boxAt = new Uint8Array(g.w * g.h);
  for (const b of g.boxes) boxAt[b] = 1;
  let player = g.player;
  let moves = '';
  for (const room of ordered) {
    const pushes = pushAStar(g, [...room.cells, room.entry], room.cells, room.goals, room.boxes, room.entry);
    if (pushes === null) return null;
    const toEntry = walkPath(g, player, room.entry, boxAt);
    if (toEntry === null) return null;
    moves += toEntry;
    player = room.entry;
    for (const p of pushes) {
      const step = DY[p.dir] * g.w + DX[p.dir];
      const stand = p.box - step;
      const path = walkPath(g, player, stand, boxAt);
      if (path === null) return null;
      moves += path;
      moves += CH[p.dir];
      boxAt[p.box] = 0;
      boxAt[p.box + step] = 1;
      player = p.box;
    }
  }
  return moves;
}

function solveGlobal(g: Grid): string | null {
  // Whole-level push search is only feasible for a handful of boxes; for a
  // big level whose rooms could not be split out, give up fast rather than
  // burn the run's time budget on a hopeless search.
  if (g.boxes.length > 6) return null;
  const floor: number[] = [];
  for (let i = 0; i < g.w * g.h; i++) if (!g.wall[i]) floor.push(i);
  const goals: number[] = [];
  for (let i = 0; i < g.w * g.h; i++) if (g.goal[i]) goals.push(i);
  const pushes = pushAStar(g, floor, floor, goals, g.boxes, g.player);
  if (pushes === null) return null;
  const boxAt = new Uint8Array(g.w * g.h);
  for (const b of g.boxes) boxAt[b] = 1;
  let player = g.player;
  let moves = '';
  for (const p of pushes) {
    const step = DY[p.dir] * g.w + DX[p.dir];
    const stand = p.box - step;
    const path = walkPath(g, player, stand, boxAt);
    if (path === null) return null;
    moves += path;
    moves += CH[p.dir];
    boxAt[p.box] = 0;
    boxAt[p.box + step] = 1;
    player = p.box;
  }
  return moves;
}

function solve(level: string): string {
  const g = parseLevel(level);
  const rooms = decompose(g);
  if (rooms !== null) {
    const moves = solveWithRooms(g, rooms);
    if (moves !== null && replayPushes(g, moves) >= 0) return moves;
  }
  const gt = transposeGrid(g);
  const roomsT = decompose(gt);
  if (roomsT !== null) {
    const movesT = solveWithRooms(gt, roomsT);
    if (movesT !== null) {
      const moves = transposeMoves(movesT);
      if (replayPushes(g, moves) >= 0) return moves;
    }
  }
  const moves = solveGlobal(g);
  if (moves !== null && replayPushes(g, moves) >= 0) return moves;
  return '';
}

// ---------------------------------------------------------------------------
// Main loop (kept from the starter)
// ---------------------------------------------------------------------------

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
