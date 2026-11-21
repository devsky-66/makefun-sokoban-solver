# Room-decomposed push-optimal Sokoban solver

A `main.ts` entry for the [make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121)
`sokoban-solver-01` bounty: given a Sokoban level in XSB, it writes the
player's moves (`u`/`d`/`l`/`r`) that push every box onto a goal in as few
pushes as possible.

## The idea

The levels in this competition all share one shape: a one-cell-wide hallway
with one to four rooms hanging off it. Each room has a single door, its own
boxes and an equal number of goals, and a box pushed out of its door into the
hallway can never be pushed back — so every box stays in its room, and a
level's push-optimal count is the sum of its rooms' optima. The solver
exploits that directly:

1. **Decompose.** Find the hallway (the row the player starts in) and, for
   each door cell next to it, flood the room on the far side. The split is
   validated — rooms must not overlap, and together they must hold every box
   and every goal with the counts balanced — before it is trusted.
2. **Solve each room optimally.** A push-space A* search: a state is the
   sorted box cells plus the player's reachable region, successors are
   pushes, and the heuristic is the cheapest assignment of boxes to goals
   under precomputed static floor distances (admissible: one push moves one
   box by one square). Dead states are pruned with reverse-pull dead squares
   and a frozen-box fixpoint (a box whose every push direction is blocked by
   a wall or an already-frozen box, and which sits off a goal, can never
   move). The search budget is a fixed node count, never a clock, so runs are
   byte-identical.
3. **Stitch.** Plain BFS player walks connect the rooms: walk to a room's
   door, replay its pushes (walking to the square behind each box first),
   walk out to the hallway and on to the next door. A room whose goal sits
   on its door is solved last, since it traps the player inside. The finished
   move string is replayed on the level before it is returned.

If a level does not match the hallway-and-rooms shape, the solver retries
with the level transposed (a vertical hallway), then falls back to a
whole-level push A* for small levels, and otherwise writes an empty (valid,
zero-point) answer rather than an illegal one.

## Usage

```sh
# solve the practice set
mkdir -p out
MAKEFUN_INPUT_DIR=path/to/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` needs only Node 22, no dependencies, no network. It reads every
instance under `MAKEFUN_INPUT_DIR` (default `/input`) and writes
`MAKEFUN_OUTPUT_DIR/<id>/solution.txt` for each.

## Results (practice set)

| instance | rooms | optimal pushes | pushes found | points |
|----------|-------|----------------|--------------|--------|
| i001     | 1     | 13             | 13           | 1000   |
| i002     | 1     | 16             | 16           | 1000   |
| i003     | 2     | 29             | 29           | 1000   |
| i004     | 3     | 47             | 47           | 1000   |
| i005     | 4     | 60             | 60           | 1000   |

Total **1,000,000** via `grade-local` (practice), all five instances at their
push optimum, in ~0.4 s for the whole set (limit 60 s), with the two
grade-local runs reproducing byte for byte.

## License

MIT — see [LICENSE](LICENSE).
