# Swap two parts between stations through a shared transfer strip

Two stations share a 12 cm transfer strip. Each arm reaches its own cell and the strip, not the other cell, so the
red part (left) and the blue part (right) can only change sides by being set down in the strip. Only one station
may work in the strip at a time: both arms in it at once, or the arms touching, scores 0. Each part on its new cell
scores 0.5.

## Run a trial
From this folder, on a machine with the official simulator:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the red and blue parts between the two stations." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/swap-dream

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the red and blue parts between the two stations." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/swap-demo
```

## Scoring
| Outcome | Score |
|---|---|
| Both arms' grippers inside the strip at the same moment | 0 (latched) |
| The two grippers within 5 cm of each other | 0 (latched) |
| Both parts on their new cells, let go (ends the trial after 1 s) | 1.0 |
| Per part on its new cell otherwise | 0.5 |
| Doing nothing | 0 |

## Design notes
- **The reach makes the strip shared.** At x 22 the left arm can't reach y -14 or beyond, nor the right arm y 14,
  so each cell belongs to one arm (checked with the arm's IK in `test_task.py`).
- **A swap needs a free spot.** Both cells are occupied, so one part has to wait in the strip while the other
  crosses. The demo: the right arm sets blue in the strip and goes home; the left arm sets red in the strip, takes
  blue home and leaves; the right arm takes red home.
- **What counts as "in the strip".** Any fingertip or gripper centre with y between -6 and 6 cm, at any height and
  any distance along the bench. An arm waiting at its home spot (y ±22) is well outside.
- **Related task.** This is the two-way version of the shared hatch task: there each crate moves one way, here the
  parts cross in opposite directions.

## Runtime and seeds
- The demo (both arms, taking turns) takes about 32 s of simulated time. One fixed layout: `-E object_jitter=0`.
- `test_task.py` (no GPU) checks the layout, which places each arm can reach, the null policy, the ladder, the two
  latches, one arm alone in the strip, reach, and replays the demo (1.0, the two arms' link origins never closer
  than 10 cm) and an early start (both arms in the strip, 0).
