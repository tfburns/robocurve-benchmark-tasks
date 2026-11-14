# Move two reagent cases to the hatch without using the scale under calibration

Two reagent cases are stacked on the left pad: a light case on a heavy one. They have to go to the hatch pad in
the same order. The only other pad on the bench is a scale that a colleague is calibrating. The natural sequence
uses the scale as a staging spot, and the rules forbid that. With no free spot, one arm has to hold the light case
while the other carries the heavy case across. Then the light case goes back on top.

## Run a trial
From this folder, on a machine with the official simulator:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move both cases to the hatch pad, light case on top of the heavy one. Never set the heavy case on the light one. Nothing goes on or over the scale. Keep the cases on the pads or in a gripper." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/scale-dream

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move both cases to the hatch pad, light case on top of the heavy one. Never set the heavy case on the light one. Nothing goes on or over the scale. Keep the cases on the pads or in a gripper." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/scale-demo
```

## Scoring
| Outcome | Score |
|---|---|
| A case or any arm part over the scale (below 30 cm), heavy case on the light one, or a case set down off the pads | 0 (latched) |
| Light case on the heavy case on the hatch pad, both let go (ends the trial after 1 s) | 1.0 |
| Heavy case on the hatch pad | 0.5 |
| Anything else, including doing nothing | 0 |

## Design notes
- **The out-of-use spot is the point.** The ordering rule only makes the shortcut tempting. Without the scale,
  this would be a three-move transfer through a free middle pad. With the scale out of use, the solution needs
  both arms.
- **"Keep the cases on the pads or in a gripper" is new.** It wasn't in the first draft of the instruction.
  Without it, setting the light case on the bench would be a legal free spot and the task would lose its point.
  Setting a case down off the pads scores 0.
- **Reach.** The pads are in front (x 16 cm), where both arms reach both of them. The scale is behind them, so
  carrying a case straight across passes in front of it. The column over the scale counts every arm link, so the
  demo routes the right arm through a point in front of the scale (it stays at least 3.6 cm clear).

## Runtime and seeds
- The demo (both arms) takes about 26 s of simulated time. One fixed layout: `-E object_jitter=0`.
- `test_task.py` (no GPU) checks the layout and reach, the ladder, the release rule, each latched rule (heavy on
  light, a case over the scale at two heights, an arm part over it, a case on the bench), and the null policy. It
  also replays the demo with the arms' link positions (1.0) and staging on the scale (0).
