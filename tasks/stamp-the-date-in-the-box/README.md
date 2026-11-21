# Date-stamp a delivery note with a self-inking stamp: the date upright, square in the box, pressed to the bottom, once

**Instruction:** "Date-stamp the delivery note."  **Category:** Societal relatability.  **Run limit:** 700 steps (35 s at 20 Hz).

## Why this task

Self-inking date stamps (Trodat 4810/Printy-type: a frame, a handle on a spring, a rubber die that flips from the ink
pad onto the paper at the bottom of the stroke) are on desks in post rooms, goods-in bays, surgeries, libraries,
schools and offices everywhere, and "date-stamp it when it comes in" is an ordinary instruction for delivery notes,
invoices and letters. Using one well takes three things anyone who has stamped forms knows:

1. turn it so the date will read the right way up (the window on the handle shows the date as it will print);
2. set the foot down square inside the box;
3. press the handle all the way down and let it spring back: a half press leaves no print (the die never reaches
   the paper), a tilted press smudges it, and a second press spoils the form.

For a robot this is: grip a 3.6 cm-wide handle, lift and turn the stamp a quarter turn so the date reads correctly,
place a 6.4 x 3.2 cm foot inside a 7.4 x 4.2 cm box (5 mm to spare each way), press a sprung handle through its full
15 mm without tilting the stamp, release, and stand the stamp back on the desk.

## Scene (positions in cm from the midpoint between the arm bases; x forward, y left; heights above the table top)

- **Delivery note:** A5 (14.8 x 21 cm) paper, loose on the desk, centred at x 37, y 12: a blue header band, five
  printed lines, and an empty black-outlined "date received" box 4.2 cm (x) by 7.4 cm (y) centred at x 34.5, y 7.5.
- **Date stamp:** standing on the desk at x 24, y 27, its long side along x: grey frame 6.4 x 3.2 cm with an inked
  die (5.6 x 2.6 cm) in its foot, black handle cap 6.8 x 3.6 cm on top, 7.2 cm tall. The cap carries a white window
  with the date 21 11 26 as it will print, reading away from the robot as the stamp stands at the start; to print it upright
  in the box (reading left to right, as the robot sees it) the stamp has to be turned a quarter turn clockwise.

## Scoring

The handle's travel into the frame is read every step. The imprint is made when the travel reaches 13 of its 15 mm
(the die meets the paper in the last 2 mm); at that moment the scorer records where the foot stands on the note, how
it is turned relative to the note and how far it is tilted.

- An imprint is **smudged** if the stamp is tilted more than 4 degrees (or the foot is not on the paper),
  **outside** if its centre is more than 7 mm from the box's centre along either axis (the print then still lies inside the outline), **upside down** if the stamp is
  turned 180 degrees (within 8), **askew** if turned otherwise more than 8 degrees, and **good** otherwise.
- Stamping a second time (the handle back up past 5 mm and down again) spoils the note: score 0.
- **1.0:** exactly one good imprint, and the stamp then let go and standing upright on the desk or the note, no pad
  tip within 3 cm of it, held for 1 s (success).
- Otherwise partial credit: **0.5** for one good imprint with the stamp not yet put down; **0.25** for one clean
  imprint that is upside down, askew or outside the box; **0** for no imprint, a smudged one, or two.

On a real setup the same rules are read from the paper afterwards (where the print is, which way up, whether it is
complete or smudged, how many) and an overhead video (the stamp put down and let go).

## Fidelity

- The stamp is built with task-local `bodykit.py`: a loose 45 g frame and a loose 30 g handle joined by a sprung
  slider with 15 mm of travel (0.4 N at rest, 1.6 N at the bottom). A real self-inking stamp needs roughly 15 to
  30 N; the spring here is lighter so the shared grippers are not loaded, and the stroke is the same.
- The flipping die and the ink are not modelled: the imprint is the foot's pose at the bottom of the stroke. The
  date window is printed on the cap.
- The note is a loose 5 g sheet 1 mm thick.

## Scripted plans (left arm)

- **demo:** grip the cap, lift, turn the stamp a quarter turn clockwise on the way to the box, set the foot down in
  the box, press the handle down while still gripping it (commanded 18 mm; it bottoms at 15 mm), let it
  spring back, lift it off and stand it back on the desk beside the note.
- **naive:** the same, but the cap pressed only 8 mm: no imprint.
- **unsafe:** pressed fully, but the stamp turned a quarter turn the other way: the date printed upside down.

## Results

| Run | Score | Notes |
|---|---|---|
| Scripted demo (Isaac Sim, 21 Nov 2026, dev node) | 1.0 | success at step 344; imprint 0.3 mm and 3.5 mm from the box's centre, turned -88 degrees, tilt 0.2 degrees |
| Naive | 0.0 | handle pressed 6.6 mm: no imprint |
| Unsafe | 0.25 | one clean imprint, upside down (turned +85 degrees) |
| Dream-1 | not run yet | |

Offline tests: `pytest test_task.py` (10 tests).

## Run it

```
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \
  --instruction "Date-stamp the delivery note."
```
