# Dial 123 on a rotary phone: a finger in each digit's hole, round to the finger stop, out, and let the dial run back

**Instruction:** "Dial 123 on the phone."  **Category:** Societal relatability.  **Run limit:** 800 steps (40 s at 20 Hz).

## Why this task

Rotary-dial phones are not just museum pieces: reproductions of the GPO 746 (the British Post Office's standard phone
from 1959; 1.0 kg, 21.5 x 16 cm) are on sale new at UK and US retailers, and rotary phones are familiar in homes,
hotels, films and escape rooms. Dialling one is a movement most adults know and many children now find puzzling:

1. put one finger into the hole for the digit;
2. turn the finger wheel clockwise until the finger meets the metal finger stop;
3. take the finger out and let the wheel run back by itself; then the next digit.

The digit is sent while the wheel runs back, as one break in the line for each 30 degrees of turn, at 10 pulses a
second set by the dial's governor. As britishtelephones.com ("How does a British dial work") puts it, the finger plate
"is rotated by the finger from the selected digit or letter to the finger stop and then allowed to return unimpeded to
the normal position"; "the governor accounts for the resistance experienced if the return rotation of the dial is
speeded-up by hand". So the two everyday ways to get a wrong number are physical: stopping short of the finger stop
(fewer pulses: a smaller digit, or none), and riding the wheel back with the finger (the pulse timing is upset).

For a robot this needs one finger put into a 1.5 cm hole, a controlled arc of 60 to 120 degrees about the dial's
centre against a spring, with the finger turning to stay in the hole, until a hard stop it must feel, and a clean
release, three times, in the right holes. 123 is the UK speaking clock.

## Scene (positions in cm from the midpoint between the arm bases; x forward, y left; heights above the table top)

- **Phone:** ivory, base 16 x 21.5 cm, 6 cm tall, raised back with the cradle; 1.0 kg, loose on rubber feet,
  centred at x 30, y 16. Hook-switch plungers up (off the hook).
- **Dial:** centred at x 27.2, y 16, top 6.7 cm up. Black finger wheel 10 cm across, 4 mm thick, ten holes (gaps
  1.5 cm across between spokes, centres 3.6 cm from the middle) over a white number ring with the digits printed on it,
  one seen through each hole. Seen from the robot: 1 at about 1:30 on a clock face, each next digit 30 degrees anticlockwise,
  0 last at about 4:30; the steel finger stop reaches in over the wheel at about 4 o'clock.
- **Handset:** off the hook, lying on the desk to the phone's right (x 34, y -3), its cord running to the phone.

## Scoring

The wheel's turn (degrees, clockwise from rest) is read every control step.

- A turn starts once the wheel is 10 degrees round and ends when it is back within 4 degrees of rest. It has then sent
  floor((largest turn - 15)/30) pulses; 10 pulses is 0. Turned to the stop, that is 1 for about 60 degrees, 2 for
  about 90, 3 for about 120.
- A finger in a hole (a pad tip below the wheel's top face, over the wheel) for 0.3 s or more while the wheel runs
  back garbles that digit.
- **Score 0 for the whole trial** if a digit is garbled, if the digits sent so far are not the start of 123 (a wrong
  or extra digit), or if the phone is pushed 2 cm across the desk.
- **At the end: 1.0** if exactly 123 was sent, the wheel is at rest and no pad tip is within 3 cm above the dial
  (success: that held for 1 s). Otherwise, partial credit for progress: **0.25 per correct digit sent** (0.25 for
  "1", 0.5 for "12", 0.75 for "123" with the finger still over the dial). Possible scores: 0, 0.25, 0.5, 0.75, 1.0.
  A wrong number never earns credit: any wrong, extra or garbled digit is 0.

On a real setup the same rules are read from an overhead video (finger to the stop, out, free run-back) and the
phone's own dialling: a line simulator or analogue telephone adapter shows the digits received.

## Fidelity

- The phone, the wheel and the handset are built with task-local `bodykit.py` (one change: a hinge may have a velocity
  target). The wheel is a loose 40 g body on a hinge limited to 0 to 340 degrees, whose damped velocity drive stands
  in for the wound spring and the governor: free, it runs back at 300 degrees/s (10 pulses a second); held or turned,
  it pushes back with about 0.03 N m (some 0.8 N at a hole).
- The holes are the gaps between eleven spokes (the 60 degrees under the stop is solid); the finger stop is a rigid
  part of the phone.
- The dial lies flat; on a GPO 746 it leans back about 20 degrees. The ring's digits are plain seven-segment figures
  (no letters).
- The handset and its cord are fixed and visual only. There is no electrical model: the scorer decodes the pulses
  from the wheel's turn as a real dial does.

## Scripted plans (left arm, closed fingers used as one finger)

The closed pads' tip is about 15 x 8 mm, so the plan turns the wrist to keep the 15 mm side along the hole's radius
and the 8 mm side across the 1.5 cm gap, and follows the hole round in 10-degree steps.

- **demo:** for each of 1, 2, 3: finger into the hole (pad tips 3.5 mm below the wheel's top), round until the stop
  halts it, straight out, wait for the run-back.
- **naive:** the same, but each turn stopped 25 degrees short of the finger stop.
- **unsafe:** each turn to the stop, but the finger follows the wheel back down instead of coming out.

## Results

| Run | Score | Notes |
|---|---|---|
| Scripted demo (Isaac Sim, 21 Nov 2026; dev node and L40S scoring run agree) | 1.0 | success at step 431; turns 56, 85, 115 degrees |
| Naive | 0.5 | 1 sent nothing, then 1 and 2: "12" |
| Unsafe | 0.0 | digit 1 ridden back with the finger (30 steps) |
| Dream-1 (flash, 320 x 176, seed 1, 800 steps) | 0.0 | brought the left gripper over the phone's near edge and hovered there; never put a finger in a hole (no digit sent) |

Offline tests: `pytest test_task.py` (11 tests: pulse decoding, hole layout, the 1 s hold, partial credit, short turns, wrong and
extra digits, riding back, finger left over the dial, phone pushed, plans).

## Run it

```
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \
  --instruction "Dial 123 on the phone."
```
