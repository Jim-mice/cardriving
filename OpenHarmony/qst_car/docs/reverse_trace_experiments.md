# Reverse trace experiments

## Reverse V1 on-car result

The first reverse trace experiment used continuous reverse translation and
differential correction at the same time.

- On `sensor=00`, `L=-100`, `R=-100` tracked a straight line stably for an
  extended period.
- At approximately `ms=12920`, the physical right sensor detected black:
  `sensor=01`.
- V1 then held `L=-60`, `R=-120` for about 500 ms.
- It did not return to `00`; the state evolved from `01` to `11`.
- `11` caused the intended safe stop.

Therefore the continuous “reverse translation plus differential correction”
candidate failed on the car. The V1 code and `REVERSE` / `REVERSEDEBUG`
telemetry are retained as experimental evidence, but V1 is not the active
test mode.

## Reverse V2 candidate

Reverse V2 separates reverse translation from heading correction:

- `00`: reverse straight using `-100/-100`.
- `01` or `10`: stop reverse translation and use a candidate in-place turn.
- Two stable `00` samples are required before a short settling stop and a
  return to straight reverse.
- Alignment is limited to 500 ms; timeout or `11` latches a safe stop.

This is an experimental candidate and has **not** yet been verified on the
car. It is not connected to competition DEADEND recovery.

## Reverse V2 first on-car result

- `00` with `L=-100`, `R=-100` continued straight reverse normally.
- The first obvious deviation was `sensor=10` (physical left sensor on black).
- The first V2 candidate used `sensor=10 -> L=-90, R=+90`.
- It could briefly produce `10 -> 00`, but BACK repeatedly soon observed `10`
  again.
- Alignment duration grew across repetitions, then `10 -> 11` caused the
  intended safety stop.

This suggests that a momentary sensor return to `00` may not have removed the
vehicle heading error; it is a hypothesis, not a proven cause. The next
single-variable experiment swaps only the in-place yaw direction:

- `sensor=10` (left sensor hit) -> yaw right: `L=+90`, `R=-90`.
- `sensor=01` (right sensor hit) -> yaw left: `L=-90`, `R=+90`.

Reverse speed, alignment speed magnitude, timeout, settling time, debounce,
and clear-sample count remain unchanged. This second V2 candidate is pending
on-car verification.

## Reverse V2 second on-car result

- `00` with `L=-100`, `R=-100` remained a stable straight reverse command.
- At `ms=6700`, `sensor=10` appeared.
- V2 used the swapped in-place candidate `L=+90`, `R=-90` (yaw right).
- From `ms=6700` to `ms=7210`, the sensor remained `10` for about 510 ms;
  `consecutive_10` reached 18.
- The ALIGN timeout then issued the intended safety stop.

This straight-line result does not support the “stop translation, rotate in
place, wait for `00`” candidate for the current sensor geometry. No V2 speed,
timeout, settling, debounce, or clear-sample parameter was retuned.

## Reverse V3 hypothesis

Reverse V3 returns to simultaneous reverse translation and correction, but
uses the yaw opposite to forward TRACE:

- `00` -> `-100/-100`.
- `10` (left sensor hit) -> `-60/-120`, reverse trajectory correction left,
  chassis yaw right.
- `01` (right sensor hit) -> `-120/-60`, reverse trajectory correction right,
  chassis yaw left.
- `11` -> latched stop.

V3 intentionally does not use V2 ALIGN, SETTLE, or ALIGN timeout states.

## Reverse V3 latest on-car result

Reverse V3 was verified on a straight reverse run with the following table:

- `00` -> `-100/-100`.
- `10` -> `-60/-120`, yaw right.
- `01` -> `-120/-60`, yaw left.

The first two `10` corrections returned to `00`. During a later sustained
`10`, visual observation showed the chassis first recover from `\\` to `|`,
then continue past centre to `/` and nearly `-` while the left sensor still
reported black. Therefore the V3 yaw direction is effective; the failure is
not treated as evidence that its direction or correction magnitude is wrong.
The front-mounted sensors are trailing feedback while reversing, so their
reported `10` can persist after the chassis heading has recovered. Continuous
correction consequently oversteers: `\\ -> | -> / -> -`.

## Reverse V4 hypothesis (not yet on-car verified)

Reverse V4 retains the V3 motor commands but changes only the control timing:

- `00`: BACK, `L=-100`, `R=-100`.
- `10` (physical left on black): fixed 100 ms yaw-RIGHT pulse,
  `L=-60`, `R=-120`.
- `01` (physical right on black): fixed 100 ms yaw-LEFT pulse,
  `L=-120`, `R=-60`.
- Every pulse is followed unconditionally by 150 ms straight reverse
  (`-100/-100`) before the next stable-sensor observation can start a pulse.
- `11` has highest priority and latches STOP.
- Four consecutive pulses triggered by the same side without a `00` or an
  opposite-side observation latch STOP as a safety limit.

The 100 ms pulse and 150 ms probe are initial test parameters only. They are
not calibrated and must not be represented as a verified result.

## Reverse V5 latest stable on-car outcomes

Two repeatable outcomes were observed. With an overly large fixed S-shift the
vehicle crossed beyond the line; both probes returned white and static
`sensor=00` was incorrectly reported as recenter success. With an insufficient
shift, the original probe stayed on black until the safety stop. Static `00`
is therefore ambiguous and does not prove that the line is between the probes.

## Reverse V6 first on-car result

V6 eliminated the V5 false success caused by a static final `00`. The stable
failure is now `origin=10/01 -> heading -> lateral -> SHIFT_BACK`, with the
original side remaining active through the 500 ms search window, followed by
`LATERAL_CAPTURE_TIMEOUT` and STOP. Before expanding that window, the
recovery-local previous-sensor transition and control cadence must be verified.

## Reverse V6 hypothesis (not yet on-car verified)

V6 records the known-side history and captures the first debounced transition
from `10` or `01` to `00`, stopping lateral motion immediately. Heading and
lateral yaw-out durations are maxima, not mandatory durations; shift-back has
a 500 ms safety maximum. Yaw restore is symmetric to the executed yaw-out
duration, followed by a short verification run.

## Reverse V4 latest stable on-car result

Repeated straight-line tests showed that a V4 heading pulse can recover the
chassis heading from a deviation such as `| -> \\` back to approximately
parallel with the line. With `sensor=10`, however, the chassis may remain
laterally offset while the left probe stays on black. Straight reverse then
keeps the chassis parallel and the sensor remains `10`; repeated same-side
pulses eventually reach `PULSE_LIMIT`. The remaining error is therefore
primarily lateral position, not heading.

## Reverse V5 hypothesis (not yet on-car verified)

V5 keeps the verified heading pulse/probe values, then treats a first
same-side probe result as a possible lateral offset. It applies a bounded
S-shaped action: heading yaw-out, straight reverse shift, opposite yaw-back,
and a short recheck. The purpose is to move the line back between the two
probes (`sensor=00`) while restoring a parallel heading. No V5 direction or
parameter has been validated by visual on-car observation yet.
## Reverse V6 second on-car result

V6 confirmed that even `origin -> first stable 00` can be an outside-line
state. A second stable outcome remained on the original side and timed out.

## Reverse V7 hypothesis (not yet on-car verified)

V7 requires edge-to-edge evidence: `10 -> 00 -> 01` or `01 -> 00 -> 10`, then
returns from the confirmed opposite edge to `00` before final yaw restore and
verification. A single static or first-transition `00` is never sufficient.

## Reverse V7 first execution result

The first V7 run cannot be used to evaluate edge-to-edge behavior because
`SWEEP_TO_OPPOSITE` still sent the heading differential command instead of
straight `-100/-100`. The observed `00` timeout and `11` stop outcomes may
both result from that incorrect execution. The corrected sweep command has
not yet been verified on the car.
## Reverse V7 corrected-entry experiment (not yet on-car verified)

After correcting the sweep motor command to straight `-100/-100`, the car
still held `sensor=10` for about 1.45 s and reached `OPPOSITE_EDGE_TIMEOUT`.
Visual inspection showed that the chassis was already nearly parallel with the
line when straight sweep began, so it had little lateral velocity. The prior
structure effectively applied the same yaw for about 100 ms of heading plus
80 ms of yaw-out (roughly 180-200 ms with loop quantization). The next run uses
one 60 ms `SWEEP_ENTRY_YAW` followed immediately by straight sweep. The
edge-to-edge criterion remains unchanged and is not yet validated with this
corrected entry.
## Reverse V7 entry-chain correction

The previous V7 run still showed about 200 ms from `HEADING_START` to
`SWEEP_START` because the 100 ms heading phase was followed by the 60 ms entry
phase. V7 now enters the single 60 ms `SWEEP_ENTRY_YAW` phase directly from
`REV7_BACK`; the old heading states remain only as preserved experiment code.
The next run should show `SWEEP_ENTRY_YAW_START` followed by `SWEEP_START` in
approximately 60-90 ms, with straight `-100/-100` sweep. No V7 parameter was
otherwise changed.
## Corrected V7 60 ms on-car result

The single 60 ms entry yaw was executed and the sweep command was genuinely
straight `-100/-100`. The original sensor remained `10` (or symmetrically
`01`) throughout the approximately 1.5 s sweep; no center-window transition
occurred before the safety stop. This indicates insufficient lateral sweep
component. The next single-variable trial uses `V7_SWEEP_ENTRY_YAW_MS=90U`.

## Corrected V7 90 ms on-car result

The corrected single-entry experiment was repeated from both physical edges.

- From `origin=10`, V7 issued the 90 ms entry yaw-RIGHT command
  (`L=-60`, `R=-120`), then the straight sweep command (`L=-100`,
  `R=-100`). The sensor remained `10` until the 1500 ms opposite-edge
  timeout. Visual observation showed the chassis roughly parallel to the line
  while the original (left) probe remained on black.
- From `origin=01`, the mirror experiment issued the 90 ms entry yaw-LEFT
  command (`L=-120`, `R=-60`), then the same straight sweep. The sensor
  remained `01` until the 1500 ms timeout, with the right probe remaining on
  black and the chassis similarly parallel to the line.

The mirrored results indicate that 90 ms is still insufficient to establish a
useful lateral sweep component. The next experiment changes only the single
entry-yaw duration from 90 ms to 120 ms; motor commands, the 1500 ms sweep
limit, and edge-to-edge confirmation remain unchanged. This is a hypothesis
awaiting on-car verification.

## Reverse V7 60/90/120 ms observation and V8 hypothesis

Across multiple corrected V7 entry-yaw trials (60 ms, 90 ms, and 120 ms), the
most frequent stable physical result was persistent `10` or persistent `01`:
one probe remained on the black line while the chassis was approximately
parallel to the line and continued reversing along it. V7 eventually stopped
only because its edge-to-edge timeout and safety rules did not accept this
state as reacquired tracking.

V5/V6/V7 also established that static `00` is not reliable single-sample
evidence of centered reverse tracking: it can mean either a center window or
that both probes have completely left the line. V8 therefore tests a different
hypothesis: persistent contact on one physical side (`10` or `01`) may be a
usable reverse-tracking observable on a straight line.

V8 uses one 90 ms entry yaw, then confirms persistent original-side contact
for 300 ms before accepting an edge lock. It does not force center
reacquisition, does not implement edge-following turns, and remains unverified
by on-car testing.

## Reverse V8 passive edge-lock baseline

The right-edge straight-line test established `EDGE_LOCKED` from stable
`sensor=01` at approximately 6650 ms after the 90 ms entry and 300 ms
candidate confirmation. It then remained `01` with straight reverse
`L=-100`, `R=-100` for about 4.5 seconds, until stable `11` at about 11180 ms
caused the required `SENSOR_11_STOP`. Visual observation was a mostly parallel
reverse path with one probe following the line edge. Symmetric persistent `10`
behavior has also been observed on the left side.

This supports persistent `10`/`01` as a reverse edge-lock observable, but the
passive straight command cannot suppress slow lateral drift into `11` or `00`.
V8.1 therefore keeps the edge state as the target and tests one 30 ms
bang-bang correction pulse using raw GPIO early warning: raw `00` pulses
toward the original edge and raw `11` pulses away from it. Stable `11` remains
an immediate stop. V8.1 is awaiting on-car verification.

## Reverse V8.1 on-car correction observations

The passive RIGHT edge lock was repeatedly established after the normal 90 ms
entry and 300 ms candidate phase. The raw early-warning directions now have
direct on-car evidence:

- A RIGHT-edge gap (`stable=00`) followed by the toward-line pulse
  (`yaw=LEFT`, `L=-120`, `R=-60`) could recover the original `01` edge.
- A RIGHT-edge raw `11` followed by the away-line pulse (`yaw=RIGHT`,
  `L=-60`, `R=-120`) could return raw sensing to `01`.

The same run also showed bang-bang hunting: raw `00` could receive a toward
pulse and subsequently reach raw `11`. A later `EDGE_LOST` stop was followed
about 200 ms later by the original `01` edge returning, so the current 300 ms
gap timeout may include dynamic motion rather than a final loss. This is an
observation, not proof; V8.1 parameters remain unchanged for the next test.
One 30 ms away pulse can correct an instantaneous raw condition but has not
yet demonstrated removal of long-term heading drift into stable `11`.
## Reverse V8.2 — simple stable-state servo (pending vehicle test)

V8.1 confirmed that its RIGHT-edge toward and away directions can restore the
instantaneous edge reading, but its candidate/gap/lock/pulse safety machine
also stopped during normal edge dynamics. A LEFT-edge run stayed at `10` for
about 3.6 s, changed to stable `11`, stopped, then returned to `10` about
120 ms later. Therefore neither `00` nor `11` is fatal to the V8.2 *armed*
reverse controller.

V8.2 uses only the existing two-sample stable state: `00` is straight
`-100/-100`; `10` remembers LEFT and boosts the right wheel `-100/-120`;
`01` remembers RIGHT and boosts the left wheel `-120/-100`. With `11`, the
controller boosts the original edge-side wheel to ride back: LEFT becomes
`-120/-100`, RIGHT becomes `-100/-120`. An initial unknown `11` remains
stopped in WAIT_CLEAR until a non-`11` state clears. V8.1 candidate/gap/lock,
raw pulses and STOP timeouts remain preserved as inactive experiment code.
V8.2 is pending vehicle verification.

## Reverse V8.2 transit/search revision (pending vehicle test)

Latest V8.2 vehicle evidence showed the useful sequence `10 -> 00 -> 01`:
the car visibly corrected from one side and slightly crossed the black line.
The failure followed `01 -> 00`, where the former code immediately restored
open-loop `-100/-100`; the car then fully left the line while the ambiguous
`00` reading was still treated as normal straight reverse.

After the first observed edge, V8.2 now treats stable `00` as edge transit,
not normal straight reverse. LEFT history keeps `-100/-120` while expecting
`01`; RIGHT history keeps `-120/-100` while expecting `10`. If the expected
edge is not seen within 500 ms, one opposite-direction search runs for at
most 800 ms. Only if both phases remain stable `00` does V8.2 stop with
`SIMPLE_LINE_LOST`. Stable `11` remains the existing nonfatal ride-back
behavior after a valid edge history. This revision is pending vehicle test.

## Reverse V8.2 finite-nudge revision (pending vehicle test)

The latest on-car run showed that the first stable LEFT edge (`10`) held
`-100/-120` continuously from roughly 4890 ms to 6520 ms, about 1.6 s. The
operator observed that only a light initial heading correction was needed;
the long differential command accumulated excessive heading and seeded the
later long-distance oscillation. This supports the hypothesis that reverse
sensor side-contact duration includes geometric delay and is not equal to the
required heading-correction duration.

The active V8.2 simple servo therefore keeps BASE=100 and BOOST=120 but makes
`10`, `01`, and valid-history `11` entry-triggered finite nudges. Each first
entry receives its existing differential command for at most 120 ms (four
30-ms control periods), then holds `-100/-100` if the same stable sensor state
persists. The same state cannot automatically rearm; it must first be left and
then re-entered. Transit remains 500 ms and search-back remains 800 ms.
This revision is pending vehicle test.

## Reverse V8.3 first finite-nudge vehicle run — HOLD apply defect

The first finite-nudge run confirmed the intended qualitative behavior for a
LEFT edge (`10`): the initial 120 ms RIGHT nudge was visibly much gentler than
the earlier approximately 1.6 s continuous differential command, without an
immediate large heading overshoot. However, after the nudge completed, the
controller entered `REV8_SIMPLE_EDGE_LEFT_HOLD` and incorrectly transmitted
`L=0 R=0`, stopping with the left probe still on the line. This invalidates the
run as an evaluation of the 120 ms nudge, BOOST=120, transit, or search
parameters. The cause is classified as a HOLD motor-apply defect: the HOLD
states were absent from the V8 simple apply switch and therefore retained its
default zero commands. The correction maps every V8.3 HOLD state to
`-100/-100`; on-car verification remains pending.

## Reverse V8.4 — paired-turn recenter experiment (pending vehicle test)

V8.3 produced some directionally gentle finite-nudge behavior, but recent
vehicle observation showed that a persistent `10` or `01` is still a dangerous
edge condition, not a suitable long-term hold target. In one LEFT case, `10`
remained for about 7.4 s after the nudge; it could later collapse either
outward to ambiguous `00` or inward to `11`. By contrast, `00` appeared easier
to maintain once the vehicle had been deliberately brought there.

V8.4 therefore removes V8.3 hold/transit/search from its active path while
retaining their source history. A stable LEFT edge (`10`) uses one paired,
symmetric shift: left (the black/1-side) slows to `-95`, right (the white/0-
side) accelerates to `-105`. A controlled `10 -> 00` is recorded as a
TRUSTED_00 candidate. The controller immediately applies the exact opposite
pair (`-105/-95`) for the measured shift duration, capped at 600 ms, then
returns to `-100/-100`. RIGHT is mirrored. This is an equal-time S-shaped
lateral recenter hypothesis, not a verified result.

The V8.4 pair actions always use one wheel at BASE-5 and the other at BASE+5;
they never use the earlier one-wheel-only `-100/-120` commands. A `11` with a
known origin runs the matching reverse paired direction rather than holding or
immediately stopping. A shift that cannot reach `00` or `11` within 600 ms
enters PAIR_ABORT at `-100/-100` for one control period and then stops for the
attended straight-line experiment. Raw GPIO remains telemetry-only; all motor
decisions use the existing two-sample stable sensor state. V8.4 awaits its
first on-car test.

## Reverse V8.4 vehicle observations and V8.5 micro-paired experiment

V8.4 confirmed that a gentle `-95/-105` LEFT-origin shift can remain at `10`
for its full 600 ms without presenting an observed emergency; stopping on that
timeout is therefore inappropriate for the attended low-level reverse line
controller. A separate LEFT run reached `10 -> 00` after about 90 ms, performed
an approximately equal opposite restore, then quickly returned to `10`. This
supports the signed paired direction but shows that one small lateral movement
may simply leave the vehicle near the same edge rather than requiring a stop
or a larger single turn.

V8.5 retains all V8.4 source and instead tests repeated micro S-pairs at a
slower base command: straight and settle are `-95/-95`; each shift or restore
uses only `-90/-100` or its mirror `-100/-90`. A shift lasts at most 60 ms and
is followed by an equal-time opposite restore, then a 60 ms straight settle.
If the current stable edge remains, a new *complete* micro pair begins; it
never appends another same-direction yaw phase. Normal `00`, `10`, `01`, and
known-history `11` do not timeout-stop in the V8.5 active path. The goal is
dynamic attitude stability with short small `/` and `\\` excursions, rather
than proving whether every ambiguous `00` is physically centered. V8.5 is
pending vehicle verification.

## Reverse V8.5 observations and V8.6 biased-micro experiment

V8.5 first showed that a LEFT-edge micro shift (`-90/-100`) could reach `00`
quickly, so the low 90/100 differential has real effect. During persistent
edge contact, however, repeated 60 ms shift plus equal 60 ms opposite restore
left each pair with approximately zero signed yaw; pair counts grew into the
teens without enough long-term correction. In stable `11`, the same paired
structure could repeat more than 50 times without adequate net escape.

V8.6 retains the 90/95/100 speed limits but makes each normal edge cycle
biased: 60 ms shift, at most 30 ms opposite restore, and 60 ms straight
settle. Persistent `10` consequently accumulates small net RIGHT correction;
persistent `01` mirrors LEFT. Stable `11` uses no opposite restore: the last
RIGHT edge receives repeated RIGHT 30 ms escape pulses separated by 60 ms
straight, while the last LEFT edge mirrors LEFT. Normal runtime 00/10/01/11
does not timeout-stop; raw GPIO remains telemetry only. This is an attended
long-straight experiment, pending vehicle validation.

## V8.6 observation classification correction

Only stable `10` and `01` carry signed steering information: they identify the
physical LEFT or RIGHT probe on black. Stable `00` and `11` are direction
ambiguous. In particular, `11` may occur during a normal reverse return from a
wrong branch into the main route, so a low-level reverse servo must not choose
LEFT or RIGHT solely from last-edge history. The V8.6 active path therefore
removes directional 11 escape. Both ambiguous states command only
`-95/-95`, without search, turn, or stop, until a new signed `10` or `01`
observation arrives. Signed edge cycles remain 60 ms shift, at most 30 ms
partial restore, and 60 ms settle; a new opposite signed observation overrides
an in-progress timed cycle immediately. This is pending on-car validation.
## Forward Trajectory Replay / Straight V1

This is a separate, attended straight-branch experiment.  It does not change
Reverse V8.6 and it is not connected to Race V2.  The vehicle starts at a
branch entrance with a stable `00`, runs the already validated forward TRACE
controller toward one terminal horizontal line, and records the actual motor
commands immediately before each STM32 transmission.

At the first debounced terminal `11`, recording stops, the vehicle stops for
300 ms, then the recorded frames are replayed in strict reverse time order.
For each frame the commands are independently negated: `Lrev=-Lfwd` and
`Rrev=-Rfwd`.  The left/right wheel order is never exchanged.  Reverse sensor
states are telemetry-only in this experiment; V8.6 and all reactive reverse
steering are bypassed.

The record buffer is fixed static RAM, 1200 frames (36 seconds at a 30 ms
control period).  Its six-byte frame is `{ int16 left, int16 right, uint8
stable_sensor, uint8 reserved }`, for 7200 static bytes.  The buffer never
wraps: if it fills, the experiment sends STOP and reports `REPLAY_BUFFER_FULL`.

This experiment tests only whether recently executed forward motor history can
serve as useful open-loop reverse trajectory memory on a straight branch.
Future experiments may compare this baseline with encoder-indexed replay or
with a small signed `10/01` feedback term; neither is present here.

### Pre-marker diagnostic capture

The first field test ended at an apparent marker before the intended terminal
line, so the marker rule is deliberately unchanged while evidence is gathered.
During forward recording, a separate static 100-frame ring retains about three
seconds of raw physical GPIO13/GPIO14 values, debounced stable sensor, previous
stable sensor, actual sent motor commands, TRACE action, and control timestamp.
On the first stable `11`, the car queues that ring for low-priority UDP output
as `REPLAY event=PRE_MARKER_HISTORY` before stopping.  It also emits
`END_MARKER_CONTEXT` with the preceding five stable states and time since the
last 00/10/01 plus the preceding stable-00 duration.  These fields are logging
only: they do not alter the single-marker rule, forward TRACE parameters, or
reverse replay.

### Marker signature V1

Field evidence showed a false replay end marker with `00 -> 10` followed by
roughly 300--340 ms of stable `10`, then `11`.  This is a normal forward
left-correction geometry, not a terminal cross-line.  Replay therefore judges
only the *entry* into stable `11`: it accepts direct `00 -> 11`, or
`00 -> 10/01 -> 11` only when the signed 10/01 preamble is at most 120 ms.
A longer signed preamble is rejected for that whole continuous 11 interval and
is passed unchanged to the existing forward `TraceControlStep()`; it does not
STOP or start replay.  The positive TRACE parameters and the replay reverse
sequence remain unchanged.
### CROSS_AND_PROBE_V1

Earlier experiments showed that first `11`, short signed preambles, moving `11` dwell of 120 ms, and a clean-straight assumption are insufficient marker classifiers. CROSS_AND_PROBE_V1 actively probes every non-11 to 11 transition: it crosses with 100/100, uses a 120 ms forward guard after an 11-to-00 exit, then scans 60/120 for 90 ms, 120/60 for 180 ms, and 60/120 for 90 ms. It records raw and stable sensors and stops with an experimental result; it never reverses.
