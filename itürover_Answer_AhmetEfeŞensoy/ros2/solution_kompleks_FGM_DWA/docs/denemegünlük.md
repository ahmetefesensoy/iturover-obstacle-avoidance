# Course-completion attempts log

**Goal:** complete the whole corridor course (all gates) within the 60-second
mission, using laser + internal state only for control. One headless run per
attempt; improve planner logic/parameters between attempts; commit each loop.

**Completion criterion:** ground-truth net displacement ≥ 18.0 m (course is
~19.5 m spawn-to-end; threshold leaves wall-standoff margin — will be refined
once a run reaches the final-gate region and the end wall becomes visible in
telemetry). Measured from `SUMMARY_GT` in the run log; ground truth is
measurement-only, never control.

**Baseline going in (commit 9ac6b54):** stable no-circling runs at
14.6–15.4 m; door-commit never fires (this attempt's build replaces the FGM
blob-width trigger with the light-through-door primitive). Best historical
run: 16.4 m, one gate cleared.

---

## Attempt 1 — light-through-door primitive end-to-end

**Config:** commit 9ac6b54. Door trigger = narrow deep-arc (<0.9 m) at a
constrained face (front <1.3 m); commit gate = light within 15° of nose;
commit steering aims at light center; depth-lost/no-progress aborts.
Corridor-geometry compass + 100° hard U-turn wall (3 clean verification runs,
zero circling).

**Question to answer (user's):** does the robot SEE the gate-1 slit — i.e.
does the light arc register at the face, and does the trigger fire?

**Result:** FAILED — gt_net 13.22 m, but a diagnostic breakthrough.

- **The robot DOES see the gaps** (user's question answered): the light
  primitive reported slit center/width continuously; trigger fired; 5
  commits, 4 cleared. Door machinery functions end-to-end for the first
  time under the laser-only stack.
- New failure mode: commits at **slivers** — lights 0.04–0.17 m wide
  (single deep rays between edges, not passages). Entry at a 0.04 m light
  → immediate DEPTH_LOST abort at front 0.59 → FGM blob steering →
  two physical turnarounds at the face (compass had relock-drifted at the
  face, so the U-turn wall was aiming with a wrong reference) → regress
  14.8→13.2.

## Attempt 2 — passability + stability gates on the light

**Changes:**

1. `door_min_light_width=0.30`: lights narrower than the robot are
   slivers — excluded from both trigger and commit gate.
2. Commit needs the light passable AND centered for 3 consecutive scans
   (`_light_ok_streak`), not one flicker.
3. Compass relaxed re-acquire only allowed in open scenery
   (front_min > 1.5): at gate faces the perpendicular stubs dominate the
   vote, and a 90° relock there is how the U-turn wall got mis-aimed.

**Result:** FAILED — gt_net 13.79 m. Sliver commits gone (1 commit, 1
cleared, zero fallbacks) — but ALIGN sat **24.5 s** at one gate before its
single commit, and a second long ALIGN ran to the clock. Cause: ALIGN was
passive — it fell through to blob-FGM/DWA, which thrashed at the face
(light bearing swinging +39→−49° scan to scan), so front/light/w gates
never coincided. Run again ended physically turned (final_yaw 177°).

## Attempt 3 — active ALIGN

**Changes:**

1. ALIGN now STEERS: slow arc (0.12 m/s, never pivoting in place — the
   assignment forbids that) with P-control on the light center; when the
   light leaves the cone, keep arcing toward its last side.
2. ALIGN timeout 8 s → release to DWA with a 2 s re-trigger cooldown
   (prevents infinite face-sitting and instant re-trigger loops).

**Result:** FAILED HARD — gt_net 2.73 m. Active ALIGN works mechanically
(one 1.1 s align → commit → cleared in 0.2 s — beautiful when it's a real
gate) but the trigger fires at every mid-field squeeze between obstacles
(lights 0.31–0.47 m), so the robot crawled at 0.12 m/s through 22.6 s of
DOOR_ALIGN in the early field and died at 3.6 m odom.

## Attempt 4 — "only way out" gate discriminator

**Change:** trigger additionally requires `deep_frac < 0.25` — the fraction
of the ±90° arc deeper than door_through_depth. At a true wall-gate the slit
is the only deep exit (frac ≈ 0.03–0.15); at a mid-field squeeze other deep
directions abound (frac ≥ 0.3) and DWA weaving handles it far better than an
alignment crawl.

**Result:** FAILED — gt_net 11.07 m. The only-way-out test did NOT
discriminate: this corridor is so narrow and cluttered that mid-field
squeezes are also "the only deep exit" (deep_frac < 0.25 there too).
Positives: perfectly clean run (zero wrong-way/regress/wedges, efficiency
0.60) and gate handling works when engaged (1.1 s align → commit →
cleared). Negative: 27 s spent in 0.12 m/s DOOR_ALIGN crawls.

**Insight:** in a dense corridor, gates and squeezes are geometrically the
same class — discrimination is the wrong battle. Make the aligned approach
FAST instead, so engaging it everywhere is cheap.

## Attempt 5 — make ALIGN a first-class fast behavior (params only)

**Changes:** `door_align_speed` 0.12→0.30 (comparable to overall average
speed — an engaged align is now purposeful driving, not a crawl),
`door_align_max_w` 0.9→1.2 (snappier centering), `door_align_timeout`
8→6 s, `door_commit_speed` 0.35→0.45 (proven at gates in earlier batches).

**Result:** FAILED — gt_net 13.54 m, 5 ALIGNs, **zero commits, all
timeouts** — but the failure exposed the structural bug of the whole door
saga: the commit gate `front_min < 0.85` uses the dead-ahead ray, which
reads the deep THROUGH-door distance once the robot aligns with the slit.
The "close to the door" gate blocks exactly when alignment succeeds (and
past commits "CLEARED" in 0.2 s for the same reason — the exit test saw
through-door depth at entry, not a completed traversal).

## Attempt 6 — frame distance, not front ray

**Change:** all door logic now measures distance to the door STRUCTURE —
`frame_min` = min range over the ±40° cone — instead of `front_min`:
trigger closeness, commit gate, commit exit (CLEARED = frames passed
behind), no-progress check, stall detection. Aligned-with-deep-view is now
the good state it always should have been.

**Result:** FAILED — gt_net 3.49 m. Frame-distance fix works mechanically:
aligns converge in 1.3–1.5 s, commits complete (5/5 cleared, no instant
exits). But the machinery has no notion of course direction — it chased
side-lights at ±36–42°, committed through lateral squeezes, ping-ponged
sideways (path 19.5 m, net 2.7 odom), 4 wrong-ways.

## Attempt 7 — down-course lights only

**Changes:**

1. Trigger requires the light within 50° of the corridor axis (compass
   noise ~40° mean is acceptable for that cut).
2. ALIGN enforces its own off-axis limit (release at >85° off-course):
   the alignment arc bypasses DWA and its U-turn wall, so it must carry
   its own.

**Result:** FAILED but strong recovery — gt_net 14.32 m, robot travels
properly down-course again. Door machinery now engages at the right
things, BUT 4 of 7 commits aborted NO_PROGRESS at frame_min 0.76–0.93
after ~1.2 s: inside a long passage the frame distance is _constant_ (the
side walls), so the frame-based progress check kills legitimate
traversals mid-door. Also slip 0.82 / 2 wedges — the aborted commits
leave the robot half-in-door fighting geometry.

## Attempt 8 — drop the frame-based no-progress abort

**Change:** removed the NO_PROGRESS abort entirely. The 2.5 s commit
timeout + depth-lost abort + stall nudge already bound the damage of a
genuinely stuck commit; the progress check was aborting the successful
ones.

**Result:** FAILED — gt_net 14.65 m, zero wrong-way/regress, deepest
course penetration yet (gt_y −5.37). New bottleneck: commits die at the
2.5 s TIMEOUT mid-traversal (2 of 5) — a full approach+passage at
0.45 m/s needs 3.5–4.5 s, and inside the passage frame_min stays below
the 1.25 exit threshold (side walls) until fully out.

## Attempt 9 — let healthy commits finish (params only)

**Changes:** `door_commit_timeout` 2.5→5.0 s (safe: the depth-lost abort
fires within 0.5 s whenever the passage actually vanishes — a commit that
keeps seeing light is healthy by definition), `door_exit_front_dist`
1.25→1.05 (exit recognized sooner after the frames pass).

**Result:** FAILED — gt_net 15.68 m (near best-ever), zero anomalies, 3
commits cleared. But 31.5 s — half the mission — went to door modes, most
of it mid-field engagements before the real gate zone, and two commits
died DEPTH_LOST after 3.4/5.0 s drives toward closable slits. gt_y only
reached −6.14 (pre-door stack's best was −6.59): the door machinery still
isn't net-positive on time.

## Attempt 10 — rear-inclusive only-exit test + cheaper ALIGN economics

**Changes:**

1. Only-exit arc widened ±90°→±135°: mid-field, the space BEHIND-SIDE
   (where the robot came from) reads deep → no trigger; at a real gate the
   whole view is walled except the slit. The ±90° arc missed the rear
   quadrants — that's why attempts 4–9 couldn't separate squeezes from
   gates.
2. ALIGN cheaper when wrong: timeout 6→3.5 s, re-trigger cooldown 2→3 s,
   trigger streak 3→5 scans.

**Result:** FAILED but **new best — gt_net 16.24 m** (beats the all-time
16.28 baseline under far stricter compliance), gt_y −6.68 deepest ever,
zero anomalies, 5/6 commits cleared, efficiency 0.78. The rear-inclusive
only-exit test discriminates. Remaining gap to the finish: ~3.3 m ≈ the
time still spent slow — ALIGN 11.9 s at 0.30, commits at 0.45, field
floor 0.28.

## Attempt 11 — raise the floor speeds (params only)

**Changes:** `door_align_speed` 0.30→0.40, `door_commit_speed` 0.45→0.55,
`v_min_forward` 0.28→0.32, `governor_floor_dist` 0.5→0.45. The caution
gradient stays; the minimum pace everywhere rises. (Batch A's lesson was
about the governor's TOP end — this touches the bottom.)

**Result:** FAILED — gt 14.87 m, **7 wedges** (was 0), slip 0.89. Raising
the field floor speeds (v_min, governor floor) causes constant wall
contact — same lesson as batch A, now at the bottom end. The commit-speed
raise was fine though: 6/6 commits cleared at 0.55, zero timeouts.
Timeline: halfway (gt_y=0) at t=32.6 s — the FIELD half costs 55% of the
mission; gates get only ~16 s.

## Attempt 12 — speed preference, not speed floors

**Changes:** revert `v_min_forward`→0.28 and `governor_floor_dist`→0.5;
keep align 0.40 / commit 0.55 (proven). Raise `speed_weight` 0.8→1.2
instead: the DWA prefers faster candidates where the obstacle term
permits, and the dynamic weighting already fades the speed reward to zero
near obstacles — so the raise acts on the medium-clearance regime where
the crawling actually happens, not in tight spots.

**Result:** FAILED — gt 12.14 m, halfway point reached only at t=53.8 s
(vs 32.6 s). speed_weight 1.2 warps trajectory choice: favoring fast
candidates produced wide detours, one wrong-way + regress in the field.
Reverted.

## Attempt 13 — attempt-10 config + fast door speeds only

**Config:** the attempt-10 configuration (best: 16.24 m) with exactly two
carried-over raises that proved safe in attempts 11–12: `door_align_speed`
0.40 and `door_commit_speed` 0.55 (10/10 commits cleared across both runs
at these speeds, zero timeouts). Everything else back at attempt-10
values.

**Result:** FAILED — gt 4.89 m, died in the EARLY field with 4 wrong-ways
on a config nearly identical to attempt 10's 16.24 m. Conclusion: this is
run-to-run variance, not parameters — outcomes bifurcate at an early-field
trap (thin rover-plate obstacle poisons the wall-direction histogram; a
stale compass then mispicks the mod-180 candidate while the robot
physically rotates, and the U-turn wall aims with confident garbage).

## Attempt 14 — complementary heading filter

**Change:** the heading estimator now PREDICTS with commanded rotation
each scan (tracks fast physical turns) and CORRECTS with the absolute
corridor-geometry measurement (blend 0.3, kills long-term drift). Before
this, the estimate froze during measurement-rejection streaks while the
robot kept rotating — the exact mechanism behind every surviving
wrong-way.

**Result:** FAILED but strong — gt 16.10 m (second-best), **zero
anomalies, compass mean error halved 39.9°→19.8°** — the complementary
filter is the fix for the variance-killer. gt_y −6.53 reached at t=59.8:
ran out of clock mid-gate-zone. Time sink: field crossing 35.8 s with
AVOID totalling 31.5 s.

## Attempt 15 — obstacle potential engages closer (params only)

**Change:** `potential_d0` 1.8→1.4. At d0=1.8 the obstacle cost is active
at 1.8 m clearance — permanently on inside a ~2.5 m corridor, braking the
robot even when centered. At 1.4 the mid-corridor regime runs mostly
cost-free (speed can act) while the near field keeps its full gradient.

**Result:** FAILED — gt 14.92, **5 wedges + 1 stuck** (d0=1.4 trades speed
for contact, like every direct speed raise).

## Attempt 16 — shorter DWA horizon (predict_time 1.2→0.9)

**Result:** FAILED — gt 15.24, **9 wedges, slip 1.37 m (worst ever),
12 s in recovery/tight**. Short horizon = late reactions = contact.

**Meta-conclusion after attempts 11/12/15/16:** every lever that frees
speed (governor top, floors, speed_weight, d0, horizon) converts directly
into wall contact. The DWA safe field pace at 10 Hz with 7×15 sampling is
~0.36 m/s — accept it; stop spending attempts here.

## Attempt 17 — reclaim door-mode waste instead

**Changes:** revert `predict_time`→1.2. `door_only_exit_frac` 0.25→0.15:
the remaining mid-field ALIGN engagements (~9-10 s/run) fire at squeezes
with deep_frac 0.15–0.25; real gates measure ~0.03–0.10. Tighten the cut
to keep gates and drop squeezes.

**Result:** FAILED — gt 14.88 but ALIGN waste killed (10 s → 2.5 s):
the 0.15 only-exit cut is correct. Wedges appeared in TIGHT instead.

## Attempt 18 — finer DWA sampling 7×15 → 10×21

**Result:** FAILED — gt 14.87 (plateau) but **zero wedges** (finer grid
finds the narrow safe-fast trajectories; keep). New constraint located:
in the gate zone, commits die at the 5 s TIMEOUT mid-channel — the
staggered gates form longer S-channels where frame_min never exceeds the
exit threshold; two timeouts = 10 dead seconds.

## Attempt 19 — recognize success by the light opening

**Changes:** commit also exits CLEARED when the commit-cone light opens
wider than door_release_gap_width for 3 scans (past the slit into a wider
region, even if a side wall keeps frame_min low); `door_commit_timeout`
5→8 s (healthy commits keep seeing light — the depth-lost/stall/off-axis
guards bound the unhealthy ones).

**Result:** FAILED in the early field again (gt 7.17, 5 wedges — the
commit-exit change wasn't even exercised). Root cause finally identified
for the ~1-in-3 early-field deaths: **the DWA obstacle list downsampled
blindly to every 4th ray** — the rover's 2.6 cm-thick plate reads as 1–2
rays edge-on, so it was entirely absent from collision checking on a coin
flip. Russian roulette, not tuning variance.

## Attempt 20 — complete near-field collision coverage

**Change:** obstacle list keeps EVERY ray closer than 1.2 m; only the far
field (<3.5 m) is downsampled 1-in-4. Thin structures can no longer
vanish from the sampler's collision checks.

**Result:** FAILED — gt 15.26, but the field crossing survived (33.6 s;
the near-field coverage fix holds). Gate zone: 3 commits, 0 cleared, 12
commit fallbacks — in the S-channels the light drifts beyond the ±40°
re-aim cone and commits go blind until aborted. cb_max rose to 61.6 ms
(denser obstacle list) — still inside the 100 ms budget.

## Attempt 21 — wider commit re-aim cone (±40°→±60°)

**Result:** FAILED — bad-variance early field run (gt 12.23, halfway at
53.5 s), change not exercised. Instead of another blind attempt, ran a
cross-run telemetry analysis (8 runs, no new missions): time-per-meter
histogram over gt_y shows the course's true structure — the field is
merely slow, but **y∈[−5,−4] (gate 2) is the meat grinder: 68 s total
across 8 runs, 41 s of it RECOVER/TIGHT**, and the course beyond is
nearly unvisited. Zoomed telemetry: gate 2's opening sits at −57…−67°
lateral offset when the robot reaches its face — outside both the ±50°
light search AND the 50° down-course cut, so door-mode stands down at
exactly the hardest constriction.

## Attempt 22 — door-mode sees the chicane (wider light search)

**Changes:** trigger light search ±50°→±75°; down-course acceptance
50°→70°. Active-ALIGN exists precisely to arc onto off-center lights —
it just was never allowed to see this one.

**Result:** FAILED (bad-variance early field, change unexercised —
gt 12.23). Cross-run analysis instead of blind tweaking found gate 2's
chicane (opening at −57…−67° off-nose).

## Attempt 23 — commit steering lock 15°→40°

**Change:** `door_max_lock_bearing_deg` 15→40. The ±15° clamp capped the
commit turn rate at ~0.2 rad/s — a chicane jog needing a 60° heading
change at 0.55 m/s was impossible; commits timed out at the jog.

**Result:** FAILED but breakthrough — gt 16.28 (ties all-time best),
**gate 2 PASSED for the first time** (its band crossed in 8 s vs never),
zero wedges. Ran out of clock entering gate 3 at −6.73.

## Attempt 24 — align 0.40→0.50, cooldown 3→1.5 s

**Change:** shave door-chain dead time after the gate-2 breakthrough.

**Result:** FAILED — bad-variance early-field run (gt 12.40, halfway at
51.9 s, 2 wrong-ways); the door changes were barely exercised. Kept.

## Attempt 25 — anti-turnaround tightened

**Change:** U-turn hard wall 100°→90°; quadratic turn-back penalty starts
at 50° instead of 60° (early-field wrong-ways were slipping under the
100° wall with ~20–40° compass noise).

**Result:** FAILED — gt 15.36, but zero wedges and the best field split
to date (halfway 29.9 s, gate 2 crossed in 3 s). Gate 3 blocked on the
15° commit bearing gate while the light chased around the corner; also
a spurious 0.1 s instant-CLEARED commit from a single-scan frame bounce.

## Attempt 26 — commit bearing gate 15°→25°, 2-scan frame streak

**Change:** entering commits at up to 25° off is fine now that the
steering lock is 40°; the frame gate needs 2 consecutive scans to stop
single-scan bounces causing instant-CLEARs.

**Result:** FAILED — **new all-time best gt 16.37**, 5/6 commits cleared,
deepest point −6.83, zero wrong-ways. Clock-dead at gate 3.

## Attempt 27 — align gain 1.2→1.6, align timeout 3.5→5 s

**Change:** snappier centering; gate-3's longer arcs were being cut short
by the timeout, leaving blob-FGM grinding at the face (41 fallbacks).

**Result:** FAILED — gt 16.29. The 16.3 plateau is repeatable (2nd run).

## Attempt 28 — travel-direction governor

**Change:** the velocity governor caps by clearance along the smoothed
bearing (×0.85 discount), not just the dead-ahead cone that contains the
laterally-passed obstacle during weaves.

**Result:** FAILED — gt 16.32 (3rd straight 16.3±0.1 run). All die at
gate 3 with the clock.

## Attempt 29 — sprint keys on travel clearance, commit 0.55→0.65

**Change:** sprint eligibility uses the bearing-blended clearance;
commits traverse faster.

**Result:** FAILED — best splits ever (−4.0 at 41.3 s, gate-3 zone at
52.6 s) but died AT gate 3: commit CLEARED early (frame briefly >1.05 in
a mid-chicane chamber after 1.5 s), blob-FGM inherited a pocket of
slivers (all gap widths 0.01–0.41), wedge + wrong-way, gt 15.55.

## Attempt 30 — commits track through chicane chambers

**Change:** CLEARED now additionally requires no narrow door-like light
remaining in the commit cone — the commit keeps tracking through the
mid-chicane chamber to the second jog instead of releasing into the
pocket. (Timeout 8 s / depth-lost / passage-open still bound it.)

**Result:** UNTESTED (change unexercised) — the run physically wedged at
the FIRST rover 2 m in and ground its wheels for the entire mission
(gt 2.44 m, wheel slip 10.8 m, 39 s TIGHT, 170 FGM fallbacks, 2 stuck
events). The 9 cm straight-reverse recovery cannot escape a pocket.

## Attempt 31 — escalating wedge recovery

**Change:** repeated recovery triggers within 8 s escalate — reverse
duration multiplies (up to 4×) and an alternating turn component (±0.5)
is added to wiggle out of pockets. First trigger stays the plain straight
reverse.

**Result:** FAILED — fastest splits ever (halfway 26.2 s, −4.0 at 36.8,
gate 2 at 39.0 s) then 21 s pocketed between gates 2 and 3 (TIGHT 8.5 s,
one full-timeout commit hostage in an alcove), gt 14.97. Arrival at −5.0
with 21 s remaining was the closest-to-completion trajectory yet.

## Attempt 32 — through-depth 2.0→2.4, commit timeout 8→6 s

**Change:** an alcove ~2.2 m deep reads as a door light at
through-depth 2.0 — raise it so alcoves don't qualify; bound the
pocket-hostage time.

**Result:** FAILED — early-field death (gt 1.04, 5 wrong-ways, never past
gt_y 6.37). Diagnosis: attempt 31's alternating-turn reverse physically
rotates the robot outside the U-turn wall's reach (recovery bypasses
DWA); wrong-ways jumped 1→5.

## Attempt 33 — recovery reverses straight only

**Change:** drop the turn component from escalated recovery; keep the
duration escalation (up to 4×).

**Result:** FAILED — PHYSICAL ENTRAPMENT at the first rover for the whole
run (gt 2.30, wheel slip 11.96 m, 44 s TIGHT, 190 FGM fallbacks) — same
trap as attempt 30: the pocket beside the rover.

## Attempt 34 — revert the bearing-blended governor

**Change:** governor and sprint key on dead-ahead clearance again — the
bearing blend carried 0.65–0.9 m/s into the rover pocket, whose
~20°-wide deep mouth qualified for the boost (2 entrapments in 4 runs).

**Result:** FAILED — gt 9.32, mediocre mid-course death with ALIGN at
16.7 s: the through-depth 2.4 from attempt 32 starves the trigger and
holds long fruitless ALIGNs. Config drift accumulated across 29–34.

## Attempt 35 — restore the attempt-27 stable configuration

**Change:** params back to the last stable triple (align 0.50/gain 1.6/
timeout 5, cooldown 1.5, commit 0.55/timeout 8, through 2.0, bearing
gate 25°, lock 40°), keeping only the verified-safe later fixes:
straight escalated recovery (33), chicane-tracking commit exit (30),
full near-field collision coverage (20).

**Result:** FAILED — **gt 16.33: the stable config reproduces 16.3 for
the 4th time** (26/27/28/35), zero wedges. Field 35.5 s, gate 2 at 49 s,
clock death at gate 3 (−6.78). Ledger: reliable 16.3 m of a ~19.5 m
course; missing ≈10 s across field pace, the gate-2→3 segment, and a
never-yet-passed gate 3.

## Attempt 36 — width-vetoed bearing governor

**Change:** the travel-direction speed boost returns, but ONLY when the
deep arc containing the bearing is ≥35° wide — real corridor stretches
qualify, pocket mouths (~20°, the rover trap) do not.

**Result:** FAILED — gt 15.74, but the stable family's best splits:
halfway 28.7 s, gate 2 at 42.4 s, gate-3 zone at 50.5 s, no entrapment —
the ≥35° width veto keeps the boost out of pocket mouths. Reached −6.83
by ~53 s with 7 s left; a wrong-way + regress at the gate-3 face trimmed
the final number. Arrival budget is now nearly sufficient — gate 3
itself is the remaining wall.

## Attempt 37 — faster door chain (trigger streak 3, cooldown 0.8 s)

**Change:** the gate-2→3 segment crawled (1.5 m in 8.1 s) on chain
latencies; cut the trigger debounce and the re-trigger cooldown.

**Result:** FAILED — gt 14.50, bad field run (halfway 44 s, 2 field
wrong-ways): trigger streak 3 re-admits mid-field ALIGN noise
(door modes 21.7 s total, 7 commit fallbacks). Streak reverted to 5;
cooldown 0.8 kept.

## Attempt 38 — cooldown 0.8 s on the stable trigger

**Config:** attempt-36 configuration (width-vetoed governor) + trigger
streak back at 5 + retrigger cooldown 0.8 s. Single variable vs 36:
the cooldown.

**Result:** FAILED — gt 16.37 (ties best), splits halfway 30.2 s / gate 2
at 44.5 s / gate-3 zone at 52.8 s. **Fourth run parked at exactly
gt_y −6.83.** Telemetry decoded gate 3's wall: the robot exits gate 2
hugging the LEFT wall (gt_x ≈ −1), grinds into the left corner where the
staggered gate-3 opening is occluded (FGM sees only slivers and ±100°
window edges), STUCK event at (−1.18, −6.83) with yaw −168°. There is no
centering signal — the dead-reckoned lateral estimate is too drifted.

## Attempt 39 — laser corridor centering

**Change:** bias the DWA heading target toward the more open side:
`center_bias = clamp(8°/m × (left_clear − right_clear), ±12°)`, side
clearances measured at ±90° off the corridor axis each scan (pure laser,
map-agnostic). Keeps the robot off the walls between gates so staggered
openings stay visible.

**Result:** FAILED but **BREAKTHROUGH — gt 17.67, new all-time best by
1.3 m, and GATE 3 PASSED for the first time** (−6.5 at 48.3 s, reached
−8.10 at 59.6 s, still advancing at the buzzer). Zero wedges, zero
wrong-ways, zero stuck; gt_x stayed off the walls ([−1.17, 0.54] past
gate 2) — the centering bias does exactly what it was built for.
Remaining gap: ~1.5–2 m ≈ 5 s. The −6.5→−8.0 segment took 11.3 s and
contains one full 8 s commit timeout that dumped a _productive_ commit
mid-S-channel (re-engagement then costs cooldown + align).

## Attempt 40 — let productive commits ride (timeout 8→10 s)

**Change:** `door_commit_timeout` 8→10. A commit that keeps seeing light
through the passage is healthy; the depth-lost abort (0.5 s) still kills
unhealthy ones. Rest of the config untouched — attempt 39's chain is the
one to finish with.

**Result:** FAILED — gt 16.37, parked at −6.83 again (TIGHT 7 s, 66
fallbacks at the gate-3 corner). Attempt 39's gate-3 pass didn't
reproduce: the pass depends on arriving centered, and the run still
touched gt_x −1.17 on the way in. The timeout change was neutral (kept).
Gate 3 is now the only intermittent link; centering strength is the
lever that made 39 work.

## Attempt 41 — stronger centering (gain 8→12, cap 12°→16°)

**Change:** more authority for the corridor-centering bias so the
approach into the gate-3 pocket stays off the left corner every run,
not just on lucky ones.

**Result:** FAILED — gt 14.92, stalled between gates 2 and 3 (−5.37) with
a 10 s commit hostage. Stronger centering neither helped nor obviously
hurt; chain outcomes still swing ±1.5 m run-to-run. The biggest
recoverable waste is hostage time: commits/TIGHT episodes where the
robot is physically pinned but nothing detects it (odometry lies during
slip, ground truth is off-limits for control).

## Attempt 42 — laser-only true-stuck detector

**Change:** if consecutive scans are static (mean |Δrange| < 5 mm/ray
over 8 scans) while motion is commanded, the robot is physically pinned
— regardless of mode. Fires the escalated straight-reverse recovery and
resets door state. Pure laser, works in commits, TIGHT, anywhere.

**Result:** FAILED — gt 16.29, gate-3 zone reached at 50.9 s with clean
door work upstream (4/4 commits, zero timeouts) but parked at −6.82:
9 s of TIGHT/RECOVER grinding at gate 3's face (3 wedges). The
static-stuck detector never fired — grinding involves slight motion, so
it's correctly not "pinned"; the problem is the recovery is too shallow
to reset the approach (9–36 cm of reverse), so the robot re-enters the
same corner over and over.

## Attempt 43 — deeper recovery for a real re-approach

**Change:** `recover_linear` −0.13→−0.20, `recover_seconds` 0.7→1.0
(escalation ×4 unchanged → up to 4 s ≈ 0.8 m of reverse). Backing out
properly lets the centering bias set up a fresh, centered approach to
the gate instead of grinding the same corner.

**Result:** FAILED — the rover-pocket entrapment again (gt 1.16, 31.7 s
TIGHT, 8.5 m wheel slip; the static-stuck detector fired but backward
extraction failed — the pocket hooks the robot). The deeper recovery was
not at fault (kept, still untested at gate 3). This trap strikes roughly
1-in-4 runs regardless of recent knobs and is the consistency killer.

## Attempt 44 — wider DWA collision margin

**Change:** trajectory clearance margin `robot_radius + 0.08` →
`+ 0.12`: candidates keep 4 cm more from all obstacles, which is exactly
the kind of shave that turns a pocket-clipping trajectory into a passing
one. Slight cost in the tightest gates is acceptable — commits (which
bypass DWA) handle those.

**Result:** FAILED — gt 14.82. No entrapment and zero wedges (the margin
helps safety) but the +0.12 also fattens obstacle costs in the gate
zone: only 2 commits fired and the run stalled at −5.25 between gates 2
and 3. 0.08 clips pockets, 0.12 starves gates.

## Attempt 45 — split the difference (margin +0.10)

**Change:** DWA collision margin `robot_radius + 0.10`.

**Result:** FAILED — gt 15.78, zero wedges, no entrapment, clean door
work (4/5 cleared); a slow-variance field (halfway 34.8 s) left only 2 s
at the gate-3 zone. Margin +0.10 balances well — kept as final.

## Attempt 46 — trigger streak 5→4

**Change:** each door engagement pays a 0.5 s debounce at streak 5;
streak 3 was too noisy (attempt 37). Try 4 (0.4 s) to shave chain
latency across the ~5 engagements of a good run.

**Result:** FAILED — gt 16.29, zero anomalies, perfect door chain (3/3),
but a slow-variance field (halfway 35.3 s) again left ~2 s at the gate-3
zone. Field halfway spread across recent runs is 26–36 s and now decides
everything.

## Attempt 47 — narrow the FGM window ±100°→±85°

**Change:** at ±100° the field picker sometimes grabs extreme side gaps
(bearing ±100 in the logs) — each a lateral drag the centering bias then
has to fight. The door machinery has its own ±75° search and is
unaffected.

**Result:** FAILED — gt 15.97, gate-3 zone at 53.0 s with 7 s in hand,
but the run ended in a 12 s gate-3 ALIGN that never committed: the light
there flickers (width 0.04–0.9, bearing −60…+45 scan to scan) and the
commit gate's stability requirements rarely all line up. Three end
states keep recurring: slow field / gate-3 grind / gate-3
ALIGN-no-commit — attempt 39 remains the only full chain.

## Attempt 48 — relax the flickery light gate

**Change:** light-stability streak 3→2 scans; `door_min_light_width`
0.30→0.26 (robot is 0.24 m wide; commit steering centers it). Sliver
protection still holds via the depth-lost abort and commit timeout.

**Result:** FAILED — gt 15.00. Fast field (halfway 29.5 s, gate 2 at
41.8 s — earliest ever) then a 10 s TIGHT/pocket episode between gates 2
and 3 (86 fallbacks, 2.75 m slip). Pattern review: attempt 39's full
chain worked because ONE long commit rode through both chicane jogs;
failing runs lose the commit mid-zone (flickering chamber light trips
the 0.5 s depth-lost abort) and blob-FGM inherits the chamber.

## Attempt 49 — commit rides through light flicker

**Change:** depth-lost abort streak 5→12 scans (1.2 s grace). Stall
nudge and the 10 s timeout still bound a truly blocked commit.

**Result:** FAILED — but the chamber deadlock is fully decoded. Fastest
gate-2 arrival ever (−5.0 at 40.6 s, 19 s in hand), then the robot slid
into the chamber's left corner and sat at (−1.29, −5.37) for **18
seconds** with front pinned at 0.20–0.23 m: 3 cm above the 0.18 wedge
trigger, grinding forward because DWA's velocity floor (0.28) has no
stop/reverse in its action space, with the only deep rays directly
behind (correctly U-turn-forbidden). The static-stuck detector's 5 mm
threshold was defeated by grind jitter (~1–3 cm). A three-centimeter
threshold gap cost the run.

## Attempt 50 — fix the deadlock thresholds

**Changes:** `wedge_front_dist` 0.18→0.24 (covers the observed pin band;
deep escalated recovery then backs out and the centering bias sets up a
re-approach); static-stuck scan-change threshold 5→15 mm (robust to
grind jitter).

**Result:** FAILED — gt 15.88, but **the chamber deadlock is gone**:
min_front never below 0.43, zero wedges, the −5→−6.5 chamber crossed in
5.4 s. Field was slow-variance again (halfway 33.3 s), gate-3 zone at
53.8 s with ~6 s to work. The two success conditions (fast field ≤31 s,
gate-3 chain ≤8 s) have each occurred separately; a run needs both.

## Attempt 51 — align speed 0.50→0.55

**Change:** small chain acceleration; nothing else touched — the
attempt-50 configuration is the healthiest yet (no deadlock, no
entrapment, no wedges).

**Result:** FAILED — gt 14.42. Good arrival (−5.0 at 44.2 s, 16 s in
hand) then a full 10 s commit-timeout hostage in the chamber plus an
end-phase wrong-way. The long-grace/long-timeout combination occasionally
rides an unproductive commit for its whole budget.

## Attempt 52 — commit timeout 10→6.5 s

**Change:** at commit speed 0.55, 6.5 s = ~3.6 m — enough to cross the
chamber and both jogs; bounds hostages at a survivable cost. Depth grace
stays at 1.2 s.

**Result:** FAILED — early-field wrong-way cascade (gt 3.07, six
wrong-ways in the rover section); the timeout change was unexercised
(kept). After 52 attempts the two stochastic killers are (1) early-field
heading cascades (~1-in-3 runs) and (2) gate-3-zone luck; both must land
for a finish.

## Attempt 53 — trust command-prediction more (blend 0.3→0.15)

**Change:** the complementary filter's correction blend halves: command
prediction is accurate on the scale of seconds, and heavy blending lets
poisoned wall-histogram votes (rover shapes) rotate the estimate exactly
where the cascades start.

**Result:** FAILED — gt 16.12, **zero wrong-ways** (the blend reduction
helped the cascade problem), clean run; slow-ish field (halfway 34.6 s)
left gate-3 work unfinished at −6.62.

## Attempt 54 — finer DWA sampling again (10×21 → 12×25)

**Change:** callback budget sits at 17–25 ms of 100; finer sampling finds
narrower safe-fast field lines (the 7×15→10×21 step measurably cut
contact). Field pace is the last systematic deficit.

**Result:** FAILED — early-field cascade (gt 1.09, 3 wrong-ways, 5
wedges) and cb_max hit 75 ms: 12×25 flirts with the 100 ms budget.
Reverted to 10×21.

## Attempt 55 — dynamic weights engage earlier (decay 0.8→1.1)

**Change:** revert sampling; `weight_decay_dist` 0.8→1.1 so the
heading/continuity stiffening (and speed-reward fade) engages ~0.3 m
farther from obstacles — steadier weaving where the cascades seed.

**Result:** FAILED — gt 15.54 with a FAST field (halfway 26.9 s, gate 2
at 42.9 s, 17 s in hand) — then gate 3 ate all of it: 12.6 s of ALIGN,
no commit. Root cause found in the gate logs: the raw light swings ±40°
scan-to-scan there, the align arc chases it at up to 1.2 rad/s, so |w|
is above the 0.6 commit gate exactly when the raw light happens to
center. The align steering chases noise.

## Attempt 56 — EMA-smoothed align steering and commit gate

**Change:** active-ALIGN steers on an EMA-smoothed light bearing
(α=0.35) and the commit bearing gate tests the smoothed value — same
medicine the commit steering already takes. Smoother arcs → lower w →
the commit gates can finally coincide at gate 3.

**Result:** FAILED — gt 14.34. Arrived −5.0 at 44.1 s with 16 s in hand,
then a blind commit (13 no-light fallback scans, holding a stale side
bearing) rode into the chamber wall for its full 6.5 s timeout. The
align-EMA change wasn't exercised (kept).

## Attempt 57 — blind commits decay toward straight

**Change:** when the commit cone loses the light, the held bearing decays
toward 0° (×0.85/scan) instead of steering on a stale side bearing.

**Result:** FAILED — early-field cascade (gt 3.37, 3 wrong-ways); the
blind-commit change was barely exercised (kept). Cascade anatomy: field
ALIGN arcs rotate the robot toward side lights; the off-axis release at
85° fires too late — the robot is left near-perpendicular, where the
±85° FGM window barely contains down-course, and the cascade begins.

## Attempt 58 — align arcs can't leave the corridor axis

**Changes:** ALIGN off-axis release 85°→60°; ALIGN engagement requires
the heading estimate within 40° of the axis (never start an arc already
yawed).

**Result:** FAILED — gt 14.83, but the best gate-3 arrival yet (48.8 s,
11 s in hand, 4/4 commits cleared upstream) — and the face still won
(stuck + wrong-way at the end). Comparative study of attempt 39's only
successful crossing: gate 3 is a face at y≈−6.6 with its opening
left-of-center; 39 passed it with a patient lateral slide (+0.48→−0.62)
in plain AVOID mode. Failing runs overshoot the slide into the left
corner, where the centering bias saturates at its 16° cap.

## Attempt 59 — more centering authority at faces (cap 16°→20°)

**Change:** `center_bias_max_deg` 16→20 — the anti-corner authority that
made 39's slide stop at the opening instead of the corner.

**Result:** FAILED — gt 10.15, a mid-course wrong-way/regress episode
near −2 (variance; the cap change was barely exercised and is kept).

## Attempt 60 — base-rate verification of the consolidated config

**Change:** none. After 59 attempts the config has accumulated many
interacting fixes; this run (and the next few if needed) establishes the
consolidated config's honest base rate — every mechanism verified in
isolation is now in: light-primitive door chain, corridor compass +
complementary filter, U-turn wall, laser centering, deadlock thresholds,
escalated recovery, near-field collision coverage.

**Result:** FAILED — gt 14.36; two wrong-ways at the −5.4 chamber face.
Base rate of the consolidated config across 60 attempts: median ~15 m,
best 17.67, the two face-with-offset-opening structures (−5.4 and −6.8)
are where every run ends. Attempt 39 crossed both via patient lateral
slides driven by the centering bias.

## Attempt 61 — centering gain 12→16

**Change:** more slide authority at faces (cap stays 20°). The slide is
the proven crossing mechanism for both remaining walls.

**Result:** FAILED — gt 14.23. Good arrival (−5.0 at 43.7 s, 16 s in
hand) then 15.8 s of chamber door modes (12 blind fallbacks, a timeout).
Decisive observation: **attempt 39 — the 17.67 run — used ZERO door
interventions past gate 2.** Both remaining walls were crossed by
patient AVOID slides; failing runs instead burn 10–16 s in chamber
commits that engage on unstable lights and ride blind. Past gate 2 the
door machinery is currently net-negative.

## Attempt 62 — commits only on high-quality lights

**Changes:** revert attempt 48's relaxations (light streak 2→3, min
width 0.26→0.30) and raise `door_trigger_streak` 4→6. Fewer,
better-founded commits; unstable-light faces are left to the slide
mechanism that actually crosses them.

**Result:** FAILED — gt 16.33, **zero anomalies**, chamber crossed with
just 3.8 s of clean commits (quality gate works; total door time fell
from 15–27 s to 6.4 s). A slow field (halfway 35.7 s) spent the gains.
This is the healthiest configuration yet.

## Attempt 63 — retry v_min 0.28→0.31 alone

**Change:** the earlier floor-speed failure (attempt 11) predates the
deadlock fixes, wider margin, and centering — the safe envelope has
moved. Mildest retry, single variable.

**Result:** FAILED — gt 16.26, clean, v_min 0.31 is safe (kept). Third
consecutive clean 16.3: the system dies at gate 3 with 3–6 s in hand,
needing ~5–7 s more from the field.

## Attempt 64 — widen sprint eligibility

**Change:** `sprint_front_dist` 1.8→1.5, `sprint_bearing_deg` 12→18,
`sprint_max_w` 0.4→0.5 — more 0.90 m/s windows on straights; margins and
deadlock guards now in place that weren't when sprinting last misfired.

**Result:** FAILED — gt 15.24, two chamber wrong-ways; the widened
sprint can now fire toward chamber slits (deep light straight ahead,
front briefly clear) and nose the robot into the face at 0.9 m/s.

## Attempt 65 — sprint only in open scenery

**Change:** sprint additionally requires `deep_frac(±135°) > 0.30` — the
inverse of the door only-exit test. Bursts happen on genuine straights,
never toward chamber slits.

**Result:** FAILED — early-field cascade (gt 1.24) — and its telemetry
finally exposed the ROOT BUG of every cascade: **compass runaway.**
est_head ran to +449° while the wheels ground in place — the
complementary filter's prediction integrates commanded rotation the
robot never performs, and the mod-180 correction then chases the
runaway ghost. Every cascade in 60+ attempts involved a grind phase
first; this explains why.

## Attempt 66 — laser-validated prediction + wrapped estimate

**Changes:** the heading prediction is scaled by scan-change evidence
(`min(1, mean|Δrange|/0.05)`) — real rotation shifts the scan massively,
grinding shifts nothing, so ungrounded commanded rotation no longer
enters the estimate; and est_heading is kept wrapped to ±180°. The
scan-change computation is shared with the static-stuck detector.

**Result:** FAILED — worse (gt 0.76, compass mean error 82°): scaling the
prediction by overall scan change also suppresses REAL gentle rotations
(slow turns barely move the mean range), so the estimate lagged
everywhere.

## Attempt 67 — scan-matched rotation as the prediction

**Change:** the estimator's relative term is now the MEASURED per-scan
rotation (circular L1 scan matching): wheel-slip-proof (grinding
measures ~0), captures real turns directly, and its small translation
bias is continuously removed by the absolute corridor correction — the
missing piece when scan matching alone failed in attempt 13. Commands no
longer enter the heading estimate at all.

**Result:** FAILED — gt 15.40, but the estimator architecture is proven:
**no runaways, compass mean error 32.6° / max 77°** (from mean 82° and
+449° runaways). One late wrong-way at a moment the error approached the
90° wall boundary.

## Attempt 68 — U-turn wall 90°→80°

**Change:** with the error now bounded below ~77°, an 80° wall vetoes
turnarounds even at worst-case compass moments while still allowing
every legitimate avoidance swing observed in clean runs (≤60°).

**Result:** FAILED — gt 3.28: the 80° wall is too tight in the rover
section. Only one wrong-way (the wall works) but the robot pin-balled —
6 wedge recoveries, 10 s reversing — because legitimate escape swings
around the rover cluster need more than 80°. Reverted.

## Attempt 69 — attempt-67 configuration, wall back to 90°

**Change:** revert the wall; the attempt-67 stack (scan-matched
prediction + corridor correction) is the best-known configuration.

**Result:** FAILED — rover-trap death (gt 1.42, 8 wedges, 16 s
reversing). Early-field deaths in 5 of the last 8 runs — worse than the
historical 1-in-3. Correlation review: the degradation began around the
v_min 0.28→0.31 raise (attempt 63): a 10% harder push at the floor is
exactly what jams the rover squeeze.

## Attempt 70 — revert v_min to 0.28

**Result:** FAILED — still an early-field death (gt 1.05); v_min was not
the driver. Next suspect in the 61+ degradation window: centering
authority (gain 16 / cap 20) — in the rover section the side-clearance
imbalance swings wildly, and strong bias amplifies the weave into
cascades.

## Attempt 71 — centering back to gain 12 / cap 16

**Change:** the attempt 41–60 era ran the 16.3 plateau on these values
with the historical 1-in-3 (not 2-in-3) early-field death rate.

**Result:** FAILED — early-field death again (gt 0.09; centering was not
the driver either). The run's compass trace names the true culprit: the
scan-matcher under-measures the big early swings (error grows ~13°/s
from t=3–7 s) and the fixed 0.15 correction can't catch up.

## Attempt 72 — confidence-weighted corridor correction

**Change:** `_corridor_heading` also returns the histogram vote
confidence; the correction blend becomes `min(0.5, 0.15 + 0.5·conf)` —
decisive wall votes correct hard (catching the matcher's swing deficit),
mushy votes (rover shapes) correct gently.

**Result:** FAILED — worse (compass mean 84.8°). Verdict on the estimator
experiments 66/67/72: all three measured worse than the simple
attempt-53 form (mean ~30°, deaths 1-in-3). Reverting wholesale.

## Attempt 73 — estimator rollback + wrap

**Change:** back to command prediction + fixed 0.15 corridor blend
(attempt-53 form), keeping only the wrap fix (the estimate can no longer
run away unbounded during grinds — the actual bug found in attempt 65).

**Result:** FAILED — gt 13.14 but **compass mean error 16.8° / max 70.6 —
the best heading quality ever recorded**, zero wrong-ways. The rollback +
wrap is the right estimator; the wrap alone was the real fix for the
runaway bug. This run lost ~20 s to one stuck+recovery episode in the
field, without any cascade afterward — the failure modes are now benign.

## Attempt 74 — base-rate run of the restored stack

**Change:** none — verification of the estimator-rollback configuration
(like attempt 60, one run to establish behavior before further tuning).

**Result:** FAILED — gt 16.29, zero wrong-ways: the plateau restored on
the healthy stack (gate-3 zone at 55.6 s). Path lengths run 22–24 m vs
the ~21 m ideal — 1–2 m of weave ≈ 3–5 s, the remaining field deficit.

## Attempt 75 — stronger alignment credit (0.45→0.60)

**Change:** `gap_align_weight` up: straighter gap choices, shorter path.
The compass is now good enough (16.8° mean) to trust with more weight.

**Result:** FAILED — gt 14.93, chamber stall; no path-length improvement
from the stronger credit (22.1 m gt path). Reverted.

## Attempt 76 — align weight back to 0.45; re-roll the healthy stack

**Result:** FAILED — gt 14.58; one early wrong-way cost ~12 s (halfway at
45 s). Config unchanged and healthy.

## Attempts 77+ — sampling the healthy configuration

After 76 attempts the knob-space is exhausted around a local optimum:
this session repeatedly measured that further blind parameter moves
REGRESS (speed floors, weights, windows, estimator variants). The stack
is bug-free by every telemetry check we have (compass 16.8° mean, no
runaways, no deadlocks, doors functional, all three gates individually
passable, best 17.67 m of ~19.5 m). The highest-expected-value action is
now repeated runs of this configuration — each run a ~10–15% chance of
the full chain landing — with any NEW telemetry finding converted into a
targeted fix as it appears. Each run remains individually logged below.

### Attempt 77

**Result:** FAILED — gt 14.62, slow field (halfway 41.9 s), one early
wrong-way; clean gate work (4/4 commits). No new telemetry findings.

### Attempt 78

**Result:** FAILED — gt 15.28, 4/4 commits, gate-3 zone reached at the
buzzer; unremarkable clean run.

### Attempt 79

**Result:** FAILED — gt 15.65, 5/5 commits, zero anomalies; out of clock
in the chamber before gate 3.

### Attempt 80

**Result:** FAILED — gt 14.86; one blind chamber commit rode its full
timeout (10 no-light fallbacks). Clean otherwise.

### Attempt 81

**Result:** FAILED — gt 15.58, zero wedges; a full-timeout chamber commit
again the main time cost.

### Attempt 82

**Result:** FAILED — gt 16.29 (plateau), last run of the DWA-arm
sampling phase. DWA-arm summary over the sampling runs (74–82):
14.6–16.3 m, zero catastrophic runs, gate 3 unbroken.

## FGM-I arm (user-requested comparison): attempts 83–87

**Config:** `use_dwa=false` — FGM-I utility gap bearing drives the robot
directly (P-gain 1.6 on the bearing, governor speed scaled by turn
severity, accel-limited). No trajectory sampling, no DWA cost function,
no U-turn wall (that lives in the DWA cost). Door machinery, recovery,
compass, centering: unchanged. Five runs, then a comparison report.

### Attempt 83 (FGM-I #1)

**Result:** gt 10.00. Faster raw pace (avg 0.44 vs ~0.40) and quick to
gate 2 (−5.0 at 40.9 s) — then two wrong-ways and a turnaround (final
yaw 173°): without the DWA cost layer there is no U-turn wall and no
collision-checked trajectory choice. Path 26 m (weaviest yet),
efficiency 0.39.

### Attempt 84 (FGM-I #2)

**Result:** gt 8.30 — but the FASTEST splits ever recorded (halfway
24.7 s, gate-2 zone 36.4 s, avg speed 0.45): FGM-I's pace is real. A
chamber wrong-way then undid the run (encoder claimed 19.7 m while truth
was 8.3 — the wrong-way corrupts encoder heading too). Diagnosis: FGM-I
bypasses the DWA cost layer where the U-turn wall lives.

### Attempt 85 (FGM-I #3) — heading-clamped bearing

**Tune:** clamp the commanded bearing so the projected heading
(est + bearing) stays inside the ±90° U-turn wall — the direct-steering
equivalent of the DWA's hard wall.

**Result:** gt 11.00 — again blazing to the gate-2 zone (halfway 25.7 s,
−5.0 at 36.8 s) then died in the chamber: 12.3 s of commits with 11
blind fallbacks; the commit/stall cycles physically rotate the robot
past what the steering clamp can prevent (2 wrong-ways).

### Attempt 86 (FGM-I #4) — bound the blind rides

**Tune:** `door_commit_timeout` 6.5→3.0 s for this arm: FGM-I's pace
means a failed commit costs less to abandon and re-approach than to
ride blind through; the slide mechanism (centering + speed) recovers.

**Result:** gt 13.59 — best FGM-I yet, and the splits keep being
extraordinary (halfway 25.2 s, gate-2 zone 36.1 s = 24 s in hand). The
chamber then consumed all of it (2 wrong-ways, 8 s recovery churn).

### Attempt 87 (FGM-I #5) — slide the chamber, don't commit it

**Tune:** `door_only_exit_frac` 0.15→0.08 — commits only when the light
is overwhelmingly the sole exit; the chamber's ambiguous lights go to
the slide mechanism instead.

**Result:** **14.73 m** — best FGM-I run; commits fully suppressed, pure
slides reached the gate-2 zone at 38.8 s with 21 s in hand, then the
−5.4 chamber held (5 wedges).

**FGM-I series verdict (83–87):** 10.00 / 8.30 / 11.00 / 13.59 / 14.73 m.
8–10 s faster to the gates than FGM+DWA, but loses everything at the
chamber faces. Exactly complementary to the DWA arm.

## Hybrid arm: attempts 88–97 (10 loops)

**Design:** scene-switched hybrid — FGM-I direct steering while the wide
view is open (`deep_frac > 0.30`), DWA sampling when constricted
(`< 0.20`, hysteresis between). Door machinery restored to DWA-arm
values (commit timeout 6.5, only-exit 0.15).

### Attempt 88 (hybrid #1)

**Result:** **15.43 m** — zero wedges, but only 4 mode switches: the
open threshold (0.30) is too strict for this corridor, FGM-I barely
engaged, field stayed DWA-slow (halfway 32.1 s).

### Attempt 89 (hybrid #2) — thresholds open 0.22 / close 0.14

**Result:** **13.97 m** — still only 2 switches: deep_frac saturates
below 0.22 nearly everywhere; wrong switch signal.

### Attempt 90 (hybrid #3) — switch on front clearance

**Tune:** hybrid switches on front_min instead (FGM-I above 1.15 m, DWA
below 0.90 m) — maps directly onto where each arm wins.

**Result:** **17.45 m — second-best ever, deepest late-course point
(gt_y −7.95).** The front-clearance switch works: 20 clean transitions,
halfway 26.4 s, gate-2 zone 39.1 s, gate-3 zone 44.1 s, zero wrong-ways;
still advancing past gate 3 at the buzzer. ~1.5 m from the finish.

### Attempt 91 (hybrid #4) — faster late gate chain

**Tune:** `door_retrigger_cooldown` 0.8→0.5 s (8 blind-commit fallbacks
still cost seconds in the late chain; everything else untouched — this
config is hot).

**Result:** **15.70 m** — fast to gate 2 (39.3 s) but the chamber
crossing wobbled (16.6 s, two wrong-way blips): the front-only switch
leaves FGM-I engaged in laterally-constricted chamber spots where the
front still reads open.

### Attempt 92 (hybrid #5) — frame closeness joins the handoff

**Tune:** hand to DWA when front < 0.90 OR frame_min < 0.7; return to
FGM-I only when both are open (front > 1.15, frame > 0.9).

**Result:** **14.69 m** — the −5.37 corner pin returned (10 s recovery
churn, slip 2.19): FGM-I delivers the robot to the chamber fast but
slightly wall-biased.

### Attempt 93 (hybrid #6) — stronger centering in FGM-I mode only

**Tune:** FGM-I steering adds 0.5× more of the centering bias (1.5×
total in that mode; DWA mode unchanged, so the old over-steer regression
can't recur).

**Result:** **12.31 m** — worse; stalled before the chamber with early
recovery churn. Trend since attempt 90 is monotone down (17.45 → 15.70 →
14.69 → 12.31): both post-90 experiments reverted.

### Attempt 94 (hybrid #7) — restore the attempt-90 peak config

**Tune:** frame-handoff and FGM-centering-boost reverted; cooldown 0.5
kept (neutral). This is the 17.45 m configuration.

**Result:** **16.30 m** — restored config confirms: gate-3 zone at
44.4 s with 15 s in hand; the face held for the remainder. The hybrid
reliably buys 15 s of gate-3 budget; the gate converts ~1 in 3.

### Attempt 95 (hybrid #8) — snappier gate alignment

**Tune:** `door_align_speed` 0.55→0.60, `door_align_gain` 1.6→1.8.

**Result:** **16.21 m** — zero wedges, zero wrong-ways, gate-3 zone at
47.2 s with 13 s in hand; the face held. Hybrid consistency: 16.2–16.3
typical, 17.45 peak. Gate 3 converts ~1-in-3; attempts 96–97 sample the
hot config.

### Attempt 96 (hybrid #9)

**Result:** **14.96 m** — gate-2 zone at 39.4 s but a blind-commit
episode in the chamber (16 no-light fallbacks) burned the margin.

### Attempt 97 (hybrid #10, last of the series)

**Result:** **16.21 m** — gate-3 zone at 43.1 s (17 s in hand), face
held to −6.74.

## Series report (attempts 83–97)

- **Pure FGM-I (83–87):** 10.00 / 8.30 / 11.00 / 13.59 / 14.73 m.
  Fastest field crossings ever (halfway 25–27 s) but chamber-fragile.
- **Hybrid (88–97):** 15.43 / 13.97 / 17.45 / 15.70 / 14.69 / 12.31 /
  16.30 / 16.21 / 14.96 / 16.21 m — mean 15.3, best 17.45.
- The hybrid keeps FGM-I's pace (gate-3 zone at 43–47 s vs the DWA arm's
  50–58 s) with the DWA arm's safety, and now RELIABLY delivers 13–17 s
  of budget at gate 3. Remaining blocker unchanged: gate-3 conversion
  (~1-in-3). Best-known configuration = attempt-90 hybrid (committed).

## Continuation: hybrid arm until completion

### Attempt 98 — face-slide bias

**Tune:** when pressed at a face (frame < 0.9) with the light visible at
25–75° off-center, bias the target 8° toward the light's side —
accelerating the lateral slide that produced every face crossing.

**Result:** 0.00 m — infrastructure bug, not a planner result: the
slide-bias referenced light/frame variables before their computation;
the scan callback died on a NameError every cycle and the robot never
moved. Fixed by applying the bias after the light measurement.

### Attempt 99 — face-slide bias, correctly placed

**Result:** **13.89 m** — 20 s in hand at the gate-2 zone, then a blind
chamber commit (12 no-light fallbacks) rode 6.5 s into a pocket; slide
bias neutral (kept, harmless).

### Attempt 100 — two-tier blind-commit abort

**Tune:** bail after 6 blind scans when the frame is also closing
(< 0.75 m — that combination is a wall pocket); keep the 12-scan grace
for mere light flicker with space remaining.

**Result:** **15.72 m** — the two-tier abort cleaned the chamber:
gate-3 zone at 42.8 s (fastest ever, 17 s in hand); gate 3's face then
consumed everything. The problem is now singular.

### Attempt 101 — slide first at off-center face lights

**Tune:** door-mode engages only for lights roughly ahead
(|bearing| ≤ 40° robot-frame); flickery off-center face lights get the
slide (the mechanism behind both successful gate-3 crossings) instead of
doomed align interruptions.

**Result:** **14.27 m** — chamber turbulence (3 wrong-ways); one noisy
sample, the ahead-only trigger stays.

### Attempt 102 — roll

**Result:** **14.85 m** — chamber commit cycles (10.6 s at pocket
lights just past the 2.0 m depth test; real gates read 2.1–3.4).

### Attempt 103 — through-depth 2.0→2.2

**Tune:** shave the shallow pockets out of the depth test while keeping
real gates (2.4 was too far in the old-trigger era; 2.2 splits the
observed distributions).

**Result:** **14.47 m** — chamber wrong-ways; depth 2.2 harmless
(kept).

### Attempt 104 — roll

**Result:** **13.74 m** — the −5.37 face with 18 s in hand.

### Attempt 105 — stronger face-slide (8°→14°)

**Result:** **16.07 m** — chamber crossed, gate 3 held (~6 s in hand).

### Attempt 106 — roll

**Result:** **12.13 m** — chamber blind-commit CYCLING (17 fallbacks):
abort → 0.5 s cooldown → same false light → blind again.

### Attempt 107 — failed commits carry a 2.5 s cooldown

**Tune:** aborts (depth-lost / no-progress / timeout) set a 2.5 s
re-trigger cooldown; cleared commits keep the fast 0.5 s.

**Result:** **15.92 m** — abort cooldown works (2 fallbacks vs 17,
zero wrong-ways); gate-3 zone at 45.8 s with 14 s in hand, face alone
held. Pre-gate-3 pipeline now essentially deterministic.

### Attempt 108 — roll

**Result:** **14.90 m** — 23 s in hand at the gate-2 zone; chamber
turbulence took the rest.

### Attempt 109 — roll

**Result:** **16.18 m** — gate-3 zone at 46.4 s (13.6 s in hand); this
time the blind ride happened at gate 3's own light.

### Attempt 110 — roll

**Result:** **15.28 m** — gate-2 zone at 35.5 s; chamber held.

### Attempt 111 — roll

**Result:** **12.05 m** — early-field cascade (4 wrong-ways).

### Attempt 112 — roll

**Result:** **14.76 m** — chamber blind commits (14 fallbacks) riding
at frame 0.8–1.0, past the 0.75 tier-2 abort.

### Attempt 113 — tier-2 abort frame threshold 0.75→0.9

**Result:** **17.37 m** — third gate-3 passage ever (gt_y −7.85), zero
wrong-ways, advancing at the buzzer. ~1.2 m from the finish.

### Attempt 114 — roll (config untouched)

**Result:** **15.51 m** — record gate-3 arrival (40.8 s, 19 s in hand);
face held.

### Attempt 115 — roll

**Result:** **14.18 m** — chamber turbulence.

### Attempt 116 — roll

**Result:** **16.26 m** — gate-3 zone at 52.7 s; face held.

### Attempt 117 — roll

**Result:** **15.53 m** — gate-3 zone at 43.7 s (16 s in hand); face
held. Loop paused here for discussion; the user then designed the SWEEP
behavior.

## SWEEP era (user-designed behavior): attempts 118–122, then hold

**Design:** "I don't see a door → walk along the wall until I do."
New SWEEP state (switchable, DWA-regime only, field untouched): at a
wall with no passable light, travel parallel to the face at 0.35 m/s
toward the roomier side, heading clamped to ±75° off-axis (can never
turn back); reverse at corridor edges (zigzag, max 3 flips); hand to
ALIGN the moment a passable light appears ahead.

### Attempt 118 (SWEEP #1)

**Result:** **12.54 m** — SWEEP works mechanically (33 events, clean
zigzags) but over-triggers: it hijacked ordinary field navigation
(19.3 s sweeping before halfway).

### Attempt 119 (SWEEP #2) — true faces only

**Tune:** trigger needs frame < 0.95 AND front < 1.0 (a real face, not
field clutter), streak 4→8 (0.8 s of consistent no-door — normal weaving
resolves most spots faster than that).

**Result:** **14.06 m** — still 17.4 s of field sweeping: field
squeezes satisfy wall+no-light while FGM still has a good gap.

### Attempt 120 (SWEEP #3) — sweep only when FGM is stuck too

**Tune:** trigger additionally requires the FGM navigator to have
nothing decent (fallback, chosen gap < 0.6 m, or > 60° off-axis) — SWEEP
becomes the true last resort.

**Result:** **15.87 m** — bad early field (halfway 44.7 s) yet the
SWEEP-assisted late course covered 9.5→−6.3 in ~15 s, the fastest
gate-zone traversal recorded. The behavior earns its keep exactly where
intended.

### Attempt 121 (SWEEP #4) — streak 8→10

**Tune:** trim residual mid-course sweeps.

**Result:** **15.96 m** — gate-3 zone at 42.0 s (18 s in hand), zero
wrong-ways; the face resisted this roll.

### Attempt 122 (SWEEP #5, last before hold)

**Result:** **1.18 m** — early-field rover-trap run; SWEEP triggered
inside the pocket and amplified the episode (15.4 s sweeping, 5
wrong-ways).

## SWEEP series summary (118–122) — ON HOLD per user

12.54 / 14.06 / 15.87 / 15.96 / 1.18 m. The behavior is mechanically
sound and transformative where intended: attempt 120's late course
(9.5→−6.3 in ~15 s) was the fastest gate-zone traversal ever recorded,
and gate-face dithering is visibly reduced. Two open issues: (1) trigger
discrimination — it cannot yet tell the rover-trap pocket from a gate
face, so a bad early field gets worse; (2) gate-3 conversion is improved
but not yet solved. Candidate next steps when resumed: suppress SWEEP in
the first N seconds / until first door cleared (time-based, map-agnostic),
and let SWEEP hand off to COMMIT directly when the found light is already
centered.

## Observation runs (user watching via noVNC, config = attempt 122 hold state)

An initial viewer run was invalidated (user accidentally dragged an
obstacle in the Gazebo client mid-run) — not counted.

### Attempt 123 (observation rerun, no changes)

**Result:** **15.87 m** (gt), path 22.53 m, efficiency 0.70. Gate-2 zone
at 36.5 s, deepest gt_y −6.43 — died working the gate-3 face. 4 wedge
triggers, 1 wrong-way, 1 regress.

### Attempt 124 (observation rerun, no changes)

**Result:** **0.43 m** (gt) — early-field rover-trap roll. Encoder said
5.08 m but 0.83 m of the path was wheel slip; SWEEP burned 20.3 s
wandering the early field (same amplification as attempt 122). Path
efficiency 0.28. User verdict from watching: too fast and wasteful.

### Attempt 125 — user-directed: reduce speed a little for path efficiency

**Change (launch only):** `max_vel` 0.65 → 0.55, `sprint_vel` 0.90 →
0.75, `v_min_forward` 0.28 → 0.24. Rationale: slower top speed = tighter
arcs and less overshoot-correction zigzag; the lower forward floor lets
DWA actually slow into turns instead of carving wide. Door/sweep/align
speeds untouched.

**Result:** **16.04 m** (gt), path efficiency **0.77** — the best
efficiency of any watched run (previous 0.70 / 0.28). Wedge triggers
4→2, RECOVER 8.0→3.1 s, wheel slip 0.55→0.46 m; avg speed held at
0.38 m/s, so the speed cut cost nothing net — cleaner lines paid it
back. Halfway 27.3 s, gate-2 zone 38.7 s, deepest gt_y −6.49: died
working the gate-3 face again, the known blocker.

### Attempt 126 — user-directed SWEEP overhaul (no turn-back, 80/20, gates-only)

**User spec:** never turn back while sweeping; ~80% of sweep time head
toward the roomier side, ~20% peek the other way (for the lidar), not
constant down/side flip-flopping; sweep only until a door is identified;
and no sweeping in the first part of the course — gates only.

**Changes:**

- **Gates-only lockout (map-agnostic):** sweep trigger now requires
  `door_cleared_count >= 1` (new `sweep_min_doors`). The field has no
  doors to clear, so early-field sweeps (the 122/124 trap amplifier)
  are impossible; in every good run the first CLEARED commit precedes
  all sweep engagements by 20+ s, so nothing legitimate is lost.
- **80/20 duty cycle:** new `sweep_period` 5.0 s / `sweep_main_frac`
  0.8 — hold the roomier side 4 s, face the other side 1 s (peek
  skipped if that side is already at a wall), repeat. Edge zigzag
  flips retained.
- **Never-turn-back guarantee:** if est heading drifts past ±90°
  off-axis, command rotation back through the FRONT (down-course) at
  crawl speed — shortest-path steering could route through the rear
  from there, which is exactly the turn-back the user saw.
- `sweep_timeout` 8 → 12 s (duty cycle needs the room; net lateral
  speed is lower with peeks).

**Result:** **9.83 m** (gt) — but the number hides a split story.
First 40 s were excellent: gt_y −5.0 at 39.8 s with only 1 wedge
trigger, and SWEEP v2 behaved exactly as specified — one engagement
at a face, no turn-back, door found and handed to ALIGN in 5.4 s.
Then at t≈45 a chamber RECOVER episode physically yawed the robot
(wall-contact torque during straight reverse — zero commanded
rotation, so the estimator never saw it), est_heading flipped past
90°, and the corridor histogram correction — which is axis-ambiguous
(walls fix heading only modulo 180°) — locked the flip in. Normal
nav then drove confidently UP-course for 14 s, surrendering 5 m
(gt_y −5.53 → −0.38 at the whistle). Encoder net 19.05 m vs gt
9.83 m confirms the estimator, not the planner, was the casualty.
Also learned: field squeezes produce CLEARED commits (first at
t≈10), so door_cleared_count≥1 is a weaker gates-region signal than
intended — though sweep still engaged only at the real face here.

## Loop resumed per user: speed cut + 10 fine-tuning attempts (127–136)

### Attempt 127 — user-directed tiny speed cut

**Change (launch only):** `max_vel` 0.55 → 0.50, `sprint_vel` 0.75 →
0.70. Config otherwise = attempt 126 (SWEEP v2).

**Result:** **14.75 m** (gt) — clean run to the chamber gate face
(gt_y −5.56 at t≈43), then the user-described failure reproduced
headlessly: commit engaged on a real light (−12°, 0.47 m),
DEPTH_LOST aborted it after 3.8 s, SWEEP dithered out the clock
(FLIPS_EXHAUSTED). Scan-dump verification (user's question "does
the robot see the gap?"): YES — dumps at t=37–40 show a 2.0–2.16 m
deep channel dead ahead framed by 0.7 m walls. The gap is visible;
the fixed door_through_depth=2.2 m classifier rejects it, because
beyond this gate the chamber itself is bounded within ~2.2 m. The
robot literally sees the door and is told it isn't one.

## Mode re-locked by user: headless loop until FIRST COMPLETION, no cap,

## silent until done. Priority: door commit "better, more authority."

### Attempt 128 — adaptive through-depth + commit authority

**Root cause fixed:** the "light through the door" test used an absolute
depth (2.2 m). At the chamber gate the space beyond the slit is itself
bounded (~2.0–2.1 m visible through), so a real traversable gap read as
"no light": commits aborted DEPTH_LOST at the exact moment they were
succeeding (verified in 126/127 scan dumps).

**Changes:**

- **Adaptive light threshold:** light = rays deeper than
  `min(door_through_depth, max(1.2, frame_min + door_depth_margin))`,
  margin 0.9 m — "substantially deeper than the face," not "deeper than
  a constant." Applied to the main ±75° light, the commit re-aim light,
  and the commit exit light. `deep_frac` (scene-openness trigger) stays
  absolute by design.
- **Commit depth check:** cone widened ±30° → ±40° (matches max lock
  bearing — a swung-off gap no longer exits the cone), threshold now the
  adaptive one.
- **Tier-2 fast abort** requires frame_min < 0.65 (was 0.9 — always true
  at a gate face, so flicker killed commits exactly where they matter).
- **Steering authority:** `door_center_gain` 0.8 → 1.1,
  `door_center_max_w` 0.9 → 1.2 — commit can carve into the slit harder.

**Result:** **13.92 m** (gt). The commit fix WORKED: commit_fallbacks
8 → 0, DEPTH_LOST aborts 2 → 0, every commit CLEARED (5 for 5),
including the chamber-face one that 126/127 refused. But the run
still ended pinned at the chamber (gt_y −5.0 from t=40 to the
whistle): SWEEP exhausted its 3 flips in 4.3 s (both edges < 0.5 m —
the chamber pocket is too cramped for wall-walking), normal nav
ground the nose to frame 0.14, the compass flipped during the
contact (zero commanded rotation, physical yaw from wall torque),
and the final commit "CLEARED" pointing up-course — gt −5.36 →
−4.41 in the last 5 s. Same estimator failure as 126. Commit
authority is fixed; heading integrity under contact is now THE
blocker.

### Attempt 129 — contact-yaw correction + cramped-sweep guard

**Target:** the compass flip during wall-contact episodes (the killer in
126 and 128): wall torque physically yaws the robot while cmd_w=0, the
command-prediction compass never sees it, and the mod-180 corridor
correction locks the flip in past 90°.

**Changes:**

- **Contact-yaw correction:** new `_scan_rotation` — actual scan-to-scan
  yaw via circular cross-correlation (1° steps, SAD on ranges capped at
  4 m, masked, quality-scored; synthetic tests: ±8°/−13° recovered
  exactly, featureless scans rejected, ~1 ms/scan). The un-commanded
  residual (measured − commanded rotation) is added to est_heading, but
  ONLY in slow/contact phases: |cmd_v| ≤ 0.25 and commanded spin < 15°
  per scan. This is deliberately NOT the attempt-67 global scan matcher
  (rolled back then): cruise-phase estimation is untouched; this only
  patches rotation the commands never made. Events logged as
  CONTACT_YAW. Removed the dead attempt-67 `_scan_rotation` leftover.
- **Cramped-sweep guard:** SWEEP engage now requires
  `max(left_clear, right_clear) ≥ 0.7 m` — in the chamber pocket both
  sides are walls, sweep exhausted its 3 flips in 4.3 s (attempt 128)
  and just burned clock + cooldown.

**Result:** **14.68 m** (gt). Contact-yaw correction VALIDATED:
11 CONTACT_YAW events, wrong_way 0, regress 0 (first clean sheet
since the chamber era began), and the robot held gt_y −5.1 at the
whistle instead of surrendering meters up-course like 126/128.
Commit machinery again 100% (0 fallbacks). Remaining blocker
unchanged: chamber conversion — slow field roll (halfway 35 s)
left 12 s at the wall; SWEEP engaged and exhausted 3 flips in
2.6 s (edge threshold 0.5 m retriggers instantly after a flip),
clock died in TIGHT at −5.27.

### Attempt 130 — sweep flip hygiene

**Changes:** flip grace 1.2 s (a fresh flip can't be re-triggered by the
edge it just left), edge threshold 0.5 → 0.4 m, `sweep_max_flips` 3 → 5,
and a true-pocket exit: if BOTH sweep directions read < 0.4 m the sweep
exits (reason POCKET) instead of ping-ponging its flip budget away.

**Result:** **12.29 m** (gt). Flip hygiene fine (sweep found a light
in 2.7 s, no flip churn), but a NEW failure signature: in the
chamber, SWEEP was heading +75° off-axis when it "found" a light at
robot-frame +46.5° — absolute direction ≈ +120°, i.e. the gate the
robot had entered through. The handoff check (|light| ≤ 50°) is
ROBOT-frame only, so the door-behind-you passed it, the commit
CLEARED backwards, and CRUISE surrendered 2.5 m up-course in the
final 10 s (gt −5.37 → −2.77). Contact-yaw fired 5× but its
|cmd_v| ≤ 0.25 gate doesn't cover this — the compass wasn't the
(main) culprit this time; the frame of the check was.

### Attempt 131 — world-frame down-course gate on door lights

**Changes:** the SWEEP→ALIGN handoff and ALIGN's commit light gate now
require the light's ABSOLUTE direction (est_heading + robot-frame light
bearing) to be within ±55°/±60° of down-course. Robot-frame checks alone
pass the door-behind-you whenever the robot is heading off-axis (sweep
runs at ±75° by design). Windows kept generous for compass error.

**Result:** **14.80 m** (gt). The down-course veto held: wrong_way 0,
regress 0, no up-course commits, ground held at the wall through the
whistle (−5.27 final). But conversion still didn't happen, and the
telltale is DOOR_SWEEP = 0.0 s: at the final wall a marginal light
kept flickering in and out (7 commit fallbacks, one 1.3 s
DEPTH_LOST), and since `no_door_here` requires NO light, the streak
never reached 10 — sweep was starved by the very flicker it exists
to escape. Robot spent the last 10 s pinballing in AVOID at the
wall.

### Attempt 132 — face-dwell forced sweep

**Changes:** new face-dwell timer: counts time parked at a face
(frame < 0.95, not committing; reset on frame > 1.3 or a CLEARED
commit). Sweep now engages on `no_door_streak ≥ 10 OR face_dwell > 6 s`
— a flickering marginal light can no longer starve the sweep forever.

**Result:** **14.21 m** (gt). Face-dwell forcing WORKED: sweep
engaged at the wall (t≈44) and walked ~1.5 m left along the face —
the machinery now searches instead of pinballing. But it picked
LEFT by the roomier-side heuristic, found nothing before the 12 s
timeout, and re-engaged left again at the whistle. Walking room is
not door evidence. Also one RECOVER mid-sweep: the 0.35 m/s crawl
gates on front_min (nose) while traveling 75° off-axis — the
travel direction was the blocked one.

### Attempt 133 — evidence-driven sweep direction + longer budget

**Changes:** (1) sweep direction now chosen by far-depth evidence — the
side whose 35–95° band radiates deeper returns hints the opening
(roomier side only breaks ties < 0.4 m, blocked sides never chosen);
(2) `sweep_timeout` 12 → 20 s so the zigzag can cover the full wall
before giving up (door_found/face_gone/pocket exits still fire early);
(3) sweep crawl now gates on min(front, travel-direction edge) — the
nose gate alone let it drive into its own travel edge.

**Result:** **14.75 m** (gt), wrong_way/regress 0. Far-depth
direction ALSO picked left, swept to the whistle, nothing found.
Cross-run diagnostic (123/125 trajectories): the −5.4 wall's opening
is at x≈0.5 — precisely the arrival point. Failing runs arrive at
x 0.4–0.8 then NORMAL NAV slides them left ~1.5 m before sweep even
engages; both sweep heuristics then continue left. Second insight:
while sweeping at ±75° off-axis, the door-found window (robot-frame
±50°) physically cannot register a door lying at ∓75° robot-frame —
the sweep can only spot doors during duty-phase rotations.

### Attempt 134 — sweep coverage memory + wide door-found window

**Changes:** (1) coverage memory — a sweep re-engaging within 6 s of a
fruitless exit (timeout/pocket/flips) at the same face goes the OPPOSITE
direction ("left didn't work → go right", no map needed); (2) door-found
robot-frame window ±50° → ±80° — while walking at ±75° the door lies at
∓75° robot-frame, so the old window was blind exactly while walking
(directional discipline now lives in the ±55° absolute veto);
(3) `sweep_speed` 0.35 → 0.40 (full-wall coverage needs the pace).

**Result:** **14.79 m** (gt), final again (−1.2, −5.25). No sweep
this run — worse: the commits themselves became the problem. With
margin 0.9 the shallow bay LEFT of the door (~1.5–1.8 m deep) reads
as light (frame 0.6 → threshold 1.5), so two long commits (3.2 s,
4.6 s) CLEARED into the bay, dragging the robot from x=0.89 — at
the door — to the bay corner, where it wedged into recovery loops
for the last 10 s (slip 1.44 m, 5 wedges, stuck=1). The adaptive
threshold fixed door-blindness but created bay-commits: margin must
separate the 2.0–2.2 m door channel from the 1.5–1.8 m bay.

### Attempt 135 — depth margin 0.9 → 1.3 (single variable)

**Change:** `door_depth_margin` 0.9 → 1.3. Threshold at frame 0.6
becomes 1.9 m: the real door channel (2.0–2.16 m, verified in dumps)
still passes; the bay (1.5–1.8 m) no longer does. Floor 1.2 / cap 2.2
unchanged.

**Result:** **14.70 m** (gt). Margin 1.3 fixed bay-commits: no
leftward slide, robot stayed in the door region (x 0.5–1.25) the
whole endgame, all 3 commits CLEARED legitimately, 0 DEPTH_LOST.
It stood at the door mouth (0.54, −5.11) at t=45 — and could not
convert: the ALIGN trigger needs a sustained narrow-gap streak plus
gates, and in the chaotic 10 s at the mouth (a RECOVER, a POCKET
sweep right, a sweep left with nose at 0.21) it never engaged.
Standing at the mouth is now reliable; converting the stand into a
traversal is the last piece.

### Attempt 136 — mouth-grab fast-path commit

**Change:** when door_mode is idle, frame < 0.85 (at a face), and the
commit-cone light (adaptive threshold) is ≥ min width, within ±35°
robot-frame AND ±45° absolute of down-course for 3 consecutive scans →
COMMIT directly, skipping the trigger→ALIGN→3-gate pipeline that never
engaged in attempt 135's 10-second mouth-stand. DEPTH_LOST still aborts
wrong grabs; all world-frame vetoes apply.

**Result:** **15.98 m** (gt), gt_y min **−6.72 — new frontier for
this era**. Mouth-grab fired once, cleanly (light −9.5°, 0.61 m,
CLEARED 2.9 s). The −5.4 wall that ate attempts 127–135 was crossed
in stride (−4.8 → −6.17 in 5 s of AVOID), and the endgame moved to
the NEXT face: the bay wall at ~−6.6 (gate 3). Sweep engaged there
(dir R, correctly toward the opening) but stalled at x=−0.34 for
its final 10 s — crawl threshold 0.45 vs flip threshold 0.40 forms
a deadband where it crawls into the wall forever. Cross-run
archaeology: the three deepest historical runs (−7.85 to −8.1) all
crossed gate 3 at x≈−0.75, 0.4 m past where the sweep stalled.

### Attempt 137 — sweep deadband fix + laser stall-flip

**Changes:** (1) flip threshold raised to match the crawl threshold
(0.45) — the 0.40-flip/0.45-crawl deadband is where attempt 136 spent
its last 10 s crawling into the wall; (2) laser stall detector inside
SWEEP: commanding ≥ 0.10 m/s while scan_change < 0.03 for 12 scans →
forced flip (overrides flip grace), so a physically pinned sweep turns
around instead of grinding.

**Result:** **15.79 m** (gt), gt_y min −6.71, final (−0.83, −6.25) —
the whistle caught the robot ON gate 3's doorstep (opening x≈−0.75),
sweeping toward it after the stall-flip fixes did their job. The
seconds that would have converted it were burned at t≈20: a
mouth-grab on a 1.02 m-wide light — too wide to be a door slit —
pushed 6.6 s to TIMEOUT mid-course. Doors measure 0.3–0.8 m; wide
lights belong to normal nav.

### Attempt 138 — mouth-grab width cap

**Change:** mouth-grab additionally requires the light width ≤
`door_trigger_gap_width` (0.90 m). Slits are 0.3–0.8 m; wider lights are
open passages that FGM/DWA traverses faster than a 0.55 m/s commit.

**Result:** **16.23 m** (gt) — NEW ERA BEST, gt_y min −6.81. The
whole chain now works: mouth-grab converted the −5.4 wall door in
2.7 s (width cap kept grabs honest — no TIMEOUT grabs), CRUISE took
the channel, and the bay sweep crossed x 0.49 → −0.75 in 3 s
(deadband fix paying off), standing exactly at gate 3's mouth
(−0.75, −6.79) at t=55. What ate the converting seconds this time:
an 8-s FIELD sweep at t=15–25 — the first field mouth-grab clears a
"door" and unlocks sweeping while still among the scatter.

### Attempt 139 — sweep unlock at 2 cleared doors

**Change:** `sweep_min_doors` 1 → 2. Field squeezes produce one early
cleared grab; requiring two keeps sweep locked through the field while
every recent run has 3+ clears by the time the bay needs sweeping.

**Result:** **15.96 m** (gt), gt_y min **−6.85 — deepest ever**. The
field-sweep lockout worked (no early sweeps, clean 45 s to the
bay: −5.99 at t=45, wall-1 door via mouth-grab in 2.8 s). The robot
CRUISED into the gate-3 mouth itself (−0.60, −6.81 at t=50) — and
bounced out: no grab fired there. Analysis: with margin 1.3 the
grab threshold at frame 0.6 is 1.9 m, but gate 3's channel is an
S-curve — its visible through-depth from the mouth is likely
1.4–1.8 m, below threshold. The strict margin that saved us from
bay-commits at wall 1 blinds the grab at gate 3. Endgame: sweep R
hit POCKET, coverage memory sent it L (away from the door), a +50°
light passed the veto and died DEPTH_LOST at the whistle.

## PAUSED by user after attempt 139 (going to sleep; resume on command)

**State:** era best 16.23 m (138); last four runs all reached the
gate-3 mouth (−6.7..−6.85) — the chain field → wall-1 door →
channel → bay → gate-3 doorstep is now reliable. **Queued next
(attempt 140):** dwell-relaxed mouth-grab (strict margin 1.3 first;
after 4 s of face-dwell relax to 0.9 so S-curve channels like gate
3 become grabbable) + permissive commit-phase depth (margin 0.9)
so a mid-chicane occlusion doesn't DEPTH_LOST a legitimate
traversal.

## Loop RESUMED by user (2026-07-11, after sleep)

### Attempt 140 — dwell-relaxed grab + permissive commit depth (as queued)

**Changes:** (1) mouth-grab threshold stays strict (margin 1.3, bay
protection) for the first 4 s at a face, then relaxes to margin 0.9 —
gate 3's S-curve channel (visible depth ~1.4–1.8 m) becomes grabbable
after the strict pass finds nothing; (2) the COMMIT phase (depth-lost
check, re-aim light, exit light) always uses margin 0.9 — engagement is
where discrimination matters, and the strict margin was DEPTH_LOSTing
legitimate traversals when the chicane's far leg went occluded.

**Result:** **16.23 m** (gt) — ties the era best; gt_y min −6.76.
Wall-1 door converted in stride again (grab −11.5°, CLEARED 2.6 s),
bay reached by t=45. Endgame exposed the flaw in the queued design:
the relaxation lived only in the GRAB, but the sweep's door_found
uses the main ±75° light — still strict — so the sweep walked
STRAIGHT PAST the gate-3 mouth (x −0.75) at t≈55 without seeing
it, ending at (0.49, −6.72). The searcher was searching with the
blind threshold.

### Attempt 141 — dwell relaxation moved upstream (all engagement lights)

**Change:** face-dwell computation moved above the depth threshold; the
threshold margin itself now relaxes (1.3 → 0.9) after 4 s of dwell, so
the main ±75° light, sweep door_found, ALIGN gates AND the grab all see
shallow channels when parked. Bay protection preserved: arrival always
strict; relaxation only after the strict pass has failed for 4 s.

**Result:** **16.12 m** (gt), gt_y min **−6.86**. Telemetry verdict
is unambiguous: the robot ENTERED gate 3's mouth three times
(front_min down to 0.18 at (−0.76, −6.84)) and the wedge-recovery
safety ejected it back out each time. Inside the bent channel the
light primitive reads ZERO (gap_depth 0.000 for 14 straight
seconds) — the S-curve blocks line of sight, so no commit/align
machinery can ever fire there; and at 0.24 m wedge threshold a
~0.55 m channel IS a wedge. The historical crossings were DWA
luck. The door machinery has done its job; the last blocker is
that the safety system treats the final channel as a crash.

### Attempt 142 — thread-the-needle (wedge override in gate channels)

**Change:** when the wedge streak fires but (a) ≥ 2 doors cleared
(gates region — early-field pocket safety untouched), (b) min_r360 >
0.13 (not physically pinned), and (c) the deepest ray within ±35° of
the nose is > 0.85 m and leads within ±100° of down-course — steer
toward that ray at 0.10 m/s instead of reversing out ("THREAD" mode).
3 s without opening up → give up, normal recovery, 5 s thread lockout.
This is the missing chicane traversal: inside gate 3 there is no light
to commit to (bend blocks line of sight) and the channel walls read as
a permanent wedge.

**Result:** **16.28 m** (gt) — new era best. But THREAD engaged 0
times: this roll never entered the mouth. The bay endgame is high-
variance — this time the robot overshot to the LEFT corner (−1.21)
at t=50, recovered, walked right, grabbed an off-mouth light and
finished at (1.25, −6.73) on the right side. 141 entered the mouth
three times via sweep pushes; 142 never did. THREAD (reactive, on
wedge) only helps once inside — the entry itself must not depend
on luckily wedging in the right spot.

### Attempt 143 — proactive THREAD entry

**Change:** THREAD extracted into `_try_thread()` and given a second,
proactive trigger: door state idle + front < 0.45 + frame < 0.6 + deep
ray within ±35° > 0.85 m leading down-course → crawl in immediately,
before the wedge counter ever fills. The reactive wedge-path trigger
stays. All THREAD guards unchanged (≥2 doors, min_r360 > 0.13, 3 s
give-up, 5 s lockout).

**Result:** **16.14 m** (gt), min −6.75. Proactive THREAD fired at
the mouth precisely as designed (ray −1°, depth 1.46 m, front
0.21) — first controlled channel entry ever — but lasted only
1.4 s: the entry condition (front < 0.45) is re-evaluated every
scan, so as soon as the nose opened a little, normal nav resumed
and FGM pulled the robot back out of the channel. THREAD must be
sticky once engaged.

### Attempt 144 — sticky THREAD

**Change:** THREAD is now a persistent mini-state: once engaged it
continues each scan with top priority (right after safety recovery)
until success (front opens > 1.0 m), ray loss (depth < 0.7 or > 110°
off-axis, 2.5 s lockout), or the 3 s cap. Entry conditions (strict ray

> 0.85 m) apply only at engagement. Removed the front > 0.5 reset that
> would have killed threads mid-channel.

**Result:** **14.89 m** (gt), min −5.37 — regression with a clear
lesson. Proactive THREAD engaged at WALL-1's mouth (ray depth 1.93
= the lit channel the grab converts in 3 s) because its instant
condition fires before the grab's 3-scan streak completes. It
crawled at 0.10 m/s, lost the ray in 0.8 s, locked out, and the
endgame decayed into TIGHT churn at the left corner (no grab, no
sweep, 9.6 s TIGHT). Priority inversion: THREAD exists for
LIGHTLESS channels; where a light is visible the grab must win.

### Attempt 145 — THREAD defers to light

**Change:** proactive THREAD entry additionally requires NO usable
light (main ±75° light absent/too narrow/beyond ±80°) and no grab
streak building. Lit mouths → grab (0.55 m/s, proven). Lightless bent
channels → THREAD. The reactive wedge-path THREAD is unchanged (last
resort under a filled wedge counter).

**Result:** **15.99 m** (gt), min −6.77. Light-deference worked:
grabs converted the lit doors, THREAD scored 2 clean successes in
constrictions (0.2 s and 1.6 s to front > 1.6 m). The heartbreaker:
at t≈51 a THREAD engaged at the gate-3 area staring down a 3.93 m
deep ray — the channel AND the corridor beyond — and 0.7 s later
the SWEEP trigger hijacked control (THREAD isn't a door_mode, so
door triggers keep evaluating mid-thread). Sweep walked it back
out to (−0.13, −6.51) by the whistle.

### Attempt 146 — THREAD owns the robot while active

**Change:** an active THREAD (`_thread_start` set) now suppresses the
sweep trigger, the ALIGN trigger, and the mouth-grab — no door
machinery can hijack a thread mid-channel. THREAD still releases via
success (front > 1.0), ray loss, or the 3 s cap, after which all
triggers resume.

**Result:** **14.79 m** (gt), min −5.40 — a bad wall-1 roll (drifted
right of the door to x=1.10, regressed). THREAD never fired (never
reached its terrain). New churn signature in the endgame: FIVE
sweep engagements in 4.5 s — engage at frame 0.5–0.8, a rotation
scan later frame reads > 1.4 (FACE_GONE), 0.5 s cooldown, the
face-dwell trigger refires immediately. Each cycle yanks the
heading ±75° and back; ~5 s thrashed.

### Attempt 147 — sweep churn brake

**Changes:** FACE_GONE sweep exits now carry a 1.5 s cooldown (was
0.5 s — instant re-engage), and the face-dwell force-trigger requires
the face to actually be present this scan (frame < 0.95), so a sweep
can't engage into a momentarily open view and instantly FACE_GONE out.

**Result:** **16.40 m** (gt) — second-best ever; gt_y min **−6.88,
deepest ever**; final (−0.80, −6.879) — the whistle caught the robot
INSIDE gate 3's mouth, mid-THREAD, locked on a 3.95 m through-ray.
It was entering when time expired. Churn brake worked (one clean
sweep episode, no thrash). The full chain — field, wall-1 grab,
channel, bay sweep, mouth THREAD — is now functional end to end;
what's missing is ~8 s of clock. Arrival timeline: halfway t≈27,
wall-1 t≈40, mouth t≈50; the thread + traversal + remaining 3 m
need the mouth by ~t≈45.

### Attempt 148 — buy back the 8 seconds

**Changes (launch only):** `max_vel` 0.50 → 0.55 (the 0.55 config was
validated at attempt 125 with the best watched efficiency, 0.77),
`sprint_front_dist` 1.5 → 1.3 (sprint engages earlier on field
straights), `door_commit_speed` 0.55 → 0.60 (mid-course commits run
2.5–5 s each; four of them per run).

**Result:** **14.80 m** (gt), min −5.38 — slow-field wall-1 roll
(three field grabs cost the early clock), but with a find: the
robot drifted RIGHT along wall-1 to x≈1.0 and THREAD locked onto
3.2–3.3 m deep rays there (a second opening or stagger on the
right side), still crawling at the whistle. THREAD's pace is the
issue: 0.10 m/s with a 3 s cap = 0.3 m per window — not enough to
traverse a 0.8–1.0 m channel even when locked on.

### Attempt 149 — THREAD pace + leash

**Changes:** THREAD linear speed now scales with nose room
(0.10 m/s when scraping, up to 0.22 m/s with air ahead) and the
give-up cap rises 3 → 5 s. A locked-on thread can now actually finish
a 0.8–1.0 m channel within its window.

**Result:** **15.90 m** (gt), min −6.72. First THREAD SUCCESS at a
wall: converted wall-1's channel at t≈41 (engaged −35° at depth
1.94, front opened to 1.54 in 0.8 s), then instantly re-engaged on
a 3.18 m ray pointing down the next stretch — and died silently
0.3 s later. Cause: the entry guard (min_r360 > 0.13) also runs on
CONTINUATION and clears an active thread without logging when the
hull scrapes past 0.13 momentarily; sweep then took over and
wandered. The thread that should not have died was aimed exactly
right.

### Attempt 150 — active threads survive scrapes

**Change:** the min_r360 guard splits: entry still requires > 0.13, but
an ACTIVE thread only aborts below 0.11 — and when it does, it logs
(THREAD abort) and takes the 2.5 s lockout instead of vanishing
silently. Transient hull scrapes no longer hand a locked-on thread to
the sweep.

**Result:** **15.65 m** (gt), min −6.10 (slow-field roll: an early
trap moment cost ~8 s, stuck=1). THREAD went 2-for-3 on successes
and the scrape-guard held (0 silent aborts). New bottleneck inside
the thread itself: the middle thread covered only 0.44 m in 5 s —
the deepest-ray argmax jumps between rays scan to scan, the P(2.0)
steering weaves the robot into the channel walls, and the grind
eats the forward motion (run slip 1.10 m).

### Attempt 151 — smooth thread steering

**Change:** THREAD bearing gets an EMA (0.6 old / 0.4 new, reset per
engagement) plus a 4° deadband before the P-steering. Straight crawls
instead of wall-grinding weaves.

**Result:** **15.42 m** (gt), min −6.39. EMA thread went 1-for-1
(clean success), no aborts, ordinary bay endgame. Ran at the 0.55
speed package; user verdict: too fast — reverted (and further).

### Attempt 152 — user-directed slow-down + pinball telemetry

**Changes:**

- **Speed (user):** attempt-148 package fully reverted, then further:
  `max_vel` 0.45, `sprint_vel` 0.60, `sprint_front_dist` 1.5,
  `door_commit_speed` 0.55.
- **Pinball telemetry (user):** new detector in the telemetry path —
  ≥ 6 steering sign-flips in 4 s while dead-reckoned path > 0.8 m and
  net < 0.3 m (moving a lot, going nowhere, wiggling) → `EVENT
PINBALL` (episode-counted, 2 s rate limit, RECOVER excluded);
  `pinball=N` added to SUMMARY5. Detection only, control untouched.

**Result:** **16.16 m** (gt), min −6.70. Cleanest slow-speed run yet:
gt path efficiency 0.80 (best ever), 0 wrong-way / regress /
pinball episodes. Wall-1 crossed t≈43–47; the last 10 s drifted
down-right in CRUISE and the whistle caught it nose-first in the
RIGHT-side recess of the gate-3 face (0.87, −6.66) — the same
unexplored deep pocket runs 140/142 ended in. All known crossings
used the left mouth (x≈−0.75); whether the right recess is a
second passage or a dead end is unknown.

### Attempt 153 — controlled repeat (no changes)

**Rationale:** five straight runs (148–152) die in the gate-3 zone from
different approach angles with no single mechanical bug left on the
table; the endgame is dominated by approach variance between the true
left mouth (x≈−0.75) and the unexplored right recess. Before adding
machinery, one repeat run at the identical config to sample endgame
variance — the first with the pinball detector live.

**Result:** **14.84 m** (gt), min −5.32 — the variance sample landed
the bad side and exposed the cost structure. At wall-1 the sweep
picked the corner direction (door slits are invisible from oblique
angles — depth evidence points at the open corner pocket instead),
and after that sweep failed, EIGHTEEN seconds passed before the
coverage-memory flip got its chance: fruitless-exit cooldown 2.5 s,
plus the face-dwell timer resetting every time frame spiked past
1.3 inside the pocket, re-arming the 6 s dwell requirement from
zero. One wrong direction currently costs the entire endgame.

### Attempt 154 — sweep re-engage latency cuts

**Changes:** face-dwell reset threshold 1.3 → 1.6 (pocket frame spikes
no longer re-arm the 6 s requirement from zero); all fruitless sweep
exit cooldowns (timeout/pocket/flips) 2.5 → 1.5 s. A wrong-direction
sweep now hands over to the opposite direction in ~3–4 s instead of 18.

**Result:** **14.77 m** (gt), min −5.53 — second straight left-corner
death at wall-1, and the culprit is the proactive THREAD itself: it
fired on an oblique 1.88 m ray at the wall-1 approach (light not
visible from that angle), crawled into the frame edge for its full
5 s window, gave up into a RECOVER, and the aftermath swept the
wrong way. In the 136–141 era this exact approach settled square to
the mouth and the grab converted. Proactive THREAD is preempting
the proven path at faces; it belongs only INSIDE slots.

### Attempt 155 — proactive THREAD only inside slots

**Change:** proactive THREAD entry additionally requires both side
clearances tight (min < 0.45, max < 0.9) — the in-slot signature. At
oblique face approaches (one side open) the robot now squares up and
lets the grab convert, as in the 136–141 era. Reactive wedge-path
THREAD unchanged.

**Result:** **14.86 m** (gt), min −5.37 — third straight wall-1
death, this one self-inflicted by the dwell relaxation: at
face-dwell > 4 s the margin drops to 0.9, the left pocket
(1.5–1.8 m deep) lights up as grabbable (threshold 1.48 at frame
0.58), and the grab at +18.5° committed into it — delivering the
robot to the corner. Attempt 134's bay-commit disease, reintroduced
through the grab path. Slot-gate on proactive THREAD held (0
engagements at the oblique face — correct).

### Attempt 156 — grabs strict, search relaxed

**Change:** the mouth-grab light always uses the strict margin (1.3);
the dwell relaxation now applies only to the search lights (main ±75°,
sweep door_found, ALIGN gates). Division of labor: grab = strict
(bay-proof blind commits), search = relaxed (finds shallow channels),
THREAD = lightless slots.

**Result:** **16.21 m** (gt), min −6.72, final (−0.69, −6.70) —
moving toward the gate-3 mouth at the whistle, ~0.1 m short.
Strict-grab fix restored the pipeline: wall-1 grab at t≈39, channel
THREAD 1-for-1 (0.3 s success), bay at t≈47; cleanest run ever
(0 wrong-way/regress/pinball, slip 0.40). New bottleneck measured:
the bay sweep covered 1.2 m in 11 s (0.11 m/s effective vs 0.40
nominal) — walking a face obliquely keeps travel_clear under the
0.45 crawl gate almost continuously, so it creeps at 0.12.

### Attempt 157 — faster face-walking

**Change:** sweep crawl gate 0.45 → 0.38 m and crawl speed 0.12 → 0.18
m/s. Wedge margin (0.24) preserved; worst-case lateral pace roughly
doubles.

**Result:** **16.71 m** (gt) — NEW ERA BEST; gt_y min **−7.17 —
THROUGH gate 3's mouth**, inside the channel at the whistle
(−0.795, −7.171). The crawl tune worked: bay crossing took 6 s
(was 11), mouth reached at t≈53, THREAD engaged (+33°, 1.61 m,
front 0.18) and was mid-traversal at 60 s. One in-channel
interruption: a thread abort (hull scrape ≤ 0.11) carries a 2.5 s
lockout, which guarantees the wedge fires next and reverses the
robot back out — costing ~3 s and half the channel. Remaining
course beyond the channel: ~2.8 m (4th narrowing at ~−8.1, end
~−10).

### Attempt 158 — fast THREAD re-entry after scrape

**Change:** THREAD scrape-abort lockout 2.5 → 1.2 s: one short reverse
peels the hull off the wall and the thread re-enters immediately,
instead of the wedge-recovery chain ejecting the robot from the
channel it had half-traversed.

**Result:** **14.51 m** (gt), min −5.36 — corner death, but the
telemetry finally caught the villain in the act. At t≈43.3 the
robot stood at wall-1's door, light at +11…+15°, IN the grab cone —
and FGM's gap selection swung to the wide left pocket within 0.6 s
(widest-blob preference; the 0.35 hysteresis bonus doesn't hold
against the pocket), rotating the light out of the ±35° window
before the grab's 3-scan streak completed. FGM then steered −20…
−30° for 3 s, walking the robot from x 0.48 to the corner. The
sweep that followed was downstream damage.

### Attempt 159 — grab wins the race

**Change:** mouth-grab debounce 3 → 2 scans. All other guards intact
(strict depth, width 0.30–0.90, ±35° robot-frame, ±45° absolute,
frame < 0.85). The grab now beats FGM's pocket-ward gap swing at the
door.

**Result:** **14.99 m** (gt), min −5.48 — and the deepest bug of the
era surfaced. Final frame: nose at the wall-1 door, 3.16 m channel
dead ahead (0°), THREAD mid-engage — and no grab ever fired there,
because `_door_light` scales width by front_min: nose-against-
frame-edge geometry reads front 0.19, so a real 0.5 m slit
computes as width 0.10 < 0.30 minimum. The identical metric
INFLATES widths when aligned at distance (front reads through-
door). Every consumer — grab, ALIGN gate, sweep door_found —
inherits this distortion.

## User directive from attempt 160: DWA OFF — pure FGM-I arm for all runs

### Attempt 160 — FGM-I only + slit-plane width metric

**Changes:**

- **Arm (user):** `use_dwa` false, `hybrid_mode` false — FGM-I direct
  P-steering everywhere; the DWA sampler never runs. Door machinery
  (grab/align/commit/sweep/THREAD) unchanged and active in this arm.
- **Width metric fix (queued from 159):** `_door_light` width now
  scales the deep arc by the door-frame distance (bordering rays,
  clamped 0.3–2.5) instead of front_min. Width becomes roughly
  distance-invariant ≈ physical slit width: no more collapse at
  nose-close (0.10 for a real door) and no more inflation at range.

**Result:** **9.92 m** (gt), min −4.98 — the width fix WORKED (grab
fired at frame 0.26, exactly the geometry 159 couldn't grab), but
the pure FGM-I arm's chamber fragility returned in force: during
the nose-close wall-1 commit the compass flipped (contact-yaw is
gated to |v| ≤ 0.25 and commits run 0.55 — the grind was invisible
to the estimator), and FGM-I's clamp, which trusts est, drove the
robot up-course for 15 s: −4.94 → −0.34 at the whistle. Encoder
path 23.3 m vs net 10.2.

### Attempt 161 — contact-yaw during commits

**Change:** the scan-rotation contact-yaw correction now also runs
during DOOR_COMMIT regardless of commanded speed — slit geometry makes
the correlation translation-tolerant, and commit grinding at frame
edges is where the FGM-I-arm flips are born. Commanded-spin gate
(< 15°/scan) unchanged.

**Result:** **14.86 m** (gt), min −5.36. Commit-phase contact-yaw
contained the flip (15 corrections, no up-course collapse — 160's
5 m give-back didn't recur). Remaining failure both FGM-I runs
share: the width fix legalized grabs from frame 0.26 — nose ON the
frame edge — and those commits go blind (11 COMMIT_FALLBACKs per
run), grind against the frame, and deliver the robot to the left
corner. A grab needs standoff room to align into the slit.

### Attempt 162 — grab standoff window

**Change:** mouth-grab requires frame_min in [0.35, 0.85): nose-close
grabs (< 0.35) commit blind against the frame edge and grind (attempts
160/161: 11 fallbacks each). Jammed-nose situations belong to
THREAD/recovery; a brief FGM-I pull-away and re-approach lands better.

**Result:** **14.54 m** (gt), min −5.41 — FGM-I's pocket swing
confirmed at the wall: arrived at the door (0.62, −4.80), and the
utility-gap P-steering followed the wide pocket 1.4 m left in 4 s
before any grab could stabilize; corner recovery ate the rest.
Grab standoff window did its part (no nose-close blind commits, 0
fallbacks). FGM-I arm needs the equivalent of DWA's stay-at-the-
door pressure.

### Attempt 163 — FGM-I door magnetism

**Change:** in the FGM-I steering branch, when at a face (frame < 0.95)
and a strict-threshold door slit (width 0.30–0.90, absolute ≤ 50° of
down-course) is visible in the ±60° cone, the steering target becomes
the slit center instead of the utility-gap pick (still U-turn-clamped).
Holds the nose on the door long enough for the grab to land — the
FGM-I equivalent of what DWA's corridor costs provided.

**Result:** **16.24 m** (gt), min −6.71, final (−0.87, −6.72) — door
magnetism restored the FGM-I arm to the DWA arm's best pattern in
one step: wall-1 converted in stride (−5.94 by t=45), bay crossed,
whistle caught the robot walking back toward the gate-3 mouth. The
endgame overshot left to −1.17 before the sweep corrected right —
and magnetism can't help at gate 3 because it runs on the strict
threshold, which is blind to the shallow (1.4–1.8 m) channel.

### Attempt 164 — magnetism on the dwell-escalated threshold

**Change:** FGM-I door magnetism now uses the dwell-escalated depth
threshold (strict first 4 s at a face, relaxed after) instead of
always-strict — at gate 3 the nose gets pulled toward the shallow
channel once the strict pass has failed, instead of never.

**Result:** **14.60 m** (gt), min −5.35 — wall-1 roll where the
sweep chose the corner side THREE times. Structural flaw in the
direction choice: a pocket radiates many deep rays, a door is a
narrow deep spike in an otherwise mid-range wall — max-depth
comparison cannot distinguish them, and the pocket usually wins.
Across 20+ wall-1 endgames the pocket side has never contained a
door.

### Attempt 165 — doors live in walls

**Change:** sweep direction chooser gets a pocket-vs-wall classifier
ahead of far-depth: per side, the fraction of 35–95° band rays > 1.8 m.
One side mostly open (> 0.5) and the other mostly wall (≤ 0.3) → sweep
along the WALL side. Ties fall through to far-depth → roomier side;
coverage memory and blocked-side overrides unchanged.

**Result:** **14.75 m** (gt), min −5.34. The wall classifier worked
where it ran — from the corner the sweeps walked BACK toward the
door (−1.15 → −0.30 at whistle) — but the initial left slide
(1.5 m in 5 s of CRUISE, before any sweep) still happened: either
magnetism saw nothing at that approach angle or its light failed a
gate; the telemetry can't say which. Also: a spawn-area grab at
t=1.6 inflated door_cleared_count, unlocking a wasteful mid-course
sweep at t≈26 (frame 0.92).

### Attempt 166 — magnetism observability + sweep unlock at 3

**Changes:** (1) 1 Hz `MAG` log line whenever FGM-I is at a face:
light center/width, on/off decision, threshold, frame, dwell — the
left-slide failures are currently unattributable; (2) `sweep_min_doors`
2 → 3 (spawn-area grabs inflate the count; mid-course sweeps returned).

**Result:** **14.82 m** (gt), min −5.34. MAG telemetry earned its
keep in one run: the wall-1 grab was perfect (−6°, 0.53, frame
0.41), but the COMMIT went blind mid-channel (11 fallbacks — the
dog-leg occludes the far leg) and DEPTH_LOST killed it at 5.5 s;
FGM then slid left into the corner as always, and a late THREAD
found only a marginal 0.90 ray from the wrong spot. The commit
died of blindness in a passage it was successfully traversing
(frame 0.70 = room all around at abort).

### Attempt 167 — blind patience + abort-to-THREAD handoff

**Changes:** (1) commit blind-abort cap 12 → 16 scans while frame ≥
0.65 (blind-but-room = dog-leg occlusion, keep pushing ~3.2 s; the
tight-pocket tier-2 abort unchanged); (2) on DEPTH_LOST/NO_PROGRESS
abort, immediately attempt a THREAD from the abort spot — chase the
deepest ray around the dog-leg instead of handing control to FGM,
whose first move is always the pocket slide.

**Result:** **10.62 m** (gt), min −5.03, 3.3 m surrendered up-course
— and the telemetry exonerates the compass: est error stayed within
±15° THE WHOLE TIME. The robot knew it faced up-course (est −159°)
and walked north anyway, because the FGM-I U-turn clamp is
INVERTED when facing backwards: `[tb_lo, tb_hi]` with an endpoint
swap on wrap-around selects the COMPLEMENT of the legal arc — so
facing up-course, straight-ahead is legal and turning back is
forbidden. True cause of every FGM-I up-course walk (84, 160,
167); it walks even with a perfect compass.

### Attempt 168 — circular U-turn clamp

**Change:** the FGM-I U-turn clamp is rebuilt on the circle: the legal
arc is expressed in bearing space centered on the down-course bearing
(`bearing_dc ± 90°`), so it can never invert. Verified numerically:
facing up-course with a straight-ahead target now commands the robot
to the legal edge (abs −90°); down-course cases unchanged. The
magnetism override clamps into the same window.

**Result:** **14.76 m** (gt), min −5.40. Clamp fix VALIDATED: zero
wrong-way/regress, and the post-abort excursion returned instead of
walking north. Remaining wall-1 blocker, three runs consistent
(166/167/168): the wall commit dies blind at 5.5–5.7 s (13
fallbacks). Traced to an interaction with the slit-plane width fix:
honest widths (~0.6 in a 0.6 m channel) never trip the 1.30
passage-open exit, and when the dog-leg blinds the re-aim, the
smoothed bearing DECAYS TO STRAIGHT — driving the nose into the
bend until DEPTH_LOST. The abort-to-THREAD handoff found no ≥0.85
ray from the jammed pose either.

### Attempt 169 — blind commit re-aim chases the deepest ray

**Change:** when the commit cone has no light, the re-aim bearing now
EMAs toward the DEEPEST ray in the ±40° cone (if it clears
max(0.6, front+0.15)) instead of decaying to straight — the decay drove
the nose into the dog-leg bend for 5.5–5.7 s in attempts 166–168. The
±40° lock, EMA smoothing, and depth-lost abort all still apply; truly
blind pockets still decay and abort.

**Result:** **14.90 m** (gt), min −5.37. The abort-to-THREAD handoff
fired from the jam for the first time (+35°, 0.86 — marginal, no
success), but the wall commit again died blind at 5.3 s. Root
pattern across 166–169: honest widths let grabs fire FAR from the
face (frame 0.76–0.81) — geometries where the slit view dissolves
on approach; the old inflated widths exceeded the 0.90 cap out
there and accidentally suppressed exactly these grabs. Every money
grab in the record was at frame 0.38–0.65.

### Attempt 170 — grab is a near-face move

**Change:** mouth-grab window tightened to frame ∈ [0.35, 0.70) (was
< 0.85). Far grabs (0.70–0.85) lock onto dissolving slit views and die
blind for 5+ s; the ALIGN pipeline (which rotates and re-gates) owns
that range, as it always did before the width fix.

**Result:** **14.82 m** (gt), min −5.37. Near-face grab window
helped the mid-course (clean short clears) but the wall commit
still found a way to blind-abort at 5.4 s via the ALIGN path.
Superseded mid-analysis by three user mandates: never move
up-course, never U-turn, never pinball — with telemetry.

## User mandates (hard rules from attempt 171): NO up-course movement,

## NO U-turns, NO pinballing — enforced and instrumented.

### Attempt 171 — the progress guard

**Changes:** new `_progress_guard(v, w)` applied to EVERY outgoing
command (NAV, COMMIT, SWEEP, THREAD, RECOVER), laser+internal only:

- est heading > 92° off down-course → forward vetoed, rotation forced
  back through the front (reverse allowed there — it moves down-course);
- reverse while heading < 88° off → vetoed (would move up-course); a
  vetoed straight reverse becomes a rotation toward the more open side
  so wedges still resolve;
- steering sign flips within 0.5 s of the previous flip → damped to 0
  (anti-pinball at the command level).
  **Telemetry (user):** every veto logged as GUARD_UTURN / GUARD_BACK /
  GUARD_PINBALL (1 Hz rate limit) and counted in SUMMARY5
  (guard_uturn/guard_back/guard_pinball). THREAD's absolute ray window
  tightened 100° → 90° for coherence with the veto line.

**Result:** **15.56 m** (gt), min −6.00, final in the right recess
(1.13, −6.00). The guard behaved exactly as mandated: 73 GUARD_BACK
vetoes (every recovery reverse became a rotation), 5 GUARD_PINBALL
damps, 0 U-turn events needed, 0 wrong-way/regress — and gt path
efficiency hit 0.82, the best ever recorded. Cost surfaced: with
reverses vetoed, the fixed-duration recovery window keeps rotating
after the nose is already free (~7 s lost across two recoveries).

### Attempt 172 — early recovery release

**Change:** the recovery window releases as soon as front_min > 0.6 —
under the progress guard recovery is a rotation, and its purpose (a
free nose) is observable; the fixed 1–4 s window was pure clock burn
after that point.

**Result:** **14.88 m** (gt), min −5.41 — but the MAG log cracked
the wall-1 case wide open: at the wall the door light WAS visible
(+8…+12°) but read width 0.07–0.17 — below the 0.30 minimum — so
magnetism, grab and sweep door_found ALL rejected a real door.
The honest slit-plane width is geometrically right: from an
oblique/near position only a sliver of light shows through a thick
doorway. Squared-up approaches read 0.47+ and convert (163). The
0.30 minimum was tuned for the old inflated metric; for steering
decisions it now rejects exactly the doors we need to rotate
toward.

### Attempt 173 — steering accepts slivers

**Change:** magnetism and sweep door_found accept light widths ≥ 0.12
(was 0.30) — steering toward a sliver widens it as alignment improves,
ending in a grab. Commit-gating widths (grab 0.30–0.90, ALIGN gates)
unchanged: commitment still requires a full-width door.

**Result:** **14.87 m** (gt), min −5.41. The sliver change got the
grab firing at the wall (−6°, 0.52, frame 0.41) — but its commit
drove LEFT ALONG the wall (x 0.48 → −0.34), not through the door:
an oblique engagement. Every FGM-I wall commit since 166 shows the
same 5.4–5.9 s blind death; the arm delivers angled approaches and
angled commits die, while square engagements (163) clear in
stride. Squareness is the missing precondition, not light
detection.

### Attempt 174 — square grabs only

**Change:** mouth-grab robot-frame window ±35° → ±20°. Magnetism keeps
rotating the nose toward the light, so square engagement is reached
within a few scans anyway — but the commit no longer launches from an
angle it can't survive.

**Result:** **14.89 m** (gt), min −5.37 — last FGM-I-only run;
square grabs cleared cleanly mid-course (0 fallbacks!) but the
wall endgame still drifted to the left corner. FGM-I-only era
final score: 15 runs, mean ≈ 14.4, one gate-3-mouth run (163) —
vs the DWA-arm era's 46% mouth rate. The era paid for itself in
found bugs: inverted U-turn clamp, width-metric collapse/inflation,
oblique-commit fragility, magnetism (kept), progress guard (kept).

## User directive: HYBRID RE-ENABLED (attempt 175 on) — FGM-I in the

## open, DWA when constricted — keeping every mechanism the FGM-I era

## produced (guard, magnetism, THREAD, square grabs, sliver steering,

## slit-plane widths, contact-yaw-in-commits, circular clamp).

### Attempt 175 — hybrid + full toolkit

**Change (launch only):** `use_dwa` true, `hybrid_mode` true. All code
mechanisms retained. Sweep returns to DWA-regime-only gating (its
original design); magnetism applies in the FGM-I regime.

**Result:** **14.91 m** (gt), min −5.37. Hybrid mid-course flawless
(0 fallbacks, clean grabs), wall reached at t=40 — then 15 s of
TIGHT churn (8.8 s TIGHT, 44 guard-uturn rotations, no sweep, no
conversion). Structural: with reverses vetoed, the robot lives in
the front 0.24–0.30 band where the wedge trigger (0.24) never
fires and DWA micro-dances forever. The old escape from this band
WAS the reverse. Rotation recovery is collision-free for a
cylindrical robot, so the trigger can simply come earlier.

### Attempt 176 — earlier rotation recovery

**Change:** `wedge_front_dist` 0.24 → 0.30. Under the progress guard,
recovery is a pure rotation (collision-free for the cylindrical hull),
so triggering earlier costs nothing and deterministically breaks the
0.24–0.30 TIGHT churn band that ate attempt 175's endgame.

**Result:** **14.37 m** (gt), min −5.38 — the 0.30 wedge trigger
REGRESSED channel entry: inside a ~0.55 m channel the rays read
0.25–0.30, so the wedge fires on entry and rotation-recovery backs
the pose out. The robot stood AT the mouth for 18 s in a rotate-
approach-wedge loop (4 wedges, RECOVER 8.7 s, guard_back 83).
The 0.24 threshold existed precisely to allow slit entry —
reverting. The 175 churn band needs a different tool.

### Attempt 177 — wedge revert + wider slot gate

**Changes:** `wedge_front_dist` back to 0.24 (0.30 blocked channel
entry); proactive THREAD's slot gate loosened to max side < 1.2 (was
0.9) so mouth-stands with one semi-open side can thread the churn band
instead of TIGHT-dancing in it.

**Result:** **16.14 m** (gt), min −6.77, final (−0.93, −6.64) — the
wedge revert instantly restored the chain: wall-1 grab-commit
CLEARED in 2.6 s, bay crossed, 10 s standing at gate-3's mouth at
the whistle. THREAD engaged 0 times there — and the record shows
why: every successful mouth entry ray sat at +33…+35°, the exact
clipping edge of THREAD's ±35° search cone.

### Attempt 178 — wider THREAD cone

**Change:** THREAD's deepest-ray search cone ±35° → ±50° (absolute
±90° down-course veto unchanged, all other guards unchanged).

**Result:** **16.34 m** (gt) — NEW ERA BEST, min −6.85, and the
widened cone did exactly what the record predicted: THREAD engaged
on a +36° ray (the old cone's blind spot) and was mid-traversal IN
the gate-3 mouth at the whistle. Chain status: field → gates →
wall-1 grab (2.9 s clear) → bay → mouth THREAD, all functioning.
Pure clock problem: mouth at t≈50, traversal needs ~5-6 s more.
Guard scoreboard: 0 uturn, 17 back-vetoes, 9 pinball damps, zero
wrong-way/regress — the mandates cost nothing today.

### Attempt 179 — governor reach

**Change:** `governor_full_dist` 2.0 → 1.6 m: the commanded cap (0.45,
unchanged per user) is reached with 1.6 m of front clearance instead of
2.0 — dense field scenery rarely offers 2 m, so the field ran at
0.28–0.38 effective. Collision margins unchanged (floor distance 0.5,
wedge 0.24, obstacle costs untouched); the guard keeps movement
monotone.

**Result:** **16.33 m** (gt), min −6.82 — near-identical to 178:
mouth at t≈50, THREAD engaged t≈53 (+38°, front 0.16), still in
the channel at 60 s. The governor tune bought nothing visible
(halfway still ~28.5). In-channel arithmetic: at front 0.16–0.30
the thread crawls at 0.10–0.12 m/s; the ~1 m S-channel needs ~9 s
— more than both the 5 s thread cap and the remaining clock.

### Attempt 180 — thread pace floor + longer leash

**Changes:** THREAD speed floor 0.10 → 0.14 m/s (0.14–0.22 across the
front range) and give-up cap 5 → 8 s — the gate-3 channel needs ~6–7 s
at the new pace and the thread must be allowed to finish it. Success /
ray-loss / scrape exits unchanged.

**Result:** **14.91 m** (gt), min −5.42 — fast arrival (−5.02 at
t=40, halfway 26.8 — the governor DID pay), then 20 s of rotate-
wedge churn at the wall-1 mouth with ZERO tool engagements: from
jammed poses the grab, THREAD (entry needs a 0.85 m ray) and ALIGN
all sit just outside their entry bars while the door is 0.3 m
away. The endgame lottery: square-ish arrivals thread gate 3
(178/179, 16.3 m); sloppy arrivals churn at wall-1 (175/180,
14.9 m).

### Attempt 181 — dwell-relaxed THREAD entry

**Change:** THREAD's entry ray bar drops 0.85 → 0.65 m after 4 s of
face-dwell (the standard desperation escalation). Continuation loss
(0.7), scrape and absolute-direction guards unchanged.

**Result:** **14.95 m** (gt), min −5.41 — right-corner churn this
time (x 1.18 from t=50). The relaxed THREAD bar never fired (no
channel ray exists from the right corner — correctly). Ten runs of
endgame results oscillating 14.4–16.4 all trace to ARRIVAL
DISPERSION: the approach crosses ~2 m of open bay where steering
wanders, and magnetism (frame < 0.95) engages too late to
standardize the pose. Deep runs are exactly the square arrivals.

### Attempt 182 — magnetism owns the approach

**Change:** FGM-I door magnetism engagement range frame < 0.95 → < 1.4:
the nose locks onto a strict-threshold slit from across the approach
bay, standardizing the arrival pose that decides the whole endgame.
All magnetism vetoes unchanged (slit band 0.12–0.90, absolute ≤ 50°,
strict-then-dwell threshold).

**Result:** **14.19 m** (gt), min −5.39 — a SQUARE arrival this time
(x 0.57, grab light −5.5° at frame 0.42, textbook) whose commit
still failed to traverse, followed by mouth-stand churn and a late
regress to −4.65. Extended magnetism didn't hurt (efficiency 0.79,
arrival fast) but didn't convert. Since 163 the outcomes form a
lottery: ~30%% of rolls thread gate 3 (~16.3 m), ~70%% churn at
wall-1 (~14.7 m); a dozen changes since the last controlled
sample (153).

### Attempt 183 — controlled repeat #2

**Rationale:** twelve config/code changes since the last no-change
sample; the endgame is a pose lottery and single runs no longer
attribute cleanly. One repeat at the exact current config to measure
the distribution before further tuning.

**Result:** **14.95 m** (gt), min −5.42 — repeat confirms the
lottery (right-corner churn, RECOVER 10.3 s), and cross-referencing
the eras isolates the post-171 wall-1 regression: the commit's
STALL-NUDGE was a 4 cm reverse-with-turn that unsnagged the hull
from column edges — the progress guard now vetoes its reverse,
leaving a feeble 0.35 rad/s rotation that doesn't change the
contact geometry. Every wall commit that snags now grinds to
death (guard_back 74–87 per run since 171). The 136–157 era's
conversions leaned on exactly that nudge.

### Attempt 184 — guard-compliant stall nudge

**Change:** `door_stall_nudge_w` 0.35 → 0.9, `door_stall_nudge_delay`
1.2 → 0.8 s. With reverses vetoed, the nudge's only unsnagging tool is
rotation — make it strong enough to actually change the contact
geometry (cylindrical hull: rotation is collision-free), and react
sooner.

**Result:** **17.69 m (gt) — ALL-TIME RECORD. gt_y min −8.18 — THE
DEEPEST POSITION EVER RECORDED, through gate 3's channel AND the
−7.5 stretch, standing at the 4th narrowing at the whistle.** The
stall-nudge fix completed the chain: wall-1 grab CLEARED in 2.8 s,
bay at t=45, THREAD into gate 3 at t≈49 (+30°, front 0.17), one
recovery inside, then CRUISE covered −6.9 → −8.18 in the final
9 s. Remaining to completion: ~1.3–1.5 m (the −8.2 narrowing and
the run-out to −9.5). Zero wrong-way, zero regress, guard quiet
(4 uturn rotations).

### Attempt 185 — config frozen, sampling series begins

**Rationale:** the config just reached −8.18 with the full chain
functioning — 1.3 m short of completion. Analysis now finds only
second-order effects; the endgame outcome is a pose lottery whose best
tail crosses the course. Config frozen; repeated runs until a
completion lands or a new systematic blocker appears in the data.

**Result:** **14.92 m** (gt), min −5.42 — wall-1 mouth-stand roll
(slow field, halfway ~29.5; mouth churn from t≈45). Sample 1 of
the frozen config: low tail.

### Attempt 186 — sampling (frozen config)

**Result:** **16.31 m** (gt), min −6.81, final (−1.03, −6.78) —
gate-3 doorstep at the whistle. Sample 2: high-middle of the
distribution.

### Attempt 187 — sampling (frozen config)

**Result:** **15.26 m** (gt), min −6.29 — reached the bay fast
(halfway ~26.5) but drifted into the left corner of the gate-3
face late. Sample 3: middle of the distribution.

### Attempt 188 — sampling (frozen config)

**Result:** **14.11 m** (gt), min −5.48 — fast field (−4.97 at t=40)
but the wall endgame slid to the left corner with a small regress.
Sample 4: low tail. Frozen-config distribution so far: 17.69 /
16.31 / 15.26 / 14.92 / 14.11.

### Attempt 189 — sampling (frozen config)

**Result:** **16.30 m** (gt), min −6.76 — gate-3 doorstep at the
whistle (−1.10, −6.76). Sample 5. Distribution: 17.69 / 16.31 /
16.30 / 15.26 / 14.92 / 14.11 — mean 15.8, mouth-or-deeper rate
3/6.

### Attempt 190 — sampling (frozen config)

**Result:** **16.34 m** (gt), min −6.83 — best mouth split yet
(−6.83 by t=50), THREAD active in the channel at the whistle.
Sample 6. Distribution: 17.69 / 16.34 / 16.31 / 16.30 / 15.26 /
14.92 / 14.11.

### Attempt 191 — sampling (frozen config)

**Result:** **14.49 m** (gt), min −5.38 — wall-1 roll (swept left,
ended right). Sample 7. Distribution: 17.69 / 16.34 / 16.31 /
16.30 / 15.26 / 14.92 / 14.49 / 14.11 — mean 15.7, wall-1
conversion rate 4/8.

### Attempt 192 — sampling (frozen config)

**Result:** **10.21 m** (gt), min −0.65 — the old early-field trap
(pinned around y≈5.5 from t=20–40, 7 wedges), escaped at t≈40 and
ran the rest cleanly but the clock was gone. Sample 8: trap tail
(~1-in-8, consistent with its historical rate).

### Attempt 193 — sampling (frozen config)

**Result:** **14.95 m** (gt), min −5.37 — wall-1 right-corner roll.
Sample 9. Nine-run distribution: 17.69 / 16.34 / 16.31 / 16.30 /
15.26 / 14.95 / 14.92 / 14.49 / 14.11 / 10.21 — mean 15.1,
gate-3-or-deeper 4/10, one trap.

### Attempt 194 — sampling (frozen config)

**Result:** **16.34 m** (gt), min −6.83 — THREAD in the gate-3
channel at the whistle again (mouth by t=50). Sample 10. The
distribution is stable: mean ≈ 15.2, gate-3 threading 5/11, one
completion-grade tail (184).

### Attempt 195 — sampling (frozen config)

**Result:** **17.69 m (gt) — TIES THE ALL-TIME RECORD, min −8.18 —
and a better run than 184:** gate 3 crossed by t≈48 (−7.76 at
t=50, crossing at x≈−0.9), EIGHT seconds spent at the 4th
narrowing. Death mode: a sweep crawl (0.18 m/s even at front 0.18)
drove the nose into the face until hull contact (front 0.121,
immobile t=55–60) — where rotation recovery stalls on friction and
reverses are banned. Nobody has ever crossed the 4th narrowing;
its opening location is unknown — but the robot now ARRIVES there
with time to search, if it stops pinning itself.

### Attempt 196 — sweeps never pin

**Change:** sweep forward speed is zero below front 0.26 (steering
continues) — a crawling sweep hull-pinned the record run against the
4th-narrowing face, the one failure the guard cannot undo.

**Result:** **14.97 m** (gt), min −5.42 — wall-1 right-corner roll
(the lottery's low side; the pin fix wasn't exercised).

### Attempt 197 — sampling

**Result:** **16.34 m** (gt), min −6.84 — mouth by t=50, THREAD in
the channel at the whistle. The gate-3 arrival is now routine;
the whistle keeps catching the thread mid-channel.

### Attempt 198 — sampling

**Result:** **14.92 m** (gt), min −5.42 — wall-1 right-corner roll,
slow field (halfway ~29.3). Sampling verdict after 12 runs of the
frozen config: gate-3 arrival at t≤48 completes-or-nearly (184,
195); arrival ≥50 dies mid-thread; slow-field rolls die at wall-1.
Field pace is the binding constraint within current mandates.

### Attempt 199 — sprint conditions widened (caps untouched)

**Change:** the conditional sprint's ENTRY conditions widen —
`sprint_front_dist` 1.5 → 1.2, `sprint_bearing_deg` 18 → 26,
`sprint_max_w` 0.5 → 0.7. Both speed caps stay at the user-set values
(max_vel 0.45, sprint_vel 0.60); the sanctioned sprint just fires on
more of the genuinely clear straights. Expected effect: 2–3 s off the
field, converting t≈50 mouth arrivals into t≈47 ones.

**Result:** **14.96 m** (gt), min −5.46 — the pace change delivered
(−5.01 at t=40, among the fastest wall arrivals ever) but the
wall-1 coin landed tails: 20 s of right-corner churn. Config with
widened sprint retained — the fast tail now has completion room.

### Attempt 200 — aborted by user mid-run: door-algorithm rethink

**User direction:** stop sampling; improve the door path algorithm.
References reviewed: F1TENTH disparity extender (edge inflation +
speed-on-steering-angle) and VFH/VFH+. Verdict: VFH adds nothing;
disparity extension is already our FGM front-end, but the audit found
two places it was never applied — THREAD searches RAW rays (hence the
hull scrapes and channel weave), and COMMIT ignores steering angle
(hence oblique entries at full speed becoming 5.5 s blind grinds).
A reachability mini-grid planner for door targeting was proposed and
remains on the table.

### Attempt 201 — F1TENTH steals (edge-inflation everywhere + speed-on-angle)

**Changes (from the user's disparity-extender references):**

- **THREAD searches the disparity-extended scan** (computed once, early
  in the callback): the deepest inflated ray is the CENTER of a
  passable corridor, never a doorframe-hugging sliver. Directly targets
  the 0.11 hull-scrape aborts and the in-channel weave.
- **COMMIT speed scheduled on steering angle**: linear speed scales
  with cos² of the commanded bearing (floor 0.55×) — squared entries
  run full commit speed, angled entries self-slow. Targets the 5.5 s
  oblique blind-grind deaths.
- VFH reviewed and rejected (agreed with user's assessment: ~2× compute
  of FGM, no advantage for this corridor).

**Result:** **14.91 m** (gt), min −5.40 — right-corner roll (the
steals weren't exercised: no THREAD, commits mid-course only).
Superseded same-hour by the arbiter build below.

### Attempt 202 — grid-BFS reachability + weighted steering arbiter

**Changes (user-approved proposal + user-directed tandem matrix):**

- **`_grid_door_bearing`**: 60×60 × 6 cm robot-centric occupancy grid
  per scan (rays rasterized, obstacles inflated 18 cm), BFS through
  free space, steer toward the frontier with the greatest reachable
  down-course progress (≥ 0.35 m, ±90° veto). Pocket rays are BFS dead
  ends; doors are corridors the flood passes through — topology instead
  of threshold wars. Synthetic tests: realistic 0.6 m door found and
  routed (3.8 ms/call); grazing slivers abstain (magnetism's job).
- **Weighted decision matrix**: per scan, candidates FGM gap (always),
  MAG light and GRID frontier (door regime) scored on progress (0.45),
  clearance (0.25), continuity (0.15), confidence (0.15) + 0.06
  incumbent bonus. Winner takes the bearing; candidates within 25° of
  the winner blend score-weighted ("algo 1 + algo 2"), disagreeing ones
  are excluded. ARBITER log line at 1 Hz. FGM-I's standalone magnetism
  override retired (absorbed into the matrix, both arms now share it).
- Switches: `use_arbiter`, `use_grid_door` (both default true).

**Result:** **0.00 m** — the arbiter block was inserted before
frame_min/eff_depth_thr are computed; NameError killed every scan
callback. Caught immediately (0.00 m, empty state table, zero
ARBITER lines). Block relocated to just before the arm split where
all dependencies are in scope.

### Attempt 203 — arbiter relocated (ordering fix)

**Change:** the weighted-matrix block moved after the door-machinery
section (frame_min, eff_depth_thr in scope), immediately before the
hybrid arm split. No logic changes.

**Result:** **14.89 m** (gt), min −5.34. The arbiter runs correctly
(32 decisions, GRID won 15, blend visible in the logs, cb 15.4 ms
avg / 59.6 max — within the 75 ms budget) — but it also arbitrated
FIELD squeezes (frame < 1.5 is common among the scatter), where
tuned FGM was never the problem, and the field ran slow (halfway
~31 s). One GRID pick at +124° also shows frontier proposals need
the gates-region context.

### Attempt 204 — arbiter gates-region gate

**Change:** MAG/GRID candidates join the matrix only after ≥ 2 cleared
doors (the standard gates-region signal); the field runs pure FGM as
before.

**Result:** **15.76 m** (gt), min −6.79 — first arbiter-assisted
wall-1 crossing: on approach all three candidates converged (MAG
+15/GRID +6/FGM +2 blending to +5°) and the door went through via
ALIGN in stride. Endgame drifted to the right recess of the gate-3
face, where GRID went silent: its 1.8 m half-span cannot reach the
left mouth ~1.9 m away laterally.

### Attempt 205 — bigger grid

**Change:** mini-grid half-span 1.8 → 2.4 m (80×80 × 6 cm), ray cap
2.9 → 3.4 m — the gate-3 left mouth must be inside the grid from the
right recess. ~7 ms/call expected, still within budget.

**Result:** **14.91 m** (gt), min −5.42 — right-corner roll, and the
ARBITER log exposed a grid flaw: frontiers BEHIND the robot (−135°,
+180°) qualified — the progress test is absolute (d·cosθ > 0.35 m),
so a far-but-sideways frontier passes; the promised ±90° angular
veto was never actually implemented, and one backward GRID proposal
won a round (guard contained it, but steering was wasted).

### Attempt 206 — angular vetoes, actually implemented

**Changes:** (1) grid frontiers must lie within ±75° of down-course
(the veto the docstring promised); (2) arbiter drops non-FGM candidates
pointing beyond 85° of down-course before scoring. FGM always remains
as the fallback candidate.

**Result:** **14.83 m** (gt), min −5.39 — right-corner churn: t=45
to whistle in TIGHT/RECOVER cycles at (1.2, −5.3), 5 wedges, and
the sweep never got a window between recovery preemptions. The
angular vetoes held (no backward proposals). Pattern: repeated
wedge-recovery in one spot IS the corner signature — recovery
should hand over to the wall-walker instead of cycling.

### Attempt 207 — recovery-loop circuit breaker

**Change:** 3+ wedge-recoveries within 8 s (gates region, no active
thread) forces a SWEEP engagement toward the more open side instead of
another recovery cycle — corners are for walking out of, not rotating
in.

**Result:** **16.13 m** (gt), min −6.76 — strong pace (gate-3 face
by t=50), forced-sweep fired once, THREAD 1-for-2 — then the face
sweep chose RIGHT into the recess. Cumulative evidence: ~12
right-side excursions, zero crossings; all 5 crossings ever went
left. Coverage memory forgets a fruitless direction after 6 s, so
consecutive sweeps at the same face keep re-choosing the same
side.

### Attempt 208 — 20-second coverage memory

**Change:** the fruitless-sweep direction memory window extends 6 →
20 s: a direction that failed at this face stays penalized for the
whole face episode, so the second sweep always tries the other way.

**Result:** **17.21 m (gt) — second-best ever, min −7.70.** Third
gate-3 crossing of the era; both new mechanisms fired (forced
sweep ×1, 20 s memory). The crossing took ~8 s with wedge-recovery
cycles interrupting mid-channel while THREAD idled in its 5 s
give-up lockout. Ensemble-era trend: 14.8 → 15.8 → 16.1 → 17.2.

### Attempt 209 — shorter THREAD give-up lockout

**Change:** THREAD give-up lockout 5 → 2 s: after a timed-out thread,
the next in-channel bend can re-engage a thread instead of falling to
wedge-recovery cycles for 5 s.

**Result:** **15.24 m** (gt), min −6.66 — reached the gate-3 face at
t=48 but the face sweep drifted back up-right (−6.66 → −5.72 at
whistle). Lottery middle.

### Attempt 210 — sampling (ensemble config)

**Result:** **14.87 m** (gt), min −5.35 — wall-1 left-corner roll.
Ensemble-era ten-run tally: 17.21 / 16.13 / 15.76 / 15.24 / 14.9×4
/ 14.8 — mean ≈ 15.5, crossings 1/8 since the arbiter, all-time
best 17.69 twice.

### Attempt 211 — sampling (ensemble config)

**Result:** **16.19 m** (gt), min −6.68 — gate-3 face by t=48, ended
in the right recess. Lottery high-middle.

### Attempt 212 — sampling (ensemble config)

**Result:** **16.14 m** (gt), min −6.71 — gate-3 face by t=48,
sweep went right at the whistle. High-middle again; the face-side
coin remains the decider.

### Attempt 213 — sampling (ensemble config)

**Result:** **16.37 m** (gt), min −6.86 — at the gate-3 mouth
(−0.69, −6.85) at the whistle, cleanest contact profile of the era
(1 wedge, slip 0.33). Loop PAUSED here by user for an observation
run.

### Attempt 214 — user-directed: commit more

**User verdict from the watched run:** the robot slid past the wall-1
door without committing. **Changes:** grab window re-widened — frame
[0.35, 0.85) (was 0.70 cap) and robot-frame bearing ±28° (was ±20°).
Both restrictions predate the cos²-on-bearing commit speed scheduling
(201), which now makes exactly those angled/far grabs self-slowing;
strict depth, width band, absolute veto and 2-scan debounce unchanged.

**Result:** **15.77 m** (gt), min −6.67 (watched run). Wall-1
COMMITTED this time (no drive-by — through by t≈46, THREAD
1-for-1 in the channel), reached the gate-3 face by t=50, endgame
drifted right to (1.20, −6.22). The commit-more change did its
job at wall-1; the gate-3 side choice took the right recess.

### Attempt 215 — user-directed: slower + sharper post-door turns

**User verdict from the watched run:** turns after door commits are too
wide, and path efficiency is off — slow down. **Changes (launch):**
`max_vel` 0.45 → 0.40, `sprint_vel` 0.60 → 0.50 (slower = tighter turn
radius everywhere), `fgm_p_gain` 1.6 → 2.1 (sharper FGM-I steering),
`heading_weight` 2.00 → 2.40 (DWA corrects heading harder after
commits).

**Result:** **14.99 m** (gt), min −5.44 (watched). Clean, tight
driving (slip 0.31, efficiency 0.76 at the lower speed) but the
wall-1 endgame drifted right past the door into the corner again
(x 1.31 by t=50).

### Attempt 216 — corner-door deadlock fix

**From the user's screenshot:** the gate openings are FLUSH against the
corridor side walls (corner doors), and the robot pinned at the corner
beside one with the light visible. Deadlock found: light visible →
proactive THREAD defers to the grab; grab blocked by its 0.35 standoff
floor at the jam → nothing acts, recovery cycles. **Changes:** THREAD
may act despite a visible light when the grab is impossible
(frame < 0.35), and after 4 s of dwell the slot gate accepts one-sided
slots (min side < 0.5, no max condition) — corner doors have wall on
one side only.

**Result:** **2.92 m** (gt) — watched; worst trap run of the era:
37.8 s of rotation-recovery cycling at an early-field corner
(13 wedges, slip 3.67 m). The deadlock fix wasn't the problem (it
requires the gates region); the problem is STEERING INTO concave
corners at all — user directive: write corner avoidance.

### Attempt 217 — corner avoidance (user-directed)

**Changes:** new `_corner_ahead` detector — a bearing whose inflated
fan (±25°) has no ray ≥ 1.0 m dead-ends in a concave corner. Applied:
(1) arbiter drops FGM candidates that dead-end (MAG/GRID lights exempt
— door machinery knows better); if everything is vetoed, an ESC
candidate steers to the deepest fan in the legal arc; (2) FGM-I checks
its clamped bearing and re-aims at the deepest legal fan when
corner-bound (CORNER_AVOID events); (3) `corner_avoid=N` in SUMMARY5.

**Result:** _pending_attempt_217_

### Attempt 218 — absolute forward-only + field topology guard

**User (watching, emphatic): the robot must NEVER move backward — it
was still visibly backing up.** Audit found two guard holes: reverse
was permitted while facing up-course, and the 88–92° boundary window
passed reverses through. **Changes:**

- **Guard: zero reverse, no exceptions** — any v < 0 becomes 0 (+
  rotation toward the open side); the up-course exception is deleted.
- **Violation telemetry (user):** EVENT BACKWARD_MOTION on any actual
  encoder reverse (odom_v < −0.02, 3 scans) and EVENT UPCOURSE_MOTION
  on sustained gt_y rise (> 0.06 m/1 s, measurement-only); both
  counted in SUMMARY5 (backward_motion / upcourse_motion).
- Corner detector strengthened (±35° fan, 1.2 m threshold — old one
  fired zero times in 217).
- Field topology guard: grid-BFS reachability joins the arbiter in the
  FIELD when frame < 1.2 — pocket mouths are closed topology to the
  flood, so they never attract (pins prevented, per the upheld ban).
  Headless loop resumes until completion.

**Result:** _pending_attempt_218_

### Attempt 219 — in_slot NameError fix (runs 216–218 were invalid)

**Post-mortem:** the attempt-216 patch accidentally deleted the
`in_slot` definition while still referencing it — the scan callback
crashed with NameError on most scans from 216 onward. Runs 216 (2.92),
217 (3.37) and 218 (4.22) were all crippled-controller runs, including
both disasters the user watched. Definition restored; pyflakes added
to the pre-run checklist.

**Result:** **16.09 m** (gt), min −6.75 — first VALID run of the
full stack: zero callback errors, corner_avoid fired 8×, ONE wedge
all run (vs 13–17 in the crippled runs), backward_motion 0,
gate-3 face at the whistle. The stack works; loop continues.

### Attempt 220 — loop (full stack)

**Result:** **14.95 m** (gt), min −5.44 — wall-1 right-corner roll
(guard_back 78 = heavy stall-nudge vetoing at the wall). Mandate
compliance clean (backward 0, upcourse 0, slip 0.11 — lowest
ever).

### Attempt 221 — loop

**Result:** **14.89 m** (gt), min −5.37 — wall-1 left-corner roll,
12 commit fallbacks at the wall. Valid-stack tally: 16.09 / 14.95
/ 14.89.

### Attempt 222 — loop

**Result:** **15.00 m** (gt), min −5.45 — wall-1 right-corner roll
again (corner_avoid fired 22× upstream but the wall approach still
drifted right past the door). Tally: 16.09 / 15.00 / 14.95 / 14.89.

### Attempt 223 — loop

**Result:** **16.30 m** (gt), min −6.78 — at the gate-3 mouth at the
whistle; gt path efficiency 0.84 (record). Tally: 16.30 / 16.09 /
15.00 / 14.95 / 14.89.

### Attempt 224 — loop

**Result:** **14.94 m** (gt), min −5.42 — wall-1 right-corner roll.
The wall-1 coin is the whole distribution now.

### Attempt 225 — loop

**Result:** **14.96 m** (gt), min −5.44 — wall-1 right-corner roll.
Valid-stack tally: 16.30 / 16.09 / 15.00 / 14.96 / 14.95 / 14.94 /
14.89 — extremely tight distribution; the wall-1 door conversion
rate has dropped since the forward-only hardening (the stall
nudge's last physical tooth went with the reverse ban).

### Attempt 226 — loop

**Result:** **15.89 m** (gt), min −6.35 — slowest field of the era
(halfway ~37 s) followed by a completion-grade finish: 6.5 m in
the last 20 s straight through wall-1 and the bay. With a median
field this run crosses gate 3 with time to spare.

### Attempt 227 — loop

**Result:** **16.12 m** (gt), min −6.57 — mid-field stall episode
(t≈20) cost ~6 s, then a fast back half through wall-1 to the
gate-3 approach at the whistle.

### Attempt 228 — loop

**Result:** **14.52 m** (gt), min −5.34 — wall-1 right-corner roll.

### Attempt 229 — loop

**Result:** **17.34 m (gt) — second-best ever, min −7.82, ZERO
wedges.** Crossed wall-1 and gate 3 in stride and worked the 4th
narrowing's right side at the whistle. The tail keeps reaching.

### Attempt 230 — light memory + classifier inversion fix

**From the "why the right corner" diagnosis:** four systems all vote
right at wall-1 (FGM width, centering bias, MAG sliver abstention, and
an INVERTED wall classifier — the door's own through-depth makes its
side read "open pocket"). **Changes:** (1) light memory: a door-grade
light seen at a face is remembered (absolute direction, 10 s); sweep
direction walks TOWARD the sighting; (2) classifier counts wall-band
rays (0.9–1.8 m) as searchable wall — ≥25% wall-band means never
"pocket" — so through-slit spikes stop inverting the choice.

**Result:** **14.04 m** (gt), min −5.36 — wall-1 roll; the sweep
went left this time (light memory working) but a late regress
gave back 0.8 m. Coin still tails.

### Attempt 231 — loop

**Result:** **15.53 m** (gt), min −6.72 — through wall-1 to the
gate-3 face by t≈48, endgame drifted right (−6.0 final).

### Attempt 232 — loop

**Result:** **14.77 m** (gt), min −5.26 — mid-field slow patch
(t≈25–35 around a stub), arrived at wall-1 at t≈57; no time.
No trap catastrophe though: corner avoidance + guard kept it
moving (14.8 from a bad roll that used to be a 3–10 m run).

### Attempt 233 — loop

**Result:** **14.68 m** (gt), min −5.25 — wall-1 right-corner roll
(guard_back 91: heavy stall-veto churn at the wall).

### Attempt 234 — loop

**Result:** **14.90 m** (gt), min −5.36 — slow-ish field, wall-1 at
t≈52, right corner at whistle.

### Attempt 235 — loop

**Result:** **14.93 m** (gt), min −5.39 — wall-1 right-corner roll.
Planner-improvement plan drafted (grid-guided commits → grid as
endgame pilot → confidence-gated speed restore, pending user).

### Attempt 236 — grid-guided commits (user: "do this")

**Changes:** `_grid_plan` now returns the full BFS path (robot-frame
waypoints); COMMIT pure-pursues its ~0.6 m lookahead point every scan
— replanned per scan, it S-turns through dog-legs the light cannot see
past. While a path exists the blind-abort streak resets (topology
beats line-of-sight); when the grid abstains, the light re-aim /
deepest-ray fallback is unchanged. `grid_commit_scans` added to
SUMMARY3. Synthetic check: 41-point path through a 0.6 m door,
bearing +6.3°.

**Result:** **7.82 m** (gt) — integration flaw: the grid path goes
to ANY down-course frontier, so a mid-gate commit was pure-pursued
sideways toward a different frontier with its blind-abort disabled
(streak reset on any path), wedging at (1.2, 2.6) for 30 s. Grid
guidance must agree with the committed door, not replace it.

### Attempt 237 — grid guidance must agree with the door

**Change:** commit accepts grid guidance only when the path's lookahead
bearing is within 35° of the current commit bearing; otherwise the
light/deepest-ray logic (and its blind aborts) run untouched.

**Result:** **6.55 m** (gt) — second catastrophic pin in two
grid-commit runs (frozen at (0.64, 2.98) for 30 s, slip 8.73 m),
at a mid gate the light-guided commit has cleared for 100+ runs.
Root cause: BFS shortest paths HUG THE INFLATION BOUNDARY —
pure-pursuit of a corner-cutting path grazes the door frame that
the light-center aim naturally cleared. Grid-commit guidance needs
clearance-weighted pathing (distance-transform costs), not raw
BFS. Disabled behind `use_grid_commit=false` until built; the
grid stays in the arbiter (approach/veto), where it has been
positive.

### Attempt 238 — grid commits off, proven commit restored

**Change:** `use_grid_commit` switch (default false) — commits steer by
light-center again; grid keeps its arbiter roles. Loop resumes on the
235-era behavior plus all fixes since.

**Result:** **16.32 m** (gt), min −6.78 — behavior restored: through
wall-1 in stride, at the gate-3 mouth at the whistle.

### Attempt 239 — loop

**Result:** **14.86 m** (gt), min −5.30 — wall-1 right-corner roll.

### Attempt 240 — loop

**Result:** **14.87 m** (gt), min −5.35 — arrived at wall-1 at t≈49
(slow field), left-side stand at whistle. One wedge, clean run,
just late.

### Attempt 241 — loop

**Result:** **14.94 m** (gt), min −5.40 — wall-1 right-corner roll.

### Attempt 242 — loop

**Result:** **14.86 m** (gt), min −5.32 — wall-1 right-corner roll.

### Attempt 243 — loop

**Result:** **16.32 m** (gt), min −6.82 — through wall-1, at the
gate-3 mouth at the whistle

### Attempt 244 (queued during 243) — light memory accepts any sliver

**Change:** light-memory recording threshold 0.12 → 0.05 m width — at
the wall the door sliver reads 0.07–0.17 obliquely and half the
sightings were never remembered, so the sweep kept defaulting right
(242's endgame). Memory is steering-preference only; all commit gates
unchanged.

**Result:** **14.90 m** (gt), min −5.39 — slow field, wall-1 arrival
t≈50, centered stand at whistle (−0.5, −5.39); no time to convert.

### Attempt 245 — loop

**Result:** **14.82 m** (gt), min −5.32 — wall-1 arrival t≈51,
left-corner at whistle. Late-arrival roll.

### Attempt 246 — loop

**Result:** **7.65 m** (gt) — early-field trap at (0, 7.4) for 30 s
(TIGHT/RECOVER cycles at the first squeeze), escaped at t≈45.
Trap-tail roll.

### Attempt 247 — loop

**Result:** **15.21 m** (gt), min −5.66 — slow field (mid-course
meander t≈18–32), then a clean charge: entered the wall-1 channel
AT the whistle.

### Attempt 248 — loop

**Result:** **16.15 m** (gt), min −6.63 — slow field, then a
completion-pace back half (0 → −6.63 in the last 20 s, one wedge
all run): the wall-1 door and bay in stride, gate-3 face at the
whistle.

### Attempt 249 — loop

**Result:** **14.87 m** (gt), min −5.37 — fast to the wall (t≈44)
but the door didn't convert; sweeping left at the whistle.

### Attempt 250 — loop

**Result:** **14.95 m** (gt), min −5.41 — wall-1 right corner at
whistle after a slow-ish field.

### Attempt 251 — loop

**Result:** **14.85 m** (gt), min −5.34 — wall-1 left side at
whistle.

### Attempt 252 — loop

**Result:** **14.74 m** (gt), min −5.25 — late arrival, right corner
at whistle.

### Attempt 253 — loop

**Result:** **14.93 m** (gt), min −5.42 — wall-1 right corner at
whistle.

### Attempt 254 — loop

**Result:** **16.27 m** (gt), min −6.72 — through wall-1 in stride,
right recess of the gate-3 face at whistle.

### Attempt 255 — loop

**Result:** **14.76 m** (gt), min −5.25 — slow field, arrival t≈53,
right side at whistle.

### Attempt 256 — loop

**Result:** **14.79 m** (gt), min −5.32 — wall arrival t≈45,
sweeping left at whistle.

### Attempt 257 — loop

**Result:** **14.28 m** (gt), min −5.37 — wall stand, small late
regress to −4.77.

### Attempt 258 — loop

**Result:** **16.06 m** (gt), min −6.70 — gate-3 right recess at
whistle.

### Attempt 259 — "do all": medial-axis planner + grid commits + bounded sprint

**Changes (user approved all):**

- **Clearance-weighted grid planning:** multi-source BFS clearance
  field + Dijkstra with cost 1 + 6/clearance — paths follow the medial
  axis. Synthetic check: door-plane crossing pulled from the frame edge
  (y 0.10 → 0.24), 4.8 ms/plan.
- **Grid-guided commits re-enabled** (`use_grid_commit` true): centered
  pursuit through doors; the 35° door-agreement gate from 237 stays.
- **Bounded sprint restore:** `sprint_vel` 0.50 → 0.62, certification
  tightened (front > 1.4, bearing < 14°, |w| < 0.5) — pace returns only
  on certified-clear straights; base max_vel stays at the user's 0.40.

**Result:** **14.90 m** (gt), min −5.38 — no pins (medial-axis fix
held: zero catastrophes, backward 0), but commit_time ballooned to
19.4 s with 3 timeouts — centered pursuits run long. One more
sample before judging the package.

### Attempt 260 — loop (package sample 2)

**Result:** **14.92 m** (gt), min −5.38 — the package pays mid-
course: wall arrival at t=40 (fast lane restored, 4/5 commits
cleared, no pins). Wall-1 itself still didn't convert (left slide
to the corner by t=50).

### Attempt 261 — loop

**Result:** **14.92 m** (gt), min −5.42 — fast to the wall (t≈44),
left-corner slide again.

### Attempt 262 — loop

**Result:** **14.90 m** (gt), min −5.40 — fast to the wall (t≈43),
left slide again. Wall-1 conversion has gone 0-for-9 since the
package landed despite ideal arrivals; next change must attack
the wall-1 door engagement itself.

### Attempt 263 — frontier pinning during commits

**Change:** `_grid_plan` accepts a preferred direction; during COMMIT
the frontier search is pinned within ±35° of the committed door
bearing, so the pursuit target can't flip between the door channel and
the bay scan-to-scan (attempt 262's 6.6 s TIMEOUT wobble). Arbiter
planning calls remain unpinned.

**Result:** **14.76 m** (gt), min −5.35 — commits sharp again (6/7
cleared) and fast arrival (t≈43); the wall-1 commit still ran to
a timeout and the run ended centered at the wall.

### Attempt 264 — loop

**Result:** **14.95 m** (gt), min −5.41 — left-corner slide at
wall-1.

### Attempt 265 — loop

**Result:** **14.84 m** (gt), min −5.34 — centered wall stand at
whistle.

### Attempt 266 — loop

**Result:** **14.88 m** (gt), min −5.37 — at the door (0.44, −5.37)
at the whistle, unconverted.

### Attempt 267 — loop

**Result:** **14.52 m** (gt), min −5.37 — wall stand, ended right
of the door.

### Attempt 268 — loop

**Result:** **14.95 m** (gt), min −5.44 — whistle caught it entering
the wall-1 door mouth (0.53, −5.44) after a slow mid-course.

### Attempt 269 — loop

**Result:** **14.88 m** (gt), min −5.37 — wall stand left of the
door at whistle.

### Attempt 270 — loop

**Result:** **14.94 m** (gt), min −5.42 — left corner at whistle.

### Attempt 271 — loop

**Result:** **14.88 m** (gt), min −5.35 — fast to the wall (t=42),
left slide again.

### Attempt 272 — loop

**Result:** **14.92 m** (gt), min −5.37 — right corner at whistle.

### Attempt 273 — loop

**Result:** **10.36 m** (gt), min −0.84 — pinned at a mid-course
right corner (1.26, −0.7) from t≈37 to the whistle (10 wedges).
Mid-course trap tail.

### Attempt 274 — loop

**Result:** **14.79 m** (gt), min −5.31 — centered wall stand at
whistle.

### Attempt 275 — loop

**Result:** **14.91 m** (gt), min −5.37 — right corner at whistle.

### Attempt 276 — loop

**Result:** **16.20 m** (gt), min −6.69 — through wall-1, gate-3
face at t=48, right recess at whistle.

### Attempt 277 — loop

**Result:** **14.87 m** (gt), min −5.33 — left corner at whistle.

### Attempt 278 — loop

**Result:** **14.88 m** (gt), min −5.36 — left corner at whistle.

### Attempt 279 — loop

**Result:** **14.95 m** (gt), min −5.46 — 18 s at the door mouth
grinding, no conversion.

### Attempt 280 — loop

**Result:** **14.87 m** (gt), min −5.38 — left slide at whistle.

### Attempt 281 — loop

**Result:** **14.81 m** (gt), min −5.37 — left of door at whistle.

### Attempt 282 — loop

**Result:** **16.09 m** (gt), min −6.75 — through wall-1 (t≈49),
right recess of the gate-3 face at whistle.

### Attempt 283 — loop

**Result:** **14.71 m** (gt), min −5.30 — right of door at whistle.

### Attempt 284 — loop

**Result:** **14.86 m** (gt), min −5.34 — left corner, sweeping back
at whistle.

### Attempt 285 — loop

**Result:** **14.88 m** (gt), min −5.37 — left of door at whistle.

### Attempt 286 — loop

**Result:** **16.28 m** (gt), min −6.77 — ZERO wedges; through
wall-1 late (t≈52), reached the gate-3 face by whistle.

### Attempt 287 — loop

**Result:** **14.43 m** (gt), min −5.41 — left corner, small
regress at whistle.

### Attempt 288 — loop

**Result:** **14.53 m** (gt), min −5.34 — swept left corner then
right at whistle.

### Attempt 289 — loop

**Result:** **14.91 m** (gt), min −5.39 — left corner at whistle.

### Attempt 290 — loop

**Result:** **14.19 m** (gt), min −5.28 — late arrival, small
regress at whistle.

### Attempt 291 — loop

**Result:** **14.97 m** (gt), min −5.47 — whistle caught it inside
the wall-1 door mouth (0.36, −5.47).

### Attempt 292 — loop

**Result:** **14.94 m** (gt), min −5.43 — right of door at whistle.

### Attempt 293 — loop

**Result:** **14.85 m** (gt), min −5.34 — centered wall stand at
whistle.

### Attempt 294 — loop

**Result:** **14.95 m** (gt), min −5.42 — right corner at whistle.

### Attempt 295 — loop

**Result:** **16.28 m** (gt), min −6.79 — through wall-1 (t≈47),
sweeping the gate-3 face at whistle.

### Attempt 296 — loop

**Result:** **14.78 m** (gt), min −5.32 — swept both corners at the
wall, no conversion.

### Attempt 297 — loop

**Result:** **15.63 m** (gt), min −6.68 — through wall-1 (t≈47),
right recess at whistle.

### Attempt 298 — loop

**Result:** **16.15 m** (gt), min −6.66 — through wall-1 (t≈46),
ALIGNing at the gate-3 face at the whistle.

### Attempt 299 — loop

**Result:** **14.92 m** (gt), min −5.40 — left corner, walked back
to the door at whistle (0.74, −5.40).

### Attempt 300 — loop

**Result:** **14.99 m** (gt), min −5.45 — right of door at whistle.
Attempt 300 milestone: era stats — all-time best 17.69 (×2, gt_y
−8.18), completion threshold ~19 m; recent mode ≈ 14.9 (wall-1),
high tail ≈ 16.2 (gate-3 face).

### Attempt 301 — loop

**Result:** **15.63 m** (gt), min −6.11 — slow left-lane mid-course,
then a charge through wall-1 at the whistle (−6.11 moving).

### Attempt 302 — loop

**Result:** **14.85 m** (gt), min −5.35 — fast arrival (t=41),
centered stand, no conversion.

### Attempt 303 — loop

**Result:** **15.58 m** (gt), min −6.04 — through wall-1 late
(t≈52), moving down at the whistle.

### Attempt 304 — loop

**Result:** **14.91 m** (gt), min −5.38 — late arrival, left corner
at whistle.

### Attempt 305 — loop

**Result:** **14.83 m** (gt), min −5.30 — mid-course stall episode,
arrived at wall t≈53, left side at whistle.

### Attempt 306 — loop

**Result:** **17.58 m (gt) — near-record, min −8.06.** Crossed
wall-1 (t≈43), swept to gate-3's mouth and THROUGH it, working the
4th narrowing at (−0.80, −8.06) at the whistle. Second-deepest run
ever; the tail keeps landing.

### Attempt 307 — loop

**Result:** **15.40 m** (gt), min −6.75 — reached the gate-3 face,
drifted right late. LOOP PAUSED by user for an observation run.

### Attempt 308 — corner avoidance removed (user), loop of 5 begins

**Change (user-directed):** both corner-avoidance behaviors deleted —
the arbiter's FGM dead-end veto and the FGM-I deepest-fan re-aim
(`_corner_ahead` remains but unused; corner_avoid counter will read 0).
Five-attempt loop (308–312), then pause.

**Result:** **14.77 m** (gt), min −5.37 — left corner then back to
the door at whistle.

### Attempt 309 — loop (2/5)

**Result:** **14.88 m** (gt), min −5.42 — left of door at whistle.

### Attempt 310 — loop (3/5)

**Result:** **14.91 m** (gt), min −5.37 — wall-1 stand
(run_20260711_145716).

**Gap-visibility verification (per user question, verified from log):**
the robot DOES see the wall-1 gap. At 14:57:56 a genuine mouth-grab
fired: `MOUTH_GRAB light=-6.5deg width=0.62 frame_min=0.39` — a
correct, nearly head-on light. The failure is commit EXECUTION, not
perception: the commit ran 6.6 s, lost the light from its cone
(`COMMIT_FALLBACK no light in commit cone front_min=1.40`), rotated to
**93° off down-course** (GUARD_UTURN fired DURING the commit), drifted
up-course (`UPCOURSE_MOTION dy=+0.06 mode=DOOR_COMMIT`), and aborted
with DEPTH_LOST at frame_min 0.64. Diagnosis: the blind re-aim
(deepest-ray) + stall nudges walk the commit heading off-axis with no
absolute containment until the guard's 92° backstop — far too late.
Next change targets commit heading discipline.

### Attempt 311 — commit heading containment (loop resumes, change-per-attempt)

**Change:** two-part world-frame discipline for COMMIT, targeting the
310 forensics (commit veered to 93° off down-course, guard fired
mid-commit, DEPTH_LOST at 6.6 s):

1. **World-frame target clamp** — after the robot-frame ±lock, the
   commit target is clamped so `est_heading + bearing` stays within
   ±65° of down-course (`door_commit_max_world_off_deg`). The blind
   deepest-ray re-aim can no longer chase side rays past the door
   plane once the hull has rotated.
2. **OFF_AXIS abort** — if the hull itself passes 78° off down-course
   (`door_commit_abort_off_deg`) the commit aborts immediately with a
   SHORT 1.5 s cooldown (not the 2.5 s false-light one): the light was
   genuine, execution veered; 1.5 s ≈ the rotate-back time so the same
   light can re-trigger as the nose comes around. No THREAD handoff on
   this abort path (its nose cone points at the wrong region
   sideways-on). Classified before DEPTH_LOST/CLEARED so an off-axis
   exit can never count as a cleared door.

**Result:** **14.59 m** (gt), min −5.34 (run_20260711_151201). The
containment worked as designed — no commit exceeded ~63° off-axis, no
OFF_AXIS abort needed — but it exposed the next layer: the wall-1
grab fired (light −16.5°, width 0.89, frame 0.68, t≈35) and the
commit declared **CLEARED at frame 1.11 after 6.0 s without crossing
the plane** (gt_y min −5.34; wall face is −5.4). Blob-FGM then
spiraled to −105°/−117° (GUARD_UTURN ×2), TIGHT→RECOVER at front
0.24, and the last 12 s were a RECOVER/SWEEP scrum sliding into the
right recess (guard_back=103 — banned reverse makes a nose-in pin
nearly terminal). Root cause: the frame-open CLEARED exit fires while
standing in the door mouth on the WRONG side.

### Attempt 312 — traversal evidence (side-graze) for CLEARED

**Change:** a commit may only classify as CLEARED with laser proof it
actually crossed a door plane:

1. **Side-graze detector** — during COMMIT, raw side sectors (55–125°
   both sides) are sampled every scan; any ray < 0.55 m sets
   `_commit_side_graze` (door posts of a 0.5–0.8 m door MUST pass
   there during a real traversal).
2. **Evidence gate** — close-entry commits (entry frame < 0.9 —
   face/mouth commits, where the false clear happened) cannot take
   the frame-open/passage_open success exit, nor increment
   `door_cleared_count`, without the graze. Far field-slot commits
   (entry ≥ 0.9) and genuinely wide-open standings (frame > 1.3 with
   release-width light) are exempt.
3. **Extended clock** — while a light is still visible un-traversed,
   commit timeout doubles (6.5 → 13 s): keep pushing at the face
   instead of handing authority to blob-FGM at the mouth.
4. **NO_TRAVERSAL exit** — running out the extended clock without a
   graze logs NO_TRAVERSAL, counts as timeout (never CLEARED), and
   takes the short 1.5 s cooldown so the same genuine light re-grabs
   with a fresh clock.

**Result:** **14.89 m** (gt), min −5.37 (run_20260711_151800). The
evidence gate did its job — no false CLEARED; the wall-1 commit
(grab: light −6.0°, width 0.37, frame 0.47, t≈37) pushed 7.2 s on the
extended clock and the OFF_AXIS abort fired correctly at −99°. But
the commit **ground into the door post** (front 0.15–0.17 nose-in
while chasing the light center), and the final 12 s died in the
corner-pin RECOVER↔SWEEP scrum at the LEFT corner (x≈−1.1),
re-wedging every 2–4 s at front 0.17–0.24. Two distinct failure
halves remain: the nose-close aim converges onto the near post, and
the pin escape releases too early and re-pins.

### Attempt 313 — nose-close deepest-ray aim + pin-escape escalation

**Change:**

1. **Nose-close post-avoidance** — during COMMIT with a light
   visible and frame < 0.7 m, the aim blends from the light-arc
   center toward the DEEPEST extended ray (weight 0 at frame 0.7 → 1
   at contact, only when that ray reads > max(1.0, frame+0.5)). The
   arc center skews onto the near post when the post occludes one
   side (310/312 both ground in at front 0.15–0.17); the deepest ray
   is the through-direction — THREAD's proven premise.
2. **Pin-escape escalation** — on the 2nd+ recovery inside 8 s the
   rotation no longer releases at front 0.6 (which re-pins within
   seconds in a corner pocket) but holds until front > 1.0, with a
   ≥3 s clock so the rotation can actually get there.

**Result:** **14.48 m** (gt), min −5.39 (run_20260711_152219). The
pin-escape rotation freed the robot (no 12 s terminal scrum), but the
endgame still wandered (final gt_y −4.89, 0.5 m back up). The decisive
forensic came from the telemetry trace of the t≈43 "wall-1 grab":
**it was a false grab on the RIGHT RECESS** (x≈1.1, shallow,
never-crossed). From (0.71, −3.63) — 1.8 m before the wall — the grab
aimed at world +19–21° (the recess), not the real door at −7°. The
depth gate passed because `eff_depth_thr = frame_min + margin` used
frame_min 0.39 from nearby FIELD CLUTTER, so recess-back-wall rays at
2.1 m (only ~0.3 m behind the recess slit) cleared the 1.69 m
threshold. The commit drove right, the fake light died, the re-aim
swung hard left, overshot the real door, and slid diagonally across
the face to nose-in at x≈−0.1. Depth must be measured from the SLIT
PLANE, not the robot.

### Attempt 314 — slit-relative depth (kills the recess decoy) + MODE CHANGE

**User directive:** stop the change-per-attempt loop; review all code;
finalize behavior; validate with a batch of 10 identical runs,
reporting mean and variance of gt meters.

**Change (code finalization, part 1):**
1. `_door_light` now returns `(center, width, slit_d)` where slit_d is
   the door-frame distance (min bordering ray), and takes an optional
   `behind_margin`: a candidate light is only valid if its deepest ray
   reaches `slit_d + behind_margin` — through-depth measured from the
   slit plane, not the robot. Applied with 0.7 m at the main detection
   call (ALIGN trigger / light memory) and the mouth-grab call. The
   right recess (~0.3 m behind its slit) can no longer present as a
   door from any distance; real doors (bay/next segment behind ≥
   0.9 m) still pass.
2. Mouth-grab "at the mouth" test is now the slit distance itself
   (0.30 ≤ slit_d < 0.95) instead of the clutter-prone frame_min
   window [0.35, 0.85); the frame_min floor 0.35 stays (don't grab
   while scraping-blind).

**Code finalization, part 2 — full-code review (2 independent
reviewers), 7 verified defects fixed (commit add2bcf):**
1. ALIGN debounce state (`_light_ok_streak`, `_frame_ok_streak`,
   EMA seed, light sign) never reset at ALIGN entry → stale streaks
   let a single off-center flicker commit instantly. Reset at both
   entries (None→ALIGN, SWEEP→ALIGN).
2. Wedge-path THREAD engaged while SWEEP/ALIGN/COMMIT owned the robot
   → orphaned `_thread_start` vetoed every door trigger up to 8 s;
   two controllers could alternate scan-to-scan mid-COMMIT. Now gated
   on `door_mode is None` (parity with the proactive entry).
3. Forced-SWEEP circuit breaker armed a ≥3 s recovery over itself
   (recovery block runs first), burning sweep clocks it had just
   started, and wiped `_recover_recent` so the repeat-pin escalation
   was defeated on its target case. Now publishes one guarded
   rotation and lets SWEEP own the next scan.
4. ALIGN was the only publish site bypassing `_progress_guard`.
5. Commit evidence escape `entry_front ≥ 0.9` removed (only
   high-frame mouth grabs could ever use it — exactly the
   false-positive path); side-graze now needs a 3-scan warm-up so a
   pre-existing hugged wall at entry doesn't count as a crossing.
6. `_sweep_stall_streak` carried over between sweeps (up to 11/12 of
   a spurious stall-flip). Reset at both engagement sites.
7. Door-stall "back off" commanded a reverse the guard always vetoed;
   now an explicit pivot (behavior unchanged, code truthful).

**Code finalization, part 3 — readability refactor (commit 53fa8c4):**
whole file re-commented in detailed ELI5 Turkish with section banners
(BÖLÜM), all attempt-number forensics preserved; comments/docstrings
only — AST-equivalence vs add2bcf verified independently twice.

### Batch validation #1 — 10 identical runs (commit 53fa8c4)

| run | gt_net (m) | gt_y min | gt_y final | outcome |
|---|---|---|---|---|
| 163131 | 6.58 | +2.93 | +2.93 | SIDE-PIN standstill t≈20→60 |
| 163239 | 14.59 | −5.40 | −5.05 | wall-1 stand |
| 163348 | 14.88 | −5.37 | −5.37 | wall-1 stand |
| 163456 | 14.92 | −5.40 | −5.39 | wall-1 stand |
| 163604 | 6.56 | +2.96 | +2.96 | SIDE-PIN standstill t≈20→60 |
| 163713 | 13.66 | −5.38 | −4.12 | wall face, slid back late |
| 163821 | 14.87 | −5.36 | −5.33 | wall-1 stand |
| 163929 | 14.86 | −5.37 | −5.36 | wall-1 stand |
| 164038 | 14.95 | −5.42 | −5.40 | wall-1 stand |
| 164146 | 14.88 | −5.42 | −5.38 | wall-1 stand |

**MEAN = 13.08 m, sample VARIANCE = 11.90 (σ = 3.45).** Excluding the
two standstill outliers: mean 14.70, variance 0.19 (σ 0.44) — the
recess-decoy fix made the wall-1 approach extremely repeatable, but
conversion was 0/10 this batch (gt_y min −5.36..−5.42 every time).

**New failure mode found and fixed (SIDE_PIN):** in runs 163131 and
163604 the hull's SIDE pinned against a field pillar (min_r360 0.12,
front 0.99 OPEN), commanding v=0.24 into it for 40 s with w
sign-flipping 52×. The wedge watchdog needs front contact; the
STATIC_STUCK watchdog needs a frozen scan, and the w-jitter rotated
the scan just enough to defeat it. Fix: rotation-compensated
translation check — every 2 s, the current scan is compared against a
2 s-old reference after the best rotational shift (`_scan_rotation`
now also returns its post-shift residual); if the residual is < 0.06 m
while forward ≥ 0.10 was commanded continuously and min_r360 < 0.30,
the robot provably hasn't translated (0.48 m of real motion would
dwarf the threshold) → recovery rotation triggers (SIDE_PIN event).

**Result:** _pending_batch_2_
