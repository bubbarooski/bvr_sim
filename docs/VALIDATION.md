# How Accurate Is This Simulator?

**An accuracy assessment of `intercept_sim`, measured against published
performance data for the real aircraft, missile, and sensors it models.**

---

## What is being assessed

`intercept_sim` simulates a beyond-visual-range air engagement: an F-16
flying under a six-degree-of-freedom flight model, an AIM-120-class
missile guided by proportional navigation, pulse-Doppler radar, a
launcher-to-missile datalink, and probabilistic lethality.

This document answers one question — **how closely does it reproduce
reality?** — across six layers of the modeling stack and several phases of
flight. It is not a description of how the simulator was built or tuned.

Read it with the simulator's purpose in mind: it is a learning tool built
from public data, deliberately approximate, and not a validated model of
any real system (REFERENCE.md, *What this simulator is and isn't*). The
checks below say how far that approximation sits from the published
figures it is built on — not that those figures are the real system's.

Every figure below is produced by a program in this repository. To
regenerate the entire report:

```
cmake --build build --config Release --target real_world_validation
./build/Release/tools/real_world_validation.exe
```

**120 checks, 7 outside tolerance**, each reported rather than tuned
away. They and the limits on everything else are documented in
[Where it is weakest](#where-it-is-weakest).

### Reading the sources

Performance data for military hardware is uneven in quality. Every target
value is tagged:

- **[P]** — primary: flight manual, government or NASA technical report,
  manufacturer datasheet, service fact sheet, or international standard
- **[S]** — secondary: reference sites, technical communities, derived
  estimates

A **[S]** figure is not worthless, but a model measured against one
inherits its uncertainty, and a validation document that hides the
distinction is not doing its job. Full citations are in
[SOURCES.md](SOURCES.md).

---

## Summary of results

| Layer | What was checked | Result |
|---|---|---|
| **1. Atmosphere** | Density and speed of sound, 0–100 000 ft; agreement with JSBSim | **< 0.001%**; JSBSim to 1 part in 10⁹ |
| **2. F-16** | Geometry, thrust, turn, acceleration, ceiling, climb envelope, top speed, Mach effects | Against the F-16C flight manual [[F-16C flight manual]](SOURCES.md#f-16c-flight-manual): accelerations within **3%**, sustained turns within **3.5%** (15 000–50 000 ft, subsonic; M1.2 at 40 000 ft −4.1%), and flown at the manual's weight within **2.3%** at 15 000 ft, combat ceilings within **2%**, 500 ft/min envelope within **3%** at 8 of 11 points (MIL M0.6 +4.3%, MAX AB M1.0 +3.2%, M1.8 −3.9%), quickest turns within **3.5%** from sea level to 50 000 ft, fuel flow within **5%** of the manual's fuel-flow chart (MAX AB at sea level −6.2%) and cruise fuel flow within **5%** at 16 of 18 points (240 KTAS at sea level −6.1%, 330 KTAS at 25 000 ft −5.7%); corner velocity **383 kt** (330–440); drag due to lift within **5%** of NASA flight data at Mach 0.6–1.6 |
| **3. Missile** | Mass, geometry, motor, burn phases, normal force and turn ceiling | **≤ 2.8%** on mass, motor and burn phases; normal-force tables fitted to a build-up within **~10%** of NASA TM X-3310 [[Jorgensen 1976]](SOURCES.md#jorgensen-1976) supersonic to 32° (tables within 5% of it at the ceiling, ±13% in turning drag); turn ceiling **+79%** over one CFD study [S] at 700 m/s (known discrepancy, not tuned) |
| **4. Radar** | Range laws, RCS scaling, horizon | **Exact** (< 0.05%) |
| **5. Navigation** | — | Nothing to check: the missile and aircraft know their own position |
| **6. Lethality** | Empirical vs. nominal kill probability | **1.3 percentage points** over 500 runs (one standard error) |

The pattern is consistent: **the physics layers are essentially exact,
and the residual error lives in the aircraft**, mostly in figures that
were published at a heavier weight than the model flies.

---

## Layer 1 — Atmosphere

Everything above depends on this: lift, drag, thrust lapse, the speed of
sound that converts velocity into Mach, and the radar horizon.

The simulator evaluates the **U.S. Standard Atmosphere 1976** [[ISA]](SOURCES.md#isa) in closed
form, in English units. The check re-derives the same standard
independently — in SI units, from its published layer constants — so a
units slip in the implementation cannot cancel against itself. Points
include altitudes between 5 000 ft multiples and above 65 000 ft
deliberately: a table sampled every 5 000 ft and clamped at 65 000 ft
would be wrong exactly there.

| Altitude | Speed of sound | Density | Error |
|---|---|---|---|
| Sea level | 1116.45 ft/s | 2.377×10⁻³ sl/ft³ | < 0.001% |
| 12 500 ft | 1067.42 ft/s | 1.622×10⁻³ sl/ft³ | < 0.001% |
| 25 000 ft | 1016.10 ft/s | 1.066×10⁻³ sl/ft³ | < 0.001% |
| 40 000 ft | 968.07 ft/s | 5.87×10⁻⁴ sl/ft³ | < 0.001% |
| 70 000 ft | 970.90 ft/s | 1.39×10⁻⁴ sl/ft³ | < 0.001% |
| 100 000 ft | 990.90 ft/s | 3.3×10⁻⁵ sl/ft³ | < 0.001% |

The speed of sound *rises* above 65 000 ft: the stratosphere warms with
height above 20 km, and the model reproduces it rather than holding the
tropopause value. That is the air a lofted missile coasts through.

Every aircraft flies JSBSim's own atmosphere rather than this one, so the
two must agree or a missile and its target at the same altitude would be
in different air. They are checked against each other directly in the
regression suite: **agreement to 1 part in 10⁹** in density and speed of
sound from sea level to 100 000 ft, including on hot and cold days.

**Assessment: exact.** **[P]**

---

## Layer 2 — The F-16

### What aircraft this actually is

Worth stating precisely, because it determines which published figures
are the right ones to compare against.

The flight model is JSBSim's F-16, which describes itself as an *F-16A
Block 32* but is configured with an **F100-PW-229** engine — a 29 000 lbf
powerplant from the Block 50/52 generation. Published targets below are
therefore taken from **F-16C Block 50/52** data, which is the real
aircraft this configuration most closely represents.

Every aircraft spawns at the model's own loading, **20 630 lb**, and every
flown figure below is measured at that weight. That sits between the
Block 50/52's published empty (18 238 lb) and normal loaded (26 463 lb)
weights — a light aircraft, which matters when reading the performance
rows.

| Quantity | Simulated | Published | Error |
|---|---|---|---|
| Wing area | 299.99 ft² | 300 ft² | **exact** |
| Installed thrust (afterburner) | 29 000 lbf | 29 000 lbf | **exact** |
| Structural limit | 9.0 G | 9.0 G | **exact** |
| Spawn weight | 20 630 lb | 18 238–26 463 lb | in range |

Thrust-to-weight at spawn weight is **1.41**, against a published band of
1.10 (normal loaded) to 1.59 (empty).

### Turn performance

Turn rate is the single most important aerodynamic quantity in an
engagement simulator: it determines whether an aircraft can defeat an
incoming missile. Every figure here is **flown, not computed**.

**Sustained rate** is measured the way a flight test measures it: a level
turn held at constant Mach 0.8 and constant altitude, tightened until
something stops it — the engine can no longer hold the speed, the stick
reaches its stop, or the structure reaches 9 G. Elevator holds load
factor, bank holds altitude, and throttle holds speed, all with integral
action, so none carries a standing error into the result. The project's
autopilot is deliberately not in this loop: it clamps its demand to the
scenario G envelope, which would make this measure the envelope instead
of the aircraft. Each row reports which limit set it.

| Condition | Simulated | Published | Error | Held at | Limited by |
|---|---|---|---|---|---|
| **M0.8 / 15 000 ft** | **15.34 °/s** | **14 °/s** | **+9.6%** | 7.02 G | thrust |
| M0.8 / 5 000 ft | 18.92 °/s | — | — | 8.99 G | stick stop (9 G) |
| M0.8 / 10 000 ft | 18.04 °/s | — | — | 8.40 G | thrust |
| M0.8 / 25 000 ft | 10.52 °/s | — | — | 4.67 G | thrust |

Mach and altitude are held to within Mach 0.001 and 5 ft in every row.
Only the first condition has a published figure. The other three are
reported because their **trend** is checkable without one: at constant
Mach, thinner air gives less thrust and less lift for the same stick, so
the load factor the aircraft can sustain cannot rise with altitude. It
does not. The rate itself need not fall: at 5 000 and 10 000 ft both
turns stop at 9 G, and Mach 0.8 is a faster true airspeed in the warmer
air lower down, so the same G turns more slowly there. (This check used
to assert the rate, which only held while thrust bound everywhere.)
These rows are flown closed-loop at the spawn weight; the flight manual's
sustained turns, at its 22 000 lb, are checked under
Acceleration, climb, ceiling, and top speed below: by energy balance, and flown at 15 000 ft.

The +9.6% at the published condition is weight. The published 14 °/s, a
secondary source, describes a thrust-limited turn at a combat loading;
this one is at the model's 20 630 lb spawn weight. The flight manual's
chart at 22 000 lb gives 13.85 °/s at the same condition; flown at that
weight the model holds 13.54 °/s (−2.3%), and the energy balance gives
13.86 °/s (below).

**Instantaneous rate and corner velocity** are flown the way a flight
test flies them. From level flight at 1 000 ft the aircraft rolls to 80°
in a level turn, then pulls full aft stick at full afterburner while bank
holds altitude. The peak rate is taken over quarter-second windows, and
only while the flight path stays within 10° of level; in a climbing or
sagging spiral the ground track swings faster than the turn. Entry speeds
are swept from 250 to 550 kt. Corner velocity is by definition the speed
at which the best rate occurs. The flight control system's own limits,
9 G and an angle of attack of about 25°, set the pull.

| Quantity | Simulated | Published | Result |
|---|---|---|---|
| Corner velocity, sea level | **383 kt** | 330–440 kt | **in range** |
| Peak instantaneous rate, sea level | 25.17 °/s | ~25.5 °/s | −1.3% |
| Same, flown through the autopilot | 23.29 °/s at 333 kt | — | 91% of the aircraft |

The F-16's flight control system blends its AoA limit down as load factor
rises (the G/AoA blend), and the model does too, calibrated to the flight
manual's quickest turns below, which puts corner velocity where the manual
does. With the AoA limit held at ~25° at every load factor these would read
335 kt and 28.5 °/s: 9 G would come at a lower speed than the real aircraft
reaches it.

**Quickest turns** are the flight manual's own instantaneous figures: the
"quickest turn" label on each Turn Performance chart (MAX AB, 22 000 lb,
T.O. GR1F-16CJ-1-1 [[F-16C flight manual]](SOURCES.md#f-16c-flight-manual) Figs. A8-40 to A8-47), the best rate the aircraft
reaches at that altitude. The same flown pull, at 22 000 lb with fuel
frozen, is swept over entry speed in 10 kt steps and the best rate kept.
Sea level is flown at 1 000 ft: the rig at 0 ft sits on the terrain.

| Altitude | Simulated | Flight manual | Error | Speed, load factor |
|---|---|---|---|---|
| 1 000 ft | 24.27 °/s | 24.8 °/s | −2.1% | 402 kt, 8.9 G |
| 5 000 ft | 22.69 °/s | 22.7 °/s | −0.1% | 433 kt, 9.0 G |
| 10 000 ft | 20.93 °/s | 20.8 °/s | +0.6% | 463 kt, 8.9 G |
| 15 000 ft | 19.31 °/s | 19.0 °/s | +1.6% | 502 kt, 9.0 G |
| 20 000 ft | 17.91 °/s | 17.4 °/s | +2.9% | 549 kt, 9.1 G |
| 30 000 ft | 14.44 °/s | 14.4 °/s | +0.3% | 639 kt, 8.8 G |
| 40 000 ft | 9.70 °/s | 9.6 °/s | +1.1% | 638 kt, 6.1 G |
| 50 000 ft | 6.09 °/s | 5.9 °/s | +3.3% | 655 kt, 4.0 G |

Speeds are true airspeed. Two parameters are calibrated to these rows,
not measured: the slope of the G/AoA blend (0.8° of AoA limit per G,
fitted to the 9 G rows) and the loss of high-AoA lift with Mach (fitted
to the 30 000 and 40 000 ft rows, which reach the AoA limit near
Mach 0.9–1.1). No F-16 figure for either was found; REFERENCE.md §4.4
and §5 give the derivation.

The 50 000 ft row first read +9.2%, with the pull overshooting the AoA
limit to 40°, then +5.1% with the limiter's lead scheduled on dynamic
pressure (REFERENCE.md §4.4), the overshoot still 5° (24.5° against a
blended limit near 19°). After the flaps were corrected (below) it is
+3.3% with the overshoot about 1° (19.7° against 18.6°): the extra
overshoot had come from the flaps' lift.

The last row checks what the scenarios actually get. They fly through the
autopilot. It flies a level turn at the bank cap, and a purely
proportional roll loop would settle a few degrees short of the cap: near
84° of bank that is the difference between 9 G and 6. The bank loop has
integral action near the target, so a hard heading change pulls the
envelope's G. It reaches 93% of the aircraft's rate (23.3 against 25.2 °/s);
the steep-turn altitude hold below, which stops it climbing out of the turn,
accounts for the difference from the 96% of an unconstrained turn.

**Steep-turn altitude hold.** The same hard turn, held for 25 s with the
autopilot's altitude hold working against it, is the scenarios' steep turn.
A pilot holds one to 100 ft (the FAA steep-turn standard, the tolerance
here). Flown at four altitude and speed pairs:

| Condition | Worst altitude error | Bank / peak load factor |
|---|---|---|
| 10 000 ft, 450 kt | 44 ft | 85° / 8.0 G |
| 15 000 ft, 520 kt | 79 ft | 87° / 9.1 G |
| 25 000 ft, 500 kt | 54 ft | 84° / 6.6 G |
| 35 000 ft, 550 kt | 49 ft | 82° / 5.4 G |

The first version held the aircraft 190–340 ft high, climbing 26 ft/s
through the roll-in and then settling at a standing offset. Cause: the
vertical channel took the vertical share of the lift as `N_z cos φ`, which
at 85° of bank and 15–18° of AoA is 0.6 g; the aircraft was level, because
thrust along the nose (`N_x sin θ`, 0.15 g in afterburner) and the side
force (`N_y`, 0.2 g) supply the rest. The loop thought the aircraft was
sinking and no gain could fix a feed-forward error. It now uses the
accelerometers' exact vertical specific force (REFERENCE.md, the cascaded
autopilot). Also in `real_world_validation` and `AircraftManeuvers.SteepTurnAltitudeHold*`.

### What the turn performance depends on

Six properties of the airframe model and its envelope set the turn
performance. All six were found by measuring against the published figures
rather than by tuning:

1. **The pitch flight control system** reads full aft stick as a load-factor
   command with an angle-of-attack limiter (REFERENCE.md §4.4). A bounded
   pitch-rate demand, as in the stock JSBSim system, is worth only about
   4 G at 300 kt.
2. **The flaperon mixing.** The symmetric flaperon deflection, which drives
   flap lift and drag, is the flap schedule alone (see 6 below). The stock
   mixing summed left and right flaperons with the flap schedule entering
   at opposite signs, so the schedule cancelled and the aileron took its
   place: in a hard pull, 0.55 of aileron cost −0.55 of lift coefficient,
   a third of the wing's lift. Its gain also converted to radians the wrong
   way round, a further ×8.
3. **The envelope's `CL_max` is 1.65**, the wing's lift at the F-16's
   published ~25° AoA limit, measured on the model's NASA wind-tunnel lift
   tables, so the autopilot asks for what the airframe can give. (A value of
   1.2 asks for too little.)
4. **The AoA limit falls with load factor** (the G/AoA blend,
   `cl_max_loss_per_g`) and the envelope's lift limit follows it. Held at
   ~25° at 9 G it would put corner velocity at 335 kt and the sea-level
   quickest turn 15% fast.
5. **The AoA limiter's lead on pitch rate grows as dynamic pressure falls.**
   A lead sized for the dynamic pressure of a low-altitude fight lets the
   aircraft reach 40° AoA at 50 000 ft before full nose-down elevator
   catches it.
6. **The flaps are neutral on a clean aircraft.** The flight manual says
   they droop only with the gear handle down (or ALT FLAPS EXTEND), 20°
   below 240 knots reducing to retracted at 370 (T.O. GR1F-16CJ-1-1
   [[F-16C flight manual]](SOURCES.md#f-16c-flight-manual), Flight Controls:
   Trailing Edge Flaps). The stock schedule's 20° below 250 KCAS in any
   configuration hides in everything fitted at altitude below 250 KCAS
   (Mach 0.5–0.7 above 25 000 ft): the accelerations' thrust and the polar
   break fitted to the turn charts. It shows in the fuel flow: a clean
   aircraft at 35 000 ft and 420 KTAS burned 46% too much flying with 20° of
   flap. The leading edge flap follows the schedule published with the NASA
   tables (Nguyen et al., TP-1538 [[Nguyen 1979]](SOURCES.md#nguyen-1979):
   1.38 α − 9.05 q̄/p + 1.45 deg, limited to −2° and 25°) rather than stock
   steps to 15° and 25° at fixed AoAs; that choice is almost neutral (under
   0.3% anywhere checked). The thrust and drag fits and the FCS design are
   made on this aircraft.

### Lift, stability and drag due to lift with Mach

The model's aerodynamic tables are NASA TP-1538 [[Nguyen 1979]](SOURCES.md#nguyen-1979) low-speed wind-tunnel
data. On their own they give the same lift-curve slope at every Mach, and
drag due to lift with no transonic growth. Both are now Mach-dependent,
each against F-16C measurements where they exist:

| Quantity | Mach | F-16C measured | Model | Source |
|---|---|---|---|---|
| Lift-curve slope (per deg) | 1.6 | 0.068 | 0.068–0.074 | NASA TP-3355 [[Fox 1993]](SOURCES.md#fox-1993) Fig. 31 (tunnel) |
| | 1.8 | 0.058 | 0.058–0.063 | TP-3355 |
| | 2.0 | 0.052 | 0.055 | TP-3355 |
| Static margin, % MAC | 1.6 | 25.7 | 22.9–26.7 | TP-3355 Fig. 30 (tunnel) |
| Drag due to lift, ΔCD/ΔCL² | 0.6 | 0.122 | 0.119 | NASA TP-3414 [[Saltzman 1994]](SOURCES.md#saltzman-1994) Fig. 24 (flight) |
| | 0.9 | 0.139 | 0.133 | TP-3414 |
| | 1.3 | 0.268 | 0.280 | TP-3414 |
| | 1.6 | 0.28 | 0.286 | TP-3355 Fig. 30 (tunnel polar) |

Model ranges are across the altitudes identified, which sit at different
trim angles of attack.

- **Lift** is a Mach factor on the angle-of-attack-dependent lift. Below
  Mach 0.9 it follows the DATCOM finite-wing slope [[USAF DATCOM]](SOURCES.md#usaf-datcom) with Prandtl–Glauert
  compressibility (+8% at Mach 0.6, +24% at 0.9). From 1.6 up it is the
  measured slopes. Between 0.9 and 1.6 there is no data and it is faired
  linearly. Where the factor exceeds 1, it blends back to 1 by 20° AoA,
  so subsonic maximum lift stays at its low-speed value instead of being
  inflated. Supersonic, it applies at every AoA, and the G envelope
  scales `CL_max` with it (0.90 at Mach 1.8, 0.81 at 2.0).
- **Stability.** The model was 4–7% MAC less stable supersonic than
  measured. Its supersonic pitching-moment rows now hold the measured
  Mach 1.6 aerodynamic-centre position, taking the model's spawn CG as
  the F-16C's 0.35 MAC operational CG. Above Mach 1.6 they hold that
  position as the lift slope falls.
- **Drag due to lift** had almost none supersonic: 0.018 at Mach 1.3
  against a flight-measured 0.268. Without lift's Mach factor balanced by
  it, the lift change alone made the Mach 0.8 sustained turn 31% too good.
  A compressibility term ΔK(M)·CL² now brings every measured point within
  5%.

The pitch flight control system was re-identified and regenerated on the
new aerodynamics (REFERENCE.md §4.4), and the turn and top-speed figures
in this section are flown on it.

### Fuel flow

JSBSim burns fuel as thrust × TSFC (military power) or thrust × ATSFC
(afterburner). The stock engine gave afterburning a constant 2.05
lb/hr/lbf and military power a constant 0.74, with a temperature factor,
so the acceleration tables' fuel came out 23% low at sea level and 11% low
at 10 000 ft. A real engine's specific fuel consumption rises with Mach
and falls with air temperature, and varies with throttle setting.

**Afterburning** is `ATSFC = c0 · θ^p · (1 + c1 · M)` (θ the static
temperature ratio): c0 = 1.8695, p = 0.7947, c1 = 0.2604, held below Mach 0.3.
**Military power** is the same form at full power times a part-throttle
factor (nine numbers: the full-power line `0.637 · θ^0.579 · (1 + 0.639 M)`
with small Mach and temperature corrections, and a factor in
`x = 1 − N2norm` that is 1 at full power and rises to about 1.7 at
N2norm 0.35, with a shallow bucket near 85% N2; REFERENCE.md, Fuel flow,
has them all). A static F100-PW-229 is published near 0.74 military and
2.05 afterburning; the fit gives 0.77 and 2.02 at Mach 0.3 at sea level.
**[P]** The shape is the standard one for a turbofan (Raymer [[Raymer]](SOURCES.md#raymer), *Aircraft
Design*, installed engine performance: TSFC rises with Mach and as the
square root of temperature); every number is **calibrated**
(`tools/fit/fuel_fit.py`):

- **Full power** to the flight manual's fuel-flow chart, Fig. A8-4 (MIL and
  MAX AB fuel flow against Mach, 1 000 to 60 000 ft, standard day, flight
  test; its curves digitised from the PDF's vector paths into
  `reference/fuel_flow.csv`). Only chart points where the thrust tables
  themselves rest on flight data are fitted (the Mach range of the
  acceleration tables at each altitude, to 30 000 ft MIL and 40 000 ft
  MAX AB); elsewhere the thrust is an extrapolation and a fuel flow fitted
  there would put its error into the TSFC. 3.0% rms (MIL), 3.4% (MAX AB).
- **Part throttle** to the manual's Subsonic Cruise tables (drag index 0,
  total fuel flow in steady level flight by altitude, true airspeed and
  gross weight: 960 values, `reference/cruise_fuel.csv`, parsed from the PDF
  text). In steady flight thrust equals drag, so each point measures fuel
  flow at the throttle setting that balances drag. `tools/drivers/cruise_sample`
  flies 720 of the points in the model for the thrust and engine speed it
  needs; the fit is to every gross weight but 24 000 lb, 3.3% rms over 559
  points. The 24 000 lb points are the validation rows below and were not
  fitted.

**The cruise points carry the model's drag as well as its fuel flow, and
the two cannot be separated from the manual alone.** In steady flight the
manual gives drag × TSFC; the fuel-flow chart gives thrust × TSFC; the
accelerations give thrust − drag. Three equations in four unknowns (drag,
thrust and two TSFCs), so a drag error and a part-throttle TSFC error
trade exactly. This was tested rather than assumed. The first fit
(JSBSim's own part-throttle rise, nothing fitted to cruise) missed by
12–17% at Mach 0.75–0.94 above 25 000 ft. A clean aircraft was flying with
20° of flap (an aircraft defect, fixed; see What fixing turn performance
took), and what remained was a ±8% pattern that follows lift coefficient
and Mach: the model's drag looked low at CL 0.45–0.5, Mach 0.55–0.6 and
high at CL 0.25–0.35, Mach 0.8–0.94. The turn charts show the same sign
at high CL and, unlike the cruise points, measure drag without needing the
TSFC, so the drag fitted to them was made Mach-dependent (polar-break
bullet above). It halved the turn residuals and put every subsonic
sustained turn within 3.5%, **and left the cruise residual where it was**
(held-out 2.8% rms before, 3.0% after). So the cruise residual is not, at
the lift coefficients the turn charts reach, a drag error; what is left is
better described as an effective part-throttle TSFC of this engine
installation in Mach, temperature and throttle, calibrated to the manual
and not a physical model. Where the drag at CL 0.2–0.5 differs from the
manual's, the factor still absorbs it; the manual's data cannot say.

Checks, all flown through the simulator:

| Check | Result |
|---|---|
| Acceleration tables' fuel columns (fuel to reach each tabulated speed; not fitted) | within 5% (worst column), MIL and MAX AB, five weights each |
| Fig. A8-4 along the flown accelerations (fuel burned once the engine's fuel flow has caught up, against the chart at the Mach flown), MAX AB at 10 000 / 20 000 / 30 000 / 40 000 ft | +1.5 / +2.9 / −0.3 / +0.2% |
| Same, MAX AB at sea level | **−6.2%** (OUT) |
| Same, MIL at sea level / 10 000 / 20 000 / 30 000 ft | −0.6 / +2.4 / −1.9 / −0.2% |
| Held-out cruise points, all altitudes, 24 000 lb (112 points) | 3.0% rms; 101 of 112 within 5%, worst −7.6% |

The sea-level afterburning row is the one place the single ATSFC form falls
short: the chart's sea-level ATSFC rises with Mach about twice as fast as
at altitude, which three parameters cannot follow. Three richer forms were
tried against the chart (a Mach² term, a Mach × temperature term, and a
ram-pressure term): each moved the fit's rms by under 0.2% and the sea-level
Mach 1.0–1.2 flows (chart 85–96 k lb/hr against the model's 81–91 k) by
under 1%, so the form was left at three parameters.

**Steady cruise validation** (eighteen of the held-out 24 000 lb points,
sea level to 40 000 ft, 240–570 KTAS, flown with the autopilot holding
altitude and speed; tolerance 5%):

| Altitude | KTAS | Simulated | Manual | Error |
|---|---|---|---|---|
| Sea level | 240 / 360 / 480 / 570 | 2 537 / 3 822 / 5 878 / 9 101 | 2 701 / 3 846 / 6 039 / 8 920 | **−6.1** (OUT) / −0.6 / −2.7 / +2.0% |
| 8 000 ft | 300 / 420 / 540 | 2 664 / 3 884 / 6 067 | 2 653 / 3 819 / 6 071 | +0.4 / +1.7 / −0.1% |
| 16 000 ft | 270 / 390 / 510 | 2 365 / 2 898 / 4 146 | 2 391 / 2 808 / 4 198 | −1.1 / +3.2 / −1.2% |
| 25 000 ft | 330 / 450 / 570 | 2 078 / 2 735 / 5 221 | 2 205 / 2 624 / 5 388 | **−5.7** (OUT) / +4.2 / −3.1% |
| 35 000 ft | 420 / 480 / 540 | 2 008 / 2 338 / 3 508 | 2 082 / 2 291 / 3 371 | −3.6 / +2.0 / +4.1% |
| 40 000 ft | 480 / 510 | 2 136 / 2 359 | 2 207 / 2 373 | −3.2 / −0.6% |

Sixteen of the eighteen rows are within 5%, the Mach 0.75–0.94 ones within 4.2%.

### Acceleration, climb, ceiling, and top speed

The reference here is the F-16C's own flight manual [[F-16C flight manual]](SOURCES.md#f-16c-flight-manual), T.O. GR1F-16CJ-1-1
(Appendix A, F100-PW-229, "data basis flight test"), at drag index 0:
the basic aircraft with tip missiles, standard day. **[P]** Everything
below is flown through the whole simulator, its checks listed in the
type's `reference/validation.json` and `climb_envelope.csv`.

**Level accelerations** (MAX AB from 200 KIAS, 20 000 lb; the autopilot
holds altitude and fuel burns; sea level flown at 200 ft):

| Acceleration | Simulated | Manual | Error |
|---|---|---|---|
| Sea level → M1.21 (800 KIAS limit) | 37.9 s | 37 s | +2.3% |
| 10 000 ft → M1.41 (800 KIAS limit) | 46.5 s | 46 s | +1.0% |
| 20 000 ft → M1.68 (800 KIAS limit) | 73.9 s | 72 s | +2.6% |
| 30 000 ft → M1.75 | 86.9 s | 86 s | +1.0% |
| 40 000 ft → M1.83 | 121.8 s | 123 s | −0.9% |

**Sustained turns** (the Ps = 0 line of the Turn Performance charts, MAX
AB, 22 000 lb):

| Condition | Simulated | Manual | Error |
|---|---|---|---|
| M0.6 / 15 000 ft | 12.69 °/s | 12.54 °/s | +1.2% |
| M0.8 / 15 000 ft | 13.86 °/s | 13.85 °/s | +0.1% |
| M0.9 / 15 000 ft | 13.72 °/s | 13.93 °/s | −1.5% |
| M0.6 / 30 000 ft | 6.44 °/s | 6.56 °/s | −1.9% |
| M0.8 / 30 000 ft | 7.57 °/s | 7.57 °/s | −0.1% |
| M0.9 / 30 000 ft | 7.84 °/s | 7.88 °/s | −0.5% |
| M0.8 / 40 000 ft | 4.44 °/s | 4.45 °/s | −0.1% |
| M0.9 / 40 000 ft | 4.79 °/s | 4.69 °/s | +2.0% |
| M1.2 / 40 000 ft | 3.91 °/s | 4.08 °/s | −4.1% |
| **M0.8 / 50 000 ft** | **1.74 °/s** | **1.75 °/s** | **−0.4%** |
| **M0.9 / 50 000 ft** | **2.16 °/s** | **2.24 °/s** | **−3.4%** |

**Climb envelope** (Fig. A8-2: the altitude, at each Mach, where 500
ft/min of climb remains at 1 g; 20 000 lb):

| Power, Mach | Simulated | Manual | Error |
|---|---|---|---|
| MIL, M0.6 / 0.7 / 0.8 / 0.9 | 44 850 / 47 460 / 49 040 / 51 470 ft | 43 000 / 46 200 / 48 500 / 50 800 ft | +4.3 (OUT) / +2.7 / +1.1 / +1.3% |
| **MAX AB, M0.80** | **55 400 ft** | **55 900 ft** | **−0.9%** |
| **MAX AB, M0.88** | **57 850 ft** | **57 350 ft** | **+0.9%** |
| **MAX AB, M1.0** | **57 680 ft** | **55 900 ft** | **+3.2%** (OUT) |
| **MAX AB, M1.2** | **57 250 ft** | **59 000 ft** | **−3.0%** |
| **MAX AB, M1.4** | **58 760 ft** | **60 000 ft** | **−2.1%** |
| **MAX AB, M1.6** | **58 720 ft** | **59 300 ft** | **−1.0%** |
| **MAX AB, M1.8** | **55 550 ft** | **57 800 ft** | **−3.9%** (OUT) |
| MAX AB combat ceiling (Fig. A8-1), 20 000 / 24 000 lb | 56 680 / 53 500 ft | 56 300 / 53 000 ft | +0.7 / +1.0% |
| **MIL combat ceiling (Fig. A8-1), 20 000 / 24 000 lb** | **50 230 / 47 050 ft** | **50 300 / 46 700 ft** | **−0.1 / +0.7%** |

Bold rows are independent of every fit; the rest check that a fit
survives the simulator. What was fitted, and to what:

- **Thrust at 1 g** (REFERENCE.md §4.1) to the level accelerations: the
  model's own drag and AoA, measured by flying it, and thrust at each
  table Mach chosen so `(W/g) dV/dt = T cos α − D` reproduces the times.
  Afterburning to 0.4–0.6 s rms up to 20 000 ft and 0.5–3.7 s at 30 000 and
  40 000 ft; military to 0.7–2.0 s. The two highest altitudes are timed
  to the last speed with real acceleration left: the tables run on to
  within a few hundredths of Mach of the top speed, where the time to go
  is asymptotic.
- **Drag above the polar break** to the sustained turns at 15 000–40 000
  ft. The model's drag due to lift is NASA's flight data to a lift
  coefficient of about 0.65 [[Saltzman 1994]](SOURCES.md#saltzman-1994);
  the turns reach 0.75–1.2, where the model falls short by an amount
  growing with CL. A sustained turn is where full-power thrust equals
  drag, so with thrust fitted at 1 g each point gives the drag coefficient
  at its CL directly (the thrust's share of the force normal to the path,
  T sin α, comes off the lift), and any error in the 1 g drag cancels,
  because the thrust carries it too. One term,
  ΔCD = c(M) · max(0, |CL| − 0.65), with c(M) a table: +0.099 / +0.124 /
  +0.094 / +0.068 / +0.057 at Mach 0.5 / 0.6 / 0.7 / 0.8 / 0.9, falling to
  0 at 1.05. Fitted to 17 subsonic points, 0.0045 rms in CD. The turns
  cannot say where the shortfall starts, so the CL below which the term is
  zero is held at the top of the measured data rather than fitted: left
  free it runs to the fit's lower bound and puts drag into 1 g flight.
  It fades out between Mach 0.9 and 1.05: supersonic the manual shows no
  shortfall (the model is ~6% high at Mach 1.2–1.3, CL 0.5, left alone).
- **Afterburning thrust above 40 000 ft**: pressure scaling alone left the
  combat ceiling 2 900 ft high and the 50 000 ft turns 10–14% too good.
  Augmentor combustion efficiency falls with burner pressure, so
  afterburning thrust lapses faster than pressure there; modelled as
  `T ∝ δ^1.25` above 40 000 ft, the exponent calibrated to the combat
  ceiling chart. The 50 000 ft turns and the MAX AB climb envelope, not
  used, then land within 0–5%.
- **Military thrust above 30 000 ft** to the MIL climb envelope. The
  accelerations stop at 30 000 ft, and the generic table's 30 000 →
  40 000 ft step put the MIL ceiling 8% low and peaked the envelope at the
  wrong Mach. The 40 000 ft column is fitted so Ps at 1 g is 500 ft/min
  at the four envelope points (within ±1.3 ft/s); above it, thrust scales
  with pressure. The MIL combat ceiling chart, not used, lands within 2%.

**Fuel flow** is fitted to the manual's fuel-flow chart and checked
against two other measurements; see Fuel flow below.

Climb rate is 1 g specific excess power: a steady full-afterburner climb
turns all of it into height. At 20 000 lb the model's best is 69 600
ft/min at Mach 0.86 at 200 ft, the peak over 2 s windows. The 50 000
ft/min figure sometimes quoted for this aircraft has no traceable source.
The manual's sea-level acceleration at this weight averages about 63 500
ft/min between Mach 0.76 and 0.98 (7 s, so the table's 1 s rounding
allows several percent either way).

**The published top speeds are limits, not top speeds.** Mach 2.05 at
40 000 ft and 1.2 at sea level are airspeed limits. In the manual the
aircraft reaches 800 KIAS (Mach 1.21) at sea level after 37 s and Mach
2.03 at 40 000 ft, both while still accelerating. Where thrust would
finally equal drag is not published. The model's thrust-limited top
speeds are Mach 2.038 at 40 000 ft and 1.331 at 1 000 ft, beyond the
placard. The check is now that it reaches both limits.

The compressibility drag is fitted to measured F-16C zero-lift drag. (A crude
four-point add-on, +0.023 at Mach 1.1 falling to +0.015 at 1.8 and held
beyond, would rise too early and be 25–30% low supersonic.)

| Mach | Measured clean CD₀ | Source | Model |
|---|---|---|---|
| 0.6 | 0.020 | NASA TP-3414 [[Saltzman 1994]](SOURCES.md#saltzman-1994), flight (Table 5: 0.0199) | 0.021 |
| 0.9 | 0.023 | TP-3414, flight | 0.023 |
| 1.3 | 0.046 | TP-3414, flight | 0.046 |
| 1.6 | 0.049 | NASA TP-3355 [[Fox 1993]](SOURCES.md#fox-1993), F-16C tunnel model | 0.049 |
| 1.8 | 0.048 | TP-3355 | 0.048 |
| 2.0 | 0.047 | TP-3355 | 0.047 |
| 2.16 | 0.046 | TP-3355 | 0.046 |

The TP-3414 points are read from its Figure 23. The TP-3355 points are
minimum drag from its Figure 31, measured with AIM-9J wingtip missiles.
That report's own Figure 22 puts two such missiles at 0.0008–0.0009 on its
2.8 ft² reference, or 0.0017–0.0019 on the F-16C's 1.333 ft², which is
subtracted to give the clean value. Between Mach 0.9 and 1.3, where there
are no measurements, the drag rise follows Raymer's [[Raymer]](SOURCES.md#raymer) standard
construction. Drag-divergence Mach is where drag has risen 0.002 above
subsonic, and the 0.9 flight point is exactly that. Critical Mach sits
0.08 below it. Drag at Mach 1.05 equals drag at Mach 1.2, taken here as
the measured Mach 1.3 value. The subsonic base drag is the model's own
NASA low-speed database and is left alone (0.021 against flight 0.0199).

Five things set the top speeds, and three of them show how an apparent
"match" can be luck:

1. **Gear up at spawn.** JSBSim spawns every model gear-down, and the F-16's
   gear-drag term adds about 60% to clean parasite drag for every flight.
   The simulator raises the gear at spawn; left down, top speed would be
   Mach 1.59 where raising it gives 2.018 (see 2).
2. **The pitch controller.** That 2.018 is itself an artifact of the stock
   pitch FCS, which costs drag in flight: with the designed controller the
   aircraft reaches Mach 2.59. The check commands an unreachable speed: a
   command of only 1 400 kt, which is Mach 2.44 at 40 000 ft, would measure
   the command rather than the aircraft.
3. **Drag from measurements**, as above. With it and the generic thrust
   tables both top speeds land within 3.5% of the published figures —
   which are limits, so that agreement means nothing. The flap drag term is
   on the deflection's magnitude: with the flaperon mixing correct (Turn
   performance, above) the −2° supersonic flap schedule reaches the drag at
   all, and the stock signed term would subtract drag. The supersonic
   compressibility rows are booked net of that flap drag, so the model's
   total equals the measurement in the measured configuration.
4. **Thrust from the flight manual.** The generic tables (Aero-Matic's
   [[Aero-Matic]](SOURCES.md#aero-matic) augmented-turbofan lapse, written
   identically for every afterburning engine) give 35–50% too much excess
   power subsonic at 40 000 ft and 40–60% too little at Mach 1.45–1.75, and
   their 60 000 ft column of zeros halves thrust by 55 000 ft. Fitted to
   the manual's accelerations, static sea-level afterburning thrust comes
   out at 27 713 lbf against the engine's 29 100 lbf uninstalled rating,
   without being fitted to it, and thrust at 30 000–40 000 ft peaks near
   Mach 1.7–1.8 and falls past 1.9, as a fixed normal-shock inlet's
   recovery does.

### The afterburner works

Worth confirming explicitly, since a missing afterburner would explain
every row above and would be easy to miss:

| Setting | Thrust at 20 000 ft / 450 kt |
|---|---|
| Military power | 9 548 lbf |
| Full afterburner | **26 656 lbf (+179%)** |

The augmentation is producing.

---

## Layer 3 — The AIM-120 missile

### Mass, geometry, and motor

| Quantity | Simulated | Published | Error |
|---|---|---|---|
| Launch mass | 152.00 kg | 152 kg | **−0.0%** |
| Body diameter | 178.01 mm | 178 mm | **+0.0%** |
| Propellant mass | 60.14 kg | 60.15 kg | **−0.0%** |
| Propellant mass fraction | 39.6% | 30–50% typical | in range |
| **Specific impulse (implied)** | **247.8 s** | **~245 s** | **+1.2%** |

The last row is the most diagnostic number in the missile model, and it
deserves explanation because it is not a parameter — it is a *consequence*
that must land inside a physical bound.

A rocket motor's specific impulse is its total impulse divided by the
weight of propellant burned:

```
Isp = total impulse / (propellant mass × g)
```

This is a hard chemical limit. A solid motor burning HTPB — which the
AIM-120 does — achieves roughly **245 seconds**. A model that produces its
impulse from too little propellant is describing a motor that cannot be
built, and more importantly is flying a missile whose *mass history* is
wrong: too little mass shed means it coasts heavy, drags wrongly, and
arrives with the wrong energy and the wrong control authority.

The simulator's thrust profile and mass split satisfy this constraint
independently, to 1.2%.

### Flight phases

Launched from 25 000 ft at 400 kt, flown straight and unguided so that
propulsion and drag are isolated from guidance entirely:

| Phase | End speed | Mean acceleration |
|---|---|---|
| Boost (0–3 s) | 2 403 ft/s | +575.9 ft/s² |
| Sustain (3–8 s) | 4 177 ft/s | +354.7 ft/s² |
| Coast (+20 s) | 2 611 ft/s | −78.3 ft/s² |
| **Peak** | **Mach 4.11** | vs. published Mach 4 — **+2.8%** |

The phase ordering is a physical necessity independent of any published
figure, and holds: thrust exceeds drag hardest during boost, less during
sustain, and drag alone decelerates in coast.

Altitude dependence — thinner air means less drag, so the same motor
reaches a higher peak Mach and loses energy more slowly:

| Launch altitude | Peak Mach |
|---|---|
| 5 000 ft | 3.53 |
| 25 000 ft | 4.11 |
| 40 000 ft | 4.48 |

This is the terminal-energy behavior the whole engagement model rests on,
and it emerges from the drag and mass models rather than being scripted.

### Normal force and turn ceiling

The turn ceiling is `CN_max(M) q S_ref / W`, capped at 30 G
(REFERENCE.md §6.5). The shipped model is two tables over Mach, a lift
slope and `CN_max` = CN at α_max 30°, **fitted** to a component build-up
from published methods on the AIM-120C geometry; the tables reproduce it within 5% at the ceiling
and ±13% in drag due to lift. The checks below are of that build-up, whose
methods were checked against wind-tunnel
data for a body / body-wing / body-wing-tail family (NASA TM X-3310 [[Jorgensen 1976]](SOURCES.md#jorgensen-1976), L/d
10, Re_d ≈ 2×10⁵). Mean relative CN error by α band 0–20° / 20–32° / 32°+:

| Mach | Body | Body + wing | Body + wing + tail |
|---|---|---|---|
| 0.6 | +133% / +52% / −5% | +23% / +90% / +101% | +21% / +60% / +78% |
| 0.9 | +119% / +1% / −23% | +14% / +71% / +78% | +10% / +47% / +65% |
| 1.5 | +13% / −3% / +3% | +5% / +15% / +25% | +0% / +13% / +29% |
| 2.0 | −7% / −4% / −3% | +3% / +5% / +2% | +3% / +4% / +5% |

Supersonic, where the AIM-120 flies most of its turns, the build-up is
within ~10% to 32°. Subsonic body CN at low α is small in absolute terms
(the +100% rows are CN 0.3–1.3); the subsonic fin rows over-predict above
20° because TM X-3310's aspect-3.8 wing stalls and the AIM-120's
aspect-1 fins are not expected to. That part is **unchecked**, not
validated. TM X-3128 (body N1C1, M0.6, two Reynolds numbers): the model
predicts the Reynolds drop in the right direction but ~2× too large at
α 42°.

| Condition | Available lateral G | Check |
|---|---|---|
| Early — 2 500 ft/s, full mass, 25 000 ft | **30.0 G** (structural cap) | |
| Late — 1 200 ft/s, dry, 25 000 ft | **19.1 G** (aero-limited) | falls as it slows: ok |
| M1.2, 8 km, dry | ≥ 10 G | IASGATG 2014 CFD [[IASGATG 2014]](SOURCES.md#iasgatg-2014) [S] "≥ 10 G transonic": **ok** |
| 700 m/s, 10 km, dry | **53.8 G** aero limit | IASGATG: 30 G stops here (α 30°): **+79%, OUT** |

The last row is a **known discrepancy, reported and not tuned**. The CFD
study's CN(30°) is 12–13; published crossflow theory gives ≈ 19 for this
body, most of the difference being the body's own crossflow lift. No
measured data for a body of L/d ≈ 20 was found to decide between them,
and the study is a single secondary source, so the published methods are
kept. The early figure (30 G) is the structural cap, itself unsourced
(no published limit; the CFD study's test load). The old fin envelope
is not a fin-based formula.

### Gravity

There is no published figure to measure this against; it is
first-principles physics, checked against its closed form. The guided
missile feels gravity, and its autopilot's gravity bias cancels the part
across the body axis (REFERENCE.md §6.8). A known-answer check flies a
drag-free, burnt-out missile through five cases and matches the closed
form to < 10⁻⁹ ft/s²: level with the bias (no acceleration), level
without it (1 G down), a 30° climb (`g sin γ` along the axis only), after
losing lock (still none), and below 1 G of available lift (sinks by the
shortfall). The flight-phase table above is level flight, where the bias
cancels gravity exactly, so it is unchanged.

The long tail-chase misses close by up to 2 km (median), because a missile
following a diving target gains energy from the dive.

### Steering across the body axis

Also first-principles, with no published figure to fit: lift acts only
across the body axis, so the guidance command must too (REFERENCE.md
§7.6). True PN's command, which lies across the line of sight, is rotated
into the plane across the air velocity keeping its magnitude. A known-answer check flies a drag-free,
burnt-out level missile with a chosen LOS rate that has a part along the
velocity, and matches to < 10⁻⁹: the rotated true-PN command (no
along-axis part, magnitude `N V_c |λ̇|`),
and a true-PN command lying exactly along the axis (dropped). With the
rotation removed, this check, the guidance-law re-derivation and the
memory-coast check all fail.

Steering across the body axis, against applying the along-axis part as free
thrust or braking (+190 to +310 ft/s over a tail-chase flight, −960 ft/s in
the crossing shot, ~0 head-on), moves the hit count over 13 missile scenario
files × 20 seeds from 150 to 148. Head-on shots change only by Pk draws,
crossing flights are ~29 s shorter, and tail-chase median misses grow by
0.5–2 km. A long shot that coasts after burnout can end with a collision lead
angle past the seeker's 45° gimbal limit, since the free thrust no longer
holds it up.

The guidance-accuracy check (stationary target, 5 km) now measures
closest approach between ticks. At ~3 500 ft/s its 20 Hz samples are
~50 m apart, and with the fix the nearest sample landed 26 m from a pass
whose true closest approach is 0.00 m; the 15 m bound is unchanged.

### Actuator lag

**Not validated — no data.** The AIM-120's autopilot and fin-servo
response is not published, so its 0.2 s lag is an estimate (REFERENCE.md
§6.6), not a fitted or checked value. What is checked is that it does not
matter at the level of the estimate's uncertainty: 0.1 / 0.2 / 0.3 s give
135 / 136 / 132 hits over 20 seeds × 13 scenarios against 136 with no
lag. Every flip is a kill-probability roll at the same miss distance except
one at 0.3 s (crossing seed 20, a 30 km miss becoming a hit). The model mechanics are
checked exactly (step response against the discrete recurrence, and the
held lift staying across a turning body axis).

### Launch turn and datalink fallback

**Not validated — no data.** No public source describes the AIM-120's
behaviour after an off-boresight launch or after its own seeker loses
the target. The 15° hand-over angle of the launch turn is an estimate
(REFERENCE.md §6.9), as is the choice to fall back on the datalink
(§8.1). Both exist because without them the launch zone shows a launch just
inside the seeker's gimbal failing where one just outside it hits. What is
checked: the turn asks for the full turn ceiling toward the target and
ends inside the hand-over angle, and an active seeker that loses the
target falls back only while the launcher still tracks it (unit tests).
Over 13 scenarios × 20 seeds, the turn and fallback move hits from 162 to
159; every flip but one is a kill-probability roll at 0 m (crossing 20 → 16).
The turn costs crossing ~4.5 s of flight time.

### What the missile physics does to engagements

Gravity and its autopilot bias, steering only across the body axis, a real
normal force with drag due to lift, the guidance response lag, and self-destruct (REFERENCE.md §6 and §12.7) together move hits
over 13 curated scenarios × 20 seeds from 150 to **136** against a missile
with none of them past gravity and its bias. The 14 lost hits are all one
long shot: its missile pays for the energy it spends turning and can no
longer reach its target. Every other change in hit count is a kill-probability
roll at the same miss distance. Misses in the tail chases grow (median
4.8 → 5.5 km, 9.2 → 11.0 km, 9.8 → 13.3 km in three of them), and the
median crossing flight falls from 112 s to 86 s.

### What the loft does to engagements### What the loft does to engagements

The loft (REFERENCE.md §6.10) raises hits from 124 to 179 over the 11
scenarios × 20 seeds of an earlier scenario set. Of 77 flipped runs, 18 are
kill-probability rolls; all the rest but one are misses that become hits
because the missile arrives sooner and faster (a 44.3 nmi launch flies 90 s
instead of 107 s). The teaching scenarios (`REFERENCE.md` §14) are each
checked to show their lesson; their remaining misses mostly end on the 120 s
time-of-flight estimate.

---

## Layer 4 — Radar and RWR

Range laws are checkable exactly, because they are analytic.

| Property | Simulated | Theory | Error |
|---|---|---|---|
| Falloff over 2× range | 12.041 dB | 12.0412 dB (`40log₁₀2`) | **exact** |
| Falloff over 4× range | 24.082 dB | 24.0824 dB (`40log₁₀4`) | **exact** |
| Signal ratio for 2× RCS | 2.000× | 2× (linear) | **exact** |
| Radar horizon, both at 25 000 ft | 719.607 km | 719.607 km | **exact** |
| Refraction gain over geometric | 1.155× | `√(4/3)` = 1.1547 | **exact** |
| Swerling 1 mean SNR for Pd 0.9, Pfa 10⁻⁶, one look | 21.144 dB | 21.1436 dB (MathWorks [[MathWorks]](SOURCES.md#mathworks)) | **exact** (closed form) |

The `R⁴` falloff is the defining property of two-way radar and the reason
detection range scales only as the fourth root of transmit power. The
simulator reproduces it to numerical precision.

These are checked on the F-16 type's own receiver budget. Detection is by
a simulated square-law detector — a Swerling 1 return plus receiver noise
against a threshold set by the false-alarm probability (REFERENCE.md 9.10) — and the receiver budget's grounding is a different kind of claim
from these exact laws:

| Budget term | Basis |
|---|---|
| Antenna gain 33.0 dBi | Inside the 32.9–34.3 dBi of the F-16 radar's 75.3 × 48.8 cm array [P] |
| Average power 200 W, peak 20 kW | APG-66 family (SASS variant) [P] — a family figure, not the APG-68's |
| Frequency 9.86 GHz | [S] |
| Noise figure 4 dB | Estimate, recorded as such |
| Time on target 47 ms, frame time 4.4 s | The APG-66's 60°/s scan rate and 0.2 s turnaround (US Patent 4,673,938 [[Phillips 1987]](SOURCES.md#phillips-1987)) [P]; a ±60°, 2-bar frame is our default |
| Track confirmation 3 of 5 frames | "Three or more scans" (*Fundamentals of Naval Weapons Systems*, ch. 6 [[Navy FNWS]](SOURCES.md#navy-fnws)) [P]; the 3-frame drop is an estimate |
| STT frame 0.1 s, lock broken after 10 frames | Both estimates: no published STT update rate was found; STT blinding other targets is US Patent 4,673,938 [P]. The 10-frame break replaced a 3-frame one that broke locks every ~4 s at launch range (REFERENCE.md 9.6) |
| Search threshold 11.40 dB | ln(1/Pfa) at Pfa 10⁻⁶, exact for a square-law detector; the detector's Monte Carlo Pd matches the Swerling 1 closed form and Marcum's steady-target value (unit tests) |
| System loss 7.96 dB | **Fitted**: one look at a 5 m² Swerling 1 target is a coin flip at the cited 50 nmi, in free space (the cited range is observed, so the air's ~0.4 dB is inside the loss). **Just below** the 8–10 dB that Lincoln Laboratory's [[O'Donnell 2002]](SOURCES.md#odonnell-2002) worked examples carry — see Where it is weakest |

The cited range itself is the weak point: published APG-68 ranges run from
35 to 57 nmi and none states its target; 50 nmi is at the top. The seeker's
budget (X-band) is mostly estimates and its fitted 3.5 dB loss absorbs
them. The RWR's −65 dBm sensitivity is the Navy EW handbook's typical
figure [P]; its 0 dBi antenna is an estimate. With them the RWR hears the
F-16's radar in its main lobe out to ~465 nmi against the radar's own
50 nmi — the one-way/two-way asymmetry, now with sourced numbers. In
practice the Earth cuts it off first: the RWR shares the radar's 4/3-Earth
horizon (389 nmi with both at 25 000 ft). Its missile warning (an active
seeker transmitting at the aircraft) takes no link budget: on the seeker's
100 W average power alone the link reaches ~8.8 nmi, and any pulsed duty
cycle below ~0.6 carries it past the ~10 nmi acquisition range
(REFERENCE.md 10.4).

The seeker has the radar's Doppler notch (REFERENCE.md 8.3): the
same beam-on-the-ground and ground-relative 15 m/s rule, with the seeker's
own beam, 1.2 λ/D for the 0.15 m dish behind its gain estimate (13.75°).
The dish, the factor and the threshold are all **estimates** — no AIM-120
clutter-filter figure is public. A unit test (`ActiveSeekerDopplerNotch`)
checks the geometry and both effects: a beaming co-altitude target delays
acquisition and, held past the memory coast, breaks a measuring seeker's
lock; with sky behind the beam, or a closing target, nothing changes.

Clear-air absorption at X-band is a fraction of a dB at these altitudes and is
carried by each radar's fitted system loss (REFERENCE.md 9.9; ITU-R P.676-13
[[ITU-R P.676]](SOURCES.md#itu-r-p676) Annex 1 [P]).

**The target's RCS is an estimate from geometry, not RCS data**
(REFERENCE.md 9.2). No measured RCS of the F-16 — or any aircraft here —
is public, so its mean RCS seen from a direction is its projected area that
way: four body-axis ellipsoids (fuselage, wing, horizontal tail, fin) sized
from the repo's own geometry (JSBSim `f16.xml` metrics and structure
points, the type's hit box), with two estimates (4 % thickness, horizontal
tail aspect ratio 3.0). Projected area is exactly a large sphere's RCS
(πr²) and grows with the number of scatterers a Swerling 1 target is
pictured as; it is not a measurement of any real airframe. It gives 3.78 m²
nose-on — inside the ~1–5 m² of open-literature frontal estimates for the
type — 25.4 m² beam-on and 55.8 m² from above or below, and it follows the
target's full attitude, so a banking target shows its planform. What can be
checked is checked (unit tests): the C++ projected areas equal
`tools/fit/rcs_shape.py`'s to 10⁻⁹ m², the pattern is ordered nose < beam <
planform, and bank and pitch rotate the view as they should.

The horizon uses the **4/3-Earth model**, standard radar practice: the
atmosphere's refractive index falls with altitude, bending the beam gently
downward so it follows the curvature slightly. Radar therefore sees about
15% further than pure geometry would allow. The last row confirms the
refraction is actually present rather than the geometric horizon being
reported.

---

## Layer 5 — Navigation

The missile knows its own position, so there is no navigation model to
validate: the one navigation error that changes an engagement is the
launcher's radar track relayed to the missile, whose error follows from the
radar's beam and signal-to-noise ratio (REFERENCE.md §9.11, §11.3).

---

## Layer 6 — Lethality

Kill probability is a sigmoid in miss distance, rescaled to 1 at zero
miss (REFERENCE §12.4), and the model is a Bernoulli trial against it.
The check is whether the empirical kill rate converges to the nominal
probability the sigmoid returns at the miss distances actually achieved.

Over **500 runs** of the head-on baseline engagement, differing only in
random seed (re-measured 2026-10-29), every missile passed through the
airframe: **500 kills at a nominal Pk of 1**, with zero runs failing to
resolve. Where the roll is uncertain, 300 seeded trials at a fixed 10 ft
miss (`Pk = 0.7675`) gave an empirical **0.740**, about one binomial
standard error (2.4 points) low, and every miss among them was labelled
`warhead=no_kill`. That is the expected behavior of a correctly wired
stochastic draw: the terminal roll is neither biased nor double-drawn.

---

## Where it is weakest

Five checks of 117 are outside tolerance:
- the 500 ft/min afterburning climb envelope at Mach 1.0, 1.2 and 1.8;
- sea-level afterburning fuel flow (−5.9%, Fuel flow above);
- the missile's turn ceiling at 700 m/s against the IASGATG CFD study [[IASGATG 2014]](SOURCES.md#iasgatg-2014)
  (+79%, a known discrepancy, Normal force and turn ceiling above).

Below are those, plus the honest limits on everything above.

**Supersonic climb at the edge of the envelope is a little off**: the
500 ft/min envelope at Mach 1.2 and 1.8 is 3.0% and 4.5% low (1 800 and
2 600 ft), and at Mach 1.0 3.1% high. Supersonic drag due to lift is
NASA's (TP-3414 [[Saltzman 1994]](SOURCES.md#saltzman-1994) flight at Mach 1.3, TP-3355 [[Fox 1993]](SOURCES.md#fox-1993) tunnel at 1.6) and the manual's
sustained turn at Mach 1.2 / 40 000 ft also says the model has slightly
too much of it (−6%); the flight-test and tunnel sources and the manual
disagree by about that much, and the model keeps the NASA values.

**High-altitude thrust is calibrated, not measured.** Above the highest
fitted acceleration (40 000 ft afterburning, 30 000 ft military) there are
no engine data. Afterburning thrust lapses as `δ^1.25`, the exponent
calibrated to the combat ceiling chart; military thrust at 40 000 ft is
fitted to the MIL climb envelope. Each is then checked against charts it
was not fitted to (50 000 ft turns, MAX AB envelope, MIL combat ceiling),
which agree within 0–5%. The fitted MIL thrust at 40 000 ft keeps 88% of
its 30 000 ft value at Mach 1.0, more than pressure alone (65%) would
leave; that probably absorbs a small error in the 30 000 ft MIL fit near
Mach 1, where it fits worst.

**Mach effects rest on few measured points.** Lift-curve slope and
static margin are measured only at Mach 1.6 and above (tunnel), and drag
due to lift at Mach 0.6, 0.9, 1.3 (flight) and 1.6 (tunnel). Below Mach
0.9 lift follows DATCOM theory [[USAF DATCOM]](SOURCES.md#usaf-datcom), between 0.9 and 1.6 it is faired, and
above 1.6 the static margin is held at the Mach 1.6 measurement. Subsonic
maximum lift is held at its low-speed value; real wings lose some with
Mach, and no F-16 figure was found. Subsonic static stability is the
model's own (NASA TP-1538 [[Nguyen 1979]](SOURCES.md#nguyen-1979) low-speed data plus an undocumented Mach term);
no F-16C measurement was found to check it.

**The AoA limiter is calibrated, not measured.** The 1 g limit, about
25°, matches published descriptions. The G/AoA blend (the limit falls by
0.8° per G) and the loss of high-AoA lift with Mach are fitted to the
flight manual's quickest turns, which they then reproduce within 3% up
to 40 000 ft. Neither is a published F-16 schedule; the real FLCS's
blend is not public. At 50 000 ft a full pull still overshoots the
blended limit by about 1°.

**Thrust is fitted, so the accelerations confirm the fit, not the
engine.** Installed thrust is whatever makes the model's drag reproduce
the flight manual's accelerations. Where the drag is wrong, thrust
absorbs the error. Two places where that shows: military-power
accelerations fit worse than afterburning ones up to 20 000 ft (0.7–2.0 s against
0.4–0.6 s rms), running slow at 20 000 lb and fast at 28 000 lb at
30 000 ft, which points to slightly too little drag due to lift around
lift coefficient 0.4–0.65 (below the lift coefficient the turn charts
measure); and at sea level near Mach 1 the model's drag falls slightly as
weight rises, where the manual shows speed insensitive to weight.

**Fuel flow is calibrated, and drag cannot be separated from it.** It
reproduces the manual's fuel-flow chart to about 3% rms and cruise to 5%
at 18 of 18 points, but sea-level afterburning is 6% low, and the
part-throttle factor is an effective one that absorbs any drag error at
CL 0.2–0.5, which the manual's data cannot separate from it (Fuel flow
above). JSBSim
slews fuel flow at 1 000 pph/s at military power and 5 000 pph/s in
afterburner, so after a throttle step the fuel burned lags the thrust by
several seconds: afterburner from cruise at sea level burns 14% less fuel
over the manual's acceleration than the tables show. It is in JSBSim's
source, not its data.

**Drag between Mach 0.9 and 1.3 is constructed, not measured.** The
drag-rise shape there follows Raymer's [[Raymer]](SOURCES.md#raymer) standard method between the
measured points. Supersonic drag above Mach 1.6 comes from a wind-tunnel
model, not flight.

**The missile's zero-lift drag is not from data.** The AIM-120's
`Cd(M)` table (`REFERENCE.md` §6.4) is a hand-shaped curve: flat 0.30
subsonic, a 0.60 peak at Mach 1, easing to 0.30 at Mach 4. Only its shape
is checked; its magnitudes have not been compared with any published
AIM-120 or comparable-missile drag. Coast-phase deceleration, the peak
Mach in Flight phases above, and everything that depends on range
therefore rest on an unsourced curve. One table covers the whole
flight (`REFERENCE.md` §6.4). The table is kept as an illustrative
shape on purpose: no measured AIM-120 drag is public to
check it against. Its accuracy against
the real missile is therefore unknown, and the sim's ranges and flight
times should be read as trends.

**The missile's turn ceiling rests on thin ground.** Its normal-force
tables are fitted to published theory checked against wind-tunnel data,
but:
- it is applied to an airframe whose nose, chords and fin stations are
  measured off a photograph (±0.5 in);
- it is untrimmed, so drag due to lift is slightly low and the ceiling
  slightly high;
- the two numbers that bound it, α_max 30° and the 30 G structural cap,
  both come from one secondary CFD study;
- that same study's turn ceiling disagrees with the model by 79% at
  700 m/s (Normal force and turn ceiling above).

**The missile's seeker settings are not published.** The gimbal cone
(45°), memory coast (1.5 s), tracking-loop lag (0.15 s) and the 3-of-5
acquisition rule are not published. Nor is any of the active seeker's
receiver hardware: its band (X) has one secondary source, and its power,
antenna, noise figure and integration time are estimates; one fitted loss
makes it acquire a 5 m² target at a commonly quoted ~10 nmi. They are
plausible and internally consistent, but they are not data.

**The radar's detection range rests on one disagreeing citation.** The
receiver budget is grounded term by term, but the 50 nmi its one fitted
loss is fitted to sits at the top of a 35–57 nmi spread of secondary
claims, none of which states its target, its fluctuation model or what
"detection" meant. Read as a per-look coin flip against a fluctuating
(Swerling 1) fighter, it needs a 7.96 dB loss, just below the 8–10 dB
Lincoln Laboratory [[O'Donnell 2002]](SOURCES.md#odonnell-2002) calls typical: the citation asks slightly more of this
hardware than is usual. Against a steady target the same citation needed
9.1 dB.
Every launch range in the simulator scales with it. The citation states no
altitude either, and with no absorption modelled the radar reaches as far
at sea level as at altitude; real air would cost it ~10 % low down.

**A fighter's fluctuation statistics are assumed, not measured.** Swerling
1 rests on one general [P] statement that observed aircraft data agree with
Cases 1 and 2 (Fehlner, 1963 [[Fehlner 1963]](SOURCES.md#fehlner-1963)); no fighter-specific RCS distribution was
found, and Swerling's own report was not read (it is not openly hosted).
The choice moves confirmation ranges: under a 13-of-15 firing tier (REFERENCE.md 9.6,
*Why one threshold*) a stationary reference's 50 % confirmation would sit at
29.4 nmi for Swerling 1, 33.1 nmi for Swerling 3 (one dominant scatterer) and
34.9 nmi under a 3 dB log-normal jitter. Every look is independent
of the last — real returns correlate over short aspect changes, and at a
4.4 s frame that matters little.

**Every target's RCS is a silhouette, not a measurement.** The F-16's
mean RCS is its projected area toward the radar, an estimate
from the repo's geometry with no RCS data behind it. A real fighter's
pattern is spikier — flat sides and surfaces flash far above their
silhouette over a degree or two, and the inlet, nozzle and radar antenna
behind the radome dominate the nose-on return — and none of that is in
the model, which instead rises smoothly and quickly off the nose (13.7 m²
at 30° against 3.78 m² nose-on). The least certain input, the fuselage
width, moves the nose-on value between 3.0 and 4.6 m². The change from the
hand-set heading-only table (2.0 m² nose-on, 15 m² beam-on, itself
unsourced) moved every launch earlier: `scenario_basic`'s median launch
from 23.7 to 31.1 nmi.

**The air absorbs nothing and it never rains.** Clear-air absorption is
not modelled (the fitted loss carries its share at the reference range);
rain, which at X-band can cost several times the clear-air loss, is not
modelled at all. At the 25 000 ft the scenarios fly the gap is small; low
down it is not.

**The radar's track error rests on textbook forms and three estimates.**
The error law — angle ∝ beamwidth / √SNR, range ∝ pulse width / √SNR — is
Lincoln Laboratory's, and the monopulse slope reproduces its 10:1 beam
splitting at 13 dB; the aperture is the F-16 array's. But the beamwidth
factor (1.2 λ/D), the pulse width (1 µs, inside the published 0.81–4 µs)
and the tracker's 3 g manoeuvre allowance are estimates, and no APG-68
accuracy figure is public to check the result against. There is no bias
or floor error (radome, boresight), so close-in tracks are better than a
real radar's. The allowance matters most: it sets how well the track
knows the target's velocity at long range (tens of m/s), which is what a
stale datalink track carries forward.

**The missile's 120 s self-destruct time is an estimate.** No primary
source gives an AIM-120 time of flight or battery life. 120 s was set
above every hit the simulator produced (≤ 86 s), but with the loft
hits fly up to 119 s, and the limit now shapes outcomes: in the
220-run batch it ends 18 shots, and at 200 s five of them would hit
(179 → 184). It, not the physically grounded 2 G floor, ends the lost
shots it touches (`REFERENCE.md` §12.7).

**Missile range is checked only against a band.** Published AMRAAM
ranges (75–160 km, 40–86 nmi, by variant) assume a lofted trajectory and
state no altitude, speed or target. The missile lofts
(`REFERENCE.md` §6.10), and its head-on Rmax — 46–53 nmi at 5 000 ft,
60–70 at 25 000 ft, 72–81 at 45 000 ft — falls inside that band, from
which nothing finer can be concluded. The loft schedule itself is fitted
to this model (no public AIM-120 loft angle exists), and head-on Rmax is
still set by the 120 s time-of-flight estimate above, not by energy.

**Turn performance is validated at one condition.** Mach 0.8 at 15 000 ft.
The altitude trend is verified for shape across four conditions, but only
one has a published value behind it.

**Several targets are secondary sources**, specifically the F-16 turn
figures and the AMRAAM mass and motor data. Where a conclusion rests on
one, its robustness is stated explicitly — the specific-impulse bound, for
instance, holds even at ±30% error in the propellant estimate, because
the physical limit it tests against is far away.

**Not modeled at all**, and therefore not assessed: terrain and terrain
masking, electronic countermeasures, chaff and flares, multipath,
precipitation, humidity, atmospheric absorption, turbulence and wind
shear (one steady wind is modeled), and any
aircraft other than the F-16. The full catalogue is in
`REFERENCE.md` §16.

---

## On the method

Two things distinguish these checks from the simulator's other test
suites, and both are deliberate.

**They compare against reality, not against the code.** The project
carries a suite of closed-form checks that verify the code computes the equation
it claims — that missile integration matches `x₀ + v₀t + ½at²` to twelve decimal places,
that the radar equation falls off at exactly `40log₁₀R`. Every one of
those can pass while the thing being computed describes a vehicle that
does not exist. This document is the only place that question is asked.

**They are capable of failing.** A validation suite that only ever
confirms is measuring nothing. These checks have caught real defects — the
propulsion model was found implying a 340-second specific impulse against
a ~245 s chemical limit, an error invisible to four tiers of internal
testing because the code was computing its impossible equation perfectly.
What appears above is the state of the simulator as it stands.

---

## Sources

Every source cited here is listed in [SOURCES.md](SOURCES.md), tagged
**[P]** or **[S]**, with what it was used for.

### Companion documents

- [REFERENCE.md](REFERENCE.md) — the modeling and simulation guide: every
  equation the simulator evaluates, with derivations and parameter
  provenance.
- [SCHEMA.md](SCHEMA.md) — scenario authoring reference.
- `tests/real_world_validation.cpp` — the program producing every
  measurement above.
