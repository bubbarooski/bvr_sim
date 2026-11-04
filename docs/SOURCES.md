# Sources

Every source the simulator's numbers and methods rest on, in one place.
[REFERENCE.md](REFERENCE.md) and [VALIDATION.md](VALIDATION.md) cite these
inline by key — for example [[Nguyen 1979]](#nguyen-1979) — and explain
there how each value was derived; this list says what each source is and
what it was used for.

Each entry is tagged:

- **[P]** — primary: flight manual, government or NASA technical report,
  manufacturer datasheet, service fact sheet, patent, international
  standard, or first-principles physics
- **[S]** — secondary: reference sites, textbooks recalled without a page
  check, technical communities, simulator and game material, derived
  estimates

A figure from an [S] source is not worthless, but a model measured
against one inherits its uncertainty. Sources that were consulted and
supplied no value say so; lists of search results that were seen but not
read are kept at the end of each group so nothing consulted goes uncited.
Within a group, primary sources come first.

---

## Atmosphere and the Earth

### ISA

**[P]** U.S. Standard Atmosphere 1976 / International Standard Atmosphere.

Used for: troposphere lapse rate, tropopause at 36 089 ft, sea-level
density 0.0023769 slug/ft³, sea-level speed of sound 1116.45 ft/s —
[REFERENCE §3.1](REFERENCE.md#31-us-standard-atmosphere-1976),
[VALIDATION Layer 1](VALIDATION.md#layer-1--atmosphere).

### IUGG

**[P]** International Union of Geodesy and Geophysics mean Earth radius,
6 371 000 m.

Used for: the Earth radius in the radar horizon —
[REFERENCE §9.5](REFERENCE.md#95-radar-horizon).

### 4/3 Earth

**[P]** The 4/3-Earth refraction model — standard radar engineering
practice for line of sight in a normally refracting atmosphere.

Used for: the radar and RWR horizon —
[REFERENCE §9.5](REFERENCE.md#95-radar-horizon),
[REFERENCE §10.1](REFERENCE.md#101-the-friis-one-way-link),
[VALIDATION Layer 4](VALIDATION.md#layer-4--radar-and-rwr).

---

## F-16 — airframe, aerodynamics and flight control

### Nguyen 1979

**[P]** L. T. Nguyen et al., *Simulator Study of Stall/Post-Stall
Characteristics of a Fighter Airplane With Relaxed Longitudinal Static
Stability*, NASA TP-1538, December 1979.
<https://ntrs.nasa.gov/citations/19800005879>

Used for: the wind-tunnel database behind the JSBSim F-16's lift, drag
and moment tables (the model file cites it). The envelope's `CL_max` of
1.65 is the lift these tables give at the ~25° AoA limit, with the
leading-edge flaps scheduled down —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[REFERENCE §5.1](REFERENCE.md#51-available-load-factor-vn),
[VALIDATION: What the turn performance depends on](VALIDATION.md#what-the-turn-performance-depends-on).

### JSBSim

**[P]** JSBSim Flight Dynamics Model, with its General Dynamics F-16A
model (Block 32, basic US configuration) and F100-PW-229 engine deck.
Open source and directly inspectable. <https://jsbsim.sourceforge.net/>

Used for: the aircraft's six-degree-of-freedom flight model, and the
model file's own metrics (`data/jsbsim_data/aircraft/f16/f16.xml`: wing
area 300 ft², span 30 ft, mean chord 11.32 ft, horizontal and vertical
tail areas 63.7 and 54.75 ft², and the structure points — intake lip,
eyepoint, ventral fins, main gear, fin tip) that size the RCS shape's
ellipsoids —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[REFERENCE §9.2](REFERENCE.md#92-radar-cross-section),
[VALIDATION Layer 2](VALIDATION.md#layer-2--the-f-16).

### Saltzman 1994

**[P]** E. J. Saltzman and J. W. Hicks, *In-Flight Lift-Drag
Characteristics for a Forward-Swept Wing Aircraft (and Comparisons With
Contemporary Aircraft)*, NASA TP-3414, December 1994.
<https://web.stanford.edu/~cantwell/AA200_Course_Material/NASA_Tech_paper_3414_X29_performance_1994.pdf>

Used for: Table 5, F-16C subsonic zero-lift drag 0.0199 (flight-derived);
Figure 23, F-16C zero-lift drag at Mach 0.6, 0.9 and 1.3 (read as 0.020,
0.023, 0.046), whose 0.9 and 1.3 points set the compressibility drag;
Figure 24, F-16C drag due to lift ΔCD/ΔCL² 0.122 / 0.139 / 0.268 at Mach
0.6 / 0.9 / 1.3, which sets the compressibility drag due to lift. Figure
16 (X-29A lift-curve slope against Mach) is used only as a fighter's
characteristic shape. The F-16C data are from unpublished flight tests
cited by the authors —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[VALIDATION: Lift, stability and drag due to lift](VALIDATION.md#lift-stability-and-drag-due-to-lift-with-mach),
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

Also: the top of its F-16C flight-measured lift range (CL ≈ 0.65) is where the
drag above the polar break starts to be added (a term fitted to the flight manual's
sustained turns) — [REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim).

### Fox 1993

**[P]** M. C. Fox and D. K. Forrest, *Supersonic Aerodynamic
Characteristics of an Advanced F-16 Derivative Aircraft Configuration*,
NASA TP-3355, July 1993.
<https://ntrs.nasa.gov/api/citations/19930022544/downloads/19930022544.pdf>

Used for: Figure 31, F-16C wind-tunnel minimum drag on its own reference
area at Mach 1.6, 1.8, 2.0 and 2.16 (0.051, 0.050, 0.0487, 0.0478; LEF and
TEF at −2°, AIM-9J wingtip missiles), with Figure 22's drag of two AIM-9L
missiles removed — these set the supersonic drag; Figure 31's F-16C
lift-curve slope (0.068 / 0.058 / 0.052 / 0.048 per deg), which sets the
supersonic lift factor; Figure 30 (Mach 1.6), the pitching-moment slope
−0.257 about 0.35 MAC, which sets the supersonic static margin, and the
polar, whose ΔCD/ΔCL² ≈ 0.28 sets the Mach 1.6 drag due to lift. Its
AIM-120 models were also consulted; they were half-buried in the test
configuration and not usable —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[VALIDATION: Lift, stability and drag due to lift](VALIDATION.md#lift-stability-and-drag-due-to-lift-with-mach),
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

### MIL-F-8785C

**[P]** MIL-F-8785C, *Flying Qualities of Piloted Airplanes*.

Used for: §3.2.2, the short-period requirements the pitch flight control
system is designed to — control anticipation parameter 0.28–3.6 and
damping 0.35–1.30 for Level 1, Category A —
[REFERENCE §4.4](REFERENCE.md#44-the-f-16-pitch-flight-control-system).

### Jobe 1984

**[P]** C. E. Jobe, *Prediction of Aerodynamic Drag*, AFWAL-TM-84-203, Air
Force Wright Aeronautical Laboratories, 1984.
<https://apps.dtic.mil/sti/tr/pdf/ADA326842.pdf>

Used for: context only — drag-prediction methods with a component
breakdown of YF-16 flight-test drag. Its F-16 values are in figures and
were not used numerically.

### USAF DATCOM

**[S]** USAF Stability and Control DATCOM — the Helmbold–Polhamus
finite-wing lift-curve slope with Prandtl–Glauert compressibility.

Used for: the subsonic lift-curve-slope Mach factor (with the F-16 wing's
aspect ratio 3.0 and 22.7° half-chord sweep: +8% at Mach 0.6, +17% at
0.8, +24% at 0.9, relative to Mach 0.2) —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[VALIDATION: Lift, stability and drag due to lift](VALIDATION.md#lift-stability-and-drag-due-to-lift-with-mach).

### FTSW 2022

**[S]** *The Safety Value of Protected Flight Controls*, Flight Test
Safety Workshop 2022; with forum posts quoting the F-16 flight manual
(T.O. 1F-16C-1).
<https://www.flighttestsafety.org/images/FTSW_2022_-_The_Safety_Value_of_Protected_Flight_Controls.pdf> ·
<https://forum.falcon-bms.com/post/3027> ·
<https://www.f-16.net/forum/viewtopic.php?p=125840>

Used for: the F-16's AoA limit, about 25° in 1 g flight (25.2°, blending
to about 25.5°), set as a margin below the directional-stability loss
near 29° — the AoA at which `CL_max` is read
([REFERENCE §5.1](REFERENCE.md#51-available-load-factor-vn)). The site
refuses automated downloads, so this is taken from its published summary
as returned by search. The forum posts agree on ~25° at 1 g and add a
G/AoA blend (about 15° at 9 G with Category I stores), which is
low-confidence and not modelled.

### Mason F-16

**[S]** W. H. Mason, Virginia Tech, *General Dynamics F-16 Fighting
Falcon* course slides, drawing on Droste and Walker's case study.
<https://archive.aoe.vt.edu/mason/Mason_f/F16S04.pdf>

Used for: background for the AoA limiter — the F-16 FLCS feature list
(AoA limiter: basic, roll-rate and yaw-rate; ARI scheduled on AoA and
Mach).

### Brandt 2004

**[S]** S. A. Brandt, R. J. Stiles, J. J. Bertin and R. Whitford,
*Introduction to Aeronautics: A Design Perspective*, AIAA, 2004.
<https://books.google.com/books/about/Introduction_to_Aeronautics.html?id=NJE3wTL6IDYC>

Used for: nothing — its F-16 case study and drag/thrust curves were
sought for the top-speed analysis, but the tables were not accessible.

### ICCFD9 2016

**[S]** ICCFD9-2016-239, F-16 aerodynamic analysis by CFD.
<https://iccfd9.itu.edu.tr/assets/pdf/papers/ICCFD9-2016-239.pdf>

Used for: context only — total drag at Mach 0.3, 0.9 and 1.6, with
pressure drag about 80% of the total at Mach 1.6, which is why
supersonic drag is dominated by wave drag. A CFD study, not flight data;
no number is taken from it.

---

## F-16 — engine and installed thrust

### Aero-Matic

**[P]** JSBSim Aero-Matic source, `utils/aeromatic++/Systems/Propulsion.cpp`.
<https://github.com/JSBSim-Team/jsbsim/blob/master/utils/aeromatic%2B%2B/Systems/Propulsion.cpp>

Used for: showing that the F100 engine file's augmented-thrust table is
the one generic table Aero-Matic writes for every afterburning engine,
not F100 data. Its shape is kept only where the flight manual's data do
not reach —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

### F100 ratings

**[S]** F100-PW-229 ratings and test conditions.
<https://www.flightglobal.com/pw-completes-f100-229a-altitude-testing/19565.article> ·
<https://engineering.purdue.edu/AAE/research/propulsion/Info/jets/tfans/f100> ·
<https://www.f-16.net/forum/viewtopic.php?p=134361>

Used for: 29 100 lbf maximum afterburning and 17 800 lbf maximum dry
thrust, and altitude testing to Mach 2.3 at 40 000 ft simulated. None
gives installed thrust at altitude; the fitted static sea-level thrust
(27 713 lbf) is compared against the 29 100 lbf rating as a check —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

---

## F-16 — published performance and flight test

### F-16C flight manual

**[P]** *Flight Manual Supplement, F-16C/D Blocks 50 and 52+*, T.O.
GR1F-16CJ-1-1, Hellenic Air Force, 15 June 2003, Appendix A (F100-PW-229,
"data basis flight test"); with the flight manual itself (GR1F-16CJ-1,
same source). Public copy, cited by Wikipedia's F-16C specifications.
<https://publicintelligence.net/hellenic-air-force-f-16cd-flight-manuals/>

Used for:

- the MIL and MAX AB level-acceleration tables at drag index 0, sea level
  to 40 000 ft and 20 000–36 000 lb (Figs. A8-5 to A8-13: time, distance
  and fuel from 200 KIAS), to which the engine's installed thrust is
  fitted;
- the combat ceiling chart (Fig. A8-1, MAX AB and MIL), to which the
  afterburning lapse above 40 000 ft is calibrated and against which MIL
  is checked; the 500 ft/min climb envelope (Fig. A8-2), to whose MIL
  curve military thrust at 40 000 ft is fitted and whose MAX AB curve is
  an independent check; the definition of combat speed (500 ft/min of
  climb remaining) and the climb-schedule definitions;
- the airspeed limits (800 KIAS, Mach 2.03 at 40 000 ft) that the
  published top speeds turn out to be; drag index 0 = basic aircraft with
  tip missiles;
- the Turn Performance charts (Figs. A8-40, -41, -43, -45, -47; MAX AB,
  22 000 lb), whose Ps = 0 lines give the sustained turns the drag above
  the polar break is fitted to (15 000–40 000 ft) and checked against
  (50 000 ft);
- the Fuel Flow chart (Fig. A8-4, MIL and MAX AB fuel flow against Mach,
  1 000 to 60 000 ft), to which the engine's TSFC is fitted, and the
  Subsonic Cruise tables (total fuel flow at drag index 0, sea level to
  40 000 ft), which check part-throttle fuel flow and, at weights other
  than 24 000 lb, fit the part-throttle factor;
- the Flight Controls description of the flaps (the TEFs droop only with
  the gear handle down, 20° below 240 knots to retracted at 370).

Tables parsed from the PDF text; chart curves digitised from the PDF's
vector paths, or read off high-resolution renders where a curve is drawn
in fragments. The data are in `data/aircraft/f16c_block50/reference/` —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[REFERENCE §4.4](REFERENCE.md#44-the-f-16-pitch-flight-control-system),
[VALIDATION: Turn performance](VALIDATION.md#turn-performance),
[VALIDATION: Fuel flow](VALIDATION.md#fuel-flow),
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

### USAF F-16 fact sheet

**[P]** U.S. Air Force, *F-16 Fighting Falcon* fact sheet.
<https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104505/f-16-fighting-falcon/> ·
<https://wingsoverhouston.com/wp-content/uploads/2016/08/F16-Fact-Sheet.pdf>

Used for: structural limit, engine and dimensions. Its ceiling, "above
50 000 feet", is a floor at no stated weight: consulted, not used as a
target. af.mil refuses automated access, so the figures were read from
reproductions of the sheet.

### Baugher

**[S]** Joe Baugher, *F-16C Block 50/52* specifications.
<http://www.joebaugher.com/usaf_fighters/f16_4c.html>

Used for: maximum speed Mach 2.02 at 40 000 ft (short endurance) and Mach
1.89 sustained, both at 27 245 lb; wing area 300 ft²; empty 18 238 lb;
normal loaded 26 463 lb; thrust 29 000 lbf —
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

### Wikipedia F-16

**[S]** Wikipedia, *General Dynamics F-16 Fighting Falcon*, specifications
(F-16C Block 50 and 52).
<https://en.wikipedia.org/wiki/General_Dynamics_F-16_Fighting_Falcon>

Used for: maximum speed Mach 2.05 at 40 000 ft clean and Mach 1.2 at sea
level (both citing Frawley, *The International Directory of Military
Aircraft*, 2002, p. 114); 7 000 lb internal fuel; 50 000 ft service
ceiling. Its specifications cite the F-16C flight manual above, which
shows Mach 2.05 and 1.2 to be airspeed limits rather than thrust-limited
speeds —
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

### F-16 turn figures

**[S]** F-16 turn performance figures from aviation reference sites.
<https://boltflight.com/f-16-turn-rate-a-detailed-analysis-of-maneuverability/> ·
<https://www.f-16.net/forum/viewtopic.php?f=2&t=26829>

Used for: 14 °/s sustained at Mach 0.8 / 15 000 ft; ~25.5 °/s
instantaneous at low altitude; corner velocity 330–440 KCAS —
[VALIDATION: Turn performance](VALIDATION.md#turn-performance).

### f-16.net F-16A/B

**[S]** f-16.net, *F-16A/B* specifications.
<https://www.f-16.net/f-16_versions_article3.html>

Used for: nothing now — service ceiling 55 000 ft and initial climb
62 000 ft/min, no weight stated, for the F-16A/B with the less powerful
F100-PW-200. Superseded as the ceiling target by the F-16C flight manual.

---

## AIM-120 missile

### USAF AIM-120 fact sheet

**[P]** U.S. Air Force, *AIM-120 AMRAAM* fact sheet; with the USAF ACC
*AIM-120* fact sheet and NAVAIR *AMRAAM* page.
<https://www.acc.af.mil/Portals/92/Docs/Fact%20Sheets%20-%202020%20Update/AIM-120_Formatted.pdf> ·
<https://www.navair.navy.mil/product/AMRAAM>

Used for: length 143.9 in, diameter 7 in; a transmitter and monopulse
seeker (no band stated) —
[REFERENCE §6.4](REFERENCE.md#64-aerodynamic-drag).

### US 6,742,433

**[P]** US Patent 6,742,433 (Raytheon / General Dynamics, 2004).

Used for: the AIM-120C's tail fins fill a 12.4 in square box (A/B:
17.44 in), so the X-mounted span is its diagonal, ~17.5 in —
[REFERENCE §6.5](REFERENCE.md#65-missile-normal-force-and-turn-ceiling).

### Watson ADA355385

**[P]** Watson, DTIC ADA355385.

Used for: AIM-120C wing and tail spans 17.5 in (table). Its Fig 2 C-5
side view gives the nose length, chords and surface stations, measured
off the image and scaled by the diameter (±0.5 in, photo-derived — no
dimensioned drawing is public). The figures are kept as `docs/amraam.png`
(Distribution A, approved for public release) so the measurements can be
re-checked —
[REFERENCE §6.5](REFERENCE.md#65-missile-normal-force-and-turn-ceiling).

### USAF AIM-120 caption

**[P]** USAF AIM-120 image caption (af.mil): "wing span AIM-120A/B, 1 ft.
9 in.; AIM-120C, 1 ft. 5 in.".

Used for: corroboration of Watson's 17.5 in, no value. Read from a search
excerpt only; the page refuses automated access.

### Wikipedia AIM-120

**[S]** Wikipedia, *AIM-120 AMRAAM*. <https://en.wikipedia.org/wiki/AIM-120_AMRAAM>

Used for: launch mass 152 kg, diameter 178 mm, length 3.65 m, Mach 4
maximum speed, boost-sustain motor, HTPB reduced-smoke propellant, range
by variant —
[VALIDATION: Mass, geometry, and motor](VALIDATION.md#mass-geometry-and-motor).

### WPU-6/B estimates

**[S]** AIM-120 WPU-6/B motor estimates, community technical discussion.
<https://www.key.aero/forum/modern-military-aviation/missiles-and-munitions/100370-aim-120-range-questions>

Used for: 156 lb motor mass, ~85% propellant fraction, ~245 s specific
impulse for an HTPB solid. Treated as approximate; the conclusions drawn
from it are robust to substantial error —
[VALIDATION: Mass, geometry, and motor](VALIDATION.md#mass-geometry-and-motor).

### Solid propellant Isp

**[S]** Specific impulse of composite solid propellants, standard
propulsion reference material.

Used for: the 230–260 s band, the physical bound on the motor model —
[REFERENCE §6.2](REFERENCE.md#62-three-stage-propulsion),
[VALIDATION: Mass, geometry, and motor](VALIDATION.md#mass-geometry-and-motor).

### designation-systems

**[S]** designation-systems.net, *AIM-120*.

Used for: tail span 44.7 cm, agreeing with the patent.

### f-16.net AIM-120

**[S]** f-16.net, AIM-120 armament article.
<https://www.f-16.net/f-16_armament_article3.html>

Used for: the C "will have 'clipped wings' to reduce its box size from
17.4 to 12.5 inches", consistent with the patent (corroboration, no
value); and "active X-band radar terminal seeker", "high-power
solid-state transmitter" — the seeker's band
([REFERENCE §8.3](REFERENCE.md#83-the-active-seeker)).

### Jane's seeker band

**[S]** DCS forum post quoting *Jane's Land-Based Air Defence 1999–2000*:
"Raytheon I/J band" (8–20 GHz). <https://forum.dcs.world/topic/115438-sarh-vs-arh/>

Used for: corroboration that the seeker is X-band.

### IASGATG 2014

**[S]** IASGATG 2014, AIM-120C CFD study (for DCS).

Used for: maximum α 30° (fin separation), 30 G held from Mach 4 down to
~700 m/s at 10 km, ≥ 10 G transonic. Source of `alpha_max_deg` 30 and the
30 G structural figure; the 700 m/s corner is a known discrepancy —
[REFERENCE §6.5](REFERENCE.md#65-missile-normal-force-and-turn-ceiling),
[VALIDATION: Normal force and turn ceiling](VALIDATION.md#normal-force-and-turn-ceiling).

### AIM-120 span and G claims

**[S]** Navy fact file, military.com, Wikipedia and Janes (AIM-120C wing
span ~19 in, 484 mm); f-16.net and War Thunder forum threads on AMRAAM G
limits.

Used for: nothing — the 19 in span is superseded by Watson's 17.5 in
(none says how it was measured); the threads give a 25–40 G spread of
claims.

---

## Missile aerodynamics — methods and wind-tunnel data

These underlie the component build-up from which the missile's normal-force
tables are computed
([REFERENCE §6.5](REFERENCE.md#65-missile-normal-force-and-turn-ceiling)); the
measured data also check the tables.

### Allen 1951

**[P]** H. J. Allen and E. W. Perkins, NACA Report 1048, 1951.

Used for: body potential plus viscous crossflow normal force,
`sin 2α cos(α/2) + η Cdc (A_p/S) sin²α`.

### Jorgensen 1973a

**[P]** L. H. Jorgensen, NASA TN D-6996, 1973.

Used for: crossflow drag coefficient against crossflow Mach (Fig 1) and
Reynolds number (Fig 2), the compressibility fade (Fig 3), and η against
fineness ratio (Fig 4); digitised, approximate readings.

### Jorgensen 1973b

**[P]** L. H. Jorgensen, NASA TN D-7228, 1973.

Used for: the body-wing-tail component build-up scheme.

### Pitts 1957

**[P]** W. C. Pitts, J. N. Nielsen and G. E. Kaattari, NACA Report 1307,
1957.

Used for: slender-body interference factors K_W(B), K_B(W) and the
wing-vortex downwash on the tail.

### Jorgensen 1976

**[P]** L. H. Jorgensen and Nelson, NASA TM X-3310, 1976.

Used for: measured CN for a body, body-wing and body-wing-tail, Mach
0.6–2.0 to ~50° — the check table and the fin tip-loss choices —
[VALIDATION: Normal force and turn ceiling](VALIDATION.md#normal-force-and-turn-ceiling).

### TM X-3128

**[P]** NASA TM X-3128.

Used for: body N1C1 CN at two Reynolds numbers, Mach 0.6 — the
Reynolds-drop check —
[VALIDATION: Normal force and turn ceiling](VALIDATION.md#normal-force-and-turn-ceiling).

### NTRS 19680026721

**[P]** NASA NTRS 19680026721.

Used for: the trend that normal-force coefficient rises with fineness
ratio (body-alone cylinders of L/d 6 and 8 to 90°), which argues against
the CFD study's lower CN; no value. Found while searching for measured
data on a body as long as the AIM-120's (L/d ≈ 20); none exists in
readable form —
[REFERENCE §6.5](REFERENCE.md#65-missile-normal-force-and-turn-ceiling).

### Missile DATCOM

**[P]** Missile DATCOM user manual, 1997.

Used for: the methods (Helmbold subsonic fin slope, supersonic tip
loss); not run.

### Fleeman

**[S]** E. L. Fleeman, *Tactical Missile Design*, AIAA.

Used for: standard forms of the same build-up, as a cross-check; and the
hint that the subsonic drag table is low
([REFERENCE §6.4](REFERENCE.md#64-aerodynamic-drag)). Recalled textbook
forms; no page was re-checked.

---

## Radar

### Richards

**[S]** M. A. Richards, *Binary Integration Gain*.
<https://radarsp.weebly.com/uploads/2/1/4/7/21471216/binary_integration_gain.pdf>

Used for: M-of-N detection — the frame-by-frame track confirmation —
[REFERENCE §9.6](REFERENCE.md#96-frames-and-track-confirmation).

### Radartutorial

**[S]** Radartutorial, *Track Initiation* and *AN/APG-68*.
<https://www.radartutorial.eu/10.processing/sp22.en.html> ·
<https://www.radartutorial.eu/19.kartei/08.airborne/en/karte024.en.html>

Used for: track initiation by M-of-N confirmation
([REFERENCE §9.6](REFERENCE.md#96-frames-and-track-confirmation)); the
APG-68 as X-band pulse-Doppler, 35 nm look-up / 27.5 nm look-down
("supposed to have"), no RCS. With the Wikipedia figure below and a
105 km-against-5 m² claim on a wiki mirror, the published APG-68 ranges
disagree (35–57 nmi) and none states its target; the model's 50 nmi sits
at the top of that spread
([REFERENCE §9.8](REFERENCE.md#98-parameter-grounding)).

### ScienceDirect TWS

**[S]** *Track-While-Scan*, ScienceDirect Topics.
<https://www.sciencedirect.com/topics/engineering/track-while-scan>

Used for: track-while-scan as one track update per scan —
[REFERENCE §9.6](REFERENCE.md#96-frames-and-track-confirmation).

### Fehlner 1963

**[P]** L. F. Fehlner, "Improved Data for the Classical Determination of
Radar Detection Range", *APL Technical Digest* 3(2), Johns Hopkins APL,
Nov–Dec 1963.
<https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/APL-V03-N02/APL-03-02-Fehlner.pdf>

Used for: Marcum's steady-target and Swerling's fluctuating-target
detection results (square-law detector, false-alarm number, Marcum's
series, eq 8), and "observed data on aircraft targets agree with the
density distribution assumed for Cases 1 and 2" — the exponential RCS
distribution, the basis for the aircraft's fluctuation case —
[REFERENCE §9.10](REFERENCE.md#910-probability-of-detection).

### Swerling 1954

**[P]** P. Swerling, *Probability of Detection for Fluctuating Targets*,
RAND RM-1217, 1954 (ASTIA AD 80638); republished with J. I. Marcum's
papers in *IRE Trans. Information Theory* IT-6, April 1960.
<https://www.rand.org/pubs/research_memoranda/RM1217.html>

Used for: the four fluctuation cases. Catalog record and abstract read;
the report itself was not (RAND does not host it openly). Its cases are
taken through Fehlner and the MathWorks check values —
[REFERENCE §9.10](REFERENCE.md#910-probability-of-detection).

### Albersheim 1981

**[P]** W. J. Albersheim, "A Closed-Form Approximation to Robertson's
Detection Characteristics", *Proc. IEEE* 69(7), 1981.

Used for: the required SNR for a given Pd and Pfa (11.2 dB at Pd 0.5,
Pfa 10⁻⁶), checked against MathWorks' published values —
[REFERENCE §9.10](REFERENCE.md#910-probability-of-detection).

### MathWorks

**[S]** MathWorks documentation: *Modeling Radar Detectability Factors*,
`shnidman`, `albersheim`, and *Designing a Basic Monostatic Pulse Radar*.
<https://www.mathworks.com/help/radar/ug/modeling-radar-detectability-factors.html> ·
<https://www.mathworks.com/help/phased/ref/shnidman.html> ·
<https://www.mathworks.com/help/phased/ref/albersheim.html> ·
<https://www.mathworks.com/help/phased/ug/designing-a-basic-monostatic-pulse-radar.html>

Used for: check values only — single-look detectability at Pd 0.9 / Pfa
10⁻⁶, steady target 13.1217 dB (an approximation; the exact Marcum value
is 13.18 dB), Swerling 1 21.1436 dB (exactly the closed form);
fluctuation loss 8.17 dB at N = 10; Albersheim 13.1 dB at Pd 0.9 / Pfa
10⁻⁶ and 4.99 dB for 10 pulses. `shnidman` (citing Richards,
*Fundamentals of Radar Signal Processing*, 2005) was read for the method
only —
[REFERENCE §9.10](REFERENCE.md#910-probability-of-detection),
[VALIDATION Layer 4](VALIDATION.md#layer-4--radar-and-rwr).

### Wikipedia Swerling

**[S]** Wikipedia, *Fluctuation loss* and *Peter Swerling*.
<https://en.wikipedia.org/wiki/Fluctuation_loss> ·
<https://en.wikipedia.org/wiki/Peter_Swerling>

Used for: the Swerling I density p(σ) = (1/σ̄) exp(−σ/σ̄) (chi-squared,
two degrees of freedom) and the scan-to-scan definition of Case I
(citing Skolnik 2001 and Swerling 1954); RM-1217's four cases built on
Marcum's steady-target analysis, and its ASTIA number, used to look for
the report —
[REFERENCE §9.10](REFERENCE.md#910-probability-of-detection).

### Knott 1993

**[S]** E. F. Knott, J. F. Shaeffer and M. T. Tuley, *Radar Cross
Section*, 2nd ed., Artech House, 1993.

Used for: the physical-optics result that an optically large sphere's
RCS is its projected area, σ = πr² — first-principles radar physics, the
basis for reading a silhouette as a mean RCS. Recalled, no page checked.
No RCS data were consulted for the F-16: the shape is an estimate from
the JSBSim model's geometry, and the ~1–5 m² frontal range quoted in the
open literature is not re-checked —
[REFERENCE §9.2](REFERENCE.md#92-radar-cross-section).

### Forecast International APG-66

**[P]** Forecast International, *APG-66(V)*, Airborne Electronics
Forecast, October 2003 (archived 10/2004; also archived as record 391).
<https://www.forecastinternational.com/archive/disp_pdf.cfm?DACH_RECNO=326> ·
<https://www.forecastinternational.com/archive/disp_pdf.cfm?DACH_RECNO=391>

Used for: the F-16's original fire-control radar, in the same nose —
planar array 75.3 × 48.8 cm, which sets the antenna gain (32.9–34.3 dBi
at 50–70% aperture efficiency; the model's 33 dBi sits in it);
9.7–9.9 GHz; look-up 25–40 nm and look-down 20–30 nm (APG-66(V)1),
29–48 / 24–36 nm ((V)2), no RCS stated; pulse widths 0.81–4 µs ((V)2);
scan width ±10/30/60°, 1, 2 or 4 elevation bars, ±60° gimbal; TWS on up
to 10 targets over "±60°/+25°", AMRAAM datalink. Its 20 kW peak / 200 W
average / 0.5–15 kHz PRF are for the SASS ground and maritime variant
(1 × 3 m antenna), not the fighter set; the 200 W average and 20 kW peak
are used as the family's transmitter estimate —
[REFERENCE §9.8](REFERENCE.md#98-parameter-grounding),
[REFERENCE §9.6](REFERENCE.md#96-frames-and-track-confirmation),
[REFERENCE §9.11](REFERENCE.md#911-track-measurement-and-filtering).

### O'Donnell 2002

**[P]** R. M. O'Donnell, *Introduction to Radar Systems*, MIT Lincoln
Laboratory: *The Radar Equation* (13 June 2002), *MTI and Pulse Doppler
Processing* (19 June 2002), and *Parameter Estimation and Tracking*
(31 July 2008).
<https://www.ll.mit.edu/media/6946> · <https://www.ll.mit.edu/media/6976> ·
<https://www.ll.mit.edu/media/6981> · course page
<https://www.ll.mit.edu/outreach/web-based-course-radar-introduction-radar-systems>

Used for:

- the SNR form of the radar equation (noise `k Ts Bn`, total loss `L`)
  and its dwell form in average power and time on target; the worked
  airport-surveillance-radar example (reproduced to 0.1 dB as a check);
  typical totals of 8–10 dB loss, beam-shape loss 2–4 dB, ~3 dB field
  margin — the plausibility range for the fitted system loss
  ([REFERENCE §9.1](REFERENCE.md#91-the-monostatic-radar-equation));
- slides 32–34 of *MTI and Pulse Doppler*: main-beam clutter is the
  ground lit by the main beam, sidelobe clutter spans ±2V/λ — the Doppler
  notch applies only while the beam touches the ground
  ([REFERENCE §9.3](REFERENCE.md#93-pulse-doppler-clutter-notch));
- *Parameter Estimation and Tracking*, slides 8, 9, 27 and 37: estimation
  error ∝ resolution / √SNR, monopulse angle error ~0.10 beamwidth at
  13 dB ("10:1 beam splitting"), and alpha-beta or Kalman tracking that
  coasts on missed scans — the per-look measurement error and the
  tracker's structure
  ([REFERENCE §9.11](REFERENCE.md#911-track-measurement-and-filtering)).

### Skolnik

**[S]** M. I. Skolnik, *Introduction to Radar Systems*; D. K. Barton,
radar texts — standard forms, recalled, no page checked.

Used for: the monopulse and range accuracy forms σ_θ = θ₃ / (k_m √(2 SNR))
with k_m ≈ 1.6 (which reproduces the Lincoln Laboratory curve, 0.099
beamwidth at 13 dB) and σ_R = c τ / (2 √(2 SNR))
([REFERENCE §9.11](REFERENCE.md#911-track-measurement-and-filtering));
the aperture beamwidth and first-null rules (1.2 λ/D half-power, λ/D and
~1.5 λ/D first null for uniform and tapered apertures), which give the
beamwidth and why the notch's one-beamwidth reach is the strong part of
the lobe (an estimate)
([REFERENCE §9.3](REFERENCE.md#93-pulse-doppler-clutter-notch)). The
1 µs pulse width (inside the APG-66(V)2's 0.81–4 µs) and the 3 g track
maneuver allowance are estimates.

### NAWCWD TP 8347

**[P]** NAWCWD TP 8347, *Electronic Warfare and Radar Systems Engineering
Handbook*, Naval Air Warfare Center Weapons Division (public release),
section *Receiver Sensitivity / Noise*. Read through the RF Cafe mirror
(its noise-figure table is an image and was not read).
<https://www.rfcafe.com/references/electrical/ew-radar-handbook/receiver-sensitivity-noise.htm> ·
<https://everyspec.com/USN/NAVY-General/NAWCWD_TP_8347_4TH-ED_2013_55921/>

Used for: sensitivity `S_min = (S/N)_min · kT₀B · NF`; `kT₀` = −174 dBm/Hz
at T₀ = 290 K; typical sensitivities RWR −65 dBm, pulse radar −94 dBm, CW
missile seeker −138 dBm; S/N of 3–10 dB to detect, 10–20 dB to track.
The RWR sensitivity is taken from here —
[REFERENCE §9.8](REFERENCE.md#98-parameter-grounding),
[REFERENCE §10.1](REFERENCE.md#101-the-friis-one-way-link).

### Wikipedia APG-68

**[S]** Wikipedia, *AN/APG-68*. <https://en.wikipedia.org/wiki/AN/APG-68>

Used for: ~9.86 GHz, which sets the wavelength; "max detection range
around 50 mi (80 km)" citing Deagel, with no RCS —
[REFERENCE §9.8](REFERENCE.md#98-parameter-grounding).

### Phillips 1987

**[P]** R. A. Phillips and C. A. Cassidy, *Situation awareness mode*, US
Patent 4,673,938 (filed 1985, granted 1987), assigned to the US Air Force
(Westinghouse), describing the F-16's AN/APG-66.
<https://patents.google.com/patent/US4673938>

Used for: antenna scan rate 60°/s, turn-around 0.2 s; search phase 1–6 s;
a conventional Kalman tracker needs 2–4 s to lock on; acceptable
fire-control delay 2–4 s at long range, 0.5–1 s at short; detection in 3
of 8 PRFs; TWS on up to 10 targets; STT blinds the aircraft to other
targets. The 60°/s puts a primary source under the 47 ms time on target —
[REFERENCE §9.6](REFERENCE.md#96-frames-and-track-confirmation),
[REFERENCE §9.8](REFERENCE.md#98-parameter-grounding),
[REFERENCE §12.8](REFERENCE.md#128-launch-zone),
[VALIDATION Layer 4](VALIDATION.md#layer-4--radar-and-rwr).

### Navy FNWS

**[P]** US Navy, *Fundamentals of Naval Weapons Systems*, ch. 6
*Track-While-Scan Concepts*. <https://man.fas.org/dod-101/navy/docs/fun/part06.htm>

Used for: one track update per scan; "three or more scans are usually
necessary to establish a valid track"; first detection to fire-control
solution "on the order of seconds" —
[REFERENCE §9.6](REFERENCE.md#96-frames-and-track-confirmation),
[VALIDATION Layer 4](VALIDATION.md#layer-4--radar-and-rwr).

### French 2001

**[S]** S. French, *AN/APG-68(V5) Operations Guide*, 2001 — a Falcon 4 /
eFalcon game manual. <https://e-haf.org/PublicFTP/BIBLIO8HKH/Systems/f4_AN-APG-68-v5.pdf>

Used for: context only — 4.9° beam, 2.2° bar step, 2.5 s per ±60° bar +
0.5 s per bar change, TWS ≤ 3 bars and ±25° with a designated target, STT
drops other contacts and alerts the target's RWR.

### Falcon BMS

**[S]** Falcon BMS forum, scan rate. <https://forum.falcon-bms.com/post/6845>

Used for: 2–2.5 s per 120° bar (~50–60°/s), a simulator community's
estimate and the basis of the estimated time on target, since backed by
Phillips 1987's 60°/s —
[REFERENCE §9.8](REFERENCE.md#98-parameter-grounding).

### FAS RWR

**[S]** FAS, *AN/ALR-56* and *AN/ALR-69*.
<https://man.fas.org/dod-101/sys/ac/equip/an-alr-56.htm> ·
<https://man.fas.org/dod-101/sys/ac/equip/an-alr-69.htm>

Used for: RWR architecture only (the ALR-56M a fast-scanning superhet);
no sensitivity. Consulted, no value.

### X-band LNAs

**[S]** X-band low-noise amplifiers (K-PA KXL1 announcement; academic LNA
papers).
<https://www.everythingrf.com/news/details/22182-k-pa-unveils-x-band-gan-mmic-lna-for-advanced-radar-and-sar-receiver-systems>

Used for: component noise figures of ~1.1–1.5 dB. The receiver's system
noise figure (LNA plus protector and front-end losses) is an estimate
built on these, not a sourced fighter-radar figure —
[REFERENCE §9.8](REFERENCE.md#98-parameter-grounding).

### Radar forum material

**[S]** Forum-grade material, read, no number used: f-16.net (APG-68
display; brochure quote "track up to ten targets ... simultaneous launch
of AMRAAM"; 1–2 s update interval)
<https://www.f-16.net/forum/viewtopic.php?p=429300> ·
<https://www.f-16.net/forum/viewtopic.php?p=480249>; DCS forums (TWS
volumes, contested)
<https://forum.dcs.world/topic/303895-agp-68v5-radar-azimuth-and-elevation-bar-issue/> ·
<https://forum.dcs.world/topic/364428-tws-scan-limits-restore-bug-continued>;
Falcon BMS (soft vs hard lock) <https://forum.falcon-bms.com/post/70162>;
War Thunder forum
<https://forum.warthunder.com/t/f-16-fighting-falcon-history-performance-discussion/916/658>;
RF Essentials (track dwell 0.5–5 Hz, unsourced)
<https://rfessentials.com/rf-knowledge-base/what-is-the-track-while-scan-mode-of-a-phased-array-radar-and-how-does-it-alloca/>;
VTOL VR wiki <https://vtolvr.wiki.gg/wiki/Radar_(MFD_Page)>.

Used for: nothing in the model; consulted only.

### Radar search results

**[S]** Seen in search results only, not read, not used for any value.

- Fluctuation models: MathWorks, *Swerling 3 Target Models*
  <https://www.mathworks.com/help/phased/ug/swerling-3-target-models.html>;
  TU Delft, *Radar Drone Detection: Drone RCS Analysis* (drones, not
  fighters)
  <https://repository.tudelft.nl/record/uuid:b43b0e50-f4e8-483e-bc5b-6e8de4692026>;
  RF Essentials, *What is the Swerling target model…* (quotes ~21 dB and
  13.2 dB inconsistently)
  <https://rfessentials.com/rf-knowledge-base/what-is-the-swerling-target-model-and-how-does-it-affect-the-probability-of-dete/>;
  encyclosearch.org mirror of an encyclopedia entry
  <https://encyclosearch.org/encyclosphere/database/cache/52754355309f973ce9f27d66406cbe92a98eb2b5339b532890fb74898fe5f4f9/article.txt>;
  Lund University EITN90 lecture 6 (link returned 404)
  <https://www.eit.lth.se/fileadmin/eit/courses/eitn90/2019/lectures/lecture6.pdf>;
  DTIC AD0602121 and AD0488613 (downloads returned an HTML page)
  <https://apps.dtic.mil/sti/pdfs/AD0602121.pdf> ·
  <https://apps.dtic.mil/sti/pdfs/AD0488613.pdf>;
  RAND P-3037, *More on Detection of Fluctuating Targets* (catalog entry)
  <https://www.rand.org/pubs/papers/P3037.html>.
- Monopulse: Sherman, *Monopulse Principle and Techniques* (1984); Mosca
  (1969); Blair & Brandt-Pearce (1998); KFUPM monopulse-vs-MUSIC
  <https://pure.kfupm.edu.sa/en/publications/comparison-of-monopulse-and-music-estimator-for-angular-parameter/>;
  IHU semi-active tracking SNR <https://radar.ihu.ac.ir/article_202733.html?lang=en>;
  US11402488B2 <https://patents.google.com/patent/US11402488B2/en>;
  Leida xuebao 2014 <https://doaj.org/article/936a24e794b84d5c9e571620495ae134>;
  Journal of Radars 3(4) <https://radars.ac.cn/en/article/doi/10.3724/SP.J.1300.2014.13077>;
  ISC <https://search.isc.ac/inventory/10/90430.htm>;
  MDPI Electronics 11(11):1760 <https://www.mdpi.com/2079-9292/11/11/1760/html>.
- Scan and track: G. W. Stimson, *Introduction to Airborne Radar*
  <https://archive.org/details/introduction-to-airborne-radar>; IDA/OSD,
  *F-16 APG-66 Fire Control Radar Case Study*, DTIC ADA142075
  <https://apps.dtic.mil/sti/tr/pdf/ADA142075.pdf>; *F-16 Pulse Doppler
  Radar (AN/APG-66) Performance*, 1983 (Scribd)
  <https://www.scribd.com/document/429057021/F-16-Pulse-Doppler-Radar-AN-APG-66-Performance-1983>;
  US 4,540,985 <https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4540985>;
  US 5,361,069 <https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5361069>;
  Wikipedia, *Track while scan* <https://en.wikipedia.org/wiki/Track_while_scan>;
  GlobalSecurity, *AN/APG-66* <https://www.globalsecurity.org/military/systems/aircraft/systems/an-apg-66.htm>;
  secretprojects.co.uk APQ-81 thread (2 s frame)
  <https://www.secretprojects.co.uk/forum/index.php/topic,4266.msg33647.html>.

Used for: nothing in the model; consulted only.

---

## Launch zone

No public source gives the AIM-120's own launch envelope, so Rmax is the
sim's own missile flown out (`tools/drivers/launch_zone.cpp`); what is
taken from outside is only what Rmax means
([REFERENCE §12.8](REFERENCE.md#128-launch-zone)).

### Dantas 2021

**[S]** J. P. A. Dantas, A. N. Costa, D. Geraldo, M. R. O. A. Maximo and
T. Yoneyama, *Weapon Engagement Zone Maximum Launch Range Estimation
Using a Deep Neural Network*, arXiv:2111.04474v2, 2021 (Brazilian Air
Force IEAv / ITA). A preprint, hence [S]. <https://arxiv.org/abs/2111.04474>

Used for: the definition of Rmax, "the maximum distance in which the
missile will hit a non-maneuvering target"; Rmin, the no-escape zone
(+5 G evasion) and LAR / DLZ as other names for the same zone; its grid
(shooter altitude 1 000–45 000 ft, speeds 400–600 kt, target heading,
off-boresight ±60°), which the tool's grid resembles; and its note that
its own missile lofts, as this one does —
[REFERENCE §12.8](REFERENCE.md#128-launch-zone).

### DCS pre-launch cues

**[S]** DCS forum, *Pre-launch cues on AIM-120*, quoting the F-15E manual
(read in search snippets only).
<https://forum.dcs.world/topic/246924-pre-launch-cues-on-aim-120/>

Used for: context for what is not modelled — Rpi (max range with current
steering, target holding speed and heading), Rtr (target turning to tail
at launch), Rmin.

### Launch zone search results

**[S]** Seen in search results only, not read: Veda Inc., *An
Introduction to Launch Acceptability Regions*, report V 0501U/2.510,
12 May 1967, DTIC AD0816286 (fetch refused)
<https://apps.dtic.mil/sti/pdfs/AD0816286.pdf>; Cernat & Chelaru, *A
Complex Mathematical Model for Computation of the Dynamic Launch Zone for
Air-To-Air Homing Missiles*, 2008 (paywalled)
<https://www.argospress.com/articles/2008/a-complex-mathematical-model-for-computation-of-the-dynamic-launch-zone-for-air-to-air-homing-missiles>;
EP 3449203 B1 (Launch Success Zone wording)
<https://data.epo.org/publication-server/rest/v1.0/publication-dates/20200513/patents/EP3449203NWB1/document.html>.

Used for: nothing in the model; consulted only.

---

## Guidance, autopilot and response time

### Palumbo 2010

**[S]** N. F. Palumbo, R. A. Blauwkamp and J. M. Lloyd, *Basic Principles
of Homing Guidance*, Johns Hopkins APL Technical Digest 29(1), 2010.
<https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf>

Used for: the true-PN definition (p. 29: true PN
`a_M · 1_n = N V_c λ̇`, "normal to the LOS", Eqs. 19–20);
the guidance loop as one first-order lag whose time constant is the sum
of seeker, filter and flight-control lags (p. 36, Eqs. 41–42); example
autopilot time constants 0.1–0.5 s (p. 37, Fig. 14), which supports the
single-lag model and the range the 0.35 s estimate sits in, not the
value. The gravity bias, cancelling `g` across the body axis, is first
principles and cites no source. Checked against the PDF —
[REFERENCE §7.1](REFERENCE.md#71-the-guidance-law),
[REFERENCE §7.4](REFERENCE.md#74-guidance-response-time),
[REFERENCE §7.6](REFERENCE.md#76-steering-across-the-body-axis).

### Zarchan

**[S]** P. Zarchan, *Ballistic Missile Defense Guidance and Control
Issues*, Science & Global Security.
<https://scienceandglobalsecurity.org/archive/sgs08zarchan.pdf>

Used for: the lower bound on guidance system time constant, set by
acceleration saturation driven by sensor noise —
[REFERENCE §7.4](REFERENCE.md#74-guidance-response-time).

### Response time consulted

**[S]** Consulted for the missile's response time; none supplies a number
used in the model (the 0.35 s default is an estimate,
[REFERENCE §7.4](REFERENCE.md#74-guidance-response-time)):

- E. L. Fleeman, *Tactical Missile Design* / *Missile Design and System
  Engineering* (AIAA course listing) — names the missile time constant as
  a miss-distance driver; no value reached.
  <https://aiaa.org/courses/missile-design-a-comprehensive-guide-to-propulsion-aerodynamics-weight-flight-performance-guidance-lethality-system-engineering-and-development/>
- Lone Star Analysis, *First Order Conceptual Design for Turbojet
  Missiles and Guided Bombs* (Fleeman method) — method only.
  <https://lone-star.com/wp-content/uploads/2021/07/First-Order-Conceptual-Design-for-Turbojet-Missiles-and-Guided-Bombs.pdf>
- MIT Lincoln Laboratory, *Miss distance analysis for command guided
  missiles* (J. Guid. Control Dyn., 1988) — abstract only.
  <https://www.ll.mit.edu/r-d/publications/miss-distance-analysis-command-guided-missiles>
- US Patents 10,323,907 and 10,914,559 — one reported a maneuver time
  constant reduced from 0.5 to 0.4 s; which was not confirmed, so the
  figure is not cited.
  <https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10323907> ·
  <https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10914559>
- MathWorks, *Design a Guidance System in MATLAB and Simulink* —
  structure only.
  <https://www.mathworks.com/help/simulink/slref/designing-a-guidance-system-in-matlab-and-simulink.html>
- Beijing Institute of Technology, *Analysis of tactical missile
  three-loop lateral acceleration autopilot in the time and frequency
  domain* — abstract only.
  <https://pure.bit.edu.cn/en/publications/analysis-of-tactical-missile-three-loop-lateral-acceleration-auto/fingerprints>

Used for: nothing in the model; consulted only.

---

## Loft and self-destruct

### Wikipedia Missile lofting

**[S]** Wikipedia, *Missile lofting* (citing Shin et al., ICAS 2024;
Berglund & Licata, NATO RTO 2001); with D. Barrie, IISS, as quoted there
(the original was not opened). <https://en.wikipedia.org/wiki/Missile_lofting>

Used for: the concept — loft trades a climb for lower drag and more
energy at the target; the AIM-120D3's range extension "is provided by
trajectory shaping rather than by any solid motor modification" — and
the band the simulated apexes are checked against (air-to-air lofts peak
at 20 000–110 000 ft, most at 80 000–100 000 ft). No public source gives
an AIM-120 loft angle, threshold range or apex, so the schedule is
fitted to this model —
[REFERENCE §6.10](REFERENCE.md#610-loft).

### VRS wiki

**[S]** VRS Support Wiki, *AIM-120 AMRAAM* (simulator manual, origin not
stated). <https://forums.vrsimulations.com/support/index.php/AIM-120_AMRAAM>

Used for: the concept, not a number — the AMRAAM's end of useful flight
defined kinematically: the countdown is "the maximum time of flight of
the missile to reach Raero", its non-maneuvering aerodynamic limit. The
basis for making an energy floor the primary self-destruct trigger and
the timer only a backstop. No primary source gives an AIM-120 maximum
time of flight; the 120 s backstop is an estimate and the 2 G floor
follows from the model's own turn ceiling —
[REFERENCE §12.7](REFERENCE.md#127-self-destruct).

### Game forum figures

**[S]** Consulted, not used for any value:

- War Thunder forum, AIM-120 thread — the game's loft climbs at "20-25
  degrees", and "for the AIM-120 we presume it is around 80" seconds of
  flight, both statements about the game's model.
  <https://forum.warthunder.com/t/the-aim-120-amraam-history-design-performance-discussion/2584/635> ·
  <https://forum.warthunder.com/t/the-aim-120-amraam-history-design-performance-discussion/2584/140>
- DCS World forum threads (*proximity fusing*, *aim 7m battery power
  range*), through search excerpts only — AIM-120C kills ~100 s after
  launch, a game-file 140 s for the AIM-120C-8; nothing verified.
  <https://forum.dcs.world/topic/244411-proximity-fusing/> ·
  <https://forum.dcs.world/topic/242859-aim-7m-battery-power-range>

Used for: nothing in the model; consulted only.

---

## Aerodynamic methods

### Raymer

**[S]** D. P. Raymer, *Aircraft Design: A Conceptual Approach*, AIAA.

Used for: the turbofan TSFC form `(a + b M) sqrt(theta)` that the
fuel-flow fit takes its shape from (the coefficients are fitted to the
flight manual's chart, not Raymer's); §12.5, the transonic drag-rise
construction used between the measured Mach 0.9 and 1.3 points
(drag-divergence Mach at +0.002 over subsonic drag, critical Mach 0.08
below it, drag at Mach 1.05 equal to drag at 1.2) and fairing between
subsonic theory and supersonic data for the lift-curve slope —
[REFERENCE §4.1](REFERENCE.md#41-six-dof-dynamics-and-trim),
[VALIDATION: Fuel flow](VALIDATION.md#fuel-flow),
[VALIDATION: Acceleration, climb, ceiling, and top speed](VALIDATION.md#acceleration-climb-ceiling-and-top-speed).

### Mason configuration aerodynamics

**[S]** W. H. Mason, Virginia Tech, *Configuration Aerodynamics* course
notes. <https://archive.aoe.vt.edu/mason/Mason_f/ConfigAeroDrag.pdf> ·
<https://archive.aoe.vt.edu/mason/Mason_f/SupersonicPres.pdf>

Used for: background on drag build-up and supersonic aerodynamics for
the drag analysis.

---

## Atmosphere

### ITU-R P.676

**[P]** ITU-R P.676-13 (08/2022), *Attenuation by atmospheric gases and
related effects*, Annex 1; and ITU-R P.835-7 (08/2024), *Reference
standard atmospheres*, Annex 1 §1.2.
<https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.676-13-202208-I!!PDF-E.pdf> ·
<https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.835-7-202408-I!!PDF-E.pdf>

Used for: the size of clear-air attenuation at X-band — the specific
attenuation of oxygen and water vapour and the reference water-vapour
profile, 7.5 exp(−h / 2 km) g/m³ —
[REFERENCE §9.9](REFERENCE.md#99-atmospheric-attenuation).
