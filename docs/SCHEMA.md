# bvr_sim — Scenario JSON Schema

Every entity in every scenario is created through this pipeline; nothing
is hardcoded in `src/app/main.cpp`. This document is the authoring reference: each
top-level and per-entity key, its unit, its default, and what it controls.

For *why* a parameter exists and what equation consumes it, see
`REFERENCE.md` (the modeling and simulation guide).

**`src/scenario/ParamRegistry.cpp` is the executable mirror of this
document.** One entry per tunable field, keyed by the JSON Pointer that
addresses it, and it drives all three consumers: the viewer's editor, the
scenario saver, and (by sharing the pointer vocabulary) the batch runner's
overrides. A field documented here but missing from the registry is
invisible in the viewer and silently dropped on save, so **add the
registry entry in the same commit as the schema field.**

Identity and initial-condition fields are deliberately absent from the
registry: `entities[].name`, `type`, and the whole `components.jsbsim`
block. Names are how the saver resolves an entity back to its array
index, and an initial condition has no live value to round-trip against
once the flight model is running.

---

## Unit conventions

The simulator has one internal convention and the JSON deviates from it in
a few specific places. Mismatching them is the easiest way to break a
scenario silently — an entity spawns under the ground plane, a radar never
detects anything.

- **World space** (`transform.pos`, waypoint `points`): feet, **Y-up**, XZ is the horizontal ground
  plane. `pos` arrays are `[x, y, z]` = `[east, up, north]`. Altitude in
  the wrong slot parses fine — it just places the entity on or under the
  ground.
- **Heading / yaw**: degrees, compass convention (0 = north, 90 = east).
- **Meters in JSON, feet internally**: `kill_radius_m`, `max_range_m`,
  `arrival_radius_m`, `static_rcs_m2`, `wing_area_m2`, `reference_area_m2`,
  `arming_range_m`, `falloff_width_m` are converted once at load time
  (`kFeetPerMeter = 3.280839895`).
- **SI throughout**: every radar quantity -- the aircraft type's `radar`
  block, `airborne_radar`, `rwr`, and `seeker.active_seeker` (watts, hertz,
  seconds, linear antenna gain, and SNR / noise figure / loss in dB) --
  stays in SI. The radar equation is computed in meters and watts; only the
  range term is converted from feet.
- **Speed**: knots (`airspeed_kts`, `speeds_kts`).
- **Force and mass**: the read-only `thrust_profile` rows show newtons and
  kilograms (`boost_thrust_n`, `sustain_thrust_n`), converted once to lbf and
  slug internally; a missile type file (`data/missiles/<id>/missile.json`)
  is written directly in lbf and slug (`boost_thrust_lbf`, `dry_mass_slug`).

---

## Top-level fields

| Field | Required | Type | Meaning |
|---|---|---|---|
| `seed` | yes | uint64 | Seeds the simulator's single shared RNG stream. Same seed + same scenario = bit-identical run. A non-negative integer: a negative or fractional seed is a load error rather than a silently different one. |
| `sim_speed` | yes | double | Initial time-scale multiplier, greater than 0. The GUI can change it live. |
| `description` | no | string | What the scenario teaches and what to watch, shown at the top of the GUI's Scenario section. The sim never reads it. |
| `environment` | no | object | Scenario-wide atmospheric conditions. Omit for a standard day. |
| `missile_doctrine` | no | object | Missile and seeker configuration. Omit entirely for scenarios with no missile engagement. |
| `entities` | yes | array | Every aircraft (and any other `Transform`-bearing object). |

A key the loader does not read — in the scenario, an entity, `transform`,
`components`, or any block inside it — is a load error naming the key, not
something silently ignored: `missle_doctrine` or `loopFrom` fails rather than
loading a scenario without the engagement or the loop.

---

## Teaching knobs

![The Parameters section with Advanced mode on](images/parameter-editor.png)

A scenario sets **only** the teaching knobs below (plus each entity's
placement and the structural fields: types, names, initial flight
condition, waypoints). They are the values worth changing to see a
concept in the result; every other number belongs to an aircraft or
missile type file (`data/aircraft`, `data/missiles`). The tables further
down still list those type values, marked **read-only**: the GUI shows
them greyed out, and a scenario key (or batch override) for one is a
load error naming the file it lives in. So is a key that is no parameter
at all, and so is a knob value outside the range the GUI offers for it (a
negative nav gain, a wind speed over 300 kt): the error names the key,
the value and the range.

| Knob | Key | Teaches |
|---|---|---|
| Nav gain | `missile_doctrine.nav_gain` | How hard PN steers; too low lags, too high chases noise |
| Seeker mode | `missile_doctrine.seeker.mode` | Datalink midcourse vs pitbull vs semi-active |
| Gimbal half-cone | `missile_doctrine.seeker.fov_half_angle_deg` | Off-boresight limits; losing the target off the gimbal |
| Memory coast | `missile_doctrine.seeker.memory_coast_time_s` | Riding out a fade or a notch |
| Datalink update interval | `missile_doctrine.datalink.update_interval_s` | Stale midcourse information |
| Max time of flight | `missile_doctrine.self_destruct.max_time_of_flight_s` | Battery / flight-time limit |
| Kill radius | `missile_doctrine.kill_radius_m` | Miss distance vs lethality |
| Max range | `missile_doctrine.max_range_m` | Where guidance gives up |
| Launch range fraction | `missile_doctrine.launch_range_fraction` | Shooting at Rmax vs closer: first shot vs end-game energy |
| Launch mode | `missile_doctrine.launch_mode` | TWS vs STT: a fresher track for the missile vs a lock warning and a blind radar |
| Loft | `missile_doctrine.loft` | Climbing into thin air: longer reach and a faster arrival on a long shot |
| Seeker reference range | `missile_doctrine.seeker.reference_range_nmi` | Acquisition range ~ receiver sensitivity^(1/4) |
| Radar reference range | `entities[i].components.airborne_radar.reference_range_nmi` | The R⁴ law; detect without being heard further |
| Target RCS scale | `entities[i].components.signature.rcs_scale` | Range ~ RCS^(1/4) |
| Temperature offset | `environment.temperature_offset_c` | Air density → thrust, drag, range |
| Wind speed / from | `environment.wind_speed_kts`, `wind_from_deg` | Air-relative vs ground-relative motion |

The GUI shows kill radius, max range, gimbal half-cone, memory coast,
seeker reference range and target RCS scale only in its Advanced mode
(the GUI's Settings popup); the rest, and placement, are always shown.
Every lesson's suggested change is one of those. All are editable the same
way everywhere else.


---

## `environment`

Conditions shared by everything that flies. Every field is optional and
defaults to a standard day. They apply to the aircraft (JSBSim's own
atmosphere, before trim) and to everything else that reads the atmosphere —
missile drag, the missile and aircraft G envelopes — so the two always fly
the same air.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `temperature_offset_c` | no | °C | `0.0` | ISA deviation: every layer of the standard atmosphere runs this much hotter (positive) or colder. Sea-level pressure stays standard, so hot air is thinner near the ground and denser high up. Live-editable. |
| `wind_speed_kts` | no | kt | `0.0` | One steady wind, the same at every altitude. Aircraft and missile both fly relative to the moving air (an aircraft's `airspeed_kts` is through the air, so its ground speed is airspeed plus wind), and waypoint steering crabs into the crosswind to hold its track. Live-editable. |
| `wind_from_deg` | no | deg | `0.0` | Compass direction the wind blows **from**: 270 is a westerly, moving air eastward. Live-editable. |

```json
"environment": { "temperature_offset_c": 15, "wind_speed_kts": 40, "wind_from_deg": 270 }
```

---

## `missile_doctrine`

Present only if the scenario includes a missile engagement between a
`"Pursuer"`-named and `"Target"`-named entity. Those two names are the
only ones with special meaning — `LaunchSystem` looks them up by name.

The doctrine names the **missile type** it fires (see *Missile types*
below). Only the teaching knobs above may be set here; every row marked
**read-only** is the type's value, shown in the viewer and a load error
as a scenario key.

### Core

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `missile` | yes | string | — | Missile type: a directory under `data/missiles/` holding a `missile.json`. Every current scenario uses `"aim120"`. |
| `kill_radius_m` | yes | m | — | Distance from the *nearest point on the target's airframe* (an oriented bounding box, not the center point) at which lethality is evaluated. |
| `max_range_m` | yes | m | — | Range beyond which guidance gives up: `MISS` event, transition to unpowered ballistic coast. |
| `launch_range_fraction` | no | — | `1.0` | Teaching knob: fire once the launcher's confirmed radar track puts the target inside this fraction of the missile's maximum launch range (its launch zone, at the estimated geometry); 1 = at Rmax. The launch then waits the radar's fire-control delay. REFERENCE.md 12.8. |
| `launch_mode` | no | — | `"TWS"` | Teaching knob: `"TWS"` fires on the scanning radar's confirmed track; `"STT"` locks the target in single-target track when the launch gate opens and fires from the lock, held until the missile no longer needs it. A SARH missile always uses STT. REFERENCE.md 9.6, 12.8. |
| `loft` | no | — | `true` | Teaching knob: a long MIDCOURSE_ARH shot climbs on the type's loft schedule, then pushes over onto the target; `false` flies it straight. REFERENCE.md 6.10. |
| `nav_gain` | yes | — | — | Proportional-navigation gain `N`. Typical range 3–6. A flat `missile_doctrine` field like the rest of this section, but the viewer gives it its own **Missile / Proportional Navigation** heading — it is the one value here that changes how the missile flies rather than what it is made of. |
| `arming_range_m` | read-only | m | type (`50.0`) | Minimum distance the missile must fly from its own launch point before a kill-radius pass can register. |
| `falloff_width_m` | read-only | m | type (`5.0`, `fuze.falloff_width_m`) | Width parameter of the Pk-vs-miss-distance sigmoid around `kill_radius_m`. Larger = more gradual. |
| `reference_area_m2` | read-only | m² | type (`0.0249`, 0.584 ft diameter body) | Drag reference (cross-sectional) area. Distinct from radar cross-section. |

### `datalink`

The launcher-to-missile datalink carrying the target's position during
midcourse. Optional. The missile steers to the launcher's radar track
(its error comes from the radar — see `airborne_radar` and REFERENCE.md,
Track measurement and filtering) until its own seeker acquires. See
REFERENCE.md, Datalink target state.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `update_interval_s` | no | s | `1.0` | How often the launcher sends its radar's current track. Between updates the missile extrapolates on the last reported velocity. |

Updates stop entirely when the launcher's radar track stops being a
confirmed track, recorded as a `DATALINK_LOST` event; the missile then flies an
increasingly stale estimate.

### `autopilot`

The missile always flies true PN with the gravity bias on. The largest angle
of attack (`alpha_max_deg`) belongs to the missile type's normal-force model
(REFERENCE.md § Missile normal force and turn ceiling), not to the scenario.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `launch_turn_exit_deg` | read-only | deg | type's (15) | After launch the autopilot turns toward the line of sight at full available G until the target is within this angle of the velocity vector, then the guidance law takes over for the rest of the flight. `0`: no launch turn. See REFERENCE.md § Launch turn. |

### `self_destruct`

Shown in the viewer under **Missile / Self-destruct**. See REFERENCE.md
§12.7.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `min_available_g` | read-only | G | type (`2.0`) | After motor burnout, the missile self-destructs once the G it can still pull falls below this. `0` disables. |
| `max_time_of_flight_s` | no | s | type (`120.0`) | The missile self-destructs this long after launch, guided or coasting. `0` disables. |

### `thrust_profile`

Read-only: the missile type's motor.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `boost_duration_s` | read-only | s | type (`3.0`) | Boost stage duration. |
| `boost_thrust_n` | read-only | N | type (`25221`, 5670 lbf) | Boost stage axial thrust. |
| `sustain_duration_s` | read-only | s | type (`5.0`) | Sustain stage duration. |
| `sustain_thrust_n` | read-only | N | type (`14100`, 3170 lbf) | Sustain stage axial thrust. |
| `dry_mass_kg` | read-only | kg | type (`91.9`, 6.294 slug) | Missile mass with the motor burnt out. |
| `propellant_mass_kg` | read-only | kg | type (`60.1`, 4.121 slug) | Propellant load, burnt off linearly across boost + sustain. |

The mass fields belong to this block rather than a separate one because
they are not independent of it. The four thrust fields set **total
impulse**; divided by the propellant *weight*, that gives the motor's
**specific impulse**, which for a solid HTPB propellant cannot exceed
roughly 245 s. Raise the thrust without raising the propellant and the
scenario describes a motor that cannot be built.

The AIM-120 type's values sum to its published 152 kg launch mass and reproduce
its published motor load, giving 247.8 s. If you change one of these six
numbers, check the other five. See
[VALIDATION.md](VALIDATION.md) — this exact
constraint caught a real error in the shipped defaults.

### `seeker`

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `mode` | no | enum | type (`"MIDCOURSE_ARH"`) | `"ARH"` \| `"SARH"` \| `"MIDCOURSE_ARH"`. See REFERENCE.md § Seeker Modeling. |
| `fov_half_angle_deg` | no | deg | type (`45.0`) | Seeker gimbal cone half-angle around the missile's velocity vector. |
| `memory_coast_time_s` | no | s | type (`1.5`) | How long the seeker coasts on its last LOS rate after losing lock before going ballistic (or, for an active `MIDCOURSE_ARH` seeker whose launcher still tracks the target, handing back to datalink midcourse). |
| `reference_range_nmi` | no | nmi | type fit (`10`) | Teaching knob: the range at which one active-seeker look at a 5 m² Swerling 1 target is a coin flip. The receiver gains 40·log10(this / the type's fitted range) dB, so every acquisition range scales by the same ratio (REFERENCE.md 9.8). |

#### `seeker.active_seeker`

Read-only: the missile type's receiver budget (the
`reference_range_nmi` knob above scales it).

The missile's own active radar seeker. Consulted only by
`MIDCOURSE_ARH`. A sweep detects when its square-law detector output
(REFERENCE.md 9.10) reaches `detection_threshold_db`. The AIM-120 type's
`system_loss_db` is fitted so one look at a Swerling 1 5 m² target is a
coin flip at 10 nmi, in free space.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `average_power_w` | read-only | W | type (`100.0`) | Seeker average transmitted power. |
| `antenna_gain` | read-only | linear | type (`148.0`) | Seeker antenna gain (~21.7 dBi). |
| `frequency_hz` | read-only | Hz | type (`1.0e10`) | X-band. |
| `noise_figure_db` | read-only | dB | type (`4.0`) | Receiver noise figure. |
| `integration_time_s` | read-only | s | type (`0.01`) | Coherent integration per sweep. |
| `system_loss_db` | read-only | dB | type (fitted) | Every other loss (the air's absorption included; REFERENCE.md 9.9). |
| `detection_threshold_db` | read-only | dB | type (`11.40`, ln(1/pfa)) | Detector output over the noise a sweep needs to count as a detection. |
| `sweep_period_s` | read-only | s | type (`0.1`) | Seeker sampling interval. |

### Missile types (`data/missiles/<id>/missile.json`)

Not part of a scenario: one file per missile type, shared by every
scenario that names it. Adding a missile is adding a directory here.
Every field is required; a missing or out-of-range one fails the
scenario load naming the file, and so does a key the loader does not
read (a misspelling never loads as a different missile), `notes`
excepted. English units, except the fuze's arming range (metres, like
the doctrine field it backs).

| Field | Unit | Meaning |
|---|---|---|
| `name` | — | Display name. |
| `propulsion.boost_duration_s`, `boost_thrust_lbf`, `sustain_duration_s`, `sustain_thrust_lbf` | s, lbf | The three-stage motor. |
| `mass.dry_mass_slug`, `propellant_mass_slug` | slug | Burnt-out mass and propellant load. |
| `aerodynamics.reference_area_ft2` | ft² | Reference area for drag and the normal force. |
| `aerodynamics.cd_vs_mach` | — | Zero-lift drag table with the motor off (coasting), `[mach, cd]` rows in strictly increasing Mach. Held at the end values outside it. |
| `aerodynamics.normal_force.alpha_max_deg` | deg | Largest flown angle of attack, in (0, 90]. `cn_max_vs_mach` is the normal force at this angle. |
| `aerodynamics.normal_force.cn_alpha_per_deg_vs_mach` | per deg | The lift slope `a`: `[mach, value]` rows, strictly increasing Mach, positive. CN = a α + b α² (REFERENCE.md 6.5). |
| `aerodynamics.normal_force.cn_max_vs_mach` | — | CN at `alpha_max_deg`, which pins `b` and sets the turn ceiling: `[mach, value]` rows; must be at least slope × alpha_max at every Mach. For the AIM-120 both tables are computed by `tools/fit/fit_normal_force.py`. |
| `aerodynamics.structural_g_limit` | G | Structural cap on the turn ceiling. |
| `seeker.mode` | enum | `"ARH"` \| `"SARH"` \| `"MIDCOURSE_ARH"`. |
| `seeker.fov_half_angle_deg`, `memory_coast_time_s` | deg, s | Gimbal cone, memory coast. |
| `seeker.active_seeker.average_power_w`, `antenna_gain`, `frequency_hz`, `noise_figure_db`, `integration_time_s`, `system_loss_db` | W, —, Hz, dB, s, dB | The active seeker's receiver budget. |
| `seeker.active_seeker.pfa` | — | False-alarm probability per look; sets the detection threshold, ln(1/pfa). Type only. |
| `seeker.active_seeker.sweep_period_s` | s | Seeker sampling interval. |
| `autopilot.response_time_s` | s | The guidance response time: one first-order lag between demanded and delivered lateral acceleration, for the seeker, fins and airframe together; not negative, 0 = instant. Type only. |
| `autopilot.launch_turn_exit_deg` | deg | Off-boresight angle at which the post-launch max-G turn hands over to the guidance law, in [0, 180); 0 = no launch turn. |
| `loft.start_range_nmi`, `full_range_nmi`, `max_climb_deg` | nmi, nmi, deg | The loft schedule: no climb at or below `start_range_nmi` launch range, rising linearly to `max_climb_deg` at `full_range_nmi`, held beyond. `0 <= start < full`, angle in [0, 90); 0 = never lofts. Type only (REFERENCE §6.10). |
| `loft.climb_time_constant_s`, `min_available_g` | s, G | How fast the flight-path angle follows the climb command (positive), and the turn ceiling under which a burnt-out missile ends its climb (not negative). Type only. |
| `seeker.active_seeker.confirm_hits`, `confirm_window` | sweeps | M-of-N acquisition: this many detections in the last `confirm_window` sweeps. Whole numbers, window 1–31, hits 1 to the window. Type only; no doctrine override. |
| `seeker.active_seeker.antenna_diameter_m`, `beamwidth_factor` | m, — | The seeker's Doppler-notch beam, `beamwidth_factor` × λ / `antenna_diameter_m` either side of the line of sight (REFERENCE §8.3). Positive. Type only. |
| `seeker.active_seeker.notch_velocity_threshold_mps` | m/s | The seeker's Doppler notch: a target whose ground-relative radial speed is under this, with ground in the seeker's beam, is not seen. 0 = no notch. Type only. |
| `fuze.arming_range_m` | m | Distance from launch before a kill can register. |
| `self_destruct.min_available_g` | G | Available-G floor, checked after burnout. Must be above 0 and below `structural_g_limit`. |
| `self_destruct.max_time_of_flight_s` | s | Time-of-flight backstop. Must be longer than boost + sustain. |
| `notes` | — | Free text: where each value comes from. Not read. |

**`fit/launch_zone.json`** (beside `missile.json`): the type's maximum
launch range, generated by `tools/drivers/launch_zone.cpp` (`launch_zone --missile
<id>`). Do not hand-edit. Optional while the type is being built (the
tool loads the type before the table exists); a table whose
`missile_file_hash` does not match the current `missile.json` still loads,
with a warning to rerun the tool.

| Field | Unit | Meaning |
|---|---|---|
| `missile_file_hash` | — | FNV-1a 64 of the `missile.json` it was generated from, carriage returns ignored. |
| `axes.shooter_altitude_ft`, `shooter_mach`, `target_mach`, `aspect_deg`, `off_boresight_deg` | ft, —, —, deg, deg | The grid, each strictly increasing. Aspect is where the shooter sits as seen from the target's tail, in [-180, 180] (0 = tail chase, ±180 = head-on); off-boresight is the target's angle off the shooter's nose, in [0, 180] (the other side is the mirror image). |
| `rmax_nmi` | nmi | One Rmax per grid cell, last axis fastest. Read multilinearly, held at the grid's edges. |
| `definition`, `conditions`, `generated_by`, `missile` | — | Provenance: what Rmax means and what the launches held fixed. Not read. |

---

## `entities[]`

Each entry spawns one entity with `Transform` + `Tag`, plus whichever of
`components` it declares.

| Field | Required | Unit | Meaning |
|---|---|---|---|
| `name` | yes | — | Tag shown everywhere in the UI. Unique within the scenario: a repeated name is a load error, because everything that finds an entity by name takes the first match. `"Pursuer"` / `"Target"` are the two names `LaunchSystem` looks up to run a missile engagement. |
| `type` | yes | — | Free-text (`"aircraft"` in every current scenario), required but not stored or used to gate behavior — `components.jsbsim`'s presence is what determines whether an entity flies. |
| `transform.pos` | yes | ft `[x,y,z]` | Initial position. |
| `transform.heading` | yes | deg | Initial compass heading (0 = N, 90 = E). |
| `components` | yes | object | Open bag of optional component blocks. Any combination is valid. |

### `components.jsbsim`

Attaches a full JSBSim-flown aircraft (`JSBSimComponent`,
`AircraftComponent`, `AutopilotComponent` and its type's
`GLimitComponent`). Without this block the entity is static or
kinematic-only — it still renders and is still detectable by radar if it
has an RCS, but nothing flies it.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `aircraft` | yes | string | — | Aircraft type: a directory under `data/aircraft/` holding an `aircraft.json` (see *Aircraft types* below). Every current scenario uses `"f16c_block50"`. |
| `altitude_ft` | yes | ft | — | Initial altitude, and the autopilot's initial `targetAltitudeFt`. Must equal `transform.pos[1]`: they are one start condition written twice, and a file where they differ is a load error. A batch override of either key is carried to the other. |
| `heading_deg` | yes | deg | — | Initial JSBSim heading IC, and the initial `targetHeadingDeg`. Must equal `transform.heading` (compared around the circle, so 360 equals 0); a batch override of either key is carried to the other. |
| `airspeed_kts` | yes | kts | — | Initial **true** airspeed IC, and the initial `targetSpeedKts`. |

An aircraft spawns with its type's fuel load, the flight model file's own (F-16: 3 000 lb of 6 972 lb
internal; REFERENCE.md 4).

`model` (a JSBSim model name) is not accepted: the aircraft type, named by
`aircraft`, owns the flight model, so two aircraft of one type cannot
disagree about it. A scenario naming `model` fails to load with a message
saying so.

Loading this block runs the aircraft initialization sequence: load the
type's flight model, set initial conditions, spin the engine up,
`RunIC()`, then a full `DoTrim`. Load fails loudly (throws) if the type or
its model can't be found or trim can't be solved.

### Flight envelope (read-only)

Every aircraft flies its type's V-n envelope (`envelope` in the type file),
and none of its numbers is a teaching knob. The viewer shows the envelope
read-only; the fields are:

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `clmax` | read-only | — | type's | Maximum lift coefficient, which sets the aerodynamic G limit. The F-16's is 1.65: its lift at the ~25° AoA limit, from the NASA TP-1538 wind-tunnel tables (REFERENCE.md §5.1). This is the low-speed value; when the aircraft's model defines `aero/function/kCLmax-mach` (the F-16 does), the envelope scales it by that live Mach factor. |
| `cl_max_loss_per_g` | read-only | — | type's | Lift coefficient the flight control system gives up per G above 1 (its AoA limit falls with load factor, the G/AoA blend). Lift available at `n` G is `CL_max − cl_max_loss_per_g·(n − 1)`. The F-16's is 0.049 (REFERENCE.md §5.1). |
| `wing_area_m2` | read-only | m² | type's | Wing reference area. The weight is not a field: the envelope reads JSBSim's live `inertia/weight-lbs`, so available G rises as fuel burns off. |
| `structural_g_limit_pos` | read-only | G | type's | Positive structural cap. |
| `structural_g_limit_neg` | read-only | G | type's | Negative structural cap. Itself negative, e.g. `-3.0`. |
| `damage_accumulation_rate` | read-only | damage/g·s | type's | How fast cumulative structural damage accrues during overstress; 1.0 is structural failure. |

On save, a field is written only where the live value differs from the
type's, so a scenario stays free of copies of type data.

### Aircraft types (`data/aircraft/<id>/aircraft.json`)

Not part of a scenario: one file per aircraft type, shared by every
scenario that names it. Adding an aircraft is adding a directory here
(plus its JSBSim model under `data/jsbsim_data/`). Every field is
required, except the optional `radar`, `rwr` and `signature` blocks; a
missing or out-of-range one fails the scenario load naming the file, and
so does a key the loader does not read (a misspelled `rwr` never loads
as an aircraft with no warning receiver), `notes` excepted.

| Field | Unit | Meaning |
|---|---|---|
| `name` | — | Display name. |
| `jsbsim_model` | — | JSBSim model: `data/jsbsim_data/aircraft/<model>/<model>.xml`. |
| `geometry.wing_area_ft2` | ft² | Wing reference area for the envelope; should equal the model's `metrics/Sw-sqft`. |
| `geometry.hit_box_ft` | ft | `span`, `height`, `length` of the bounding box a missile's miss distance is measured against. |
| `envelope.cl_max` | — | Lift coefficient at the AoA limit. |
| `envelope.cl_max_loss_per_g` | — | Lift the FCS gives up per G above 1 (G/AoA blend); 0 or more. |
| `envelope.structural_g_limit_pos` / `_neg` | G | Structural limits (positive above 1, negative below 0). |
| `envelope.damage_accumulation_rate` | damage/g·s | Overstress damage rate, as for `g_limit`. |
| `flight_controls.g_per_unit_stick` | G | Load factor the FCS commands per unit of aft stick, above 1 G. |
| `flight_controls.military_throttle_cmd` / `max_throttle_cmd` | — | Throttle commands for military power and for the most thrust the engine gives. |
| `fuel.internal_tank_prefix` | — | Tanks whose JSBSim name starts with this are internal. |
| `radar` | — | Optional: the type's fire-control radar (below). An aircraft carries it only if its scenario entry has an `airborne_radar` block. |
| `radar.frame_time_s` | s | One search / TWS frame: every target in the cone gets one look per frame. |
| `radar.stt_frame_time_s` | s | One single-target-track frame: while locked, one look at the locked target per frame and none at anything else. Positive, at most `frame_time_s`. |
| `radar.confirm_frames`, `confirm_window_frames`, `drop_frames` | frames | Track confirmation at `confirm_frames` of the last `confirm_window_frames` (at most 15); a confirmed track drops after `drop_frames` in a row without a detection. |
| `radar.stt_drop_frames` | frames | A lock breaks after this many STT frames in a row without a detection (whole, at least 1). |
| `radar.fire_control_delay_s` | s | How long the launch gate must stay open before the missile fires. |
| `radar.average_power_w`, `peak_power_w` | W | Average power (what the integrated return scales with) and peak power (what a radar warning receiver hears). Peak at least average. |
| `radar.antenna_gain`, `frequency_hz` | —, Hz | Linear antenna gain; operating frequency. |
| `radar.noise_figure_db`, `time_on_target_s`, `system_loss_db` | dB, s, dB | Receiver noise figure; coherent time on target per sweep; every other loss (fitted to the cited detection range). |
| `radar.pfa` | — | False-alarm probability per look; sets the search threshold, ln(1/pfa) over the noise. |
| `radar.fov_half_angle_deg`, `notch_velocity_threshold_mps`, `min_range_m` | deg, m/s, m | FOV cone, Doppler notch and blind range. |
| `radar.antenna_width_m`, `beamwidth_factor`, `monopulse_slope`, `pulse_width_s`, `track_maneuver_g` | m, —, —, s, g | Measurement and tracking: half-power beamwidth `beamwidth_factor × λ / aperture` per axis; angle error per look `beamwidth / (monopulse_slope √(2 SNR))`; range error `c × pulse_width_s / (2 √(2 SNR))`; the Kalman tracker's 1-σ target acceleration. Positive (maneuver ≥ 0). REFERENCE.md 9.11. |
| `radar.antenna_height_m` | m | Aperture height: with `beamwidth_factor` and the wavelength, the elevation beamwidth; the notch applies only while the beam, one beamwidth either side of the line of sight, touches the ground (REFERENCE.md 9.3). Positive. |
| `rwr` | — | Optional: the type's radar warning receiver. An aircraft carries it only if its scenario entry has an `evasion.rwr` block. |
| `rwr.sensitivity_dbm`, `antenna_gain` | dBm, — | Weakest pulse it registers; linear antenna gain (positive). |
| `signature` | — | Optional: the type's radar signature. Every aircraft of a type with one is radar-visible; a type without one is invisible. |
| `signature.fluctuation_model` | enum | `"swerling_1"` \| `"steady"`, how the return fluctuates look to look (REFERENCE.md 9.10). |
| `signature.rcs_shape` | list | 1-8 body-axis ellipsoids `{"part", "semi_axes_ft": {"span", "height", "length"}}` (all positive): the mean RCS seen from a direction is their projected area that way, an ESTIMATE from geometry generated by `tools/fit/rcs_shape.py` (REFERENCE.md 9.2). Its nose-on area is the type's static RCS; `static_rcs_m2` in a type file is a load error. |
| `notes` | — | Free text: where each value comes from. Not read. |

### `components.velocity`

```json
"velocity": { "linear": [0.0, 0.0, 150.0] }
```

ft/s. Attaches a plain `Velocity` component integrated by simple Euler.
Used by component-test scenarios for a straight-line target with no
JSBSim involved. Note that a pulse-Doppler radar cannot see a target with
no ground-relative motion, so a radar-test target needs a real velocity
here.

### `components.radar_cross_section`

Only for an entity with no aircraft type (a kinematic test target): it is
what makes the entity detectable, and both fields are required. On an
aircraft (an entity with a `jsbsim` block) it is a load error: an
aircraft's signature is its type's `signature` alone.

| Field | Required | Unit | Meaning |
|---|---|---|---|
| `static_rcs_m2` | yes | m² | Mean radar cross-section, the same from every direction (no shape). |
| `fluctuation_model` | yes | enum | `"swerling_1"` (exponential return, fresh each look) or `"steady"`. |

### `components.signature`

On an aircraft: the one teaching knob on its radar signature. The shape
and fluctuation are the type's. An entity with no `jsbsim` block may carry
it too, scaling its own `radar_cross_section`; a `signature` on an entity
with neither is a load error.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `rcs_scale` | no | × | `1` | Multiplies the RCS every radar and seeker sees, after aspect. Detection range goes as its fourth root: ×16 is seen twice as far. The RWR is unaffected (it hears the emitter, not the reflection). |

### `components.airborne_radar`

Switches on the aircraft type's fire-control radar (the type file's
`radar` block) for this aircraft: `"airborne_radar": {}` flies the
type's radar as it is. The only key it takes is the
`reference_range_nmi` teaching knob; every other row is the type's
value, read-only. A sweep detects at a tier when its square-law detector output
(REFERENCE.md 9.10) reaches the one search threshold. Every target in
the cone gets one look per frame; a track confirms at 3 of 5 frames and
drops after 3 frames without a detection (REFERENCE.md 9.6). A confirmed
track inside the launch zone is what triggers a launch (REFERENCE.md
12.8) -- a Pursuer with no `airborne_radar` block never launches, at any
range. Needs a `jsbsim` block (the radar is the aircraft type's).

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `reference_range_nmi` | no | nmi | type fit (`50`) | Teaching knob: the range at which one look at a 5 m² Swerling 1 target is a coin flip. The receiver gains 40·log10(this / the type's fitted range) dB, so search, track and firing ranges all scale by the same ratio; the transmitted peak power an RWR hears is unchanged (REFERENCE.md 9.8). |
| `frame_time_s` | read-only | s | type (`4.4`) | One search / track-while-scan frame: every target in the cone gets one look per frame. |
| `stt_frame_time_s` | read-only | s | type (`0.1`) | One single-target-track frame: while locked, the radar looks at its one target this often and at nothing else. |
| `fire_control_delay_s` | read-only | s | type (`3.0`) | How long the launch gate must stay open before the missile fires. |
| `average_power_w` | read-only | W | type | Average transmitted power. |
| `peak_power_w` | read-only | W | type | Peak pulse power, what a radar warning receiver hears. |
| `antenna_gain` | read-only | linear | type | Antenna gain. |
| `frequency_hz` | read-only | Hz | type | Operating frequency. |
| `noise_figure_db` | read-only | dB | type | Receiver noise figure. |
| `time_on_target_s` | read-only | s | type | Coherent time on target per sweep. |
| `system_loss_db` | read-only | dB | type | Every other loss (the air's absorption included; REFERENCE.md 9.9). |
| `fov_half_angle_deg` | read-only | deg | type | Symmetric cone half-angle around the nose. Outside it is an automatic miss with no radar-equation call and no RNG draw. |
| `search_threshold_db` | read-only | dB | type (ln(1/pfa)) | The one detection threshold over the noise. |
| `notch_velocity_threshold_mps` | read-only | m/s | type | Pulse-Doppler clutter notch: a target whose ground-relative radial velocity is below this is filtered as clutter regardless of SNR — while the beam touches the ground. |
| `antenna_height_m` | read-only | m | type | Aperture height; with the wavelength it sets the beam's vertical size. |
| `antenna_width_m` | read-only | m | type | Aperture width; with the wavelength it sets the azimuth beamwidth. |
| `beamwidth_factor` | read-only | — | type | Half-power beamwidth in wavelengths per aperture size (~1.2 tapered). |
| `monopulse_slope` | read-only | — | type | Angle error per look = beamwidth / (this × √(2·SNR)). |
| `pulse_width_s` | read-only | s | type | Range error per look = c × this / (2 √(2·SNR)). |
| `track_maneuver_g` | read-only | g | type | The tracker's 1-σ allowance for target acceleration. |
| `min_range_m` | read-only | m | type | Blind range: closer than this is a forced miss regardless of SNR. |

### `components.waypoints`

General-purpose route queue, attachable to any entity that also has an
`AutopilotComponent` (i.e. a `jsbsim` block). Can also be built and edited
live in the GUI. A route can be any length; the 3D view draws the next 32
points of it.

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `points` | yes | ft `[x,y,z]`[] | — | Route points, in order. `queue.front()` is always the current one. |
| `speeds_kts` | no | (number\|null)[] | — | Parallel to `points`. Index *i* overrides target speed when that waypoint becomes current; `null` or a missing trailing entry means "leave speed alone". May be shorter than `points`. |
| `arrival_radius_m` | no | m | `150.0` | Distance at which a waypoint counts as reached and the queue advances. Greater than 0: a radius that can never be met is a load error. |
| `loop` | no | bool | `false` | If true, a reached waypoint is re-appended to the back of the queue instead of discarded. |
| `loop_from` | no | index | `0` | Needs `loop: true`. Points before this index are a lead-in flown once; the route then repeats from this point to the end (`[A,B,C,D]`, `loop_from: 1` flies A B C D B C D ...). `0` loops the whole route. Must index one of `points`. |

### `components.evasion`

Attaches the aircraft type's radar warning receiver (the type file's
`rwr` block) as an `RwrComponent`, and a default empty `WaypointComponent`
if the entity doesn't already have one. Needs a `jsbsim` block. This is
purely a sensor attachment — it attaches no autonomous decision-making.
What reacts to RWR illumination, if anything, is whatever externally
drives the entity's waypoint queue. The receiver is the type's alone:
`rwr` takes no keys (each row below is read-only).

```json
"evasion": { "rwr": {} }
```

| Field | Required | Unit | Default | Meaning |
|---|---|---|---|---|
| `rwr.sensitivity_dbm` | read-only | dBm | type | Weakest received pulse that registers as "illuminated at all", at the antenna port. |
| `rwr.antenna_gain` | read-only | linear | type | Receive antenna gain `Gr` in the Friis link budget (which carries the illuminator's peak power). |
| `rwr.signal_jitter_db` | read-only | dB | type (`0`) | Standard deviation of Gaussian jitter on the received-power calculation each sweep. |
| `rwr.sweep_period_s` | read-only | s | type (`0.5`) | RWR sampling interval. |

---

## Scenario directory layout

| Directory | Contents |
|---|---|
| `scenarios/1_core_*.json` | Teaching scenarios, one concept each. |
| `scenarios/2_combined_*.json` | Teaching scenarios where two or three concepts interact. |
| `scenarios/3_showcase_*.json` | Longer engagements with routes, climbs and turns, showing the sim's breadth. |
| `scenarios/custom/` | User-authored scenarios. `_template.json` is the starting point for **New Scenario** in the GUI, and **Save As** writes here. |
| `tests/fixtures/scenarios/` | Frozen scenarios the validation tests load, plus `scenario_timing_5min`, the timing report's benchmark. Not teaching scenarios: their geometry is baked into test assertions or timed measurements. |

**Save As** re-reads the currently-loaded scenario's source JSON from
disk and writes a copy with the session's edits applied:

- every tunable field — the set the GUI's Parameters panel shows, one
  per `ParamRegistry` entry — whose live value differs from what the file
  would load. Placement is saved from each entity's spawn pose, so a save
  mid-run writes the start line, not wherever the aircraft has flown to;
- each entity's full waypoint route, including points already passed,
  in authored order. An entity whose only route is the empty one
  `evasion` attaches gets no `waypoints` block. After a route is edited in
  the GUI the saved route is the queue as drawn; an unedited looping route
  that has already rotated is saved in its authored order.

Every other value is copied through unchanged, including fields no live
component stores (the `aircraft` type name, entity `type`, the `jsbsim`
initial conditions), which could not be reconstructed from world state.
The saver is a patcher, not a general-purpose serializer. The file is
re-serialised, so keys are written in alphabetical order with a 4-space
indent: the values are the source's, but a hand-ordered file will show a
layout diff. A save that cannot be completed (a live value that is not a
finite number, a write that fails) reports why and does not claim success.

---

## Tick recording

The GUI's **Record** button (time panel) and `bvr_batch --record`
write the same CSV: the state that changes during a run, sampled after
every tick (the batch's `--record-decimation N` keeps every Nth). Static
per-type data is not repeated; it is in the type files. The GUI writes to
`recordings/<scenario>_<date>_<time>.csv` beside the executable, and a
Reset or Reload starts a new file. The batch writes `<dir>/run_<id>.csv`.

The file is long format: one row per entity with a transform, then one row
per radar track, per sample. `kind` says which a row is, and a cell that
does not apply to a row's kind is empty. Booleans are `0`/`1`. Positions
are in the world frame (x east, y up, z north), in feet.

| Column | Rows | Unit | Meaning |
|---|---|---|---|
| `sim_time_s` | all | s | Sim time the state refers to. |
| `kind` | all | | `aircraft`, `missile`, `track` (or `other` for a bare velocity entity). |
| `entity`, `name` | all | | The entity (for a track, the radar's owner) and its tag. |
| `target` | missile, track | | The missile's target; the tracked entity. |
| `destroyed` | entity | | Marked destroyed (a hit, structural failure or the ground). |
| `x_ft`, `y_ft`, `z_ft` | entity | ft | Position. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | entity | deg | Attitude. |
| `vx_fps`, `vy_fps`, `vz_fps`, `speed_fps` | entity | ft/s | Velocity over the ground and its magnitude. |
| `mach` | aircraft, missile | | Mach on the velocity through the air. |
| `kcas`, `alpha_deg`, `beta_deg`, `qbar_psf` | aircraft | kt, deg, deg, lbf/ft² | JSBSim calibrated airspeed, angle of attack, sideslip, dynamic pressure. |
| `nz_g`, `nx_g`, `ny_g` | aircraft | g | Body load factors (JSBSim `accelerations/N*`). |
| `p_deg_s`, `q_deg_s` | aircraft | deg/s | Roll and pitch rate. |
| `weight_lbf`, `fuel_lbf` | aircraft | lbf | Gross weight and fuel on board. |
| `thrust_lbf` | aircraft, missile | lbf | Engine thrust; the missile motor's thrust for its phase (boost, sustain, 0 after burnout). |
| `aileron_cmd`, `elevator_cmd`, `throttle_cmd` | aircraft | normalised | The autopilot's stick and throttle. |
| `ap_heading_deg`, `ap_altitude_ft`, `ap_speed_kts` | aircraft | | The autopilot's live setpoints. |
| `overstress_damage` | aircraft | | Accumulated structural damage (failure at 1). |
| `waypoints_left` | entity with a route | | Points still queued. |
| `radar_stt_target` | aircraft | | The entity the radar holds in single-target track; empty while scanning. |
| `rwr_mode`, `rwr_illuminated`, `rwr_missile_warning` | aircraft | | RWR perceived mode (`NONE`/`SEARCH`/`LOCK`) and warnings. |
| `ax_fps2`, `ay_fps2`, `az_fps2` | missile | ft/s² | Total acceleration (thrust, drag, gravity, guidance). |
| `achieved_lateral_g` | missile | g | Guidance acceleration achieved after the response lag. |
| `mass_slug` | missile | slug | Mass, propellant burning off over the motor's burn. |
| `launched`, `guidance_active` | missile | | Fired; still guiding (0 once resolved). |
| `seeker_mode`, `seeker_active`, `has_lock`, `time_since_lock_lost_s` | missile | | Seeker type, whether its own seeker is radiating, lock and memory-coast time. |
| `launch_turn_done`, `lofting` | missile | | Launch-turn and loft state. |
| `range_to_target_ft` | missile | ft | True range to the target. |
| `track_state`, `frame_hits`, `missed_frames` | track | | `NONE`/`TENTATIVE`/`CONFIRMED`; hits in the M-of-N window; consecutive misses. |
| `est_age_s` | missile, track | s | Age of the estimate: datalink staleness, or time since the radar last detected the target. |
| `datalink_lost` | missile | | The launcher's track was lost, so the datalink stopped. |
| `est_x_ft` … `est_vz_fps` | missile, track | ft, ft/s | Target estimate: the missile's datalinked belief, or the radar track's filtered state. |
| `est_pos_sigma_ft` | track | ft | 1-sigma position uncertainty of the track filter (root of the covariance trace). |
| `est_error_ft` | missile, track | ft | Distance from the estimate to the target's true position. |

A 120-second engagement recorded every tick is about 18 MB.
