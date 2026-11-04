"""Fit a radar's system loss to its cited detection range.

A radar in this simulator detects on one look when its square-law detector
output crosses a threshold. The output is the target's return plus
receiver noise; the return's mean is the SNR form of the radar equation
(MIT Lincoln Laboratory, O'Donnell 2002)

    SNR = Pavg G^2 lam^2 sigma T / ((4 pi)^3 R^4 k T0 F L)

and a fighter's return fluctuates as Swerling 1, for which a look against
threshold T (noise power units) detects with probability exp(-T / (1 + SNR)).
The search threshold follows from the false-alarm probability alone,
T = ln(1 / pfa).

Every budget term but the system loss L is sourced or a recorded estimate,
in the type file with its provenance. L is the one free parameter:
CALIBRATED here so one look at a reference target of the cited size is a
coin flip (Pd 0.5) at exactly the cited range. A plausible L (total losses
of roughly 8-10 dB are typical, per the same notes) says the cited range is
consistent with the hardware; an implausible one says it is not. (There
is one detection threshold: a track confirms on frames at the search
threshold, so nothing else is fitted here.)

Two kinds of radar, same fit:

    python tools/fit/radar_budget.py aircraft f16c_block50   # the type's "radar"
    python tools/fit/radar_budget.py missile aim120          # its active seeker

Inputs: the type file (data/aircraft/<id>/aircraft.json "radar", or
data/missiles/<id>/missile.json "seeker.active_seeker") for every term but
those fitted, and fit/radar_fit.json beside it for what they are fitted to:

    {"range_nmi": 50, "rcs_m2": 5, "reference_fluctuation": "swerling_1",
     "notes": {...}}

No aspect factor: rcs_m2 is the target as the citation quotes it. The fit
is in free space: the sim models no atmospheric absorption, so whatever the air takes from a cited range is
inside the fitted loss. Writes the fitted values back into
the type file in place (formatting preserved). --check prints the fit and
writes nothing. The unit tests assert the type's per-look Pd 0.5 ranges
equal the fitted ones, so an edit to any other term that is not followed by
a re-fit fails loudly.
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
C = 299792458.0         # m/s, exact
K = 1.380649e-23        # J/K, exact
T0 = 290.0              # K, the IEEE noise-figure reference
NMI_M = 1852.0


def threshold_linear(pfa):
    """Square-law threshold, noise power units: noise alone exceeds it with probability pfa."""
    return math.log(1.0 / pfa)


def swerling1_required_snr(threshold, pd):
    """Mean SNR (linear) at which a Swerling 1 look against `threshold` detects with probability pd."""
    return threshold / math.log(1.0 / pd) - 1.0


def snr_db_lossless(avg_power_w, gain, frequency_hz, noise_figure_db, time_s, rcs_m2, range_m):
    lam = C / frequency_hz
    snr = (avg_power_w * gain ** 2 * lam ** 2 * rcs_m2 * time_s) / (
        (4.0 * math.pi) ** 3 * range_m ** 4 * K * T0 * 10.0 ** (noise_figure_db / 10.0))
    return 10.0 * math.log10(snr)


def load(kind, type_id):
    if kind == "aircraft":
        type_dir = REPO_ROOT / "data" / "aircraft" / type_id
        type_file = type_dir / "aircraft.json"
        block = json.loads(type_file.read_text())["radar"]
        terms = dict(avg_power_w=block["average_power_w"], gain=block["antenna_gain"],
                     frequency_hz=block["frequency_hz"], noise_figure_db=block["noise_figure_db"],
                     time_s=block["time_on_target_s"])
    else:
        type_dir = REPO_ROOT / "data" / "missiles" / type_id
        type_file = type_dir / "missile.json"
        block = json.loads(type_file.read_text())["seeker"]["active_seeker"]
        terms = dict(avg_power_w=block["average_power_w"], gain=block["antenna_gain"],
                     frequency_hz=block["frequency_hz"], noise_figure_db=block["noise_figure_db"],
                     time_s=block["integration_time_s"])
    fit = json.loads((type_dir / "fit" / "radar_fit.json").read_text())
    if fit.get("reference_fluctuation") != "swerling_1":
        raise SystemExit("radar_fit.json: only a swerling_1 reference target is supported")
    return type_file, block, terms, block["pfa"], fit


def mean_snr_db(terms, fit, loss_db, range_nmi):
    """Mean SNR in dB of the reference target at range_nmi."""
    return snr_db_lossless(rcs_m2=fit["rcs_m2"], range_m=range_nmi * NMI_M, **terms) - loss_db


def write_field(text, key, value):
    new_text, count = re.subn(r'("' + key + r'":\s*)[-+0-9.eE]+', lambda m: m.group(1) + f"{value:.4f}", text)
    if count != 1:
        raise SystemExit(f"expected exactly one {key}, found {count}")
    return new_text


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("kind", choices=["aircraft", "missile"])
    parser.add_argument("type_id")
    parser.add_argument("--check", action="store_true", help="print the fit, write nothing")
    args = parser.parse_args()

    type_file, block, terms, pfa, fit = load(args.kind, args.type_id)
    t_search = threshold_linear(pfa)
    required_db = 10.0 * math.log10(swerling1_required_snr(t_search, 0.5))
    snr0_db = mean_snr_db(terms, fit, 0.0, fit["range_nmi"])
    loss_db = round(snr0_db - required_db, 4)

    print(f"{args.kind} {args.type_id}: {fit['rcs_m2']} m2 Swerling 1 at {fit['range_nmi']} nmi")
    print(f"  search threshold {10 * math.log10(t_search):.2f} dB (pfa {pfa:g}); "
          f"Pd 0.5 needs mean SNR {required_db:.2f} dB")
    print(f"  SNR with no loss {snr0_db:.2f} dB")
    print(f"  ->  system_loss_db {loss_db:.4f}  (was {block['system_loss_db']})")
    updates = {"system_loss_db": loss_db}

    if args.check:
        return 0

    # Bytes, not text: a text-mode write on Windows would turn every line
    # ending into CRLF.
    text = type_file.read_bytes().decode("utf-8")
    for key, value in updates.items():
        text = write_field(text, key, value)
    type_file.write_bytes(text.encode("utf-8"))
    print(f"  wrote {type_file.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
