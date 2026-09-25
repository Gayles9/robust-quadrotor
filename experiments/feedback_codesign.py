"""One bounded observed-data design cycle, not runtime control or qualification.

See docs/progress/2026-09-24-feedback-codesign.md for the frozen objective.
No optional dependencies, reserved seeds, filter changes, or nonlinear gain sweep.
"""

import argparse
import hashlib
import io
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.feedback_bandwidth_validation import make_configuration
from experiments.feedback_motor_damping_probe import SEEDS, _run_configuration
from quadrotor_math.attitude_control import attitude_error_body
from quadrotor_math.cascade_analysis import hover_axis_zero_order_hold

FloatArray = NDArray[np.float64]
REPORT_SHA256 = "86d2c651b31eaf7a4afada0c0987a16008d05c6dd2be7ce10f13fbec51cadf90"
LOW = np.array([4.0, 2.0, 4.0, 8.0, 0.0])
HIGH = np.array([40.0, 12.0, 24.0, 80.0, 2.0])
ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Gains:
    kp: float
    kv: float
    ka: float
    kr: float
    damping: float

    def __post_init__(self) -> None:
        values = (self.kp, self.kv, self.ka, self.kr, self.damping)
        if any(isinstance(v, (bool, np.bool_)) or not np.isscalar(v) for v in values):
            raise ValueError("gains must be real scalars within frozen bounds")
        a = np.asarray(values)
        if a.dtype.kind not in "fi" or not np.all(np.isfinite(a)):
            raise ValueError("gains must be finite real scalars")
        if np.any(a < LOW) or np.any(a > HIGH):
            raise ValueError("gains outside frozen bounds")

    def array(self) -> FloatArray:
        return np.array([self.kp, self.kv, self.ka, self.kr, self.damping], dtype=np.float64)


def decode(point: FloatArray) -> Gains:
    z = np.asarray(point)
    if z.shape != (5,) or z.dtype.kind not in "fi" or not np.all(np.isfinite(z)):
        raise ValueError("point must be a finite five-vector")
    if np.any(z < 0) or np.any(z > 1):
        raise ValueError("point outside unit box")
    result = LOW.copy()
    result[:4] *= (HIGH[:4] / LOW[:4]) ** z[:4]
    result[4] = 2 * z[4]
    return Gains(*np.minimum(result, HIGH).tolist())


def encode(gains: Gains) -> FloatArray:
    result = gains.array()
    result[:4] = np.log(result[:4] / LOW[:4]) / np.log(HIGH[:4] / LOW[:4])
    result[4] /= 2
    return result


def design_points() -> list[Gains]:
    rng = np.random.Generator(np.random.PCG64(250924))
    z = (np.arange(510)[:, None] + rng.random((510, 5))) / 510
    for axis in range(5):
        rng.shuffle(z[:, axis])
    return [Gains(6.4, 4, 8, 16, 0), Gains(14.4, 6, 12, 36, 0.5)] + [decode(row) for row in z]


def lifted_model(gains: Gains) -> tuple[FloatArray, FloatArray]:
    """Return A (5,5), B (5,6), sampled just before each outer update.

    x=[p,v,eta,r,a], eta=-pitch North / roll East. Independent standardized
    inputs are [ep,ev,eeta0,er0,eeta1,er1]; the latter pairs act at the two
    separate inner updates. Error signs are estimate minus truth.
    """
    kp, kv, ka, kr, damping = gains.array()
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)
    inner = np.eye(6)
    inner[:5, :5] = phi
    inner[:5] += np.outer(gamma, [0, 0, -kr * ka, -kr, -damping, kr * ka])
    reset = np.eye(6)
    reset[5] = [-kp / 9.81, -kv / 9.81, 0, 0, 0, 0]
    outer_error = np.zeros((6, 6))
    outer_error[5, :2] = [-kp * 0.02 / 9.81, -kv * 0.02 / 9.81]
    first, second = np.zeros((6, 6)), np.zeros((6, 6))
    kick = np.outer(gamma, [-kr * ka * np.deg2rad(1), -kr * 0.002])
    first[:5, 2:4], second[:5, 4:6] = kick, kick
    return (inner @ inner @ reset)[:5, :5], (inner @ (inner @ outer_error + first) + second)[:5]


def noise_response(a: FloatArray, b: FloatArray) -> float:
    """Position RMS for the declared white design inputs, not actual ESKF RMS."""
    if np.max(np.abs(np.linalg.eigvals(a))) >= 1:
        raise ValueError("noise response requires a stable transition")
    scale = np.array([0.07, 0.2, 0.1, 1, 20])
    aa, bb = a / scale[:, None] * scale, b / scale[:, None]
    q = bb @ bb.T
    p = np.linalg.solve(np.eye(25) - np.kron(aa, aa), q.reshape(25)).reshape(5, 5)
    p = (p + p.T) / 2
    if not np.all(np.isfinite(p)) or np.linalg.eigvalsh(p).min() < -1e-10:
        raise ValueError("invalid Lyapunov covariance")
    if np.max(abs(p - aa @ p @ aa.T - q)) > 1e-9 * max(1, np.max(abs(p))):
        raise ValueError("Lyapunov residual exceeds tolerance")
    return float(np.sqrt(max(0, p[0, 0])) * scale[0])


def local_metrics(gains: Gains) -> dict[str, Any]:
    a, b = lifted_model(gains)
    poles = np.log(np.linalg.eigvals(a).astype(complex)) / 0.02
    stable = bool(np.max(poles.real) < -3 and np.min(-poles.real / abs(poles)) >= 0.55)
    return {
        "gains": gains.array().tolist(),
        "poles_real_imag": np.column_stack((poles.real, poles.imag)).tolist(),
        "stable_screen": stable,
        "noise_response_m": noise_response(a, b) if stable else None,
    }


def load_development(folder: Path) -> tuple[dict[str, FloatArray], list[dict[str, Any]]]:
    """Authenticate the exact observed report and each required data archive."""
    raw = (folder / "report.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != REPORT_SHA256:
        raise ValueError("unexpected development report identity")
    report = json.loads(raw)
    errors, initials, actuals, inputs = [], [], [], []
    times = np.arange(1001) * 0.01
    for i in range(20):
        trial = report["trials"][i]
        if trial["job"] != {"case": "hover", "seed": 93000 + i, "noiseless": False}:
            raise ValueError("unexpected observed job")
        part = trial["history_parts"][0]
        if Path(part["file"]).name != part["file"]:
            raise ValueError("archive name must be a basename")
        raw = (folder / part["file"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != part["sha256"]:
            raise ValueError("development archive digest mismatch")
        with np.load(io.BytesIO(raw), allow_pickle=False) as archive:
            t = archive["mission_time_s"][:4001:4]
            if not np.array_equal(t, times):
                raise ValueError("unexpected development clock")
            keys = [
                "mission_position_W",
                "mission_velocity_W",
                "mission_q_WB",
                "mission_omega_B",
                "estimate_position_W",
                "estimate_velocity_W",
                "estimate_q_WB",
                "angular_velocity_estimate_B",
            ]
            h = {key: archive[key][:4001:4] for key in keys}
        angle, estimated_angle = [
            np.array([attitude_error_body(np.array([1.0, 0, 0, 0]), q) for q in h[key]])
            for key in ("mission_q_WB", "estimate_q_WB")
        ]
        ep = (h["estimate_position_W"] - h["mission_position_W"])[:, :2]
        ev = (h["estimate_velocity_W"] - h["mission_velocity_W"])[:, :2]
        et = (estimated_angle - angle)[:, [1, 0]] * [-1, 1]
        er = (h["angular_velocity_estimate_B"] - h["mission_omega_B"])[:, [1, 0]] * [-1, 1]
        errors.append(np.stack((ep, ev, et, er), axis=-1))
        initials.append(
            np.column_stack(
                (
                    h["mission_position_W"][0, :2],
                    h["mission_velocity_W"][0, :2],
                    angle[0, [1, 0]] * [-1, 1],
                    h["mission_omega_B"][0, [1, 0]] * [-1, 1],
                    np.zeros(2),
                )
            )
        )
        actuals.append(h["mission_position_W"][:, :2])
        inputs.append({"job": trial["job"], "file": part["file"], "sha256": part["sha256"]})
    return {
        "errors": np.stack(errors, axis=1),
        "initial": np.array(initials),
        "actual": np.stack(actuals, axis=1),
    }, inputs


def replay(gains: list[Gains], data: dict[str, FloatArray]) -> list[dict[str, float]]:
    """Vectorized exact linear inner ZOH; errors remain frozen external inputs.

    Moments use pitch inertia for North and roll inertia for East. No saturation
    is applied to this *linear* design model. Nonlinear screening is separate.
    The t=10 endpoint is scored but no command at that time contributes effort.
    """
    e, initial, actual = data["errors"], data["initial"], data["actual"]
    n = len(initial)
    if e.shape != (1001, n, 2, 4) or initial.shape != (n, 2, 5) or actual.shape != (1001, n, 2):
        raise ValueError("unexpected development array shape")
    if n < 1 or not all(np.all(np.isfinite(v)) for v in (e, initial, actual)):
        raise ValueError("development arrays must be finite and nonempty")
    if not gains:
        return []
    kp, kv, ka, kr, damping = np.array([g.array() for g in gains]).T[:, :, None, None]
    x = np.broadcast_to(initial, (len(gains), n, 2, 5)).copy()
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)
    peak, pos_energy, effort, moment_peak, rate_peak, discrepancy = np.zeros((6, len(gains)))
    desired = np.zeros_like(x[..., 0])
    for k in range(1001):
        p2 = np.sum(x[..., 0] ** 2, axis=2)
        if k >= 500:
            peak = np.maximum(peak, np.sqrt(np.max(p2, axis=1)))
            pos_energy += np.sum(p2, axis=1) / (501 * n)
        discrepancy = np.maximum(
            discrepancy, np.max(np.linalg.norm(x[..., 0] - actual[k], axis=2), axis=1)
        )
        if k == 1000:
            break
        if k % 2 == 0:
            desired = -(kp * (x[..., 0] + e[k, :, :, 0]) + kv * (x[..., 1] + e[k, :, :, 1])) / 9.81
        rate = ka * (desired - x[..., 2] - e[k, :, :, 2])
        command = kr * (rate - x[..., 3] - e[k, :, :, 3]) - damping * x[..., 4]
        moment = command * [0.025, 0.020]
        rate_peak = np.maximum(rate_peak, np.max(abs(rate), axis=(1, 2)))
        moment_peak = np.maximum(moment_peak, np.max(abs(moment), axis=(1, 2)))
        effort += np.sum(moment**2, axis=(1, 2)) / (1000 * n)
        x = x @ phi.T + command[..., None] * gamma
    return [
        dict(
            zip(
                (
                    "peak_m",
                    "position_mse_m2",
                    "moment_mse_Nm2",
                    "moment_peak_Nm",
                    "rate_peak_rad_s",
                    "recorded_position_discrepancy_m",
                ),
                map(float, values),
                strict=True,
            )
        )
        for values in zip(
            peak, pos_energy, effort, moment_peak, rate_peak, discrepancy, strict=True
        )
    ]


def objective(row: dict[str, Any]) -> float:
    return float(
        (row["peak_m"] / 0.07) ** 2
        + 0.20 * row["position_mse_m2"] / 0.03**2
        + 0.10 * row["moment_mse_Nm2"] / 0.15**2
        + 0.10 * (row["moment_peak_Nm"] / 0.8) ** 2
        + 0.10 * (row["noise_response_m"] / 0.02) ** 2
    )


def evaluate(gains: list[Gains], data: dict[str, FloatArray]) -> list[dict[str, Any]]:
    rows = [local_metrics(g) for g in gains]
    indices = [i for i, r in enumerate(rows) if r["stable_screen"]]
    for i, metric in zip(indices, replay([gains[i] for i in indices], data), strict=True):
        rows[i].update(metric)
        rows[i]["admissible"] = metric["moment_peak_Nm"] <= 1.6 and metric["rate_peak_rad_s"] <= 2
        rows[i]["objective"] = objective(rows[i])
    for row in rows:
        row.setdefault("admissible", False)
    return rows


def select(rows: list[dict[str, Any]]) -> dict[str, Any]:
    candidates = [r for r in rows if r["admissible"]]
    if not candidates:
        raise ValueError("no admissible design within the frozen search budget")
    return min(candidates, key=lambda r: r["objective"])


def synthesize(data: dict[str, FloatArray]) -> list[dict[str, Any]]:
    rows = evaluate(design_points(), data)
    for step in (0.10, 0.05, 0.025):
        if not any(row["admissible"] for row in rows):
            break  # Preserve the failed ledger instead of discarding it in an exception.
        center = encode(Gains(*select(rows)["gains"]))
        neighbors = [
            decode(np.clip(center + sign * step * axis, 0, 1))
            for axis in np.eye(5)
            for sign in (-1, 1)
        ]
        rows.extend(evaluate(neighbors, data))
    return rows


def candidate_configuration(seed: int, gains: Gains) -> dict[str, Any]:
    if type(seed) is not int or seed not in SEEDS:
        raise ValueError("only declared observed seeds are supported")
    kwargs = make_configuration(
        {"case": "hover", "seed": seed, "noiseless": False}, design_version=2
    )
    kwargs["position_controller"] = replace(
        kwargs["position_controller"],
        position_gain_W=np.array([gains.kp, gains.kp, 2.25]),
        velocity_gain_W=np.array([gains.kv, gains.kv, 3.0]),
    )
    kwargs["attitude_controller"] = replace(
        kwargs["attitude_controller"],
        attitude_gain_B=np.array([gains.ka, gains.ka, 2.0]),
        rate_gain_B=np.array([gains.kr, gains.kr, 8.0]),
    )
    return kwargs


def run_candidate(
    seed: int, output: Path, gains: Gains, *, horizon_s: float = 10.0
) -> dict[str, Any]:
    row = _run_configuration(
        seed,
        candidate_configuration(seed, gains),
        output,
        damping=gains.damping,
        horizon_s=horizon_s,
    )
    row["passes_margin_screen"] = bool(row["passes_prefix_screen"] and row["peak_m"] < 0.07)
    return row


def selected_gains(record: dict[str, Any]) -> Gains:
    """Refuse stale sources, wrong development identity or a changed winner."""
    if record["source_sha256"] != source_sha256(ROOT):
        raise ValueError("execution sources changed after selection")
    if record["input_report_sha256"] != REPORT_SHA256:
        raise ValueError("unexpected development report identity")
    if not 1 <= len(record["evaluations"]) <= 542 or record["budget"] != 542:
        raise ValueError("unexpected evaluation budget")
    if record["selected"] is None or record["selected"] != select(record["evaluations"]):
        raise ValueError("selection is not the minimum admissible objective")
    return Gains(*record["selected"]["gains"])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    design = sub.add_parser("synthesize")
    design.add_argument("--original", type=Path, required=True)
    design.add_argument("--output", type=Path, required=True)
    probe = sub.add_parser("probe")
    probe.add_argument("--selection", type=Path, required=True)
    probe.add_argument("--output", type=Path, required=True)
    probe.add_argument("--workers", type=int, choices=range(1, 5), default=3)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    record: dict[str, Any] = {
        "date_utc": datetime.now(UTC).isoformat(),
        "source_sha256": source_sha256(ROOT),
        "scope_sha256": hashlib.sha256(
            (ROOT / "docs/progress/2026-09-24-feedback-codesign.md").read_bytes()
        ).hexdigest(),
        "semantics": "Observed-data development only; not full-mission or fresh qualification.",
    }
    if args.command == "synthesize":
        data, inputs = load_development(args.original)
        with (args.output / "development-inputs.npz").open("xb") as stream:
            np.savez_compressed(stream, allow_pickle=False, **data)
        rows = synthesize(data)
        success = any(row["admissible"] for row in rows)
        record.update(
            input_report_sha256=REPORT_SHA256,
            inputs=inputs,
            evaluations=rows,
            selected=select(rows) if success else None,
            budget=542,
        )
        result_path = args.output / "selection.json"
    else:
        raw = args.selection.read_bytes()
        selected = json.loads(raw)
        gains = selected_gains(selected)
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            rows = list(pool.map(partial(run_candidate, output=args.output, gains=gains), SEEDS))
        success = all(r["passes_margin_screen"] for r in rows)
        record.update(
            selection_sha256=hashlib.sha256(raw).hexdigest(),
            gains=gains.array().tolist(),
            results=rows,
            passes_margin_screen=success,
        )
        result_path = args.output / "report.json"
    result_path.write_bytes(canonical_json(record) + b"\n")
    print(json.dumps({k: v for k, v in record.items() if k not in ("evaluations", "inputs")}))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
