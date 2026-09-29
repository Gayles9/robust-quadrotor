"""ADR0039 saved whole-flight budgets and prospective local sensitivities; no flights."""

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import source_sha256
from experiments.durable_release_evidence import save_arrays, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.hover_response import cascade_model, cascade_transition, cascade_updates
from experiments.residual_hover_diagnostic import CHANNELS
from experiments.robustness_evidence import ensure, equal_json
from experiments.supported_start import plain_configuration
from experiments.supported_validation_protocol import CLEAN, Job, configuration, jobs

Array = NDArray[np.float64]
Complex = NDArray[np.complex128]
ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0039-whole-flight-error-budget.md"
INPUT_SHA = "09c53c8e86f94f16b685fa6f297e54ecd4bcc3a7e78cb7c806217ed020910276"
MANIFEST_SHA = "1a67b992e5daa5a299bbe5cf4c31927386b71637c14172e673c82316ff768d0f"
PRECEDING_SOURCE = "01d663368ed67174c9b300fdd2ef7450781da5f6ec3947e7f9a9b2d8473f2b0e"
INPUTS = ("position_m", "velocity_m_s", "inclination_rad", "inclination_rate_rad_s")


@dataclass(frozen=True)
class Lift:
    """x[m+1]=A x[m]+B u[m], y[m,j]=C[j] x[m]+D[j] u[m].

    x has five physical states; u contains two outer samples and inclination/
    inclination-rate samples at each inner epoch. Outputs precede clock updates.
    Inclination errors are horizontal acceleration-axis errors divided by g;
    near level they are a signed rotation of roll/pitch errors in NED.
    """

    A: Array
    B: Array
    C: Array
    D: Array
    phases: NDArray[np.int64]
    groups: NDArray[np.int64]
    h: float

    @property
    def period(self) -> float:
        return len(self.C) * self.h


def lift_cascade(
    *,
    h: float,
    tau: float,
    kp: float,
    kv: float,
    ka: float,
    kr: float,
    gravity: float,
    outer_stride: int,
    inner_stride: int,
) -> Lift:
    """Exact lifting of the existing sampled local model, including intersample output."""
    ensure(
        type(outer_stride) is int
        and type(inner_stride) is int
        and outer_stride >= inner_stride > 0
        and outer_stride % inner_stride == 0,
        "nested lift clocks",
    )
    ensure(np.isfinite(gravity) and gravity > 0, "positive gravity")
    flow = cascade_transition(h, tau)
    outer, inner = cascade_updates(kp, kv, ka, kr)
    phases = np.array([0, 0] + [k for k in range(0, outer_stride, inner_stride) for _ in range(2)])
    groups = np.array([0, 1] + [v for _ in range(0, outer_stride, inner_stride) for v in (2, 3)])
    width = len(phases)
    state, drive = np.eye(7)[:, :5], np.zeros((7, width))
    C, D = np.empty((outer_stride, 5)), np.empty((outer_stride, width))
    for k in range(outer_stride):
        C[k], D[k] = state[0], drive[0]
        if k == 0:
            state, drive = outer @ state, outer @ drive
            drive[5, :2] = -kp, -kv
        if k % inner_stride == 0:
            state, drive = inner @ state, inner @ drive
            index = 2 + 2 * (k // inner_stride)
            drive[6, index : index + 2] = -kr * ka * gravity, -kr * gravity
        state, drive = flow @ state, flow @ drive
    ensure(np.max(np.abs(np.linalg.eigvals(state[:5]))) < 1, "stable local lift")
    return Lift(state[:5], drive[:5], C, D, phases, groups, h)


def phasor(lift: Lift, frequency_hz: float) -> tuple[Complex, Complex, Complex]:
    """Return periodic input, state and all phase outputs for four unit sinusoids."""
    ensure(np.isfinite(frequency_hz) and 0 <= frequency_hz <= 0.5 / lift.period, "frequency range")
    omega = 2 * np.pi * frequency_hz
    U = np.zeros((len(lift.groups), 4), dtype=complex)
    U[np.arange(len(U)), lift.groups] = np.exp(1j * omega * lift.phases * lift.h)
    X = np.linalg.solve(np.exp(1j * omega * lift.period) * np.eye(5) - lift.A, lift.B @ U)
    Y = lift.C @ X + lift.D @ U
    return (
        np.asarray(U, dtype=np.complex128),
        np.asarray(X, dtype=np.complex128),
        np.asarray(Y, dtype=np.complex128),
    )


def impulse_bounds(lift: Lift, periods: int) -> tuple[Array, Array]:
    """Exact finite-horizon l-infinity induced bounds for zero initial state.

    Coefficients retain each inner sample; absolute values are summed only after
    grouping the independent sample slots. Each input bound is a horizontal
    vector-norm bound, so triangle inequality needs no extra sqrt(2) factor.
    Returns bounds[m,phase,group] and kernel[lag-1,phase,input_slot].
    """
    ensure(type(periods) is int and periods > 0, "positive period count")
    kernel = np.empty((periods, *lift.D.shape))
    state = lift.B.copy()
    for lag in range(periods):
        kernel[lag] = lift.C @ state
        state = lift.A @ state
    bounds = np.empty((periods + 1, len(lift.C), 4))
    for group in range(4):
        selected = lift.groups == group
        bounds[0, :, group] = np.abs(lift.D[:, selected]).sum(axis=1)
        bounds[1:, :, group] = bounds[0, :, group] + np.cumsum(
            np.abs(kernel[:, :, selected]).sum(axis=2), axis=0
        )
    return bounds, kernel


def gram_budget(channels: Array, error: Array) -> dict[str, Any]:
    """Uncentred Gram matrix: MSE=sum(G), including every signed cross term."""
    ensure(channels.ndim == 3 and channels.shape[2] == 3 and len(channels) > 0, "Gram channels")
    ensure(error.shape == (len(channels), 3), "Gram error shape")
    ensure(np.all(np.isfinite(channels)) and np.all(np.isfinite(error)), "finite Gram data")
    closure = float(np.max(np.abs(channels.sum(axis=1) - error)))
    ensure(closure <= 1e-10, "Gram response sum")
    G = np.einsum("nci,ndi->cd", channels, channels) / len(channels)
    mse = float(np.mean(np.sum(error**2, axis=1)))
    ensure(abs(float(G.sum()) - mse) <= 1e-12, "Gram MSE closure")
    diagonal = float(np.trace(G))
    return dict(
        samples=len(channels),
        gram_m2=G.tolist(),
        mean_error_W=error.mean(axis=0).tolist(),
        mse_m2=mse,
        diagonal_m2=diagonal,
        cross_m2=float(G.sum() - diagonal),
        signed_shares_m2=G.sum(axis=1).tolist(),
        rms_m=float(np.sqrt(mse)),
        triangle_rms_bound_m=float(np.sqrt(np.diag(G)).sum()),
        response_closure_m=closure,
    )


def information_structure(gravity: float) -> dict[str, Any]:
    """Independent local position/velocity/tilt observability comparison in SI units."""
    ensure(np.isfinite(gravity) and gravity > 0, "information gravity")
    F = np.zeros((15, 15))
    F[:3, 3:6] = np.eye(3)
    F[3, 7], F[4, 6] = -gravity, gravity
    F[3:6, 9:12] = -np.eye(3)
    F[6:9, 12:15] = -np.eye(3)
    position, velocity, tilt = np.eye(15)[:3], np.eye(15)[3:6], np.eye(15)[6:8]
    null = np.zeros((15, 4))
    null[6, 0], null[10, 0] = 1, gravity
    null[7, 1], null[9, 1] = 1, -gravity
    null[8, 2], null[14, 3] = 1, 1
    result: dict[str, Any] = dict(F=F.tolist(), null_vectors=null.tolist(), cases={})
    for name, H in dict(
        existing_position=position,
        added_velocity=np.vstack((position, velocity)),
        added_inclination=np.vstack((position, tilt)),
    ).items():
        observability = np.vstack([H @ np.linalg.matrix_power(F, k) for k in range(15)])
        result["cases"][name] = dict(
            rank=int(np.linalg.matrix_rank(observability)),
            null_output_norm=np.linalg.norm(observability @ null, axis=0).tolist(),
        )
    return result


def preceding_source_sha256() -> str:
    """Bind all previous execution files; exclude only this new analysis module."""
    digest = hashlib.sha256()
    for path in sorted(
        [
            *ROOT.glob("src/quadrotor_math/*.py"),
            *ROOT.glob("experiments/*.py"),
            ROOT / "pyproject.toml",
            ROOT / "uv.lock",
        ]
    ):
        if path == Path(__file__).resolve():
            continue
        data = path.read_bytes()
        digest.update(path.relative_to(ROOT).as_posix().encode() + b"\0")
        digest.update(str(len(data)).encode() + b"\0" + data)
    return digest.hexdigest()


def authenticate(root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """The immutable archive manifest binds payloads absent from the original report."""
    manifest = json.loads(authenticated(root / "manifest.json", MANIFEST_SHA))
    for item in manifest:
        name = Path(item["file"])
        ensure(not name.is_absolute() and ".." not in name.parts, "relative evidence path")
        data = authenticated(root / name, item["sha256"])
        ensure(len(data) == item["bytes"], "evidence byte count")
    report = json.loads(authenticated(root / "diagnosis/report.json", INPUT_SHA))
    ensure(
        [r["name"] for r in report["cases"]] == [j.name for j in jobs() if j.case in CLEAN],
        "full ledger",
    )
    ensure(report["summary"]["prior_frozen_comparison_passed"] is False, "retain failed gate")
    return report, manifest


def sensor_budget(args: dict[str, Any]) -> dict[str, Any]:
    """Audit per-sample noise, walks, derivative noise and shared-draw correlations."""
    sensors = args["sensors"]
    dt = sensors.local_position_schedule.sample_period_s
    sigma = sensors.position.local_position_noise_standard_deviation_W
    # (n[k]-n[k-1])/dt and (n[k]-2n[k-1]+n[k-2])/dt².
    variance = sigma**2
    return dict(
        sensors=plain_configuration(sensors),
        numerics=plain_configuration(args["numerics"]),
        position_controller=plain_configuration(args["position_controller"]),
        attitude_controller=plain_configuration(args["attitude_controller"]),
        innovation_policy=plain_configuration(args["estimator_configuration"].innovation_policy),
        difference_velocity_std_m_s=(np.sqrt(2) * sigma / dt).tolist(),
        difference_acceleration_std_m_s2=(np.sqrt(6) * sigma / dt**2).tolist(),
        position_difference_velocity_covariance_m2_s=(variance / dt).tolist(),
        successive_difference_velocity_covariance_m2_s2=(-variance / dt**2).tolist(),
        interpretation="Differencing is not a new independent observation or an estimator design.",
    )


def check_phasor(lift: Lift, params: dict[str, Any], frequency: float) -> float:
    """Compare steady complex phasors with the separate real time-domain cascade."""
    _, X, Y = phasor(lift, frequency)
    stride = len(lift.C)
    t = np.asarray(np.arange(10 * stride + 1) * lift.h, dtype=np.float64)
    oscillation = np.exp(2j * np.pi * frequency * t)
    error = 0.0
    for group in range(4):
        drives = [np.zeros((len(t), 2)) for _ in range(4)]
        drives[group] = np.column_stack((oscillation.real, oscillation.imag))
        for index in (2, 3):
            drives[index] *= params["gravity"]
        initial = np.column_stack((X[:, group].real, X[:, group].imag))
        states, _ = cascade_model(
            t,
            initial,
            *drives,
            **{k: v for k, v in params.items() if k not in ("h", "gravity")},
        )
        for k in range(len(t)):
            value = Y[k % stride, group] * np.exp(
                2j * np.pi * frequency * (k // stride) * lift.period
            )
            error = max(error, float(np.max(np.abs(states[k, 0] - [value.real, value.imag]))))
    ensure(error <= 1e-10, "independent phasor evolution")
    return error


def run(evidence: Path, output: Path) -> dict[str, Any]:
    ensure(preceding_source_sha256() == PRECEDING_SOURCE, "preceding execution identity")
    report, manifest = authenticate(evidence)
    args = configuration(Job("nominal_hover", 47001), "campaign")
    outer, inner, numerics = (
        args[k] for k in ("position_controller", "attitude_controller", "numerics")
    )
    params = dict(
        h=numerics.time_step_s,
        tau=args["truth_rotors"].motor_time_constant_s,
        kp=float(outer.position_gain_W[0]),
        kv=float(outer.velocity_gain_W[0]),
        ka=float(inner.attitude_gain_B[0]),
        kr=float(inner.rate_gain_B[0]),
        gravity=args["truth_world"].gravity_acceleration,
        outer_stride=numerics.position_stride,
        inner_stride=numerics.attitude_stride,
    )
    lift = lift_cascade(**params)
    frequency = np.r_[0.0, np.geomspace(0.01, 25.0, 241)]
    response = np.stack([phasor(lift, float(f))[2] for f in frequency])
    dc = np.array(
        [
            -1,
            -params["kv"] / params["kp"],
            -params["gravity"] / params["kp"],
            -params["gravity"] / (params["ka"] * params["kp"]),
        ]
    )
    dc_error = float(np.max(np.abs(response[0] - dc)))
    ensure(dc_error <= 1e-10, "analytic DC sensitivity")
    frequency_error = max(check_phasor(lift, params, f) for f in (0.0, 0.2, 2.5, 25.0))
    bounds, kernel = impulse_bounds(lift, round(11 / lift.period))
    time = np.arange(bounds.shape[0])[:, None] * lift.period + np.arange(len(lift.C)) * lift.h
    mask = (time >= 5) & (time <= 11)
    maxima = bounds[mask].max(axis=0)
    output.mkdir(parents=True, exist_ok=False)
    frozen = dict(
        audited_commit="b4fc5821fe41474bc69edf6d92d9a97286da8824",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        input_manifest_sha256=MANIFEST_SHA,
        params=params,
        channels=CHANNELS,
        inputs=INPUTS,
    )
    save_json(output, "protocol.json", frozen)
    save_json(output, "input-manifest.json", manifest)
    save_arrays(
        output / "sampled-sensitivity.npz",
        dict(
            A=lift.A,
            B=lift.B,
            C=lift.C,
            D=lift.D,
            phases=lift.phases,
            groups=lift.groups,
            frequency_hz=frequency,
            response=response,
            bound_time_s=time,
            bounds=bounds,
            kernel=kernel,
        ),
    )
    rows, ledger, changed = [], [], []
    for row in report["cases"]:
        target = evidence / "diagnosis" / row["name"]
        equal_json(json.loads((target / "case.json").read_bytes()), row, "source case")
        arms, traces = {}, {}
        for label, arm in row["arms"].items():
            with np.load(target / (label + "-response.npz"), allow_pickle=False) as saved:
                t, channels, error = (
                    saved["time_s"],
                    saved["response_error_W"],
                    saved["true_error_W"],
                )
            land = 11 if row["case"] == "nominal_hover" else 13
            masks = dict(
                full=t >= 0,
                initialize=t < 1,
                takeoff=(t >= 1) & (t < 5),
                active=(t >= 5) & (t < land),
                landing=t >= land,
            )
            if row["case"] == "nominal_hover":
                masks["scored_hover"] = (t >= 5) & (t <= 11)
            budgets = {name: gram_budget(channels[m], error[m]) for name, m in masks.items()}
            ensure(
                abs(budgets["full"]["mse_m2"] - arm["metrics"]["tracking_rmse_m"] ** 2) <= 1e-12,
                "original score",
            )
            traces[label] = json.loads((target / (label + "-updates.json")).read_bytes())["events"]
            ensure(len(traces[label]) == arm["events"], "complete event ledger")
            for event in traces[label]:
                threshold = 11.345 if event["sensor"] == "position" else 6.635
                ensure(event["fused"] == (event["nis"] <= threshold), "saved gate branch")
                ledger.append(
                    dict(
                        job=row["name"],
                        arm=label,
                        sensor=event["sensor"],
                        time_s=event["time_s"],
                        nis=event["nis"],
                        threshold=threshold,
                        margin=threshold - event["nis"],
                        fused=event["fused"],
                    )
                )
            arms[label] = dict(metrics=arm["metrics"], windows=budgets)
        for control in ("original", "boundary"):
            end = row["comparisons"][control]["common_time_s"]
            a, b = ([e for e in traces[k] if e["time_s"] <= end] for k in (control, "combined"))
            for left, right in zip(a, b, strict=True):
                ensure(
                    (left["time_s"], left["sensor"]) == (right["time_s"], right["sensor"]),
                    "paired event clock",
                )
                if left["fused"] != right["fused"]:
                    changed.append(
                        dict(
                            job=row["name"],
                            control=control,
                            time_s=left["time_s"],
                            sensor=left["sensor"],
                            control_nis=left["nis"],
                            combined_nis=right["nis"],
                        )
                    )
        rows.append(dict(name=row["name"], arms=arms))
        print(row["name"], "Gram budgets and gate ledger PASS", flush=True)
    save_json(output, "gate-ledger.json", ledger)
    result = dict(
        protocol=frozen,
        sensors=sensor_budget(args),
        information=information_structure(params["gravity"]),
        sensitivity=dict(
            dc=dc.tolist(),
            dc_max_error=dc_error,
            phasor_max_error=frequency_error,
            finite_hover_bounds=maxima.tolist(),
            single_input_8cm_ceilings=(0.08 / maxima).tolist(),
            spectral_radius=float(np.max(np.abs(np.linalg.eigvals(lift.A)))),
        ),
        cases=rows,
        gate_changes=changed,
        summary=dict(
            complete_histories=len(rows) * 3,
            observation_events=len(ledger),
            rejected_events=sum(not e["fused"] for e in ledger),
            paired_gate_changes=len(changed),
            new_scientific_flights=0,
            prior_comparison_passed=False,
            flight_qualification=False,
            implementation_go=False,
            independent_inclination_contract_justified=True,
        ),
    )
    ensure(source_sha256(ROOT) == frozen["source_sha256"], "source changed during review")
    save_json(output, "report.json", result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.evidence, args.output)["summary"], indent=2))


if __name__ == "__main__":
    main()
