"""Direct transfer-function oracle independent of production section updates."""

import numpy as np
import pytest

from quadrotor_math.geometric_filter import FeedbackDerivativeFilter


def test_constant_initialization_ownership_and_index_contract():
    f = FeedbackDerivativeFilter(0.02)
    c = np.array([0.02, -0.03, 0.05])
    for k in range(20):
        d, dd, new = f.step(c, k)
        np.testing.assert_array_equal(d, 0)
        np.testing.assert_array_equal(dd, 0)
        assert f.next_index == k
        f = new
    c[:] = 99
    assert np.max(f.sections) < 1
    assert not f.sections.flags.writeable
    for index in [19, 21, True, 20.0]:
        with pytest.raises(ValueError, match="index"):
            f.step(np.ones(3), index)
    assert f.next_index == 20


def test_filter_matches_direct_third_order_transfer_recurrence():
    h, w = 0.02, 30.0
    f = FeedbackDerivativeFilter(h, w)
    # Bilinear transforms: denominator (s+w)^3, numerator w^3*s**d.
    plus, minus = np.array([1.0, 1]), np.array([1.0, -1])
    base = (2 / h) * minus + w * plus
    den = np.polynomial.polynomial.polypow(base, 3)
    outputs = [[], []]
    cs = np.random.default_rng(19).normal(0, 0.02, (1500, 3))
    cs[0] = 0
    history = []
    for k, c in enumerate(cs):
        d, dd, f = f.step(c, k)
        history.append((d, dd))
        for r in (1, 2):
            num = (
                w**3
                * (2 / h) ** r
                * np.polynomial.polynomial.polymul(
                    np.polynomial.polynomial.polypow(minus, r),
                    np.polynomial.polynomial.polypow(plus, 3 - r),
                )
            )
            y = sum(num[i] * cs[k - i] for i in range(min(3, k) + 1))
            y -= sum(den[i] * outputs[r - 1][k - i] for i in range(1, min(3, k) + 1))
            outputs[r - 1].append(y / den[0])
    np.testing.assert_allclose(np.asarray(history)[:, 0], outputs[0], atol=1e-13)
    np.testing.assert_allclose(np.asarray(history)[:, 1], outputs[1], atol=5e-12)


@pytest.mark.parametrize(
    "h,w", [(0, 30), (0.02, 0), (0.04, 30), (0.02, np.inf), (True, 30), (1e-320, 30)]
)
def test_unresolved_or_invalid_period_rejected(h, w):
    with pytest.raises(ValueError):
        FeedbackDerivativeFilter(h, w)


def test_failed_step_leaves_immutable_state_reusable():
    f = FeedbackDerivativeFilter(0.02)
    _, _, f = f.step(np.zeros(3), 0)
    with pytest.raises(ValueError):
        f.step(np.full(3, 1e308), 1)
    _, _, good = f.step(np.ones(3), 1)
    assert good.next_index == 2 and f.next_index == 1


@pytest.mark.parametrize("frequency", [0.1, 0.25, 0.5])
def test_frequency_response_delay_and_gain(frequency):
    h, w = 0.02, 30.0
    f = FeedbackDerivativeFilter(h, w)
    times = np.arange(4000) * h
    out = []
    for k, t in enumerate(times):
        d, dd, f = f.step(np.full(3, np.cos(2 * np.pi * frequency * t)), k)
        out.append([d[0], dd[0]])
    keep = times >= 10
    basis = np.column_stack(
        [np.cos(2 * np.pi * frequency * times[keep]), np.sin(2 * np.pi * frequency * times[keep])]
    )
    fit = np.linalg.lstsq(basis, np.array(out)[keep], rcond=None)[0]
    for r in (1, 2):
        response = (fit[0, r - 1] - 1j * fit[1, r - 1]) / (1j * 2 * np.pi * frequency) ** r
        assert abs(abs(response) - 1) <= 0.03
        assert abs(np.angle(response, deg=True)) <= 20


def test_impulse_noise_gains_below_raw_cubic_stencils():
    f = FeedbackDerivativeFilter(0.02)
    out = []
    for k in range(500):
        d, dd, f = f.step(np.full(3, float(k == 1)), k)
        out.append([d[0], dd[0]])
    gains = np.linalg.norm(out, axis=0)
    raw = [np.linalg.norm([11, -18, 9, -2]) / (6 * 0.02), np.linalg.norm([2, -5, 4, -1]) / 0.02**2]
    assert np.all(gains / raw <= 0.1)


def test_rebase_removes_only_estimate_jumps_and_preserves_physical_history():
    original = FeedbackDerivativeFilter(0.02)
    shifted = FeedbackDerivativeFilter(0.02)
    offset = np.zeros(3)
    for k in range(150):
        # A smooth physical input continues through multiple estimator revisions.
        c = np.array([0.001 * k, 0.01 * np.sin(k / 12), 0.002 * k])
        if k in (15, 16, 71):
            jump = np.array([0.03, -0.08, 0.02]) * (1 if k != 16 else -2)
            before = shifted
            shifted = shifted.rebase(jump)
            offset += jump
            assert shifted.next_index == before.next_index
            assert not shifted.previous.flags.writeable
            assert not shifted.sections.flags.writeable
        d, dd, original = original.step(c, k)
        actual_d, actual_dd, shifted = shifted.step(c + offset, k)
        np.testing.assert_allclose(actual_d, d, atol=3e-15)
        np.testing.assert_allclose(actual_dd, dd, atol=3e-13)
    assert np.linalg.norm(d) > 0.01
    np.testing.assert_allclose(shifted.previous - original.previous, offset, atol=1e-16)


def test_rebase_is_owned_transactional_and_does_not_advance_clock():
    f = FeedbackDerivativeFilter(0.02)
    _, _, f = f.step(np.ones(3), 0)
    before = f.sections.copy()
    jump = np.array([0.1, 0.2, 0.3])
    revised = f.rebase(jump)
    jump[:] = 100
    np.testing.assert_array_equal(f.sections, before)
    np.testing.assert_array_equal(revised.previous, [1.1, 1.2, 1.3])
    assert revised.next_index == f.next_index == 1
    for invalid in (np.ones(2), np.full(3, np.inf), np.full(3, np.nan)):
        with pytest.raises(ValueError):
            f.rebase(invalid)
    enormous = FeedbackDerivativeFilter(
        0.02, previous=np.full(3, 1e308), sections=np.full((3, 3), 1e308)
    )
    with pytest.raises(ValueError, match="finite"):
        enormous.rebase(np.full(3, 1e308))
    np.testing.assert_array_equal(f.sections, before)
