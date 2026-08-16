import numpy as np

from quadrotor_math.randomness import create_rng


def test_create_rng_repeats_sequence_for_same_seed() -> None:
    first_rng = create_rng(12345)
    second_rng = create_rng(12345)

    np.testing.assert_array_equal(first_rng.standard_normal(5), second_rng.standard_normal(5))
