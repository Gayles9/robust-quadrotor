import numpy as np
from numpy.random import Generator


def create_rng(seed: int) -> Generator:
    """Create a reproducible, independent NumPy generator from a seed."""
    return np.random.default_rng(seed)
