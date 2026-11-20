import numpy as np
import pytest
from isaac_composer.crowd_safety import yielding_velocities

def test_crossing_stops_before_overlap_even_when_future_endpoints_are_separate():
    result = yielding_velocities([[-1, 0], [0, -1]], [[2, 0], [0, 2]], [.3, .3])
    assert not np.any(result)


def test_approach_waiting_person_stops_but_departing_person_can_move():
    assert not np.any(yielding_velocities([[0, 0], [1, 0]], [[1, 0], [0, 0]], [.3, .3]))
    result = yielding_velocities([[0, 0], [1, 0]], [[-1, 0], [1, 0]], [.3, .3])
    np.testing.assert_equal(result, [[-1, 0], [1, 0]])


def test_stopping_conflict_propagates_to_following_person():
    result = yielding_velocities([[0, 0], [1, 0], [2, 0]], [[1, 0], [1, 0], [0, 0]], [.3] * 3)
    assert not np.any(result)

