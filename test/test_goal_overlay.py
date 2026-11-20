import numpy as np

from isaac_composer.goal_overlay import annotation_projection


def test_roof_cutoff_does_not_hide_goal_annotations_or_change_screen_location():
    near, far = 10.0, 1000.0
    projection = np.array([[1.7, 0, 0, 0], [0, 2.8, 0, 0],
                           [0, 0, -(far+near)/(far-near), -1],
                           [0, 0, -2*far*near/(far-near), 0]])
    goal_in_camera = np.array([1.5, -1.0, -12.5, 1.0])
    before = goal_in_camera @ projection
    before /= before[3]
    adjusted = np.asarray(annotation_projection(projection)).reshape(4, 4)
    after = goal_in_camera @ adjusted
    after /= after[3]
    assert before[2] < 0 < after[2] < 1
    np.testing.assert_allclose(after[:2], before[:2])
