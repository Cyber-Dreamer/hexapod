import pytest
from kinematics.ik import InverseKinematics

def test_ik_solve():
    ik = InverseKinematics(leg_length=10)
    theta, d = ik.solve(3, 4, 0)
    assert round(theta, 2) == 0.93  # atan2(4,3)
    assert round(d, 2) == 5.0
