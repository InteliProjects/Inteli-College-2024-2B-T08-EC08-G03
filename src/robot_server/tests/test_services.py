from app.services import get_available_poses, move_robot_async 
import pytest

def test_get_available_poses():
    poses = get_available_poses()
    assert len(poses) == 10
    assert poses[0]["x"] == 0.368
    assert poses[0]["y"] == 0.049

def test_move_robot_to_pose_invalid_index():
    with pytest.raises(ValueError):
        _ = move_robot_async(-1)
    with pytest.raises(ValueError):
        _= move_robot_async(10)
