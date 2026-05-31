import os


def test_urdf_exists():
    path = os.path.join(os.path.dirname(__file__), '../urdf/mserve.urdf')
    assert os.path.exists(path)
