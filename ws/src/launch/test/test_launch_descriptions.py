import pytest
from launch import LaunchDescription


def test_launch_description():
    assert isinstance(LaunchDescription(), LaunchDescription)
