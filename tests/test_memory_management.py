"""Tests for robot_hw.core.memory_management."""

from __future__ import annotations

import pytest

from robot_hw.core.memory_management import MemoryManager


def test_allocate_and_release():
    """Allocating and releasing memory should work without error."""
    mm = MemoryManager(total_bytes=4096)
    handle = mm.allocate(1024)
    assert handle is not None
    mm.release(handle)


def test_allocate_exceeds_capacity():
    """Allocating more than available should raise MemoryError."""
    mm = MemoryManager(total_bytes=512)
    with pytest.raises(MemoryError):
        mm.allocate(1024)


def test_check_health():
    """check_health() should return True for a healthy manager."""
    mm = MemoryManager(total_bytes=4096)
    assert mm.check_health() is True
