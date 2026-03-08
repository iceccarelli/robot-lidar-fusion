"""Tests for robot_hw.core.concurrency_management."""

from __future__ import annotations


def test_concurrency_manager_creation():
    """ConcurrencyManager should instantiate without error."""
    from robot_hw.core.concurrency_management import ConcurrencyManager

    cm = ConcurrencyManager()
    assert cm is not None


def test_acquire_and_release():
    """acquire() should return a context manager that releases on exit."""
    from robot_hw.core.concurrency_management import ConcurrencyManager

    cm = ConcurrencyManager()
    with cm.acquire("hardware"):
        pass  # Should not raise
