"""
Concurrency Management Module
=============================

This module provides a simple concurrency manager for coordinating
access to shared resources.  Real‑time control systems must avoid
deadlocks and race conditions when multiple threads or coroutines
operate concurrently.  The `ConcurrencyManager` centralises lock
creation and acquisition, making it easy for modules to coordinate
without accidentally holding multiple locks in inconsistent orders.

The manager exposes two primary interfaces:

* :meth:`get_lock` returns a `threading.Lock` associated with a
  particular resource name.
* :meth:`acquire` is a context manager that acquires and releases the
  lock automatically.

Locks are created lazily on first use.  Each lock is re‑entrant by
design of Python's `threading.Lock`, but it is good practice not to
acquire the same lock twice in the same thread to avoid deadlocks.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager, suppress
from threading import Lock


class ConcurrencyManager:
    """Manage locks for shared resource access.

    The manager stores a mapping from string names to lock objects.
    Names should correspond to the resources being protected (e.g.,
    "battery", "memory", "hardware").  Lock acquisition order should
    be consistent across modules to avoid deadlocks.
    """

    def __init__(self) -> None:
        self._locks: dict[str, Lock] = {}

    def get_lock(self, name: str) -> Lock:
        """Return (and create if necessary) a lock for the given name.

        Locks are stored in a dictionary keyed by resource name.  All
        locks are simple mutual exclusion locks (``threading.Lock``).
        Future extensions may wrap locks to include priority ceilings.

        Parameters
        ----------
        name : str
            Identifier of the resource to protect.

        Returns
        -------
        threading.Lock
            A lock instance associated with ``name``.
        """
        if name not in self._locks:
            self._locks[name] = Lock()
        return self._locks[name]

    @contextmanager
    def acquire(self, name: str, timeout: float | None = None) -> Iterator[None]:
        """Context manager to acquire and release a named lock with optional timeout.

        If ``timeout`` is provided, the manager will attempt to acquire the lock
        within the specified number of seconds.  If the lock cannot be
        acquired in time, a ``TimeoutError`` is raised.  Without a timeout,
        the call blocks until the lock is available.  Locks are always
        released upon exiting the context.

        Usage::

            with concurrency_manager.acquire("hardware", timeout=0.05):
                # perform hardware operations safely
                ...

        Parameters
        ----------
        name : str
            Name of the resource to lock.
        timeout : float, optional
            Maximum time to wait for the lock, in seconds.  If None,
            block indefinitely.

        Yields
        ------
        None
            Control is yielded once the lock has been acquired.

        Raises
        ------
        TimeoutError
            If the lock cannot be acquired within ``timeout`` seconds.
        """
        lock = self.get_lock(name)
        # Attempt to acquire with optional timeout
        if timeout is not None:
            acquired = lock.acquire(timeout=timeout)
            if not acquired:
                # Failed to acquire lock within timeout; raise
                raise TimeoutError(f"Timeout acquiring lock '{name}' after {timeout} s")
        else:
            lock.acquire()
        try:
            yield
        finally:
            # Only release if we actually hold the lock
            with suppress(RuntimeError):
                lock.release()

    @contextmanager
    def acquire_many(self, names: list[str], timeout: float | None = None) -> Iterator[None]:
        """Context manager to acquire multiple named locks with optional timeout.

        Locks are acquired in sorted name order to avoid deadlocks.  If a
        timeout is provided, it applies to acquiring each lock.  If any
        lock cannot be acquired within the timeout, all previously
        acquired locks are released and a ``TimeoutError`` is raised.

        Parameters
        ----------
        names : list[str]
            Names of resources whose locks should be acquired.
        timeout : float, optional
            Maximum time to wait for each lock.  If None, block indefinitely.

        Yields
        ------
        None
            Control is yielded once all locks have been acquired.

        Raises
        ------
        TimeoutError
            If any lock could not be acquired within the timeout.
        """
        unique_names = sorted(set(names))
        acquired_names: list[str] = []
        try:
            for name in unique_names:
                lock = self.get_lock(name)
                ok = lock.acquire(timeout=timeout) if timeout is not None else lock.acquire()
                if not ok:
                    # Failed to acquire; release previously acquired locks
                    for n in reversed(acquired_names):
                        with suppress(RuntimeError):
                            self._locks[n].release()
                    raise TimeoutError(f"Timeout acquiring lock '{name}' after {timeout} s")
                acquired_names.append(name)
            yield
        finally:
            # Release locks in reverse order
            for name in reversed(acquired_names):
                with suppress(RuntimeError):
                    self._locks[name].release()

    # ------------------------------------------------------------------
    # Priority management (ceiling mechanism)
    # ------------------------------------------------------------------
    def set_priority_ceiling(self, name: str, ceiling: int) -> None:
        """Associate a priority ceiling with a resource name.

        In a real‑time operating system, priority ceiling protocols
        prevent priority inversion by raising the priority of threads
        that acquire a lock.  This simplified implementation stores
        the ceiling for external reference; it does not modify
        thread priorities.

        Parameters
        ----------
        name : str
            Resource name for which to set the priority ceiling.
        ceiling : int
            The priority ceiling value (higher means more critical).
        """
        if not hasattr(self, '_priority_ceilings'):
            self._priority_ceilings: dict[str, int] = {}
        self._priority_ceilings[name] = int(ceiling)

    def get_priority_ceiling(self, name: str) -> int | None:
        """Retrieve the priority ceiling associated with a resource.

        Returns None if no ceiling has been set.

        Parameters
        ----------
        name : str
            Resource name.

        Returns
        -------
        int | None
            The priority ceiling value or None.
        """
        return getattr(self, '_priority_ceilings', {}).get(name)
