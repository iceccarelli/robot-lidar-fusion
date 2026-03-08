"""
Memory Management Module
========================

The `MemoryManager` is responsible for handling memory buffers used
throughout the robot control system.  In contrast to typical Python
programs, a real‑time control system cannot rely on the garbage
collector to release memory deterministically.  The memory manager
pre‑allocates memory regions for sensor data, logging and internal
state and provides methods to allocate and release memory blocks on
request.

This module provides only an interface and high‑level structure; the
actual memory allocation (e.g. using `mmap`, `ctypes` or specialised
buffers) is platform dependent and should be implemented in the
execution environment.  Calls to allocate or release memory should
never block for an unbounded time.
"""

from __future__ import annotations

from typing import Any


class MemoryManager:
    """Deterministic memory allocator for real‑time systems.

    Parameters
    ----------
    total_bytes : int
        The total amount of memory (in bytes) that can be managed by
        this manager.  This defines the upper bound on all
        allocations.

    Notes
    -----
    In a real robot, the memory manager would allocate from a pool of
    pinned or real‑time capable memory regions.  Here we outline the
    interface; the implementation must ensure that allocations
    complete in deterministic time and that no memory leak can occur.
    """

    def __init__(self, total_bytes: int) -> None:
        self.total_bytes = max(total_bytes, 0)
        self.available_bytes = self.total_bytes
        # Internal bookkeeping for allocated blocks; the key is a
        # Python object returned by allocate(), value is the size of
        # allocation.  In a real implementation, the object might be
        # a memoryview or pointer.
        self._allocations: dict[Any, int] = {}

    def allocate(self, size: int) -> Any:
        """Reserve a block of memory and return a handle.

        Parameters
        ----------
        size : int
            Number of bytes to allocate.  Must be positive and
            available within the manager.

        Returns
        -------
        object
            A handle representing the allocated memory.  Clients
            should treat this as opaque and pass it back to
            :meth:`release` when done.

        Raises
        ------
        MemoryError
            If there is insufficient memory to satisfy the request.
        """
        if size <= 0:
            raise ValueError("Allocation size must be positive")
        if size > self.available_bytes:
            raise MemoryError("Insufficient memory available")
        # In a real system, allocate a memory region; here we return
        # an opaque object as a handle.  Using an object() ensures
        # uniqueness and hashability (unlike bytearray, which is
        # unhashable).  Clients should treat the handle as opaque and
        # not inspect its identity.
        handle = object()
        self._allocations[handle] = size
        self.available_bytes -= size
        return handle

    def release(self, handle: Any) -> None:
        """Release a previously allocated memory block.

        Parameters
        ----------
        handle : object
            The handle returned by :meth:`allocate`.

        Notes
        -----
        Double frees or releasing unknown handles are ignored.  In a
        production system, such errors should be logged and may
        indicate a bug.
        """
        size = self._allocations.pop(handle, None)
        if size is not None:
            self.available_bytes += size

    def check_health(self) -> bool:
        """Return True if memory usage is within safe limits.

        This helper can be invoked by the orchestrator to detect
        runaway memory consumption.  It is considered healthy if at
        least 10 % of the total memory remains free.  The threshold
        can be adjusted as needed.
        """
        if self.total_bytes == 0:
            return True
        free_ratio = self.available_bytes / self.total_bytes
        return free_ratio >= 0.1
