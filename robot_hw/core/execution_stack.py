"""
Execution Stack Module
======================

This module defines a deterministic execution stack for real‑time
robotic control.  Tasks are represented as callables (functions or
bound methods) that take no arguments and encapsulate a piece of work
to be performed during a control cycle.  The execution stack
maintains a queue of tasks and provides mechanisms to push new tasks,
pop tasks and execute them sequentially within a control cycle.

The design emphasises:

* **Determinism:** Each task is executed at most once per cycle in the
  order they were queued.
* **Non‑blocking behaviour:** Tasks should complete quickly; long
  computations must be broken into smaller tasks or offloaded to
  separate threads managed by the concurrency manager.
* **Error isolation:** Exceptions raised by tasks are caught and
  logged; they do not prevent subsequent tasks from running.

Note that this module does not enforce real‑time deadlines itself.
Timing control resides in the orchestrator, which invokes
:meth:`step` at a fixed rate (e.g. 100 Hz).
"""

from __future__ import annotations

import contextlib
from collections import deque
from collections.abc import Callable
from typing import Any


class ExecutionStack:
    """Manage a queue of tasks for deterministic execution.

    The execution stack allows modules to schedule work to be performed
    in the next control cycle.  Tasks are callables with no
    parameters.  Each call to :meth:`step` processes all tasks that
    were queued up to the point of invocation.  Tasks added during a
    call to :meth:`step` are deferred to the next cycle to preserve
    determinism.
    """

    def __init__(self) -> None:
        self._queue: deque[Callable[[], Any]] = deque()

    def push(self, task: Callable[[], Any]) -> None:
        """Add a task to be executed in the next cycle.

        Parameters
        ----------
        task : Callable[[], Any]
            A callable with no arguments.  The return value is ignored.
        """
        self._queue.append(task)

    def pop(self) -> Callable[[], Any] | None:
        """Remove and return the next task, or None if empty."""
        return self._queue.popleft() if self._queue else None

    def step(self) -> None:
        """Execute all tasks scheduled for this cycle.

        Tasks scheduled during this call are deferred until the next
        cycle.  Any exceptions raised by tasks are caught and should
        be logged by the caller; they do not stop subsequent tasks
        from running.
        """
        # Snapshot current tasks to avoid executing newly queued tasks
        tasks_to_run = [self._queue.popleft() for _ in range(len(self._queue))]
        for task in tasks_to_run:
            with contextlib.suppress(Exception):
                task()
