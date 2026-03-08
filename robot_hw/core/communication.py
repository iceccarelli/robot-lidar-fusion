"""Communication Interface Module
===============================

This module defines the :class:`CommunicationInterface`, which
facilitates data exchange between the robot and external systems.  It
provides methods for sending telemetry (metrics, logs, events) and
receiving commands or configuration updates from operators or remote
mission control.  The interface abstracts over specific communication
protocols (Ethernet, serial, wireless) and formats (JSON, protobuf),
allowing them to be swapped or extended as needed.

In stage A this class is a skeleton; future stages will add actual
transport mechanisms, message definitions and error handling.
"""

from __future__ import annotations

from typing import Any

from robot_hw.robot_config import RobotConfig


class CommunicationInterface:
    """Handle telemetry and teleoperation communication.

    The communication interface abstracts over underlying transport
    mechanisms (e.g. TCP/IP, serial, CAN bus) to provide simple APIs
    for sending telemetry and receiving operator commands.  In this
    reference implementation, telemetry is stored in an internal log
    and commands are queued for retrieval by the orchestrator.  The
    interface can be extended to integrate with real networking stacks
    in future stages.

    Parameters
    ----------
    config : RobotConfig
        System configuration providing keys and endpoints for
        telemetry and remote control.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        # Placeholder for connection objects (sockets, serial ports, etc.)
        self._connection: Any | None = None
        # Store sent telemetry messages for inspection; in real use,
        # messages would be transmitted to a remote endpoint
        self._telemetry_log: list[dict[str, Any]] = []
        # Queue of incoming commands from remote systems.  Each entry
        # is a dictionary representing a command or configuration update.
        self._incoming_commands: list[dict[str, Any]] = []

    def connect(self) -> None:
        """Establish a connection to the remote endpoint.

        In this minimal implementation the method sets an internal
        connection flag.  In a real implementation it would open
        network sockets or serial ports using credentials and
        configuration keys provided via :class:`RobotConfig`.
        """
        self._connection = True

    def send_telemetry(self, data: dict[str, Any]) -> None:
        """Send telemetry data to the remote endpoint.

        Telemetry messages are appended to an internal log.  The
        orchestrator calls this method each cycle with metrics such as
        battery level, thermal state and hazard flags.  In future
        implementations this method should serialise the data into
        JSON or another format and transmit it over the configured
        transport.

        Parameters
        ----------
        data : dict[str, Any]
            Arbitrary dictionary containing metrics, state and event
            information to be transmitted.
        """
        if not isinstance(data, dict):
            return
        # Append to internal log
        self._telemetry_log.append(dict(data))

    def receive_commands(self) -> dict[str, Any]:
        """Receive remote commands or configuration updates.

        Commands are returned and cleared from the internal queue.
        Each call returns a single command dictionary.  In a real
        implementation this would block on a network socket or
        message queue until data arrives.

        Returns
        -------
        dict[str, Any]
            A command or configuration update.  Returns an empty
            dictionary if no commands are queued.
        """
        if not self._incoming_commands:
            return {}
        return self._incoming_commands.pop(0)

    # ------------------------------------------------------------------
    # Teleoperation support
    # ------------------------------------------------------------------
    def queue_command(self, command: dict[str, Any]) -> None:
        """Enqueue a command from a remote operator.

        This helper can be used in tests or simulations to simulate
        incoming commands.  Each command should be a dictionary with
        well‑defined keys understood by the orchestrator (e.g.
        ``{"add_goal": {"x": 1.0, "y": 2.0}}``).

        Parameters
        ----------
        command : dict[str, Any]
            Command or configuration update to queue.
        """
        if isinstance(command, dict):
            self._incoming_commands.append(dict(command))
