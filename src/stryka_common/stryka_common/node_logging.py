"""STRYKA node logging / observability standard.

Every STRYKA node follows the same pattern so a running system can be read from
its logs and topics alone:

  * a heartbeat message published at a fixed rate carrying node name, state and
    uptime (``std_msgs/String`` on ``~/heartbeat``);
  * the current state logged at a fixed rate -- not only when it changes, so a
    stuck node is visible in a tail of the log;
  * every state transition logged with the trigger that caused it.

``NodeMonitor`` wires all three into an existing rclpy node. One line in a
node's ``__init__`` makes the node compliant; call :meth:`record_transition`
from the one place the node changes state.

This lives in ``stryka_common`` rather than in any single node package because
every future node (perception, tracking, guidance) imports it, and none of
those should have to depend on ``stryka_mission`` to get it.
"""

from __future__ import annotations

import time
from typing import Callable

from rclpy.node import Node
from std_msgs.msg import String

HEARTBEAT_TOPIC: str = "~/heartbeat"

DEFAULT_HEARTBEAT_HZ: float = 1.0
DEFAULT_STATE_LOG_HZ: float = 0.5


class NodeMonitor:
    """Attach the STRYKA heartbeat + state-logging standard to a node.

    Args:
        node: the rclpy node to instrument.
        state_fn: zero-argument callable returning the node's current state as
            a short string (e.g. the name of a state-machine member).
        heartbeat_hz: rate for the ``~/heartbeat`` publication.
        state_log_hz: rate at which the current state is written to the logger.
    """

    def __init__(
        self,
        node: Node,
        state_fn: Callable[[], str],
        *,
        heartbeat_hz: float = DEFAULT_HEARTBEAT_HZ,
        state_log_hz: float = DEFAULT_STATE_LOG_HZ,
    ) -> None:
        self._node = node
        self._state_fn = state_fn
        self._start = time.monotonic()

        self._heartbeat_pub = node.create_publisher(String, HEARTBEAT_TOPIC, 10)
        self._heartbeat_timer = node.create_timer(
            1.0 / heartbeat_hz, self._publish_heartbeat
        )
        self._state_log_timer = node.create_timer(
            1.0 / state_log_hz, self._log_state
        )

    @property
    def uptime_s(self) -> float:
        """Seconds since this monitor was constructed."""
        return time.monotonic() - self._start

    def _publish_heartbeat(self) -> None:
        msg = String()
        msg.data = (
            f"node={self._node.get_name()} "
            f"state={self._state_fn()} "
            f"uptime={self.uptime_s:.1f}s"
        )
        self._heartbeat_pub.publish(msg)

    def _log_state(self) -> None:
        self._node.get_logger().info(
            f"[{self._state_fn()}] uptime={self.uptime_s:.1f}s"
        )

    def record_transition(self, old: str, new: str, reason: str) -> None:
        """Log a state transition with the trigger that caused it."""
        self._node.get_logger().info(f"STATE {old} -> {new}  ({reason})")
