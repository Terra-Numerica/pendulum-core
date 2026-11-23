#!/usr/bin/env python3
import time

import zmq


class PendulumPyClient:
    """Bridge to the low-level interface.

    Communicates with the low-level interface using two TCP sockets (one to
    receive the current state of the pendulum, one to send actuation commands).
    Mirrors the C++ `PendulumCppClient`.
    """

    DEFAULT_HOST = "localhost"
    DEFAULT_STATE_PORT = 10001
    DEFAULT_COMMAND_PORT = 10002

    # Number of state coordinates.
    N_STATES = 5

    def __init__(self, host=DEFAULT_HOST, state_port=DEFAULT_STATE_PORT, command_port=DEFAULT_COMMAND_PORT, wait=-1):
        """Initializes socket connections.

        Connections are established at `tcp://[host]:[port]`.

        :param host: host for the sockets.
        :param state_port: port of the socket used to read the current state of the pendulum.
        :param command_port: port of the socket used to send commands to the low-level interface.
        :param wait: time to wait for the interface, in seconds. If less than or equal to zero,
            wait indefinitely. If positive, check every second if a message is available, until
            `wait` seconds have elapsed. If no message is received within the allotted time, a
            `RuntimeError` is raised.
        """
        self._context = zmq.Context()
        # Socket to send commands to the low-level interface.
        self._command_pub = self._context.socket(zmq.PUB)
        self._command_pub.connect(f"tcp://{host}:{command_port}")
        # Socket to receive the current state from the low-level interface.
        self._state_sub = self._context.socket(zmq.SUB)
        self._state_sub.set(zmq.CONFLATE, 1)
        self._state_sub.connect(f"tcp://{host}:{state_port}")
        self._state_sub.subscribe("")

        self._time = None
        self._position = None
        self._angle = None
        self._linear_velocity = None
        self._angular_velocity = None

        # Wait for the low-level interface to be up and running.
        if wait <= 0:
            self.read_state(blocking=True)
        else:
            for _ in range(int(wait)):
                time.sleep(1)
                if self.read_state(blocking=False):
                    break
            else:
                self.close()
                raise RuntimeError(
                    "PendulumPyClient: failed to establish a connection with the low-level interface "
                    "within the allotted time."
                )

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        """Closes the sockets and terminates the ZeroMQ context."""
        self._context.destroy(linger=0)

    @property
    def time(self):
        """Current time of the pendulum."""
        return self._time

    @property
    def position(self):
        """Current position of the pendulum."""
        return self._position

    @property
    def angle(self):
        """Current angle of the pendulum."""
        return self._angle

    @property
    def linear_velocity(self):
        """Current linear velocity of the pendulum."""
        return self._linear_velocity

    @property
    def angular_velocity(self):
        """Current angular velocity of the pendulum."""
        return self._angular_velocity

    def read_state(self, blocking=True):
        """Tries to read the state of the pendulum from the interface.

        While `blocking` is `True` by default, it is recommended to always pass it as a keyword
        argument, *e.g.* `p.read_state(blocking=True)`, so that it is clear whether the call is
        blocking or not.

        :param blocking: if `True`, do not return until a message has been received. If `False`,
            return immediately if no message is available.
        :return: `True` if a message has been received and processed. If the call is blocking,
            it always returns `True`.
        """
        if blocking:
            msg = self._state_sub.recv_string()
        else:
            try:
                msg = self._state_sub.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                return False

        # Each part of the message should be a float.
        parts = msg.split(" ")
        if len(parts) != self.N_STATES:
            raise RuntimeError(
                f"Malformed state message received. Expected {self.N_STATES} values, got {len(parts)}. "
                f"Message: {msg}"
            )
        self._time, self._position, self._angle, self._linear_velocity, self._angular_velocity = map(float, parts)
        return True

    def send_command(self, pwm):
        """Sends a PWM command to the low-level interface.

        :param pwm: the PWM signal to be sent, an integer between -255 and 255.
            It is cast to `int` before being sent.
        """
        self._command_pub.send_string(str(int(pwm)))
