#!/usr/bin/env python3
"""GoToBlock state machine.

Kept independent of rclpy so it can be unit tested: it only maps the current state
plus an event to the next state.

LOCALIZING is folded into SEARCHING while a goal runs - the goal is allowed to leave
as soon as the target colour is valid and /robot_pose has been received. The
transition stays in the table because it describes the intended sequence, and the
action server walks it explicitly before navigating.
"""

TRANSITIONS = {
    'IDLE':        {'start': 'SEARCHING'},
    'SEARCHING':   {'known': 'LOCALIZING', 'timeout': 'ABORTED'},
    'LOCALIZING':  {'ok': 'NAVIGATING',    'timeout': 'ABORTED'},
    'NAVIGATING':  {'near': 'APPROACHING', 'blocked': 'NAVIGATING'},
    'APPROACHING': {'arrived': 'DONE',     'timeout': 'ABORTED'},
    'DONE': {}, 'ABORTED': {},
}

# State strings reported to the client. Consumers recognise only these values, so
# LOCALIZING is never published and maps back to SEARCHING.
FEEDBACK_STATES = {
    'IDLE': 'SEARCHING',
    'SEARCHING': 'SEARCHING',
    'LOCALIZING': 'SEARCHING',
    'NAVIGATING': 'NAVIGATING',
    'APPROACHING': 'APPROACHING',
    'DONE': 'DONE',
    'ABORTED': 'ABORTED',
}


class InvalidTransition(Exception):
    """Raised when the current state does not accept the event."""


class StateMachine:
    """One instance per GoToBlock goal; no global state, so instances are independent."""

    def __init__(self, initial='IDLE'):
        if initial not in TRANSITIONS:
            raise ValueError(f'未知初始状态: {initial}')
        self.state = initial

    def can(self, event):
        return event in TRANSITIONS[self.state]

    def on(self, event):
        """Advance the machine; an illegal event raises, since it means the caller is wrong."""
        nxt = TRANSITIONS[self.state].get(event)
        if nxt is None:
            raise InvalidTransition(f'{self.state} 不接受事件 {event}')
        self.state = nxt
        return nxt

    def is_terminal(self):
        return not TRANSITIONS[self.state]

    def feedback_state(self):
        return FEEDBACK_STATES.get(self.state, self.state)
