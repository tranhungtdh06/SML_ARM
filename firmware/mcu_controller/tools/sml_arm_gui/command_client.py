"""Firmware text protocol: commands are joint-indexed (1..6)."""
from dataclasses import dataclass


@dataclass
class JointState:
    available: bool = False
    running: bool = False
    position_valid: bool = False
    angle: float | None = None


class CommandClient:
    def __init__(self, transport):
        self.transport = transport

    def send(self, verb, joint=1, *values):
        parts = [verb.upper(), str(joint)] + [str(value) for value in values]
        return self.transport.send(' '.join(parts))

    def ping(self):
        return self.transport.send('PING')

    def stop_all(self, hard=False):
        return self.transport.send('ESTOP ALL' if hard else 'STOP ALL')

    @staticmethod
    def parse_fields(line):
        fields = {}
        for token in line.split():
            if '=' in token:
                key, value = token.split('=', 1)
                fields[key] = value
        return fields
