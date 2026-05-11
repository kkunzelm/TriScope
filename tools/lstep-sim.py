#!/usr/bin/env python3
"""Minimal LStep MCL3 simulator.

Versteht das U-Prefix-Protokoll soweit, dass die GUI ohne Hardware
durchgespielt werden kann. Bewusst einfach gehalten – keine Geometrie,
keine Endschalter-Logik, keine Geschwindigkeits-Simulation.

Aufruf:
    python3 lstep-sim.py /dev/pts/N
oder typischerweise via run-sim.sh.
"""

from __future__ import annotations

import os
import select
import sys
import time

CR = 0x0D
U  = 0x55

# Schreib-Register
REG_PRESEL_X     = 0
REG_PRESEL_Y     = 1
REG_PRESEL_Z     = 2
REG_COMMAND      = 7
REG_RAMP         = 8
REG_SPEED        = 9
REG_REDUCTION    = 10
REG_ACTIVE_AXES  = 11

# Lese-Bytes (Schreib-Reg + 64)
READ_POS_X       = 67   # 'C'
READ_POS_Y       = 68   # 'D'
READ_POS_Z       = 69   # 'E'
READ_STATUS      = 70   # 'F'
READ_ACTIVE_AXES = 75   # 'K'
READ_START       = 80   # 'P'


class FakeStage:
    def __init__(self, fd: int):
        self.fd = fd
        self.pos        = [0, 0, 0]              # interne Einheiten X,Y,Z
        self.preselect  = [0, 0, 0]
        self.active_axes = 7
        self.speed = 50
        self.ramp  = 500
        self.last_move: str | None = None

    # --- I/O Helfer -----------------------------------------------------

    def reply(self, text: str) -> None:
        data = text.encode('latin-1') + bytes([CR])
        os.write(self.fd, data)
        sys.stderr.write(f"  <- {text!r}\n")

    # --- Frame-Handler --------------------------------------------------

    def handle_u_frame(self, reg: int, payload: bytes) -> None:
        text = payload.decode('latin-1', errors='replace')
        sys.stderr.write(f"  -> U(reg={reg} 0x{reg:02X}) payload={text!r}\n")

        # Lese-Register
        if reg == READ_POS_X:
            self.reply(str(self.pos[0])); return
        if reg == READ_POS_Y:
            self.reply(str(self.pos[1])); return
        if reg == READ_POS_Z:
            self.reply(str(self.pos[2])); return
        if reg == READ_STATUS:
            self.reply("OK..."); return
        if reg == READ_ACTIVE_AXES:
            self.reply(str(self.active_axes)); return
        if reg == READ_START:
            self._execute_pending_move()
            return

        # Schreib-Register
        if reg == REG_PRESEL_X:
            self.preselect[0] = self._parse_int(text); return
        if reg == REG_PRESEL_Y:
            self.preselect[1] = self._parse_int(text); return
        if reg == REG_PRESEL_Z:
            self.preselect[2] = self._parse_int(text); return
        if reg == REG_COMMAND:
            # erstes Zeichen der Payload entscheidet die Bewegungsart
            self.last_move = text[:1] if text else None
            return
        if reg == REG_RAMP:
            self.ramp = self._parse_int(text); return
        if reg == REG_SPEED:
            self.speed = self._parse_int(text); return
        if reg == REG_ACTIVE_AXES:
            self.active_axes = self._parse_int(text) & 0x07; return
        # Init-Sets etc. – stille schlucken

    def handle_ascii_frame(self, frame: bytes) -> None:
        text = frame.decode('latin-1', errors='replace')
        sys.stderr.write(f"  -> ascii {text!r}\n")
        if text == "?ver":
            self.reply("Vers:LS31.00.011")
        # !cts 1 etc. – stille schlucken

    # --- Bewegungs-Simulation ------------------------------------------

    def _execute_pending_move(self) -> None:
        cmd = self.last_move
        self.last_move = None
        if cmd == 'c':
            time.sleep(0.3)
            self.pos = [0, 0, 0]
            self.reply("AAA-.")
            return
        if cmd == 'r':
            time.sleep(0.1)
            for ax in range(3):
                if self.active_axes & (1 << ax):
                    self.pos[ax] = self.preselect[ax]
            self.reply("@@@-.")
            return
        if cmd == 'v':
            time.sleep(0.1)
            for ax in range(3):
                if self.active_axes & (1 << ax):
                    self.pos[ax] += self.preselect[ax]
            self.reply("@@@-.")
            return
        if cmd == 'l':
            time.sleep(0.3)
            self.reply("DDD-.")
            return
        # Unbekannt -> trotzdem freundlich antworten
        self.reply("@@@-.")

    @staticmethod
    def _parse_int(text: str) -> int:
        try:
            return int(text)
        except ValueError:
            return 0


def reader_loop(stage: FakeStage) -> None:
    """Bytestrom-State-Machine: U,reg,payload,CR  oder  ascii...,CR."""
    state = 'idle'
    reg = 0
    payload = bytearray()
    ascii_buf = bytearray()

    while True:
        rlist, _, _ = select.select([stage.fd], [], [], 1.0)
        if not rlist:
            continue
        try:
            data = os.read(stage.fd, 256)
        except OSError:
            time.sleep(0.05)
            continue
        if not data:
            continue

        for b in data:
            if state == 'idle':
                if b == U:
                    state = 'u_reg'
                elif b == CR:
                    pass  # leerer Frame
                else:
                    state = 'ascii'
                    ascii_buf = bytearray([b])
            elif state == 'u_reg':
                reg = b
                payload = bytearray()
                state = 'u_payload'
            elif state == 'u_payload':
                if b == CR:
                    stage.handle_u_frame(reg, bytes(payload))
                    state = 'idle'
                else:
                    payload.append(b)
            elif state == 'ascii':
                if b == CR:
                    stage.handle_ascii_frame(bytes(ascii_buf))
                    state = 'idle'
                else:
                    ascii_buf.append(b)


def main() -> int:
    if len(sys.argv) != 2:
        sys.stderr.write("Aufruf: lstep-sim.py /dev/pts/N\n")
        return 2
    path = sys.argv[1]

    # Raw, non-blocking-fähig öffnen. socat hat den Pty bereits in raw mode versetzt.
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    sys.stderr.write(f"LStep-Simulator gebunden an {path}\n")

    stage = FakeStage(fd)
    try:
        reader_loop(stage)
    except KeyboardInterrupt:
        sys.stderr.write("\nSimulator beendet.\n")
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
