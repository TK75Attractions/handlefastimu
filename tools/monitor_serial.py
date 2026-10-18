#!/usr/bin/env python3
"""Capture a calibrated RGP/1 endurance run without resetting the board on exit.

Requires pyserial. All RX bytes, TX commands and host timestamps are retained.
The duration starts at the first TEL, after an explicit gyro calibration.
Keep the connected IMUs stationary during calibration.
"""
import argparse
import binascii
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import secrets
import statistics
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration", type=float, default=1800)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    started_at = datetime.now().astimezone().isoformat()
    counters = Counter()
    intervals = Counter()
    anomalies = []
    events = []
    statuses = []
    minutes = []
    angles = [[], []]
    window = [[], []]
    deltas = [0.0, 0.0]
    previous_angles = None
    first_angles = None
    last_angles = None
    last_seq = None
    last_time = None
    last_tel_time = None
    last_tel_host = None
    first_tel_host = None
    first_tel_time = None
    host_gaps_max = 0.0
    pending = {}
    tx_seq = 0
    last_status = None
    next_status = 0.0
    next_snapshot = 0.0
    next_minute = 60.0
    setup_phase = 0
    failure = None
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, 115200, 0.2
    port.dtr = False
    port.rts = False

    def elapsed():
        return time.monotonic() - started

    def anomaly(kind, **details):
        counters[kind] += 1
        item = dict(host_elapsed_s=round(elapsed(), 6), kind=kind, **details)
        if len(anomalies) < 500:
            anomalies.append(item)
        if counters[kind] <= 5:
            print(json.dumps(item), flush=True)

    def describe(values):
        if not values:
            return None
        return dict(min=min(values), max=max(values), mean=statistics.fmean(values),
                    stddev=statistics.pstdev(values), first=values[0], last=values[-1])

    def snapshot(done=False):
        duration = 0 if first_tel_host is None else time.monotonic() - first_tel_host
        data = dict(started_at=started_at, port=args.port, requested_duration_s=args.duration,
                    completed=done, failure=failure, elapsed_s=elapsed(), telemetry_duration_s=duration,
                    counters=dict(counters), firmware_intervals_ms=dict(sorted(intervals.items())),
                    max_host_gap_ms=host_gaps_max * 1000, first_tel_firmware_ms=first_tel_time,
                    last_tel_firmware_ms=last_tel_time, first_angles=first_angles, last_angles=last_angles,
                    angle_stats=[describe(a) for a in angles], max_angle_step_deg=deltas,
                    recent_angles=[describe(a) for a in window], minutes=minutes,
                    statuses=statuses, events=events, anomalies=anomalies,
                    pending_commands=pending)
        temporary = args.output / "summary.tmp"
        temporary.write_text(json.dumps(data, indent=2) + "\n")
        temporary.replace(args.output / "summary.json")
        print(json.dumps({k: data[k] for k in ("completed", "elapsed_s", "telemetry_duration_s",
                                              "counters", "last_angles", "recent_angles")}), flush=True)

    try:
        port.open()
        with (args.output / "serial.log").open("wb") as raw, (args.output / "frames.jsonl").open("w") as frames:
            def send(command):
                nonlocal tx_seq
                body = f"CMD,{tx_seq},0,{command}".encode("ascii")
                wire = b"@" + body + f"*{binascii.crc_hqx(body, 0xffff):04X}\n".encode("ascii")
                pending[tx_seq] = dict(command=command, sent_at=elapsed(), retries=0, wire=wire.decode("ascii"))
                port.write(wire)
                frames.write(json.dumps(dict(host_elapsed_s=elapsed(), direction="TX", wire=wire.decode("ascii"))) + "\n")
                tx_seq += 1

            while True:
                now = time.monotonic()
                age = now - started
                if (args.output / "STOP").exists():
                    raise RuntimeError("Stopped for diagnosis or firmware update")
                if first_tel_host is not None and now - first_tel_host >= args.duration:
                    break
                if first_tel_host is None and age > 90:
                    raise RuntimeError("No TEL within 90 seconds of startup/calibration")
                if setup_phase == 0 and age >= 2:
                    send("OPEN_SESSION," + secrets.token_hex(8).upper())
                    setup_phase = 1
                elif setup_phase == 1 and not pending:
                    send("GET_STATUS")
                    setup_phase = 2
                elif setup_phase == 2 and not pending:
                    if last_status and last_status[0] in ("WAIT_ANGLE", "READY"):
                        send("CALIBRATE,GYRO")
                        setup_phase = 3
                    elif age > next_status:
                        send("GET_STATUS")
                        next_status = age + 2
                elif setup_phase == 3 and first_tel_host is not None and age >= next_status:
                    send("GET_STATUS")
                    next_status = age + 30
                for seq, request in list(pending.items()):
                    if age - request["sent_at"] >= 1:
                        if request["retries"] >= 2:
                            anomaly("ack_timeout", seq=seq, command=request["command"])
                            raise RuntimeError("Command acknowledgement timeout")
                        port.write(request["wire"].encode("ascii"))
                        request["retries"] += 1
                        request["sent_at"] = age
                        counters["command_retries"] += 1
                wire = port.readline()
                if wire:
                    raw.write(wire)
                    frames.write(json.dumps(dict(host_elapsed_s=elapsed(), direction="RX",
                                                 wire=wire.decode("ascii", "backslashreplace"))) + "\n")
                    if not wire.startswith(b"@"):
                        counters["non_protocol_lines"] += 1
                        text = wire.decode("ascii", "backslashreplace").strip()
                        if first_tel_host is None and ("[E]" not in text and "Error" not in text):
                            counters["startup_output_lines"] += 1
                        else:
                            anomaly("non_protocol_output", text=text)
                        if age >= next_snapshot:
                            snapshot()
                            next_snapshot = age + 30
                        continue
                    try:
                        body, crc = wire.rstrip(b"\r\n")[1:].rsplit(b"*", 1)
                        if len(crc) != 4 or binascii.crc_hqx(body, 0xffff) != int(crc, 16):
                            raise ValueError("CRC mismatch")
                        fields = body.decode("ascii").split(",")
                        seq, stamp = int(fields[1]), int(fields[2])
                    except (ValueError, IndexError, UnicodeError) as exc:
                        anomaly("invalid_frame", error=str(exc))
                        continue
                    counters[fields[0]] += 1
                    if last_seq is not None and seq != (last_seq + 1) % 2**32:
                        anomaly("sequence_discontinuity", previous=last_seq, current=seq)
                    if last_time is not None and (stamp - last_time) % 2**32 > 2**31:
                        anomaly("firmware_clock_backwards", previous=last_time, current=stamp)
                    last_seq, last_time = seq, stamp
                    if fields[0] == "ACK":
                        pending.pop(int(fields[3]), None)
                        if fields[5] not in ("OK", "ACCEPTED"):
                            anomaly("command_rejected", fields=fields[3:])
                        if fields[4] == "GET_STATUS" and fields[5] == "OK":
                            last_status = fields[6:]
                            statuses.append(dict(host_elapsed_s=age, values=last_status))
                            if first_tel_host is not None and last_status != ["READY", "0", "1", "1", "1"]:
                                anomaly("unexpected_status", values=last_status)
                    elif fields[0] in ("ERR", "EVT", "SYS"):
                        events.append(dict(host_elapsed_s=age, fields=fields))
                        print(body.decode("ascii"), flush=True)
                        if fields[0] == "ERR":
                            anomaly("firmware_error", fields=fields)
                        if first_tel_host is not None and fields[3:] == ["HELLO", "RGP", "1.0", "1.0.0"]:
                            anomaly("unexpected_reboot")
                        if first_tel_host is not None and fields[3] == "STATE" and fields[4] != "READY":
                            anomaly("left_ready", state=fields[4])
                    elif fields[0] == "TEL":
                        try:
                            values = list(map(float, fields[3:]))
                            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                                raise ValueError("TEL requires six finite values")
                            if any(abs(values[i]) > 3600 for i in (0, 3)):
                                raise ValueError("angle outside supported range")
                        except ValueError as exc:
                            anomaly("invalid_tel", error=str(exc))
                            continue
                        if any(values[i] != 0 for i in (1, 2, 4, 5)):
                            anomaly("nonzero_unconnected_pedal", values=values)
                        current = [values[0], values[3]]
                        if first_tel_host is None:
                            first_tel_host, first_tel_time, first_angles = now, stamp, current
                        if last_tel_time is not None:
                            dt = (stamp - last_tel_time) % 2**32
                            intervals[dt] += 1
                            host_gaps_max = max(host_gaps_max, now - last_tel_host)
                            if dt > 30:
                                anomaly("tel_gap_over_30ms", firmware_interval_ms=dt)
                            if dt < 5:
                                anomaly("tel_burst_under_5ms", firmware_interval_ms=dt)
                            for i in range(2):
                                step = abs(current[i] - previous_angles[i])
                                deltas[i] = max(deltas[i], step)
                                if step > 5:
                                    anomaly("angle_step_over_5deg", player=i + 1, step_deg=step)
                        last_tel_time, last_tel_host = stamp, now
                        last_angles = previous_angles = current
                        for i in range(2):
                            angles[i].append(current[i])
                            window[i].append(current[i])
                if first_tel_host is not None and now - last_tel_host > 1:
                    raise RuntimeError("TEL stalled for over one second")
                if first_tel_host is not None and now - first_tel_host >= next_minute:
                    minutes.append(dict(telemetry_elapsed_s=now - first_tel_host,
                                        angles=[describe(a) for a in window], tel_count=counters["TEL"]))
                    window = [[], []]
                    next_minute += 60
                if age >= next_snapshot:
                    raw.flush()
                    frames.flush()
                    snapshot()
                    next_snapshot = age + 30
    except (Exception, KeyboardInterrupt) as exc:
        failure = f"{type(exc).__name__}: {exc}"
        print(failure, flush=True)
    finally:
        port.close()
        snapshot(done=failure is None and first_tel_host is not None)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
