"""
Universal BLE PID autotuner for STM32 balance car.

Protocol:
  APID,seq=N,kp=X,ki=X,kd=X*CS
  SPID,seq=N,kp=X,ki=X,kd=X*CS
  TPID,seq=N,kp=X,ki=X,kd=X*CS

Telemetry:
  D,seq=N,asp=F,apv=F,au=F,spv=F,su=F,tpv=F,tu=F,t=N*CS
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import re
from dataclasses import asdict, dataclass, replace
from typing import Iterable

import numpy as np

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    raise SystemExit("Missing dependency: bleak. Install with: pip install bleak numpy")


DEFAULT_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
DEFAULT_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

TELEMETRY_RE = re.compile(
    r"D,seq=(\d+),asp=([-\d.]+),apv=([-\d.]+),au=([-\d.]+),"
    r"spv=([-\d.]+),su=([-\d.]+),tpv=([-\d.]+),tu=([-\d.]+),t=(\d+)(?:\*[0-9A-Fa-f]{2})?"
)


@dataclass(frozen=True)
class LoopPID:
    kp: float
    ki: float
    kd: float


@dataclass(frozen=True)
class TriplePID:
    angle: LoopPID
    speed: LoopPID
    turn: LoopPID


@dataclass(frozen=True)
class Candidate:
    round_name: str
    label: str
    pid: TriplePID


@dataclass
class EvalMetrics:
    sample_count: int
    angle_std: float
    angle_mean_err: float
    angle_output_jitter: float
    speed_mean: float
    speed_std: float
    turn_mean: float
    turn_std: float
    score: float


@dataclass
class EvalResult:
    candidate: Candidate
    metrics: EvalMetrics
    safe: bool
    note: str = ""


class BLELink:
    def __init__(
        self,
        device_name: str,
        service_uuid: str,
        char_uuid: str,
        scan_timeout: float,
        connect_timeout: float,
        chunk_size: int,
        chunk_delay: float,
        poll_interval: float,
    ):
        self.device_name = device_name
        self.service_uuid = service_uuid
        self.char_uuid = char_uuid
        self.scan_timeout = scan_timeout
        self.connect_timeout = connect_timeout
        self.chunk_size = chunk_size
        self.chunk_delay = chunk_delay
        self.poll_interval = poll_interval

        self.client: BleakClient | None = None
        self.frames: list[dict] = []
        self._rx_buffer = ""
        self._last_key: tuple[int, int] | None = None
        self._notify_enabled = False
        self._poll_task: asyncio.Task | None = None

    async def connect(self) -> None:
        print(f"[BLE] Scanning device by name: {self.device_name}", flush=True)
        device = await BleakScanner.find_device_by_name(self.device_name, timeout=self.scan_timeout)
        if device is None:
            raise RuntimeError(
                f"Device '{self.device_name}' not found. "
                "On Windows, forget/remove the device in Settings and retry."
            )

        print(f"[BLE] Found: {device.name} ({device.address})", flush=True)
        self.client = BleakClient(device, timeout=self.connect_timeout)
        await self.client.connect()
        print(f"[BLE] Connected: {self.client.is_connected}", flush=True)

        try:
            await self.client.start_notify(self.char_uuid, self._on_notify)
            self._notify_enabled = True
            print("[BLE] RX mode: notify", flush=True)
        except Exception as exc:
            self._notify_enabled = False
            print(f"[BLE] notify failed: {exc}", flush=True)
            print("[BLE] fallback: read_gatt_char polling", flush=True)
            self._poll_task = asyncio.create_task(self._poll_loop())

    async def close(self) -> None:
        if self._poll_task is not None:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except BaseException:
                pass
            self._poll_task = None

        if self.client is None:
            return

        if self._notify_enabled:
            try:
                await self.client.stop_notify(self.char_uuid)
            except Exception:
                pass

        try:
            await self.client.disconnect()
        except Exception:
            pass

    async def send_text(self, text: str) -> None:
        if self.client is None:
            raise RuntimeError("BLE not connected")
        payload = text.encode("ascii", errors="ignore")
        for i in range(0, len(payload), self.chunk_size):
            chunk = payload[i : i + self.chunk_size]
            # Write-with-response is slower but much more stable on HC-04BLE.
            await self.client.write_gatt_char(self.char_uuid, chunk, response=True)
            await asyncio.sleep(self.chunk_delay)

    async def send_pid_triplet(self, pid: TriplePID, seq: int) -> int:
        for prefix, loop in (("APID", pid.angle), ("SPID", pid.speed), ("TPID", pid.turn)):
            body = f"{prefix},seq={seq},kp={loop.kp:.4f},ki={loop.ki:.4f},kd={loop.kd:.4f}"
            frame = f"{body}*{xor_checksum(body)}\n"
            await self.send_text(frame)
            seq += 1
            await asyncio.sleep(0.03)
        return seq

    def frame_index(self) -> int:
        return len(self.frames)

    def frames_since(self, index: int) -> list[dict]:
        return self.frames[index:]

    def _on_notify(self, _sender: int, data: bytearray) -> None:
        self._feed_bytes(bytes(data))

    async def _poll_loop(self) -> None:
        assert self.client is not None
        while True:
            try:
                data = await self.client.read_gatt_char(self.char_uuid)
                if data:
                    self._feed_bytes(bytes(data))
            except asyncio.CancelledError:
                raise
            except Exception:
                pass
            await asyncio.sleep(self.poll_interval)

    def _feed_bytes(self, data: bytes) -> None:
        self._rx_buffer += data.decode("ascii", errors="ignore")
        while "\n" in self._rx_buffer:
            raw_line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            line = raw_line.strip()
            if line:
                self._parse_line(line)

    def _parse_line(self, line: str) -> None:
        # One UART line may contain auxiliary payload like:
        #   [plot,...]D,seq=...,asp=...*CS
        # so we extract D-frames with finditer instead of strict line match.
        matches = list(TELEMETRY_RE.finditer(line))
        if not matches:
            return
        for match in matches:
            frame = {
                "seq": int(match.group(1)),
                "asp": float(match.group(2)),
                "apv": float(match.group(3)),
                "au": float(match.group(4)),
                "spv": float(match.group(5)),
                "su": float(match.group(6)),
                "tpv": float(match.group(7)),
                "tu": float(match.group(8)),
                "t": int(match.group(9)),
            }
            key = (frame["seq"], frame["t"])
            if key == self._last_key:
                continue
            self._last_key = key
            self.frames.append(frame)
            if len(self.frames) > 60000:
                self.frames = self.frames[-40000:]


def xor_checksum(body: str) -> str:
    value = 0
    for ch in body:
        value ^= ord(ch)
    return f"{value:02X}"


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def clamp_loop(loop: LoopPID, limits: tuple[float, float, float, float, float, float]) -> LoopPID:
    kp_lo, kp_hi, ki_lo, ki_hi, kd_lo, kd_hi = limits
    return LoopPID(
        kp=clamp(loop.kp, kp_lo, kp_hi),
        ki=clamp(loop.ki, ki_lo, ki_hi),
        kd=clamp(loop.kd, kd_lo, kd_hi),
    )


def clamp_triplet(pid: TriplePID) -> TriplePID:
    angle_limits = (1.5, 5.0, 0.01, 0.3, 1.0, 5.0)
    speed_limits = (0.5, 3.0, 0.01, 0.1, 0.0, 0.0)
    turn_limits = (2.0, 8.0, 1.0, 5.0, 0.0, 0.0)
    return TriplePID(
        angle=clamp_loop(pid.angle, angle_limits),
        speed=clamp_loop(pid.speed, speed_limits),
        turn=clamp_loop(pid.turn, turn_limits),
    )


def compute_metrics(frames: list[dict]) -> EvalMetrics:
    if len(frames) < 15:
        return EvalMetrics(
            sample_count=len(frames),
            angle_std=math.inf,
            angle_mean_err=math.inf,
            angle_output_jitter=math.inf,
            speed_mean=math.inf,
            speed_std=math.inf,
            turn_mean=math.inf,
            turn_std=math.inf,
            score=math.inf,
        )

    asp = np.array([f["asp"] for f in frames], dtype=float)
    apv = np.array([f["apv"] for f in frames], dtype=float)
    au = np.array([f["au"] for f in frames], dtype=float)
    spv = np.array([f["spv"] for f in frames], dtype=float)
    tpv = np.array([f["tpv"] for f in frames], dtype=float)

    angle_std = float(np.std(apv))
    angle_mean_err = float(np.mean(np.abs(asp - apv)))
    angle_output_jitter = float(np.std(np.diff(au))) if len(au) > 1 else 0.0
    speed_mean = float(np.mean(spv))
    speed_std = float(np.std(spv))
    turn_mean = float(np.mean(tpv))
    turn_std = float(np.std(tpv))

    score = (
        angle_std * 3.0
        + angle_mean_err * 2.0
        + angle_output_jitter * 0.1
        + abs(speed_mean) * 1.0
        + speed_std * 0.5
        + abs(turn_mean) * 0.5
        + turn_std * 0.3
    )

    return EvalMetrics(
        sample_count=len(frames),
        angle_std=angle_std,
        angle_mean_err=angle_mean_err,
        angle_output_jitter=angle_output_jitter,
        speed_mean=speed_mean,
        speed_std=speed_std,
        turn_mean=turn_mean,
        turn_std=turn_std,
        score=float(score),
    )


def dedupe_candidates(candidates: Iterable[Candidate]) -> list[Candidate]:
    seen = set()
    result = []
    for c in candidates:
        key = (
            round(c.pid.angle.kp, 4),
            round(c.pid.angle.ki, 4),
            round(c.pid.angle.kd, 4),
            round(c.pid.speed.kp, 4),
            round(c.pid.speed.ki, 4),
            round(c.pid.speed.kd, 4),
            round(c.pid.turn.kp, 4),
            round(c.pid.turn.ki, 4),
            round(c.pid.turn.kd, 4),
        )
        if key in seen:
            continue
        seen.add(key)
        result.append(c)
    return result


def build_round1(base: TriplePID) -> list[Candidate]:
    b = base
    candidates = [
        Candidate("R1", "baseline", b),
        Candidate("R1", "angle_kp_down", replace(b, angle=replace(b.angle, kp=b.angle.kp * 0.8))),
        Candidate("R1", "angle_kp_up", replace(b, angle=replace(b.angle, kp=b.angle.kp * 1.2))),
        Candidate("R1", "angle_kd_down", replace(b, angle=replace(b.angle, kd=b.angle.kd * 0.8))),
        Candidate("R1", "angle_kd_up", replace(b, angle=replace(b.angle, kd=b.angle.kd * 1.2))),
        Candidate("R1", "angle_ki_down", replace(b, angle=replace(b.angle, ki=b.angle.ki * 0.7))),
        Candidate("R1", "angle_ki_up", replace(b, angle=replace(b.angle, ki=b.angle.ki * 1.3))),
        Candidate("R1", "speed_kp_down", replace(b, speed=replace(b.speed, kp=b.speed.kp * 0.8))),
        Candidate("R1", "speed_kp_up", replace(b, speed=replace(b.speed, kp=b.speed.kp * 1.2))),
        Candidate("R1", "speed_ki_down", replace(b, speed=replace(b.speed, ki=b.speed.ki * 0.7))),
        Candidate("R1", "speed_ki_up", replace(b, speed=replace(b.speed, ki=b.speed.ki * 1.3))),
        Candidate("R1", "turn_kp_down", replace(b, turn=replace(b.turn, kp=b.turn.kp * 0.8))),
        Candidate("R1", "turn_kp_up", replace(b, turn=replace(b.turn, kp=b.turn.kp * 1.2))),
        Candidate("R1", "turn_ki_down", replace(b, turn=replace(b.turn, ki=b.turn.ki * 0.8))),
        Candidate("R1", "turn_ki_up", replace(b, turn=replace(b.turn, ki=b.turn.ki * 1.2))),
    ]
    return dedupe_candidates(
        [Candidate(c.round_name, c.label, clamp_triplet(c.pid)) for c in candidates]
    )


def build_round2(best: TriplePID) -> list[Candidate]:
    b = best
    candidates = [
        Candidate("R2", "best_center", b),
        Candidate("R2", "angle_kp_plus10", replace(b, angle=replace(b.angle, kp=b.angle.kp * 1.10))),
        Candidate("R2", "angle_kp_minus10", replace(b, angle=replace(b.angle, kp=b.angle.kp * 0.90))),
        Candidate("R2", "angle_kd_plus10", replace(b, angle=replace(b.angle, kd=b.angle.kd * 1.10))),
        Candidate("R2", "angle_kd_minus10", replace(b, angle=replace(b.angle, kd=b.angle.kd * 0.90))),
        Candidate(
            "R2",
            "speed_pair_plus10",
            replace(b, speed=replace(b.speed, kp=b.speed.kp * 1.10, ki=b.speed.ki * 1.10)),
        ),
        Candidate(
            "R2",
            "speed_pair_minus10",
            replace(b, speed=replace(b.speed, kp=b.speed.kp * 0.90, ki=b.speed.ki * 0.90)),
        ),
        Candidate(
            "R2",
            "turn_pair_plus10",
            replace(b, turn=replace(b.turn, kp=b.turn.kp * 1.10, ki=b.turn.ki * 1.10)),
        ),
        Candidate(
            "R2",
            "turn_pair_minus10",
            replace(b, turn=replace(b.turn, kp=b.turn.kp * 0.90, ki=b.turn.ki * 0.90)),
        ),
        Candidate(
            "R2",
            "angle_ki_plus10_kp_minus5",
            replace(b, angle=replace(b.angle, ki=b.angle.ki * 1.10, kp=b.angle.kp * 0.95)),
        ),
    ]
    return dedupe_candidates(
        [Candidate(c.round_name, c.label, clamp_triplet(c.pid)) for c in candidates]
    )


async def evaluate_candidate(
    link: BLELink,
    candidate: Candidate,
    best_known: TriplePID,
    settle_sec: float,
    collect_sec: float,
    max_angle_std: float,
    seq: int,
) -> tuple[EvalResult, int]:
    print(
        f"[{candidate.round_name}] {candidate.label}: "
        f"A({candidate.pid.angle.kp:.3f},{candidate.pid.angle.ki:.3f},{candidate.pid.angle.kd:.3f}) "
        f"S({candidate.pid.speed.kp:.3f},{candidate.pid.speed.ki:.3f},{candidate.pid.speed.kd:.3f}) "
        f"T({candidate.pid.turn.kp:.3f},{candidate.pid.turn.ki:.3f},{candidate.pid.turn.kd:.3f})",
        flush=True,
    )

    seq = await link.send_pid_triplet(candidate.pid, seq)
    await asyncio.sleep(settle_sec)
    start_index = link.frame_index()
    await asyncio.sleep(collect_sec)
    frames = link.frames_since(start_index)

    metrics = compute_metrics(frames)
    safe = metrics.angle_std <= max_angle_std and math.isfinite(metrics.score)
    note = ""

    if not safe:
        note = (
            "safety rollback: angle_std exceeds threshold"
            if math.isfinite(metrics.angle_std)
            else "invalid sample window"
        )
        print(
            f"  -> unsafe ({note}), rollback to best-known PID",
            flush=True,
        )
        seq = await link.send_pid_triplet(best_known, seq)
        await asyncio.sleep(1.0)

    print(
        "  -> score={:.4f}, n={}, angle_std={:.4f}, angle_err={:.4f}, speed_std={:.4f}, turn_std={:.4f}".format(
            metrics.score,
            metrics.sample_count,
            metrics.angle_std,
            metrics.angle_mean_err,
            metrics.speed_std,
            metrics.turn_std,
        ),
        flush=True,
    )

    return EvalResult(candidate=candidate, metrics=metrics, safe=safe, note=note), seq


def print_ranking(results: list[EvalResult]) -> None:
    ranked = sorted(results, key=lambda r: r.metrics.score)
    print("\n=== Candidate Ranking (lower score is better) ===", flush=True)
    print(
        "Rank  Round Label                    Score     Safe  N    AngleStd  AngleErr  SpeedStd  TurnStd",
        flush=True,
    )
    for idx, res in enumerate(ranked, start=1):
        print(
            "{:<5} {:<5} {:<24} {:>8.3f}  {:<5} {:>4} {:>8.3f} {:>8.3f} {:>8.3f} {:>8.3f}".format(
                idx,
                res.candidate.round_name,
                res.candidate.label[:24],
                res.metrics.score,
                "yes" if res.safe else "no",
                res.metrics.sample_count,
                res.metrics.angle_std,
                res.metrics.angle_mean_err,
                res.metrics.speed_std,
                res.metrics.turn_std,
            ),
            flush=True,
        )


def pick_best_safe(results: list[EvalResult], fallback: TriplePID) -> TriplePID:
    safe = [r for r in results if r.safe and math.isfinite(r.metrics.score)]
    if not safe:
        return fallback
    best = min(safe, key=lambda r: r.metrics.score)
    return best.candidate.pid


def suggestion(final_metrics: EvalMetrics) -> str:
    if final_metrics.sample_count < 30 or not math.isfinite(final_metrics.score):
        return "Data quality is too low. Improve telemetry continuity before further tuning."
    if final_metrics.angle_std <= 2.0 and final_metrics.angle_mean_err <= 1.0:
        return "Current tuning is usable. Only do minor fine-tuning if behavior still feels soft."
    if final_metrics.angle_std > 5.0:
        return "Angle loop is still noisy. Prioritize angle Kd increase or angle Kp reduction."
    if abs(final_metrics.speed_mean) > 1.5:
        return "Speed bias is visible. Re-center speed target and refine speed Ki."
    return "More tuning is recommended around the top 3 candidates (+/-5% local search)."


def serialize_pid(pid: TriplePID) -> dict:
    def r(v: float) -> float:
        return round(float(v), 4)

    return {
        "angle": {"kp": r(pid.angle.kp), "ki": r(pid.angle.ki), "kd": r(pid.angle.kd)},
        "speed": {"kp": r(pid.speed.kp), "ki": r(pid.speed.ki), "kd": r(pid.speed.kd)},
        "turn": {"kp": r(pid.turn.kp), "ki": r(pid.turn.ki), "kd": r(pid.turn.kd)},
    }


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="BLE universal PID autotuner for balance car")
    p.add_argument("--device-name", default="HC-04BLE")
    p.add_argument("--service-uuid", default=DEFAULT_SERVICE_UUID)
    p.add_argument("--char-uuid", default=DEFAULT_CHAR_UUID)
    p.add_argument("--scan-timeout", type=float, default=10.0)
    p.add_argument("--connect-timeout", type=float, default=15.0)
    p.add_argument("--settle-sec", type=float, default=2.0)
    p.add_argument("--collect-sec", type=float, default=5.0)
    p.add_argument("--max-angle-std", type=float, default=10.0)
    p.add_argument("--chunk-size", type=int, default=20)
    p.add_argument("--chunk-delay", type=float, default=0.03)
    p.add_argument("--poll-interval", type=float, default=0.08)
    p.add_argument("--output-json", default="")
    return p


async def run(args: argparse.Namespace) -> int:
    base = TriplePID(
        angle=LoopPID(2.7, 0.1, 2.5),
        speed=LoopPID(1.5, 0.03, 0.0),
        turn=LoopPID(4.0, 3.0, 0.0),
    )
    base = clamp_triplet(base)

    link = BLELink(
        device_name=args.device_name,
        service_uuid=args.service_uuid,
        char_uuid=args.char_uuid,
        scan_timeout=args.scan_timeout,
        connect_timeout=args.connect_timeout,
        chunk_size=args.chunk_size,
        chunk_delay=args.chunk_delay,
        poll_interval=args.poll_interval,
    )

    all_results: list[EvalResult] = []
    seq = 1
    terminated_early = False
    terminate_reason = ""

    try:
        await link.connect()

        print("\n[Init] Applying base PID and warming up...", flush=True)
        seq = await link.send_pid_triplet(base, seq)
        await asyncio.sleep(1.0)

        round1 = build_round1(base)
        print(f"\n[Round-1] Broad search candidates: {len(round1)}", flush=True)
        best_known = base
        for idx, cand in enumerate(round1, start=1):
            print(f"[Round-1] Progress {idx}/{len(round1)}", flush=True)
            try:
                result, seq = await evaluate_candidate(
                    link=link,
                    candidate=cand,
                    best_known=best_known,
                    settle_sec=args.settle_sec,
                    collect_sec=args.collect_sec,
                    max_angle_std=args.max_angle_std,
                    seq=seq,
                )
            except Exception as exc:
                terminated_early = True
                terminate_reason = f"Round-1 stopped at {cand.label}: {exc}"
                print(f"[Round-1] ERROR: {terminate_reason}", flush=True)
                break
            all_results.append(result)
            if result.safe:
                best_known = pick_best_safe(all_results, best_known)

        best_after_r1 = pick_best_safe(all_results, base)
        print("\n[Round-1] Best candidate selected for local search.", flush=True)

        round2 = build_round2(best_after_r1)
        print(f"\n[Round-2] Fine search candidates: {len(round2)}", flush=True)
        best_known = best_after_r1
        if not terminated_early:
            for idx, cand in enumerate(round2, start=1):
                print(f"[Round-2] Progress {idx}/{len(round2)}", flush=True)
                try:
                    result, seq = await evaluate_candidate(
                        link=link,
                        candidate=cand,
                        best_known=best_known,
                        settle_sec=args.settle_sec,
                        collect_sec=args.collect_sec,
                        max_angle_std=args.max_angle_std,
                        seq=seq,
                    )
                except Exception as exc:
                    terminated_early = True
                    terminate_reason = f"Round-2 stopped at {cand.label}: {exc}"
                    print(f"[Round-2] ERROR: {terminate_reason}", flush=True)
                    break
                all_results.append(result)
                if result.safe:
                    best_known = pick_best_safe(all_results, best_known)

        best_pid = pick_best_safe(all_results, base)
        print("\n[Final] Validate best PID once more...", flush=True)
        final_candidate = Candidate("FINAL", "best_validation", best_pid)
        try:
            final_result, seq = await evaluate_candidate(
                link=link,
                candidate=final_candidate,
                best_known=best_pid,
                settle_sec=args.settle_sec,
                collect_sec=args.collect_sec,
                max_angle_std=args.max_angle_std,
                seq=seq,
            )
        except Exception as exc:
            terminated_early = True
            if not terminate_reason:
                terminate_reason = f"Final validation failed: {exc}"
            print(f"[Final] ERROR: {exc}", flush=True)
            final_result = EvalResult(
                candidate=final_candidate,
                metrics=EvalMetrics(
                    sample_count=0,
                    angle_std=math.inf,
                    angle_mean_err=math.inf,
                    angle_output_jitter=math.inf,
                    speed_mean=math.inf,
                    speed_std=math.inf,
                    turn_mean=math.inf,
                    turn_std=math.inf,
                    score=math.inf,
                ),
                safe=False,
                note=str(exc),
            )

        print_ranking(all_results + [final_result])
        final_metrics = final_result.metrics

        output = {
            "best_pid": serialize_pid(best_pid),
            "final_score": final_metrics.score,
            "final_metrics": asdict(final_metrics),
            "ranking": [
                {
                    "round": r.candidate.round_name,
                    "label": r.candidate.label,
                    "pid": serialize_pid(r.candidate.pid),
                    "metrics": asdict(r.metrics),
                    "safe": r.safe,
                    "note": r.note,
                }
                for r in sorted(all_results + [final_result], key=lambda x: x.metrics.score)
            ],
            "suggestion": suggestion(final_metrics),
            "terminated_early": terminated_early,
            "terminate_reason": terminate_reason,
        }

        print("\n=== Best PID ===", flush=True)
        print(json.dumps(output["best_pid"], ensure_ascii=False, indent=2), flush=True)
        print(f"Final score: {output['final_score']:.4f}", flush=True)
        print(f"Suggestion: {output['suggestion']}", flush=True)

        if args.output_json:
            with open(args.output_json, "w", encoding="utf-8") as f:
                json.dump(output, f, ensure_ascii=False, indent=2)
            print(f"[Output] Saved: {args.output_json}", flush=True)

        return 0
    finally:
        await link.close()


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
