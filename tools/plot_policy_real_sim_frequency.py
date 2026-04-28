#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch


ACTION_JOINT_KEYS: List[str] = [
    "right.hipz",
    "right.hipx",
    "right.hipy",
    "right.knee",
    "right.ankle_pitch",
    "right.ankle_roll",
    "left.hipz",
    "left.hipx",
    "left.hipy",
    "left.knee",
    "left.ankle_pitch",
    "left.ankle_roll",
]


OBS_SLICES: Dict[str, slice] = {
    "projected_gravity": slice(0, 3),
    "joint_pos": slice(3, 15),
    "joint_vel": slice(15, 27),
    "actions_obs": slice(27, 39),
    "command": slice(39, 42),
}


@dataclass
class LogData:
    time_s: np.ndarray
    obs: np.ndarray
    action_pre_scale: np.ndarray
    fs_hz: float


def _load_debug_log(path: Path) -> LogData:
    time_list: List[float] = []
    obs_list: List[np.ndarray] = []
    action_list: List[np.ndarray] = []

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            obs_raw = row.get("observation", "")
            action_raw = row.get("action_pre_scale", "")
            if not obs_raw or not action_raw:
                continue
            try:
                obs_vec = np.asarray(json.loads(obs_raw), dtype=np.float64).reshape(-1)
                act_vec = np.asarray(json.loads(action_raw), dtype=np.float64).reshape(-1)
                t = float(row.get("time_s", "nan"))
            except Exception:
                continue
            if not np.isfinite(t):
                continue
            if obs_vec.size < 42 or act_vec.size < 12:
                continue
            time_list.append(t)
            obs_list.append(obs_vec[:42])
            action_list.append(act_vec[:12])

    if not time_list:
        raise RuntimeError(f"No valid rows found in {path}")

    time_s = np.asarray(time_list, dtype=np.float64)
    obs = np.asarray(obs_list, dtype=np.float64)
    action_pre_scale = np.asarray(action_list, dtype=np.float64)

    dt = np.diff(time_s)
    dt = dt[np.isfinite(dt) & (dt > 1e-6)]
    if dt.size == 0:
        raise RuntimeError(f"Could not estimate sampling time in {path}")
    fs_hz = float(1.0 / np.median(dt))

    return LogData(time_s=time_s, obs=obs, action_pre_scale=action_pre_scale, fs_hz=fs_hz)


def _activity_score(obs: np.ndarray, action_pre_scale: np.ndarray) -> np.ndarray:
    q = obs[:, OBS_SLICES["joint_pos"]]
    dq = np.linalg.norm(np.diff(q, axis=0), axis=1)
    da = np.linalg.norm(np.diff(action_pre_scale, axis=0), axis=1)
    score = dq + 0.4 * da
    score = np.r_[0.0, score]
    return score


def _longest_true_segment(mask: np.ndarray) -> Tuple[int, int]:
    if mask.size == 0 or not np.any(mask):
        return (0, mask.size)
    best_start = 0
    best_end = 0
    cur_start = None
    for i, v in enumerate(mask):
        if v and cur_start is None:
            cur_start = i
        if (not v) and (cur_start is not None):
            if (i - cur_start) > (best_end - best_start):
                best_start, best_end = cur_start, i
            cur_start = None
    if cur_start is not None and (mask.size - cur_start) > (best_end - best_start):
        best_start, best_end = cur_start, mask.size
    return (best_start, best_end)


def _select_active_segment(
    data: LogData,
    *,
    activity_quantile: float,
    min_duration_s: float,
) -> Tuple[LogData, Dict[str, float]]:
    score = _activity_score(data.obs, data.action_pre_scale)
    q = float(np.clip(activity_quantile, 0.0, 100.0))
    thr = float(np.percentile(score, q))
    mask = score >= thr
    start, end = _longest_true_segment(mask)
    min_len = max(8, int(round(min_duration_s * data.fs_hz)))
    if (end - start) < min_len:
        start, end = (0, data.time_s.size)
        used_active = False
    else:
        used_active = True

    out = LogData(
        time_s=data.time_s[start:end].copy(),
        obs=data.obs[start:end].copy(),
        action_pre_scale=data.action_pre_scale[start:end].copy(),
        fs_hz=data.fs_hz,
    )
    meta = {
        "threshold": thr,
        "quantile": q,
        "used_active_segment": 1.0 if used_active else 0.0,
        "start_idx": float(start),
        "end_idx": float(end),
        "n_samples": float(end - start),
        "duration_s": float(out.time_s[-1] - out.time_s[0]) if out.time_s.size >= 2 else 0.0,
    }
    return out, meta


def _joint_spectrum(
    x: np.ndarray,
    fs_hz: float,
    *,
    nperseg_cap: int = 1024,
) -> Tuple[np.ndarray, np.ndarray]:
    if x.ndim != 2 or x.shape[1] != 12:
        raise ValueError("Expected signal matrix with shape [N, 12]")
    n = x.shape[0]
    if n < 8:
        raise ValueError("Not enough samples for spectrum")
    nperseg = min(nperseg_cap, n)
    noverlap = nperseg // 2
    freq, pxx = welch(
        x - np.mean(x, axis=0, keepdims=True),
        fs=fs_hz,
        nperseg=nperseg,
        noverlap=noverlap,
        axis=0,
        detrend="constant",
        scaling="spectrum",
    )
    amp = np.sqrt(np.maximum(pxx, 0.0))
    return freq, amp


def _dominant_freq_hz(freq: np.ndarray, amp: np.ndarray, max_freq_hz: float) -> np.ndarray:
    use = (freq > 0.0) & (freq <= float(max_freq_hz))
    if not np.any(use):
        return np.zeros(amp.shape[1], dtype=np.float64)
    f = freq[use]
    a = amp[use, :]
    idx = np.argmax(a, axis=0)
    return f[idx]


def _plot_grid(
    *,
    freq_real: np.ndarray,
    amp_real: np.ndarray,
    freq_sim: np.ndarray,
    amp_sim: np.ndarray,
    title: str,
    out_png: Path,
    max_freq_hz: float,
) -> None:
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 3, figsize=(18, 14), sharex=True)
    axes = axes.reshape(-1)
    for j, name in enumerate(ACTION_JOINT_KEYS):
        ax = axes[j]
        use_r = freq_real <= max_freq_hz
        use_s = freq_sim <= max_freq_hz
        ax.plot(freq_real[use_r], amp_real[use_r, j], label="real", linewidth=1.6)
        ax.plot(freq_sim[use_s], amp_sim[use_s, j], label="sim", linewidth=1.3, alpha=0.9)
        ax.set_title(name, fontsize=10)
        ax.grid(True, alpha=0.25)
        if j % 3 == 0:
            ax.set_ylabel("amp (RMS)")
        if j >= 9:
            ax.set_xlabel("Hz")
        if j == 0:
            ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def _compare_policy(
    policy_dir: Path,
    *,
    max_freq_hz: float,
    activity_quantile: float,
    min_active_duration_s: float,
    use_full_sim: bool,
) -> Dict[str, object]:
    real_path = policy_dir / "debug_ctrl_real.csv"
    sim_path = policy_dir / "debug_ctrl_sim.csv"
    real_all = _load_debug_log(real_path)
    sim_all = _load_debug_log(sim_path)

    real_seg, real_meta = _select_active_segment(
        real_all,
        activity_quantile=activity_quantile,
        min_duration_s=min_active_duration_s,
    )

    if use_full_sim:
        sim_seg = sim_all
        sim_meta = {
            "used_active_segment": 0.0,
            "start_idx": 0.0,
            "end_idx": float(sim_all.time_s.size),
            "n_samples": float(sim_all.time_s.size),
            "duration_s": float(sim_all.time_s[-1] - sim_all.time_s[0]) if sim_all.time_s.size >= 2 else 0.0,
        }
    else:
        sim_seg, sim_meta = _select_active_segment(
            sim_all,
            activity_quantile=activity_quantile,
            min_duration_s=min_active_duration_s,
        )

    compare_root = policy_dir / "frequency_compare"
    compare_root.mkdir(parents=True, exist_ok=True)

    # Actions sent to robot.
    f_real_a, A_real = _joint_spectrum(real_seg.action_pre_scale, real_seg.fs_hz)
    f_sim_a, A_sim = _joint_spectrum(sim_seg.action_pre_scale, sim_seg.fs_hz)
    _plot_grid(
        freq_real=f_real_a,
        amp_real=A_real,
        freq_sim=f_sim_a,
        amp_sim=A_sim,
        title=f"{policy_dir.name}: action_pre_scale spectrum (real vs sim)",
        out_png=compare_root / "actions_real_vs_sim.png",
        max_freq_hz=max_freq_hz,
    )

    # Observations per joint: joint_pos and joint_vel terms.
    obs_real_pos = real_seg.obs[:, OBS_SLICES["joint_pos"]]
    obs_sim_pos = sim_seg.obs[:, OBS_SLICES["joint_pos"]]
    f_real_q, Q_real = _joint_spectrum(obs_real_pos, real_seg.fs_hz)
    f_sim_q, Q_sim = _joint_spectrum(obs_sim_pos, sim_seg.fs_hz)
    _plot_grid(
        freq_real=f_real_q,
        amp_real=Q_real,
        freq_sim=f_sim_q,
        amp_sim=Q_sim,
        title=f"{policy_dir.name}: observed joint_pos spectrum (real vs sim)",
        out_png=compare_root / "obs_joint_pos_real_vs_sim.png",
        max_freq_hz=max_freq_hz,
    )

    obs_real_qd = real_seg.obs[:, OBS_SLICES["joint_vel"]]
    obs_sim_qd = sim_seg.obs[:, OBS_SLICES["joint_vel"]]
    f_real_qd, Qd_real = _joint_spectrum(obs_real_qd, real_seg.fs_hz)
    f_sim_qd, Qd_sim = _joint_spectrum(obs_sim_qd, sim_seg.fs_hz)
    _plot_grid(
        freq_real=f_real_qd,
        amp_real=Qd_real,
        freq_sim=f_sim_qd,
        amp_sim=Qd_sim,
        title=f"{policy_dir.name}: observed joint_vel spectrum (real vs sim)",
        out_png=compare_root / "obs_joint_vel_real_vs_sim.png",
        max_freq_hz=max_freq_hz,
    )

    summary: Dict[str, object] = {
        "policy": policy_dir.name,
        "real_log": str(real_path),
        "sim_log": str(sim_path),
        "fs_real_hz": real_seg.fs_hz,
        "fs_sim_hz": sim_seg.fs_hz,
        "real_segment": real_meta,
        "sim_segment": sim_meta,
        "output_dir": str(compare_root),
        "peak_hz": {
            "actions": {
                "real": _dominant_freq_hz(f_real_a, A_real, max_freq_hz).round(4).tolist(),
                "sim": _dominant_freq_hz(f_sim_a, A_sim, max_freq_hz).round(4).tolist(),
            },
            "obs_joint_pos": {
                "real": _dominant_freq_hz(f_real_q, Q_real, max_freq_hz).round(4).tolist(),
                "sim": _dominant_freq_hz(f_sim_q, Q_sim, max_freq_hz).round(4).tolist(),
            },
            "obs_joint_vel": {
                "real": _dominant_freq_hz(f_real_qd, Qd_real, max_freq_hz).round(4).tolist(),
                "sim": _dominant_freq_hz(f_sim_qd, Qd_sim, max_freq_hz).round(4).tolist(),
            },
        },
    }
    with (compare_root / "summary.json").open("w") as f:
        json.dump(summary, f, indent=2)
    return summary


def _pretty_print_summary(summary: Dict[str, object]) -> None:
    policy = str(summary["policy"])
    fs_real = float(summary["fs_real_hz"])
    fs_sim = float(summary["fs_sim_hz"])
    real_seg = summary["real_segment"]
    sim_seg = summary["sim_segment"]
    if not isinstance(real_seg, dict) or not isinstance(sim_seg, dict):
        return
    print(f"\n=== {policy} ===")
    print(
        f"fs_real={fs_real:.3f} Hz, fs_sim={fs_sim:.3f} Hz, "
        f"real_duration={float(real_seg['duration_s']):.2f}s, sim_duration={float(sim_seg['duration_s']):.2f}s"
    )
    print(
        f"real_segment idx=[{int(real_seg['start_idx'])}:{int(real_seg['end_idx'])}] "
        f"n={int(real_seg['n_samples'])} thr={float(real_seg['threshold']):.6g}"
    )

    peak = summary.get("peak_hz")
    if not isinstance(peak, dict):
        return
    for key in ("actions", "obs_joint_pos", "obs_joint_vel"):
        block = peak.get(key, {})
        if not isinstance(block, dict):
            continue
        real = block.get("real", [])
        sim = block.get("sim", [])
        if isinstance(real, list) and isinstance(sim, list) and len(real) == 12 and len(sim) == 12:
            pairs = ", ".join([f"{ACTION_JOINT_KEYS[i]} r={real[i]:.2f} s={sim[i]:.2f}" for i in range(12)])
            print(f"peaks[{key}] {pairs}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare real-vs-sim frequency content per joint for RL-agent logs "
            "(action_pre_scale and observation joint terms)."
        )
    )
    parser.add_argument(
        "--policies",
        nargs="+",
        default=["codex_iteration_3", "codex_iteration_4"],
        help="Policy folder names under control/policy.",
    )
    parser.add_argument(
        "--policy-root",
        default="control/policy",
        help="Root folder containing policy directories.",
    )
    parser.add_argument(
        "--max-freq-hz",
        type=float,
        default=10.0,
        help="Maximum frequency shown in plots and used for dominant-frequency summary.",
    )
    parser.add_argument(
        "--activity-quantile",
        type=float,
        default=75.0,
        help="Real-log activity threshold quantile (0..100) built from joint_pos/action variation score.",
    )
    parser.add_argument(
        "--min-active-duration-s",
        type=float,
        default=2.0,
        help="Minimum active segment duration for real logs; fallback to full log if shorter.",
    )
    parser.add_argument(
        "--sim-active-only",
        action="store_true",
        help="Apply same activity-segment extraction on sim logs (default uses full sim log).",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    root = Path(args.policy_root)
    for name in args.policies:
        policy_dir = root / name
        if not policy_dir.exists():
            raise FileNotFoundError(f"Policy directory not found: {policy_dir}")
        summary = _compare_policy(
            policy_dir,
            max_freq_hz=float(args.max_freq_hz),
            activity_quantile=float(args.activity_quantile),
            min_active_duration_s=float(args.min_active_duration_s),
            use_full_sim=not bool(args.sim_active_only),
        )
        _pretty_print_summary(summary)
        print(f"saved: {policy_dir / 'frequency_compare'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
