#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml


JOINT_NAMES = [
    "hipz_right",
    "hipx_right",
    "hipy_right",
    "knee_right",
    "ankley_right",
    "anklex_right",
    "hipz_left",
    "hipx_left",
    "hipy_left",
    "knee_left",
    "ankley_left",
    "anklex_left",
]

POLICY_META_KEYS = {
    "concatenate_terms",
    "concatenate_dim",
    "enable_corruption",
    "history_length",
    "flatten_history_dim",
    "nan_check_per_term",
    "nan_policy",
}

SUPPORTED_POLICY_TERMS = {
    "actions",
    "command",
    "velocity_commands",
    "projected_gravity",
    "base_lin_vel",
    "base_ang_vel",
    "joint_pos",
    "joint_vel",
    "controlled_joint_pos",
    "controlled_joint_vel",
    "joint_torque",
    "joint_torques",
    "joint_effort",
    "joint_efforts",
}


def _load_isaac_yaml(path: Path) -> dict[str, Any]:
    class Loader(yaml.SafeLoader):
        pass

    def _construct_python_tuple(loader: yaml.Loader, node: yaml.Node) -> list[Any]:
        return list(loader.construct_sequence(node))

    def _construct_object_apply(loader: yaml.Loader, suffix: str, node: yaml.Node) -> Any:
        if isinstance(node, yaml.SequenceNode):
            seq = list(loader.construct_sequence(node))
        elif isinstance(node, yaml.MappingNode):
            seq = loader.construct_mapping(node)
        else:
            return loader.construct_scalar(node)

        if suffix == "builtins.slice" and isinstance(seq, list):
            vals = (seq + [None, None, None])[:3]
            return {"slice_start": vals[0], "slice_stop": vals[1], "slice_step": vals[2]}
        return seq

    Loader.add_constructor("tag:yaml.org,2002:python/tuple", _construct_python_tuple)
    Loader.add_multi_constructor("tag:yaml.org,2002:python/object/apply:", _construct_object_apply)

    raw = yaml.load(path.read_text(), Loader=Loader)
    if not isinstance(raw, dict):
        raise ValueError(f"Expected top-level mapping in {path}, got {type(raw).__name__}")
    return raw


def _get_nested(cfg: dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = cfg
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _joint_scale_from_config(scale_cfg: Any, joint_name: str) -> float:
    if isinstance(scale_cfg, (int, float)):
        return float(scale_cfg)
    if not isinstance(scale_cfg, dict):
        return 1.0

    if joint_name in scale_cfg:
        return _as_float(scale_cfg[joint_name], 1.0)

    joint_lower = joint_name.lower()
    for key, value in scale_cfg.items():
        key_lower = str(key).lower()
        if key_lower == ".*":
            return _as_float(value, 1.0)
        if "hipz" in joint_lower and "hipz" in key_lower:
            return _as_float(value, 1.0)
        if "hipx" in joint_lower and "hipx" in key_lower:
            return _as_float(value, 1.0)
        if "hipy" in joint_lower and "hipy" in key_lower:
            return _as_float(value, 1.0)
        if "knee" in joint_lower and "knee" in key_lower:
            return _as_float(value, 1.0)
        if "ankley" in joint_lower and ("ankley" in key_lower or "ankle_pitch" in key_lower):
            return _as_float(value, 1.0)
        if "anklex" in joint_lower and ("anklex" in key_lower or "ankle_roll" in key_lower):
            return _as_float(value, 1.0)
    return 1.0


def _extract_policy_terms(raw_cfg: dict[str, Any]) -> dict[str, dict[str, Any]]:
    obs_policy = _get_nested(raw_cfg, "observations", "policy", default={})
    if not isinstance(obs_policy, dict):
        raise ValueError("Missing observations.policy in Isaac config")

    terms: dict[str, dict[str, Any]] = {}
    for name, term_cfg in obs_policy.items():
        if name in POLICY_META_KEYS:
            continue
        if not isinstance(term_cfg, dict) or "func" not in term_cfg:
            continue
        if name not in SUPPORTED_POLICY_TERMS:
            continue
        out_term: dict[str, Any] = {}
        if "scale" in term_cfg and term_cfg["scale"] is not None:
            out_term["scale"] = _as_float(term_cfg["scale"], 1.0)
        terms[str(name)] = out_term

    if not terms:
        raise ValueError("No supported policy terms found in observations.policy")
    return terms


def _extract_joint_pos_ref(raw_cfg: dict[str, Any]) -> dict[str, float]:
    init_joint_pos = _get_nested(raw_cfg, "scene", "robot", "init_state", "joint_pos", default={})
    if not isinstance(init_joint_pos, dict):
        init_joint_pos = {}

    out: dict[str, float] = {}
    for name in JOINT_NAMES:
        out[name] = _as_float(init_joint_pos.get(name, 0.0), 0.0)
    return out


def _extract_command_ranges(raw_cfg: dict[str, Any]) -> dict[str, list[float]]:
    ranges = _get_nested(raw_cfg, "commands", "base_velocity", "ranges", default={})
    if not isinstance(ranges, dict):
        ranges = {}

    def _pair(key: str, default: tuple[float, float]) -> list[float]:
        raw = ranges.get(key, default)
        if isinstance(raw, (list, tuple)) and len(raw) >= 2:
            return [_as_float(raw[0], default[0]), _as_float(raw[1], default[1])]
        return [float(default[0]), float(default[1])]

    return {
        "lin_vel_x": _pair("lin_vel_x", (-0.5, 0.5)),
        "lin_vel_y": _pair("lin_vel_y", (-0.5, 0.5)),
        "ang_vel_z": _pair("ang_vel_z", (-0.3, 0.3)),
    }


def build_compat_config(raw_cfg: dict[str, Any]) -> dict[str, Any]:
    sim_dt = _as_float(_get_nested(raw_cfg, "sim", "dt", default=0.005), 0.005)
    decimation = int(_as_float(raw_cfg.get("decimation", 4), 4.0))
    hz = 50.0
    if sim_dt > 0.0 and decimation > 0:
        hz = 1.0 / (sim_dt * float(decimation))

    action_scale_cfg = _get_nested(raw_cfg, "actions", "joint_pos", "scale", default=1.0)
    action_scale_map = {name: _joint_scale_from_config(action_scale_cfg, name) for name in JOINT_NAMES}

    policy_history_len = int(_as_float(_get_nested(raw_cfg, "observations", "policy", "history_length", default=1), 1.0))

    compat = {
        "inference_hz": float(hz),
        "action_scale": 1.0,
        "joint_vel_source": "auto",
        "env_cfg": {
            "value": {
                "actions": {
                    "joint_pos": {
                        "joint_names": list(JOINT_NAMES),
                        "scale": action_scale_map,
                        "offset": 0.0,
                        "preserve_order": False,
                        "use_default_offset": True,
                    }
                },
                "commands": {
                    "twist": {
                        "ranges": _extract_command_ranges(raw_cfg),
                    }
                },
                "observations": {
                    "policy": {
                        "history_length": max(1, policy_history_len),
                        "terms": _extract_policy_terms(raw_cfg),
                    }
                },
                "scene": {
                    "entities": {
                        "robot": {
                            "init_state": {
                                "joint_pos": _extract_joint_pos_ref(raw_cfg),
                            }
                        }
                    }
                },
            }
        },
    }
    return compat


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert Isaac-style policy config to RLAgent-compatible config.")
    parser.add_argument("input", type=Path, help="Path to Isaac config.yaml")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output YAML path (default: <input_dir>/model_25000_env.yaml)",
    )
    args = parser.parse_args()

    input_path = args.input
    output_path = args.output or (input_path.parent / "model_25000_env.yaml")

    raw_cfg = _load_isaac_yaml(input_path)
    compat_cfg = build_compat_config(raw_cfg)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(compat_cfg, sort_keys=False))
    print(f"Wrote: {output_path}")


if __name__ == "__main__":
    main()
