#!/usr/bin/env python3
"""Persistent 8-motor pose storage for the iROI robot arm.

Pose coordinates are OUTPUT/JOINТ angles in degrees, measured relative to the
per-motor absolute encoder calibration handled by motor_control_node.

Important separation:
- zero_config*.json: motor/encoder calibration reference
- arm_poses.json: named robot poses in calibrated joint coordinates

All pose records always contain motor IDs 1..8. Unknown/unmeasured values are
stored as JSON null (Python None).
"""

from __future__ import annotations

import json
import os
from copy import deepcopy
from typing import Dict, Iterable, Mapping, Optional

POSE_FILE_VERSION = 1
ALL_MOTOR_IDS = tuple(range(1, 9))
DEFAULT_POSE_ID = 0
DEFAULT_POSE_NAME = "attention"


def empty_angles() -> Dict[str, Optional[float]]:
    return {str(mid): None for mid in ALL_MOTOR_IDS}


def default_pose_document() -> dict:
    return {
        "version": POSE_FILE_VERSION,
        "poses": {
            str(DEFAULT_POSE_ID): {
                "name": DEFAULT_POSE_NAME,
                "angles": empty_angles(),
            }
        },
    }


class PoseManager:
    """Load, normalize, query, and atomically save 8-axis poses."""

    def __init__(self, path: str):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.data = default_pose_document()
        self.load_or_create()

    def load_or_create(self) -> None:
        if not os.path.exists(self.path):
            self.data = default_pose_document()
            self.save()
            return

        try:
            with open(self.path, "r", encoding="utf-8") as f:
                raw = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"pose 파일 읽기 실패: {self.path}: {exc}") from exc

        self.data = self._normalize_document(raw)

    @staticmethod
    def _normalize_document(raw: object) -> dict:
        if not isinstance(raw, dict):
            raise RuntimeError("pose 파일 최상위 값은 JSON object여야 합니다.")

        version = raw.get("version")
        if version != POSE_FILE_VERSION:
            raise RuntimeError(
                f"지원하지 않는 pose 파일 version={version!r}. "
                f"현재 지원 version={POSE_FILE_VERSION}"
            )

        poses_raw = raw.get("poses")
        if not isinstance(poses_raw, dict):
            raise RuntimeError('pose 파일에 "poses" object가 없습니다.')

        normalized = {"version": POSE_FILE_VERSION, "poses": {}}

        for pose_key, pose_raw in poses_raw.items():
            try:
                pose_id = int(pose_key)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"pose ID는 정수여야 합니다: {pose_key!r}") from exc
            if pose_id < 0:
                raise RuntimeError(f"pose ID는 0 이상이어야 합니다: {pose_id}")
            if not isinstance(pose_raw, dict):
                raise RuntimeError(f"pose {pose_id} 값은 object여야 합니다.")

            name = str(pose_raw.get("name") or f"pose_{pose_id}")
            angles_raw = pose_raw.get("angles", {})
            if not isinstance(angles_raw, dict):
                raise RuntimeError(f"pose {pose_id}.angles는 object여야 합니다.")

            angles = empty_angles()
            for mid in ALL_MOTOR_IDS:
                value = angles_raw.get(str(mid), angles_raw.get(mid))
                if value is None:
                    angles[str(mid)] = None
                else:
                    try:
                        angles[str(mid)] = float(value)
                    except (TypeError, ValueError) as exc:
                        raise RuntimeError(
                            f"pose {pose_id}, motor {mid} 각도는 숫자 또는 null이어야 합니다: {value!r}"
                        ) from exc

            normalized["poses"][str(pose_id)] = {
                "name": name,
                "angles": angles,
            }

        # Pose 0 is mandatory and reserved for attention/startup posture.
        if "0" not in normalized["poses"]:
            normalized["poses"]["0"] = {
                "name": DEFAULT_POSE_NAME,
                "angles": empty_angles(),
            }

        # Pose 0 name may be customized, but it remains reserved as startup pose.
        return normalized

    def save(self) -> None:
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        tmp_path = self.path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)
            f.write("\n")
        os.replace(tmp_path, self.path)

    def list_poses(self) -> list[dict]:
        rows = []
        for pose_key in sorted(self.data["poses"], key=lambda x: int(x)):
            pose = self.data["poses"][pose_key]
            measured = sum(v is not None for v in pose["angles"].values())
            rows.append(
                {
                    "pose_id": int(pose_key),
                    "name": pose["name"],
                    "measured": measured,
                    "total": len(ALL_MOTOR_IDS),
                }
            )
        return rows

    def has_pose(self, pose_id: int) -> bool:
        return str(int(pose_id)) in self.data["poses"]

    def get_pose(self, pose_id: int) -> dict:
        key = str(int(pose_id))
        if key not in self.data["poses"]:
            raise KeyError(f"pose {pose_id}가 없습니다.")
        return deepcopy(self.data["poses"][key])

    def save_pose(
        self,
        pose_id: int,
        angles: Mapping[int | str, Optional[float]],
        name: Optional[str] = None,
    ) -> dict:
        pose_id = int(pose_id)
        if pose_id < 0:
            raise ValueError("pose ID는 0 이상이어야 합니다.")

        normalized_angles = empty_angles()
        for mid in ALL_MOTOR_IDS:
            value = angles.get(mid, angles.get(str(mid)))
            normalized_angles[str(mid)] = None if value is None else float(value)

        key = str(pose_id)
        old = self.data["poses"].get(key)
        if name is None:
            if old is not None:
                name = old.get("name") or f"pose_{pose_id}"
            else:
                name = DEFAULT_POSE_NAME if pose_id == 0 else f"pose_{pose_id}"

        self.data["poses"][key] = {
            "name": str(name),
            "angles": normalized_angles,
        }
        self.save()
        return self.get_pose(pose_id)

    def delete_pose(self, pose_id: int) -> None:
        pose_id = int(pose_id)
        if pose_id == 0:
            raise ValueError("pose 0은 차렷/startup 예약 pose라 삭제할 수 없습니다.")
        key = str(pose_id)
        if key not in self.data["poses"]:
            raise KeyError(f"pose {pose_id}가 없습니다.")
        del self.data["poses"][key]
        self.save()

    def angles_for_motor_ids(
        self,
        pose_id: int,
        motor_ids: Iterable[int],
    ) -> Dict[int, Optional[float]]:
        pose = self.get_pose(pose_id)
        result: Dict[int, Optional[float]] = {}
        for mid in motor_ids:
            mid = int(mid)
            if mid not in ALL_MOTOR_IDS:
                raise ValueError(f"지원 범위 밖 motor ID: {mid}")
            result[mid] = pose["angles"][str(mid)]
        return result
